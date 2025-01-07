#include "mode.h"
#include "Plane.h"

// 进入RTL模式
bool ModeRTL::_enter()
{
    // 设置前一个航点为当前位置
    plane.prev_WP_loc = plane.current_loc;
    // 执行RTL操作，设置RTL高度
    plane.do_RTL(plane.get_RTL_altitude_cm());
    // 重置爬升完成标志
    plane.rtl.done_climb = false;
#if HAL_QUADPLANE_ENABLED
    // 设置VTOL接近阶段为RTL
    plane.vtol_approach_s.approach_stage = Plane::VTOLApproach::Stage::RTL;

    // 四旋翼特定检查
    if (plane.quadplane.available()) {
        // 如果在引导等待起飞状态，将RTL视为QLAND，以应对GUIDED->AUTO起飞序列中的故障保护
        if (plane.quadplane.guided_wait_takeoff_on_mode_enter) {
            plane.set_mode(plane.mode_qland, ModeReason::QLAND_INSTEAD_OF_RTL);
            return true;
        }

        // 如果Q_RTL_MODE为QRTL_ALWAYS，立即切换到QRTL模式
        if (plane.quadplane.rtl_mode == QuadPlane::RTL_MODE::QRTL_ALWAYS) {
            plane.set_mode(plane.mode_qrtl, ModeReason::QRTL_INSTEAD_OF_RTL);
            return true;
        }

        // 如果预期VTOL着陆且四旋翼电机处于活动状态，并且在QRTL半径内且低于QRTL高度，则切换到QRTL
        const bool vtol_landing = (plane.quadplane.rtl_mode == QuadPlane::RTL_MODE::SWITCH_QRTL) || (plane.quadplane.rtl_mode == QuadPlane::RTL_MODE::VTOL_APPROACH_QRTL);
        if (vtol_landing && (quadplane.motors->get_desired_spool_state() == AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED)) {
            int32_t alt_cm;
            if ((plane.current_loc.get_distance(plane.next_WP_loc) < plane.mode_qrtl.get_VTOL_return_radius()) &&
                plane.current_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, alt_cm) && (alt_cm < plane.quadplane.qrtl_alt*100)) {
                plane.set_mode(plane.mode_qrtl, ModeReason::QRTL_INSTEAD_OF_RTL);
                return true;
            }
        }
    }
#endif

    return true;
}

// 更新RTL模式
void ModeRTL::update()
{
    // 计算导航滚转、俯仰和油门
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();

    bool alt_threshold_reached = false;
    if (plane.flight_option_enabled(FlightOptions::CLIMB_BEFORE_TURN)) {
        // 在转弯前爬升到RTL_ALTITUDE。这会覆盖RTL_CLIMB_MIN。
        alt_threshold_reached = plane.current_loc.alt > plane.next_WP_loc.alt;
    } else if (plane.g2.rtl_climb_min > 0) {
        // 当RTL首次开始时，限制坡度角到LEVEL_ROLL_LIMIT，直到我们爬升了RTL_CLIMB_MIN米
        alt_threshold_reached = (plane.current_loc.alt - plane.prev_WP_loc.alt)*0.01 > plane.g2.rtl_climb_min;
    } else {
        return;
    }

    if (!plane.rtl.done_climb && alt_threshold_reached) {
        // 爬升完成，设置滑翔坡度
        plane.prev_WP_loc = plane.current_loc;
        plane.setup_glide_slope();
        plane.rtl.done_climb = true;
    }
    if (!plane.rtl.done_climb) {
        // 限制滚转角度作为故障保护，这样如果出现问题，飞机最终会转回并进入RTL，而不是完全直线飞行
        // 这也为对抗风留有余地
        plane.roll_limit_cd = MIN(plane.roll_limit_cd, plane.g.level_roll_limit*100);
        plane.nav_roll_cd = constrain_int32(plane.nav_roll_cd, -plane.roll_limit_cd, plane.roll_limit_cd);
    }
}

// RTL导航
void ModeRTL::navigate()
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.available()) {
        if (plane.quadplane.rtl_mode == QuadPlane::RTL_MODE::VTOL_APPROACH_QRTL) {
            // VTOL接近着陆
            AP_Mission::Mission_Command cmd;
            cmd.content.location = plane.next_WP_loc;
            plane.verify_landing_vtol_approach(cmd);
            if (plane.vtol_approach_s.approach_stage == Plane::VTOLApproach::Stage::VTOL_LANDING) {
                plane.set_mode(plane.mode_qrtl, ModeReason::RTL_COMPLETE_SWITCHING_TO_VTOL_LAND_RTL);
            }
            return;
        }

        // 检查是否需要切换到QRTL模式
        if ((AP_HAL::millis() - plane.last_mode_change_ms > 1000) && switch_QRTL()) {
            return;
        }
    }
#endif

    // 设置盘旋半径和方向
    uint16_t radius = abs(plane.g.rtl_radius);
    if (radius > 0) {
        plane.loiter.direction = (plane.g.rtl_radius < 0) ? -1 : 1;
    }

    // 更新盘旋
    plane.update_loiter(radius);

    // 检查是否需要执行自动着陆
    if (!plane.auto_state.checked_for_autoland) {
        if ((plane.g.rtl_autoland == RtlAutoland::RTL_IMMEDIATE_DO_LAND_START) ||
            (plane.g.rtl_autoland == RtlAutoland::RTL_THEN_DO_LAND_START &&
            plane.reached_loiter_target() && 
            labs(plane.calc_altitude_error_cm()) < 1000))
            {
                // 我们已经到达RTL点，检查是否有着陆序列
                if (plane.have_position && plane.mission.jump_to_landing_sequence(plane.current_loc)) {
                    // 从RTL切换到AUTO模式
                    plane.mission.set_force_resume(true);
                    if (plane.set_mode(plane.mode_auto, ModeReason::RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND)) {
                        // 返回，这样我们不会改变半径并且不会运行rtl update_loiter()
                        return;
                    }
                }

                // 防止在每个循环中运行昂贵的jump_to_landing_sequence
                plane.auto_state.checked_for_autoland = true;
            }
    }
}

#if HAL_QUADPLANE_ENABLED
// 如果启用并且在半径内，切换到QRTL
bool ModeRTL::switch_QRTL()
{
    if (plane.quadplane.rtl_mode != QuadPlane::RTL_MODE::SWITCH_QRTL) {
        return false;
    }

    // 获取QRTL半径
    uint16_t qrtl_radius = abs(plane.g.rtl_radius);
    if (qrtl_radius == 0) {
        qrtl_radius = abs(plane.aparm.loiter_radius);
    }

    // 检查是否满足切换到QRTL的条件
    if (plane.nav_controller->reached_loiter_target() ||
         plane.current_loc.past_interval_finish_line(plane.prev_WP_loc, plane.next_WP_loc) ||
         plane.auto_state.wp_distance < MAX(qrtl_radius, plane.quadplane.stopping_distance())) {
        // 对于RTL模式下的四旋翼，当我们在停止距离和RTL_RADIUS的最大值内时，切换到QRTL
        plane.set_mode(plane.mode_qrtl, ModeReason::RTL_COMPLETE_SWITCHING_TO_VTOL_LAND_RTL);
        return true;
    }

    return false;
}

#endif  // HAL_QUADPLANE_ENABLED
