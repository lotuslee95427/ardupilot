#include "mode.h"
#include "Plane.h"

// 进入自动模式
bool ModeAuto::_enter()
{
#if HAL_QUADPLANE_ENABLED
    // 检查是否应该由于在guided_wait_takeoff状态下缺少起飞而拒绝自动模式
    if (plane.previous_mode == &plane.mode_guided &&
        quadplane.guided_wait_takeoff_on_mode_enter) {
        if (!plane.mission.starts_with_takeoff_cmd()) {
            gcs().send_text(MAV_SEVERITY_ERROR,"Takeoff waypoint required");
            quadplane.guided_wait_takeoff = true;
            return false;
        }
    }
    
    // 设置VTOL模式
    if (plane.quadplane.available() && plane.quadplane.enable == 2) {
        plane.auto_state.vtol_mode = true;
    } else {
        plane.auto_state.vtol_mode = false;
    }
#else
    plane.auto_state.vtol_mode = false;
#endif
    // 初始化航点位置
    plane.next_WP_loc = plane.prev_WP_loc = plane.current_loc;
    // 根据MIS_AUTORESET开始或恢复任务
    plane.mission.start_or_resume();

    // 处理看门狗重启情况
    if (hal.util->was_watchdog_armed()) {
        if (hal.util->persistent_data.waypoint_num != 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "Watchdog: resume WP %u", hal.util->persistent_data.waypoint_num);
            plane.mission.set_current_cmd(hal.util->persistent_data.waypoint_num);
            hal.util->persistent_data.waypoint_num = 0;
        }
    }

#if HAL_SOARING_ENABLED
    // 初始化滑翔控制器
    plane.g2.soaring_controller.init_cruising();
#endif

    return true;
}

// 退出自动模式
void ModeAuto::_exit()
{
    if (plane.mission.state() == AP_Mission::MISSION_RUNNING) {
        plane.mission.stop();

        // 检查是否需要重新开始着陆序列
        bool restart = plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND;
#if HAL_QUADPLANE_ENABLED
        if (plane.quadplane.is_vtol_land(plane.mission.get_current_nav_cmd().id)) {
            restart = false;
        }
#endif
        if (restart) {
            plane.landing.restart_landing_sequence();
        }
    }
    plane.auto_state.started_flying_in_auto_ms = 0;
}

// 更新自动模式
void ModeAuto::update()
{
    // 检查任务是否正在运行
    if (plane.mission.state() != AP_Mission::MISSION_RUNNING) {
        plane.set_mode(plane.mode_rtl, ModeReason::MISSION_END);
        gcs().send_text(MAV_SEVERITY_INFO, "Aircraft in auto without a running mission");
        return;
    }

    uint16_t nav_cmd_id = plane.mission.get_current_nav_cmd().id;

#if HAL_QUADPLANE_ENABLED
    // 处理VTOL自动模式
    if (plane.quadplane.in_vtol_auto()) {
        plane.quadplane.control_auto();
        return;
    }
#endif

#if AP_PLANE_GLIDER_PULLUP_ENABLED
    // 处理滑翔机拉升
    if (pullup.in_pullup()) {
        return;
    }
#endif

    // 处理不同的导航命令
    if (nav_cmd_id == MAV_CMD_NAV_TAKEOFF ||
        (nav_cmd_id == MAV_CMD_NAV_LAND && plane.flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING)) {
        plane.takeoff_calc_roll();
        plane.takeoff_calc_pitch();
        plane.calc_throttle();
    } else if (nav_cmd_id == MAV_CMD_NAV_LAND) {
        plane.calc_nav_roll();
        plane.calc_nav_pitch();

        // 允许着陆限制横滚限制
        plane.nav_roll_cd = plane.landing.constrain_roll(plane.nav_roll_cd, plane.g.level_roll_limit*100UL);

        if (plane.landing.is_throttle_suppressed()) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        } else {
            plane.calc_throttle();
        }
#if AP_SCRIPTING_ENABLED
    } else if (nav_cmd_id == MAV_CMD_NAV_SCRIPT_TIME) {
        // 处理脚本导航命令
        plane.nav_roll_cd = ahrs.roll_sensor;
        plane.nav_pitch_cd = ahrs.pitch_sensor;
#endif
    } else {
        // 正常自动飞行
        if (nav_cmd_id != MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT) {
            plane.steer_state.hold_course_cd = -1;
        }
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }
}

// 执行导航
void ModeAuto::navigate()
{
    if (AP::ahrs().home_is_set()) {
        plane.mission.update();
    }
}

// 检查是否执行自动导航
bool ModeAuto::does_auto_navigation() const
{
#if AP_SCRIPTING_ENABLED
   return (!plane.nav_scripting_active());
#endif
   return true;
}

// 检查是否执行自动油门控制
bool ModeAuto::does_auto_throttle() const
{
#if AP_SCRIPTING_ENABLED
   return (!plane.nav_scripting_active());
#endif
   return true;
}

// 执行预解锁检查
bool ModeAuto::_pre_arm_checks(size_t buflen, char *buffer) const
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.enabled()) {
        if (plane.quadplane.option_is_set(QuadPlane::OPTION::ONLY_ARM_IN_QMODE_OR_AUTO) &&
                !plane.quadplane.is_vtol_takeoff(plane.mission.get_current_nav_cmd().id)) {
            hal.util->snprintf(buffer, buflen, "not in VTOL takeoff");
            return false;
        }
        if (!plane.mission.starts_with_takeoff_cmd()) {
            hal.util->snprintf(buffer, buflen, "missing takeoff waypoint");
            return false;
        }
    }
#endif
    return true;
}

// 检查是否正在着陆
bool ModeAuto::is_landing() const
{
    return (plane.flight_stage == AP_FixedWing::FlightStage::LAND);
}

// 运行自动模式
void ModeAuto::run()
{
#if AP_PLANE_GLIDER_PULLUP_ENABLED
    if (pullup.in_pullup()) {
        pullup.stabilize_pullup();
        return;
    }
#endif
    
    if (plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_ALTITUDE_WAIT) {
        // 高度等待命令处理
        wiggle_servos();

        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, 0.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, 0.0);

        SRV_Channels::set_output_to_trim(SRV_Channel::k_throttle);
        SRV_Channels::set_output_to_trim(SRV_Channel::k_throttleLeft);
        SRV_Channels::set_output_to_trim(SRV_Channel::k_throttleRight);

        // 放松姿态控制
        reset_controllers();

    } else {
        // 正常飞行，运行基类方法
        Mode::run();
    }
}
