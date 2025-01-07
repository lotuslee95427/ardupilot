#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

// 进入QRTL模式
bool ModeQRTL::_enter()
{
    // 如果在引导等待起飞状态，将QRTL视为QLAND，以应对GUIDED->AUTO起飞序列期间的故障保护
    if (plane.quadplane.guided_wait_takeoff_on_mode_enter) {
       plane.set_mode(plane.mode_qland, ModeReason::QLAND_INSTEAD_OF_RTL);
       return true;
    }
    submode = SubMode::RTL;
    plane.prev_WP_loc = plane.current_loc;

    // 计算RTL高度
    int32_t RTL_alt_abs_cm = plane.home.alt + quadplane.qrtl_alt*100UL;
    if (quadplane.motors->get_desired_spool_state() == AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED) {
        // VTOL电机处于活动状态，可能在VTOL飞行或辅助飞行中
        Location destination = plane.calc_best_rally_or_home_location(plane.current_loc, RTL_alt_abs_cm);
        const float dist = plane.current_loc.get_distance(destination);
        const float radius = get_VTOL_return_radius();

        // 至少爬升到家的周围圆锥体高度（QRTL高度）和半径
        // 始终至少爬升到Q_RTL_ALT_MIN，将Q_RTL_ALT_MIN限制在Q_LAND_FINAL_ALT和Q_RTL_ALT之间
        const float min_climb = constrain_float(quadplane.qrtl_alt_min, quadplane.land_final_alt, quadplane.qrtl_alt);
        const float target_alt = MAX(quadplane.qrtl_alt * (dist / MAX(radius, dist)), min_climb);

#if AP_TERRAIN_AVAILABLE
        const bool use_terrain = plane.terrain_enabled_in_mode(mode_number());
#else
        const bool use_terrain = false;
#endif

        const float dist_to_climb = target_alt - plane.relative_ground_altitude(plane.g.rangefinder_landing, use_terrain);
        if (is_positive(dist_to_climb)) {
            // 返回前爬升，只使用下一个航点高度
            submode = SubMode::climb;
            plane.next_WP_loc = plane.current_loc;
#if AP_TERRAIN_AVAILABLE
            int32_t curent_alt_terrain_cm;
            if (use_terrain && plane.current_loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, curent_alt_terrain_cm)) {
                plane.next_WP_loc.set_alt_cm(curent_alt_terrain_cm + dist_to_climb * 100UL, Location::AltFrame::ABOVE_TERRAIN);
                return true;
            }
#endif
            plane.next_WP_loc.set_alt_cm(plane.current_loc.alt + dist_to_climb * 100UL, plane.current_loc.get_alt_frame());
            return true;

        } else if (dist < radius) {
            // 在家的"圆锥体"上方，如果低于QRTL高度则以当前高度返回
            int32_t current_alt_abs_cm;
            if (plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, current_alt_abs_cm)) {
                RTL_alt_abs_cm = MIN(RTL_alt_abs_cm, current_alt_abs_cm);
            }

            // 靠近目的地且VTOL电机已在运行，不进行过渡也不爬升
            gcs().send_text(MAV_SEVERITY_INFO,"VTOL position1 d=%.1f r=%.1f", dist, radius);
            poscontrol.set_state(QuadPlane::QPOS_POSITION1);
        }
    }

    // 使用do_RTL()设置next_WP_loc
    plane.do_RTL(RTL_alt_abs_cm);
    quadplane.poscontrol_init_approach();

    // 确定是否需要缓慢下降
    int32_t from_alt;
    int32_t to_alt;
    if (plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE,from_alt) && plane.next_WP_loc.get_alt_cm(Location::AltFrame::ABSOLUTE,to_alt)) {
        poscontrol.slow_descent = from_alt > to_alt;
        return true;
    }
    // 默认回到旧方法
    poscontrol.slow_descent = (plane.current_loc.alt > plane.next_WP_loc.alt);
    return true;
}

// 更新QRTL模式
void ModeQRTL::update()
{
    plane.mode_qstabilize.update();
}

// 处理QRTL模式
void ModeQRTL::run()
{
    const uint32_t now = AP_HAL::millis();
    if (quadplane.tailsitter.in_vtol_transition(now)) {
        // 尾坐式飞机在VTOL过渡的前向上升阶段运行前向控制器
        Mode::run();
        return;
    }

    switch (submode) {
        case SubMode::climb: {
            // 请求零速度
            Vector2f vel, accel;
            pos_control->input_vel_accel_xy(vel, accel);
            quadplane.run_xy_controller();

            // 导航滚转和俯仰由位置控制器控制
            plane.nav_roll_cd = pos_control->get_roll_cd();
            plane.nav_pitch_cd = pos_control->get_pitch_cd();

            plane.quadplane.assign_tilt_to_fwd_thr();

            if (quadplane.transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd)) {
                pos_control->set_externally_limited_xy();
            }
            // 无pilot输入时的风向标
            quadplane.disable_yaw_rate_time_constant();
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                          plane.nav_pitch_cd,
                                                                          quadplane.get_weathervane_yaw_rate_cds());

            // 以全WP导航速度爬升
            quadplane.set_climb_rate_cms(quadplane.wp_nav->get_default_speed_up());
            quadplane.run_z_controller();

            // 当停止点达到目标高度时爬升完成
            Vector3p stopping_point;
            pos_control->get_stopping_point_z_cm(stopping_point.z);
            Location stopping_loc = Location(stopping_point.tofloat(), Location::AltFrame::ABOVE_ORIGIN);

            ftype alt_diff;
            if (!stopping_loc.get_alt_distance(plane.next_WP_loc, alt_diff) || is_positive(alt_diff)) {
                // 爬升完成或无法获取高度差，返回家
                submode = SubMode::RTL;
                plane.prev_WP_loc = plane.current_loc;

                int32_t RTL_alt_abs_cm = plane.home.alt + quadplane.qrtl_alt*100UL;
                Location destination = plane.calc_best_rally_or_home_location(plane.current_loc, RTL_alt_abs_cm);
                const float dist = plane.current_loc.get_distance(destination);
                const float radius = get_VTOL_return_radius();
                if (dist < radius) {
                    // 如果靠近家，以当前目标高度返回
                    int32_t target_alt_abs_cm;
                    if (plane.next_WP_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, target_alt_abs_cm)) {
                        RTL_alt_abs_cm = MIN(RTL_alt_abs_cm, target_alt_abs_cm);
                    }
                    gcs().send_text(MAV_SEVERITY_INFO,"VTOL position1 d=%.1f r=%.1f", dist, radius);
                    poscontrol.set_state(QuadPlane::QPOS_POSITION1);
                }

                plane.do_RTL(RTL_alt_abs_cm);
                quadplane.poscontrol_init_approach();
                if (plane.current_loc.get_alt_distance(plane.next_WP_loc, alt_diff)) {
                    poscontrol.slow_descent = is_positive(alt_diff);
                } else {
                    // 默认回到旧方法
                    poscontrol.slow_descent = (plane.current_loc.alt > plane.next_WP_loc.alt);
                }
            }
            break;
        }

        case SubMode::RTL: {
            quadplane.vtol_position_controller();
            if (poscontrol.get_state() > QuadPlane::QPOS_POSITION2) {
                // 将目标高度更改为家的高度
                plane.next_WP_loc.alt = plane.home.alt;
            }
            if (poscontrol.get_state() >= QuadPlane::QPOS_POSITION2) {
                // 开始着陆逻辑
                quadplane.verify_vtol_land();
            }

            // 在接近时允许摇杆混合
            if (quadplane.poscontrol.get_state() == QuadPlane::QPOS_AIRBRAKE ||
                quadplane.poscontrol.get_state() == QuadPlane::QPOS_APPROACH) {
                plane.stabilize_stick_mixing_fbw();
            }
            break;
        }
    }

    // 使用固定翼表面进行稳定
    plane.stabilize_roll();
    plane.stabilize_pitch();
}

// 更新QRTL配置文件的目标高度
void ModeQRTL::update_target_altitude()
{
    // 在接近时更新高度目标
    if ((submode != SubMode::RTL) || (plane.quadplane.poscontrol.get_state() != QuadPlane::QPOS_APPROACH)) {
        Mode::update_target_altitude();
        return;
    }

    // 初始以RTL_ALT_CM接近，然后基于TECS的最大下降率下降到QRTL_ALT，
    // 给予时间在过渡前减速
    const float radius = MAX(fabsf(float(plane.aparm.loiter_radius)), fabsf(float(plane.g.rtl_radius)));
    const float rtl_alt_delta = MAX(0, plane.g.RTL_altitude - plane.quadplane.qrtl_alt);
    const float sink_time = rtl_alt_delta / MAX(0.6*plane.TECS_controller.get_max_sinkrate(), 1);
    const float sink_dist = plane.aparm.airspeed_cruise * sink_time;
    const float dist = plane.auto_state.wp_distance;
    const float rad_min = 2*radius;
    const float rad_max = 20*radius;
    float alt = linear_interpolate(0, rtl_alt_delta,
                                   dist,
                                   rad_min, MAX(rad_min, MIN(rad_max, rad_min+sink_dist)));
    Location loc = plane.next_WP_loc;
    loc.alt += alt*100;
    plane.set_target_altitude_location(loc);
}

// 仅在接近时允许油门微调
bool ModeQRTL::allows_throttle_nudging() const
{
    return (submode == SubMode::RTL) && (plane.quadplane.poscontrol.get_state() == QuadPlane::QPOS_APPROACH);
}

// 返回应使用纯VTOL飞行的目的地半径，不进行过渡到FW
float ModeQRTL::get_VTOL_return_radius() const
{
    return MAX(fabsf(float(plane.aparm.loiter_radius)), fabsf(float(plane.g.rtl_radius))) * 1.5;
}

#endif
