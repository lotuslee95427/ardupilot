#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

// 进入QLoiter模式时的初始化函数
bool ModeQLoiter::_enter()
{
    // 初始化悬停导航
    loiter_nav->clear_pilot_desired_acceleration();
    loiter_nav->init_target();

    // 设置垂直速度和加速度限制
    pos_control->set_max_speed_accel_z(-quadplane.get_pilot_velocity_z_max_dn(), quadplane.pilot_speed_z_max_up*100, quadplane.pilot_accel_z*100);
    pos_control->set_correction_speed_accel_z(-quadplane.get_pilot_velocity_z_max_dn(), quadplane.pilot_speed_z_max_up*100, quadplane.pilot_accel_z*100);

    // 初始化油门等待
    quadplane.init_throttle_wait();

    // 防止目标位置重新初始化
    quadplane.last_loiter_ms = AP_HAL::millis();

    // 清除精确着陆时间戳
    last_target_loc_set_ms = 0;

    return true;
}

// 更新QLoiter模式
void ModeQLoiter::update()
{
    plane.mode_qstabilize.update();
}

// 运行四旋翼悬停控制器
void ModeQLoiter::run()
{
    const uint32_t now = AP_HAL::millis();

#if AC_PRECLAND_ENABLED
    const uint32_t precland_timeout_ms = 250;
    // 检查精确着陆或精确悬停是否激活，并覆盖目标位置
    const uint32_t last_pos_set_ms = last_target_loc_set_ms;
    const uint32_t last_vel_set_ms = quadplane.poscontrol.last_velocity_match_ms;

    // 处理位置覆盖
    if (last_pos_set_ms != 0 && now - last_pos_set_ms < precland_timeout_ms) {
        Vector2f rel_origin;
        if (plane.next_WP_loc.get_vector_xy_from_origin_NE(rel_origin)) {
            quadplane.pos_control->set_pos_target_xy_cm(rel_origin.x, rel_origin.y);
            last_target_loc_set_ms = 0;
        }
    }

    // 处理速度覆盖
    if (last_vel_set_ms != 0 && now - last_vel_set_ms < precland_timeout_ms) {
        Vector2f target_accel;
        Vector2f target_speed_xy_cms{quadplane.poscontrol.velocity_match.x*100, quadplane.poscontrol.velocity_match.y*100};
        quadplane.pos_control->input_vel_accel_xy(target_speed_xy_cms, target_accel);
        quadplane.poscontrol.last_velocity_match_ms = 0;
    }
#endif // AC_PRECLAND_ENABLED

    // 处理尾座式飞机的VTOL转换
    if (quadplane.tailsitter.in_vtol_transition(now)) {
        Mode::run();
        return;
    }

    // 处理油门等待状态
    if (quadplane.throttle_wait) {
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0, true, 0);
        quadplane.relax_attitude_control();
        pos_control->relax_z_controller(0);
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();

        // 使用固定翼表面进行稳定
        plane.stabilize_roll();
        plane.stabilize_pitch();
        return;
    }

    // 如果电机未解锁，重新进入QLoiter模式
    if (!quadplane.motors->armed()) {
        plane.mode_qloiter._enter();
    }

    // 降落时软化控制
    if (quadplane.should_relax()) {
        loiter_nav->soften_for_landing();
    }

    // 定期重置悬停目标
    if (now - quadplane.last_loiter_ms > 500) {
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();
    }
    quadplane.last_loiter_ms = now;

    // 设置电机为全范围模式
    quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // 设置垂直速度和加速度限制
    pos_control->set_max_speed_accel_z(-quadplane.get_pilot_velocity_z_max_dn(), quadplane.pilot_speed_z_max_up*100, quadplane.pilot_accel_z*100);

    // 处理飞行员的横滚和俯仰输入
    float target_roll_cd, target_pitch_cd;
    quadplane.get_pilot_desired_lean_angles(target_roll_cd, target_pitch_cd, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());
    loiter_nav->set_pilot_desired_acceleration(target_roll_cd, target_pitch_cd);
    
    // 运行悬停控制器
    if (!pos_control->is_active_xy()) {
        pos_control->init_xy_controller();
    }
    loiter_nav->update();

    // 设置导航横滚和俯仰角
    plane.nav_roll_cd = loiter_nav->get_roll();
    plane.nav_pitch_cd = loiter_nav->get_pitch();

    // 分配倾斜到前向推力
    plane.quadplane.assign_tilt_to_fwd_thr();

    // 设置VTOL横滚和俯仰限制
    if (quadplane.transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd)) {
        pos_control->set_externally_limited_xy();
    }

    // 设置飞行员偏航速率时间常数
    quadplane.set_pilot_yaw_rate_time_constant();

    // 调用姿态控制器
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                  plane.nav_pitch_cd,
                                                                  quadplane.get_desired_yaw_rate_cds());

    // 处理着陆模式
    if (plane.control_mode == &plane.mode_qland) {
        // 检查是否进入最终着陆阶段
        if (poscontrol.get_state() < QuadPlane::QPOS_LAND_FINAL && quadplane.check_land_final()) {
            poscontrol.set_state(QuadPlane::QPOS_LAND_FINAL);
            quadplane.setup_target_position();
#if AP_ICENGINE_ENABLED
            // 如果启用，关闭内燃机
            if (quadplane.land_icengine_cut != 0) {
                plane.g2.ice_control.engine_control(0, 0, 0, false);
            }
#endif  // AP_ICENGINE_ENABLED
        }
        // 计算下降速率
        float height_above_ground = plane.relative_ground_altitude(plane.g.rangefinder_landing);
        float descent_rate_cms = quadplane.landing_descent_rate_cms(height_above_ground);

        // 设置接地预期
        if (poscontrol.get_state() == QuadPlane::QPOS_LAND_FINAL && !quadplane.option_is_set(QuadPlane::OPTION::DISABLE_GROUND_EFFECT_COMP)) {
            ahrs.set_touchdown_expected(true);
        }

        // 执行着陆
        pos_control->land_at_climb_rate_cm(-descent_rate_cms, descent_rate_cms>0);
        quadplane.check_land_complete();
    } else if (plane.control_mode == &plane.mode_guided && quadplane.guided_takeoff) {
        // 引导起飞模式
        quadplane.set_climb_rate_cms(0);
    } else {
        // 更新高度目标并调用位置控制器
        quadplane.set_climb_rate_cms(quadplane.get_pilot_desired_climb_rate_cms());
    }
    // 运行Z轴控制器
    quadplane.run_z_controller();

    // 使用固定翼表面进行稳定
    plane.stabilize_roll();
    plane.stabilize_pitch();
}

#endif
