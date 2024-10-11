#include "Copter.h"

#if MODE_LOITER_ENABLED

/*
 * Init and run calls for loiter flight mode
 * 初始化和运行悬停飞行模式的调用
 */

// loiter_init - initialise loiter controller
// loiter_init - 初始化悬停控制器
bool ModeLoiter::init(bool ignore_checks)
{
    if (!copter.failsafe.radio) {
        float target_roll, target_pitch;
        // apply SIMPLE mode transform to pilot inputs
        // 对飞行员输入应用简单模式转换
        update_simple_mode();

        // convert pilot input to lean angles
        // 将飞行员输入转换为倾斜角度
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // process pilot's roll and pitch input
        // 处理飞行员的横滚和俯仰输入
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        // 清除飞行员期望的加速度，以防无线电失效事件发生且我们未能切换到RTL模式
        loiter_nav->clear_pilot_desired_acceleration();
    }
    loiter_nav->init_target();

    // initialise the vertical position controller
    // 初始化垂直位置控制器
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // set vertical speed and acceleration limits
    // 设置垂直速度和加速度限制
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

#if AC_PRECLAND_ENABLED
    _precision_loiter_active = false;
#endif

    return true;
}

#if AC_PRECLAND_ENABLED
bool ModeLoiter::do_precision_loiter()
{
    if (!_precision_loiter_enabled) {
        return false;
    }
    if (copter.ap.land_complete_maybe) {
        return false;        // don't move on the ground
        // 不要在地面上移动
    }
    // if the pilot *really* wants to move the vehicle, let them....
    // 如果飞行员真的想移动飞行器，让他们移动
    if (loiter_nav->get_pilot_desired_acceleration().length() > 50.0f) {
        return false;
    }
    if (!copter.precland.target_acquired()) {
        return false; // we don't have a good vector
        // 我们没有一个好的向量
    }
    return true;
}

void ModeLoiter::precision_loiter_xy()
{
    loiter_nav->clear_pilot_desired_acceleration();
    Vector2f target_pos, target_vel;
    if (!copter.precland.get_target_position_cm(target_pos)) {
        target_pos = inertial_nav.get_position_xy_cm();
    }
    // get the velocity of the target
    // 获取目标的速度
    copter.precland.get_target_velocity_cms(inertial_nav.get_velocity_xy_cms(), target_vel);

    Vector2f zero;
    Vector2p landing_pos = target_pos.topostype();
    // target vel will remain zero if landing target is stationary
    // 如果着陆目标是静止的，目标速度将保持为零
    pos_control->input_pos_vel_accel_xy(landing_pos, target_vel, zero);
    // run pos controller
    // 运行位置控制器
    pos_control->update_xy_controller();
}
#endif

// loiter_run - runs the loiter controller
// loiter_run - 运行悬停控制器
// should be called at 100hz or more
// 应该以100hz或更高的频率调用
void ModeLoiter::run()
{
    float target_roll, target_pitch;
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;

    // set vertical speed and acceleration limits
    // 设置垂直速度和加速度限制
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // process pilot inputs unless we are in radio failsafe
    // 处理飞行员输入，除非我们处于无线电失效状态
    if (!copter.failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        // 对飞行员输入应用简单模式转换
        update_simple_mode();

        // convert pilot input to lean angles
        // 将飞行员输入转换为倾斜角度
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // process pilot's roll and pitch input
        // 处理飞行员的横滚和俯仰输入
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);

        // get pilot's desired yaw rate
        // 获取飞行员期望的偏航速率
        target_yaw_rate = get_pilot_desired_yaw_rate();

        // get pilot desired climb rate
        // 获取飞行员期望的爬升率
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        // 清除飞行员期望的加速度，以防无线电失效事件发生且我们未能切换到RTL模式
        loiter_nav->clear_pilot_desired_acceleration();
    }

    // relax loiter target if we might be landed
    // 如果我们可能已着陆，则放松悬停目标
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Loiter State Machine Determination
    // 悬停状态机确定
    AltHoldModeState loiter_state = get_alt_hold_state(target_climb_rate);

    // Loiter State Machine
    // 悬停状态机
    switch (loiter_state) {

    case AltHoldModeState::MotorStopped:
        // 电机停止
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        // 强制油门输出衰减到零
        loiter_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        // 着陆地面空闲
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        // 着陆起飞前
        attitude_control->reset_rate_controller_I_terms_smoothly();
        loiter_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        // 强制油门输出衰减到零
        break;

    case AltHoldModeState::Takeoff:
        // 起飞
        // initiate take-off
        // 开始起飞
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        // 获取避障调整后的爬升率
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // set position controller targets adjusted for pilot input
        // 设置根据飞行员输入调整的位置控制器目标
        takeoff.do_pilot_takeoff(target_climb_rate);

        // run loiter controller
        // 运行悬停控制器
        loiter_nav->update();

        // call attitude controller
        // 调用姿态控制器
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
        break;

    case AltHoldModeState::Flying:
        // 飞行
        // set motors to full range
        // 将电机设置为全范围
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if AC_PRECLAND_ENABLED
        bool precision_loiter_old_state = _precision_loiter_active;
        if (do_precision_loiter()) {
            precision_loiter_xy();
            _precision_loiter_active = true;
        } else {
            _precision_loiter_active = false;
        }
        if (precision_loiter_old_state && !_precision_loiter_active) {
            // prec loiter was active, not any more, let's init again as user takes control
            // 精确悬停曾经是激活的，现在不再是了，当用户接管时我们再次初始化
            loiter_nav->init_target();
        }
        // run loiter controller if we are not doing prec loiter
        // 如果我们没有进行精确悬停，则运行悬停控制器
        if (!_precision_loiter_active) {
            loiter_nav->update();
        }
#else
        loiter_nav->update();
#endif

        // call attitude controller
        // 调用姿态控制器
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);

        // get avoidance adjusted climb rate
        // 获取避障调整后的爬升率
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

#if AP_RANGEFINDER_ENABLED
        // update the vertical offset based on the surface measurement
        // 根据表面测量更新垂直偏移
        copter.surface_tracking.update_surface_offset();
#endif

        // Send the commanded climb rate to the position controller
        // 将命令的爬升率发送到位置控制器
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        break;
    }

    // run the vertical position controller and set output throttle
    // 运行垂直位置控制器并设置输出油门
    pos_control->update_z_controller();
}

uint32_t ModeLoiter::wp_distance() const
{
    return loiter_nav->get_distance_to_target();
}

int32_t ModeLoiter::wp_bearing() const
{
    return loiter_nav->get_bearing_to_target();
}

#endif
