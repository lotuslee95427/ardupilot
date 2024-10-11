#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 * 初始化和运行高度保持飞行模式的调用
 */

// althold_init - initialise althold controller
// althold_init - 初始化高度保持控制器
bool ModeAltHold::init(bool ignore_checks)
{

    // initialise the vertical position controller
    // 初始化垂直位置控制器
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // set vertical speed and acceleration limits
    // 设置垂直速度和加速度限制
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    return true;
}

// althold_run - runs the althold controller
// althold_run - 运行高度保持控制器
// should be called at 100hz or more
// 应该以100hz或更高的频率调用
void ModeAltHold::run()
{
    // set vertical speed and acceleration limits
    // 设置垂直速度和加速度限制
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    // 对飞行员输入应用简单模式转换
    update_simple_mode();

    // get pilot desired lean angles
    // 获取飞行员期望的倾斜角度
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max_cd());

    // get pilot's desired yaw rate
    // 获取飞行员期望的偏航速率
    float target_yaw_rate = get_pilot_desired_yaw_rate();

    // get pilot desired climb rate
    // 获取飞行员期望的爬升率
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // Alt Hold State Machine Determination
    // 高度保持状态机确定
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    // Alt Hold State Machine
    // 高度保持状态机
    switch (althold_state) {

    case AltHoldModeState::MotorStopped:
        // 电机停止
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        // 强制油门输出衰减到零
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        // 着陆地面空闲
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        // 着陆起飞前
        attitude_control->reset_rate_controller_I_terms_smoothly();
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
        break;

    case AltHoldModeState::Flying:
        // 飞行
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if AP_AVOIDANCE_ENABLED
        // apply avoidance
        // 应用避障
        copter.avoid.adjust_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
#endif

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

    // call attitude controller
    // 调用姿态控制器
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // run the vertical position controller and set output throttle
    // 运行垂直位置控制器并设置输出油门
    pos_control->update_z_controller();
}
