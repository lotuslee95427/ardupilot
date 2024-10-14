#include "Copter.h"

#if FRAME_CONFIG == HELI_FRAME
/*
 * Init and run calls for stabilize flight mode for trad heli
 * 传统直升机稳定飞行模式的初始化和运行调用
 */

// stabilize_init - initialise stabilize controller
// stabilize_init - 初始化稳定控制器
bool ModeStabilize_Heli::init(bool ignore_checks)
{
    // be aware that when adding code to this function that it is *NOT
    // RUN* at vehicle startup!
    // 注意,在向此函数添加代码时,它在车辆启动时*不会运行*!

    // set stab collective true to use stabilize scaled collective pitch range
    // 设置stab_collective为true,以使用稳定模式下的缩放集体螺距范围
    copter.input_manager.set_use_stab_col(true);

    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
// stabilize_run - 运行主稳定控制器
// 应该以100Hz或更高的频率调用
void ModeStabilize_Heli::run()
{
    float target_roll, target_pitch;
    float pilot_throttle_scaled;

    // apply SIMPLE mode transform to pilot inputs
    // 对飞行员输入应用SIMPLE模式转换
    update_simple_mode();

    // convert pilot input to lean angles
    // 将飞行员输入转换为倾斜角度
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    // 获取飞行员期望的偏航速率
    float target_yaw_rate = get_pilot_desired_yaw_rate();

    // get pilot's desired throttle
    // 获取飞行员期望的油门
    pilot_throttle_scaled = copter.input_manager.get_pilot_desired_collective(channel_throttle->get_control_in());

    // Tradheli should not reset roll, pitch, yaw targets when motors are not runup while flying, because
    // we may be in autorotation flight.  This is so that the servos move in a realistic fashion while disarmed
    // for operational checks. Also, unlike multicopters we do not set throttle (i.e. collective pitch) to zero
    // so the swash servos move.
    // 传统直升机在飞行时电机未启动时不应重置横滚、俯仰、偏航目标，因为我们可能处于自转飞行状态。
    // 这是为了在解除武装状态下舵机能以真实的方式移动，以进行操作检查。
    // 此外，与多旋翼不同，我们不将油门(即集体螺距)设置为零，以便斜盘舵机能够移动。

    if (!motors->armed()) {
        // Motors should be Stopped
        // 电机应该停止
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else {
        // heli will not let the spool state progress to THROTTLE_UNLIMITED until motor interlock is enabled
        // 直升机在电机联锁启用之前不会让电机状态进入THROTTLE_UNLIMITED
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        // 电机停止
        attitude_control->reset_yaw_target_and_rate(false);
        attitude_control->reset_rate_controller_I_terms();
        break;
    case AP_Motors::SpoolState::GROUND_IDLE:
        // If aircraft is landed, set target heading to current and reset the integrator
        // Otherwise motors could be at ground idle for practice autorotation
        // 如果飞机已着陆，将目标航向设置为当前航向并重置积分器
        // 否则电机可能处于地面怠速状态以进行自转练习
        if ((motors->init_targets_on_arming() && motors->using_leaky_integrator()) || (copter.ap.land_complete && !motors->using_leaky_integrator())) {
            attitude_control->reset_yaw_target_and_rate(false);
            attitude_control->reset_rate_controller_I_terms_smoothly();
        }
        break;
    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        if (copter.ap.land_complete && !motors->using_leaky_integrator()) {
            attitude_control->reset_rate_controller_I_terms_smoothly();
        }
        break;
    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        // 不执行任何操作
        break;
    }

    // call attitude controller
    // 调用姿态控制器
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle
    // 输出飞行员的油门
    attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}

#endif  //HELI_FRAME
