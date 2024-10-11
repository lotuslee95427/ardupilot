#include "Copter.h"

/*
 * Init and run calls for stabilize flight mode
 * 初始化和运行稳定飞行模式的调用
 */

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
// stabilize_run - 运行主要的稳定控制器
// 应该以100hz或更高的频率调用
void ModeStabilize::run()
{
    // apply simple mode transform to pilot inputs
    // 对飞行员输入应用简单模式转换
    update_simple_mode();

    // convert pilot input to lean angles
    // 将飞行员输入转换为倾斜角度
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    // 获取飞行员期望的偏航速率
    float target_yaw_rate = get_pilot_desired_yaw_rate();

    if (!motors->armed()) {
        // Motors should be Stopped
        // 电机应该停止
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero
               || (copter.air_mode == AirMode::AIRMODE_ENABLED && motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN)) {
        // throttle_zero is never true in air mode, but the motors should be allowed to go through ground idle
        // in order to facilitate the spoolup block
        // 在空中模式下throttle_zero永远不会为真，但电机应该允许通过地面怠速
        // 以便于启动块

        // Attempting to Land
        // 尝试着陆
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    // 获取飞行员期望的油门
    float pilot_desired_throttle = get_pilot_desired_throttle();

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        // 电机停止
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        // 着陆
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        // 在油门大于零时清除着陆标志
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        // 什么也不做
        break;
    }

    // call attitude controller
    // 调用姿态控制器
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle
    // 输出飞行员的油门
    attitude_control->set_throttle_out(pilot_desired_throttle, true, g.throttle_filt);
}
