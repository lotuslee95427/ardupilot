#include "mode.h"
#include "Plane.h"

// 更新训练模式
void ModeTraining::update()
{
    // 初始化手动控制标志
    plane.training_manual_roll = false;
    plane.training_manual_pitch = false;
    plane.update_load_factor();

    // 检查横滚角是否超过限制
    if (ahrs.roll_sensor >= plane.roll_limit_cd) {
        plane.nav_roll_cd = plane.roll_limit_cd;
    } else if (ahrs.roll_sensor <= -plane.roll_limit_cd) {
        plane.nav_roll_cd = -plane.roll_limit_cd;
    } else {
        // 在限制范围内，允许手动控制
        plane.training_manual_roll = true;
        plane.nav_roll_cd = 0;
    }

    // 检查俯仰角是否超过限制
    if (ahrs.pitch_sensor >= plane.aparm.pitch_limit_max*100) {
        plane.nav_pitch_cd = plane.aparm.pitch_limit_max*100;
    } else if (ahrs.pitch_sensor <= plane.pitch_limit_min*100) {
        plane.nav_pitch_cd = plane.pitch_limit_min*100;
    } else {
        // 在限制范围内，允许手动控制
        plane.training_manual_pitch = true;
        plane.nav_pitch_cd = 0;
    }
    // 如果是倒飞模式，反转俯仰角
    if (plane.fly_inverted()) {
        plane.nav_pitch_cd = -plane.nav_pitch_cd;
    }
}

// 训练模式的特殊稳定功能
void ModeTraining::run()
{
    // 获取经过指数处理的横滚和俯仰输入
    const float rexpo = plane.roll_in_expo(false);
    const float pexpo = plane.pitch_in_expo(false);

    // 处理横滚控制
    if (plane.training_manual_roll) {
        // 手动控制
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, rexpo);
    } else {
        // 自动稳定
        plane.stabilize_roll();
        // 允许用户退出自动横滚
        if ((plane.nav_roll_cd > 0 && rexpo < SRV_Channels::get_output_scaled(SRV_Channel::k_aileron)) ||
            (plane.nav_roll_cd < 0 && rexpo > SRV_Channels::get_output_scaled(SRV_Channel::k_aileron))) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, rexpo);
        }
    }

    // 处理俯仰控制
    if (plane.training_manual_pitch) {
        // 手动控制
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pexpo);
    } else {
        // 自动稳定
        plane.stabilize_pitch();
        // 允许用户回到水平状态
        if ((plane.nav_pitch_cd > 0 && pexpo < SRV_Channels::get_output_scaled(SRV_Channel::k_elevator)) ||
            (plane.nav_pitch_cd < 0 && pexpo > SRV_Channels::get_output_scaled(SRV_Channel::k_elevator))) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pexpo);
        }
    }

    // 始终保持方向舵手动控制
    output_rudder_and_steering(plane.rudder_in_expo(false));

    // 输出飞行员油门控制
    output_pilot_throttle();
}
