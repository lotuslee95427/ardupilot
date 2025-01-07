#include "mode.h"
#include "Plane.h"

// 进入特技模式
bool ModeAcro::_enter()
{
    // 重置锁定状态
    acro_state.locked_roll = false;
    acro_state.locked_pitch = false;
    // 获取当前四元数姿态
    IGNORE_RETURN(ahrs.get_quaternion(acro_state.q));
    return true;
}

// 更新特技模式状态
void ModeAcro::update()
{
    // 处理锁定/非锁定控制
    if (acro_state.locked_roll) {
        plane.nav_roll_cd = acro_state.locked_roll_err;
    } else {
        plane.nav_roll_cd = ahrs.roll_sensor;
    }
    if (acro_state.locked_pitch) {
        plane.nav_pitch_cd = acro_state.locked_pitch_cd;
    } else {
        plane.nav_pitch_cd = ahrs.pitch_sensor;
    }
}

// 运行特技模式
void ModeAcro::run()
{
    // 输出飞行员油门
    output_pilot_throttle();

    // 检查是否可以进行3D特技锁定
    if (plane.g.acro_locking == 2 && plane.g.acro_yaw_rate > 0 &&
        plane.yawController.rate_control_enabled()) {
        // 执行基于四元数的稳定
        stabilize_quaternion();
        return;
    }

    // 执行普通特技稳定
    stabilize();
}

// 特技模式稳定函数，执行横滚和俯仰轴的速率稳定
void ModeAcro::stabilize()
{
    const float speed_scaler = plane.get_speed_scaler();
    const float rexpo = plane.roll_in_expo(true);
    const float pexpo = plane.pitch_in_expo(true);
    float roll_rate = (rexpo/SERVO_MAX) * plane.g.acro_roll_rate;
    float pitch_rate = (pexpo/SERVO_MAX) * plane.g.acro_pitch_rate;

    // 获取当前四元数姿态
    IGNORE_RETURN(ahrs.get_quaternion(acro_state.q));

    // 检查接近俯仰极点时的特殊横滚处理
    if (plane.g.acro_locking && is_zero(roll_rate)) {
        // 无横滚摇杆输入，进入"横滚锁定"模式
        if (!acro_state.locked_roll) {
            acro_state.locked_roll = true;
            acro_state.locked_roll_err = 0;
        } else {
            acro_state.locked_roll_err += ahrs.get_gyro().x * plane.G_Dt;
        }
        int32_t roll_error_cd = -ToDeg(acro_state.locked_roll_err)*100;
        plane.nav_roll_cd = ahrs.roll_sensor + roll_error_cd;
        // 尝试将积分角误差减小到零
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.rollController.get_servo_out(roll_error_cd,
                                                                                             speed_scaler,
                                                                                             true, false));
    } else {
        // 副翼摇杆非零，使用纯速率控制
        acro_state.locked_roll = false;
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.rollController.get_rate_out(roll_rate,  speed_scaler));
    }

    // 处理俯仰控制
    if (plane.g.acro_locking && is_zero(pitch_rate)) {
        // 用户俯仰摇杆输入为零，锁定俯仰
        if (!acro_state.locked_pitch) {
            acro_state.locked_pitch = true;
            acro_state.locked_pitch_cd = ahrs.pitch_sensor;
        }
        // 尝试保持锁定的俯仰角
        plane.nav_pitch_cd = acro_state.locked_pitch_cd;
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.pitchController.get_servo_out(plane.nav_pitch_cd - ahrs.pitch_sensor,
                                                                                               speed_scaler,
                                                                                               false, false));
    } else {
        // 用户有非零俯仰输入，使用纯速率控制器
        acro_state.locked_pitch = false;
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.pitchController.get_rate_out(pitch_rate, speed_scaler));
    }

    // 处理方向舵输出
    float rudder_output;
    if (plane.g.acro_yaw_rate > 0 && plane.yawController.rate_control_enabled()) {
        // 用户要求偏航速率控制
        const float rudd_expo = plane.rudder_in_expo(true);
        const float yaw_rate = (rudd_expo/SERVO_MAX) * plane.g.acro_yaw_rate;
        rudder_output = plane.yawController.get_rate_out(yaw_rate,  speed_scaler, false);
    } else if (plane.flight_option_enabled(FlightOptions::ACRO_YAW_DAMPER)) {
        // 使用偏航控制器
        rudder_output = plane.calc_nav_yaw_coordinated();
    } else {
        // 手动方向舵控制
        rudder_output = plane.rudder_input();
    }

    // 输出方向舵和转向
    output_rudder_and_steering(rudder_output);
}

// 基于四元数的特技稳定，具有连续锁定功能。通过ACRO_LOCKING=2启用
void ModeAcro::stabilize_quaternion()
{
    const float speed_scaler = plane.get_speed_scaler();
    auto &q = acro_state.q;
    const float rexpo = plane.roll_in_expo(true);
    const float pexpo = plane.pitch_in_expo(true);
    const float yexpo = plane.rudder_in_expo(true);

    // 获取飞行员期望的速率
    float roll_rate = (rexpo/SERVO_MAX) * plane.g.acro_roll_rate;
    float pitch_rate = (pexpo/SERVO_MAX) * plane.g.acro_pitch_rate;
    float yaw_rate = (yexpo/SERVO_MAX) * plane.g.acro_yaw_rate;
    bool roll_active = !is_zero(roll_rate);
    bool pitch_active = !is_zero(pitch_rate);
    bool yaw_active = !is_zero(yaw_rate);

    // 积分目标姿态
    Vector3f r{ float(radians(roll_rate)), float(radians(pitch_rate)), float(radians(yaw_rate)) };
    r *= plane.G_Dt;
    q.rotate_fast(r);
    q.normalize();

    // 填充目标横滚/俯仰角度用于GCS/日志
    plane.nav_roll_cd = degrees(q.get_euler_roll())*100;
    plane.nav_pitch_cd = degrees(q.get_euler_pitch())*100;

    // 获取AHRS姿态
    Quaternion ahrs_q;
    IGNORE_RETURN(ahrs.get_quaternion(ahrs_q));

    // 在不飞行、无摇杆输入和零油门时将目标置零
    if (is_zero(plane.get_throttle_input()) &&
        !plane.is_flying() &&
        is_zero(roll_rate) &&
        is_zero(pitch_rate) &&
        is_zero(yaw_rate)) {
        // 处理在地面上中立摇杆、无油门的情况
        q = ahrs_q;
    }

    // 获取姿态误差
    Quaternion error_quat = ahrs_q.inverse() * q;
    Vector3f error_angle1;
    error_quat.to_axis_angle(error_angle1);

    // 限制误差积累，最大为0.2秒
    const float max_error_t = 0.2;
    float max_err_roll_rad  = radians(plane.g.acro_roll_rate*max_error_t);
    float max_err_pitch_rad = radians(plane.g.acro_pitch_rate*max_error_t);
    float max_err_yaw_rad   = radians(plane.g.acro_yaw_rate*max_error_t);

    // 根据摇杆活动状态调整最大误差
    if (!roll_active && acro_state.roll_active_last) {
        max_err_roll_rad = 0;
    }
    if (!pitch_active && acro_state.pitch_active_last) {
        max_err_pitch_rad = 0;
    }
    if (!yaw_active && acro_state.yaw_active_last) {
        max_err_yaw_rad = 0;
    }

    // 计算期望速率并限制在最大误差范围内
    Vector3f desired_rates = error_angle1;
    desired_rates.x = constrain_float(desired_rates.x, -max_err_roll_rad, max_err_roll_rad);
    desired_rates.y = constrain_float(desired_rates.y, -max_err_pitch_rad, max_err_pitch_rad);
    desired_rates.z = constrain_float(desired_rates.z, -max_err_yaw_rad, max_err_yaw_rad);

    // 基于最大误差修正目标
    q.rotate_fast(desired_rates - error_angle1);
    q.normalize();

    // 转换为期望的机体速率
    desired_rates.x /= plane.rollController.tau();
    desired_rates.y /= plane.pitchController.tau();
    desired_rates.z /= plane.pitchController.tau(); // 没有偏航tau参数，使用俯仰

    desired_rates *= degrees(1.0);

    // 根据摇杆输入更新期望速率
    if (roll_active) {
        desired_rates.x = roll_rate;
    }
    if (pitch_active) {
        desired_rates.y = pitch_rate;
    }
    if (yaw_active) {
        desired_rates.z = yaw_rate;
    }

    // 调用速率控制器
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron,  plane.rollController.get_rate_out(desired_rates.x, speed_scaler));
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.pitchController.get_rate_out(desired_rates.y, speed_scaler));
    output_rudder_and_steering(plane.yawController.get_rate_out(desired_rates.z,  speed_scaler, false));

    // 更新摇杆活动状态
    acro_state.roll_active_last = roll_active;
    acro_state.pitch_active_last = pitch_active;
    acro_state.yaw_active_last = yaw_active;
}
