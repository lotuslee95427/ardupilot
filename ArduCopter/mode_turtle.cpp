#include "Copter.h"

#if MODE_TURTLE_ENABLED

#define CRASH_FLIP_EXPO 35.0f // 翻转指数
#define CRASH_FLIP_STICK_MINF 0.15f // 最小摇杆输入
#define power3(x) ((x) * (x) * (x)) // 计算x的三次方

bool ModeTurtle::init(bool ignore_checks)
{
    // 如果已经解锁或飞行中，不进入模式
    if (motors->armed() || SRV_Channels::get_dshot_esc_type() == 0) {
        return false;
    }

    // 执行最小解锁检查
    if (!copter.mavlink_motor_control_check(*gcs().chan(0), true, "Turtle Mode")) {
        return false;
    }

    // 如果摇杆未居中或油门不为零，不进入模式
    if (!is_zero(channel_pitch->norm_input_dz())
        || !is_zero(channel_roll->norm_input_dz())
        || !is_zero(channel_yaw->norm_input_dz())
        || !is_zero(channel_throttle->norm_input_dz())) {
        return false;
    }

    // turn on notify leds
    // 打开通知LED灯
    AP_Notify::flags.esc_calibration = true;

    return true;
}

void ModeTurtle::arm_motors()
{
    if (hal.util->get_soft_armed()) {
        return;
    }

    // stop the spoolup block activating
    // 停止启动块激活
    motors->set_spoolup_block(false);

    // reverse the motors
    // 反转电机
    hal.rcout->disable_channel_mask_updates();
    change_motor_direction(true);

    // disable throttle and gps failsafe
    // 禁用油门和GPS失效保护
    g.failsafe_throttle.set(FS_THR_DISABLED);
    g.failsafe_gcs.set(FS_GCS_DISABLED);
    g.fs_ekf_action.set(0);

    // arm
    // 解锁电机
    motors->armed(true);
    hal.util->set_soft_armed(true);
}

bool ModeTurtle::allows_arming(AP_Arming::Method method) const
{
    return true;
}

void ModeTurtle::exit()
{
    disarm_motors();

    // turn off notify leds
    // 关闭通知LED灯
    AP_Notify::flags.esc_calibration = false;
}

void ModeTurtle::disarm_motors()
{
    if (!hal.util->get_soft_armed()) {
        return;
    }

    // disarm
    // 锁定电机
    motors->armed(false);
    hal.util->set_soft_armed(false);

    // un-reverse the motors
    // 取消电机反转
    change_motor_direction(false);
    hal.rcout->enable_channel_mask_updates();

    // re-enable failsafes
    // 重新启用失效保护
    g.failsafe_throttle.load();
    g.failsafe_gcs.load();
    g.fs_ekf_action.load();
}

void ModeTurtle::change_motor_direction(bool reverse)
{
    // 根据reverse参数决定电机方向命令
    // Determine motor direction command based on the reverse parameter
    AP_HAL::RCOutput::BLHeliDshotCommand direction = reverse ? AP_HAL::RCOutput::DSHOT_REVERSE : AP_HAL::RCOutput::DSHOT_NORMAL;
    AP_HAL::RCOutput::BLHeliDshotCommand inverse_direction = reverse ? AP_HAL::RCOutput::DSHOT_NORMAL : AP_HAL::RCOutput::DSHOT_REVERSE;

    // 如果没有反转掩码，则对所有通道发送方向命令
    // If there is no reversed mask, send direction command to all channels
    if (!hal.rcout->get_reversed_mask()) {
        hal.rcout->send_dshot_command(direction, AP_HAL::RCOutput::ALL_CHANNELS, 0, 10, true);
    } else {
        // 遍历所有电机
        // Iterate through all motors
        for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
            // 如果电机未启用，则跳过
            // Skip if the motor is not enabled
            if (!motors->is_motor_enabled(i)) {
                continue;
            }

            // 根据反转掩码发送相应的方向命令
            // Send the corresponding direction command based on the reversed mask
            if ((hal.rcout->get_reversed_mask() & (1U << i)) == 0) {
                hal.rcout->send_dshot_command(direction, i, 0, 10, true);
            } else {
                hal.rcout->send_dshot_command(inverse_direction, i, 0, 10, true);
            }
        }
    }
}

void ModeTurtle::run()
{
    const float flip_power_factor = 1.0f - CRASH_FLIP_EXPO * 0.01f; // 计算翻转功率因子
    const bool norc = copter.failsafe.radio || !rc().has_ever_seen_rc_input(); // 检查是否有无线电输入
    const float stick_deflection_pitch = norc ? 0.0f : channel_pitch->norm_input_dz(); // 获取俯仰摇杆偏移
    const float stick_deflection_roll = norc ? 0.0f : channel_roll->norm_input_dz(); // 获取横滚摇杆偏移
    const float stick_deflection_yaw = norc ? 0.0f : channel_yaw->norm_input_dz(); // 获取偏航摇杆偏移

    const float stick_deflection_pitch_abs = fabsf(stick_deflection_pitch); // 计算俯仰摇杆偏移的绝对值
    const float stick_deflection_roll_abs = fabsf(stick_deflection_roll); // 计算横滚摇杆偏移的绝对值
    const float stick_deflection_yaw_abs = fabsf(stick_deflection_yaw); // 计算偏航摇杆偏移的绝对值

    const float stick_deflection_pitch_expo = flip_power_factor * stick_deflection_pitch_abs + power3(stick_deflection_pitch_abs) * (1 - flip_power_factor); // 计算俯仰摇杆偏移的指数值
    const float stick_deflection_roll_expo = flip_power_factor * stick_deflection_roll_abs + power3(stick_deflection_roll_abs) * (1 - flip_power_factor); // 计算横滚摇杆偏移的指数值
    const float stick_deflection_yaw_expo = flip_power_factor * stick_deflection_yaw_abs + power3(stick_deflection_yaw_abs) * (1 - flip_power_factor); // 计算偏航摇杆偏移的指数值

    float sign_pitch = stick_deflection_pitch < 0 ? -1 : 1; // 获取俯仰摇杆偏移的符号
    float sign_roll = stick_deflection_roll < 0 ? 1 : -1; // 获取横滚摇杆偏移的符号

    float stick_deflection_length = sqrtf(sq(stick_deflection_pitch_abs) + sq(stick_deflection_roll_abs)); // 计算摇杆偏移的长度
    float stick_deflection_expo_length = sqrtf(sq(stick_deflection_pitch_expo) + sq(stick_deflection_roll_expo)); // 计算摇杆偏移的指数长度

    if (stick_deflection_yaw_abs > MAX(stick_deflection_pitch_abs, stick_deflection_roll_abs)) {
        // If yaw is the dominant, disable pitch and roll
        // 如果偏航是主要的，禁用俯仰和横滚
        stick_deflection_length = stick_deflection_yaw_abs;
        stick_deflection_expo_length = stick_deflection_yaw_expo;
        sign_roll = 0;
        sign_pitch = 0;
    }

    const float cos_phi = (stick_deflection_length > 0) ? (stick_deflection_pitch_abs + stick_deflection_roll_abs) / (sqrtf(2.0f) * stick_deflection_length) : 0; // 计算cos_phi
    const float cos_threshold = sqrtf(3.0f) / 2.0f; // cos(PI/6.0f)

    if (cos_phi < cos_threshold) {
        // Enforce either roll or pitch exclusively, if not on diagonal
        // 如果不在对角线上，强制仅使用横滚或俯仰
        if (stick_deflection_roll_abs > stick_deflection_pitch_abs) {
            sign_pitch = 0;
        } else {
            sign_roll = 0;
        }
    }

    // Apply a reasonable amount of stick deadband
    // 应用适量的摇杆死区
    const float crash_flip_stick_min_expo = flip_power_factor * CRASH_FLIP_STICK_MINF + power3(CRASH_FLIP_STICK_MINF) * (1 - flip_power_factor);
    const float flip_stick_range = 1.0f - crash_flip_stick_min_expo;
    const float flip_power = MAX(0.0f, stick_deflection_expo_length - crash_flip_stick_min_expo) / flip_stick_range;

    // at this point we have a power value in the range 0..1
    // 此时我们有一个范围在0到1之间的功率值

    // normalise the roll and pitch input to match the motors
    // 归一化横滚和俯仰输入以匹配电机
    Vector2f input{sign_roll, sign_pitch};
    motors_input = input.normalized() * 0.5;
    // we bypass spin min and friends in the deadzone because we only want spin up when the sticks are moved
    // 我们绕过死区中的最小旋转和其他因素，因为我们只希望在移动摇杆时旋转
    motors_output = !is_zero(flip_power) ? motors->thr_lin.thrust_to_actuator(flip_power) : 0.0f;
}

// actually write values to the motors
// 实际将值写入电机
void ModeTurtle::output_to_motors()
{
    // throttle needs to be raised
    // 需要提高油门
    if (is_zero(channel_throttle->norm_input_dz())) {
        const uint32_t now = AP_HAL::millis();
        if (now - last_throttle_warning_output_ms > 5000) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Turtle: raise throttle to arm");
            last_throttle_warning_output_ms = now;
        }

        disarm_motors();
        return;
    }

    arm_motors();

    // check if motor are allowed to spin
    // 检查电机是否允许旋转
    const bool allow_output = motors->armed() && motors->get_interlock();

    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        if (!motors->is_motor_enabled(i)) {
            continue;
        }

        const Vector2f output{motors->get_roll_factor(i), motors->get_pitch_factor(i)};
        // if output aligns with input then use this motor
        // 如果输出与输入对齐，则使用此电机
        if (!allow_output || (motors_input - output).length() > 0.5) {
            motors->rc_write(i, motors->get_pwm_output_min());
            continue;
        }

        int16_t pwm = motors->get_pwm_output_min() + (motors->get_pwm_output_max() - motors->get_pwm_output_min()) * motors_output;

        motors->rc_write(i, pwm);
    }
}

#endif
