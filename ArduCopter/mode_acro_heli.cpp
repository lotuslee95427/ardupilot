#include "Copter.h"

#if MODE_ACRO_ENABLED

#if FRAME_CONFIG == HELI_FRAME
/*
 * Init and run calls for acro flight mode for trad heli
 * 传统直升机特技飞行模式的初始化和运行调用
 */

// heli_acro_init - initialise acro controller
// heli_acro_init - 初始化特技控制器
bool ModeAcro_Heli::init(bool ignore_checks)
{
    // if heli is equipped with a flybar, then tell the attitude controller to pass through controls directly to servos
    // 如果直升机配备了飞行杆，则告诉姿态控制器直接将控制传递给舵机
    attitude_control->use_flybar_passthrough(motors->has_flybar(), motors->supports_yaw_passthrough());

    // 设置特技尾桨模式
    motors->set_acro_tail(true);
    
    // set stab collective false to use full collective pitch range
    // 设置稳定集体桨距为false，以使用全范围的集体桨距
    copter.input_manager.set_use_stab_col(false);

    // always successfully enter acro
    // 始终成功进入特技模式
    return true;
}

// heli_acro_run - runs the acro controller
// should be called at 100hz or more
// heli_acro_run - 运行特技控制器
// 应该以100Hz或更高的频率调用
void ModeAcro_Heli::run()
{
    float target_roll, target_pitch, target_yaw;
    float pilot_throttle_scaled;

    // Tradheli should not reset roll, pitch, yaw targets when motors are not runup while flying, because
    // we may be in autorotation flight.  This is so that the servos move in a realistic fashion while disarmed
    // for operational checks. Also, unlike multicopters we do not set throttle (i.e. collective pitch) to zero
    // so the swash servos move.
    // 传统直升机在飞行时电机未启动时不应重置横滚、俯仰、偏航目标，因为我们可能处于自转飞行状态。
    // 这是为了在解除武装状态下舵机能以真实的方式移动，以进行操作检查。
    // 此外，与多旋翼不同，我们不将油门（即集体桨距）设置为零，以便斜盘舵机能够移动。

    if (!motors->armed()) {
        // Motors should be Stopped
        // 电机应该停止
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else {
        // heli will not let the spool state progress to THROTTLE_UNLIMITED until motor interlock is enabled
        // 直升机在电机联锁启用之前不会让电机状态进展到THROTTLE_UNLIMITED
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        // 电机停止
        attitude_control->reset_target_and_rate(false);
        attitude_control->reset_rate_controller_I_terms();
        break;
    case AP_Motors::SpoolState::GROUND_IDLE:
        // If aircraft is landed, set target heading to current and reset the integrator
        // Otherwise motors could be at ground idle for practice autorotation
        // 如果飞机已着陆，将目标航向设置为当前航向并重置积分器
        // 否则电机可能处于地面怠速状态以进行自转练习
        if ((motors->init_targets_on_arming() && motors->using_leaky_integrator()) || (copter.ap.land_complete && !motors->using_leaky_integrator())) {
            attitude_control->reset_target_and_rate(false);
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

    if (!motors->has_flybar()){
        // convert the input to the desired body frame rate
        // 将输入转换为所需的机体坐标系角速率
        get_pilot_desired_angle_rates(channel_roll->norm_input_dz(), channel_pitch->norm_input_dz(), channel_yaw->norm_input_dz(), target_roll, target_pitch, target_yaw);
        // only mimic flybar response when trainer mode is disabled
        // 仅在训练模式禁用时模拟飞行杆响应
        if ((Trainer)g.acro_trainer.get() == Trainer::OFF) {
            // while landed always leak off target attitude to current attitude
            // 在着陆时始终将目标姿态泄漏到当前姿态
            if (copter.ap.land_complete) {
                virtual_flybar(target_roll, target_pitch, target_yaw, 3.0f, 3.0f);
            // while flying use acro balance parameters for leak rate
            // 在飞行时使用特技平衡参数作为泄漏率
            } else {
                virtual_flybar(target_roll, target_pitch, target_yaw, g.acro_balance_pitch, g.acro_balance_roll);
            }
        }
        if (motors->supports_yaw_passthrough()) {
            // if the tail on a flybar heli has an external gyro then
            // also use no deadzone for the yaw control and
            // pass-through the input direct to output.
            // 如果飞行杆直升机的尾部有外部陀螺仪，则
            // 对偏航控制也不使用死区，
            // 直接将输入传递到输出。
            target_yaw = channel_yaw->get_control_in_zero_dz();
        }

        // run attitude controller
        // 运行姿态控制器
        if (g2.acro_options.get() & uint8_t(AcroOptions::RATE_LOOP_ONLY)) {
            attitude_control->input_rate_bf_roll_pitch_yaw_2(target_roll, target_pitch, target_yaw);
        } else {
            attitude_control->input_rate_bf_roll_pitch_yaw(target_roll, target_pitch, target_yaw);
        }
    }else{
        /*
          for fly-bar passthrough use control_in values with no
          deadzone. This gives true pass-through.
          对于飞行杆直通模式，使用没有死区的control_in值。
          这提供了真正的直通。
         */
        float roll_in = channel_roll->get_control_in_zero_dz();
        float pitch_in = channel_pitch->get_control_in_zero_dz();
        float yaw_in;
        
        if (motors->supports_yaw_passthrough()) {
            // if the tail on a flybar heli has an external gyro then
            // also use no deadzone for the yaw control and
            // pass-through the input direct to output.
            // 如果飞行杆直升机的尾部有外部陀螺仪，则
            // 对偏航控制也不使用死区，
            // 直接将输入传递到输出。
            yaw_in = channel_yaw->get_control_in_zero_dz();
        } else {
            // if there is no external gyro then run the usual
            // ACRO_YAW_P gain on the input control, including
            // deadzone
            // 如果没有外部陀螺仪，则对输入控制应用通常的
            // ACRO_YAW_P增益，包括死区
            yaw_in = get_pilot_desired_yaw_rate();
        }

        // run attitude controller
        // 运行姿态控制器
        attitude_control->passthrough_bf_roll_pitch_rate_yaw(roll_in, pitch_in, yaw_in);
    }

    // get pilot's desired throttle
    // 获取飞行员期望的油门
    pilot_throttle_scaled = copter.input_manager.get_pilot_desired_collective(channel_throttle->get_control_in());

    // output pilot's throttle without angle boost
    // 输出飞行员的油门，不包括角度增强
    attitude_control->set_throttle_out(pilot_throttle_scaled, false, g.throttle_filt);
}


// virtual_flybar - acts like a flybar by leaking target atttitude back to current attitude
// virtual_flybar - 通过将目标姿态泄漏回当前姿态来模拟飞行杆
void ModeAcro_Heli::virtual_flybar( float &roll_out, float &pitch_out, float &yaw_out, float pitch_leak, float roll_leak)
{
    Vector3f rate_ef_level, rate_bf_level;

    // get attitude targets
    // 获取姿态目标
    const Vector3f att_target = attitude_control->get_att_target_euler_cd();

    // Calculate earth frame rate command for roll leak to current attitude
    // 计算横滚泄漏到当前姿态的地球坐标系角速率命令
    rate_ef_level.x = -wrap_180_cd(att_target.x - ahrs.roll_sensor) * roll_leak;

    // Calculate earth frame rate command for pitch leak to current attitude
    // 计算俯仰泄漏到当前姿态的地球坐标系角速率命令
    rate_ef_level.y = -wrap_180_cd(att_target.y - ahrs.pitch_sensor) * pitch_leak;

    // Calculate earth frame rate command for yaw
    // 计算偏航的地球坐标系角速率命令
    rate_ef_level.z = 0;

    // convert earth-frame leak rates to body-frame leak rates
    // 将地球坐标系泄漏率转换为机体坐标系泄漏率
    attitude_control->euler_rate_to_ang_vel(attitude_control->get_attitude_target_quat(), rate_ef_level, rate_bf_level);

    // combine earth frame rate corrections with rate requests
    // 将地球坐标系角速率修正与角速率请求相结合
    roll_out += rate_bf_level.x;
    pitch_out += rate_bf_level.y;
    yaw_out += rate_bf_level.z;

}
#endif  //HELI_FRAME
#endif  //MODE_ACRO_ENABLED
