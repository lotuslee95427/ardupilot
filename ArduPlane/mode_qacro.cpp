#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

// 进入QACRO模式
bool ModeQAcro::_enter()
{
    quadplane.throttle_wait = false;
    quadplane.transition->force_transition_complete();
    attitude_control->relax_attitude_controllers();

    // 禁用偏航速率时间常数以保持旧行为
    quadplane.disable_yaw_rate_time_constant();

    // 获取当前四元数姿态
    IGNORE_RETURN(ahrs.get_quaternion(plane.mode_acro.acro_state.q));

    return true;
}

// 更新QACRO模式
void ModeQAcro::update()
{
    // 从多旋翼姿态控制器获取导航滚转和俯仰角
    Vector3f att_target = plane.quadplane.attitude_control->get_att_target_euler_cd();
    plane.nav_pitch_cd = att_target.y;
    plane.nav_roll_cd = att_target.x;
    return;
}

/*
  控制QACRO模式
 */
void ModeQAcro::run()
{
    const uint32_t now = AP_HAL::millis();
    if (quadplane.tailsitter.in_vtol_transition(now)) {
        // 尾座式飞机在VTOL转换的前向飞行拉起阶段运行固定翼控制器
        Mode::run();
        return;
    }

    if (quadplane.throttle_wait) {
        // 如果在等待油门，设置为地面怠速状态
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0, true, 0);
        quadplane.relax_attitude_control();
    } else {
        // 设置为无限制油门状态
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // 将输入转换为所需的机体帧速率
        float target_roll = 0;
        float target_pitch = plane.channel_pitch->norm_input() * quadplane.acro_pitch_rate * 100.0f;
        float target_yaw = 0;
        if (quadplane.tailsitter.enabled()) {
            // 注意，多旋翼模式下90度Y轴旋转会交换机体帧的滚转和偏航
            target_roll =  plane.channel_rudder->norm_input() * quadplane.acro_yaw_rate * 100.0f;
            target_yaw  = -plane.channel_roll->norm_input() * quadplane.acro_roll_rate * 100.0f;
        } else {
            target_roll = plane.channel_roll->norm_input() * quadplane.acro_roll_rate * 100.0f;
            target_yaw  = plane.channel_rudder->norm_input() * quadplane.acro_yaw_rate * 100.0;
        }

        // 获取飞行员油门输入
        float throttle_out = quadplane.get_pilot_throttle();

        // 运行姿态控制器
        if (plane.g.acro_locking) {
            attitude_control->input_rate_bf_roll_pitch_yaw_3(target_roll, target_pitch, target_yaw);
        } else {
            attitude_control->input_rate_bf_roll_pitch_yaw_2(target_roll, target_pitch, target_yaw);
        }

        // 输出飞行员的油门，不进行角度增益
        attitude_control->set_throttle_out(throttle_out, false, 10.0f);
    }

    // 使用固定翼表面进行稳定
    plane.mode_acro.run();
}

#endif
