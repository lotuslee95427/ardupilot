/*
  飞机特定的高级失效保护（AP_AdvancedFailsafe）类
 */

#include "Plane.h"

#if AP_ADVANCEDFAILSAFE_ENABLED

/*
  设置所有通道的radio_out值为终止值
 */
void AP_AdvancedFailsafe_Plane::terminate_vehicle(void)
{
#if HAL_QUADPLANE_ENABLED
    // 如果四旋翼模式可用且终止动作设置为着陆
    if (plane.quadplane.available() && _terminate_action == TERMINATE_ACTION_LAND) {
        // 执行垂直起降着陆
        plane.set_mode(plane.mode_qland, ModeReason::FENCE_BREACHED);
        return;
    }
#endif

    // 禁用舵机通道直通
    plane.g2.servo_channels.disable_passthrough(true);
    
    if (_terminate_action == TERMINATE_ACTION_LAND) {
        // 执行常规着陆终止
        plane.landing.terminate();
    } else {
        // 移除襟翼斜率限制
        SRV_Channels::set_slew_rate(SRV_Channel::k_flap_auto, 0.0, 100, plane.G_Dt);
        SRV_Channels::set_slew_rate(SRV_Channel::k_flap, 0.0, 100, plane.G_Dt);

        // 空气动力学终止是默认的终止方法
        // 设置各个控制面为最大偏转
        SRV_Channels::set_output_scaled(SRV_Channel::k_flap_auto, 100.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_flap, 100.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, SERVO_MAX);
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, SERVO_MAX);
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, SERVO_MAX);
        
        if (plane.have_reverse_thrust()) {
            // 如果配置了反推力，使用TRIM值
            SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::TRIM);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::TRIM);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::TRIM);
        } else {
            // 否则使用最小值
            SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::MIN);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::MIN);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::MIN);
        }
        // 设置手动和其他通道为TRIM值
        SRV_Channels::set_output_limit(SRV_Channel::k_manual, SRV_Channel::Limit::TRIM);
        SRV_Channels::set_output_limit(SRV_Channel::k_none, SRV_Channel::Limit::TRIM);
    }

    // 输出舵机信号
    plane.servos_output();

#if HAL_QUADPLANE_ENABLED
    // 执行四旋翼模式的终止操作
    plane.quadplane.afs_terminate();
#endif

    // 同时解除武装以确保点火被切断
    plane.arming.disarm(AP_Arming::Method::AFS);
}

// 设置IO失效保护
void AP_AdvancedFailsafe_Plane::setup_IO_failsafe(void)
{
    // 设置所有辅助通道的失效保护限制
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_flap_auto, SRV_Channel::Limit::MAX);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_flap, SRV_Channel::Limit::MAX);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_aileron, SRV_Channel::Limit::MIN);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_rudder, SRV_Channel::Limit::MAX);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_elevator, SRV_Channel::Limit::MAX);
    
    if (plane.have_reverse_thrust()) {
        // 如果配置了反推力，使用TRIM值
        SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::TRIM);
        SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::TRIM);
        SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::TRIM);
    } else {
        // 正常油门，使用最小值
        SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::MIN);
        SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::MIN);
        SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::MIN);
    }
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_manual, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_none, SRV_Channel::Limit::TRIM);

#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.available()) {
        // 设置AP_Motors输出的失效保护
        uint32_t mask = plane.quadplane.motors->get_motor_mask();
        hal.rcout->set_failsafe_pwm(mask, plane.quadplane.motors->get_pwm_output_min());
    }
#endif
}

/*
  根据当前控制模式返回AFS_MODE
 */
AP_AdvancedFailsafe::control_mode AP_AdvancedFailsafe_Plane::afs_mode(void)
{
    if (plane.control_mode->does_auto_throttle()) {
        return AP_AdvancedFailsafe::AFS_AUTO;
    }
    if (plane.control_mode == &plane.mode_manual) {
        return AP_AdvancedFailsafe::AFS_MANUAL;
    }
    return AP_AdvancedFailsafe::AFS_STABILIZED;
}

// 在数据链路丢失时强制进入自动模式
void AP_AdvancedFailsafe_Plane::set_mode_auto(void)
{
    plane.set_mode(plane.mode_auto, ModeReason::GCS_FAILSAFE);
}

#endif // AP_ADVANCEDFAILSAFE_ENABLED
