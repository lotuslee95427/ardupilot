/*
  copter specific AP_AdvancedFailsafe class
 */

#include "Copter.h"

#if ADVANCED_FAILSAFE

/*
  为所有通道设置终止值的radio_out值
 */
void AP_AdvancedFailsafe_Copter::terminate_vehicle(void)
{
    // 如果终止动作设置为着陆
    if (_terminate_action == TERMINATE_ACTION_LAND) {
        // 切换到着陆模式
        copter.set_mode(Mode::Number::LAND, ModeReason::TERMINATE);
    } else {
        // 停止电机
        copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        copter.motors->output();

        // 同时解锁
        copter.arming.disarm(AP_Arming::Method::AFS);
    
        // 设置所有辅助通道到微调值
        // 设置直升机主旋翼转速通道
        SRV_Channels::set_output_limit(SRV_Channel::k_heli_rsc, SRV_Channel::Limit::TRIM);
        // 设置直升机尾桨转速通道
        SRV_Channels::set_output_limit(SRV_Channel::k_heli_tail_rsc, SRV_Channel::Limit::TRIM);
        // 设置引擎运行使能通道
        SRV_Channels::set_output_limit(SRV_Channel::k_engine_run_enable, SRV_Channel::Limit::TRIM);
        // 设置点火通道
        SRV_Channels::set_output_limit(SRV_Channel::k_ignition, SRV_Channel::Limit::TRIM);
        // 设置未使用通道
        SRV_Channels::set_output_limit(SRV_Channel::k_none, SRV_Channel::Limit::TRIM);
        // 设置手动控制通道
        SRV_Channels::set_output_limit(SRV_Channel::k_manual, SRV_Channel::Limit::TRIM);
    }

    // 输出所有通道的值
    SRV_Channels::output_ch_all();
}

void AP_AdvancedFailsafe_Copter::setup_IO_failsafe(void)
{
    // setup failsafe for all aux channels
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_heli_rsc, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_heli_tail_rsc, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_engine_run_enable, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_ignition, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_none, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_manual, SRV_Channel::Limit::TRIM);

#if FRAME_CONFIG != HELI_FRAME
    // setup AP_Motors outputs for failsafe
    uint32_t mask = copter.motors->get_motor_mask();
    hal.rcout->set_failsafe_pwm(mask, copter.motors->get_pwm_output_min());
#endif
}

/*
  return an AFS_MODE for current control mode
 */
AP_AdvancedFailsafe::control_mode AP_AdvancedFailsafe_Copter::afs_mode(void)
{
    return copter.flightmode->afs_mode();
}

//to force entering auto mode when datalink loss 
 void AP_AdvancedFailsafe_Copter::set_mode_auto(void)
 {
    copter.set_mode(Mode::Number::AUTO,ModeReason::GCS_FAILSAFE);
 }
#endif // ADVANCED_FAILSAFE
