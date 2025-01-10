#include "Copter.h"

// 定义延时常量
#define ARM_DELAY               20  // 以10Hz调用,所以是2秒
#define DISARM_DELAY            20  // 以10Hz调用,所以是2秒 
#define AUTO_TRIM_DELAY         100 // 以10Hz调用,所以是10秒
#define LOST_VEHICLE_DELAY      10  // 以10Hz调用,所以是1秒

// 自动锁定开始时间
static uint32_t auto_disarm_begin;

// arm_motors_check - 检查飞手输入以解锁或锁定飞行器
// 以10Hz频率调用
void Copter::arm_motors_check()
{
    // 解锁计数器
    static int16_t arming_counter;

    // 检查是否允许使用方向舵解锁/锁定
    AP_Arming::RudderArming arming_rudder = arming.get_rudder_arming_type();
    if (arming_rudder == AP_Arming::RudderArming::IS_DISABLED) {
        arming_counter = 0;
        return;
    }

#if TOY_MODE_ENABLED
    // 玩具模式下不允许使用摇杆解锁
    if (g2.toy_mode.enabled()) {
        // 玩具模式下不使用摇杆解锁
        return;
    }
#endif

    // 确保油门在最低位置
    if (channel_throttle->get_control_in() > 0) {
        arming_counter = 0;
        return;
    }

    // 获取偏航输入
    int16_t yaw_in = channel_yaw->get_control_in();

    // 偏航摇杆打到最右
    if (yaw_in > 4000) {

        // 增加解锁计数器,最大值为自动微调计数器加1
        if (arming_counter <= AUTO_TRIM_DELAY) {
            arming_counter++;
        }

        // 解锁电机并配置飞行参数
        if (arming_counter == ARM_DELAY && !motors->armed()) {
            // 如果解锁失败则重置计数器
            if (!arming.arm(AP_Arming::Method::RUDDER)) {
                arming_counter = 0;
            }
        }

        // 解锁电机并配置飞行参数
        if (arming_counter == AUTO_TRIM_DELAY && motors->armed() && flightmode->mode_number() == Mode::Number::STABILIZE) {
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTrim start");
            auto_trim_counter = 250;
            auto_trim_started = false;
            // 确保自动锁定不会立即触发
            auto_disarm_begin = millis();
        }

    // 偏航摇杆打到最左且允许方向舵锁定
    } else if ((yaw_in < -4000) && (arming_rudder == AP_Arming::RudderArming::ARMDISARM)) {
        if (!flightmode->has_manual_throttle() && !ap.land_complete) {
            arming_counter = 0;
            return;
        }

        // 增加计数器,最大值为锁定延时加1
        if (arming_counter <= DISARM_DELAY) {
            arming_counter++;
        }

        // 锁定电机
        if (arming_counter == DISARM_DELAY && motors->armed()) {
            arming.disarm(AP_Arming::Method::RUDDER);
        }

    // 偏航摇杆在中间位置,重置解锁计数器
    } else {
        arming_counter = 0;
    }
}

// auto_disarm_check - 如果飞行器在手动模式下油门最低且停在地面超过15秒,则自动锁定
void Copter::auto_disarm_check()
{
    // 获取当前时间
    uint32_t tnow_ms = millis();
    uint32_t disarm_delay_ms = 1000*constrain_int16(g.disarm_delay, 0, 127);

    // 如果已经锁定、自动锁定被禁用或处于THROW模式,则立即退出
    if (!motors->armed() || disarm_delay_ms == 0 || flightmode->mode_number() == Mode::Number::THROW) {
        auto_disarm_begin = tnow_ms;
        return;
    }

    // 如果电机仍在旋转,不启动自动锁定
    if (motors->get_spool_state() > AP_Motors::SpoolState::GROUND_IDLE) {
        auto_disarm_begin = tnow_ms;
        return;
    }

    // 如果使用联锁开关或电机紧急停止,则始终允许自动锁定
    if ((ap.using_interlock && !motors->get_interlock()) || SRV_Channels::get_emergency_stop()) {
#if FRAME_CONFIG != HELI_FRAME
        // 使用油门联锁开关或紧急停止时使用较短的延时,因为电机不会旋转,不太容易看出飞行器已解锁
        disarm_delay_ms /= 2;
#endif
    } else {
        // 检查油门是否在最低位置
        bool sprung_throttle_stick = (g.throttle_behavior & THR_BEHAVE_FEEDBACK_FROM_MID_STICK) != 0;
        bool thr_low;
        if (flightmode->has_manual_throttle() || !sprung_throttle_stick) {
            thr_low = ap.throttle_zero;
        } else {
            float deadband_top = get_throttle_mid() + g.throttle_deadzone;
            thr_low = channel_throttle->get_control_in() <= deadband_top;
        }

        // 如果油门不在最低位置或未着陆完成,重置计时器
        if (!thr_low || !ap.land_complete) {
            // 重置计时器
            auto_disarm_begin = tnow_ms;
        }
    }

    // 计时器到期后锁定
    if ((tnow_ms-auto_disarm_begin) >= disarm_delay_ms) {
        arming.disarm(AP_Arming::Method::DISARMDELAY);
        auto_disarm_begin = tnow_ms;
    }
}

// motors_output - 向电机库发送输出,电机库会调整并发送给电调和舵机
void Copter::motors_output()
{
#if ADVANCED_FAILSAFE
    // 这允许故障保护模块在极端情况下故意使飞行器坠毁
    // 仅在极端情况下用于满足OBC规则
    if (g2.afs.should_crash_vehicle()) {
        g2.afs.terminate_vehicle();
        if (!g2.afs.terminating_vehicle_via_landing()) {
            return;
        }
        // 着陆必须继续运行电机输出
    }
#endif

    // 更新解锁延时状态
    if (ap.in_arming_delay && (!motors->armed() || millis()-arm_time_ms > ARMING_DELAY_SEC*1.0e3f || flightmode->mode_number() == Mode::Number::THROW)) {
        ap.in_arming_delay = false;
    }

    // 输出任何舵机通道
    SRV_Channels::calc_pwm();

    // 现在cork,使所有通道输出同时发生
    SRV_Channels::cork();

    // 更新任何辅助通道的输出,用于手动直通
    SRV_Channels::output_ch_all();

    // 更新电机联锁状态
    bool interlock = motors->armed() && !ap.in_arming_delay && (!ap.using_interlock || ap.motor_interlock_switch) && !SRV_Channels::get_emergency_stop();
    if (!motors->get_interlock() && interlock) {
        motors->set_interlock(true);
        LOGGER_WRITE_EVENT(LogEvent::MOTORS_INTERLOCK_ENABLED);
    } else if (motors->get_interlock() && !interlock) {
        motors->set_interlock(false);
        LOGGER_WRITE_EVENT(LogEvent::MOTORS_INTERLOCK_DISABLED);
    }

    // 检查是否正在执行电机测试
    if (ap.motor_test) {
        // 检查是否正在执行电机测试
        motor_test_output();
    } else {
        // 向电机发送输出信号
        flightmode->output_to_motors();
    }

    // 推送所有通道
    SRV_Channels::push();
}

// 检查飞手摇杆输入以触发丢失飞行器警报
void Copter::lost_vehicle_check()
{
    // 声音警报计数器
    static uint8_t soundalarm_counter;

    // 如果辅助开关设置为飞行器警报,则禁用此功能,因为两者可能会相互干扰
    if (rc().find_channel_for_option(RC_Channel::AUX_FUNC::LOST_VEHICLE_SOUND)) {
        return;
    }

    // 确保油门在最低位置,电机未解锁,俯仰和横滚遥控达到最大值。注意:rc1=横滚 rc2=俯仰
    if (ap.throttle_zero && !motors->armed() && (channel_roll->get_control_in() > 4000) && (channel_pitch->get_control_in() > 4000)) {
        if (soundalarm_counter >= LOST_VEHICLE_DELAY) {
            if (AP_Notify::flags.vehicle_lost == false) {
                AP_Notify::flags.vehicle_lost = true;
                gcs().send_text(MAV_SEVERITY_NOTICE,"Locate Copter alarm");
            }
        } else {
            soundalarm_counter++;
        }
    } else {
        soundalarm_counter = 0;
        if (AP_Notify::flags.vehicle_lost == true) {
            AP_Notify::flags.vehicle_lost = false;
        }
    }
}
