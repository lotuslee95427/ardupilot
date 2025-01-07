#include "Plane.h"

/*
 *  故障保护支持
 *  Andrew Tridgell, 2011年12月
 */

/*
 *  我们的故障保护策略是检测主循环锁定，并切换到
 *  直接将输入从RC输入传递到RC输出。
 */

/*
 *  这个failsafe_check函数从核心定时器中断
 *  以1kHz的频率调用。
 */
void Plane::failsafe_check(void)
{
    static uint16_t last_ticks;
    static uint32_t last_timestamp;
    static bool in_failsafe;
    uint32_t tnow = micros();

    const uint16_t ticks = scheduler.ticks();
    if (ticks != last_ticks) {
        // 主循环正在运行，一切正常
        last_ticks = ticks;
        last_timestamp = tnow;
        in_failsafe = false;
        return;
    }

    if (tnow - last_timestamp > 200000) {
        // 自主循环运行以来已经过去至少0.2秒。
        // 这意味着我们遇到了麻烦，或者可能处于
        // 初始化例程或日志擦除中。开始将RC
        // 输入直接传递到输出
        in_failsafe = true;
    }

    if (in_failsafe && tnow - last_timestamp > 20000) {

        // 确保我们有最新的RC输入
        rc().read_input();

        last_timestamp = tnow;

        rc().read_input();

#if AP_ADVANCEDFAILSAFE_ENABLED
        if (in_calibration) {
            // 告诉故障保护系统我们正在校准
            // 传感器，所以不要触发故障保护
            afs.heartbeat();
        }
#endif

        if (RC_Channels::get_valid_channel_count() < 5) {
            // 我们没有任何RC输入可以传递
            return;
        }

        // 每20ms将RC输入传递到输出
        RC_Channels::clear_overrides();

        float roll = roll_in_expo(false);
        float pitch = pitch_in_expo(false);
        float throttle = get_throttle_input(true);
        float rudder = rudder_in_expo(false);

        if (!arming.is_armed_and_safety_off()) {
            throttle = 0;
        }
        
        // 设置没有对应输入通道的次要输出通道
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, roll);
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pitch);
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rudder);
        SRV_Channels::set_output_scaled(SRV_Channel::k_steering, rudder);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);

        // 这允许故障保护模块故意使飞机坠毁。
        // 仅在极端情况下用于满足OBC规则
#if AP_ADVANCEDFAILSAFE_ENABLED
        if (afs.should_crash_vehicle()) {
            afs.terminate_vehicle();
            if (!afs.terminating_vehicle_via_landing()) {
                return;
            }
        }
#endif

        // 设置有对应输入通道的次要输出通道
        SRV_Channels::copy_radio_in_out(SRV_Channel::k_manual, true);
        SRV_Channels::set_output_scaled(SRV_Channel::k_flap, 0.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_flap_auto, 0.0);

        // 设置襟副翼
        flaperon_update();

        servos_output();

        // 在SITL中，我们发送伺服输出，以便我们可以验证
        // 我们正在操纵表面
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        GCS_MAVLINK *chan = gcs().chan(0);
        if (HAVE_PAYLOAD_SPACE(chan->get_chan(), SERVO_OUTPUT_RAW)) {
            chan->send_servo_output_raw();
        }
#endif
    }
}
