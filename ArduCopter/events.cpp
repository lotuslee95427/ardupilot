#include "Copter.h"

/*
 *       当失效保护状态发生变化时调用此事件
 *       布尔值failsafe反映当前状态
 */

// 检查指定的失效保护选项是否启用
bool Copter::failsafe_option(FailsafeOption opt) const
{
    return (g2.fs_options & (uint32_t)opt);
}

// 遥控器失效保护触发事件处理函数
void Copter::failsafe_radio_on_event()
{
    // 记录失效保护错误日志
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_RADIO, LogErrorCode::FAILSAFE_OCCURRED);

    // 根据FS_THR_ENABLE参数设置期望的失效保护动作
    FailsafeAction desired_action;
    switch (g.failsafe_throttle) {
        case FS_THR_DISABLED:  // 失效保护禁用
            desired_action = FailsafeAction::NONE;
            break;
        case FS_THR_ENABLED_ALWAYS_RTL:  // 返航
        case FS_THR_ENABLED_CONTINUE_MISSION:
            desired_action = FailsafeAction::RTL;
            break;
        case FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_RTL:  // 智能返航
            desired_action = FailsafeAction::SMARTRTL;
            break;
        case FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_LAND:  // 智能返航或降落
            desired_action = FailsafeAction::SMARTRTL_LAND;
            break;
        case FS_THR_ENABLED_ALWAYS_LAND:  // 降落
            desired_action = FailsafeAction::LAND;
            break;
        case FS_THR_ENABLED_AUTO_RTL_OR_RTL:  // 自动返航或返航
            desired_action = FailsafeAction::AUTO_DO_LAND_START;
            break;
        case FS_THR_ENABLED_BRAKE_OR_LAND:  // 刹车或降落
            desired_action = FailsafeAction::BRAKE_LAND;
            break;
        default:  // 默认降落
            desired_action = FailsafeAction::LAND;
    }

    // 根据特定条件修改失效保护动作并发送GCS警告
    if (should_disarm_on_failsafe()) {
        // 在地面时立即解锁
        announce_failsafe("Radio", "Disarming");
        arming.disarm(AP_Arming::Method::RADIOFAILSAFE);
        desired_action = FailsafeAction::NONE;

    } else if (flightmode->is_landing() && ((battery.has_failsafed() && battery.get_highest_failsafe_priority() <= FAILSAFE_LAND_PRIORITY))) {
        // 当电池失效保护要求时继续降落(非用户选项)
        announce_failsafe("Radio + Battery", "Continuing Landing");
        desired_action = FailsafeAction::LAND;

    } else if (flightmode->is_landing() && failsafe_option(FailsafeOption::CONTINUE_IF_LANDING)) {
        // 当FS_OPTIONS设置为继续降落时允许继续降落
        announce_failsafe("Radio", "Continuing Landing");
        desired_action = FailsafeAction::LAND;

    } else if (flightmode->mode_number() == Mode::Number::AUTO && failsafe_option(FailsafeOption::RC_CONTINUE_IF_AUTO)) {
        // 当FS_OPTIONS设置为继续任务时允许继续自动模式
        announce_failsafe("Radio", "Continuing Auto");
        desired_action = FailsafeAction::NONE;

    } else if ((flightmode->in_guided_mode()) && failsafe_option(FailsafeOption::RC_CONTINUE_IF_GUIDED)) {
        // 当FS_OPTIONS设置为继续引导模式时允许继续引导模式
        announce_failsafe("Radio", "Continuing Guided Mode");
        desired_action = FailsafeAction::NONE;

    } else {
        announce_failsafe("Radio");
    }

    // 调用失效保护动作处理函数
    do_failsafe_action(desired_action, ModeReason::RADIO_FAILSAFE);
}

// 遥控器恢复正常事件处理函数
void Copter::failsafe_radio_off_event()
{
    // 只需记录错误已解决,无需其他操作
    // 用户现在可以控制横滚、俯仰、偏航和油门,甚至可以使用飞行模式开关恢复之前的飞行模式
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_RADIO, LogErrorCode::FAILSAFE_RESOLVED);
    gcs().send_text(MAV_SEVERITY_WARNING, "Radio Failsafe Cleared");
}

// 向地面站发送失效保护消息
void Copter::announce_failsafe(const char *type, const char *action_undertaken)
{
    if (action_undertaken != nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "%s Failsafe - %s", type, action_undertaken);
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "%s Failsafe", type);
    }
}

// 电池失效保护处理函数
void Copter::handle_battery_failsafe(const char *type_str, const int8_t action)
{
    // 记录电池失效保护错误日志
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_BATT, LogErrorCode::FAILSAFE_OCCURRED);

    FailsafeAction desired_action = (FailsafeAction)action;

    // 根据特定条件修改BATT_FS_XXX_ACT参数设置
    if (should_disarm_on_failsafe()) {
        // 在地面时立即解锁
        arming.disarm(AP_Arming::Method::BATTERYFAILSAFE);
        desired_action = FailsafeAction::NONE;
        announce_failsafe("Battery", "Disarming");

    } else if (flightmode->is_landing() && failsafe_option(FailsafeOption::CONTINUE_IF_LANDING) && desired_action != FailsafeAction::NONE) {
        // 当FS_OPTIONS设置为继续降落时允许继续降落
        desired_action = FailsafeAction::LAND;
        announce_failsafe("Battery", "Continuing Landing");
    } else {
        announce_failsafe("Battery");
    }

    // 电池失效保护选项已经使用Failsafe_Options枚举,直接使用
    do_failsafe_action(desired_action, ModeReason::BATTERY_FAILSAFE);

}

// 检查地面站失效保护
void Copter::failsafe_gcs_check()
{
    // 如果禁用或GCS从未连接,跳过GCS失效保护检查
    if (g.failsafe_gcs == FS_GCS_DISABLED) {
        return;
    }

    const uint32_t gcs_last_seen_ms = gcs().sysid_myggcs_last_seen_time_ms();
    if (gcs_last_seen_ms == 0) {
        return;
    }

    // 计算自上次GCS更新以来的时间
    // 注意:这只检查由g.sysid_my_gcs设置的设备ID的心跳包
    const uint32_t last_gcs_update_ms = millis() - gcs_last_seen_ms;
    const uint32_t gcs_timeout_ms = uint32_t(constrain_float(g2.fs_gcs_timeout * 1000.0f, 0.0f, UINT32_MAX));

    // 确定要触发的事件
    if (last_gcs_update_ms < gcs_timeout_ms && failsafe.gcs) {
        // 从GCS失效保护中恢复
        set_failsafe_gcs(false);
        failsafe_gcs_off_event();

    } else if (last_gcs_update_ms < gcs_timeout_ms && !failsafe.gcs) {
        // 无问题,不做任何操作

    } else if (last_gcs_update_ms > gcs_timeout_ms && failsafe.gcs) {
        // 已经处于失效保护状态,不做任何操作

    } else if (last_gcs_update_ms > gcs_timeout_ms && !failsafe.gcs) {
        // 新的GCS失效保护事件,触发事件
        set_failsafe_gcs(true);
        failsafe_gcs_on_event();
    }
}

// GCS失效保护触发事件处理函数
void Copter::failsafe_gcs_on_event(void)
{
    // 记录GCS失效保护错误日志
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_GCS, LogErrorCode::FAILSAFE_OCCURRED);
    RC_Channels::clear_overrides();

    // 将期望的失效保护响应转换为FailsafeAction枚举
    FailsafeAction desired_action;
    switch (g.failsafe_gcs) {
        case FS_GCS_DISABLED:  // 禁用
            desired_action = FailsafeAction::NONE;
            break;
        case FS_GCS_ENABLED_ALWAYS_RTL:  // 返航
        case FS_GCS_ENABLED_CONTINUE_MISSION:
            desired_action = FailsafeAction::RTL;
            break;
        case FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_RTL:  // 智能返航
            desired_action = FailsafeAction::SMARTRTL;
            break;
        case FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_LAND:  // 智能返航或降落
            desired_action = FailsafeAction::SMARTRTL_LAND;
            break;
        case FS_GCS_ENABLED_ALWAYS_LAND:  // 降落
            desired_action = FailsafeAction::LAND;
            break;
        case FS_GCS_ENABLED_AUTO_RTL_OR_RTL:  // 自动返航或返航
            desired_action = FailsafeAction::AUTO_DO_LAND_START;
            break;
        case FS_GCS_ENABLED_BRAKE_OR_LAND:  // 刹车或降落
            desired_action = FailsafeAction::BRAKE_LAND;
            break;
        default:  // 如果参数值无效,默认返航
            desired_action = FailsafeAction::RTL;
    }

    // 根据特定条件修改FS_GCS_ENABLE参数设置
    if (!motors->armed()) {
        desired_action = FailsafeAction::NONE;
        announce_failsafe("GCS");

    } else if (should_disarm_on_failsafe()) {
        // 在地面时立即解锁
        arming.disarm(AP_Arming::Method::GCSFAILSAFE);
        desired_action = FailsafeAction::NONE;
        announce_failsafe("GCS", "Disarming");

    } else if (flightmode->is_landing() && ((battery.has_failsafed() && battery.get_highest_failsafe_priority() <= FAILSAFE_LAND_PRIORITY))) {
        // 当电池失效保护要求时继续降落(非用户选项)
        announce_failsafe("GCS + Battery", "Continuing Landing");
        desired_action = FailsafeAction::LAND;

    } else if (flightmode->is_landing() && failsafe_option(FailsafeOption::CONTINUE_IF_LANDING)) {
        // 当FS_OPTIONS设置为继续降落时允许继续降落
        announce_failsafe("GCS", "Continuing Landing");
        desired_action = FailsafeAction::LAND;

    } else if (flightmode->mode_number() == Mode::Number::AUTO && failsafe_option(FailsafeOption::GCS_CONTINUE_IF_AUTO)) {
        // 当FS_OPTIONS设置为继续任务时允许继续自动模式
        announce_failsafe("GCS", "Continuing Auto Mode");
        desired_action = FailsafeAction::NONE;

    } else if (failsafe_option(FailsafeOption::GCS_CONTINUE_IF_PILOT_CONTROL) && !flightmode->is_autopilot()) {
        // 当FS_OPTIONS设置为在手动控制模式下继续时,应该继续
        announce_failsafe("GCS", "Continuing Pilot Control");
        desired_action = FailsafeAction::NONE;
    } else {
        announce_failsafe("GCS");
    }

    // 调用失效保护动作处理函数
    do_failsafe_action(desired_action, ModeReason::GCS_FAILSAFE);
}

// GCS恢复正常事件处理函数
void Copter::failsafe_gcs_off_event(void)
{
    gcs().send_text(MAV_SEVERITY_WARNING, "GCS Failsafe Cleared");
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_GCS, LogErrorCode::FAILSAFE_RESOLVED);
}

// 如果地形数据丢失超过几秒钟,执行地形失效保护
void Copter::failsafe_terrain_check()
{
    // 在各种模式下,在失败后的<n>毫秒内触发
    bool timeout = (failsafe.terrain_last_failure_ms - failsafe.terrain_first_failure_ms) > FS_TERRAIN_TIMEOUT_MS;
    bool trigger_event = timeout && flightmode->requires_terrain_failsafe();

    // 检查事件是否清除
    if (trigger_event != failsafe.terrain) {
        if (trigger_event) {
            failsafe_terrain_on_event();
        } else {
            LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_TERRAIN, LogErrorCode::ERROR_RESOLVED);
            failsafe.terrain = false;
        }
    }
}

// 设置地形数据状态(找到或未找到)
void Copter::failsafe_terrain_set_status(bool data_ok)
{
    uint32_t now = millis();

    // 记录第一次和最近一次失败的时间(即失败持续时间)
    if (!data_ok) {
        failsafe.terrain_last_failure_ms = now;
        if (failsafe.terrain_first_failure_ms == 0) {
            failsafe.terrain_first_failure_ms = now;
        }
    } else {
        // 在持续成功0.1秒后清除失败
        if (now - failsafe.terrain_last_failure_ms > 100) {
            failsafe.terrain_last_failure_ms = 0;
            failsafe.terrain_first_failure_ms = 0;
        }
    }
}

// 地形失效保护动作
void Copter::failsafe_terrain_on_event()
{
    failsafe.terrain = true;
    gcs().send_text(MAV_SEVERITY_CRITICAL,"Failsafe: Terrain data missing");
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_TERRAIN, LogErrorCode::FAILSAFE_OCCURRED);

    if (should_disarm_on_failsafe()) {
        arming.disarm(AP_Arming::Method::TERRAINFAILSAFE);
#if MODE_RTL_ENABLED
    } else if (flightmode->mode_number() == Mode::Number::RTL) {
        mode_rtl.restart_without_terrain();
#endif
    } else {
        set_mode_RTL_or_land_with_pause(ModeReason::TERRAIN_FAILSAFE);
    }
}

// 检查GPS故障失效保护
void Copter::gpsglitch_check()
{
    // 获取导航滤波器状态
    nav_filter_status filt_status = inertial_nav.get_filter_status();
    bool gps_glitching = filt_status.flags.gps_glitching;

    // 记录GPS故障的开始或结束。AP_Notify更新在AP_AHRS内部处理
    if (ap.gps_glitching != gps_glitching) {
        ap.gps_glitching = gps_glitching;
        if (gps_glitching) {
            // 记录GPS故障错误日志
            LOGGER_WRITE_ERROR(LogErrorSubsystem::GPS, LogErrorCode::GPS_GLITCH);
            gcs().send_text(MAV_SEVERITY_CRITICAL,"GPS Glitch or Compass error");
        } else {
            // 记录故障已解决
            LOGGER_WRITE_ERROR(LogErrorSubsystem::GPS, LogErrorCode::ERROR_RESOLVED);
            gcs().send_text(MAV_SEVERITY_CRITICAL,"Glitch cleared");
        }
    }
}

// 航位推算警告和失效保护检查
void Copter::failsafe_deadreckon_check()
{
    // 更新航位推算状态
    const char* dr_prefix_str = "Dead Reckoning";

    // 获取EKF滤波器状态
    bool ekf_dead_reckoning = inertial_nav.get_filter_status().flags.dead_reckoning;

    // 向用户提示航位推算的开始或结束
    const uint32_t now_ms = AP_HAL::millis();
    if (dead_reckoning.active != ekf_dead_reckoning) {
        dead_reckoning.active = ekf_dead_reckoning;
        if (dead_reckoning.active) {
            // 记录开始时间并发送提示
            dead_reckoning.start_ms = now_ms;
            gcs().send_text(MAV_SEVERITY_CRITICAL,"%s started", dr_prefix_str);
        } else {
            // 清除状态并发送提示
            dead_reckoning.start_ms = 0;
            dead_reckoning.timeout = false;
            gcs().send_text(MAV_SEVERITY_CRITICAL,"%s stopped", dr_prefix_str);
        }
    }

    // 检查超时
    if (dead_reckoning.active && !dead_reckoning.timeout) {
        const uint32_t dr_timeout_ms = uint32_t(constrain_float(g2.failsafe_dr_timeout * 1000.0f, 0.0f, UINT32_MAX));
        if (now_ms - dead_reckoning.start_ms > dr_timeout_ms) {
            dead_reckoning.timeout = true;
            gcs().send_text(MAV_SEVERITY_CRITICAL,"%s timeout", dr_prefix_str);
        }
    }

    // 如果航位推算失效保护被禁用则立即退出
    if (g2.failsafe_dr_enable <= 0) {
        failsafe.deadreckon = false;
        return;
    }

    // 检查是否需要执行失效保护动作
    if (failsafe.deadreckon != ekf_dead_reckoning) {
        failsafe.deadreckon = ekf_dead_reckoning;

        // 仅在需要位置估计的模式下执行动作
        if (failsafe.deadreckon && copter.flightmode->requires_GPS()) {

            // 记录错误日志
            LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_DEADRECKON, LogErrorCode::FAILSAFE_OCCURRED);

            // 在着陆状态下立即解锁
            if (should_disarm_on_failsafe()) {
                arming.disarm(AP_Arming::Method::DEADRECKON_FAILSAFE);
                return;
            }

            // 执行用户指定的动作
            do_failsafe_action((FailsafeAction)g2.failsafe_dr_enable.get(), ModeReason::DEADRECKON_FAILSAFE);
        }
    }
}

// 设置模式为RTL或带暂停的降落模式
// 这总是从失效保护调用,所以我们触发对飞手的通知
void Copter::set_mode_RTL_or_land_with_pause(ModeReason reason)
{
#if MODE_RTL_ENABLED
    // 尝试切换到RTL模式,如果失败则切换到降落模式
    if (set_mode(Mode::Number::RTL, reason)) {
        AP_Notify::events.failsafe_mode_change = 1;
        return;
    }
#endif
    // 设置为降落模式会触发对飞手的模式改变通知
    set_mode_land_with_pause(reason);
}

// 设置模式为智能RTL或带暂停的降落模式
// 这总是从失效保护调用,所以我们触发对飞手的通知
void Copter::set_mode_SmartRTL_or_land_with_pause(ModeReason reason)
{
#if MODE_SMARTRTL_ENABLED
    // 尝试切换到智能RTL模式,如果失败则切换到降落模式
    if (set_mode(Mode::Number::SMART_RTL, reason)) {
        AP_Notify::events.failsafe_mode_change = 1;
        return;
    }
#endif
    gcs().send_text(MAV_SEVERITY_WARNING, "SmartRTL Unavailable, Using Land Mode");
    set_mode_land_with_pause(reason);
}

// 设置模式为智能RTL或RTL或带暂停的降落模式
// 这总是从失效保护调用,所以我们触发对飞手的通知
void Copter::set_mode_SmartRTL_or_RTL(ModeReason reason)
{
#if MODE_SMARTRTL_ENABLED
    // 尝试切换到智能RTL模式,如果失败则尝试RTL
    // 如果RTL也失败,则降落
    if (set_mode(Mode::Number::SMART_RTL, reason)) {
        AP_Notify::events.failsafe_mode_change = 1;
        return;
    }
#endif
    gcs().send_text(MAV_SEVERITY_WARNING, "SmartRTL Unavailable, Trying RTL Mode");
    set_mode_RTL_or_land_with_pause(reason);
}

// 设置模式为自动并跳转到DO_LAND_START,由AUTO_RTL参数设置
// 这可能来自失效保护或RC选项
void Copter::set_mode_auto_do_land_start_or_RTL(ModeReason reason)
{
#if MODE_AUTO_ENABLED
    if (set_mode(Mode::Number::AUTO_RTL, reason)) {
        AP_Notify::events.failsafe_mode_change = 1;
        return;
    }
#endif

    gcs().send_text(MAV_SEVERITY_WARNING, "Trying RTL Mode");
    set_mode_RTL_or_land_with_pause(reason);
}

// 设置模式为刹车或带暂停的降落模式
// 这可能来自失效保护或RC选项
void Copter::set_mode_brake_or_land_with_pause(ModeReason reason)
{
#if MODE_BRAKE_ENABLED
    if (set_mode(Mode::Number::BRAKE, reason)) {
        AP_Notify::events.failsafe_mode_change = 1;
        return;
    }
#endif

    gcs().send_text(MAV_SEVERITY_WARNING, "Trying Land Mode");
    set_mode_land_with_pause(reason);
}

// 检查是否应该在失效保护时解锁
bool Copter::should_disarm_on_failsafe() {
    // 如果在解锁延时中,则解锁
    if (ap.in_arming_delay) {
        return true;
    }

    switch (flightmode->mode_number()) {
        case Mode::Number::STABILIZE:
        case Mode::Number::ACRO:
            // 如果油门为零或已着陆则解锁电机
            return ap.throttle_zero || ap.land_complete;
        case Mode::Number::AUTO:
        case Mode::Number::AUTO_RTL:
            // 如果任务未开始且已着陆,则解锁电机
            return !ap.auto_armed && ap.land_complete;
        default:
            // 用于定高、引导、留待、返航、绕圈、漂移、运动、翻转、自动调参、定点等模式
            // 如果已着陆则解锁
            return ap.land_complete;
    }
}

// 执行失效保护动作
void Copter::do_failsafe_action(FailsafeAction action, ModeReason reason){

    // 根据指定的动作执行相应操作
    switch (action) {
        case FailsafeAction::NONE:
            return;
        case FailsafeAction::LAND:
            set_mode_land_with_pause(reason);
            break;
        case FailsafeAction::RTL:
            set_mode_RTL_or_land_with_pause(reason);
            break;
        case FailsafeAction::SMARTRTL:
            set_mode_SmartRTL_or_RTL(reason);
            break;
        case FailsafeAction::SMARTRTL_LAND:
            set_mode_SmartRTL_or_land_with_pause(reason);
            break;
        case FailsafeAction::TERMINATE: {
#if ADVANCED_FAILSAFE
            g2.afs.gcs_terminate(true, "Failsafe");
#else
            arming.disarm(AP_Arming::Method::FAILSAFE_ACTION_TERMINATE);
#endif
            break;
        }
        case FailsafeAction::AUTO_DO_LAND_START:
            set_mode_auto_do_land_start_or_RTL(reason);
            break;
        case FailsafeAction::BRAKE_LAND:
            set_mode_brake_or_land_with_pause(reason);
            break;
    }

#if AP_GRIPPER_ENABLED
    // 如果启用了释放抓取器选项,则释放抓取器
    if (failsafe_option(FailsafeOption::RELEASE_GRIPPER)) {
        gripper.release();
    }
#endif
}
