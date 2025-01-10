#include "Copter.h"

// Code to detect a crash main ArduCopter code
#define CRASH_CHECK_TRIGGER_SEC         2       // 2 seconds inverted indicates a crash
#define CRASH_CHECK_ANGLE_DEVIATION_DEG 30.0f   // 30 degrees beyond target angle is signal we are out of control
#define CRASH_CHECK_ANGLE_MIN_DEG       15.0f   // vehicle must be leaning at least 15deg to trigger crash check
#define CRASH_CHECK_SPEED_MAX           10.0f   // vehicle must be moving at less than 10m/s to trigger crash check
#define CRASH_CHECK_ACCEL_MAX           3.0f    // vehicle must be accelerating less than 3m/s/s to be considered crashed

// Code to detect a thrust loss main ArduCopter code
#define THRUST_LOSS_CHECK_TRIGGER_SEC         1     // 1 second descent while level and high throttle indicates thrust loss
#define THRUST_LOSS_CHECK_ANGLE_DEVIATION_CD  1500  // we can't expect to maintain altitude beyond 15 degrees on all aircraft
#define THRUST_LOSS_CHECK_MINIMUM_THROTTLE    0.9f  // we can expect to maintain altitude above 90 % throttle

// Yaw imbalance check
#define YAW_IMBALANCE_IMAX_THRESHOLD 0.75f
#define YAW_IMBALANCE_WARN_MS 10000
// crash_check - 如果检测到坠机则解除电机
// 当飞行器持续超过1秒钟偏离其角度限制超过20度时,认为发生了坠机
// 以MAIN_LOOP_RATE频率调用
void Copter::crash_check()
{
    // 记录飞行器可能已坠机的迭代次数
    static uint16_t crash_counter;  

    // 如果电机未解锁、已着陆或坠机检查被禁用,则立即退出
    if (!motors->armed() || ap.land_complete || g.fs_crash_check == 0) {
        crash_counter = 0;
        return;
    }

    // 如果处于待机状态则立即退出
    if (standby_active) {
        crash_counter = 0;
        return;
    }

    // 如果处于强制飞行状态且不在着陆过程中则立即退出
    if (get_force_flying() && !flightmode->is_landing()) {
        crash_counter = 0;
        return;
    }

    // 如果不是在角度稳定飞行模式或正在翻转则立即退出
    if (!flightmode->crash_check_enabled()) {
        crash_counter = 0;
        return;
    }

#if MODE_AUTOROTATE_ENABLED
    //return immediately if in autorotation mode
    if (flightmode->mode_number() == Mode::Number::AUTOROTATE) {
        crash_counter = 0;
        return;
    }
#endif

    // vehicle not crashed if 1hz filtered acceleration is more than 3m/s (1G on Z-axis has been subtracted)
    const float filtered_acc = land_accel_ef_filter.get().length();
    if (filtered_acc >= CRASH_CHECK_ACCEL_MAX) {
        crash_counter = 0;
        return;
    }

    // check for lean angle over 15 degrees
    const float lean_angle_deg = degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch()));
    if (lean_angle_deg <= CRASH_CHECK_ANGLE_MIN_DEG) {
        crash_counter = 0;
        return;
    }

    // check for angle error over 30 degrees
    const float angle_error = attitude_control->get_att_error_angle_deg();
    if (angle_error <= CRASH_CHECK_ANGLE_DEVIATION_DEG) {
        crash_counter = 0;
        return;
    }

    // check for speed under 10m/s (if available)
    Vector3f vel_ned;
    if (ahrs.get_velocity_NED(vel_ned) && (vel_ned.length() >= CRASH_CHECK_SPEED_MAX)) {
        crash_counter = 0;
        return;
    }

    // we may be crashing
    crash_counter++;

    // check if crashing for 2 seconds
    if (crash_counter >= (CRASH_CHECK_TRIGGER_SEC * scheduler.get_loop_rate_hz())) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::CRASH_CHECK, LogErrorCode::CRASH_CHECK_CRASH);
        // send message to gcs
        gcs().send_text(MAV_SEVERITY_EMERGENCY,"Crash: Disarming: AngErr=%.0f>%.0f, Accel=%.1f<%.1f", angle_error, CRASH_CHECK_ANGLE_DEVIATION_DEG, filtered_acc, CRASH_CHECK_ACCEL_MAX);
        // disarm motors
        copter.arming.disarm(AP_Arming::Method::CRASH);
    }
}

// check for loss of thrust and trigger thrust boost in motors library
void Copter::thrust_loss_check()
{
    static uint16_t thrust_loss_counter;  // number of iterations vehicle may have been crashed

    // no-op if suppressed by flight options param
    if (copter.option_is_enabled(FlightOption::DISABLE_THRUST_LOSS_CHECK)) {
        return;
    }

    // exit immediately if thrust boost is already engaged
    if (motors->get_thrust_boost()) {
        return;
    }

    // return immediately if disarmed
    if (!motors->armed() || ap.land_complete) {
        thrust_loss_counter = 0;
        return;
    }

    // exit immediately if in standby
    if (standby_active) {
        return;
    }

    // check for desired angle over 15 degrees
    // todo: add thrust angle to AC_AttitudeControl
    const Vector3f angle_target = attitude_control->get_att_target_euler_cd();
    if (sq(angle_target.x) + sq(angle_target.y) > sq(THRUST_LOSS_CHECK_ANGLE_DEVIATION_CD)) {
        thrust_loss_counter = 0;
        return;
    }

    // check for throttle over 90% or throttle saturation
    if ((attitude_control->get_throttle_in() < THRUST_LOSS_CHECK_MINIMUM_THROTTLE) && (!motors->limit.throttle_upper)) {
        thrust_loss_counter = 0;
        return;
    }

    // check throttle is over 25% to prevent checks triggering from thrust limitations caused by low commanded throttle
    if ((attitude_control->get_throttle_in() < 0.25f)) {
        thrust_loss_counter = 0;
        return;
    }

    // check for descent
    if (!is_negative(inertial_nav.get_velocity_z_up_cms())) {
        thrust_loss_counter = 0;
        return;
    }

    // check for angle error over 30 degrees to ensure the aircraft has attitude control
    const float angle_error = attitude_control->get_att_error_angle_deg();
    if (angle_error >= CRASH_CHECK_ANGLE_DEVIATION_DEG) {
        thrust_loss_counter = 0;
        return;
    }

    // the aircraft is descending with low requested roll and pitch, at full available throttle, with attitude control
    // we may have lost thrust
    thrust_loss_counter++;

    // check if thrust loss for 1 second
    if (thrust_loss_counter >= (THRUST_LOSS_CHECK_TRIGGER_SEC * scheduler.get_loop_rate_hz())) {
        // reset counter
        thrust_loss_counter = 0;
        LOGGER_WRITE_ERROR(LogErrorSubsystem::THRUST_LOSS_CHECK, LogErrorCode::FAILSAFE_OCCURRED);
        // send message to gcs
        gcs().send_text(MAV_SEVERITY_EMERGENCY, "Potential Thrust Loss (%d)", (int)motors->get_lost_motor() + 1);
        // enable thrust loss handling
        motors->set_thrust_boost(true);
        // the motors library disables this when it is no longer needed to achieve the commanded output

#if AP_GRIPPER_ENABLED
        if (copter.option_is_enabled(FlightOption::RELEASE_GRIPPER_ON_THRUST_LOSS)) {
            gripper.release();
        }
#endif
    }
}

// check for a large yaw imbalance, could be due to badly calibrated ESC or misaligned motors
void Copter::yaw_imbalance_check()
{
    // no-op if suppressed by flight options param
    if (copter.option_is_enabled(FlightOption::DISABLE_YAW_IMBALANCE_WARNING)) {
        return;
    }

    // If I is disabled it is unlikely that the issue is not obvious
    if (!is_positive(attitude_control->get_rate_yaw_pid().kI())) {
        return;
    }

    // thrust loss is triggered, yaw issues are expected
    if (motors->get_thrust_boost()) {
        yaw_I_filt.reset(0.0f);
        return;
    }

    // return immediately if disarmed
    if (!motors->armed() || ap.land_complete) {
        yaw_I_filt.reset(0.0f);
        return;
    }

    // exit immediately if in standby
    if (standby_active) {
        yaw_I_filt.reset(0.0f);
        return;
    }

    // magnitude of low pass filtered I term
    const float I_term = attitude_control->get_rate_yaw_pid().get_pid_info().I;
    const float I = fabsf(yaw_I_filt.apply(attitude_control->get_rate_yaw_pid().get_pid_info().I,G_Dt));
    if (I > fabsf(I_term)) {
        // never allow to be larger than I
        yaw_I_filt.reset(I_term);
    }

    const float I_max = attitude_control->get_rate_yaw_pid().imax();
    if ((is_positive(I_max) && ((I > YAW_IMBALANCE_IMAX_THRESHOLD * I_max) || (is_equal(I_term,I_max))))) {
        // filtered using over percentage of I max or unfiltered = I max
        // I makes up more than percentage of total available control power
        const uint32_t now = millis();
        if (now - last_yaw_warn_ms > YAW_IMBALANCE_WARN_MS) {
            last_yaw_warn_ms = now;
            gcs().send_text(MAV_SEVERITY_EMERGENCY, "Yaw Imbalance %0.0f%%", I *100);
        }
    }
}

#if HAL_PARACHUTE_ENABLED

// Code to detect a crash main ArduCopter code
#define PARACHUTE_CHECK_TRIGGER_SEC         1       // 1 second of loss of control triggers the parachute
#define PARACHUTE_CHECK_ANGLE_DEVIATION_DEG 30.0f   // 30 degrees off from target indicates a loss of control

// 降落伞检查 - 如果检测到严重的控制丢失,则解除电机并触发降落伞
// 当飞行器持续1秒钟偏离目标横滚和俯仰角度超过30度时,认为发生了"严重的控制丢失"
// 以MAIN_LOOP_RATE频率调用
void Copter::parachute_check()
{
    static uint16_t control_loss_count;  // 记录失控状态持续的迭代次数
    static int32_t baro_alt_start;       // 记录开始失控时的气压高度

    // 如果降落伞未启用则立即退出
    if (!parachute.enabled()) {
        return;
    }

    // 将飞行状态传递给降落伞库
    parachute.set_is_flying(!ap.land_complete);

    // 将下沉率传递给降落伞库
    parachute.set_sink_rate(-inertial_nav.get_velocity_z_up_cms() * 0.01f);

    // 如果处于待机状态则立即退出
    if (standby_active) {
        return;
    }

    // 调用update以让降落伞有机会将舵机或继电器移回关闭位置
    parachute.update();

    // 如果电机未解锁则立即退出
    if (!motors->armed()) {
        control_loss_count = 0;
        return;
    }

    // 如果降落伞已触发释放则退出
    if (parachute.release_initiated()) {
        return;
    }

    // 如果不是在角度稳定飞行模式或正在翻转则立即退出
    if (!flightmode->crash_check_enabled()) {
        control_loss_count = 0;
        return;
    }

    // 确保我们正在飞行
    if (ap.land_complete) {
        control_loss_count = 0;
        return;
    }

    // 确保第一次控制丢失事件发生在最小高度之上
    if (control_loss_count == 0 && parachute.alt_min() != 0 && (current_loc.alt < (int32_t)parachute.alt_min() * 100)) {
        return;
    }

    // 基于下沉率触发降落伞释放
    parachute.check_sink_rate();

    // 检查角度误差是否超过30度
    const float angle_error = attitude_control->get_att_error_angle_deg();
    if (angle_error <= PARACHUTE_CHECK_ANGLE_DEVIATION_DEG) {
        if (control_loss_count > 0) {
            control_loss_count--;
        }
        return;
    }
    // 增加控制丢失计数器,但不超过触发阈值(PARACHUTE_CHECK_TRIGGER_SEC * 调度器循环频率)
    if (control_loss_count < (PARACHUTE_CHECK_TRIGGER_SEC*scheduler.get_loop_rate_hz())) {
        control_loss_count++;
    }

    // 如果刚开始丢失控制(计数器为1),记录当前气压高度作为参考值
    if (control_loss_count == 1) {
        baro_alt_start = baro_alt;

    // 如果当前气压高度大于等于初始高度,说明飞行器没有下降,退出检查
    } else if (baro_alt >= baro_alt_start) {
        control_loss_count = 0;
        return;

    // 待办:添加检查确认飞行器确实在下降

    // 检查是否持续丢失控制超过1秒(PARACHUTE_CHECK_TRIGGER_SEC)
    } else if (control_loss_count >= (PARACHUTE_CHECK_TRIGGER_SEC*scheduler.get_loop_rate_hz())) {
        // 重置控制丢失计数器
        control_loss_count = 0;
        // 记录错误日志:坠机检查-失控
        LOGGER_WRITE_ERROR(LogErrorSubsystem::CRASH_CHECK, LogErrorCode::CRASH_CHECK_LOSS_OF_CONTROL);
        // 触发降落伞释放
        parachute_release();
    }
}

// parachute_release - trigger the release of the parachute, disarm the motors and notify the user
void Copter::parachute_release()
{
    // release parachute
    parachute.release();

#if AP_LANDINGGEAR_ENABLED
    // deploy landing gear
    landinggear.set_position(AP_LandingGear::LandingGear_Deploy);
#endif
}

// parachute_manual_release - trigger the release of the parachute, after performing some checks for pilot error
//   checks if the vehicle is landed 
void Copter::parachute_manual_release()
{
    // exit immediately if parachute is not enabled
    if (!parachute.enabled()) {
        return;
    }

    // do not release if vehicle is landed
    // do not release if we are landed or below the minimum altitude above home
    if (ap.land_complete) {
        // warn user of reason for failure
        gcs().send_text(MAV_SEVERITY_INFO,"Parachute: Landed");
        LOGGER_WRITE_ERROR(LogErrorSubsystem::PARACHUTES, LogErrorCode::PARACHUTE_LANDED);
        return;
    }

    // do not release if we are landed or below the minimum altitude above home
    if ((parachute.alt_min() != 0 && (current_loc.alt < (int32_t)parachute.alt_min() * 100))) {
        // warn user of reason for failure
        gcs().send_text(MAV_SEVERITY_ALERT,"Parachute: Too low");
        LOGGER_WRITE_ERROR(LogErrorSubsystem::PARACHUTES, LogErrorCode::PARACHUTE_TOO_LOW);
        return;
    }

    // if we get this far release parachute
    parachute_release();
}

#endif  // HAL_PARACHUTE_ENABLED
