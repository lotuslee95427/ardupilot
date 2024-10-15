#include "Copter.h"

/*
 * 高级调用以设置和更新飞行模式逻辑
 * 单个飞行模式的逻辑在control_acro.cpp, control_stabilize.cpp等文件中
 */

/*
  Mode对象的构造函数
 */
Mode::Mode(void) :
    g(copter.g),                    // 全局参数
    g2(copter.g2),                  // 第二组全局参数
    wp_nav(copter.wp_nav),          // 航点导航控制器
    loiter_nav(copter.loiter_nav),  // 悬停导航控制器
    pos_control(copter.pos_control),// 位置控制器
    inertial_nav(copter.inertial_nav), // 惯性导航系统
    ahrs(copter.ahrs),              // 姿态航向参考系统
    attitude_control(copter.attitude_control), // 姿态控制器
    motors(copter.motors),          // 电机控制器
    channel_roll(copter.channel_roll),    // 横滚通道
    channel_pitch(copter.channel_pitch),  // 俯仰通道
    channel_throttle(copter.channel_throttle), // 油门通道
    channel_yaw(copter.channel_yaw),      // 偏航通道
    G_Dt(copter.G_Dt)               // 主循环时间间隔
{ };

#if AC_PAYLOAD_PLACE_ENABLED
PayloadPlace Mode::payload_place;
#endif

// 返回与提供的模式对应的静态控制器对象
// return the static controller object corresponding to supplied mode
Mode *Copter::mode_from_mode_num(const Mode::Number mode)
{
    Mode *ret = nullptr;

    switch (mode) {
#if MODE_ACRO_ENABLED
        case Mode::Number::ACRO:
            ret = &mode_acro;
            break;
#endif

        case Mode::Number::STABILIZE:
            ret = &mode_stabilize;
            break;

        case Mode::Number::ALT_HOLD:
            ret = &mode_althold;
            break;

#if MODE_AUTO_ENABLED
        case Mode::Number::AUTO:
            ret = &mode_auto;
            break;
#endif

#if MODE_CIRCLE_ENABLED
        case Mode::Number::CIRCLE:
            ret = &mode_circle;
            break;
#endif

#if MODE_LOITER_ENABLED
        case Mode::Number::LOITER:
            ret = &mode_loiter;
            break;
#endif

#if MODE_GUIDED_ENABLED
        case Mode::Number::GUIDED:
            ret = &mode_guided;
            break;
#endif

        case Mode::Number::LAND:
            ret = &mode_land;
            break;

#if MODE_RTL_ENABLED
        case Mode::Number::RTL:
            ret = &mode_rtl;
            break;
#endif

#if MODE_DRIFT_ENABLED
        case Mode::Number::DRIFT:
            ret = &mode_drift;
            break;
#endif

#if MODE_SPORT_ENABLED
        case Mode::Number::SPORT:
            ret = &mode_sport;
            break;
#endif

#if MODE_FLIP_ENABLED
        case Mode::Number::FLIP:
            ret = &mode_flip;
            break;
#endif

#if AUTOTUNE_ENABLED
        case Mode::Number::AUTOTUNE:
            ret = &mode_autotune;
            break;
#endif

#if MODE_POSHOLD_ENABLED
        case Mode::Number::POSHOLD:
            ret = &mode_poshold;
            break;
#endif

#if MODE_BRAKE_ENABLED
        case Mode::Number::BRAKE:
            ret = &mode_brake;
            break;
#endif

#if MODE_THROW_ENABLED
        case Mode::Number::THROW:
            ret = &mode_throw;
            break;
#endif

#if HAL_ADSB_ENABLED
        case Mode::Number::AVOID_ADSB:
            ret = &mode_avoid_adsb;
            break;
#endif

#if MODE_GUIDED_NOGPS_ENABLED
        case Mode::Number::GUIDED_NOGPS:
            ret = &mode_guided_nogps;
            break;
#endif

#if MODE_SMARTRTL_ENABLED
        case Mode::Number::SMART_RTL:
            ret = &mode_smartrtl;
            break;
#endif

#if MODE_FLOWHOLD_ENABLED
        case Mode::Number::FLOWHOLD:
            ret = (Mode *)g2.mode_flowhold_ptr;
            break;
#endif

#if MODE_FOLLOW_ENABLED
        case Mode::Number::FOLLOW:
            ret = &mode_follow;
            break;
#endif

#if MODE_ZIGZAG_ENABLED
        case Mode::Number::ZIGZAG:
            ret = &mode_zigzag;
            break;
#endif

#if MODE_SYSTEMID_ENABLED
        case Mode::Number::SYSTEMID:
            ret = (Mode *)g2.mode_systemid_ptr;
            break;
#endif

#if MODE_AUTOROTATE_ENABLED
        case Mode::Number::AUTOROTATE:
            ret = &mode_autorotate;
            break;
#endif

#if MODE_TURTLE_ENABLED
        case Mode::Number::TURTLE:
            ret = &mode_turtle;
            break;
#endif

        default:
            break;
    }

    return ret;
}


// 当尝试切换到某个模式失败时调用此函数：
// called when an attempt to change into a mode is unsuccessful:
void Copter::mode_change_failed(const Mode *mode, const char *reason)
{
    gcs().send_text(MAV_SEVERITY_WARNING, "Mode change to %s failed: %s", mode->name(), reason);
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode->mode_number()));
    // 发出失败提示音
    // make sad noise
    if (copter.ap.initialised) {
        AP_Notify::events.user_mode_change_failed = 1;
    }
}

// 检查是否可以从地面站进入此模式
// Check if this mode can be entered from the GCS
bool Copter::gcs_mode_enabled(const Mode::Number mode_num)
{
    // 可以被阻止的模式列表，索引是参数位掩码中的位号
    // List of modes that can be blocked, index is bit number in parameter bitmask
    static const uint8_t mode_list [] {
        (uint8_t)Mode::Number::STABILIZE,
        (uint8_t)Mode::Number::ACRO,
        (uint8_t)Mode::Number::ALT_HOLD,
        (uint8_t)Mode::Number::AUTO,
        (uint8_t)Mode::Number::GUIDED,
        (uint8_t)Mode::Number::LOITER,
        (uint8_t)Mode::Number::CIRCLE,
        (uint8_t)Mode::Number::DRIFT,
        (uint8_t)Mode::Number::SPORT,
        (uint8_t)Mode::Number::FLIP,
        (uint8_t)Mode::Number::AUTOTUNE,
        (uint8_t)Mode::Number::POSHOLD,
        (uint8_t)Mode::Number::BRAKE,
        (uint8_t)Mode::Number::THROW,
        (uint8_t)Mode::Number::AVOID_ADSB,
        (uint8_t)Mode::Number::GUIDED_NOGPS,
        (uint8_t)Mode::Number::SMART_RTL,
        (uint8_t)Mode::Number::FLOWHOLD,
        (uint8_t)Mode::Number::FOLLOW,
        (uint8_t)Mode::Number::ZIGZAG,
        (uint8_t)Mode::Number::SYSTEMID,
        (uint8_t)Mode::Number::AUTOROTATE,
        (uint8_t)Mode::Number::AUTO_RTL,
        (uint8_t)Mode::Number::TURTLE
    };

    if (!block_GCS_mode_change((uint8_t)mode_num, mode_list, ARRAY_SIZE(mode_list))) {
        return true;
    }

    // 模式被禁用，尝试获取模式名称以给出更好的警告
    // Mode disabled, try and grab a mode name to give a better warning.
    Mode *new_flightmode = mode_from_mode_num(mode_num);
    if (new_flightmode != nullptr) {
        mode_change_failed(new_flightmode, "GCS entry disabled (FLTMODE_GCSBLOCK)");
    } else {
        notify_no_such_mode((uint8_t)mode_num);
    }

    return false;
}

// 设置模式 - 更改飞行模式并执行任何必要的初始化
// 可选的force参数用于强制飞行模式更改（仅在首次设置模式时使用）
// 如果模式成功设置则返回true
// ACRO, STABILIZE, ALTHOLD, LAND, DRIFT和SPORT总是可以成功设置，但其他飞行模式的返回状态应该被检查，调用者应适当处理失败情况
// set_mode - change flight mode and perform any necessary initialisation
// optional force parameter used to force the flight mode change (used only first time mode is set)
// returns true if mode was successfully set
// ACRO, STABILIZE, ALTHOLD, LAND, DRIFT and SPORT can always be set successfully but the return state of other flight modes should be checked and the caller should deal with failures appropriately
bool Copter::set_mode(Mode::Number mode, ModeReason reason)
{
    // 更新最后一个原因
    // update last reason
    const ModeReason last_reason = _last_reason;
    _last_reason = reason;

    // 如果我们已经处于所需的模式，则立即返回
    // return immediately if we are already in the desired mode
    if (mode == flightmode->mode_number()) {
        control_mode_reason = reason;
        // 在自动驾驶仪启动期间设置偏航速率时间常数
        // set yaw rate time constant during autopilot startup
        if (reason == ModeReason::INITIALISED && mode == Mode::Number::STABILIZE) {
            attitude_control->set_yaw_rate_tc(g2.command_model_pilot.get_rate_tc());
        }
        // 发出成功提示音
        // make happy noise
        if (copter.ap.initialised && (reason != last_reason)) {
            AP_Notify::events.user_mode_change = 1;
        }
        return true;
    }

    // 检查是否通过参数禁用了GCS模式更改
    // Check if GCS mode change is disabled via parameter
    if ((reason == ModeReason::GCS_COMMAND) && !gcs_mode_enabled(mode)) {
        return false;
    }

#if MODE_AUTO_ENABLED
    if (mode == Mode::Number::AUTO_RTL) {
        // AUTO RTL的特殊情况，不是真正的模式，只是伪装的AUTO
        // 尝试加入返回路径，如果失败则回退到执行着陆序列
        // Special case for AUTO RTL, not a true mode, just AUTO in disguise
        // Attempt to join return path, fallback to do-land-start
        return mode_auto.return_path_or_jump_to_landing_sequence_auto_RTL(reason);
    }
#endif

    Mode *new_flightmode = mode_from_mode_num(mode);
    if (new_flightmode == nullptr) {
        notify_no_such_mode((uint8_t)mode);
        return false;
    }

    bool ignore_checks = !motors->armed();   // 如果未解锁，允许切换到任何模式。我们依赖解锁检查来执行

#if FRAME_CONFIG == HELI_FRAME
    // 如果旋翼未完全启动，不允许直升机进入非手动油门模式
    // do not allow helis to enter a non-manual throttle mode if the
    // rotor runup is not complete
    if (!ignore_checks && !new_flightmode->has_manual_throttle() &&
        (motors->get_spool_state() == AP_Motors::SpoolState::SPOOLING_UP || motors->get_spool_state() == AP_Motors::SpoolState::SPOOLING_DOWN)) {
        #if MODE_AUTOROTATE_ENABLED
            // 如果正在退出自转模式，允许模式更改，尽管旋翼未达到全速。
            // 这将减少返回到非手动油门模式时的高度损失
            //if the mode being exited is the autorotation mode allow mode change despite rotor not being at
            //full speed.  This will reduce altitude loss on bail-outs back to non-manual throttle modes
            bool in_autorotation_check = (flightmode != &mode_autorotate || new_flightmode != &mode_autorotate);
        #else
            bool in_autorotation_check = false;
        #endif

        if (!in_autorotation_check) {
            mode_change_failed(new_flightmode, "runup not complete");
            return false;
        }
    }
#endif

#if FRAME_CONFIG != HELI_FRAME
    // 确保当用户从非手动油门模式切换到手动油门模式时，飞行器不会从地面跳起
    // （例如，用户在引导模式下解锁，将油门提高到1300（不足以触发自动起飞），然后切换到手动模式）：
    // ensure vehicle doesn't leap off the ground if a user switches
    // into a manual throttle mode from a non-manual-throttle mode
    // (e.g. user arms in guided, raises throttle to 1300 (not enough to
    // trigger auto takeoff), then switches into manual):
    bool user_throttle = new_flightmode->has_manual_throttle();
#if MODE_DRIFT_ENABLED
    if (new_flightmode == &mode_drift) {
        user_throttle = true;
    }
#endif
    if (!ignore_checks &&
        ap.land_complete &&
        user_throttle &&
        !copter.flightmode->has_manual_throttle() &&
        new_flightmode->get_pilot_desired_throttle() > copter.get_non_takeoff_throttle()) {
        mode_change_failed(new_flightmode, "throttle too high");
        return false;
    }
#endif

    if (!ignore_checks &&
        new_flightmode->requires_GPS() &&
        !copter.position_ok()) {
        mode_change_failed(new_flightmode, "requires position");
        return false;
    }

    // 检查高度是否有效，如果旧模式不需要但新模式需要
    // 我们只想在可能使情况变糟的情况下停止更改模式
    // check for valid altitude if old mode did not require it but new one does
    // we only want to stop changing modes if it could make things worse
    if (!ignore_checks &&
        !copter.ekf_alt_ok() &&
        flightmode->has_manual_throttle() &&
        !new_flightmode->has_manual_throttle()) {
        mode_change_failed(new_flightmode, "need alt estimate");
        return false;
    }

#if AP_FENCE_ENABLED
    // 检查是否允许在从围栏突破恢复时更改模式
    // may not be allowed to change mode if recovering from fence breach
    if (!ignore_checks &&
        fence.enabled() &&
        fence.option_enabled(AC_Fence::OPTIONS::DISABLE_MODE_CHANGE) &&
        fence.get_breaches() &&
        motors->armed() &&
        get_control_mode_reason() == ModeReason::FENCE_BREACHED &&
        !ap.land_complete) {
        mode_change_failed(new_flightmode, "in fence recovery");
        return false;
    }
#endif

    // 初始化新的飞行模式
    // Initialize the new flight mode
    if (!new_flightmode->init(ignore_checks)) {
        mode_change_failed(new_flightmode, "init failed");
        return false;
    }

    // 执行前一个飞行模式所需的任何清理工作
    // perform any cleanup required by previous flight mode
    exit_mode(flightmode, new_flightmode);

    // 存储前一个飞行模式(仅用于传统直升机的自转)
    // store previous flight mode (only used by tradeheli's autorotation)
    prev_control_mode = flightmode->mode_number();

    // 更新飞行模式
    // update flight mode
    flightmode = new_flightmode;
    control_mode_reason = reason;
#if HAL_LOGGING_ENABLED
    // 记录新的飞行模式和原因
    logger.Write_Mode((uint8_t)flightmode->mode_number(), reason);
#endif
    // 发送心跳消息
    gcs().send_message(MSG_HEARTBEAT);

#if HAL_ADSB_ENABLED
    // 设置ADSB是否处于自动模式
    adsb.set_is_auto_mode((mode == Mode::Number::AUTO) || (mode == Mode::Number::RTL) || (mode == Mode::Number::GUIDED));
#endif

#if AP_FENCE_ENABLED
    if (fence.get_action() != AC_FENCE_ACTION_REPORT_ONLY) {
        // 飞行员在围栏突破期间请求的飞行模式更改表明飞行员正在尝试手动恢复
        // 这种飞行模式更改可能是自动的(即围栏、电池、GPS或GCS故障保护)
        // 但在这些情况下暂时禁用围栏也应该是无害的
        fence.manual_recovery_start();
    }
#endif

#if AP_CAMERA_ENABLED
    // 设置相机是否处于自动模式
    camera.set_is_auto_mode(flightmode->mode_number() == Mode::Number::AUTO);
#endif

    // 设置速率整形时间常数
    // set rate shaping time constants
#if MODE_ACRO_ENABLED || MODE_SPORT_ENABLED
    attitude_control->set_roll_pitch_rate_tc(g2.command_model_acro_rp.get_rate_tc());
#endif
    attitude_control->set_yaw_rate_tc(g2.command_model_pilot.get_rate_tc());
#if MODE_ACRO_ENABLED || MODE_DRIFT_ENABLED
    if (mode== Mode::Number::ACRO || mode== Mode::Number::DRIFT) {
        attitude_control->set_yaw_rate_tc(g2.command_model_acro_y.get_rate_tc());
    }
#endif

    // 更新通知对象
    // update notify object
    notify_flight_mode();

    // 发出愉快的声音
    // make happy noise
    if (copter.ap.initialised) {
        AP_Notify::events.user_mode_change = 1;
    }

    // 返回成功
    // return success
    return true;
}

// 设置飞行模式
bool Copter::set_mode(const uint8_t new_mode, const ModeReason reason)
{
    static_assert(sizeof(Mode::Number) == sizeof(new_mode), "The new mode can't be mapped to the vehicles mode number");
#ifdef DISALLOW_GCS_MODE_CHANGE_DURING_RC_FAILSAFE
    if (reason == ModeReason::GCS_COMMAND && copter.failsafe.radio) {
        // 在无线电故障保护期间不允许模式更改
        // don't allow mode changes while in radio failsafe
        return false;
    }
#endif
    return copter.set_mode(static_cast<Mode::Number>(new_mode), reason);
}

// 更新飞行模式 - 根据飞行模式调用适当的姿态控制器
// 以100Hz或更高的频率调用
// update_flight_mode - calls the appropriate attitude controllers based on flight mode
// called at 100hz or more
void Copter::update_flight_mode()
{
#if AP_RANGEFINDER_ENABLED
    surface_tracking.invalidate_for_logging();  // 使表面跟踪高度无效，飞行模式将在使用时设置为true
#endif
    attitude_control->landed_gain_reduction(copter.ap.land_complete); // 着陆时调整增益以减弱地面振荡

    flightmode->run();
}

// exit_mode - 在退出飞行模式时组织清理的高级调用
// exit_mode - high level call to organise cleanup as a flight mode is exited
void Copter::exit_mode(Mode *&old_flightmode,
                       Mode *&new_flightmode)
{
    // 从手动切换到自动飞行模式时平滑油门过渡
    // smooth throttle transition when switching from manual to automatic flight modes
    if (old_flightmode->has_manual_throttle() && !new_flightmode->has_manual_throttle() && motors->armed() && !ap.land_complete) {
        // 这假设所有手动飞行模式都使用get_pilot_desired_throttle来将飞行员输入转换为输出油门
        set_accel_throttle_I_from_pilot_throttle();
    }

    // 取消任何正在进行的起飞
    // cancel any takeoffs in progress
    old_flightmode->takeoff_stop();

    // 执行每种飞行模式所需的清理工作
    // perform cleanup required for each flight mode
    old_flightmode->exit();

#if FRAME_CONFIG == HELI_FRAME
    // 退出特技模式时，将飞行杆直通坚决重置为false。
    // firmly reset the flybar passthrough to false when exiting acro mode.
    if (old_flightmode == &mode_acro) {
        attitude_control->use_flybar_passthrough(false, false);
        motors->set_acro_tail(false);
    }

    // 如果我们正在从一个不使用手动油门的模式切换，
    // stab_col_ramp值应该预先加载到正确的值以避免抽搐
    // heli_stab_col_ramp实际上只应在稳定和特技模式之间切换时激活
    // if we are changing from a mode that did not use manual throttle,
    // stab col ramp value should be pre-loaded to the correct value to avoid a twitch
    // heli_stab_col_ramp should really only be active switching between Stabilize and Acro modes
    if (!old_flightmode->has_manual_throttle()){
        if (new_flightmode == &mode_stabilize){
            input_manager.set_stab_col_ramp(1.0);
        } else if (new_flightmode == &mode_acro){
            input_manager.set_stab_col_ramp(0.0);
        }
    }

    // 如果新模式不支持倒飞，请确保禁用倒飞
    // Make sure inverted flight is disabled if not supported in the new mode
    if (!new_flightmode->allows_inverted()) {
        attitude_control->set_inverted_flight(false);
    }
#endif //HELI_FRAME
}

// notify_flight_mode - 根据当前飞行模式设置通知对象。仅用于OreoLED通知设备
// notify_flight_mode - sets notify object based on current flight mode.  Only used for OreoLED notify device
void Copter::notify_flight_mode() {
    AP_Notify::flags.autopilot_mode = flightmode->is_autopilot();
    AP_Notify::flags.flight_mode = (uint8_t)flightmode->mode_number();
    notify.set_flight_mode_str(flightmode->name4());
}

// get_pilot_desired_angle - 将飞行员的横滚或俯仰输入转换为所需的倾斜角度
// 返回以厘度为单位的所需角度
// get_pilot_desired_angle - transform pilot's roll or pitch input into a desired lean angle
// returns desired angle in centi-degrees
void Mode::get_pilot_desired_lean_angles(float &roll_out_cd, float &pitch_out_cd, float angle_max_cd, float angle_limit_cd) const
{
    // 油门故障保护检查
    // throttle failsafe check
    if (copter.failsafe.radio || !rc().has_ever_seen_rc_input()) {
        roll_out_cd = 0.0;
        pitch_out_cd = 0.0;
        return;
    }

    // 将飞行员的归一化横滚或俯仰杆输入转换为横滚和俯仰欧拉角命令
    //transform pilot's normalised roll or pitch stick input into a roll and pitch euler angle command
    float roll_out_deg;
    float pitch_out_deg;
    rc_input_to_roll_pitch(channel_roll->get_control_in()*(1.0/ROLL_PITCH_YAW_INPUT_MAX), channel_pitch->get_control_in()*(1.0/ROLL_PITCH_YAW_INPUT_MAX), angle_max_cd * 0.01,  angle_limit_cd * 0.01, roll_out_deg, pitch_out_deg);

    // 转换为厘度
    // Convert to centi-degrees
    roll_out_cd = roll_out_deg * 100.0;
    pitch_out_cd = pitch_out_deg * 100.0;
}

// 将飞行员的横滚或俯仰输入转换为所需的速度
// transform pilot's roll or pitch input into a desired velocity
Vector2f Mode::get_pilot_desired_velocity(float vel_max) const
{
    Vector2f vel;

    // 油门故障保护检查
    // throttle failsafe check
    if (copter.failsafe.radio || !rc().has_ever_seen_rc_input()) {
        return vel;
    }
    // 获取横滚和俯仰输入
    // fetch roll and pitch inputs
    float roll_out = channel_roll->get_control_in();
    float pitch_out = channel_pitch->get_control_in();

    // 将横滚和俯仰输入转换为-1到+1范围
    // convert roll and pitch inputs to -1 to +1 range
    float scaler = 1.0 / (float)ROLL_PITCH_YAW_INPUT_MAX;
    roll_out *= scaler;
    pitch_out *= scaler;

    // 将横滚和俯仰输入转换为NE框架中的速度
    // convert roll and pitch inputs into velocity in NE frame
    vel = Vector2f(-pitch_out, roll_out);
    if (vel.is_zero()) {
        return vel;
    }
    copter.rotate_body_frame_to_NE(vel.x, vel.y);

    // 将方形输入范围转换为圆形输出
    // Transform square input range to circular output
    // vel_scaler是当前输入方向上到+-1.0方框边缘的向量
    // vel_scaler is the vector to the edge of the +- 1.0 square in the direction of the current input
    Vector2f vel_scaler = vel / MAX(fabsf(vel.x), fabsf(vel.y));
    // 我们通过到方框的距离与单位圆的比率来缩放输出，并乘以vel_max
    // We scale the output by the ratio of the distance to the square to the unit circle and multiply by vel_max
    vel *= vel_max / vel_scaler.length();
    return vel;
}

bool Mode::_TakeOff::triggered(const float target_climb_rate) const
{
    if (!copter.ap.land_complete) {
        // 如果我们已经在飞行，就不能起飞
        // can't take off if we're already flying
        return false;
    }
    if (target_climb_rate <= 0.0f) {
        // 除非我们想上升，否则不能起飞
        // can't takeoff unless we want to go up...
        return false;
    }

    if (copter.motors->get_spool_state() != AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        // 在转子速度运转完成之前，将飞机固定在地面上
        // hold aircraft on the ground until rotor speed runup has finished
        return false;
    }

    return true;
}

// 检查飞行器是否处于解除武装状态或已着陆状态
bool Mode::is_disarmed_or_landed() const
{
    // 如果满足以下任一条件，则认为飞行器处于解除武装状态或已着陆状态：
    // 1. 电机未武装
    // 2. 飞行器未自动武装
    // 3. 飞行器已完成着陆
    if (!motors->armed() || !copter.ap.auto_armed || copter.ap.land_complete) {
        return true;
    }
    return false;
}

// 将油门设置为零并使飞行器放松姿态
void Mode::zero_throttle_and_relax_ac(bool spool_up)
{
    if (spool_up) {
        // 如果需要保持电机转速，将期望的电机转速状态设置为无限制油门
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    } else {
        // 否则，将期望的电机转速状态设置为地面怠速
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    }
    // 将姿态控制输入设置为零（横滚、俯仰和偏航率都为0）
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);
    // 将油门输出设置为零，不使用稳定模式，应用油门滤波
    attitude_control->set_throttle_out(0.0f, false, copter.g.throttle_filt);
}

void Mode::zero_throttle_and_hold_attitude()
{
    // 运行姿态控制器
    // run attitude controller
    attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, 0.0f);
    attitude_control->set_throttle_out(0.0f, false, copter.g.throttle_filt);
}

// 处理飞行器在地面上等待起飞的情况
// handle situations where the vehicle is on the ground waiting for takeoff
// force_throttle_unlimited应该在我们希望保持电机转速（而不是降低到地面怠速）的情况下为true。
// 这对于在引导和自动模式下的传统直升机是必需的，我们总是希望在引导或自动模式下保持电机转速。
// 当传统直升机降低到地面怠速时，主旋翼会停止。
// 最终，它强制在地面上的自动和引导模式下遵守电机联锁。
// force_throttle_unlimited should be true in cases where we want to keep the motors spooled up
// (instead of spooling down to ground idle).  This is required for tradheli's in Guided and Auto
// where we always want the motor spooled up in Guided or Auto mode.  Tradheli's main rotor stops 
// when spooled down to ground idle.
// ultimately it forces the motor interlock to be obeyed in auto and guided modes when on the ground.
void Mode::make_safe_ground_handling(bool force_throttle_unlimited)
{
    if (force_throttle_unlimited) {
        // 保持转子转动 
        // keep rotors turning 
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    } else {
        // 降低到地面怠速
        // spool down to ground idle
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    }

    // 飞行器已着陆，无论转速状态如何，积分项都必须重置
    // aircraft is landed, integrator terms must be reset regardless of spool state
    attitude_control->reset_rate_controller_I_terms_smoothly();
 
    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
    case AP_Motors::SpoolState::GROUND_IDLE:
        // 在怠速状态下重置偏航目标和速率
        // reset yaw targets and rates during idle states
        attitude_control->reset_yaw_target_and_rate();
        break;
    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // 在过渡活动状态时继续正常操作
        // while transitioning though active states continue to operate normally
        break;
    }

    pos_control->relax_velocity_controller_xy();
    pos_control->update_xy_controller();
    pos_control->relax_z_controller(0.0f);   // 强制油门输出衰减到零
    pos_control->update_z_controller();
    // 我们可能需要将其移出
    // we may need to move this out
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);
}

/*
  获取用于着陆的地面高度估计
  get a height above ground estimate for landing
 */
int32_t Mode::get_alt_above_ground_cm(void)
{
    int32_t alt_above_ground_cm;
    if (copter.get_rangefinder_height_interpolated_cm(alt_above_ground_cm)) {
        return alt_above_ground_cm;
    }
    if (!pos_control->is_active_xy()) {
        return copter.current_loc.alt;
    }
    if (copter.current_loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, alt_above_ground_cm)) {
        return alt_above_ground_cm;
    }

    // 假设地球是平的：
    // Assume the Earth is flat:
    return copter.current_loc.alt;
}

void Mode::land_run_vertical_control(bool pause_descent)
{
    float cmb_rate = 0;
    bool ignore_descent_limit = false;
    if (!pause_descent) {

        // 在我们减速着陆之前不要忽略限制
        // do not ignore limits until we have slowed down for landing
        ignore_descent_limit = (MAX(g2.land_alt_low,100) > get_alt_above_ground_cm()) || copter.ap.land_complete_maybe;

        float max_land_descent_velocity;
        if (g.land_speed_high > 0) {
            max_land_descent_velocity = -g.land_speed_high;
        } else {
            max_land_descent_velocity = pos_control->get_max_speed_down_cms();
        }

        // 不要为着陆加速。
        // Don't speed up for landing.
        max_land_descent_velocity = MIN(max_land_descent_velocity, -abs(g.land_speed));

        // 计算垂直速度需求，使飞行器接近g2.land_alt_low。如果没有下面的约束，这将导致飞行器悬停在g2.land_alt_low。
        // Compute a vertical velocity demand such that the vehicle approaches g2.land_alt_low. Without the below constraint, this would cause the vehicle to hover at g2.land_alt_low.
        cmb_rate = sqrt_controller(MAX(g2.land_alt_low,100)-get_alt_above_ground_cm(), pos_control->get_pos_z_p().kP(), pos_control->get_max_accel_z_cmss(), G_Dt);

        // 约束要求的垂直速度，使其在配置的最大下降速度和配置的最小下降速度之间。
        // Constrain the demanded vertical velocity so that it is between the configured maximum descent speed and the configured minimum descent speed.
        cmb_rate = constrain_float(cmb_rate, max_land_descent_velocity, -abs(g.land_speed));

#if AC_PRECLAND_ENABLED
        const bool navigating = pos_control->is_active_xy();
        bool doing_precision_landing = !copter.ap.land_repo_active && copter.precland.target_acquired() && navigating;

        if (doing_precision_landing) {
            // 精确着陆处于活动状态
            // prec landing is active
            Vector2f target_pos;
            float target_error_cm = 0.0f;
            if (copter.precland.get_target_position_cm(target_pos)) {
                const Vector2f current_pos = inertial_nav.get_position_xy_cm();
                // 目标距离飞行器这么多厘米
                // target is this many cm away from the vehicle
                target_error_cm = (target_pos - current_pos).length();
            }
            // 检查我们是否应该下降
            // check if we should descend or not
            const float max_horiz_pos_error_cm = copter.precland.get_max_xy_error_before_descending_cm();
            Vector3f target_pos_meas;
            copter.precland.get_target_position_measurement_cm(target_pos_meas);
            if (target_error_cm > max_horiz_pos_error_cm && !is_zero(max_horiz_pos_error_cm)) {
                // 正在进行精确着陆，但离障碍物太远
                // 不下降
                // doing precland but too far away from the obstacle
                // do not descend
                cmb_rate = 0.0f;
            } else if (target_pos_meas.z > 35.0f && target_pos_meas.z < 200.0f && !copter.precland.do_fast_descend()) {
                // 非常接近地面并进行精确着陆，让我们减速以确保我们准确着陆
                // 计算所需的下降速度
                // very close to the ground and doing prec land, lets slow down to make sure we land on target
                // compute desired descent velocity
                const float precland_acceptable_error_cm = 15.0f;
                const float precland_min_descent_speed_cms = 10.0f;
                const float max_descent_speed_cms = abs(g.land_speed)*0.5f;
                const float land_slowdown = MAX(0.0f, target_error_cm*(max_descent_speed_cms/precland_acceptable_error_cm));
                cmb_rate = MIN(-precland_min_descent_speed_cms, -max_descent_speed_cms+land_slowdown);
            }
        }
#endif
    }

    // 更新高度目标并调用位置控制器
    // update altitude target and call position controller
    pos_control->land_at_climb_rate_cm(cmb_rate, ignore_descent_limit);
    pos_control->update_z_controller();
}

// 运行水平控制
void Mode::land_run_horizontal_control()
{
    Vector2f vel_correction;

    // 如果可能已经着陆,放松loiter目标
    if (copter.ap.land_complete_maybe) {
        pos_control->soften_for_landing_xy();
    }

    // 处理飞行员输入
    if (!copter.failsafe.radio) {
        // 如果油门行为设置为高油门取消着陆,且当前油门输入高于触发阈值
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            // 记录飞行员取消着陆事件
            LOGGER_WRITE_EVENT(LogEvent::LAND_CANCELLED_BY_PILOT);
            // 如果油门高,退出着陆模式
            if (!set_mode(Mode::Number::LOITER, ModeReason::THROTTLE_LAND_ESCAPE)) {
                set_mode(Mode::Number::ALT_HOLD, ModeReason::THROTTLE_LAND_ESCAPE);
            }
        }

        // 如果允许着陆重新定位
        if (g.land_repositioning) {
            // 对飞行员输入应用SIMPLE模式变换
            update_simple_mode();

            // 将飞行员输入转换为重新定位速度
            // 使用最大加速度的一半作为最大速度,确保飞机能在不到1秒内从全速重新定位停止
            const float max_pilot_vel = wp_nav->get_wp_acceleration() * 0.5;
            vel_correction = get_pilot_desired_velocity(max_pilot_vel);

            // 记录飞行员是否覆盖了横滚或俯仰
            if (!vel_correction.is_zero()) {
                if (!copter.ap.land_repo_active) {
                    // 记录着陆重新定位激活事件
                    LOGGER_WRITE_EVENT(LogEvent::LAND_REPO_ACTIVE);
                }
                copter.ap.land_repo_active = true;
#if AC_PRECLAND_ENABLED
            } else {
                // 当前没有覆盖,检查是否应该允许精确着陆
                if (copter.precland.allow_precland_after_reposition()) {
                    copter.ap.land_repo_active = false;
                }
#endif
            }
        }
    }

    // 如果精确着陆目标在视野内且飞行员没有尝试重新定位飞机,此变量将被更新
    copter.ap.prec_land_active = false;
#if AC_PRECLAND_ENABLED
    copter.ap.prec_land_active = !copter.ap.land_repo_active && copter.precland.target_acquired();
    // 运行精确着陆
    if (copter.ap.prec_land_active) {
        Vector2f target_pos, target_vel;
        if (!copter.precland.get_target_position_cm(target_pos)) {
            target_pos = inertial_nav.get_position_xy_cm();
        }
         // 获取目标速度
        copter.precland.get_target_velocity_cms(inertial_nav.get_velocity_xy_cms(), target_vel);

        Vector2f zero;
        Vector2p landing_pos = target_pos.topostype();
        // 如果着陆目标静止,目标速度将保持为零
        pos_control->input_pos_vel_accel_xy(landing_pos, target_vel, zero);
    }
#endif

    if (!copter.ap.prec_land_active) {
        Vector2f accel;
        pos_control->input_vel_accel_xy(vel_correction, accel);
    }

    // 运行位置控制器
    pos_control->update_xy_controller();
    Vector3f thrust_vector = pos_control->get_thrust_vector();

    // 如果用户请求了一个导航高度下限
    if (g2.wp_navalt_min > 0) {
        // 用户请求了一个导航姿态受限的高度下限。这用于防止着陆时的指令横滚过大,
        // 特别是对直升机而言,如果着陆后有任何位置估计漂移,会特别受影响。
        // 我们将姿态限制在这个限制以下7度,并在其上1m内线性插值
        const float attitude_limit_cd = linear_interpolate(700, copter.aparm.angle_max, get_alt_above_ground_cm(),
                                                     g2.wp_navalt_min*100U, (g2.wp_navalt_min+1)*100U);
        const float thrust_vector_max = sinf(radians(attitude_limit_cd * 0.01f)) * GRAVITY_MSS * 100.0f;
        const float thrust_vector_mag = thrust_vector.xy().length();
        if (thrust_vector_mag > thrust_vector_max) {
            float ratio = thrust_vector_max / thrust_vector_mag;
            thrust_vector.x *= ratio;
            thrust_vector.y *= ratio;

            // 告诉位置控制器我们正在应用外部限制
            pos_control->set_externally_limited_xy();
        }
    }

    // 调用姿态控制器
    attitude_control->input_thrust_vector_heading(thrust_vector, auto_yaw.get_heading());

}

// 运行正常或精确着陆(如果启用)
// pause_descent为true时表示飞机不应下降
void Mode::land_run_normal_or_precland(bool pause_descent)
{
#if AC_PRECLAND_ENABLED
    if (pause_descent || !copter.precland.enabled()) {
        // 我们不想立即开始下降或精确着陆被禁用
        // 在这两种情况下,只运行简单的着陆控制器
        land_run_horiz_and_vert_control(pause_descent);
    } else {
        // 精确着陆已启用,且我们没有暂停下降
        // 状态机负责整个精确着陆过程
        precland_run();
    }
#else
    land_run_horiz_and_vert_control(pause_descent);
#endif
}

#if AC_PRECLAND_ENABLED
// 移动到精确着陆状态机命令的位置以重试着陆
// 传入的位置预期为NED坐标系,单位为米
void Mode::precland_retry_position(const Vector3f &retry_pos)
{
    if (!copter.failsafe.radio) {
        // 如果油门行为设置为高油门取消着陆,且当前油门输入高于触发阈值
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            // 记录飞行员取消着陆事件
            LOGGER_WRITE_EVENT(LogEvent::LAND_CANCELLED_BY_PILOT);
            // 如果油门高,退出着陆模式
            if (!set_mode(Mode::Number::LOITER, ModeReason::THROTTLE_LAND_ESCAPE)) {
                set_mode(Mode::Number::ALT_HOLD, ModeReason::THROTTLE_LAND_ESCAPE);
            }
        }

        // 允许用户在重新定位期间控制。注意:从land_run_horizontal_control()复制
        // 待办:这段代码以略微不同的形式存在于几个不同的地方,应该修复
        if (g.land_repositioning) {
            float target_roll = 0.0f;
            float target_pitch = 0.0f;
            // 将飞行员输入转换为倾斜角度
            get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

            // 记录飞行员是否覆盖了横滚或俯仰
            if (!is_zero(target_roll) || !is_zero(target_pitch)) {
                if (!copter.ap.land_repo_active) {
                    // 记录着陆重新定位激活事件
                    LOGGER_WRITE_EVENT(LogEvent::LAND_REPO_ACTIVE);
                }
                // 此标志稍后将被精确着陆状态机检查,任何进一步的着陆重试都将被取消
                copter.ap.land_repo_active = true;
            }
        }
    }

    Vector3p retry_pos_NEU{retry_pos.x, retry_pos.y, retry_pos.z * -1.0f};
    // 位置控制器期望输入为NEU坐标系,单位为厘米
    retry_pos_NEU = retry_pos_NEU * 100.0f;
    pos_control->input_pos_xyz(retry_pos_NEU, 0.0f, 1000.0f);

    // 运行位置控制器
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // 调用姿态控制器
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());

}

// 运行精确着陆状态机。任何想要进行精确着陆的模式都应调用此函数。
// 这处理从精确着陆、精确着陆失败到重试和故障安全措施的所有内容
void Mode::precland_run()
{
    // 如果用户正在控制,我们将不运行状态机,而是简单地着陆(可能会也可能不会在目标上)
    if (!copter.ap.land_repo_active) {
        // 这将在稍后更新为重试位置(如果需要)
        Vector3f retry_pos;

        switch (copter.precland_statemachine.update(retry_pos)) {
        case AC_PrecLand_StateMachine::Status::RETRYING:
            // 我们想通过移动到另一个位置来重试着陆
            precland_retry_position(retry_pos);
            break;

        case AC_PrecLand_StateMachine::Status::FAILSAFE: {
            // 我们遇到了故障安全。故障安全只能意味着两件事,我们要么想永久停止直到用户接管,要么着陆
            switch (copter.precland_statemachine.get_failsafe_actions()) {
            case AC_PrecLand_StateMachine::FailSafeAction::DESCEND:
                // 正常下降,精确着陆目标肯定不在视野内
                land_run_horiz_and_vert_control();
                break;
            case AC_PrecLand_StateMachine::FailSafeAction::HOLD_POS:
                // 在此参数中发送"true"将停止下降
                land_run_horiz_and_vert_control(true);
                break;
            }
            break;
        }
        case AC_PrecLand_StateMachine::Status::ERROR:
            // 不应该发生,肯定是一个bug。报告然后下降
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            FALLTHROUGH;
        case AC_PrecLand_StateMachine::Status::DESCEND:
            // 运行着陆控制器。如果精确着陆目标在视野内,这将朝目标下降
            // 否则它将只是垂直下降
            land_run_horiz_and_vert_control();
            break;
        }
    } else {
        // 只是着陆,因为用户已经接管控制,运行任何重试或故障安全措施都没有意义
        land_run_horiz_and_vert_control();
    }
}
#endif

// 返回悬停油门值
float Mode::throttle_hover() const
{
    return motors->get_throttle_hover();
}

// 转换飞行员的手动油门输入,使悬停油门在中间位置
// 仅用于手动油门模式
// thr_mid应在0到1的范围内
// 返回0到1的油门输出
float Mode::get_pilot_desired_throttle() const
{
    const float thr_mid = throttle_hover();
    int16_t throttle_control = channel_throttle->get_control_in();

    int16_t mid_stick = copter.get_throttle_mid();
    // 防止不太可能的除以零情况
    if (mid_stick <= 0) {
        mid_stick = 500;
    }

    // 确保合理的油门值
    throttle_control = constrain_int16(throttle_control,0,1000);

    // 计算归一化的油门输入
    float throttle_in;
    if (throttle_control < mid_stick) {
        throttle_in = ((float)throttle_control)*0.5f/(float)mid_stick;
    } else {
        throttle_in = 0.5f + ((float)(throttle_control-mid_stick)) * 0.5f / (float)(1000-mid_stick);
    }

    const float expo = constrain_float(-(thr_mid-0.5f)/0.375f, -0.5f, 1.0f);
    // 使用给定的指数函数计算输出油门
    float throttle_out = throttle_in*(1.0f-expo) + expo*throttle_in*throttle_in*throttle_in;
    return throttle_out;
}

// 获取经避障调整后的爬升率
float Mode::get_avoidance_adjusted_climbrate(float target_rate)
{
#if AP_AVOIDANCE_ENABLED
    AP::ac_avoid()->adjust_velocity_z(pos_control->get_pos_z_p().kP(), pos_control->get_max_accel_z_cmss(), target_rate, G_Dt);
    return target_rate;
#else
    return target_rate;
#endif
}

// 向电机发送输出,可被子类覆盖
void Mode::output_to_motors()
{
    motors->output();
}

// 获取高度保持状态
Mode::AltHoldModeState Mode::get_alt_hold_state(float target_climb_rate_cms)
{
    // 高度保持状态机确定
    if (!motors->armed()) {
        // 飞机应移动到关闭状态
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);

        // 当飞机减速时,通过状态转换
        switch (motors->get_spool_state()) {

        case AP_Motors::SpoolState::SHUT_DOWN:
            return AltHoldModeState::MotorStopped;

        case AP_Motors::SpoolState::GROUND_IDLE:
            return AltHoldModeState::Landed_Ground_Idle;

        default:
            return AltHoldModeState::Landed_Pre_Takeoff;
        }

    } else if (takeoff.running() || takeoff.triggered(target_climb_rate_cms)) {
        // 飞机当前已着陆或正在起飞,要求正爬升率且处于THROTTLE_UNLIMITED状态
        // 飞机应进行起飞程序
        return AltHoldModeState::Takeoff;

    } else if (!copter.ap.auto_armed || copter.ap.land_complete) {
        // 飞机已解锁且已着陆
        if (target_climb_rate_cms < 0.0f && !copter.ap.using_interlock) {
            // 飞机应移动到地面怠速状态
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);

        } else {
            // 飞机应准备即将起飞
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        }

        if (motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
            // 飞机正在地面怠速等待
            return AltHoldModeState::Landed_Ground_Idle;

        } else {
            // 飞机随时可以离地
            return AltHoldModeState::Landed_Pre_Takeoff;
        }

    } else {
        // 飞机处于飞行状态
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        return AltHoldModeState::Flying;
    }
}

// 将飞行员的偏航输入转换为期望的偏航速率
// 返回期望的偏航速率(单位:厘度/秒)
float Mode::get_pilot_desired_yaw_rate() const
{
    // 油门故障安全检查
    if (copter.failsafe.radio || !rc().has_ever_seen_rc_input()) {
        return 0.0f;
    }

    // 获取偏航输入
    const float yaw_in = channel_yaw->norm_input_dz();

    // 将飞行员输入转换为期望的偏航速率
    return g2.command_model_pilot.get_rate() * 100.0 * input_expo(yaw_in, g2.command_model_pilot.get_expo());
}

// 传递函数以减少代码重复;
// 这些是移入Mode基类的候选函数。
float Mode::get_pilot_desired_climb_rate(float throttle_control)
{
    return copter.get_pilot_desired_climb_rate(throttle_control);
}

float Mode::get_non_takeoff_throttle()
{
    return copter.get_non_takeoff_throttle();
}

void Mode::update_simple_mode(void) {
    copter.update_simple_mode();
}

bool Mode::set_mode(Mode::Number mode, ModeReason reason)
{
    return copter.set_mode(mode, reason);
}

void Mode::set_land_complete(bool b)
{
    return copter.set_land_complete(b);
}

GCS_Copter &Mode::gcs()
{
    return copter.gcs();
}

uint16_t Mode::get_pilot_speed_dn()
{
    return copter.get_pilot_speed_dn();
}

// 返回停止点作为一个位置,高度基于原点高度框架
Location Mode::get_stopping_point() const
{
    Vector3p stopping_point_NEU;
    copter.pos_control->get_stopping_point_xy_cm(stopping_point_NEU.xy());
    copter.pos_control->get_stopping_point_z_cm(stopping_point_NEU.z);
    return Location { stopping_point_NEU.tofloat(), Location::AltFrame::ABOVE_ORIGIN };
}
