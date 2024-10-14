#include "Copter.h"

#if MODE_BRAKE_ENABLED

/*
 * Init and run calls for brake flight mode
 * 刹车飞行模式的初始化和运行调用
 */

// brake_init - initialise brake controller
// brake_init - 初始化刹车控制器
bool ModeBrake::init(bool ignore_checks)
{
    // initialise pos controller speed and acceleration
    // 初始化位置控制器的速度和加速度
    pos_control->set_max_speed_accel_xy(inertial_nav.get_velocity_neu_cms().length(), BRAKE_MODE_DECEL_RATE);
    pos_control->set_correction_speed_accel_xy(inertial_nav.get_velocity_neu_cms().length(), BRAKE_MODE_DECEL_RATE);

    // initialise position controller
    // 初始化位置控制器
    pos_control->init_xy_controller();

    // set vertical speed and acceleration limits
    // 设置垂直速度和加速度限制
    pos_control->set_max_speed_accel_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);
    pos_control->set_correction_speed_accel_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);

    // initialise the vertical position controller
    // 初始化垂直位置控制器
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // 初始化超时时间为0
    _timeout_ms = 0;

    return true;
}

// brake_run - runs the brake controller
// should be called at 100hz or more
// brake_run - 运行刹车控制器
// 应该以100Hz或更高的频率调用
void ModeBrake::run()
{
    // if not armed set throttle to zero and exit immediately
    // 如果未解锁，将油门设置为零并立即退出
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        pos_control->relax_z_controller(0.0f);
        return;
    }

    // set motors to full range
    // 将电机设置为全范围
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // relax stop target if we might be landed
    // 如果可能已经着陆，放松停止目标
    if (copter.ap.land_complete_maybe) {
        pos_control->soften_for_landing_xy();
    }

    // use position controller to stop
    // 使用位置控制器来停止
    Vector2f vel;
    Vector2f accel;
    pos_control->input_vel_accel_xy(vel, accel);
    pos_control->update_xy_controller();

    // call attitude controller
    // 调用姿态控制器
    attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), 0.0f);

    // 设置垂直位置目标为0爬升率
    pos_control->set_pos_target_z_from_climb_rate_cm(0.0f);
    pos_control->update_z_controller();

    // MAV_CMD_SOLO_BTN_PAUSE_CLICK (Solo only) is used to set the timeout.
    // 使用MAV_CMD_SOLO_BTN_PAUSE_CLICK（仅Solo）来设置超时
    if (_timeout_ms != 0 && millis()-_timeout_start >= _timeout_ms) {
        if (!copter.set_mode(Mode::Number::LOITER, ModeReason::BRAKE_TIMEOUT)) {
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::BRAKE_TIMEOUT);
        }
    }
}

/**
 * Set a timeout for the brake mode
 * 为刹车模式设置超时
 * 
 * @param timeout_ms [in] timeout in milliseconds
 * @param timeout_ms [in] 超时时间，以毫秒为单位
 * 
 * @note MAV_CMD_SOLO_BTN_PAUSE_CLICK (Solo only) is used to set the timeout.
 * If the timeout is reached, the mode will switch to loiter or alt hold depending on the current mode.
 * If timeout_ms is 0, the timeout is disabled.
 * 
 * @note 使用MAV_CMD_SOLO_BTN_PAUSE_CLICK（仅Solo）来设置超时。
 * 如果达到超时时间，模式将切换到悬停或定高模式，具体取决于当前模式。
 * 如果timeout_ms为0，则禁用超时。
 * 
*/
void ModeBrake::timeout_to_loiter_ms(uint32_t timeout_ms)
{
    _timeout_start = millis();
    _timeout_ms = timeout_ms;
}

#endif
