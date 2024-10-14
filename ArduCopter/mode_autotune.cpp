#include "Copter.h"

/*
  autotune mode is a wrapper around the AC_AutoTune library
  自动调谐模式是AC_AutoTune库的一个封装
 */

#if AUTOTUNE_ENABLED

bool AutoTune::init()
{
    // only allow AutoTune from some flight modes, for example Stabilize, AltHold,  PosHold or Loiter modes
    // 只允许从某些飞行模式启动自动调谐,例如稳定模式、定高模式、定点模式或悬停模式
    if (!copter.flightmode->allows_autotune()) {
        return false;
    }

    // ensure throttle is above zero
    // 确保油门大于零
    if (copter.ap.throttle_zero) {
        return false;
    }

    // ensure we are flying
    // 确保我们正在飞行
    if (!copter.motors->armed() || !copter.ap.auto_armed || copter.ap.land_complete) {
        return false;
    }

    // use position hold while tuning if we were in QLOITER
    // 如果我们处于QLOITER模式,在调谐时使用位置保持
    bool position_hold = (copter.flightmode->mode_number() == Mode::Number::LOITER || copter.flightmode->mode_number() == Mode::Number::POSHOLD);

    return init_internals(position_hold,
                          copter.attitude_control,
                          copter.pos_control,
                          copter.ahrs_view,
                          &copter.inertial_nav);
}

void AutoTune::run()
{
    // apply SIMPLE mode transform to pilot inputs
    // 对飞行员输入应用SIMPLE模式变换
    copter.update_simple_mode();

    // disarm when the landing detector says we've landed and spool state is ground idle
    // 当着陆检测器表明我们已着陆且电机处于地面怠速状态时解除武装
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }

    // if not armed set throttle to zero and exit immediately
    // 如果未武装,将油门设置为零并立即退出
    if (copter.ap.land_complete) {
        copter.flightmode->make_safe_ground_handling();
        return;
    }

    // run autotune mode
    // 运行自动调谐模式
    AC_AutoTune::run();

}


/*
  get stick input climb rate
  获取摇杆输入的爬升率
 */
float AutoTune::get_pilot_desired_climb_rate_cms(void) const
{
    float target_climb_rate = copter.get_pilot_desired_climb_rate(copter.channel_throttle->get_control_in());

    // get avoidance adjusted climb rate
    // 获取经过避障调整的爬升率
    target_climb_rate = copter.mode_autotune.get_avoidance_adjusted_climbrate(target_climb_rate);

    return target_climb_rate;
}

/*
  get stick roll, pitch and yaw rate
  获取摇杆的横滚、俯仰和偏航速率
 */
void AutoTune::get_pilot_desired_rp_yrate_cd(float &des_roll_cd, float &des_pitch_cd, float &yaw_rate_cds)
{
    copter.mode_autotune.get_pilot_desired_lean_angles(des_roll_cd, des_pitch_cd, copter.aparm.angle_max,
                                                       copter.attitude_control->get_althold_lean_angle_max_cd());
    yaw_rate_cds = copter.mode_autotune.get_pilot_desired_yaw_rate();
}

/*
  setup z controller velocity and accel limits
  设置z轴控制器的速度和加速度限制
 */
void AutoTune::init_z_limits()
{
    // set vertical speed and acceleration limits
    // 设置垂直速度和加速度限制
    copter.pos_control->set_max_speed_accel_z(-copter.get_pilot_speed_dn(), copter.g.pilot_speed_up, copter.g.pilot_accel_z);
    copter.pos_control->set_correction_speed_accel_z(-copter.get_pilot_speed_dn(), copter.g.pilot_speed_up, copter.g.pilot_accel_z);
}

#if HAL_LOGGING_ENABLED
void AutoTune::log_pids()
{
    copter.logger.Write_PID(LOG_PIDR_MSG, copter.attitude_control->get_rate_roll_pid().get_pid_info());
    copter.logger.Write_PID(LOG_PIDP_MSG, copter.attitude_control->get_rate_pitch_pid().get_pid_info());
    copter.logger.Write_PID(LOG_PIDY_MSG, copter.attitude_control->get_rate_yaw_pid().get_pid_info());
}
#endif

/*
  check if we have a good position estimate
  检查我们是否有良好的位置估计
 */
bool AutoTune::position_ok()
{
    return copter.position_ok();
}

/*
  initialise autotune mode
  初始化自动调谐模式
*/
bool ModeAutoTune::init(bool ignore_checks)
{
    return autotune.init();
}

void ModeAutoTune::run()
{
    autotune.run();
}

void ModeAutoTune::exit()
{
    autotune.stop();
}

#endif  // AUTOTUNE_ENABLED
