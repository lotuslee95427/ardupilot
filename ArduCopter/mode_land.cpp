#include "Copter.h"

// land_init - 初始化着陆控制器
bool ModeLand::init(bool ignore_checks)
{
    // 检查是否有GPS并决定执行哪种着陆方式
    control_position = copter.position_ok();

    // 设置水平速度和加速度限制
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // 初始化水平位置控制器
    if (control_position && !pos_control->is_active_xy()) {
        pos_control->init_xy_controller();
    }

    // 设置垂直速度和加速度限制
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // 初始化垂直位置控制器
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // 记录着陆开始时间
    land_start_time = millis();
    // 设置着陆暂停标志为false
    land_pause = false;

    // 重置标志，指示飞行员在着陆过程中是否应用了横滚或俯仰输入
    copter.ap.land_repo_active = false;

    // 这将在精确着陆稍后激活时设置为true
    copter.ap.prec_land_active = false;

    // 初始化偏航控制，设置为保持模式
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

#if AP_LANDINGGEAR_ENABLED
    // 可选择部署起落架
    copter.landinggear.deploy_for_landing();
#endif

#if AC_PRECLAND_ENABLED
    // 初始化精确着陆状态机
    copter.precland_statemachine.init();
#endif

    return true;
}

// land_run - 运行着陆控制器
// 应该以100Hz或更高的频率调用
void ModeLand::run()
{
    if (control_position) {
        gps_run();  // 使用GPS进行着陆控制
    } else {
        nogps_run();  // 不使用GPS进行着陆控制
    }
}

// land_gps_run - 运行使用GPS的着陆控制器
//      使用悬停控制器控制水平位置
//      应该以100Hz或更高的频率调用
void ModeLand::gps_run()
{
    // 当着陆检测器表示我们已着陆时解除武装
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }

    // 着陆状态机确定
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();  // 执行安全地面处理
    } else {
        // 将电机设置为全范围
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // 在开始下降前暂停
        if (land_pause && millis()-land_start_time >= LAND_WITH_DELAY_MS) {
            land_pause = false;
        }

        // 运行正常着陆或精确着陆（如果启用）
        land_run_normal_or_precland(land_pause);
    }
}

// land_nogps_run - 运行不使用GPS的着陆控制器
//      飞行员控制横滚和俯仰角度
//      应该以100Hz或更高的频率调用
void ModeLand::nogps_run()
{
    float target_roll = 0.0f, target_pitch = 0.0f;

    // 处理飞行员输入
    if (!copter.failsafe.radio) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            LOGGER_WRITE_EVENT(LogEvent::LAND_CANCELLED_BY_PILOT);
            // 如果油门高，退出着陆模式
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::THROTTLE_LAND_ESCAPE);
        }

        if (g.land_repositioning) {
            // 对飞行员输入应用SIMPLE模式变换
            update_simple_mode();

            // 获取飞行员期望的倾斜角度
            get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max_cd());
        }
    }

    // 当着陆检测器表示我们已着陆时解除武装
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }

    // 着陆状态机确定
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();  // 执行安全地面处理
    } else {
        // 将电机设置为全范围
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // 在开始下降前暂停
        if (land_pause && millis()-land_start_time >= LAND_WITH_DELAY_MS) {
            land_pause = false;
        }

        land_run_vertical_control(land_pause);  // 运行垂直控制
    }

    // 调用姿态控制器
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, auto_yaw.get_heading().yaw_rate_cds);
}

// do_not_use_GPS - 强制着陆模式不使用GPS，而是依赖飞行员输入来控制横滚和俯仰
//  在GPS故障保护期间调用，以确保如果我们已经处于LAND模式，我们不会使用GPS
//  如果我们还没有处于LAND模式，则没有效果
void ModeLand::do_not_use_GPS()
{
    control_position = false;
}

// set_mode_land_with_pause - 将模式设置为LAND并触发4秒延迟，然后开始下降
//  这总是从故障保护中调用，所以我们触发对飞行员的通知
void Copter::set_mode_land_with_pause(ModeReason reason)
{
    set_mode(Mode::Number::LAND, reason);
    mode_land.set_land_pause(true);

    // 提醒飞行员模式变化
    AP_Notify::events.failsafe_mode_change = 1;
}

// landing_with_GPS - 如果飞行器正在使用GPS着陆，则返回true
bool Copter::landing_with_GPS()
{
    return (flightmode->mode_number() == Mode::Number::LAND &&
            mode_land.controlling_position());
}
