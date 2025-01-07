#include "Plane.h"

// 构造函数
Mode::Mode() :
    ahrs(plane.ahrs)
#if HAL_QUADPLANE_ENABLED
    , quadplane(plane.quadplane),
    pos_control(plane.quadplane.pos_control),
    attitude_control(plane.quadplane.attitude_control),
    loiter_nav(plane.quadplane.loiter_nav),
    poscontrol(plane.quadplane.poscontrol)
#endif
{
}

// 退出模式时的处理
void Mode::exit()
{
    // 调用子类的退出函数
    _exit();
    // 如果不是自动调谐模式，停止自动调谐
    if (plane.control_mode != &plane.mode_autotune){
        plane.autotune_restore();
    }
}

// 进入模式时的处理
bool Mode::enter()
{
#if AP_SCRIPTING_ENABLED
    // 重置导航脚本使能标志
    plane.nav_scripting.enabled = false;
#endif

    // 取消倒飞状态
    plane.auto_state.inverted_flight = false;
    
    // 取消等待方向舵中立
    plane.takeoff_state.waiting_for_rudder_neutral = false;

    // 开始任务时不进行交叉跟踪
    plane.auto_state.next_wp_crosstrack = false;

    // 重置着陆检查
    plane.auto_state.checked_for_autoland = false;

    // 清零锁定航向
    plane.steer_state.locked_course_err = 0;
    plane.steer_state.locked_course = false;

    // 重置坠机检测
    plane.crash_state.is_crashed = false;
    plane.crash_state.impact_detected = false;

    // 重置外部姿态引导
    plane.guided_state.last_forced_rpy_ms.zero();
    plane.guided_state.last_forced_throttle_ms = 0;

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // 重置引导模式相关参数
    plane.guided_state.target_heading = -4;
    plane.guided_state.target_heading_type = GUIDED_HEADING_NONE;
    plane.guided_state.target_airspeed_cm = -1;
    plane.guided_state.target_alt_time_ms = 0;
    plane.guided_state.target_location.set_alt_cm(-1, Location::AltFrame::ABSOLUTE); 
#endif

#if AP_CAMERA_ENABLED
    // 设置相机自动模式
    plane.camera.set_is_auto_mode(this == &plane.mode_auto);
#endif

    // 重置初始俯仰角和最高空速
    plane.auto_state.highest_airspeed = 0;
    plane.auto_state.initial_pitch_cd = ahrs.pitch_sensor;

    // 禁用尾拖起飞模式
    plane.auto_state.fbwa_tdrag_takeoff_mode = false;

    // 将上一个航点设为当前位置
    plane.prev_WP_loc = plane.current_loc;

    // 重置盘旋开始时间
    plane.loiter.start_time_ms = 0;

    // 记录模式改变时间
    plane.last_mode_change_ms = AP_HAL::millis();

    // 设置VTOL自动状态
    plane.auto_state.vtol_mode = is_vtol_mode();
    plane.auto_state.vtol_loiter = false;

    // 初始化AUTO和GUIDED模式中用于DO_CHANGE_SPEED命令的速度变量
    plane.new_airspeed_cm = -1;
    
    // 如果在长故障安全触发前发生模式改变（来自GCS），清除延迟的长故障安全
    plane.long_failsafe_pending = false;

#if HAL_QUADPLANE_ENABLED
    // 进入四旋翼模式
    quadplane.mode_enter();
#endif

#if AP_TERRAIN_AVAILABLE
    // 重置地形跟随标志
    plane.target_altitude.terrain_following_pending = false;
#endif

    // 禁用自动模式下高度等待命令期间的伺服怠速
    plane.auto_state.idle_mode = false;

    // 调用子类的进入函数
    bool enter_result = _enter();

    if (enter_result) {
        // 以下操作必须在_enter()之后执行，因为它们使用结果来设置更多标志

        // 在自动油门模式下开始时抑制油门
        plane.throttle_suppressed = does_auto_throttle();
#if HAL_ADSB_ENABLED
        // 设置ADSB自动模式
        plane.adsb.set_is_auto_mode(does_auto_navigation());
#endif

        // 设置导航控制器数据为过时状态
        plane.nav_controller->set_data_is_stale();

        // 重置转向积分器
        plane.steerController.reset_I();

        // 更新遥控器故障保护
        plane.control_failsafe();

#if AP_FENCE_ENABLED
        // 启动手动恢复围栏突破
        plane.fence.manual_recovery_start();
#endif
        // 在某些情况下重置任务
        if (plane.mission.get_in_landing_sequence_flag() &&
            !plane.is_flying() && !plane.arming.is_armed_and_safety_off() &&
            !plane.control_mode->does_auto_navigation()) {
           GCS_SEND_TEXT(MAV_SEVERITY_INFO, "In landing sequence: mission reset");
           plane.mission.reset();
        }

        // 更新飞行阶段
        plane.update_flight_stage();
    }

    return enter_result;
}

// 检查是否为VTOL手动油门模式
bool Mode::is_vtol_man_throttle() const
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.tailsitter.is_in_fw_flight() &&
        plane.quadplane.assisted_flight) {
        // 尾坐式飞机完全过渡到Q辅助前向飞行
        return !does_auto_throttle();
    }
#endif
    return false;
}

// 更新目标高度
void Mode::update_target_altitude()
{
    Location target_location;

    if (plane.landing.is_flaring()) {
        // 着陆调平阶段，使用TECS_LAND_SINK作为目标下降率
        plane.set_target_altitude_location(plane.next_WP_loc);
    } else if (plane.landing.is_on_approach()) {
        // 着陆进近阶段，设置着陆滑行坡度
        plane.landing.setup_landing_glide_slope(plane.prev_WP_loc, plane.next_WP_loc, plane.current_loc, plane.target_altitude.offset_cm);
#if AP_RANGEFINDER_ENABLED
        // 根据测距仪数据调整着陆坡度
        plane.landing.adjust_landing_slope_for_rangefinder_bump(plane.rangefinder_state, plane.prev_WP_loc, plane.next_WP_loc, plane.current_loc, plane.auto_state.wp_distance, plane.target_altitude.offset_cm);
#endif
    } else if (plane.landing.get_target_altitude_location(target_location)) {
        // 设置目标高度位置
        plane.set_target_altitude_location(target_location);
#if HAL_SOARING_ENABLED
    } else if (plane.g2.soaring_controller.is_active() && plane.g2.soaring_controller.get_throttle_suppressed()) {
        // 滑翔模式下重置目标高度为当前高度
        plane.set_target_altitude_location(plane.current_loc);
        plane.reset_offset_altitude();
#endif
    } else if (plane.reached_loiter_target()) {
        // 到达盘旋目标后锁定最终目标高度
        plane.set_target_altitude_location(plane.next_WP_loc);
    } else if (plane.target_altitude.offset_cm != 0 && 
               !plane.current_loc.past_interval_finish_line(plane.prev_WP_loc, plane.next_WP_loc)) {
        // 控制爬升/下降率
        plane.set_target_altitude_proportion(plane.next_WP_loc, 1.0f-plane.auto_state.wp_proportion);

        // 保持在起始和结束位置的高度范围内
        plane.constrain_target_altitude_location(plane.next_WP_loc, plane.prev_WP_loc);
    } else {
        // 设置下一个航点的目标高度
        plane.set_target_altitude_location(plane.next_WP_loc);
    }
}

// 检查是否可以在此模式下解锁
bool Mode::pre_arm_checks(size_t buflen, char *buffer) const
{
    if (!_pre_arm_checks(buflen, buffer)) {
        if (strlen(buffer) == 0) {
            // 如果没有提供消息，添加一个通用消息
            hal.util->snprintf(buffer, buflen, "mode not armable");
        }
        return false;
    }

    return true;
}

// 自动和引导模式不调用此函数以绕过四旋翼模式检查
bool Mode::_pre_arm_checks(size_t buflen, char *buffer) const
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.enabled() && !is_vtol_mode() &&
            plane.quadplane.option_is_set(QuadPlane::OPTION::ONLY_ARM_IN_QMODE_OR_AUTO)) {
        hal.util->snprintf(buffer, buflen, "not Q mode");
        return false;
    }
#endif
    return true;
}

// 运行模式
void Mode::run()
{
    // 执行摇杆混合
    if ((plane.g.stick_mixing == StickMixing::FBW) || (plane.g.stick_mixing == StickMixing::DIRECT_REMOVED)) {
        plane.stabilize_stick_mixing_fbw();
    }
    // 稳定滚转、俯仰和偏航
    plane.stabilize_roll();
    plane.stabilize_pitch();
    plane.stabilize_yaw();
}

// 重置速率和转向控制器
void Mode::reset_controllers()
{
    // 重置积分器
    plane.rollController.reset_I();
    plane.pitchController.reset_I();
    plane.yawController.reset_I();

    // 重置转向控制
    plane.steer_state.locked_course = false;
    plane.steer_state.locked_course_err = 0;
}

// 检查是否正在起飞
bool Mode::is_taking_off() const
{
    return (plane.flight_stage == AP_FixedWing::FlightStage::TAKEOFF);
}

// 输出到方向舵和转向伺服通道
void Mode::output_rudder_and_steering(float val)
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, val);
    SRV_Channels::set_output_scaled(SRV_Channel::k_steering, val);
}

// 输出飞行员油门，用于没有自动油门控制的稳定模式
void Mode::output_pilot_throttle()
{
    if (plane.g.throttle_passthru_stabilize) {
        // THR_PASS_STAB设置，直接映射
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.get_throttle_input(true));
        return;
    }

    // 获取油门，但如果设置了飞行选项，则调整中心以输出TRIM_THROTTLE
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.get_adjusted_throttle_input(true));
}

// 检查是否应用油门最小/最大限制
bool Mode::use_throttle_limits() const
{
#if AP_SCRIPTING_ENABLED
    if (plane.nav_scripting_active()) {
        return false;
    }
#endif

    if (this == &plane.mode_stabilize ||
        this == &plane.mode_training ||
        this == &plane.mode_acro ||
        this == &plane.mode_fbwa ||
        this == &plane.mode_autotune) {
        // 手动油门模式
        return !plane.g.throttle_passthru_stabilize;
    }

    if (is_guided_mode() && plane.guided_throttle_passthru) {
        // GUIDED模式下手动传递油门
        return false;
    }

#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode()) {
        return quadplane.allow_forward_throttle_in_vtol_mode();
    }
#endif

    return true;
}

// 检查是否应用电池补偿到油门
bool Mode::use_battery_compensation() const
{
#if AP_SCRIPTING_ENABLED
    if (plane.nav_scripting_active()) {
        return false;
    }
#endif

    if (this == &plane.mode_stabilize ||
        this == &plane.mode_training ||
        this == &plane.mode_acro ||
        this == &plane.mode_fbwa ||
        this == &plane.mode_autotune) {
        // 手动油门模式
        return false;
    }

    if (is_guided_mode() && plane.guided_throttle_passthru) {
        // GUIDED模式下手动传递油门
        return false;
    }

#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode()) {
        return false;
    }
#endif

    return true;
}
