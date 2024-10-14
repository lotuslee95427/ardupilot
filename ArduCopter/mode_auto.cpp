#include "Copter.h"

#if MODE_AUTO_ENABLED

/*
 * Init and run calls for auto flight mode
 * 自动飞行模式的初始化和运行调用
 *
 * This file contains the implementation for Land, Waypoint navigation and Takeoff from Auto mode
 * 本文件包含自动模式下的着陆、航点导航和起飞的实现
 * Command execution code (i.e. command_logic.pde) should:
 * 命令执行代码(即command_logic.pde)应该:
 *      a) switch to Auto flight mode with set_mode() function.  This will cause auto_init to be called
 *      a) 使用set_mode()函数切换到自动飞行模式。这将导致调用auto_init
 *      b) call one of the three auto initialisation functions: auto_wp_start(), auto_takeoff_start(), auto_land_start()
 *      b) 调用三个自动初始化函数之一:auto_wp_start(), auto_takeoff_start(), auto_land_start()
 *      c) call one of the verify functions auto_wp_verify(), auto_takeoff_verify, auto_land_verify repeated to check if the command has completed
 *      c) 重复调用验证函数auto_wp_verify(), auto_takeoff_verify, auto_land_verify之一,以检查命令是否完成
 * The main loop (i.e. fast loop) will call update_flight_modes() which will in turn call auto_run() which, based upon the auto_mode variable will call
 * 主循环(即快速循环)将调用update_flight_modes(),它将依次调用auto_run(),根据auto_mode变量,auto_run将调用
 *      correct auto_wp_run, auto_takeoff_run or auto_land_run to actually implement the feature
 *      正确的auto_wp_run, auto_takeoff_run或auto_land_run来实际实现该功能
 */

/*
 *  While in the auto flight mode, navigation or do/now commands can be run.
 *  在自动飞行模式下,可以运行导航或do/now命令。
 *  Code in this file implements the navigation commands
 *  本文件中的代码实现了导航命令
 */

// auto_init - initialise auto controller
// auto_init - 初始化自动控制器
bool ModeAuto::init(bool ignore_checks)
{
    auto_RTL = false;
    if (mission.num_commands() > 1 || ignore_checks) {
        // reject switching to auto mode if landed with motors armed but first command is not a takeoff (reduce chance of flips)
        // 如果已着陆且电机已解锁,但第一个命令不是起飞,则拒绝切换到自动模式(减少翻转的机会)
        if (motors->armed() && copter.ap.land_complete && !mission.starts_with_takeoff_cmd()) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Auto: Missing Takeoff Cmd");
            return false;
        }

        _mode = SubMode::LOITER;

        // stop ROI from carrying over from previous runs of the mission
        // 停止ROI从任务的前几次运行中延续下来
        // To-Do: reset the yaw as part of auto_wp_start when the previous command was not a wp command to remove the need for this special ROI check
        // 待办:当前一个命令不是wp命令时,作为auto_wp_start的一部分重置偏航,以消除对这个特殊ROI检查的需求
        if (auto_yaw.mode() == AutoYaw::Mode::ROI) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);
        }

        // initialise waypoint and spline controller
        // 初始化航点和样条控制器
        wp_nav->wp_and_spline_init();

        // initialise desired speed overrides
        // 初始化期望速度覆盖
        desired_speed_override = {0, 0, 0};

        // set flag to start mission
        // 设置标志以开始任务
        waiting_to_start = true;

        // initialise mission change check (ignore results)
        // 初始化任务变更检查(忽略结果)
        IGNORE_RETURN(mis_change_detector.check_for_mission_change());

        // clear guided limits
        // 清除引导限制
        copter.mode_guided.limit_clear();

        // reset flag indicating if pilot has applied roll or pitch inputs during landing
        // 重置指示飞行员在着陆过程中是否应用了横滚或俯仰输入的标志
        copter.ap.land_repo_active = false;

#if AC_PRECLAND_ENABLED
        // initialise precland state machine
        // 初始化精确着陆状态机
        copter.precland_statemachine.init();
#endif

        return true;
    } else {
        return false;
    }
}

// stop mission when we leave auto mode
// 当我们离开自动模式时停止任务
void ModeAuto::exit()
{
    if (copter.mode_auto.mission.state() == AP_Mission::MISSION_RUNNING) {
        copter.mode_auto.mission.stop();
    }
#if HAL_MOUNT_ENABLED
    copter.camera_mount.set_mode_to_default();
#endif  // HAL_MOUNT_ENABLED

    auto_RTL = false;
}

// auto_run - runs the auto controller
// auto_run - 运行自动控制器
//      should be called at 100hz or more
//      应该以100Hz或更高的频率调用
void ModeAuto::run()
{
    // start or update mission
    // 开始或更新任务
    if (waiting_to_start) {
        // don't start the mission until we have an origin
        // 在我们有一个原点之前不要开始任务
        Location loc;
        if (copter.ahrs.get_origin(loc)) {
            // start/resume the mission (based on MIS_RESTART parameter)
            // 开始/恢复任务(基于MIS_RESTART参数)
            mission.start_or_resume();
            waiting_to_start = false;

            // initialise mission change check (ignore results)
            // 初始化任务变更检查(忽略结果)
            IGNORE_RETURN(mis_change_detector.check_for_mission_change());
        }
    } else {
        // check for mission changes
        // 检查任务变更
        if (mis_change_detector.check_for_mission_change()) {
            // if mission is running restart the current command if it is a waypoint or spline command
            // 如果任务正在运行,如果当前命令是航点或样条命令,则重新启动当前命令
            if ((mission.state() == AP_Mission::MISSION_RUNNING) && (_mode == SubMode::WP)) {
                if (mission.restart_current_nav_cmd()) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Auto mission changed, restarted command");
                } else {
                    // failed to restart mission for some reason
                    // 由于某种原因未能重新启动任务
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Auto mission changed but failed to restart command");
                }
            }
        }

        mission.update();
    }

    // call the correct auto controller
    // 调用正确的自动控制器
    switch (_mode) {

    case SubMode::TAKEOFF:
        takeoff_run();
        break;

    case SubMode::WP:
    case SubMode::CIRCLE_MOVE_TO_EDGE:
        wp_run();
        break;

    case SubMode::LAND:
        land_run();
        break;

    case SubMode::RTL:
        rtl_run();
        break;

    case SubMode::CIRCLE:
        circle_run();
        break;

    case SubMode::NAVGUIDED:
    case SubMode::NAV_SCRIPT_TIME:
#if AC_NAV_GUIDED || AP_SCRIPTING_ENABLED
        nav_guided_run();
#endif
        break;

    case SubMode::LOITER:
        loiter_run();
        break;

    case SubMode::LOITER_TO_ALT:
        loiter_to_alt_run();
        break;

#if AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED && AC_PAYLOAD_PLACE_ENABLED
    case SubMode::NAV_PAYLOAD_PLACE:
        payload_place.run();
        break;
#endif

    case SubMode::NAV_ATTITUDE_TIME:
        nav_attitude_time_run();
        break;
    }

    // only pretend to be in auto RTL so long as mission still thinks its in a landing sequence or the mission has completed
    // 只有在任务仍然认为它处于着陆序列中或任务已完成时,才假装处于自动RTL中
    const bool auto_rtl_active = mission.get_in_landing_sequence_flag() || mission.get_in_return_path_flag() || mission.state() == AP_Mission::mission_state::MISSION_COMPLETE;
    if (auto_RTL && !auto_rtl_active) {
        auto_RTL = false;
        // log exit from Auto RTL
        // 记录退出自动RTL
#if HAL_LOGGING_ENABLED
        copter.logger.Write_Mode((uint8_t)copter.flightmode->mode_number(), ModeReason::AUTO_RTL_EXIT);
#endif
    }
}

// return true if a position estimate is required
// 如果需要位置估计,则返回true
bool ModeAuto::requires_GPS() const
{
    // position estimate is required in all sub modes except attitude control
    // 除了姿态控制外,所有子模式都需要位置估计
    return _mode != SubMode::NAV_ATTITUDE_TIME;
}

// set submode.  This may re-trigger the vehicle's EKF failsafe if the new submode requires a position estimate
// 设置子模式。如果新的子模式需要位置估计,这可能会重新触发车辆的EKF故障保护
void ModeAuto::set_submode(SubMode new_submode)
{
    // return immediately if the submode has not been changed
    // 如果子模式没有改变,立即返回
    if (new_submode == _mode) {
        return;
    }

    // backup old mode
    // 备份旧模式
    SubMode old_submode = _mode;

    // set mode
    // 设置模式
    _mode = new_submode;

    // if changing out of the nav-attitude-time submode, recheck the EKF failsafe
    // this may trigger a flight mode change if the EKF failsafe is active
    // 如果从nav-attitude-time子模式切换出来,重新检查EKF故障保护
    // 如果EKF故障保护处于活动状态,这可能会触发飞行模式改变
    if (old_submode == SubMode::NAV_ATTITUDE_TIME) {
        copter.failsafe_ekf_recheck();
    }
}

bool ModeAuto::option_is_enabled(Option option) const
{
    return ((copter.g2.auto_options & (uint32_t)option) != 0);
}

bool ModeAuto::allows_arming(AP_Arming::Method method) const
{
    if (auto_RTL) {
        return false;
    }
    return option_is_enabled(Option::AllowArming);
}

#if WEATHERVANE_ENABLED
bool ModeAuto::allows_weathervaning() const
{
    return option_is_enabled(Option::AllowWeatherVaning);
}
#endif

// Go straight to landing sequence via DO_LAND_START, if succeeds pretend to be Auto RTL mode
// 通过DO_LAND_START直接进入着陆序列,如果成功则假装处于自动RTL模式
bool ModeAuto::jump_to_landing_sequence_auto_RTL(ModeReason reason)
{
    if (!mission.jump_to_landing_sequence(get_stopping_point())) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(Number::AUTO_RTL));
        // make sad noise
        // 发出悲伤的声音
        if (copter.ap.initialised) {
            AP_Notify::events.user_mode_change_failed = 1;
        }
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode change to AUTO RTL failed: No landing sequence found");
        return false;
    }

    return enter_auto_rtl(reason);
}

// Join mission after DO_RETURN_PATH_START waypoint, if succeeds pretend to be Auto RTL mode
// 在DO_RETURN_PATH_START航点之后加入任务,如果成功则假装处于自动RTL模式
bool ModeAuto::return_path_start_auto_RTL(ModeReason reason)
{
    if (!mission.jump_to_closest_mission_leg(get_stopping_point())) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(Number::AUTO_RTL));
        // make sad noise
        // 发出悲伤的声音
        if (copter.ap.initialised) {
            AP_Notify::events.user_mode_change_failed = 1;
        }
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode change to AUTO RTL failed: No return path found");
        return false;
    }

    return enter_auto_rtl(reason);
}

// Try join return path else do land start
// 尝试加入返回路径,否则开始着陆
bool ModeAuto::return_path_or_jump_to_landing_sequence_auto_RTL(ModeReason reason)
{
    const Location stopping_point = get_stopping_point();
    if (!mission.jump_to_closest_mission_leg(stopping_point) && !mission.jump_to_landing_sequence(stopping_point)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(Number::AUTO_RTL));
        // make sad noise
        // 发出悲伤的声音
        if (copter.ap.initialised) {
            AP_Notify::events.user_mode_change_failed = 1;
        }
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode change to AUTO RTL failed: No return path or landing sequence found");
        return false;
    }

    return enter_auto_rtl(reason);
}

// Enter auto rtl pseudo mode
// 进入自动RTL伪模式
bool ModeAuto::enter_auto_rtl(ModeReason reason) 
{
    mission.set_force_resume(true);

    // if not already in auto switch to auto
    // 如果还不在自动模式,则切换到自动模式
    if ((copter.flightmode == this) || set_mode(Mode::Number::AUTO, reason)) {
        auto_RTL = true;
#if HAL_LOGGING_ENABLED
        // log entry into AUTO RTL
        // 记录进入自动RTL
        copter.logger.Write_Mode((uint8_t)copter.flightmode->mode_number(), reason);
#endif

        // make happy noise
        // 发出愉快的声音
        if (copter.ap.initialised) {
            AP_Notify::events.user_mode_change = 1;
        }
        return true;
    }

    // 模式切换失败，恢复强制恢复标志
    // mode change failed, revert force resume flag
    mission.set_force_resume(false);

    // 记录飞行模式错误
    // log flight mode error
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(Number::AUTO_RTL));
    
    // 发出悲伤的声音
    // make sad noise
    if (copter.ap.initialised) {
        AP_Notify::events.user_mode_change_failed = 1;
    }
    return false;
}

// lua脚本使用此函数来检索活动命令的内容
// lua scripts use this to retrieve the contents of the active command
bool ModeAuto::nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2, int16_t &arg3, int16_t &arg4)
{
#if AP_SCRIPTING_ENABLED
    if (_mode == SubMode::NAV_SCRIPT_TIME) {
        // 如果当前子模式是NAV_SCRIPT_TIME，则返回相关参数
        id = nav_scripting.id;
        cmd = nav_scripting.command;
        arg1 = nav_scripting.arg1;
        arg2 = nav_scripting.arg2;
        arg3 = nav_scripting.arg3;
        arg4 = nav_scripting.arg4;
        return true;
    }
#endif
    return false;
}

// lua脚本使用此函数来指示它们何时完成命令
// lua scripts use this to indicate when they have complete the command
void ModeAuto::nav_script_time_done(uint16_t id)
{
#if AP_SCRIPTING_ENABLED
    if ((_mode == SubMode::NAV_SCRIPT_TIME) && (id == nav_scripting.id)) {
        // 如果当前子模式是NAV_SCRIPT_TIME且ID匹配，则标记为完成
        nav_scripting.done = true;
    }
#endif
}

// auto_loiter_start - 在自动模式下初始化盘旋
// auto_loiter_start - initialises loitering in auto mode
//  返回成功/失败，因为这可能被exit_mission调用
//  returns success/failure because this can be called by exit_mission
bool ModeAuto::loiter_start()
{
    // 如果GPS信号不好，则返回失败
    // return failure if GPS is bad
    if (!copter.position_ok()) {
        return false;
    }
    _mode = SubMode::LOITER;

    // 计算停止点
    // calculate stopping point
    Vector3f stopping_point;
    wp_nav->get_wp_stopping_point(stopping_point);

    // 初始化航点控制器目标为停止点
    // initialise waypoint controller target to stopping point
    wp_nav->set_wp_destination(stopping_point);

    // 保持当前航向
    // hold yaw at current heading
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

    return true;
}

// auto_rtl_start - 在AUTO飞行模式下初始化RTL
// auto_rtl_start - initialises RTL in AUTO flight mode
void ModeAuto::rtl_start()
{
    // 调用常规rtl飞行模式初始化，并要求忽略检查
    // call regular rtl flight mode initialisation and ask it to ignore checks
    if (copter.mode_rtl.init(true)) {
        set_submode(SubMode::RTL);
    } else {
        // 这种情况不应该发生，因为如果参数为true，RTL初始化永远不会失败
        // this should never happen because RTL never fails init if argument is true
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }
}

// 初始化航点控制器以实现起飞
// initialise waypoint controller to implement take-off
void ModeAuto::takeoff_start(const Location& dest_loc)
{
    if (!copter.current_loc.initialised()) {
        // 这种情况不应该发生，因为在AHRS/EKF原点设置之前不会执行任务命令
        // 此时current_loc也应该已经设置
        // this should never happen because mission commands are not executed until
        // the AHRS/EKF origin is set by which time current_loc should also have been set
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return;
    }

    // 计算当前和目标高度
    // calculate current and target altitudes
    // 默认情况下，current_alt_cm和alt_target_cm是相对于EKF原点的高度
    // by default current_alt_cm and alt_target_cm are alt-above-EKF-origin
    int32_t alt_target_cm;
    bool alt_target_terrain = false;
    float current_alt_cm = inertial_nav.get_position_z_up_cm();
    float terrain_offset;   // 地形相对于ekf原点的高度（厘米）
    if ((dest_loc.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN) && wp_nav->get_terrain_offset(terrain_offset)) {
        // 减去地形偏移，将飞行器的高度从相对于ekf原点转换为相对于地形
        // subtract terrain offset to convert vehicle's alt-above-ekf-origin to alt-above-terrain
        current_alt_cm -= terrain_offset;

        // 将alt_target_cm指定为相对于地形的高度
        // specify alt_target_cm as alt-above-terrain
        alt_target_cm = dest_loc.alt;
        alt_target_terrain = true;
    } else {
        // 设置水平目标
        // set horizontal target
        Location dest(dest_loc);
        dest.lat = copter.current_loc.lat;
        dest.lng = copter.current_loc.lng;

        // 获取相对于EKF原点的目标高度
        // get altitude target above EKF origin
        if (!dest.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, alt_target_cm)) {
            // 这种失败只可能发生在起飞高度被指定为相对于地形的高度，而我们没有地形数据的情况下
            // this failure could only happen if take-off alt was specified as an alt-above terrain and we have no terrain data
            LOGGER_WRITE_ERROR(LogErrorSubsystem::TERRAIN, LogErrorCode::MISSING_TERRAIN_DATA);
            // 回退到相对于当前高度的高度
            // fall back to altitude above current altitude
            alt_target_cm = current_alt_cm + dest.alt;
        }
    }

    // 目标合理性检查
    // sanity check target
    int32_t alt_target_min_cm = current_alt_cm + (copter.ap.land_complete ? 100 : 0);
    alt_target_cm = MAX(alt_target_cm, alt_target_min_cm);

    // 初始化偏航
    // initialise yaw
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

    // 起飞时清除i项
    // clear i term when we're taking off
    pos_control->init_z_controller();

    // 为WP_NAVALT_MIN初始化高度并设置完成高度
    // initialise alt for WP_NAVALT_MIN and set completion alt
    auto_takeoff.start(alt_target_cm, alt_target_terrain);

    // 设置子模式
    // set submode
    set_submode(SubMode::TAKEOFF);
}

// auto_wp_start - 初始化航点控制器以实现飞向特定目的地
// auto_wp_start - initialises waypoint controller to implement flying to a particular destination
bool ModeAuto::wp_start(const Location& dest_loc)
{
    // 初始化wpnav并在从起飞转换时设置原点
    // init wpnav and set origin if transitioning from takeoff
    if (!wp_nav->is_active()) {
        Vector3f stopping_point;
        if (_mode == SubMode::TAKEOFF) {
            Vector3p takeoff_complete_pos;
            if (auto_takeoff.get_completion_pos(takeoff_complete_pos)) {
                stopping_point = takeoff_complete_pos.tofloat();
            }
        }
        float des_speed_xy_cm = is_positive(desired_speed_override.xy) ? (desired_speed_override.xy * 100) : 0;
        wp_nav->wp_and_spline_init(des_speed_xy_cm, stopping_point);

        // 如果必要，覆盖上升和下降速度
        // override speeds up and down if necessary
        if (is_positive(desired_speed_override.up)) {
            wp_nav->set_speed_up(desired_speed_override.up * 100.0);
        }
        if (is_positive(desired_speed_override.down)) {
            wp_nav->set_speed_down(desired_speed_override.down * 100.0);
        }
    }

    if (!wp_nav->set_wp_destination_loc(dest_loc)) {
        return false;
    }

    // 初始化偏航
    // initialise yaw
    // 待办：仅在前一个导航命令不是WP时重置偏航。这将允许移除对ROI的特殊检查
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw.mode() != AutoYaw::Mode::ROI) {
        auto_yaw.set_mode_to_default(false);
    }

    // 设置子模式
    // set submode
    set_submode(SubMode::WP);

    return true;
}

// auto_land_start - 初始化控制器以实现着陆
// auto_land_start - initialises controller to implement a landing
void ModeAuto::land_start()
{
    // 设置水平速度和加速度限制
    // set horizontal speed and acceleration limits
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // 初始化垂直位置控制器
    // initialise the vertical position controller
    if (!pos_control->is_active_xy()) {
        pos_control->init_xy_controller();
    }

    // 设置垂直速度和加速度限制
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // 初始化垂直位置控制器
    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // 初始化偏航
    // initialise yaw
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

#if AP_LANDINGGEAR_ENABLED
    // 可选择部署着陆齿轮
    // optionally deploy landing gear
    copter.landinggear.deploy_for_landing();
#endif

    // 重置指示飞行员是否在着陆过程中应用了横滚或俯仰输入的标志
    // reset flag indicating if pilot has applied roll or pitch inputs during landing
    copter.ap.land_repo_active = false;

    // 这将在精确着陆稍后激活时设置为true
    // this will be set true if prec land is later active
    copter.ap.prec_land_active = false;

    // 设置子模式
    // set submode
    set_submode(SubMode::LAND);
}

// auto_circle_movetoedge_start - 初始化航点控制器以移动到以指定位置为中心的圆的边缘
// auto_circle_movetoedge_start - initialise waypoint controller to move to edge of a circle with it's center at the specified location
//  我们假设调用者已执行所有必需的GPS_ok检查
//  we assume the caller has performed all required GPS_ok checks
void ModeAuto::circle_movetoedge_start(const Location &circle_center, float radius_m, bool ccw_turn)
{
    // 设置圆心
    // set circle center
    copter.circle_nav->set_center(circle_center);

    // 设置圆半径
    // set circle radius
    if (!is_zero(radius_m)) {
        copter.circle_nav->set_radius_cm(radius_m * 100.0f);
    }

    // 通过使用速率设置圆的方向
    // set circle direction by using rate
    float current_rate = copter.circle_nav->get_rate();
    current_rate = ccw_turn ? -fabsf(current_rate) : fabsf(current_rate);
    copter.circle_nav->set_rate(current_rate);

    // 检查我们与圆边缘的距离
    // check our distance from edge of circle
    Vector3f circle_edge_neu;
    copter.circle_nav->get_closest_point_on_circle(circle_edge_neu);
    float dist_to_edge = (inertial_nav.get_position_neu_cm() - circle_edge_neu).length();

    // 如果超过3m，则飞向边缘
    // if more than 3m then fly to edge
    if (dist_to_edge > 300.0f) {
        // 将circle_edge_neu转换为Location
        // convert circle_edge_neu to Location
        Location circle_edge(circle_edge_neu, Location::AltFrame::ABOVE_ORIGIN);

        // 将高度转换为与命令相同
        // convert altitude to same as command
        circle_edge.set_alt_cm(circle_center.alt, circle_center.get_alt_frame());

        // 初始化wpnav以移动到圆的边缘
        // initialise wpnav to move to edge of circle
        if (!wp_nav->set_wp_destination_loc(circle_edge)) {
            // 设置目的地失败只可能是因为缺少地形数据
            // failure to set destination can only be because of missing terrain data
            copter.failsafe_terrain_on_event();
        }

        // 如果我们在圆外，指向边缘，否则保持偏航
        // if we are outside the circle, point at the edge, otherwise hold yaw
        const float dist_to_center = get_horizontal_distance_cm(inertial_nav.get_position_xy_cm().topostype(), copter.circle_nav->get_center().xy());
        // 初始化偏航
        // initialise yaw
        // 待办：仅在前一个导航命令不是WP时重置偏航。这将允许移除对ROI的特殊检查
        // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
        if (auto_yaw.mode() != AutoYaw::Mode::ROI) {
            if (dist_to_center > copter.circle_nav->get_radius() && dist_to_center > 500) {
                auto_yaw.set_mode_to_default(false);
            } else {
                // 飞行器在圆内，所以保持偏航以避免在移动到圆边缘时旋转
                // vehicle is within circle so hold yaw to avoid spinning as we move to edge of circle
                auto_yaw.set_mode(AutoYaw::Mode::HOLD);
            }
        }

        // 将子模式设置为移动到圆的边缘
        // set the submode to move to the edge of the circle
        set_submode(SubMode::CIRCLE_MOVE_TO_EDGE);
    } else {
        circle_start();
    }
}

// auto_circle_start - 初始化控制器以在AUTO飞行模式下飞行圆形轨迹
//   假设circle_nav对象已经用圆心和半径进行了初始化
// auto_circle_start - initialises controller to fly a circle in AUTO flight mode
//   assumes that circle_nav object has already been initialised with circle center and radius
void ModeAuto::circle_start()
{
    // 初始化圆形控制器
    // initialise circle controller
    copter.circle_nav->init(copter.circle_nav->get_center(), copter.circle_nav->center_is_terrain_alt(), copter.circle_nav->get_rate());

    // 如果自动偏航模式不是ROI，则设置为圆形模式
    // 如果当前是ROI模式，则保持不变
    if (auto_yaw.mode() != AutoYaw::Mode::ROI) {
        auto_yaw.set_mode(AutoYaw::Mode::CIRCLE);
    }

    // 将子模式设置为圆形
    // set submode to circle
    set_submode(SubMode::CIRCLE);
}

#if AC_NAV_GUIDED
// auto_nav_guided_start - 在AUTO模式下将控制权交给外部导航控制器
// auto_nav_guided_start - hand over control to external navigation controller in AUTO mode
void ModeAuto::nav_guided_start()
{
    // 调用常规引导飞行模式初始化
    // call regular guided flight mode initialisation
    if (!copter.mode_guided.init(true)) {
        // 这种情况不应该发生，因为引导模式从不会初始化失败
        // this should never happen because guided mode never fails to init
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return;
    }

    // 初始化引导模式的开始时间和位置，作为限制检查的参考
    // initialise guided start time and position as reference for limit checking
    copter.mode_guided.limit_init_time_and_pos();

    // 设置子模式
    // set submode
    set_submode(SubMode::NAVGUIDED);
}
#endif //AC_NAV_GUIDED

// 判断是否正在着陆
bool ModeAuto::is_landing() const
{
    switch(_mode) {
    case SubMode::LAND:
        return true;
    case SubMode::RTL:
        return copter.mode_rtl.is_landing();
    default:
        return false;
    }
    return false;
}

// 判断是否正在起飞
bool ModeAuto::is_taking_off() const
{
    return ((_mode == SubMode::TAKEOFF) && !auto_takeoff.complete);
}

#if AC_PAYLOAD_PLACE_ENABLED
// auto_payload_place_start - 初始化控制器以实现放置
// auto_payload_place_start - initialises controller to implement a placing
void PayloadPlace::start_descent()
{
    auto *pos_control = copter.pos_control;
    auto *wp_nav = copter.wp_nav;

    // 设置水平速度和加速度限制
    // set horizontal speed and acceleration limits
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // 初始化垂直位置控制器
    // initialise the vertical position controller
    if (!pos_control->is_active_xy()) {
        pos_control->init_xy_controller();
    }

    // 设置垂直速度和加速度限制
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // 初始化垂直位置控制器
    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // 初始化偏航
    // initialise yaw
    copter.flightmode->auto_yaw.set_mode(Mode::AutoYaw::Mode::HOLD);

    state = PayloadPlace::State::Descent_Start;
}
#endif

// 返回是否应使用飞行员的偏航输入来调整飞行器的航向
// returns true if pilot's yaw input should be used to adjust vehicle's heading
bool ModeAuto::use_pilot_yaw(void) const
{
    const bool allow_yaw_option = !option_is_enabled(Option::IgnorePilotYaw);
    const bool rtl_allow_yaw = (_mode == SubMode::RTL) && copter.mode_rtl.use_pilot_yaw();
    const bool landing = _mode == SubMode::LAND;
    return allow_yaw_option || rtl_allow_yaw || landing;
}

// 设置水平速度
bool ModeAuto::set_speed_xy(float speed_xy_cms)
{
    copter.wp_nav->set_speed_xy(speed_xy_cms);
    desired_speed_override.xy = speed_xy_cms * 0.01;
    return true;
}

// 设置上升速度
bool ModeAuto::set_speed_up(float speed_up_cms)
{
    copter.wp_nav->set_speed_up(speed_up_cms);
    desired_speed_override.up = speed_up_cms * 0.01;
    return true;
}

// 设置下降速度
bool ModeAuto::set_speed_down(float speed_down_cms)
{
    copter.wp_nav->set_speed_down(speed_down_cms);
    desired_speed_override.down = speed_down_cms * 0.01;
    return true;
}

// start_command - 当ap_mission库希望开始一个新命令时，将调用此函数
// start_command - this function will be called when the ap_mission lib wishes to start a new command
bool ModeAuto::start_command(const AP_Mission::Mission_Command& cmd)
{
#if HAL_LOGGING_ENABLED
    // 待办：在新命令开始/结束时记录日志
    // To-Do: logging when new commands start/end
    if (copter.should_log(MASK_LOG_CMD)) {
        copter.logger.Write_Mission_Cmd(mission, cmd);
    }
#endif

    switch(cmd.id) {

    ///
    /// 导航命令
    /// navigation commands
    ///
    case MAV_CMD_NAV_VTOL_TAKEOFF:
    case MAV_CMD_NAV_TAKEOFF:                   // 22
        do_takeoff(cmd);
        break;

    case MAV_CMD_NAV_WAYPOINT:                  // 16  导航到航点
        do_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_LAND:              // 21 降落到航点
        do_land(cmd);
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:              // 17 无限期盘旋
        do_loiter_unlimited(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TURNS:              //18 盘旋N圈
        do_circle(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TIME:              // 19 盘旋一段时间
        do_loiter_time(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TO_ALT:
        do_loiter_to_alt(cmd);
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:             //20 返回起飞点
        do_RTL();
        break;

    case MAV_CMD_NAV_SPLINE_WAYPOINT:           // 82  使用样条曲线导航到航点
        do_spline_wp(cmd);
        break;

#if AC_NAV_GUIDED
    case MAV_CMD_NAV_GUIDED_ENABLE:             // 92  接受来自外部导航计算机的导航命令
        do_nav_guided_enable(cmd);
        break;
#endif

    case MAV_CMD_NAV_DELAY:                    // 93 延迟下一个导航命令
        do_nav_delay(cmd);
        break;

#if AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED && AC_PAYLOAD_PLACE_ENABLED
    case MAV_CMD_NAV_PAYLOAD_PLACE:              // 94 在航点放置负载
        do_payload_place(cmd);
        break;
#endif

#if AP_SCRIPTING_ENABLED
    case MAV_CMD_NAV_SCRIPT_TIME:
        do_nav_script_time(cmd);
        break;
#endif

    case MAV_CMD_NAV_ATTITUDE_TIME:
        do_nav_attitude_time(cmd);
        break;

    //
    // 条件命令
    // conditional commands
    //
    case MAV_CMD_CONDITION_DELAY:             // 112 延迟
        do_wait_delay(cmd);
        break;

    case MAV_CMD_CONDITION_DISTANCE:             // 114 距离
        do_within_distance(cmd);
        break;

    case MAV_CMD_CONDITION_YAW:             // 115 偏航
        do_yaw(cmd);
        break;

    ///
    /// 执行命令
    /// do commands
    ///
    case MAV_CMD_DO_CHANGE_SPEED:             // 178 改变速度
        do_change_speed(cmd);
        break;

    case MAV_CMD_DO_SET_HOME:             // 179 设置家
        do_set_home(cmd);
        break;

    case MAV_CMD_DO_SET_ROI:                // 201 设置兴趣区域
        // 将飞行器和相机指向兴趣区域（ROI）
        // point the copter and camera at a region of interest (ROI)
        do_roi(cmd);
        break;

#if HAL_MOUNT_ENABLED
    case MAV_CMD_DO_MOUNT_CONTROL:          // 205 云台控制
        // 将相机指向指定角度
        // point the camera to a specified angle
        do_mount_control(cmd);
        break;
#endif

#if AC_NAV_GUIDED
    case MAV_CMD_DO_GUIDED_LIMITS:                      // 220  接受引导模式限制
        do_guided_limits(cmd);
        break;
#endif

#if AP_WINCH_ENABLED
    case MAV_CMD_DO_WINCH:                             // 控制绞车的任务命令
        do_winch(cmd);
        break;
#endif

    case MAV_CMD_DO_RETURN_PATH_START:
    case MAV_CMD_DO_LAND_START:
        break;

    default:
        // 无法使用该命令，允许飞行器尝试下一个命令
        // unable to use the command, allow the vehicle to try the next command
        return false;
    }

    // 始终返回成功
    // always return success
    return true;
}

// exit_mission - 任务完成时调用的函数
// exit_mission - function that is called once the mission completes
void ModeAuto::exit_mission()
{
    // 播放音调
    // play a tone
    AP_Notify::events.mission_complete = 1;
    // 如果我们不在地面上，切换到盘旋或着陆模式
    // if we are not on the ground switch to loiter or land
    if (!copter.ap.land_complete) {
        // 尝试进入盘旋模式，如果失败则着陆
        // try to enter loiter but if that fails land
        if (!loiter_start()) {
            set_mode(Mode::Number::LAND, ModeReason::MISSION_END);
        }
    } else {
        // 如果我们已经着陆，可以安全地解除武装
        // if we've landed it's safe to disarm
        copter.arming.disarm(AP_Arming::Method::MISSIONEXIT);
    }
}

// do_guided - 开始引导模式
// do_guided - start guided mode
bool ModeAuto::do_guided(const AP_Mission::Mission_Command& cmd)
{
    // 只有在引导模式下才处理引导航点
    // only process guided waypoint if we are in guided mode
    if (copter.flightmode->mode_number() != Mode::Number::GUIDED && !(copter.flightmode->mode_number() == Mode::Number::AUTO && _mode == SubMode::NAVGUIDED)) {
        return false;
    }

    // 根据不同的命令进行处理
    // switch to handle different commands
    switch (cmd.id) {

        case MAV_CMD_NAV_WAYPOINT:
        {
            // 设置wp_nav的目的地
            // set wp_nav's destination
            Location dest(cmd.content.location);
            return copter.mode_guided.set_destination(dest);
        }

        case MAV_CMD_CONDITION_YAW:
            do_yaw(cmd);
            return true;

        default:
            // 拒绝未识别的命令
            // reject unrecognised command
            return false;
    }

    return true;
}

// 获取到目标点的距离
uint32_t ModeAuto::wp_distance() const
{
    switch (_mode) {
    case SubMode::CIRCLE:
        return copter.circle_nav->get_distance_to_target();
    case SubMode::WP:
    case SubMode::CIRCLE_MOVE_TO_EDGE:
    default:
        return wp_nav->get_wp_distance_to_destination();
    }
}

// 获取到目标点的方位角
// get bearing to target waypoint
int32_t ModeAuto::wp_bearing() const
{
    switch (_mode) {
    case SubMode::CIRCLE:
        // 圆形模式下,获取到圆心的方位角
        // in circle mode, get bearing to circle center
        return copter.circle_nav->get_bearing_to_target();
    case SubMode::WP:
    case SubMode::CIRCLE_MOVE_TO_EDGE:
    default:
        // 其他模式下,获取到目标航点的方位角
        // in other modes, get bearing to destination waypoint
        return wp_nav->get_wp_bearing_to_destination();
    }
}

// 获取当前目标位置
// get current target location
bool ModeAuto::get_wp(Location& destination) const
{
    switch (_mode) {
    case SubMode::NAVGUIDED:
        // 引导导航模式下,获取引导模式的目标位置
        // in guided nav mode, get guided mode's target
        return copter.mode_guided.get_wp(destination);
    case SubMode::WP:
        // 航点模式下,获取避障后的目标位置
        // in waypoint mode, get obstacle avoidance adjusted destination
        return wp_nav->get_oa_wp_destination(destination);
    case SubMode::RTL:
        // 返航模式下,获取返航模式的目标位置
        // in RTL mode, get RTL mode's target
        return copter.mode_rtl.get_wp(destination);
    default:
        return false;
    }
}

/*******************************************************************************
验证命令处理程序

每种任务元素都有一个"验证"操作。验证操作在任务元素完成时返回true,
我们应该继续执行下一个任务元素。
如果我们不识别该命令,则返回true,以便我们继续执行下一个命令。

Verify command Handlers

Each type of mission element has a "verify" operation. The verify
operation returns true when the mission element has completed and we
should move onto the next mission element.
Return true if we do not recognize the command so that we move on to the next command
*******************************************************************************/

// verify_command - 当命令正在执行时,由ap-mission以10hz或更高频率调用的回调函数
//      我们再次检查飞行模式是否为AUTO,以避免ap-mission在我们不处于AUTO模式时触发动作
// verify_command - callback function called from ap-mission at 10hz or higher when a command is being run
//      we double check that the flight mode is AUTO to avoid the possibility of ap-mission triggering actions while we're not in AUTO mode
bool ModeAuto::verify_command(const AP_Mission::Mission_Command& cmd)
{
    if (copter.flightmode != &copter.mode_auto) {
        return false;
    }

    bool cmd_complete = false;

    switch (cmd.id) {
    //
    // 导航命令
    // navigation commands
    //
    case MAV_CMD_NAV_VTOL_TAKEOFF:
    case MAV_CMD_NAV_TAKEOFF:
        // 验证起飞
        cmd_complete = verify_takeoff();
        break;

    case MAV_CMD_NAV_WAYPOINT:
        // 验证导航航点
        cmd_complete = verify_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_LAND:
        // 验证着陆
        cmd_complete = verify_land();
        break;

#if AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED && AC_PAYLOAD_PLACE_ENABLED
    case MAV_CMD_NAV_PAYLOAD_PLACE:
        // 验证有效载荷放置
        cmd_complete = payload_place.verify();
        break;
#endif

    case MAV_CMD_NAV_LOITER_UNLIM:
        // 验证无限盘旋
        cmd_complete = verify_loiter_unlimited();
        break;

    case MAV_CMD_NAV_LOITER_TURNS:
        // 验证圆形盘旋
        cmd_complete = verify_circle(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TIME:
        // 验证定时盘旋
        cmd_complete = verify_loiter_time(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TO_ALT:
        // 验证盘旋到指定高度
        return verify_loiter_to_alt();

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        // 验证返航
        cmd_complete = verify_RTL();
        break;

    case MAV_CMD_NAV_SPLINE_WAYPOINT:
        // 验证样条曲线航点
        cmd_complete = verify_spline_wp(cmd);
        break;

#if AC_NAV_GUIDED
    case MAV_CMD_NAV_GUIDED_ENABLE:
        // 验证引导导航使能
        cmd_complete = verify_nav_guided_enable(cmd);
        break;
#endif

     case MAV_CMD_NAV_DELAY:
        // 验证导航延迟
        cmd_complete = verify_nav_delay(cmd);
        break;

#if AP_SCRIPTING_ENABLED
    case MAV_CMD_NAV_SCRIPT_TIME:
        // 验证脚本时间
        cmd_complete = verify_nav_script_time();
        break;
#endif

    case MAV_CMD_NAV_ATTITUDE_TIME:
        // 验证姿态保持时间
        cmd_complete = verify_nav_attitude_time(cmd);
        break;

    ///
    /// 条件命令
    /// conditional commands
    ///
    case MAV_CMD_CONDITION_DELAY:
        // 验证等待延迟
        cmd_complete = verify_wait_delay();
        break;

    case MAV_CMD_CONDITION_DISTANCE:
        // 验证距离内
        cmd_complete = verify_within_distance();
        break;

    case MAV_CMD_CONDITION_YAW:
        // 验证偏航
        cmd_complete = verify_yaw();
        break;

    // do命令(始终返回true)
    // do commands (always return true)
    case MAV_CMD_DO_CHANGE_SPEED:
    case MAV_CMD_DO_SET_HOME:
    case MAV_CMD_DO_SET_ROI:
#if HAL_MOUNT_ENABLED
    case MAV_CMD_DO_MOUNT_CONTROL:
#endif
#if AC_NAV_GUIDED
    case MAV_CMD_DO_GUIDED_LIMITS:
#endif
#if AP_FENCE_ENABLED
    case MAV_CMD_DO_FENCE_ENABLE:
#endif
#if AP_WINCH_ENABLED
    case MAV_CMD_DO_WINCH:
#endif
    case MAV_CMD_DO_RETURN_PATH_START:
    case MAV_CMD_DO_LAND_START:
        cmd_complete = true;
        break;

    default:
        // 错误消息
        // error message
        gcs().send_text(MAV_SEVERITY_WARNING,"跳过无效命令 #%i",cmd.id);
        // 如果我们不识别该命令,则返回true,以便我们继续执行下一个命令
        // return true if we do not recognize the command so that we move on to the next command
        cmd_complete = true;
        break;
    }


    // 向GCS发送消息
    // send message to GCS
    if (cmd_complete) {
        gcs().send_mission_item_reached_message(cmd.index);
    }

    return cmd_complete;
}

// takeoff_run - 在自动模式下执行起飞
//      由auto_run以100hz或更高频率调用
// takeoff_run - takeoff in auto mode
//      called by auto_run at 100hz or more
void ModeAuto::takeoff_run()
{
    // 如果用户不想提高油门,我们可以自动设置
    // 注意,这可能会使起飞时的解除武装检查失效
    // if the user doesn't want to raise the throttle we can set it automatically
    // note that this can defeat the disarm check on takeoff
    if (option_is_enabled(Option::AllowTakeOffWithoutRaisingThrottle)) {
        copter.set_auto_armed(true);
    }
    auto_takeoff.run();
}

// auto_wp_run - 运行自动航点控制器
//      由auto_run以100hz或更高频率调用
// auto_wp_run - runs the auto waypoint controller
//      called by auto_run at 100hz or more
void ModeAuto::wp_run()
{
    // 如果未解锁,则将油门设置为零并立即退出
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // 将电机设置为全范围
    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // 运行航点控制器
    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // WP_Nav已设置垂直位置控制目标
    // 运行垂直位置控制器并设置输出油门
    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();

    // 调用姿态控制器,使用自动偏航
    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

// auto_land_run - 在自动模式下执行着陆
//      由auto_run以100hz或更高频率调用
// auto_land_run - lands in auto mode
//      called by auto_run at 100hz or more
void ModeAuto::land_run()
{

    // 如果未解锁,则将油门设置为零并立即退出
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // 将电机设置为全范围
    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // 运行正常着陆或精确着陆(如果启用)
    // run normal landing or precision landing (if enabled)
    land_run_normal_or_precland();
}

// auto_rtl_run - 在AUTO飞行模式下执行RTL
//      由auto_run以100hz或更高频率调用
// auto_rtl_run - rtl in AUTO flight mode
//      called by auto_run at 100hz or more
void ModeAuto::rtl_run()
{
    // 调用常规rtl飞行模式运行函数
    // call regular rtl flight mode run function
    copter.mode_rtl.run(false);
}

// auto_circle_run - 在AUTO飞行模式下执行圆形
//      由auto_run以100hz或更高频率调用
// auto_circle_run - circle in AUTO flight mode
//      called by auto_run at 100hz or more
void ModeAuto::circle_run()
{
    // 调用圆形控制器
    // call circle controller
    copter.failsafe_terrain_set_status(copter.circle_nav->update());

    // WP_Nav已设置垂直位置控制目标
    // 运行垂直位置控制器并设置输出油门
    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();

    // 调用姿态控制器,使用自动偏航
    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

#if AC_NAV_GUIDED || AP_SCRIPTING_ENABLED
// auto_nav_guided_run - 允许外部导航控制器控制
//      由auto_run以100hz或更高频率调用
// auto_nav_guided_run - allows control by external navigation controller
//      called by auto_run at 100hz or more
void ModeAuto::nav_guided_run()
{
    // 调用常规引导飞行模式运行函数
    // call regular guided flight mode run function
    copter.mode_guided.run();
}
#endif  // AC_NAV_GUIDED || AP_SCRIPTING_ENABLED

// auto_loiter_run - 在AUTO飞行模式下执行盘旋
//      由auto_run以100hz或更高频率调用
// auto_loiter_run - loiter in AUTO flight mode
//      called by auto_run at 100hz or more
void ModeAuto::loiter_run()
{
    // 如果未解锁,则将油门设置为零并立即退出
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // 将电机设置为全范围
    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // 运行航点和z轴位置控制器
    // run waypoint and z-axis position controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    pos_control->update_z_controller();

    // 调用姿态控制器,使用自动偏航
    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

// auto_loiter_run - 在AUTO飞行模式下执行盘旋到高度
//      由auto_run以100hz或更高频率调用
// auto_loiter_run - loiter to altitude in AUTO flight mode
//      called by auto_run at 100hz or more
void ModeAuto::loiter_to_alt_run()
{
    // 如果未自动解锁或电机联锁未启用,则将油门设置为零并立即退出
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (is_disarmed_or_landed() || !motors->get_interlock()) {
        make_safe_ground_handling();
        return;
    }

    // 可能只需运行航点控制器:
    // possibly just run the waypoint controller:
    if (!loiter_to_alt.reached_destination_xy) {
        loiter_to_alt.reached_destination_xy = wp_nav->reached_wp_destination_xy();
        if (!loiter_to_alt.reached_destination_xy) {
            wp_run();
            return;
        }
    }

    if (!loiter_to_alt.loiter_start_done) {
        // 设置水平速度和加速度限制
        // set horizontal speed and acceleration limits
        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

        if (!pos_control->is_active_xy()) {
            pos_control->init_xy_controller();
        }

        loiter_to_alt.loiter_start_done = true;
    }
    const float alt_error_cm = copter.current_loc.alt - loiter_to_alt.alt;
    if (fabsf(alt_error_cm) < 5.0) { // 随机数
        loiter_to_alt.reached_alt = true;
    } else if (alt_error_cm * loiter_to_alt.alt_error_cm < 0) {
        // 我们之前在上方,现在在下方,或者反之
        // we were above and are now below, or vice-versa
        loiter_to_alt.reached_alt = true;
    }
    loiter_to_alt.alt_error_cm = alt_error_cm;

    // 盘旋...
    // loiter...

    land_run_horizontal_control();

    // 计算垂直速度需求,使飞行器接近所需高度
    // Compute a vertical velocity demand such that the vehicle
    // approaches the desired altitude.
    float target_climb_rate = sqrt_controller(
        -alt_error_cm,
        pos_control->get_pos_z_p().kP(),
        pos_control->get_max_accel_z_cmss(),
        G_Dt);
    target_climb_rate = constrain_float(target_climb_rate, pos_control->get_max_speed_down_cms(), pos_control->get_max_speed_up_cms());

    // 获取避障调整后的爬升率
    // get avoidance adjusted climb rate
    target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

#if AP_RANGEFINDER_ENABLED
    // 根据表面测量更新垂直偏移
    // update the vertical offset based on the surface measurement
    copter.surface_tracking.update_surface_offset();
#endif

    // 将命令的爬升率发送到位置控制器
    // Send the commanded climb rate to the position controller
    pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);

    pos_control->update_z_controller();
}

// 在指定时间内保持姿态
// maintain an attitude for a specified time
void ModeAuto::nav_attitude_time_run()
{
    // 如果未自动解锁或电机联锁未启用,则将油门设为零并立即退出
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (is_disarmed_or_landed() || !motors->get_interlock()) {
        make_safe_ground_handling();
        return;
    }

    // 限制爬升速率
    // constrain climb rate
    float target_climb_rate_cms = constrain_float(nav_attitude_time.climb_rate * 100.0, pos_control->get_max_speed_down_cms(), pos_control->get_max_speed_up_cms());

    // 获取避障调整后的爬升速率
    // get avoidance adjusted climb rate
    target_climb_rate_cms = get_avoidance_adjusted_climbrate(target_climb_rate_cms);

    // 限制和缩放倾斜角度
    // limit and scale lean angles
    const float angle_limit_cd = MAX(1000.0f, MIN(copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max_cd()));
    Vector2f target_rp_cd(nav_attitude_time.roll_deg * 100, nav_attitude_time.pitch_deg * 100);
    target_rp_cd.limit_length(angle_limit_cd);

    // 将目标发送到姿态控制器
    // send targets to attitude controller
    attitude_control->input_euler_angle_roll_pitch_yaw(target_rp_cd.x, target_rp_cd.y, nav_attitude_time.yaw_deg * 100, true);

    // 将命令的爬升速率发送到位置控制器
    // Send the commanded climb rate to the position controller
    pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate_cms);

    pos_control->update_z_controller();
}

#if AC_PAYLOAD_PLACE_ENABLED
// auto_payload_place_run - 在自动模式下放置物体
// auto_payload_place_run - places an object in auto mode
//      以100Hz或更高频率由auto_run调用
//      called by auto_run at 100hz or more
void PayloadPlace::run()
{
    const char* prefix_str = "PayloadPlace:";

    // 如果飞机已解除武装或已着陆,则进行安全地面处理
    if (copter.flightmode->is_disarmed_or_landed()) {
        copter.flightmode->make_safe_ground_handling();
        return;
    }

    // 将电机设置为全范围
    // set motors to full range
    copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    const uint32_t descent_thrust_cal_duration_ms = 2000; // 下降推力校准持续时间(毫秒)
    const uint32_t placed_check_duration_ms = 500; // 在认为已放置之前,我们必须低于油门阈值的时间

    auto &g2 = copter.g2;
    const auto &g = copter.g;
    auto &inertial_nav = copter.inertial_nav;
    auto *attitude_control = copter.attitude_control;
    const auto &pos_control = copter.pos_control;
    const auto &wp_nav = copter.wp_nav;

    // 垂直推力在添加角度增强之前从姿态控制器获取
    // Vertical thrust is taken from the attitude controller before angle boost is added
    const float thrust_level = attitude_control->get_throttle_in();
    const uint32_t now_ms = AP_HAL::millis();

    // 如果我们可能已着陆,则放松位置目标
    // 如果我们发现已着陆,则立即释放负载:
    // relax position target if we might be landed
    // if we discover we've landed then immediately release the load:
    if (copter.ap.land_complete || copter.ap.land_complete_maybe) {
        pos_control->soften_for_landing_xy();
        switch (state) {
        case State::FlyToLocation:
            // 这在wp_run()中处理
            // this is handled in wp_run()
            break;
        case State::Descent_Start:
            // 在这个循环中不做任何事
            // do nothing on this loop
            break;
        case State::Descent:
            gcs().send_text(MAV_SEVERITY_INFO, "%s landed", prefix_str);
            state = State::Release;
            break;
        case State::Release:
        case State::Releasing:
        case State::Delay:
        case State::Ascent_Start:
        case State::Ascent:
        case State::Done:
            break;
        }
    }

#if AP_GRIPPER_ENABLED
    // 如果飞行员手动释放负载:
    // if pilot releases load manually:
    if (AP::gripper().valid() && AP::gripper().released()) {
        switch (state) {
        case State::FlyToLocation:
        case State::Descent_Start:
            gcs().send_text(MAV_SEVERITY_INFO, "%s Abort: Gripper Open", prefix_str);
            // Descent_Start尚未运行,所以我们还必须初始化descent_start_altitude_cm
            // Descent_Start has not run so we must also initalise descent_start_altitude_cm
            descent_start_altitude_cm = inertial_nav.get_position_z_up_cm();
            state = State::Done;
            break;
        case State::Descent:
            gcs().send_text(MAV_SEVERITY_INFO, "%s Manual release", prefix_str);
            state = State::Release;
            break;
        case State::Release:
        case State::Releasing:
        case State::Delay:
        case State::Ascent_Start:
        case State::Ascent:
        case State::Done:
            break;
        }
    }
#endif

    switch (state) {
    case State::FlyToLocation:
        if (copter.wp_nav->reached_wp_destination()) {
            start_descent();
        }
        break;

    case State::Descent_Start:
        descent_established_time_ms = now_ms;
        descent_start_altitude_cm = inertial_nav.get_position_z_up_cm();
        // 将下降速率限制在wp_nav中设置的限制内并不是必要的,但为了安全起见这样做
        // limiting the decent rate to the limit set in wp_nav is not necessary but done for safety
        descent_speed_cms = MIN((is_positive(g2.pldp_descent_speed_ms)) ? g2.pldp_descent_speed_ms * 100.0 : abs(g.land_speed), wp_nav->get_default_speed_down());
        descent_thrust_level = 1.0;
        state = State::Descent;
        FALLTHROUGH;

    case State::Descent:
        // 检查最大下降距离
        // check maximum decent distance
        if (!is_zero(descent_max_cm) &&
            descent_start_altitude_cm - inertial_nav.get_position_z_up_cm() > descent_max_cm) {
            state = State::Ascent_Start;
            gcs().send_text(MAV_SEVERITY_WARNING, "%s Reached maximum descent", prefix_str);
            break;
        }
        // 在飞机达到恒定下降速率后校准下降推力,并在达到阈值时释放
        // calibrate the decent thrust after aircraft has reached constant decent rate and release if threshold is reached
        if (pos_control->get_vel_desired_cms().z > -0.95 * descent_speed_cms) {
            // 下降速率尚未达到descent_speed_cms
            // decent rate has not reached descent_speed_cms
            descent_established_time_ms = now_ms;
            break;
        } else if (now_ms - descent_established_time_ms < descent_thrust_cal_duration_ms) {
            // 记录descent_thrust_cal_duration_ms内的最小推力
            // record minimum thrust for descent_thrust_cal_duration_ms
            descent_thrust_level = MIN(descent_thrust_level, thrust_level);
            place_start_time_ms = now_ms;
            break;
        } else if (thrust_level > g2.pldp_thrust_placed_fraction * descent_thrust_level) {
            // 推力高于最小阈值
            // thrust is above minimum threshold
            place_start_time_ms = now_ms;
            break;
        } else if (is_positive(g2.pldp_range_finder_maximum_m)) {
            if (!copter.rangefinder_state.enabled) {
                // 中止负载放置,因为测距仪未启用
                // abort payload place because rangefinder is not enabled
                state = State::Ascent_Start;
                gcs().send_text(MAV_SEVERITY_WARNING, "%s PLDP_RNG_MAX set and rangefinder not enabled", prefix_str);
                break;
            } else if (copter.rangefinder_alt_ok() && (copter.rangefinder_state.glitch_count == 0) && (copter.rangefinder_state.alt_cm > g2.pldp_range_finder_maximum_m * 100.0)) {
                // 测距仪高度高于最大值
                // range finder altitude is above maximum
                place_start_time_ms = now_ms;
                break;
            }
        }

        // 如果我们到达这里:
        // 1. 我们已达到下降速度
        // 2. 测量了下降所需的推力水平
        // 3. 检测到我们的推力需求已减少
        // 4. 如果设置了,测距仪范围已降至最小值以下
        // 5. place_start_time_ms已初始化

        // 必须检测到负载接触地面0.5秒

        if (now_ms - place_start_time_ms > placed_check_duration_ms) {
            state = State::Release;
            gcs().send_text(MAV_SEVERITY_INFO, "%s payload release thrust threshold: %f", prefix_str, static_cast<double>(g2.pldp_thrust_placed_fraction * descent_thrust_level));
        }
        break;

    case State::Release:
        // 重新初始化垂直位置控制器以消除由于负载接触地面而产生的不连续性
        // Reinitialise vertical position controller to remove discontinuity due to touch down of payload
        pos_control->init_z_controller_no_descent();
#if AP_GRIPPER_ENABLED
        if (AP::gripper().valid()) {
            gcs().send_text(MAV_SEVERITY_INFO, "%s Releasing the gripper", prefix_str);
            AP::gripper().release();
            state = State::Releasing;
        } else {
            state = State::Delay;
        }
#else
        state = State::Delay;
#endif
        break;

    case State::Releasing:
#if AP_GRIPPER_ENABLED
        if (AP::gripper().valid() && !AP::gripper().released()) {
            break;
        }
#endif
        state = State::Delay;
        FALLTHROUGH;

    case State::Delay:
        // 如果我们到达这里,我们已经完成了释放夹持器
        // If we get here we have finished releasing the gripper
        if (now_ms - place_start_time_ms < placed_check_duration_ms + g2.pldp_delay_s * 1000.0) {
            break;
        }
        FALLTHROUGH;

    case State::Ascent_Start:
        state = State::Ascent;
        FALLTHROUGH;

    case State::Ascent: {
        // 当我们距离目标高度停止距离的10%以内时,上升完成
        // 停止距离从vel_threshold_fraction * 最大速度计算
        // Ascent complete when we are less than 10% of the stopping
        // distance from the target altitude stopping distance from
        // vel_threshold_fraction * max velocity
        const float vel_threshold_fraction = 0.1;
        const float stop_distance = 0.5 * sq(vel_threshold_fraction * copter.pos_control->get_max_speed_up_cms()) / copter.pos_control->get_max_accel_z_cmss();
        bool reached_altitude = copter.pos_control->get_pos_target_z_cm() >= descent_start_altitude_cm - stop_distance;
        if (reached_altitude) {
            state = State::Done;
        }
        break;
    }
    case State::Done:
        break;
    default:
        // 这种情况不应该发生
        // this should never happen
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        break;
    }

    switch (state) {
    case State::FlyToLocation:
        // 这种情况不应该发生
        // this should never happen
        return copter.mode_auto.wp_run();
    case State::Descent_Start:
    case State::Descent:
        copter.flightmode->land_run_horizontal_control();
        // 更新高度目标并调用位置控制器
        // update altitude target and call position controller
        pos_control->land_at_climb_rate_cm(-descent_speed_cms, true);
        break;
    case State::Release:
    case State::Releasing:
    case State::Delay:
    case State::Ascent_Start:
        copter.flightmode->land_run_horizontal_control();
        // 更新高度目标并调用位置控制器
        // update altitude target and call position controller
        pos_control->land_at_climb_rate_cm(0.0, false);
        break;
    case State::Ascent:
    case State::Done:
        float vel = 0.0;
        copter.flightmode->land_run_horizontal_control();
        pos_control->input_pos_vel_accel_z(descent_start_altitude_cm, vel, 0.0);
        break;
    }
    pos_control->update_z_controller();
}
#endif

// 将target_loc的高度设置为飞行器当前高度,但不改变target_loc的坐标系
// 在使用地形高度的情况下,可以使用地形数据库或测距仪
// 成功返回true,失败返回false
// sets the target_loc's alt to the vehicle's current alt but does not change target_loc's frame
// in the case of terrain altitudes either the terrain database or the rangefinder may be used
// returns true on success, false on failure
bool ModeAuto::shift_alt_to_current_alt(Location& target_loc) const
{
    // 如果正在使用基于测距仪的地形高度,则将高度设置为当前测距仪高度
    // if terrain alt using rangefinder is being used then set alt to current rangefinder altitude
    if ((target_loc.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN) &&
        (wp_nav->get_terrain_source() == AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER)) {
        int32_t curr_rngfnd_alt_cm;
        if (copter.get_rangefinder_height_interpolated_cm(curr_rngfnd_alt_cm)) {
            // wp_nav正在使用测距仪,所以使用当前测距仪高度
            // wp_nav is using rangefinder so use current rangefinder alt
            target_loc.set_alt_cm(MAX(curr_rngfnd_alt_cm, 200), Location::AltFrame::ABOVE_TERRAIN);
            return true;
        }
        return false;
    }

    // 复制当前位置并将坐标系更改为与目标匹配
    // take copy of current location and change frame to match target
    Location currloc = copter.current_loc;
    if (!currloc.change_alt_frame(target_loc.get_alt_frame())) {
        // 这可能由于缺少地形数据库高度而失败
        // this could fail due missing terrain database alt
        return false;
    }

    // 设置target_loc的高度
    // set target_loc's alt
    target_loc.set_alt_cm(currloc.alt, currloc.get_alt_frame());
    return true;
}

/********************************************************************************/
// Nav (Must) commands
/********************************************************************************/

// do_takeoff - 初始化起飞导航命令
// do_takeoff - initiate takeoff navigation command
void ModeAuto::do_takeoff(const AP_Mission::Mission_Command& cmd)
{
    // 设置wp导航目标为当前位置上方的安全高度
    // Set wp navigation target to safe altitude above current position
    takeoff_start(cmd.content.location);
}

// 从命令中获取位置,如果命令中未提供某些参数,则使用默认位置
// 返回包含命令位置信息的Location对象
Location ModeAuto::loc_from_cmd(const AP_Mission::Mission_Command& cmd, const Location& default_loc) const
{
    Location ret(cmd.content.location);

    // 如果经纬度为零,则使用当前经纬度
    // use current lat, lon if zero
    if (ret.lat == 0 && ret.lng == 0) {
        ret.lat = default_loc.lat;
        ret.lng = default_loc.lng;
    }
    // 如果命令中未提供高度,则使用默认高度
    // use default altitude if not provided in cmd
    if (ret.alt == 0) {
        // 设置为default_loc的高度,但使用命令的高度坐标系
        // 注意这可能会使用地形数据库
        // set to default_loc's altitude but in command's alt frame
        // note that this may use the terrain database
        int32_t default_alt;
        if (default_loc.get_alt_cm(ret.get_alt_frame(), default_alt)) {
            ret.set_alt_cm(default_alt, ret.get_alt_frame());
        } else {
            // 默认使用default_loc的高度和坐标系
            // default to default_loc's altitude and frame
            ret.set_alt_cm(default_loc.alt, default_loc.get_alt_frame());
        }
    }
    return ret;
}

// do_nav_wp - 初始化移动到下一个航点
// do_nav_wp - initiate move to next waypoint
void ModeAuto::do_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    // 计算当经纬度或高度为零时使用的默认位置
    // calculate default location used when lat, lon or alt is zero
    Location default_loc = copter.current_loc;
    if (wp_nav->is_active() && wp_nav->reached_wp_destination()) {
        if (!wp_nav->get_wp_destination_loc(default_loc)) {
            // 这种情况不应该发生
            // this should never happen
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        }
    }

    // 从命令获取航点位置并发送到wp_nav
    // get waypoint's location from command and send to wp_nav
    const Location target_loc = loc_from_cmd(cmd, default_loc);

    if (!wp_start(target_loc)) {
        // 设置下一个目的地失败只可能是由于缺少地形数据
        // failure to set next destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    // 这将用于记录我们到达或经过航点后的时间(以毫秒为单位)
    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time = 0;
    // 这是延迟时间,以秒为单位存储
    // this is the delay, stored in seconds
    loiter_time_max = cmd.p1;

    // 如果需要,设置下一个目的地
    // set next destination if necessary
    if (!set_next_wp(cmd, target_loc)) {
        // 设置下一个目的地失败只可能是由于缺少地形数据
        // failure to set next destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }
}

// 检查下一个任务命令,如果需要将其添加为目的地
// 支持直线和样条曲线航点
// cmd应该是当前命令
// default_loc应该是current_cmd的目的地,但针对用户将纬度、经度或高度设置为零的情况进行了修正
// 成功返回true,失败返回false(仅应由于无法检索地形数据而失败)
// checks the next mission command and adds it as a destination if necessary
// supports both straight line and spline waypoints
// cmd should be the current command
// default_loc should be the destination from the current_cmd but corrected for cases where user set lat, lon or alt to zero
// returns true on success, false on failure which should only happen due to a failure to retrieve terrain data
bool ModeAuto::set_next_wp(const AP_Mission::Mission_Command& current_cmd, const Location &default_loc)
{
    // 如果当前命令有延迟,意味着飞行器将在目的地停止,则不添加下一个航点
    // do not add next wp if current command has a delay meaning the vehicle will stop at the destination
    if (current_cmd.p1 > 0) {
        return true;
    }

    // 如果没有更多导航命令,则不添加下一个航点
    // do not add next wp if there are no more navigation commands
    AP_Mission::Mission_Command next_cmd;
    if (!mission.get_next_nav_cmd(current_cmd.index+1, next_cmd)) {
        return true;
    }

    // 飞行器是否应该在目标位置停止取决于下一个命令
    // whether vehicle should stop at the target position depends upon the next command
    switch (next_cmd.id) {
    case MAV_CMD_NAV_WAYPOINT:
    case MAV_CMD_NAV_LOITER_UNLIM:
#if AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED
    case MAV_CMD_NAV_PAYLOAD_PLACE:
#endif
    case MAV_CMD_NAV_LOITER_TIME: {
        const Location dest_loc = loc_from_cmd(current_cmd, default_loc);
        const Location next_dest_loc = loc_from_cmd(next_cmd, dest_loc);
        return wp_nav->set_wp_destination_next_loc(next_dest_loc);
    }
    case MAV_CMD_NAV_SPLINE_WAYPOINT: {
        // 从命令获取样条曲线的位置和下一个位置,并发送到wp_nav
        // get spline's location and next location from command and send to wp_nav
        Location next_dest_loc, next_next_dest_loc;
        bool next_next_dest_loc_is_spline;
        get_spline_from_cmd(next_cmd, default_loc, next_dest_loc, next_next_dest_loc, next_next_dest_loc_is_spline);
        return wp_nav->set_spline_destination_next_loc(next_dest_loc, next_next_dest_loc, next_next_dest_loc_is_spline);
    }
    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_LAND:
        // 停止,因为我们可能会在相对、绝对和地形高度类型之间切换
        // stop because we may change between rel,abs and terrain alt types
    case MAV_CMD_NAV_LOITER_TURNS:
    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
    case MAV_CMD_NAV_VTOL_TAKEOFF:
    case MAV_CMD_NAV_TAKEOFF:
        // 对于RTL和起飞命令总是停止
        // always stop for RTL and takeoff commands
    default:
        // 对于不支持的命令,停止更安全
        // for unsupported commands it is safer to stop
        break;
    }

    return true;
}

// do_land - 初始化着陆程序
// do_land - initiate landing procedure
void ModeAuto::do_land(const AP_Mission::Mission_Command& cmd)
{
    // To-Do: 检查我们是否已经着陆
    // To-Do: check if we have already landed

    // 如果提供了位置,我们就以当前高度飞到那个位置
    // if location provided we fly to that location at current altitude
    if (cmd.content.location.lat != 0 || cmd.content.location.lng != 0) {
        // 设置状态为飞向位置
        // set state to fly to location
        state = State::FlyToLocation;

        // 将cmd转换为location类
        // convert cmd to location class
        Location target_loc(cmd.content.location);
        if (!shift_alt_to_current_alt(target_loc)) {
            // 这只能由于缺少地形数据库高度或测距仪高度而失败
            // 使用当前相对于家的高度并报告错误
            // this can only fail due to missing terrain database alt or rangefinder alt
            // use current alt-above-home and report error
            target_loc.set_alt_cm(copter.current_loc.alt, Location::AltFrame::ABOVE_HOME);
            LOGGER_WRITE_ERROR(LogErrorSubsystem::TERRAIN, LogErrorCode::MISSING_TERRAIN_DATA);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Land: no terrain data, using alt-above-home");
        }

        if (!wp_start(target_loc)) {
            // 设置下一个目的地失败只可能是由于缺少地形数据
            // failure to set next destination can only be because of missing terrain data
            copter.failsafe_terrain_on_event();
            return;
        }
    } else {
        // 设置着陆状态
        // set landing state
        state = State::Descending;

        // 初始化着陆控制器
        // initialise landing controller
        land_start();
    }
}

// do_loiter_unlimited - 开始无限期盘旋
// 注意: 调用者应设置yaw_mode
// do_loiter_unlimited - start loitering with no end conditions
// note: caller should set yaw_mode
void ModeAuto::do_loiter_unlimited(const AP_Mission::Mission_Command& cmd)
{
    // 转换回位置
    // convert back to location
    Location target_loc(cmd.content.location);

    // 如果未提供,则使用当前位置
    // use current location if not provided
    if (target_loc.lat == 0 && target_loc.lng == 0) {
        // To-Do: 使这更简单
        // To-Do: make this simpler
        Vector3f temp_pos;
        copter.wp_nav->get_wp_stopping_point_xy(temp_pos.xy());
        const Location temp_loc(temp_pos, Location::AltFrame::ABOVE_ORIGIN);
        target_loc.lat = temp_loc.lat;
        target_loc.lng = temp_loc.lng;
    }

    // 如果未提供,则使用当前高度
    // To-Do: 使用z轴停止点而不是当前高度
    // use current altitude if not provided
    // To-Do: use z-axis stopping point instead of current alt
    if (target_loc.alt == 0) {
        // 设置为当前高度,但使用命令的高度坐标系
        // set to current altitude but in command's alt frame
        int32_t curr_alt;
        if (copter.current_loc.get_alt_cm(target_loc.get_alt_frame(),curr_alt)) {
            target_loc.set_alt_cm(curr_alt, target_loc.get_alt_frame());
        } else {
            // 默认使用当前高度作为相对于家的高度
            // default to current altitude as alt-above-home
            target_loc.set_alt_cm(copter.current_loc.alt,
                                  copter.current_loc.get_alt_frame());
        }
    }

    // 启动航点导航器并提供所需位置
    // start way point navigator and provide it the desired location
    if (!wp_start(target_loc)) {
        // 设置下一个目的地失败只可能是由于缺少地形数据
        // failure to set next destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }
}

// do_circle - 初始化圆周运动
// do_circle - initiate moving in a circle
void ModeAuto::do_circle(const AP_Mission::Mission_Command& cmd)
{
    const Location circle_center = loc_from_cmd(cmd, copter.current_loc);

    // 计算半径
    // calculate radius
    uint16_t circle_radius_m = HIGHBYTE(cmd.p1); // 圆的半径保存在p1的高字节中 // circle radius held in high byte of p1
    if (cmd.id == MAV_CMD_NAV_LOITER_TURNS &&
        cmd.type_specific_bits & (1U << 0)) {
        // 特殊存储处理允许更大的半径
        // special storage handling allows for larger radii
        circle_radius_m *= 10;
    }

    // 如果圆应该是逆时针方向,则为true
    // true if circle should be ccw
    const bool circle_direction_ccw = cmd.content.location.loiter_ccw;

    // 移动到圆的边缘(verify_circle将确保我们一到达边缘就开始绕圈)
    // move to edge of circle (verify_circle) will ensure we begin circling once we reach the edge
    circle_movetoedge_start(circle_center, circle_radius_m, circle_direction_ccw);
}

// do_loiter_time - 在一个点开始盘旋一定时间
// 注意: 调用者应设置yaw_mode
// do_loiter_time - initiate loitering at a point for a given time period
// note: caller should set yaw_mode
void ModeAuto::do_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    // 重用无限期盘旋
    // re-use loiter unlimited
    do_loiter_unlimited(cmd);

    // 设置盘旋计时器
    // setup loiter timer
    loiter_time     = 0;
    loiter_time_max = cmd.p1;     // 单位是(秒) // units are (seconds)
}

// do_loiter_alt - 在一个点开始盘旋直到达到给定高度
// 注意: 调用者应设置yaw_mode
// do_loiter_alt - initiate loitering at a point until a given altitude is reached
// note: caller should set yaw_mode
void ModeAuto::do_loiter_to_alt(const AP_Mission::Mission_Command& cmd)
{
    // 重用无限期盘旋
    // re-use loiter unlimited
    do_loiter_unlimited(cmd);

    // 如果我们不是导航到一个位置,那么我们必须调整
    // 当前位置的高度
    // if we aren't navigating to a location then we have to adjust
    // altitude for current location
    Location target_loc(cmd.content.location);
    if (target_loc.lat == 0 && target_loc.lng == 0) {
        target_loc.lat = copter.current_loc.lat;
        target_loc.lng = copter.current_loc.lng;
    }

    if (!target_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, loiter_to_alt.alt)) {
        loiter_to_alt.reached_destination_xy = true;
        loiter_to_alt.reached_alt = true;
        gcs().send_text(MAV_SEVERITY_INFO, "bad do_loiter_to_alt");
        return;
    }
    loiter_to_alt.reached_destination_xy = false;
    loiter_to_alt.loiter_start_done = false;
    loiter_to_alt.reached_alt = false;
    loiter_to_alt.alt_error_cm = 0;

    // 设置垂直速度和加速度限制
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // 设置子模式
    // set submode
    set_submode(SubMode::LOITER_TO_ALT);
}

// do_spline_wp - 初始化移动到下一个航点
// do_spline_wp - initiate move to next waypoint
void ModeAuto::do_spline_wp(const AP_Mission::Mission_Command& cmd)
{
    // 计算当经纬度或高度为零时使用的默认位置
    // calculate default location used when lat, lon or alt is zero
    Location default_loc = copter.current_loc;
    if (wp_nav->is_active() && wp_nav->reached_wp_destination()) {
        if (!wp_nav->get_wp_destination_loc(default_loc)) {
            // 这种情况不应该发生
            // this should never happen
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        }
    }

    // 从命令获取样条曲线的位置和下一个位置,并发送到wp_nav
    // get spline's location and next location from command and send to wp_nav
    Location dest_loc, next_dest_loc;
    bool next_dest_loc_is_spline;
    get_spline_from_cmd(cmd, default_loc, dest_loc, next_dest_loc, next_dest_loc_is_spline);
    if (!wp_nav->set_spline_destination_loc(dest_loc, next_dest_loc, next_dest_loc_is_spline)) {
        // 设置目的地失败只可能是由于缺少地形数据
        // failure to set destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    // 这将用于记录我们到达或经过航点后的时间(以毫秒为单位)
    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time = 0;
    // 这是延迟时间,以秒为单位存储
    // this is the delay, stored in seconds
    loiter_time_max = cmd.p1;

    // 如果需要,设置下一个目的地
    // set next destination if necessary
    if (!set_next_wp(cmd, dest_loc)) {
        // 设置下一个目的地失败只可能是由于缺少地形数据
        // failure to set next destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    // 初始化偏航
    // To-Do: 只有当前一个导航命令不是WP时才重置偏航。这将允许移除ROI的特殊检查
    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw.mode() != AutoYaw::Mode::ROI) {
        auto_yaw.set_mode_to_default(false);
    }

    // 设置子模式
    // set submode
    set_submode(SubMode::WP);
}

// 计算从任务命令构建样条曲线所需的位置
// dest_loc从cmd的位置填充,在纬度和经度或高度为零的情况下使用default_loc
// next_dest_loc和nest_dest_loc_is_spline用下一个导航命令的位置填充(如果存在)。如果不存在,则设置为dest_loc和false
// calculate locations required to build a spline curve from a mission command
// dest_loc is populated from cmd's location using default_loc in cases where the lat and lon or altitude is zero
// next_dest_loc and nest_dest_loc_is_spline is filled in with the following navigation command's location if it exists.  If it does not exist it is set to the dest_loc and false
void ModeAuto::get_spline_from_cmd(const AP_Mission::Mission_Command& cmd, const Location& default_loc, Location& dest_loc, Location& next_dest_loc, bool& next_dest_loc_is_spline)
{
    dest_loc = loc_from_cmd(cmd, default_loc);

    // 如果这个段结束时没有延迟,获取下一个导航命令
    // if there is no delay at the end of this segment get next nav command
    AP_Mission::Mission_Command temp_cmd;
    if (cmd.p1 == 0 && mission.get_next_nav_cmd(cmd.index+1, temp_cmd)) {
        next_dest_loc = loc_from_cmd(temp_cmd, dest_loc);
        next_dest_loc_is_spline = temp_cmd.id == MAV_CMD_NAV_SPLINE_WAYPOINT;
    } else {
        next_dest_loc = dest_loc;
        next_dest_loc_is_spline = false;
    }
}

#if AC_NAV_GUIDED
// do_nav_guided_enable - 开始接受外部导航计算机的命令
// do_nav_guided_enable - initiate accepting commands from external nav computer
void ModeAuto::do_nav_guided_enable(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 > 0) {
        // 在自动模式内启动引导
        // start guided within auto
        nav_guided_start();
    }
}

// do_guided_limits - 将引导限制传递给引导控制器
// do_guided_limits - pass guided limits to guided controller
void ModeAuto::do_guided_limits(const AP_Mission::Mission_Command& cmd)
{
    copter.mode_guided.limit_set(
        cmd.p1 * 1000, // 将秒转换为毫秒 // convert seconds to ms
        cmd.content.guided_limits.alt_min * 100.0f,    // 将米转换为厘米 // convert meters to cm
        cmd.content.guided_limits.alt_max * 100.0f,    // 将米转换为厘米 // convert meters to cm
        cmd.content.guided_limits.horiz_max * 100.0f); // 将米转换为厘米 // convert meters to cm
}
#endif  // AC_NAV_GUIDED

// do_nav_delay - 延迟下一个导航命令
// do_nav_delay - Delay the next navigation command
void ModeAuto::do_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    nav_delay_time_start_ms = millis();

    if (cmd.content.nav_delay.seconds > 0) {
        // 相对延迟
        // relative delay
        nav_delay_time_max_ms = cmd.content.nav_delay.seconds * 1000; // 将秒转换为毫秒 // convert seconds to milliseconds
    } else {
        // 绝对延迟到UTC时间
        // absolute delay to utc time
#if AP_RTC_ENABLED
        nav_delay_time_max_ms = AP::rtc().get_time_utc(cmd.content.nav_delay.hour_utc, cmd.content.nav_delay.min_utc, cmd.content.nav_delay.sec_utc, 0);
#else
        nav_delay_time_max_ms = 0;
#endif
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Delaying %u sec", (unsigned)(nav_delay_time_max_ms/1000));
}

#if AP_SCRIPTING_ENABLED
// 开始接受来自lua脚本的位置、速度和加速度目标
// start accepting position, velocity and acceleration targets from lua scripts
void ModeAuto::do_nav_script_time(const AP_Mission::Mission_Command& cmd)
{
    // 调用常规引导飞行模式初始化
    // call regular guided flight mode initialisation
    if (copter.mode_guided.init(true)) {
        nav_scripting.done = false;
        nav_scripting.id++;
        nav_scripting.start_ms = millis();
        nav_scripting.command = cmd.content.nav_script_time.command;
        nav_scripting.timeout_s = cmd.content.nav_script_time.timeout_s;
        nav_scripting.arg1 = cmd.content.nav_script_time.arg1.get();
        nav_scripting.arg2 = cmd.content.nav_script_time.arg2.get();
        nav_scripting.arg3 = cmd.content.nav_script_time.arg3;
        nav_scripting.arg4 = cmd.content.nav_script_time.arg4;
        set_submode(SubMode::NAV_SCRIPT_TIME);
    } else {
        // 为了安全,我们将nav_scripting设置为done,以防止任务卡住
        // for safety we set nav_scripting to done to protect against the mission getting stuck
        nav_scripting.done = true;
    }
}
#endif

// 开始在指定时间内保持一个姿态
// start maintaining an attitude for a specified time
void ModeAuto::do_nav_attitude_time(const AP_Mission::Mission_Command& cmd)
{
    // 将命令参数复制到本地结构中
    // copy command arguments into local structure
    nav_attitude_time.roll_deg = cmd.content.nav_attitude_time.roll_deg;
    nav_attitude_time.pitch_deg = cmd.content.nav_attitude_time.pitch_deg;
    nav_attitude_time.yaw_deg = cmd.content.nav_attitude_time.yaw_deg;
    nav_attitude_time.climb_rate = cmd.content.nav_attitude_time.climb_rate;
    nav_attitude_time.start_ms = AP_HAL::millis();
    set_submode(SubMode::NAV_ATTITUDE_TIME);
}

/********************************************************************************/
// 条件(可能)命令
// Condition (May) commands
/********************************************************************************/

void ModeAuto::do_wait_delay(const AP_Mission::Mission_Command& cmd)
{
    condition_start = millis();
    condition_value = cmd.content.delay.seconds * 1000;     // 将秒转换为毫秒 // convert seconds to milliseconds
}

void ModeAuto::do_within_distance(const AP_Mission::Mission_Command& cmd)
{
    condition_value  = cmd.content.distance.meters * 100;
}

void ModeAuto::do_yaw(const AP_Mission::Mission_Command& cmd)
{
    auto_yaw.set_fixed_yaw(
        cmd.content.yaw.angle_deg,
        cmd.content.yaw.turn_rate_dps,
        cmd.content.yaw.direction,
        cmd.content.yaw.relative_angle > 0);
}

/********************************************************************************/
// 执行(立即)命令
// Do (Now) commands
/********************************************************************************/

void ModeAuto::do_change_speed(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.content.speed.target_ms > 0) {
        switch (cmd.content.speed.speed_type) {
        case SPEED_TYPE_CLIMB_SPEED:
            copter.wp_nav->set_speed_up(cmd.content.speed.target_ms * 100.0f);
            desired_speed_override.up = cmd.content.speed.target_ms;
            break;
        case SPEED_TYPE_DESCENT_SPEED:
            copter.wp_nav->set_speed_down(cmd.content.speed.target_ms * 100.0f);
            desired_speed_override.down = cmd.content.speed.target_ms;
            break;
        case SPEED_TYPE_AIRSPEED:
        case SPEED_TYPE_GROUNDSPEED:
            copter.wp_nav->set_speed_xy(cmd.content.speed.target_ms * 100.0f);
            desired_speed_override.xy = cmd.content.speed.target_ms;
            break;
        }
    }
}

void ModeAuto::do_set_home(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 == 1 || (cmd.content.location.lat == 0 && cmd.content.location.lng == 0 && cmd.content.location.alt == 0)) {
        if (!copter.set_home_to_current_location(false)) {
            // 忽略失败
            // ignore failure
        }
    } else {
        if (!copter.set_home(cmd.content.location, false)) {
            // 忽略失败
            // ignore failure
        }
    }
}

// do_roi - 开始执行MAV_CMD_DO_SET_ROI所需的操作
//          这涉及将相机移动以指向ROI(感兴趣区域)
//          如果我们的云台类型不支持偏航功能,可能还需要旋转直升机以指向ROI
// TO-DO: 添加对MAV_CMD_DO_SET_ROI的其他功能的支持,包括指向给定航点
// do_roi - starts actions required by MAV_CMD_DO_SET_ROI
//          this involves either moving the camera to point at the ROI (region of interest)
//          and possibly rotating the copter to point at the ROI if our mount type does not support a yaw feature
// TO-DO: add support for other features of MAV_CMD_DO_SET_ROI including pointing at a given waypoint
void ModeAuto::do_roi(const AP_Mission::Mission_Command& cmd)
{
    auto_yaw.set_roi(cmd.content.location);
}

#if HAL_MOUNT_ENABLED
// 将相机指向指定角度
// point the camera to a specified angle
void ModeAuto::do_mount_control(const AP_Mission::Mission_Command& cmd)
{
    // 如果飞行器有相机云台但它不执行平移控制,那么就旋转整个飞行器
    // if vehicle has a camera mount but it doesn't do pan control then yaw the entire vehicle instead
    if ((copter.camera_mount.get_mount_type() != AP_Mount::Type::None) &&
        !copter.camera_mount.has_pan_control()) {
        auto_yaw.set_yaw_angle_rate(cmd.content.mount_control.yaw,0.0f);
    }
    // 将目标角度传递给相机云台
    // pass the target angles to the camera mount
    copter.camera_mount.set_angle_target(cmd.content.mount_control.roll, cmd.content.mount_control.pitch, cmd.content.mount_control.yaw, false);
}
#endif  // HAL_MOUNT_ENABLED

#if AP_WINCH_ENABLED
// 根据任务命令控制绞车
// control winch based on mission command
void ModeAuto::do_winch(const AP_Mission::Mission_Command& cmd)
{
    // 注意: 我们忽略了抓取器编号参数,因为我们只支持一个抓取器
    // Note: we ignore the gripper num parameter because we only support one gripper
    switch (cmd.content.winch.action) {
        case WINCH_RELAXED:
            g2.winch.relax();
            break;
        case WINCH_RELATIVE_LENGTH_CONTROL:
            g2.winch.release_length(cmd.content.winch.release_length);
            break;
        case WINCH_RATE_CONTROL:
            g2.winch.set_desired_rate(cmd.content.winch.release_rate);
            break;
        default:
            // 什么都不做
            // do nothing
            break;
    }
}
#endif

#if AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED && AC_PAYLOAD_PLACE_ENABLED
// do_payload_place - 初始化放置程序
// do_payload_place - initiate placing procedure
void ModeAuto::do_payload_place(const AP_Mission::Mission_Command& cmd)
{
    // 如果提供了位置,我们就以当前高度飞到那个位置
    // if location provided we fly to that location at current altitude
    if (cmd.content.location.lat != 0 || cmd.content.location.lng != 0) {
        // 设置状态为飞向位置
        // set state to fly to location
        payload_place.state = PayloadPlace::State::FlyToLocation;

        // 将命令转换为位置类
        // convert cmd to location class
        Location target_loc(cmd.content.location);
        // 尝试将高度调整为当前高度
        // 如果失败,则使用当前相对于家的高度,并报告错误
        if (!shift_alt_to_current_alt(target_loc)) {
            // 这只可能是由于缺少地形数据库高度或测距仪高度而失败
            // 使用当前相对于家的高度并报告错误
            // this can only fail due to missing terrain database alt or rangefinder alt
            // use current alt-above-home and report error
            target_loc.set_alt_cm(copter.current_loc.alt, Location::AltFrame::ABOVE_HOME);
            // 记录错误
            LOGGER_WRITE_ERROR(LogErrorSubsystem::TERRAIN, LogErrorCode::MISSING_TERRAIN_DATA);
            // 发送错误消息到地面站
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PayloadPlace: no terrain data, using alt-above-home");
        }
        // 尝试开始航点导航
        // 如果失败,则触发地形故障保护
        if (!wp_start(target_loc)) {
            // 设置下一个目的地失败只可能是由于缺少地形数据
            // failure to set next destination can only be because of missing terrain data
            copter.failsafe_terrain_on_event();
            return;
        }
    } else {
        // 初始化放置控制器
        // initialise placing controller
        payload_place.start_descent();
    }
    // 设置最大下降高度(厘米)
    payload_place.descent_max_cm = cmd.p1;

    // 设置子模式
    // set submode
    set_submode(SubMode::NAV_PAYLOAD_PLACE);
}
#endif  // AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED

// do_RTL - 开始返航
// do_RTL - start Return-to-Launch
void ModeAuto::do_RTL(void)
{
    // 在自动飞行模式下开始返航
    // start rtl in auto flight mode
    rtl_start();
}

/********************************************************************************/
// 验证导航(必须)命令
// Verify Nav (Must) commands
/********************************************************************************/

// verify_takeoff - 检查我们是否已完成起飞
// verify_takeoff - check if we have completed the takeoff
bool ModeAuto::verify_takeoff()
{
#if AP_LANDINGGEAR_ENABLED
    // 如果我们已经到达目的地
    // if we have reached our destination
    if (auto_takeoff.complete) {
        // 收起起落架
        // retract the landing gear
        copter.landinggear.retract_after_takeoff();
    }
#endif

    return auto_takeoff.complete;
}

// verify_land - 如果着陆已完成则返回true
// verify_land - returns true if landing has been completed
bool ModeAuto::verify_land()
{
    bool retval = false;

    switch (state) {
        case State::FlyToLocation:
            // 检查我们是否已到达位置
            // check if we've reached the location
            if (copter.wp_nav->reached_wp_destination()) {
                // 初始化着陆控制器
                // initialise landing controller
                land_start();

                // 进入下一个状态
                // advance to next state
                state = State::Descending;
            }
            break;

        case State::Descending:
            // 依靠THROTTLE_LAND模式正确更新着陆状态
            // rely on THROTTLE_LAND mode to correctly update landing status
            retval = copter.ap.land_complete && (motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE);
            if (retval && !mission.continue_after_land_check_for_takeoff() && copter.motors->armed()) {
                /*
                  我们希望在着陆完成时停止任务处理。
                  现在解除武装,然后返回false。
                  这将任务状态机保持在当前NAV_LAND任务项中。
                  解除武装后,任务将重置
                  we want to stop mission processing on land
                  completion. Disarm now, then return false. This
                  leaves mission state machine in the current NAV_LAND
                  mission item. After disarming the mission will reset
                */
                copter.arming.disarm(AP_Arming::Method::LANDED);
                retval = false;
            }
            break;

        default:
            // 这种情况不应该发生
            // this should never happen
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            retval = true;
            break;
    }

    // 如果我们成功着陆则返回true
    // true is returned if we've successfully landed
    return retval;
}

#if AC_PAYLOAD_PLACE_ENABLED
// verify_payload_place - 如果放置已完成则返回true
// verify_payload_place - returns true if placing has been completed
bool PayloadPlace::verify()
{
    switch (state) {
    case State::FlyToLocation:
    case State::Descent_Start:
    case State::Descent:
    case State::Release:
    case State::Releasing:
    case State::Delay:
    case State::Ascent_Start:
    case State::Ascent:
        return false;
    case State::Done:
        return true;
    }
    // 不应该到达这里
    // should never get here
    return true;
}
#endif

bool ModeAuto::verify_loiter_unlimited()
{
    return false;
}

// verify_loiter_time - 检查我们是否已经盘旋足够长的时间
// verify_loiter_time - check if we have loitered long enough
bool ModeAuto::verify_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    // 如果我们还没有到达目的地,立即返回
    // return immediately if we haven't reached our destination
    if (!copter.wp_nav->reached_wp_destination()) {
        return false;
    }

    // 启动我们的盘旋计时器
    // start our loiter timer
    if ( loiter_time == 0 ) {
        loiter_time = millis();
    }

    // 检查盘旋计时器是否已经用完
    // check if loiter timer has run out
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
        gcs().send_text(MAV_SEVERITY_INFO, "Reached command #%i",cmd.index);
        return true;
    }

    return false;
}

// verify_loiter_to_alt - 检查我们是否已经到达目的地(大致)和高度(精确)
// verify_loiter_to_alt - check if we have reached both destination
// (roughly) and altitude (precisely)
bool ModeAuto::verify_loiter_to_alt() const
{
    if (loiter_to_alt.reached_destination_xy &&
        loiter_to_alt.reached_alt) {
        return true;
    }
    return false;
}

// verify_RTL - 处理实现RTL所需的任何状态变化
// do_RTL应该先被调用一次以初始化所有变量
// 当RTL成功完成时返回true
// verify_RTL - handles any state changes required to implement RTL
// do_RTL should have been called once first to initialise all variables
// returns true with RTL has completed successfully
bool ModeAuto::verify_RTL()
{
    return (copter.mode_rtl.state_complete() && 
            (copter.mode_rtl.state() == ModeRTL::SubMode::FINAL_DESCENT || copter.mode_rtl.state() == ModeRTL::SubMode::LAND) &&
            (motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE));
}

/********************************************************************************/
// 验证条件(可能)命令
// Verify Condition (May) commands
/********************************************************************************/

bool ModeAuto::verify_wait_delay()
{
    if (millis() - condition_start > (uint32_t)MAX(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

bool ModeAuto::verify_within_distance()
{
    if (wp_distance() < (uint32_t)MAX(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

// verify_yaw - 如果我们已经到达所需的航向则返回true
// verify_yaw - return true if we have reached the desired heading
bool ModeAuto::verify_yaw()
{
    // 确保仍然处于固定偏航模式,航点控制器在执行新的航点命令时经常重新控制偏航
    // make sure still in fixed yaw mode, the waypoint controller often retakes control of yaw as it executes a new waypoint command
    auto_yaw.set_mode(AutoYaw::Mode::FIXED);

    // 检查我们是否已经到达目标航向
    // check if we have reached the target heading
    return auto_yaw.reached_fixed_yaw_target();
}

// verify_nav_wp - 检查我们是否已经到达下一个航点
// verify_nav_wp - check if we have reached the next way point
bool ModeAuto::verify_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    // 检查我们是否已经到达航点
    // check if we have reached the waypoint
    if ( !copter.wp_nav->reached_wp_destination() ) {
        return false;
    }

    // 如果需要,启动计时器
    // start timer if necessary
    if (loiter_time == 0) {
        loiter_time = millis();
        if (loiter_time_max > 0) {
            // 播放一个音调
            // play a tone
            AP_Notify::events.waypoint_complete = 1;
        }
    }

    // 检查计时器是否已经用完
    // check if timer has run out
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
        if (loiter_time_max == 0) {
            // 播放一个音调
            // play a tone
            AP_Notify::events.waypoint_complete = 1;
        }
        gcs().send_text(MAV_SEVERITY_INFO, "Reached command #%i",cmd.index);
        return true;
    }
    return false;
}

// verify_circle - 检查我们是否已经绕点足够多圈
// verify_circle - check if we have circled the point enough
bool ModeAuto::verify_circle(const AP_Mission::Mission_Command& cmd)
{
    // 检查我们是否已经到达边缘
    // check if we've reached the edge
    if (_mode == SubMode::CIRCLE_MOVE_TO_EDGE) {
        if (copter.wp_nav->reached_wp_destination()) {
            // 开始绕圈
            // start circling
            circle_start();
        }
        return false;
    }

    const float turns = cmd.get_loiter_turns();

    // 检查我们是否已经完成绕圈
    // check if we have completed circling
    return fabsf(copter.circle_nav->get_angle_total()/float(M_2PI)) >= turns;
}

// verify_spline_wp - 使用样条曲线检查我们是否已经到达下一个航点
// verify_spline_wp - check if we have reached the next way point using spline
bool ModeAuto::verify_spline_wp(const AP_Mission::Mission_Command& cmd)
{
    // 检查我们是否已经到达航点
    // check if we have reached the waypoint
    if ( !copter.wp_nav->reached_wp_destination() ) {
        return false;
    }

    // 如果需要,启动计时器
    // start timer if necessary
    if (loiter_time == 0) {
        loiter_time = millis();
    }

    // 检查计时器是否已经用完
    // check if timer has run out
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
        gcs().send_text(MAV_SEVERITY_INFO, "Reached command #%i",cmd.index);
        return true;
    }
    return false;
}

#if AC_NAV_GUIDED
// verify_nav_guided - 检查我们是否已经超出任何限制
// verify_nav_guided - check if we have breached any limits
bool ModeAuto::verify_nav_guided_enable(const AP_Mission::Mission_Command& cmd)
{
    // 如果禁用引导模式,则立即返回true,以便我们移动到下一个命令
    // if disabling guided mode then immediately return true so we move to next command
    if (cmd.p1 == 0) {
        return true;
    }

    // 检查时间和位置限制
    // check time and position limits
    return copter.mode_guided.limit_check();
}
#endif  // AC_NAV_GUIDED

// verify_nav_delay - 检查我们是否已经等待足够长的时间
// verify_nav_delay - check if we have waited long enough
bool ModeAuto::verify_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    if (millis() - nav_delay_time_start_ms > nav_delay_time_max_ms) {
        nav_delay_time_max_ms = 0;
        return true;
    }
    return false;
}

#if AP_SCRIPTING_ENABLED
// 检查verify_nav_script_time命令是否已完成
// check if verify_nav_script_time command has completed
bool ModeAuto::verify_nav_script_time()
{
    // 如果完成或超时则返回true
    // if done or timeout then return true
    if (nav_scripting.done ||
        ((nav_scripting.timeout_s > 0) &&
         (AP_HAL::millis() - nav_scripting.start_ms) > (nav_scripting.timeout_s * 1000))) {
        return true;
    }
    return false;
}
#endif

// 检查nav_attitude_time命令是否已完成
// check if nav_attitude_time command has completed
bool ModeAuto::verify_nav_attitude_time(const AP_Mission::Mission_Command& cmd)
{
    return ((AP_HAL::millis() - nav_attitude_time.start_ms) > (cmd.content.nav_attitude_time.time_sec * 1000));
}

// pause - 防止飞行器沿轨道前进
// pause - Prevent aircraft from progressing along the track
bool ModeAuto::pause()
{
    // 如果不在WP子模式或已经到达目的地,则不暂停
    // do not pause if not in the WP sub mode or already reached to the destination
    if (_mode != SubMode::WP || wp_nav->reached_wp_destination()) {
        return false;
    }

    wp_nav->set_pause();
    return true;
}

// resume - 允许飞行器沿轨道前进
// resume - Allow aircraft to progress along the track
bool ModeAuto::resume()
{
    wp_nav->set_resume();
    return true;
}

bool ModeAuto::paused() const
{
    return (wp_nav != nullptr) && wp_nav->paused();
}

#endif
