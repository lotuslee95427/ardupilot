#include "Copter.h"

#if MODE_RTL_ENABLED

/*
 * Init and run calls for RTL flight mode
 * RTL飞行模式的初始化和运行调用
 *
 * There are two parts to RTL, the high level decision making which controls which state we are in
 * and the lower implementation of the waypoint or landing controllers within those states
 * RTL分为两个部分:高层决策控制我们所处的状态,以及这些状态内的航点或着陆控制器的底层实现
 */

// rtl_init - initialise rtl controller
// rtl_init - 初始化RTL控制器
bool ModeRTL::init(bool ignore_checks)
{
    if (!ignore_checks) {
        if (!AP::ahrs().home_is_set()) {
            return false;
        }
    }
    // initialise waypoint and spline controller
    // 初始化航点和样条控制器
    wp_nav->wp_and_spline_init(g.rtl_speed_cms);
    _state = SubMode::STARTING;
    _state_complete = true; // see run() method below
                            // 参见下面的run()方法
    terrain_following_allowed = !copter.failsafe.terrain;
    // reset flag indicating if pilot has applied roll or pitch inputs during landing
    // 重置标志,指示飞行员在着陆过程中是否应用了横滚或俯仰输入
    copter.ap.land_repo_active = false;

    // this will be set true if prec land is later active
    // 如果精确着陆后来被激活,这将被设置为true
    copter.ap.prec_land_active = false;

#if AC_PRECLAND_ENABLED
    // initialise precland state machine
    // 初始化精确着陆状态机
    copter.precland_statemachine.init();
#endif

    return true;
}

// re-start RTL with terrain following disabled
// 重新启动RTL,禁用地形跟随
void ModeRTL::restart_without_terrain()
{
#if HAL_LOGGING_ENABLED
    LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::RESTARTED_RTL);
#endif
    terrain_following_allowed = false;
    _state = SubMode::STARTING;
    _state_complete = true;
    gcs().send_text(MAV_SEVERITY_CRITICAL,"Restarting RTL - Terrain data missing");
}

ModeRTL::RTLAltType ModeRTL::get_alt_type() const
{
    // sanity check parameter
    // 参数合理性检查
    switch ((ModeRTL::RTLAltType)g.rtl_alt_type) {
    case RTLAltType::RELATIVE ... RTLAltType::TERRAIN:
        return g.rtl_alt_type;
    }
    // user has an invalid value
    // 用户有一个无效值
    return RTLAltType::RELATIVE;
}

// rtl_run - runs the return-to-launch controller
// rtl_run - 运行返回起飞点控制器
// should be called at 100hz or more
// 应该以100Hz或更高的频率调用
void ModeRTL::run(bool disarm_on_land)
{
    if (!motors->armed()) {
        return;
    }

    // check if we need to move to next state
    // 检查是否需要移动到下一个状态
    if (_state_complete) {
        switch (_state) {
        case SubMode::STARTING:
            build_path();
            climb_start();
            break;
        case SubMode::INITIAL_CLIMB:
            return_start();
            break;
        case SubMode::RETURN_HOME:
            loiterathome_start();
            break;
        case SubMode::LOITER_AT_HOME:
            if (rtl_path.land || copter.failsafe.radio) {
                land_start();
            } else {
                descent_start();
            }
            break;
        case SubMode::FINAL_DESCENT:
            // do nothing
            break;
        case SubMode::LAND:
            // do nothing - rtl_land_run will take care of disarming motors
            // 什么都不做 - rtl_land_run将负责解除电机武装
            break;
        }
    }

    // call the correct run function
    // 调用正确的运行函数
    switch (_state) {

    case SubMode::STARTING:
        // should not be reached:
        // 不应该到达这里:
        _state = SubMode::INITIAL_CLIMB;
        FALLTHROUGH;

    case SubMode::INITIAL_CLIMB:
        climb_return_run();
        break;

    case SubMode::RETURN_HOME:
        climb_return_run();
        break;

    case SubMode::LOITER_AT_HOME:
        loiterathome_run();
        break;

    case SubMode::FINAL_DESCENT:
        descent_run();
        break;

    case SubMode::LAND:
        land_run(disarm_on_land);
        break;
    }
}

// rtl_climb_start - initialise climb to RTL altitude
// rtl_climb_start - 初始化爬升到RTL高度
void ModeRTL::climb_start()
{
    _state = SubMode::INITIAL_CLIMB;
    _state_complete = false;

    // set the destination
    // 设置目的地
    if (!wp_nav->set_wp_destination_loc(rtl_path.climb_target) || !wp_nav->set_wp_destination_next_loc(rtl_path.return_target)) {
        // this should not happen because rtl_build_path will have checked terrain data was available
        // 这不应该发生,因为rtl_build_path已经检查了地形数据是否可用
        gcs().send_text(MAV_SEVERITY_CRITICAL,"RTL: unexpected error setting climb target");
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
        copter.set_mode(Mode::Number::LAND, ModeReason::TERRAIN_FAILSAFE);
        return;
    }

    // hold current yaw during initial climb
    // 在初始爬升期间保持当前偏航
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);
}

// rtl_return_start - initialise return to home
// rtl_return_start - 初始化返回起飞点
void ModeRTL::return_start()
{
    _state = SubMode::RETURN_HOME;
    _state_complete = false;

    if (!wp_nav->set_wp_destination_loc(rtl_path.return_target)) {
        // failure must be caused by missing terrain data, restart RTL
        // 失败必须是由于缺少地形数据造成的,重新启动RTL
        restart_without_terrain();
    }

    // initialise yaw to point home (maybe)
    // 初始化偏航以指向起飞点(可能)
    auto_yaw.set_mode_to_default(true);
}

// rtl_climb_return_run - implements the initial climb, return home and descent portions of RTL which all rely on the wp controller
// rtl_climb_return_run - 实现RTL的初始爬升、返回起飞点和下降部分,这些都依赖于wp控制器
//      called by rtl_run at 100hz or more
//      由rtl_run以100Hz或更高的频率调用
void ModeRTL::climb_return_run()
{
    // if not armed set throttle to zero and exit immediately
    // 如果未武装,将油门设置为零并立即退出
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    // 将电机设置为全范围
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    // 运行航点控制器
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    // WP_Nav已设置垂直位置控制目标
    // 运行垂直位置控制器并设置输出油门
    pos_control->update_z_controller();

    // call attitude controller with auto yaw
    // 使用自动偏航调用姿态控制器
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());

    // check if we've completed this stage of RTL
    // 检查我们是否完成了RTL的这个阶段
    _state_complete = wp_nav->reached_wp_destination();
}

// loiterathome_start - initialise return to home
// loiterathome_start - 初始化返回起飞点
void ModeRTL::loiterathome_start()
{
    _state = SubMode::LOITER_AT_HOME;
    _state_complete = false;
    _loiter_start_time = millis();

    // yaw back to initial take-off heading yaw unless pilot has already overridden yaw
    // 除非飞行员已经覆盖了偏航,否则偏航回到初始起飞航向
    if (auto_yaw.default_mode(true) != AutoYaw::Mode::HOLD) {
        auto_yaw.set_mode(AutoYaw::Mode::RESETTOARMEDYAW);
    } else {
        auto_yaw.set_mode(AutoYaw::Mode::HOLD);
    }
}

// rtl_climb_return_descent_run - implements the initial climb, return home and descent portions of RTL which all rely on the wp controller
// rtl_climb_return_descent_run - 实现RTL的初始爬升、返回起飞点和下降部分,这些都依赖于wp控制器
//      called by rtl_run at 100hz or more
//      由rtl_run以100Hz或更高的频率调用
void ModeRTL::loiterathome_run()
{
    // if not armed set throttle to zero and exit immediately
    // 如果未武装,将油门设置为零并立即退出
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    // 将电机设置为全范围
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    // 运行航点控制器
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    // WP_Nav已设置垂直位置控制目标
    // 运行垂直位置控制器并设置输出油门
    pos_control->update_z_controller();

    // call attitude controller with auto yaw
    // 使用自动偏航调用姿态控制器
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());

    // check if we've completed this stage of RTL
    // 检查我们是否完成了RTL的这个阶段
    if ((millis() - _loiter_start_time) >= (uint32_t)g.rtl_loiter_time.get()) {
        if (auto_yaw.mode() == AutoYaw::Mode::RESETTOARMEDYAW) {
            // check if heading is within 2 degrees of heading when vehicle was armed
            // 检查航向是否在飞行器武装时的航向2度范围内
            if (abs(wrap_180_cd(ahrs.yaw_sensor-copter.initial_armed_bearing)) <= 200) {
                _state_complete = true;
            }
        } else {
            // we have loitered long enough
            // 我们已经盘旋足够长时间了
            _state_complete = true;
        }
    }
}

// rtl_descent_start - initialise descent to final alt
// rtl_descent_start - 初始化下降到最终高度
void ModeRTL::descent_start()
{
    _state = SubMode::FINAL_DESCENT;
    _state_complete = false;

    // initialise altitude target to stopping point
    // 初始化高度目标到停止点
    pos_control->init_z_controller_stopping_point();

    // initialise yaw
    // 初始化偏航
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

#if AP_LANDINGGEAR_ENABLED
    // optionally deploy landing gear
    // 可选择部署起落架
    copter.landinggear.deploy_for_landing();
#endif
}

// rtl_descent_run - implements the final descent to the RTL_ALT
// rtl_descent_run - 实现最终下降到RTL_ALT
//      called by rtl_run at 100hz or more
//      由rtl_run以100Hz或更高的频率调用
void ModeRTL::descent_run()
{
    Vector2f vel_correction;

    // if not armed set throttle to zero and exit immediately
    // 如果未武装,将油门设置为零并立即退出
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // process pilot's input
    // 处理飞行员的输入
    if (!copter.failsafe.radio) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            LOGGER_WRITE_EVENT(LogEvent::LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            // 如果油门高,退出着陆
            if (!copter.set_mode(Mode::Number::LOITER, ModeReason::THROTTLE_LAND_ESCAPE)) {
                copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::THROTTLE_LAND_ESCAPE);
            }
        }

        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            // 对飞行员输入应用SIMPLE模式转换
            update_simple_mode();

            // convert pilot input to reposition velocity
            // 将飞行员输入转换为重新定位速度
            vel_correction = get_pilot_desired_velocity(wp_nav->get_wp_acceleration() * 0.5);

            // record if pilot has overridden roll or pitch
            // 记录飞行员是否覆盖了横滚或俯仰
            if (!vel_correction.is_zero()) {
                if (!copter.ap.land_repo_active) {
                    LOGGER_WRITE_EVENT(LogEvent::LAND_REPO_ACTIVE);
                }
                copter.ap.land_repo_active = true;
            }
        }
    }

    // set motors to full range
    // 将电机设置为全范围
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    Vector2f accel;
    pos_control->input_vel_accel_xy(vel_correction, accel);
    pos_control->update_xy_controller();

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    // WP_Nav已设置垂直位置控制目标
    // 运行垂直位置控制器并设置输出油门
    pos_control->set_alt_target_with_slew(rtl_path.descent_target.alt);
    pos_control->update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    // 横滚和俯仰来自航点控制器,偏航速率来自飞行员
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());

    // check if we've reached within 20cm of final altitude
    // 检查我们是否已经到达最终高度的20cm范围内
    _state_complete = labs(rtl_path.descent_target.alt - copter.current_loc.alt) < 20;
}

// land_start - 初始化控制器以在家的位置上方悬停
void ModeRTL::land_start()
{
    _state = SubMode::LAND;
    _state_complete = false;

    // 设置水平速度和加速度限制
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // 初始化悬停目标位置
    if (!pos_control->is_active_xy()) {
        pos_control->init_xy_controller();
    }

    // 初始化垂直位置控制器
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // 初始化偏航
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

#if AP_LANDINGGEAR_ENABLED
    // 可选择部署起落架
    copter.landinggear.deploy_for_landing();
#endif
}

// 判断是否正在着陆
bool ModeRTL::is_landing() const
{
    return _state == SubMode::LAND;
}

// land_run - 运行着陆控制器以使飞机着陆
// 由rtl_run以100Hz或更高的频率调用
void ModeRTL::land_run(bool disarm_on_land)
{
    // 检查我们是否已完成RTL的这个阶段
    _state_complete = copter.ap.land_complete;

    // 当着陆检测器表示我们已着陆时解除武装
    if (disarm_on_land && copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }

    // 如果未武装，将油门设置为零并立即退出
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // 将电机设置为全范围
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // 运行正常着陆或精确着陆（如果启用）
    land_run_normal_or_precland();
}

// 构建RTL路径
void ModeRTL::build_path()
{
    // 原点是我们的停止点
    rtl_path.origin_point = get_stopping_point();
    rtl_path.origin_point.change_alt_frame(Location::AltFrame::ABOVE_HOME);

    // 计算返回目标
    compute_return_target();

    // 爬升目标是在返回高度上方的原点
    rtl_path.climb_target = Location(rtl_path.origin_point.lat, rtl_path.origin_point.lng, rtl_path.return_target.alt, rtl_path.return_target.get_alt_frame());

    // 下降目标是在rtl_alt_final高度的返回目标下方
    rtl_path.descent_target = Location(rtl_path.return_target.lat, rtl_path.return_target.lng, g.rtl_alt_final, Location::AltFrame::ABOVE_HOME);

    // 设置着陆标志
    rtl_path.land = g.rtl_alt_final <= 0;
}

// 计算返回目标 - 家或集结点
// 返回目标的高度更新为飞机可以安全返回的更高高度（框架也可能被设置）
void ModeRTL::compute_return_target()
{
    // 将返回目标设置为最近的集结点或家的位置
#if HAL_RALLY_ENABLED
    rtl_path.return_target = copter.rally.calc_best_rally_or_home_location(copter.current_loc, ahrs.get_home().alt);
    rtl_path.return_target.change_alt_frame(Location::AltFrame::ABSOLUTE);
#else
    rtl_path.return_target = ahrs.get_home();
#endif

    // curr_alt是当前高于家或高于地形的高度，取决于use_terrain
    int32_t curr_alt = copter.current_loc.alt;

    // 确定返回旅程的高度类型（高于家的高度、使用测距仪的高于地形的高度或使用地形数据库的高于地形的高度）
    ReturnTargetAltType alt_type = ReturnTargetAltType::RELATIVE;
    if (terrain_following_allowed && (get_alt_type() == RTLAltType::TERRAIN)) {
        // 将RTL_ALT_TYPE和WPNAV_RFNG_USE参数转换为ReturnTargetAltType
        switch (wp_nav->get_terrain_source()) {
        case AC_WPNav::TerrainSource::TERRAIN_UNAVAILABLE:
            alt_type = ReturnTargetAltType::RELATIVE;
            LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::RTL_MISSING_RNGFND);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RTL: no terrain data, using alt-above-home");
            break;
        case AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER:
            alt_type = ReturnTargetAltType::RANGEFINDER;
            break;
        case AC_WPNav::TerrainSource::TERRAIN_FROM_TERRAINDATABASE:
            alt_type = ReturnTargetAltType::TERRAINDATABASE;
            break;
        }
    }

    // 从测距仪设置curr_alt和return_target.alt
    if (alt_type == ReturnTargetAltType::RANGEFINDER) {
        if (copter.get_rangefinder_height_interpolated_cm(curr_alt)) {
            // 设置return_target.alt
            rtl_path.return_target.set_alt_cm(MAX(curr_alt + MAX(0, g.rtl_climb_min), MAX(g.rtl_altitude, RTL_ALT_MIN)), Location::AltFrame::ABOVE_TERRAIN);
        } else {
            // 回退到相对高度并警告用户
            alt_type = ReturnTargetAltType::RELATIVE;
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RTL: rangefinder unhealthy, using alt-above-home");
            LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::RTL_MISSING_RNGFND);
        }
    }

    // 从地形数据库设置curr_alt和return_target.alt
    if (alt_type == ReturnTargetAltType::TERRAINDATABASE) {
        // 将curr_alt设置为当前高于地形的高度
        // 将return_target.alt从绝对高度（高于MSL）转换为高于地形的高度
        // 注意：return_target可能是一个集结点，其高度设置在地形高度之上（如建筑物顶部）
        int32_t curr_terr_alt;
        if (copter.current_loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, curr_terr_alt) &&
            rtl_path.return_target.change_alt_frame(Location::AltFrame::ABOVE_TERRAIN)) {
            curr_alt = curr_terr_alt;
        } else {
            // 回退到相对高度并警告用户
            alt_type = ReturnTargetAltType::RELATIVE;
            LOGGER_WRITE_ERROR(LogErrorSubsystem::TERRAIN, LogErrorCode::MISSING_TERRAIN_DATA);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RTL: no terrain data, using alt-above-home");
        }
    }

    // 对于默认情况，我们必须将return-target alt（绝对高度）转换为高于家的高度
    if (alt_type == ReturnTargetAltType::RELATIVE) {
        if (!rtl_path.return_target.change_alt_frame(Location::AltFrame::ABOVE_HOME)) {
            // 这种情况不应该发生，但以防万一
            rtl_path.return_target.set_alt_cm(0, Location::AltFrame::ABOVE_HOME);
            gcs().send_text(MAV_SEVERITY_WARNING, "RTL: unexpected error calculating target alt");
        }
    }

    // 将新的目标高度设置为返回目标高度
    // 注意：这是高于家或地形的高度，取决于rtl_alt_type
    // 注意：忽略负高度，这可能发生在用户为集结点输入负高度或集结点处的地形比家高的情况下
    int32_t target_alt = MAX(rtl_path.return_target.alt, 0);

    // 将目标增加到当前高度 + climb_min和rtl高度的最大值
    const float min_rtl_alt = MAX(RTL_ALT_MIN, curr_alt + MAX(0, g.rtl_climb_min));
    target_alt = MAX(target_alt, MAX(g.rtl_altitude, min_rtl_alt));

    // 如果接近返回目标，减少爬升
    float rtl_return_dist_cm = rtl_path.return_target.get_distance(rtl_path.origin_point) * 100.0f;
    // 不允许非常浅的斜坡
    if (g.rtl_cone_slope >= RTL_MIN_CONE_SLOPE) {
        target_alt = MIN(target_alt, MAX(rtl_return_dist_cm * g.rtl_cone_slope, min_rtl_alt));
    }

    // 将返回的目标高度设置为新的target_alt（不改变高度类型）
    rtl_path.return_target.set_alt_cm(target_alt, (alt_type == ReturnTargetAltType::RELATIVE) ? Location::AltFrame::ABOVE_HOME : Location::AltFrame::ABOVE_TERRAIN);

#if AP_FENCE_ENABLED
    // 如果启用了高度围栏，确保不超过围栏高度
    // 注意：因为rtl_path.climb_target的高度简单地从return_target的高度复制，
    //       如果使用地形高度，下面减少return_target高度的代码可能导致
    //       车辆在RTL开始时根本不爬升。这可能过于保守，最好
    //       独立地在origin_point和return_target上应用围栏高度限制
    if ((copter.fence.get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) != 0) {
        // 获取返回目标作为高于家的高度，以便可以与围栏的高度进行比较
        if (rtl_path.return_target.get_alt_cm(Location::AltFrame::ABOVE_HOME, target_alt)) {
            float fence_alt = copter.fence.get_safe_alt_max()*100.0f;
            if (target_alt > fence_alt) {
                // 将目标高度减少到围栏高度
                rtl_path.return_target.alt -= (target_alt - fence_alt);
            }
        }
    }
#endif

    // 确保我们不下降
    rtl_path.return_target.alt = MAX(rtl_path.return_target.alt, curr_alt);
}

// 获取航点位置
bool ModeRTL::get_wp(Location& destination) const
{
    // 在使用wp_nav的状态中提供目标
    switch (_state) {
    case SubMode::STARTING:
    case SubMode::INITIAL_CLIMB:
    case SubMode::RETURN_HOME:
    case SubMode::LOITER_AT_HOME:
    case SubMode::FINAL_DESCENT:
        return wp_nav->get_oa_wp_destination(destination);
    case SubMode::LAND:
        return false;
    }

    // 我们不应该到达这里，但以防万一
    return false;
}

// 获取到目标航点的距离
uint32_t ModeRTL::wp_distance() const
{
    return wp_nav->get_wp_distance_to_destination();
}

// 获取到目标航点的方位角
int32_t ModeRTL::wp_bearing() const
{
    return wp_nav->get_wp_bearing_to_destination();
}

// 返回是否应使用飞行员的偏航输入来调整飞机的航向
bool ModeRTL::use_pilot_yaw(void) const
{
    const bool allow_yaw_option = (copter.g2.rtl_options.get() & uint32_t(Options::IgnorePilotYaw)) == 0;
    const bool land_repositioning = g.land_repositioning && (_state == SubMode::FINAL_DESCENT);
    const bool final_landing = _state == SubMode::LAND;
    return allow_yaw_option || land_repositioning || final_landing;
}

// 设置水平速度
bool ModeRTL::set_speed_xy(float speed_xy_cms)
{
    copter.wp_nav->set_speed_xy(speed_xy_cms);
    return true;
}

// 设置上升速度
bool ModeRTL::set_speed_up(float speed_up_cms)
{
    copter.wp_nav->set_speed_up(speed_up_cms);
    return true;
}

// 设置下降速度
bool ModeRTL::set_speed_down(float speed_down_cms)
{
    copter.wp_nav->set_speed_down(speed_down_cms);
    return true;
}

#endif
