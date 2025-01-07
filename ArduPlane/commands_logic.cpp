#include "Plane.h"

/********************************************************************************/
// 命令事件处理器
/********************************************************************************/
bool Plane::start_command(const AP_Mission::Mission_Command& cmd)
{
    // 默认为非VTOL悬停
    auto_state.vtol_loiter = false;

#if AP_TERRAIN_AVAILABLE
    // 重置地形跟随状态
    plane.target_altitude.terrain_following_pending = false;
#endif

#if HAL_LOGGING_ENABLED
    // 记录新命令的开始
    if (should_log(MASK_LOG_CMD)) {
        logger.Write_Mission_Cmd(mission, cmd);
    }
#endif

    // 导航命令和非导航命令的特殊处理
    if (AP_Mission::is_nav_cmd(cmd)) {
        // 设置起飞完成为真，除非在起飞过程中，否则不添加额外的升降舵
        auto_state.takeoff_complete = true;

        // 标记导航控制器数据为过时
        nav_controller->set_data_is_stale();

        // 开始非空闲状态
        auto_state.idle_mode = false;
        
        // 重置悬停开始时间，新命令是新的悬停
        loiter.start_time_ms = 0;

        // 检查下一个导航命令
        AP_Mission::Mission_Command next_nav_cmd;
        const uint16_t next_index = mission.get_current_nav_index() + 1;
        const bool have_next_cmd = mission.get_next_nav_cmd(next_index, next_nav_cmd);
        auto_state.wp_is_land_approach = have_next_cmd && (next_nav_cmd.id == MAV_CMD_NAV_LAND);
#if HAL_QUADPLANE_ENABLED
        // 如果下一个命令是VTOL着陆，则不是固定翼着陆进近
        if (have_next_cmd && quadplane.is_vtol_land(next_nav_cmd.id)) {
            auto_state.wp_is_land_approach = false;
        }
#endif
    }

    // 根据命令ID执行相应操作
    switch(cmd.id) {

    case MAV_CMD_NAV_TAKEOFF:
        // 重置坠毁状态
        crash_state.is_crashed = false;
#if HAL_QUADPLANE_ENABLED
        // 如果是VTOL起飞，执行VTOL起飞
        if (quadplane.is_vtol_takeoff(cmd.id)) {
            return quadplane.do_vtol_takeoff(cmd);
        }
#endif
        do_takeoff(cmd);
        break;

    case MAV_CMD_NAV_WAYPOINT:                  // 导航到航点
        do_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_LAND:              // 着陆到航点
#if HAL_QUADPLANE_ENABLED
        // 如果是VTOL着陆，执行VTOL着陆
        if (quadplane.is_vtol_land(cmd.id)) {
            crash_state.is_crashed = false;
            return quadplane.do_vtol_land(cmd);            
        }
#endif
        do_land(cmd);
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:              // 无限期悬停
        do_loiter_unlimited(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TURNS:              // 悬停N圈
        do_loiter_turns(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TIME:               // 悬停指定时间
        do_loiter_time(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TO_ALT:             // 悬停到指定高度
        do_loiter_to_alt(cmd);
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:          // 返回起飞点
        set_mode(mode_rtl, ModeReason::MISSION_CMD);
        break;

    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:   // 继续当前路径并改变高度
        do_continue_and_change_alt(cmd);
        break;

    case MAV_CMD_NAV_ALTITUDE_WAIT:             // 等待高度
        do_altitude_wait(cmd);
        break;

#if HAL_QUADPLANE_ENABLED
    case MAV_CMD_NAV_VTOL_TAKEOFF:              // VTOL起飞
        crash_state.is_crashed = false;
        return quadplane.do_vtol_takeoff(cmd);

    case MAV_CMD_NAV_VTOL_LAND:                 // VTOL着陆
    case MAV_CMD_NAV_PAYLOAD_PLACE:             // 投放有效载荷
        if (quadplane.landing_with_fixed_wing_spiral_approach()) {
            // 使用固定翼螺旋进近方式进行VTOL着陆
            do_landing_vtol_approach(cmd);
            break;
        } else {
            return quadplane.do_vtol_land(cmd);
        }
#endif

    // 条件命令

    case MAV_CMD_CONDITION_DELAY:               // 延迟
        do_wait_delay(cmd);
        break;

    case MAV_CMD_CONDITION_DISTANCE:            // 等待距离
        do_within_distance(cmd);
        break;

    // 执行命令

    case MAV_CMD_DO_CHANGE_SPEED:               // 改变速度
        do_change_speed(cmd);
        break;

    case MAV_CMD_DO_SET_HOME:                   // 设置Home点
        do_set_home(cmd);
        break;

    case MAV_CMD_DO_INVERTED_FLIGHT:            // 倒飞
        if (cmd.p1 == 0 || cmd.p1 == 1) {
            auto_state.inverted_flight = (bool)cmd.p1;
            gcs().send_text(MAV_SEVERITY_INFO, "Set inverted %u", cmd.p1);
        }
        break;

    case MAV_CMD_DO_LAND_START:                 // 开始着陆
        break;

    case MAV_CMD_DO_AUTOTUNE_ENABLE:            // 启用自动调谐
        autotune_enable(cmd.p1);
        break;

#if HAL_MOUNT_ENABLED
    case MAV_CMD_DO_SET_ROI:                    // 设置兴趣区域
        if (cmd.content.location.alt == 0 && cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
            // 如果启用了相机跟踪，则关闭
            if (camera_mount.get_mode() == MAV_MOUNT_MODE_GPS_POINT) {
                camera_mount.set_mode_to_default();
            }
        } else {
            // 设置云台的目标位置
            camera_mount.set_roi_target(cmd.content.location);
        }
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:              // 控制云台
        // 将相机指向指定角度
        camera_mount.set_angle_target(cmd.content.mount_control.roll, cmd.content.mount_control.pitch, cmd.content.mount_control.yaw, false);
        break;
#endif

#if HAL_QUADPLANE_ENABLED
    case MAV_CMD_DO_VTOL_TRANSITION:            // VTOL转换
        plane.quadplane.handle_do_vtol_transition((enum MAV_VTOL_STATE)cmd.content.do_vtol_transition.target_state);
        break;
#endif

#if AP_ICENGINE_ENABLED
    case MAV_CMD_DO_ENGINE_CONTROL:             // 发动机控制
        plane.g2.ice_control.engine_control(cmd.content.do_engine_control.start_control,
                                            cmd.content.do_engine_control.cold_start,
                                            cmd.content.do_engine_control.height_delay_cm*0.01f,
                                            cmd.content.do_engine_control.allow_disarmed_start);
        break;
#endif

#if AP_SCRIPTING_ENABLED
    case MAV_CMD_NAV_SCRIPT_TIME:               // 脚本时间导航
        do_nav_script_time(cmd);
        break;
#endif

    case MAV_CMD_NAV_DELAY:                     // 导航延迟
        mode_auto.do_nav_delay(cmd);
        break;
        
    default:
        // 无法使用该命令，允许飞行器尝试下一个命令
        return false;
    }

    return true;
}
/*******************************************************************************
验证命令处理程序

每种任务元素都有一个"验证"操作。验证操作在任务元素完成时返回true,
我们应该继续执行下一个任务元素。
如果我们不识别该命令,则返回true,以便我们继续执行下一个命令。
*******************************************************************************/

bool Plane::verify_command(const AP_Mission::Mission_Command& cmd)        // 返回true表示命令完成
{
    switch(cmd.id) {

    case MAV_CMD_NAV_TAKEOFF:
#if HAL_QUADPLANE_ENABLED
        if (quadplane.is_vtol_takeoff(cmd.id)) {
            return quadplane.verify_vtol_takeoff(cmd);
        }
#endif
        return verify_takeoff();

    case MAV_CMD_NAV_WAYPOINT:
        return verify_nav_wp(cmd);

    case MAV_CMD_NAV_LAND:
#if HAL_QUADPLANE_ENABLED
        if (quadplane.is_vtol_land(cmd.id)) {
            return quadplane.verify_vtol_land();
        }
#endif
        if (flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING) {
            return landing.verify_abort_landing(prev_WP_loc, next_WP_loc, current_loc, auto_state.takeoff_altitude_rel_cm, throttle_suppressed);

        } else {
            // 使用测距仪进行修正（如果可能）
            bool rangefinder_active = false;
            float height = plane.get_landing_height(rangefinder_active);

            // 对于襟翼计算，我们不想使用地形修正
            // 否则在上升地形上会过早襟翼
            height -= auto_state.terrain_correction;
            return landing.verify_land(prev_WP_loc, next_WP_loc, current_loc,
                                       height, auto_state.sink_rate, auto_state.wp_proportion, auto_state.last_flying_ms, arming.is_armed(), is_flying(),
                                       rangefinder_active);
        }

    case MAV_CMD_NAV_LOITER_UNLIM:
        return verify_loiter_unlim(cmd);

    case MAV_CMD_NAV_LOITER_TURNS:
        return verify_loiter_turns(cmd);

    case MAV_CMD_NAV_LOITER_TIME:
        return verify_loiter_time();

    case MAV_CMD_NAV_LOITER_TO_ALT:
        return verify_loiter_to_alt(cmd);

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return verify_RTL();

    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
        return verify_continue_and_change_alt();

    case MAV_CMD_NAV_ALTITUDE_WAIT:
        return mode_auto.verify_altitude_wait(cmd);

#if HAL_QUADPLANE_ENABLED
    case MAV_CMD_NAV_VTOL_TAKEOFF:
        return quadplane.verify_vtol_takeoff(cmd);
    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_PAYLOAD_PLACE:
        if (quadplane.landing_with_fixed_wing_spiral_approach() && !verify_landing_vtol_approach(cmd)) {
            // verify_landing_vtol_approach 在完成接近时返回true
            // 在这种情况下，我们转到正常的vtol着陆代码
            return false;
        } else {
            return quadplane.verify_vtol_land();
        }
#endif  // HAL_QUADPLANE_ENABLED

    // 条件命令

    case MAV_CMD_CONDITION_DELAY:
        return verify_wait_delay();

    case MAV_CMD_CONDITION_DISTANCE:
        return verify_within_distance();

#if AP_SCRIPTING_ENABLED
    case MAV_CMD_NAV_SCRIPT_TIME:
        return verify_nav_script_time(cmd);
#endif

     case MAV_CMD_NAV_DELAY:
         return mode_auto.verify_nav_delay(cmd);

    // do 命令（总是返回true）
    case MAV_CMD_DO_CHANGE_SPEED:
    case MAV_CMD_DO_SET_HOME:
    case MAV_CMD_DO_INVERTED_FLIGHT:
    case MAV_CMD_DO_LAND_START:
    case MAV_CMD_DO_FENCE_ENABLE:
    case MAV_CMD_DO_AUTOTUNE_ENABLE:
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
    case MAV_CMD_DO_SET_ROI:
    case MAV_CMD_DO_MOUNT_CONTROL:
    case MAV_CMD_DO_VTOL_TRANSITION:
    case MAV_CMD_DO_ENGINE_CONTROL:
        return true;

    default:
        // 错误消息
        gcs().send_text(MAV_SEVERITY_WARNING,"跳过无效命令 #%i",cmd.id);
        // 如果我们不识别该命令，则返回true以便我们继续执行下一个命令
        return true;
    }
}

/********************************************************************************/
//  导航（必须）命令
/********************************************************************************/

void Plane::do_RTL(int32_t rtl_altitude_AMSL_cm)
{
    auto_state.next_wp_crosstrack = false;
    auto_state.crosstrack = false;
    prev_WP_loc = current_loc;
    next_WP_loc = calc_best_rally_or_home_location(current_loc, rtl_altitude_AMSL_cm);
    setup_terrain_target_alt(next_WP_loc);
    set_target_altitude_location(next_WP_loc);

    if (aparm.loiter_radius < 0) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }

    setup_glide_slope();
    setup_turn_angle();
}

Location Plane::calc_best_rally_or_home_location(const Location &_current_loc, float rtl_home_alt_amsl_cm) const
{
#if HAL_RALLY_ENABLED
    return plane.rally.calc_best_rally_or_home_location(_current_loc, rtl_home_alt_amsl_cm);
#else
    Location destination = plane.home;
    destination.set_alt_cm(rtl_home_alt_amsl_cm, Location::AltFrame::ABSOLUTE);
    return destination;
#endif
}

/*
  开始一个NAV_TAKEOFF命令
 */
void Plane::do_takeoff(const AP_Mission::Mission_Command& cmd)
{
    prev_WP_loc = current_loc;
    set_next_WP(cmd.content.location);
    // 俯仰角（度），空速（m/s），油门（%），跟踪航点 1 或 0
    auto_state.takeoff_pitch_cd        = (int16_t)cmd.p1 * 100;
    if (auto_state.takeoff_pitch_cd <= 0) {
        // 如果任务没有指定俯仰角，则使用4度
        auto_state.takeoff_pitch_cd = 400;
    }
    auto_state.takeoff_altitude_rel_cm = next_WP_loc.alt - home.alt;
    next_WP_loc.lat = home.lat + 10;
    next_WP_loc.lng = home.lng + 10;
    auto_state.takeoff_speed_time_ms = 0;
    auto_state.takeoff_complete = false; // 设置标志以在起飞期间使用GPS地面航向。IMU将进行偏航漂移校正。
    auto_state.height_below_takeoff_to_level_off_cm = 0;
    // 该标志还用于覆盖"在地面上"的油门禁用

    // 将锁定航向清零
    steer_state.locked_course_err = 0;
    steer_state.hold_course_cd = -1;
    auto_state.baro_takeoff_alt = barometer.get_altitude();
}

void Plane::do_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    set_next_WP(cmd.content.location);
}

void Plane::do_land(const AP_Mission::Mission_Command& cmd)
{
    set_next_WP(cmd.content.location);

    // 配置中止高度和俯仰角
    // 如果NAV_LAND有中止高度则使用它，否则使用上次起飞高度，如果没有则使用50m
    if (cmd.p1 > 0) {
        auto_state.takeoff_altitude_rel_cm = (int16_t)cmd.p1 * 100;
    } else if (auto_state.takeoff_altitude_rel_cm <= 0) {
        auto_state.takeoff_altitude_rel_cm = 3000;
    }

    if (auto_state.takeoff_pitch_cd <= 0) {
        // 如果从未使用过起飞命令，默认使用保守的10度
        auto_state.takeoff_pitch_cd = 1000;
    }

#if AP_RANGEFINDER_ENABLED
    // 将测距仪状态清零，开始累积好的样本
    memset(&rangefinder_state, 0, sizeof(rangefinder_state));
#endif

    landing.do_land(cmd, relative_altitude);

    if (flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING) {
        // 如果我们处于中止状态，我们需要显式地移出中止状态，因为它是粘性的
        set_flight_stage(AP_FixedWing::FlightStage::LAND);
    }
}

#if HAL_QUADPLANE_ENABLED
void Plane::do_landing_vtol_approach(const AP_Mission::Mission_Command& cmd)
{
    //设置目标高度
    Location loc = cmd.content.location;
    loc.sanitize(current_loc);
    set_next_WP(loc);

    vtol_approach_s.approach_stage = VTOLApproach::Stage::LOITER_TO_ALT;
}
#endif
// 设置盘旋方向
void Plane::loiter_set_direction_wp(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.content.location.loiter_ccw) {
        loiter.direction = -1; // 逆时针
    } else {
        loiter.direction = 1;  // 顺时针
    }
}

// 执行无限盘旋命令
void Plane::do_loiter_unlimited(const AP_Mission::Mission_Command& cmd)
{
    Location cmdloc = cmd.content.location;
    cmdloc.sanitize(current_loc); // 确保位置有效
    set_next_WP(cmdloc); // 设置下一个航点
    loiter_set_direction_wp(cmd); // 设置盘旋方向
}

// 执行指定圈数的盘旋命令
void Plane::do_loiter_turns(const AP_Mission::Mission_Command& cmd)
{
    Location cmdloc = cmd.content.location;
    cmdloc.sanitize(current_loc);
    set_next_WP(cmdloc);
    loiter_set_direction_wp(cmd);
    const float turns = cmd.get_loiter_turns();

    loiter.total_cd = (uint32_t)(turns * 36000UL); // 计算总旋转角度（厘度）
    condition_value = 1; // 用于表示主要圈数目标尚未达成
}

// 执行指定时间的盘旋命令
void Plane::do_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    Location cmdloc = cmd.content.location;
    cmdloc.sanitize(current_loc);
    set_next_WP(cmdloc);
    loiter_set_direction_wp(cmd);

    // 到达航点时设置起始时间
    loiter.time_max_ms = cmd.p1 * (uint32_t)1000;     // 将秒转换为毫秒
    condition_value = 1; // 用于表示主要时间目标尚未达成
}

// 执行继续飞行并改变高度的命令
void Plane::do_continue_and_change_alt(const AP_Mission::Mission_Command& cmd)
{
    // 选择航向方法：任务、GPS方位投影或偏航角
    float bearing;
    if (!prev_WP_loc.same_latlon_as(next_WP_loc)) {
        // 使用基于航点的方位，这是通常情况
        steer_state.hold_course_cd = -1;
    } else if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
        // 使用基于GPS地面航向的方位保持
        steer_state.hold_course_cd = -1;
        bearing = AP::gps().ground_course();
        next_WP_loc.offset_bearing(bearing, 1000); // 将其推出1公里
    } else {
        // 使用基于偏航角的方位保持
        steer_state.hold_course_cd = wrap_360_cd(ahrs.yaw_sensor);
        bearing = ahrs.yaw_sensor * 0.01f;
        next_WP_loc.offset_bearing(bearing, 1000); // 将其推出1公里
    }

    next_WP_loc.alt = cmd.content.location.alt + home.alt;
    condition_value = cmd.p1;
    reset_offset_altitude();
}

// 执行等待高度命令
void Plane::do_altitude_wait(const AP_Mission::Mission_Command& cmd)
{
    // 在达到高度或下降速度之前，将所有舵机设置为微调
    auto_state.idle_mode = true;
#if AP_PLANE_GLIDER_PULLUP_ENABLED
    mode_auto.pullup.reset();
#endif
}

// 执行盘旋到指定高度的命令
void Plane::do_loiter_to_alt(const AP_Mission::Mission_Command& cmd)
{
    // 设置目标高度
    Location loc = cmd.content.location;
    loc.sanitize(current_loc);
    set_next_WP(loc);
    loiter_set_direction_wp(cmd);

    // 初始化为0，达到高度时设置为1
    condition_value = 0;
}

// 执行导航延迟命令
void ModeAuto::do_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    nav_delay.time_start_ms = millis();

    if (cmd.content.nav_delay.seconds > 0) {
        // 相对延迟
        nav_delay.time_max_ms = cmd.content.nav_delay.seconds * 1000; // 将秒转换为毫秒
    } else {
        // 绝对延迟到UTC时间
#if AP_RTC_ENABLED
        nav_delay.time_max_ms = AP::rtc().get_time_utc(cmd.content.nav_delay.hour_utc, cmd.content.nav_delay.min_utc, cmd.content.nav_delay.sec_utc, 0);
#else
        nav_delay.time_max_ms = 0;
#endif
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Delaying %u sec", (unsigned)(nav_delay.time_max_ms/1000));
}

/********************************************************************************/
//  验证导航（必须）命令
/********************************************************************************/
// 验证起飞是否完成
bool Plane::verify_takeoff()
{
    bool trust_ahrs_yaw = AP::ahrs().initialised();
#if AP_AHRS_DCM_ENABLED
    trust_ahrs_yaw |= ahrs.dcm_yaw_initialised();
#endif
    if (trust_ahrs_yaw && steer_state.hold_course_cd == -1) {
        const float min_gps_speed = 5;
        if (auto_state.takeoff_speed_time_ms == 0 && 
            gps.status() >= AP_GPS::GPS_OK_FIX_3D && 
            gps.ground_speed() > min_gps_speed &&
            hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
            auto_state.takeoff_speed_time_ms = millis();
        }
        if (auto_state.takeoff_speed_time_ms != 0 &&
            millis() - auto_state.takeoff_speed_time_ms >= 2000) {
            // 一旦我们达到足够的速度以获得良好的GPS航向估计
            // 我们保存当前的GPS地面航向，并根据累积的偏航角进行校正
            // 以设置起飞航向。这保持机翼水平直到我们准备好旋转，
            // 并且还允许我们应对自动起飞时的任意罗盘误差
            float takeoff_course = wrap_PI(radians(gps.ground_course())) - steer_state.locked_course_err;
            takeoff_course = wrap_PI(takeoff_course);
            steer_state.hold_course_cd = wrap_360_cd(degrees(takeoff_course)*100);
            gcs().send_text(MAV_SEVERITY_INFO, "Holding course %d at %.1fm/s (%.1f)",
                              (int)steer_state.hold_course_cd,
                              (double)gps.ground_speed(),
                              (double)degrees(steer_state.locked_course_err));
        }
    }

    if (steer_state.hold_course_cd != -1) {
        // 调用导航控制器进行航向保持
        nav_controller->update_heading_hold(steer_state.hold_course_cd);
    } else {
        nav_controller->update_level_flight();        
    }

    // 检查可选的起飞超时
    if (plane.check_takeoff_timeout()) {
        mission.reset();
    }

    // 检查是否达到起飞高度
    int32_t relative_alt_cm = adjusted_relative_altitude_cm();
    if (relative_alt_cm > auto_state.takeoff_altitude_rel_cm) {
        gcs().send_text(MAV_SEVERITY_INFO, "Takeoff complete at %.2fm",
                          (double)(relative_alt_cm*0.01f));
        steer_state.hold_course_cd = -1;
        auto_state.takeoff_complete = true;
        next_WP_loc = prev_WP_loc = current_loc;

#if AP_FENCE_ENABLED
        plane.fence.auto_enable_fence_after_takeoff();
#endif

        // 起飞完成时不进行横向跟踪，否则可能会导致转弯过于急剧
        auto_state.next_wp_crosstrack = false;
        return true;
    } else {
        return false;
    }
}

// 更新正常任务航点的导航。当航点完成时返回true
bool Plane::verify_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    steer_state.hold_course_cd = -1;

    // 根据通过标志，以常规方式前往航点或沿航点线飞行设定距离
    Location flex_next_WP_loc = next_WP_loc;

    uint8_t cmd_passby = HIGHBYTE(cmd.p1); // 超过航点的距离（米）
    uint8_t cmd_acceptance_distance = LOWBYTE(cmd.p1); // 接受到达航点的半径（米）

    if (cmd_passby > 0) {
        const float dist = prev_WP_loc.get_distance(flex_next_WP_loc);
        const float bearing_deg = degrees(prev_WP_loc.get_bearing(flex_next_WP_loc));

        if (is_positive(dist)) {
            flex_next_WP_loc.offset_bearing(bearing_deg, cmd_passby);
        }
    }

    if (auto_state.crosstrack) {
        nav_controller->update_waypoint(prev_WP_loc, flex_next_WP_loc);
    } else {
        nav_controller->update_waypoint(current_loc, flex_next_WP_loc);
    }

    // 检查用户是否指定了到航点的最大距离
    // 如果用p3覆盖 - 则不使用此值，因为它会导致严重的超飞
    if (g.waypoint_max_radius > 0 &&
        auto_state.wp_distance > (uint16_t)g.waypoint_max_radius) {
        if (current_loc.past_interval_finish_line(prev_WP_loc, flex_next_WP_loc)) {
            // 这是为了确保完成航点
            if (cmd_passby == 0) {
                prev_WP_loc = current_loc;
            }
        }
        return false;
    }

    float acceptance_distance_m = 0; // 默认为：如果超飞 - 让它飞到该点
    if (cmd_acceptance_distance > 0) {
        // 允许用户覆盖接受半径
        acceptance_distance_m = cmd_acceptance_distance;
    } else if (cmd_passby == 0) {
        acceptance_distance_m = nav_controller->turn_distance(get_wp_radius(), auto_state.next_turn_angle);
    }
    const float wp_dist = current_loc.get_distance(flex_next_WP_loc);
    if (wp_dist <= acceptance_distance_m) {
        gcs().send_text(MAV_SEVERITY_INFO, "Reached waypoint #%i dist %um",
                          (unsigned)mission.get_current_nav_cmd().index,
                          (unsigned)current_loc.get_distance(flex_next_WP_loc));
        return true;
	}

    // 我们是否已经飞过了航点？
    if (current_loc.past_interval_finish_line(prev_WP_loc, flex_next_WP_loc)) {
        gcs().send_text(MAV_SEVERITY_INFO, "Passed waypoint #%i dist %um",
                          (unsigned)mission.get_current_nav_cmd().index,
                          (unsigned)current_loc.get_distance(flex_next_WP_loc));
        return true;
    }

    return false;
}

// 验证无限盘旋
bool Plane::verify_loiter_unlim(const AP_Mission::Mission_Command &cmd)
{
    // 使用任务半径
    update_loiter(cmd.p1);
    return false;
}

// 验证定时盘旋
bool Plane::verify_loiter_time()
{
    bool result = false;
    // 任务半径始终为aparm.loiter_radius
    update_loiter(0);

    if (loiter.start_time_ms == 0) {
        if (reached_loiter_target() && loiter.sum_cd > 1) {
            // 我们已经到达目标，启动计时器
            loiter.start_time_ms = millis();
        }
    } else if (condition_value != 0) {
        // 主要目标，盘旋时间
        if ((millis() - loiter.start_time_ms) > loiter.time_max_ms) {
            // 主要目标完成，初始化次要航向目标
            condition_value = 0;
            result = verify_loiter_heading(true);
        }
    } else {
        // 次要目标，盘旋到指定航向
        result = verify_loiter_heading(false);
    }

    if (result) {
        gcs().send_text(MAV_SEVERITY_INFO,"Loiter time complete");
        auto_state.vtol_loiter = false;
    }
    return result;
}
// 验证定点盘旋转弯命令
bool Plane::verify_loiter_turns(const AP_Mission::Mission_Command &cmd)
{
    bool result = false;
    uint16_t radius = HIGHBYTE(cmd.p1);
    if (cmd.type_specific_bits & (1U<<0)) {
        // 特殊存储处理允许更大的半径
        radius *= 10;
    }
    update_loiter(radius);

    // LOITER_TURNS 对VTOL模式无意义
    auto_state.vtol_loiter = false;

    if (condition_value != 0) {
        // 主要目标：盘旋时间
        if (loiter.sum_cd > loiter.total_cd && loiter.sum_cd > 1) {
            // 主要目标完成，初始化次要航向目标
            condition_value = 0;
            result = verify_loiter_heading(true);
        }
    } else {
        // 次要目标：盘旋到指定航向
        result = verify_loiter_heading(false);
    }

    if (result) {
        gcs().send_text(MAV_SEVERITY_INFO,"定点盘旋轨迹完成");
    }
    return result;
}

// 验证盘旋到指定高度命令
// 这涉及检查我们是否已经达到所需的高度和航向
// 所需的高度只需要达到一次
bool Plane::verify_loiter_to_alt(const AP_Mission::Mission_Command &cmd)
{
    bool result = false;

    update_loiter(cmd.p1);

    // condition_value == 0 表示从未达到目标高度
    if (condition_value == 0) {
        // 主要目标：盘旋到指定高度
        if (labs(loiter.sum_cd) > 1 && (loiter.reached_target_alt || loiter.unable_to_acheive_target_alt)) {
            // 主要目标完成，初始化次要航向目标
            if (loiter.unable_to_acheive_target_alt) {
                gcs().send_text(MAV_SEVERITY_INFO,"盘旋到高度被卡在 %d", int(current_loc.alt/100));
            }

            condition_value = 1;
            result = verify_loiter_heading(true);
        }
    } else {
        // 次要目标：盘旋到指定航向
        result = verify_loiter_heading(false);
    }

    if (result) {
        gcs().send_text(MAV_SEVERITY_INFO,"盘旋到高度完成");
    }
    return result;
}

// 验证返航（RTL）
bool Plane::verify_RTL()
{
    if (g.rtl_radius < 0) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }
    update_loiter(abs(g.rtl_radius));
	if (auto_state.wp_distance <= (uint32_t)MAX(get_wp_radius(),0) || 
        reached_loiter_target()) {
			gcs().send_text(MAV_SEVERITY_INFO,"到达RTL位置");
			return true;
    } else {
        return false;
	}
}

// 验证继续并改变高度
bool Plane::verify_continue_and_change_alt()
{
    // 航点信息不可用且航向保持可用？
    if (prev_WP_loc.same_latlon_as(next_WP_loc) &&
        steer_state.hold_course_cd != -1) {
        // 保持飞行相同的航向，使用命令开始时计算的固定转向航向
        nav_controller->update_heading_hold(steer_state.hold_course_cd);
    }
    else {
        // 下一个航点是否在200米以内？
        if (current_loc.get_distance(next_WP_loc) < 200.0f) {
            // 在航线上再推进300米
            int32_t next_wp_bearing_cd = prev_WP_loc.get_bearing_to(next_WP_loc);
            next_WP_loc.offset_bearing(next_wp_bearing_cd * 0.01f, 300.0f);
        }

        // 保持飞行相同的航向
        nav_controller->update_waypoint(prev_WP_loc, next_WP_loc);
    }

    // 正在爬升？
    if (condition_value == 1 && adjusted_altitude_cm() >= next_WP_loc.alt) {
        return true;
    }
    // 正在下降？
    else if (condition_value == 2 &&
             adjusted_altitude_cm() <= next_WP_loc.alt) {
        return true;
    }    
    // 不关心是爬升还是下降
    else if (labs(adjusted_altitude_cm() - next_WP_loc.alt) <= 500) {
        return true;
    }

    return false;
}

// 检查是否已达到高度或下降速度
bool ModeAuto::verify_altitude_wait(const AP_Mission::Mission_Command &cmd)
{
#if AP_PLANE_GLIDER_PULLUP_ENABLED
    if (pullup.in_pullup()) {
        return pullup.verify_pullup();
    }
#endif

    // param1中的目标高度始终是AMSL（平均海平面以上）
    const float alt_diff = plane.current_loc.alt*0.01 - cmd.content.altitude_wait.altitude;
    bool completed = false;
    if (alt_diff > 0) {
        gcs().send_text(MAV_SEVERITY_INFO,"已达到高度");
        completed = true;
    } else if (cmd.content.altitude_wait.descent_rate > 0 &&
        plane.auto_state.sink_rate > cmd.content.altitude_wait.descent_rate) {
        gcs().send_text(MAV_SEVERITY_INFO, "已达到下降率 %.1f m/s", (double)plane.auto_state.sink_rate);
        completed = true;
    }

    if (completed) {
#if AP_PLANE_GLIDER_PULLUP_ENABLED
        if (pullup.pullup_start()) {
            // 我们正在进行拉升，ALTITUDE_WAIT在拉升完成之前不会完成
            return false;
        }
#endif
        return true;
    }

    const float time_to_alt = alt_diff / MIN(plane.auto_state.sink_rate, -0.01);

    // 如果请求，摆动舵机
    // 如果我们预计很快释放，我们不会开始摆动，因为我们不希望在释放时舵机偏离调整位置
    if (cmd.content.altitude_wait.wiggle_time != 0 &&
        (plane.auto_state.sink_rate > 0 || time_to_alt > cmd.content.altitude_wait.wiggle_time*5)) {
        if (wiggle.stage == 0 &&
            AP_HAL::millis() - wiggle.last_ms > cmd.content.altitude_wait.wiggle_time*1000) {
            wiggle.stage = 1;
            wiggle.last_ms = AP_HAL::millis();
            // idle_wiggle_stage 在 wiggle_servos() 中更新
        }
    }

    return false;
}

// 验证导航延迟 - 检查我们是否已经等待足够长的时间
bool ModeAuto::verify_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    if (AP::arming().is_armed_and_safety_off()) {
        // 在武装状态下不延迟，我们需要运行导航控制器
        return true;
    }
    if (millis() - nav_delay.time_start_ms > nav_delay.time_max_ms) {
        nav_delay.time_max_ms = 0;
        return true;
    }
    return false;
}

/********************************************************************************/
//  条件（可能）命令
/********************************************************************************/

// 执行等待延迟命令
void Plane::do_wait_delay(const AP_Mission::Mission_Command& cmd)
{
    condition_start = millis();
    condition_value  = cmd.content.delay.seconds * 1000;    // 将秒转换为毫秒
}

// 执行在指定距离内命令
void Plane::do_within_distance(const AP_Mission::Mission_Command& cmd)
{
    condition_value  = cmd.content.distance.meters;
}

/********************************************************************************/
// 验证条件（可能）命令
/********************************************************************************/

// 验证等待延迟
bool Plane::verify_wait_delay()
{
    if ((unsigned)(millis() - condition_start) > (unsigned)condition_value) {
        condition_value         = 0;
        return true;
    }
    return false;
}

// 验证在指定距离内
bool Plane::verify_within_distance()
{
    if (auto_state.wp_distance < MAX(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

/********************************************************************************/
//  执行（现在）命令
/********************************************************************************/

// 在当前位置执行定点盘旋
void Plane::do_loiter_at_location()
{
    if (aparm.loiter_radius < 0) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }
    next_WP_loc = current_loc;
}

// 执行改变速度命令
bool Plane::do_change_speed(const AP_Mission::Mission_Command& cmd)
{
    return do_change_speed(
        (uint8_t)cmd.content.speed.speed_type,
        cmd.content.speed.target_ms,
         cmd.content.speed.throttle_pct
        );
}

// 执行改变速度
bool Plane::do_change_speed(uint8_t speedtype, float speed_target_ms, float throttle_pct)
{
    switch (speedtype) {
    case 0:             // 空速
        if (is_equal(speed_target_ms, -2.0f)) {
            new_airspeed_cm = -1; // 返回默认空速
            return true;
        } else if ((speed_target_ms >= aparm.airspeed_min.get()) &&
                   (speed_target_ms <= aparm.airspeed_max.get()))  {
            new_airspeed_cm = speed_target_ms * 100; // AUTO或GUIDED模式的新空速目标
            gcs().send_text(MAV_SEVERITY_INFO, "设置空速 %u m/s", (unsigned)speed_target_ms);
            return true;
        }
        break;
    case 1:             // 地速
        gcs().send_text(MAV_SEVERITY_INFO, "设置地速 %u", (unsigned)speed_target_ms);
        aparm.min_groundspeed.set(speed_target_ms);
        return true;
    }

    if (throttle_pct > 0 && throttle_pct <= 100) {
        gcs().send_text(MAV_SEVERITY_INFO, "设置油门 %u", (unsigned)throttle_pct);
        aparm.throttle_cruise.set(throttle_pct);
        return true;
    }

    return false;
}

// 执行设置家位置命令
void Plane::do_set_home(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 == 1 && gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        if (!set_home_persistently(gps.location())) {
            // 静默忽略错误
        }
    } else {
        if (!AP::ahrs().set_home(cmd.content.location)) {
            // 静默忽略失败
        }
    }
}

// 开始命令回调 - 当ap-mission开始一个新的任务命令时调用的回调函数
// 我们再次检查飞行模式是否为AUTO，以避免ap-mission在非AUTO模式下触发动作的可能性
bool Plane::start_command_callback(const AP_Mission::Mission_Command &cmd)
{
    if (control_mode == &mode_auto) {
        return start_command(cmd);
    }
    return true;
}

// 验证命令回调 - 当命令正在运行时，ap-mission以10Hz或更高频率调用的回调函数
// 我们再次检查飞行模式是否为AUTO，以避免ap-mission在非AUTO模式下触发动作的可能性
bool Plane::verify_command_callback(const AP_Mission::Mission_Command& cmd)
{
    if (control_mode == &mode_auto) {
        bool cmd_complete = verify_command(cmd);

        // 向GCS发送消息
        if (cmd_complete) {
            gcs().send_mission_item_reached_message(cmd.index);
        }

        return cmd_complete;
    }
    return false;
}
// 当任务完成时的回调函数
// 我们再次检查飞行模式是否为AUTO，以避免ap-mission在非AUTO模式下触发动作
void Plane::exit_mission_callback()
{
    if (control_mode == &mode_auto) {
        set_mode(mode_rtl, ModeReason::MISSION_END);
        gcs().send_text(MAV_SEVERITY_INFO, "Mission complete, changing mode to RTL");
    }
}

#if HAL_QUADPLANE_ENABLED
// 验证VTOL着陆接近阶段
bool Plane::verify_landing_vtol_approach(const AP_Mission::Mission_Command &cmd)
{
    // 计算接近半径和方向
    const float radius = is_zero(quadplane.fw_land_approach_radius)? aparm.loiter_radius : quadplane.fw_land_approach_radius;
    const int8_t direction = is_negative(radius) ? -1 : 1;
    const float abs_radius = fabsf(radius);

    loiter.direction = direction;

    // 根据不同的接近阶段执行相应的操作
    switch (vtol_approach_s.approach_stage) {
        case VTOLApproach::Stage::RTL:
            {
                // 飞回家并在RTL高度盘旋
                nav_controller->update_loiter(cmd.content.location, abs_radius, direction);
                if (plane.reached_loiter_target()) {
                    // 下降到四旋翼RTL高度
                    plane.do_RTL(plane.home.alt + plane.quadplane.qrtl_alt*100UL);
                    plane.loiter_angle_reset();
                    vtol_approach_s.approach_stage = VTOLApproach::Stage::LOITER_TO_ALT;
                }
                break;
            }
        case VTOLApproach::Stage::LOITER_TO_ALT:
            {
                nav_controller->update_loiter(cmd.content.location, abs_radius, direction);

                // 检查是否达到目标高度或无法达到目标高度
                if (labs(loiter.sum_cd) > 1 && (loiter.reached_target_alt || loiter.unable_to_acheive_target_alt)) {
                    // 计算风向并选择接近路径
                    Vector3f wind = ahrs.wind_estimate();
                    vtol_approach_s.approach_direction_deg = degrees(atan2f(-wind.y, -wind.x));
                    gcs().send_text(MAV_SEVERITY_INFO, "Selected an approach path of %.1f", (double)vtol_approach_s.approach_direction_deg);
                    vtol_approach_s.approach_stage = VTOLApproach::Stage::ENSURE_RADIUS;
                }
                break;
            }
        case VTOLApproach::Stage::ENSURE_RADIUS:
            {
                // 验证飞机是否至少在预期距离之外
                // 要求角度总和至少为2厘度，因为1厘度有特殊情况
                if (((fabsF(cmd.content.location.get_distance(current_loc) - abs_radius) > 5.0f) &&
                      (cmd.content.location.get_distance(current_loc) < abs_radius)) ||
                    (labs(loiter.sum_cd) < 2)) {
                    nav_controller->update_loiter(cmd.content.location, abs_radius, direction);
                    break;
                }
                vtol_approach_s.approach_stage = VTOLApproach::Stage::WAIT_FOR_BREAKOUT;
                FALLTHROUGH;
            }
        case VTOLApproach::Stage::WAIT_FOR_BREAKOUT:
            {
                nav_controller->update_loiter(cmd.content.location, radius, direction);

                // 计算脱离方向
                const float breakout_direction_rad = radians(vtol_approach_s.approach_direction_deg + (direction > 0 ? 270 : 90));

                // 当在反方向的5度范围内时脱离
                if (fabsF(wrap_PI(ahrs.get_yaw() - breakout_direction_rad)) < radians(5.0f)) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Starting VTOL land approach path");
                    vtol_approach_s.approach_stage = VTOLApproach::Stage::APPROACH_LINE;
                    set_next_WP(cmd.content.location);
                } else {
                    break;
                }
                FALLTHROUGH;
            }
        case VTOLApproach::Stage::APPROACH_LINE:
            {
                // 投影接近路径
                Location start = cmd.content.location;
                Location end = cmd.content.location;

                // 在着陆位置两侧投影1公里的航点
                start.offset_bearing(vtol_approach_s.approach_direction_deg + 180, 1000);
                end.offset_bearing(vtol_approach_s.approach_direction_deg, 1000);

                nav_controller->update_waypoint(start, end);

                // 检查是否应该移动到下一个航点
                Location breakout_stopping_loc = cmd.content.location;
                breakout_stopping_loc.offset_bearing(vtol_approach_s.approach_direction_deg + 180, quadplane.stopping_distance());
                const bool past_finish_line = current_loc.past_interval_finish_line(start, breakout_stopping_loc);

                Location breakout_loc = cmd.content.location;
                breakout_loc.offset_bearing(vtol_approach_s.approach_direction_deg + 180, abs_radius);
                const bool half_radius = current_loc.line_path_proportion(breakout_loc, cmd.content.location) > 0.5;
                bool lined_up = true;
                Vector3f vel_NED;
                if (ahrs.get_velocity_NED(vel_NED)) {
                    const Vector2f target_vec = current_loc.get_distance_NE(cmd.content.location);
                    const float angle_err = fabsf(wrap_180(degrees(vel_NED.xy().angle(target_vec))));
                    lined_up = (angle_err < 30);
                }

                if (past_finish_line && (lined_up || half_radius)) {
                    vtol_approach_s.approach_stage = VTOLApproach::Stage::VTOL_LANDING;
                    quadplane.do_vtol_land(cmd);
                } else {
                    break;
                }
                FALLTHROUGH;
            }
        case VTOLApproach::Stage::VTOL_LANDING:
            // 这里不需要做任何事，我们应该已经进入四旋翼着陆代码
            return true;
    }

    return false;
}
#endif // HAL_QUADPLANE_ENABLED

// 验证盘旋航向
bool Plane::verify_loiter_heading(bool init)
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_auto()) {
        // 如果在VTOL自动模式，跳过航向验证
        return true;
    }
#endif

    // 获取下一个导航航点的经纬度
    AP_Mission::Mission_Command next_nav_cmd;
    if (! mission.get_next_nav_cmd(mission.get_current_nav_index() + 1,
                                   next_nav_cmd)) {
        // 没有下一个航点可以瞄准 -- 继续并跳出盘旋
        return true;
    }

    if (init) {
        loiter.sum_cd = 0;
    }

    return plane.mode_loiter.isHeadingLinedUp(next_WP_loc, next_nav_cmd.content.location);
}

// 获取航点半径
float Plane::get_wp_radius() const
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.in_vtol_mode()) {
        return plane.quadplane.wp_nav->get_wp_radius_cm() * 0.01;
    }
#endif
    return g.waypoint_radius;
}

#if AP_SCRIPTING_ENABLED
// 支持脚本化导航，带有完成验证操作
void Plane::do_nav_script_time(const AP_Mission::Mission_Command& cmd)
{
    nav_scripting.enabled = true;
    nav_scripting.id++;
    nav_scripting.start_ms = AP_HAL::millis();
    nav_scripting.current_ms = nav_scripting.start_ms;

    // 从当前的滚转率、俯仰率和油门开始
    nav_scripting.roll_rate_dps = plane.rollController.get_pid_info().target;
    nav_scripting.pitch_rate_dps = plane.pitchController.get_pid_info().target;
    nav_scripting.yaw_rate_dps = degrees(ahrs.get_gyro().z);
    nav_scripting.throttle_pct = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
}

// 等待脚本说任务项完成
bool Plane::verify_nav_script_time(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.content.nav_script_time.timeout_s > 0) {
        const uint32_t now = AP_HAL::millis();
        if (now - nav_scripting.start_ms > cmd.content.nav_script_time.timeout_s*1000U) {
            gcs().send_text(MAV_SEVERITY_INFO, "NavScriptTime timed out");
            nav_scripting.enabled = false;
            nav_scripting.rudder_offset_pct = 0;
            nav_scripting.run_yaw_rate_controller = true;
        }
    }
    return !nav_scripting.enabled;
}

// 检查是否在NAV_SCRIPT_*命令中
bool Plane::nav_scripting_active(void)
{
    if (nav_scripting.enabled && AP_HAL::millis() - nav_scripting.current_ms > 1000) {
        // 在过去1000ms内脚本没有调用set_target_throttle_rate_rpy
        nav_scripting.enabled = false;
        nav_scripting.current_ms = 0;
        nav_scripting.rudder_offset_pct = 0;
        nav_scripting.run_yaw_rate_controller = true;
        gcs().send_text(MAV_SEVERITY_INFO, "NavScript time out");
    }
    if (control_mode == &mode_auto &&
        mission.get_current_nav_cmd().id != MAV_CMD_NAV_SCRIPT_TIME) {
        nav_scripting.enabled = false;
    }
    return nav_scripting.enabled;
}

// 支持NAV_SCRIPTING任务命令
bool Plane::nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2, int16_t &arg3, int16_t &arg4)
{
    if (!nav_scripting_active()) {
        // 不在NAV_SCRIPT_TIME中
        return false;
    }
    const auto &c = mission.get_current_nav_cmd().content.nav_script_time;
    id = nav_scripting.id;
    cmd = c.command;
    arg1 = c.arg1.get();
    arg2 = c.arg2.get();
    arg3 = c.arg3;
    arg4 = c.arg4;
    return true;
}

// 当脚本完成命令时调用
void Plane::nav_script_time_done(uint16_t id)
{
    if (id == nav_scripting.id) {
        nav_scripting.enabled = false;
    }
}

// 支持NAV_SCRIPTING任务命令和其他允许模式下的特技飞行
void Plane::set_target_throttle_rate_rpy(float throttle_pct, float roll_rate_dps, float pitch_rate_dps, float yaw_rate_dps)
{
    nav_scripting.roll_rate_dps = constrain_float(roll_rate_dps, -g.acro_roll_rate, g.acro_roll_rate);
    nav_scripting.pitch_rate_dps = constrain_float(pitch_rate_dps, -g.acro_pitch_rate, g.acro_pitch_rate);
    nav_scripting.yaw_rate_dps = constrain_float(yaw_rate_dps, -g.acro_yaw_rate, g.acro_yaw_rate);
    nav_scripting.throttle_pct = constrain_float(throttle_pct, aparm.throttle_min, aparm.throttle_max);
    nav_scripting.current_ms = AP_HAL::millis();
}

// 支持特技飞行脚本中的方向舵偏移覆盖
void Plane::set_rudder_offset(float rudder_pct, bool run_yaw_rate_controller)
{
    nav_scripting.rudder_offset_pct = rudder_pct;
    nav_scripting.run_yaw_rate_controller = run_yaw_rate_controller;
}

// 在AUTO以外的模式下启用NAV_SCRIPTING接管，使用脚本时间任务命令
bool Plane::nav_scripting_enable(uint8_t mode)
{
   uint8_t current_control_mode = control_mode->mode_number();
   if (current_control_mode == mode) {
       switch (current_control_mode) {
       case Mode::Number::CIRCLE:
       case Mode::Number::STABILIZE:
       case Mode::Number::ACRO:
       case Mode::Number::FLY_BY_WIRE_A:
       case Mode::Number::FLY_BY_WIRE_B:
       case Mode::Number::CRUISE:
       case Mode::Number::LOITER:
           nav_scripting.enabled = true;
           nav_scripting.current_ms = AP_HAL::millis();
           break;
       default:
           nav_scripting.enabled = false;
       }
   } else {
       nav_scripting.enabled = false;
   }
   return nav_scripting.enabled;
}
#endif // AP_SCRIPTING_ENABLED
/*
  判断给定的命令是否为着陆命令
  注意：我们将PAYLOAD_PLACE也视为着陆命令，因为它遵循四旋翼飞行器的着陆逻辑
 */
bool Plane::is_land_command(uint16_t command) const
{
    return
        command == MAV_CMD_NAV_VTOL_LAND ||    // 垂直起降着陆命令
        command == MAV_CMD_NAV_LAND ||         // 普通着陆命令
        command == MAV_CMD_NAV_PAYLOAD_PLACE;  // 有效载荷放置命令（视为着陆）
}

/*
  判断飞机是否正在执行特定的AUTO任务命令
 */
bool Plane::in_auto_mission_id(uint16_t command) const
{
    return control_mode == &mode_auto &&              // 确保当前处于AUTO模式
           mission.get_current_nav_id() == command;   // 当前导航命令ID与给定命令匹配
}

