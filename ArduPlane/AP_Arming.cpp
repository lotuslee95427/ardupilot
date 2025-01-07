/*
  飞机特定的额外解锁检查
 */
#include "AP_Arming.h"
#include "Plane.h"

#include "qautotune.h"

constexpr uint32_t AP_ARMING_DELAY_MS = 2000; // 从解锁到电机开始加速的延迟时间

// 飞机特定的解锁参数组
const AP_Param::GroupInfo AP_Arming_Plane::var_info[] = {
    // 父类变量
    AP_NESTEDGROUPINFO(AP_Arming, 0),

    // 索引3曾用于RUDDER，现已不再使用

#if AP_PLANE_BLACKBOX_LOGGING
    // @Param: BBOX_SPD
    // @DisplayName: 黑匣子速度
    // @Description: 3D GPS速度阈值，超过此值将强制解锁飞行器以开始记录。警告：仅在飞行器没有螺旋桨且飞控不控制飞行器时使用。
    // @Units: m/s
    // @Increment: 1
    // @Range: 1 20
    // @User: Advanced
    AP_GROUPINFO("BBOX_SPD", 4, AP_Arming_Plane, blackbox_speed, 5),
#endif // AP_PLANE_BLACKBOX_LOGGING
    
    AP_GROUPEND
};

// 判断是否需要完全加载地形数据库
bool AP_Arming_Plane::terrain_database_required() const
{
#if AP_TERRAIN_AVAILABLE
    if (plane.g.terrain_follow) {
        return true;
    }
#endif
    return AP_Arming::terrain_database_required();
}

/*
  飞机特定的额外解锁检查
 */
bool AP_Arming_Plane::pre_arm_checks(bool display_failure)
{
    // 如果已解锁或不需要解锁检查，则跳过检查
    if (armed || require == (uint8_t)Required::NO) {
        return true;
    }
    // 检查是否禁用了解锁检查
    if (checks_to_perform == 0) {
        return mandatory_checks(display_failure);
    }
    // 看门狗重置后跳过解锁检查，允许远程遥控解锁
    if (hal.util->was_watchdog_armed()) {
        return true;
    }

    // 调用父类检查
    bool ret = AP_Arming::pre_arm_checks(display_failure);

#if AP_AIRSPEED_ENABLED
    // 检查空速传感器
    ret &= AP_Arming::airspeed_checks(display_failure);
#endif

    // 检查故障保护超时设置
    if (plane.g.fs_timeout_long < plane.g.fs_timeout_short && plane.g.fs_action_short != FS_ACTION_SHORT_DISABLED) {
        check_failed(display_failure, "FS_LONG_TIMEOUT < FS_SHORT_TIMEOUT");
        ret = false;
    }

    // 检查滚转限制
    if (plane.aparm.roll_limit < 3) {
        check_failed(display_failure, "ROLL_LIMIT_DEG太小 (%.1f)", plane.aparm.roll_limit.get());
        ret = false;
    }

    // 检查俯仰限制
    if (plane.aparm.pitch_limit_max < 3) {
        check_failed(display_failure, "PTCH_LIM_MAX_DEG太小 (%.1f)", plane.aparm.pitch_limit_max.get());
        ret = false;
    }

    if (plane.aparm.pitch_limit_min > -3) {
        check_failed(display_failure, "PTCH_LIM_MIN_DEG太大 (%.1f)", plane.aparm.pitch_limit_min.get());
        ret = false;
    }

    // 检查最小空速设置
    if (plane.aparm.airspeed_min < MIN_AIRSPEED_MIN) {
        check_failed(display_failure, "AIRSPEED_MIN太低 (%i < %i)", plane.aparm.airspeed_min.get(), MIN_AIRSPEED_MIN);
        ret = false;
    }

    // 检查油门故障保护值设置
    if (plane.channel_throttle->get_reverse() && 
        Plane::ThrFailsafe(plane.g.throttle_fs_enabled.get()) != Plane::ThrFailsafe::Disabled &&
        plane.g.throttle_fs_value < 
        plane.channel_throttle->get_radio_max()) {
        check_failed(display_failure, "反向油门的THR_FS_VALUE无效");
        ret = false;
    }

    // 检查遥控器信号
    ret &= rc_received_if_enabled_check(display_failure);

#if HAL_QUADPLANE_ENABLED
    // 四旋翼模式检查
    ret &= quadplane_checks(display_failure);
#endif

    // 检查ADS-B避障故障保护
    if (plane.failsafe.adsb) {
        check_failed(display_failure, "检测到ADSB威胁");
        ret = false;
    }

    // 检查油门微调
    if (plane.flight_option_enabled(FlightOptions::CENTER_THROTTLE_TRIM)){
       int16_t trim = plane.channel_throttle->get_radio_trim();
       if (trim < 1250 || trim > 1750) {
           check_failed(display_failure, "油门微调不在中心位置(%u)",trim );
           ret = false;
       }
    }

    // 检查是否在着陆序列中
    if (plane.mission.get_in_landing_sequence_flag() &&
        !plane.mission.starts_with_takeoff_cmd()) {
        check_failed(display_failure,"处于着陆序列中");
        ret = false;
    }

    // 检查当前飞行模式的解锁条件
    char failure_msg[50] {};
    if (!plane.control_mode->pre_arm_checks(ARRAY_SIZE(failure_msg), failure_msg)) {
        check_failed(display_failure, "%s %s", plane.control_mode->name(), failure_msg);
        return false;
    }

    return ret;
}

// 强制性检查
bool AP_Arming_Plane::mandatory_checks(bool display_failure)
{
    bool ret = true;

    // 检查遥控器信号
    ret &= rc_received_if_enabled_check(display_failure);

    // 调用父类强制性检查
    ret &= AP_Arming::mandatory_checks(display_failure);

    return ret;
}


#if HAL_QUADPLANE_ENABLED
// 四旋翼模式特定检查
bool AP_Arming_Plane::quadplane_checks(bool display_failure)
{
    // 如果四旋翼模式未启用，直接返回true
    if (!plane.quadplane.enabled()) {
        return true;
    }

    // 检查四旋翼模式是否可用
    if (!plane.quadplane.available()) {
        check_failed(display_failure, "四旋翼模式已启用但未运行");
        return false;
    }

    bool ret = true;

    // 检查调度器循环频率
    if (plane.scheduler.get_loop_rate_hz() < 100) {
        check_failed(display_failure, "四旋翼模式需要SCHED_LOOP_RATE >= 100");
        ret = false;
    }

    // 检查电机
    char failure_msg[50] {};
    if (!plane.quadplane.motors->arming_checks(ARRAY_SIZE(failure_msg), failure_msg)) {
        check_failed(display_failure, "电机: %s", failure_msg);
        ret = false;
    }

    // 检查最大倾角参数
    if (plane.quadplane.aparm.angle_max < 1000 || plane.quadplane.aparm.angle_max > 8000) {
        check_failed(ARMING_CHECK_PARAMETERS, display_failure, "检查Q_ANGLE_MAX");
        ret = false;
    }

    // 检查尾座式和倾转式设置
    if ((plane.quadplane.tailsitter.enable > 0) && (plane.quadplane.tiltrotor.enable > 0)) {
        check_failed(ARMING_CHECK_PARAMETERS, display_failure, "设置TAILSIT_ENABLE为0或TILT_ENABLE为0");
        ret = false;
    } else {
        // 检查尾座式设置
        if ((plane.quadplane.tailsitter.enable > 0) && !plane.quadplane.tailsitter.enabled()) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "尾座式设置不完整，需要重启");
            ret = false;
        }

        // 检查倾转式设置
        if ((plane.quadplane.tiltrotor.enable > 0) && !plane.quadplane.tiltrotor.enabled()) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "倾转式设置不完整，需要重启");
            ret = false;
        }
    }

    // 检查控制器
    if (!plane.quadplane.pos_control->pre_arm_checks("PSC", failure_msg, ARRAY_SIZE(failure_msg))) {
        check_failed(ARMING_CHECK_PARAMETERS, display_failure, "参数错误: %s", failure_msg);
        ret = false;
    }
    if (!plane.quadplane.attitude_control->pre_arm_checks("ATC", failure_msg, ARRAY_SIZE(failure_msg))) {
        check_failed(ARMING_CHECK_PARAMETERS, display_failure, "参数错误: %s", failure_msg);
        ret = false;
    }

    // 检查辅助速度设置
    if (check_enabled(ARMING_CHECK_PARAMETERS) &&
        is_zero(plane.quadplane.assist.speed) &&
        !plane.quadplane.tailsitter.enabled()) {
        check_failed(display_failure,"Q_ASSIST_SPEED未设置");
        ret = false;
    }

    // 检查前向推力使用设置
    if ((plane.quadplane.tailsitter.enable > 0) && (plane.quadplane.q_fwd_thr_use != QuadPlane::FwdThrUse::OFF)) {
        check_failed(ARMING_CHECK_PARAMETERS, display_failure, "将Q_FWD_THR_USE设为0");
        ret = false;
    }

    return ret;
}
#endif // HAL_QUADPLANE_ENABLED

// 惯性导航系统检查
bool AP_Arming_Plane::ins_checks(bool display_failure)
{
    // 调用父类检查
    if (!AP_Arming::ins_checks(display_failure)) {
        return false;
    }

    // 飞机特定的额外检查
    if (check_enabled(ARMING_CHECK_INS)) {
        char failure_msg[50] = {};
        if (!AP::ahrs().pre_arm_check(true, failure_msg, sizeof(failure_msg))) {
            check_failed(ARMING_CHECK_INS, display_failure, "AHRS: %s", failure_msg);
            return false;
        }
    }

    return true;
}

// 解锁检查
bool AP_Arming_Plane::arm_checks(AP_Arming::Method method)
{
    // 检查方向舵解锁
    if (method == AP_Arming::Method::RUDDER) {
        const AP_Arming::RudderArming arming_rudder = get_rudder_arming_type();

        if (arming_rudder == AP_Arming::RudderArming::IS_DISABLED) {
            // 参数禁止使用方向舵解锁/锁定
            return false;
        }

        // 如果油门不为零，则不允许方向舵解锁/锁定
        if (!is_zero(plane.get_throttle_input())){
            check_failed(true, "油门不为零");
            return false;
        }
    }

    // 如果禁用了解锁检查，直接返回true
    if (checks_to_perform == 0) {
        return true;
    }

    // 看门狗重置后跳过解锁检查
    if (hal.util->was_watchdog_armed()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "看门狗：跳过解锁检查");
        return true;
    }

    // 调用父类检查
    return AP_Arming::arm_checks(method);
}
/*
  更新HAL软件解锁状态
*/
void AP_Arming_Plane::change_arm_state(void)
{
    update_soft_armed();
#if HAL_QUADPLANE_ENABLED
    // 设置四旋翼的解锁状态
    plane.quadplane.set_armed(hal.util->get_soft_armed());
#endif
}

// 解锁飞机
bool AP_Arming_Plane::arm(const AP_Arming::Method method, const bool do_arming_checks)
{
    // 调用基类的解锁方法
    if (!AP_Arming::arm(method, do_arming_checks)) {
        return false;
    }

    // 更新home位置
    if (plane.update_home()) {
        // 如果EKF拒绝了resetHeightDatum调用，home位置可能仍与current_loc不同
        // 如果我们正在更新home，我们希望强制home为current_loc，以便相对高度起飞正常工作
        if (plane.ahrs.set_home(plane.current_loc)) {
            // 更新当前位置
            plane.update_current_loc();
        }
    }

    // 更改解锁状态
    change_arm_state();

    // 延迟解锁一次性触发的上升沿
    delay_arming = true;

    // 发送解锁状态文本
    send_arm_disarm_statustext("Throttle armed");

    return true;
}

/*
  解除电机锁定
 */
bool AP_Arming_Plane::disarm(const AP_Arming::Method method, bool do_disarm_checks)
{
    // 检查是否允许在飞行中解除锁定
    if (do_disarm_checks &&
        (AP_Arming::method_is_GCS(method) ||
         method == AP_Arming::Method::RUDDER)) {
        if (plane.is_flying()) {
            // 飞行中不允许通过mavlink或方向舵解除锁定
            return false;
        }
    }
    
    // 检查方向舵解除锁定是否启用
    if (do_disarm_checks && method == AP_Arming::Method::RUDDER) {
        if (get_rudder_arming_type() != AP_Arming::RudderArming::ARMDISARM) {
            gcs().send_text(MAV_SEVERITY_INFO, "Rudder disarm: disabled");
            return false;
        }
    }

    // 调用基类的解除锁定方法
    if (!AP_Arming::disarm(method, do_disarm_checks)) {
        return false;
    }

    // 如果不在自动模式，重置任务
    if (plane.control_mode != &plane.mode_auto) {
        plane.mission.reset();
    }

    // 在自动油门模式下抑制油门
    plane.throttle_suppressed = plane.control_mode->does_auto_throttle();

    // 如果没有分配空气模式开关，确保空气模式关闭
#if HAL_QUADPLANE_ENABLED
    if ((plane.quadplane.air_mode == AirMode::ON) && (rc().find_channel_for_option(RC_Channel::AUX_FUNC::AIRMODE) == nullptr)) {
        plane.quadplane.air_mode = AirMode::OFF;
    }
#endif

    // 仅在解除锁定成功时记录
    change_arm_state();

#if QAUTOTUNE_ENABLED
    // 可能保存自动调谐的参数
    plane.quadplane.qautotune.disarmed(plane.control_mode == &plane.mode_qautotune);
#endif

    // 重新初始化AUTO和GUIDED模式中用于DO_CHANGE_SPEED命令的速度变量
    plane.new_airspeed_cm = -1;

    // 发送解除锁定状态文本
    send_arm_disarm_statustext("Throttle disarmed");

    return true;
}

// 更新软件解锁状态
void AP_Arming_Plane::update_soft_armed()
{
    bool _armed = is_armed();
#if HAL_QUADPLANE_ENABLED
    // 如果四旋翼电机测试正在运行，则视为已解锁
    if (plane.quadplane.motor_test.running){
        _armed = true;
    }
#endif

    // 设置HAL软件解锁状态
    hal.util->set_soft_armed(_armed);
#if HAL_LOGGING_ENABLED
    // 更新日志记录器的解锁状态
    AP::logger().set_vehicle_armed(hal.util->get_soft_armed());
#endif

    // 更新延迟解锁一次性触发
    if (delay_arming &&
        (AP_HAL::millis() - hal.util->get_last_armed_change() >= AP_ARMING_DELAY_MS)) {
        delay_arming = false;
    }

#if AP_PLANE_BLACKBOX_LOGGING
    // 黑匣子记录逻辑
    if (blackbox_speed > 0) {
        const float speed3d = plane.gps.status() >= AP_GPS::GPS_OK_FIX_3D?plane.gps.velocity().length():0;
        const uint32_t now = AP_HAL::millis();
        if (speed3d > blackbox_speed) {
            last_over_3dspeed_ms = now;
        }
        if (!_armed && speed3d > blackbox_speed) {
            // 强制安全开启以防止电机运行
            hal.rcout->force_safety_on();
            AP_Param::set_by_name("RC_PROTOCOLS", 0);
            arm(Method::BLACKBOX, false);
            gcs().send_text(MAV_SEVERITY_WARNING, "BlackBox: arming at %.1f m/s", speed3d);
        }
        if (_armed && now - last_over_3dspeed_ms > 20000U) {
            gcs().send_text(MAV_SEVERITY_WARNING, "BlackBox: disarming at %.1f m/s", speed3d);
            disarm(Method::BLACKBOX, false);
        }
    }
#endif
}

/*
  额外的飞机任务检查
 */
bool AP_Arming_Plane::mission_checks(bool report)
{
    // 基本检查
    bool ret = AP_Arming::mission_checks(report);
    // 检查DO_LAND_START和RTL_AUTOLAND设置
    if (plane.mission.contains_item(MAV_CMD_DO_LAND_START) && plane.g.rtl_autoland == RtlAutoland::RTL_DISABLE) {
        ret = false;
        check_failed(ARMING_CHECK_MISSION, report, "DO_LAND_START set and RTL_AUTOLAND disabled");
    }
#if HAL_QUADPLANE_ENABLED
    // 四旋翼模式的额外检查
    if (plane.quadplane.available()) {
        const uint16_t num_commands = plane.mission.num_commands();
        AP_Mission::Mission_Command prev_cmd {};
        for (uint16_t i=1; i<num_commands; i++) {
            AP_Mission::Mission_Command cmd;
            if (!plane.mission.read_cmd_from_storage(i, cmd)) {
                break;
            }
            // 检查VTOL着陆距离
            if (plane.is_land_command(cmd.id) &&
                prev_cmd.id == MAV_CMD_NAV_WAYPOINT) {
                const float dist = cmd.content.location.get_distance(prev_cmd.content.location);
                const float tecs_land_speed = plane.TECS_controller.get_land_airspeed();
                const float landing_speed = is_positive(tecs_land_speed)?tecs_land_speed:plane.aparm.airspeed_cruise;
                const float min_dist = 0.75 * plane.quadplane.stopping_distance(sq(landing_speed));
                if (dist < min_dist) {
                    ret = false;
                    check_failed(ARMING_CHECK_MISSION, report, "VTOL land too short, min %.0fm", min_dist);
                }
            }
            prev_cmd = cmd;
        }
    }
#endif
    return ret;
}

// 检查RC接收机是否已接收（如果配置为使用）
bool AP_Arming_Plane::rc_received_if_enabled_check(bool display_failure)
{
    // 如果没有启用任何协议，将永远不会获得RC，不阻止解锁
    if (rc().enabled_protocols() == 0) {
        return true;
    }

    // 如果启用了RC故障保护，我们必须在解锁前接收RC
    if ((Plane::ThrFailsafe(plane.g.throttle_fs_enabled.get()) == Plane::ThrFailsafe::Enabled) && 
        !(rc().has_had_rc_receiver() || rc().has_had_rc_override())) {
        check_failed(display_failure, "Waiting for RC");
        return false;
    }

    return true;
}
