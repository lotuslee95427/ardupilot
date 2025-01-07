#include "GCS_Mavlink.h"

#include "Plane.h"
#include <AP_RPM/AP_RPM_config.h>
#include <AP_Airspeed/AP_Airspeed_config.h>
#include <AP_EFI/AP_EFI_config.h>

// 返回飞行器类型
MAV_TYPE GCS_Plane::frame_type() const
{
#if HAL_QUADPLANE_ENABLED
    return plane.quadplane.get_mav_type();
#else
    return MAV_TYPE_FIXED_WING;
#endif
}

// 返回基本模式
MAV_MODE GCS_MAVLINK_Plane::base_mode() const
{
    uint8_t _base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    // 根据当前飞行模式计算base_mode
    switch (plane.control_mode->mode_number()) {
    case Mode::Number::MANUAL:
    case Mode::Number::TRAINING:
    case Mode::Number::ACRO:
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QACRO:
        _base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        break;
#endif
    case Mode::Number::STABILIZE:
    case Mode::Number::FLY_BY_WIRE_A:
    case Mode::Number::AUTOTUNE:
    case Mode::Number::FLY_BY_WIRE_B:
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QSTABILIZE:
    case Mode::Number::QHOVER:
    case Mode::Number::QLOITER:
    case Mode::Number::QLAND:
#if QAUTOTUNE_ENABLED
    case Mode::Number::QAUTOTUNE:
#endif
#endif  // HAL_QUADPLANE_ENABLED
    case Mode::Number::CRUISE:
        _base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
        break;
    case Mode::Number::AUTO:
    case Mode::Number::RTL:
    case Mode::Number::LOITER:
    case Mode::Number::THERMAL:
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
    case Mode::Number::CIRCLE:
    case Mode::Number::TAKEOFF:
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QRTL:
    case Mode::Number::LOITER_ALT_QLAND:
#endif
        _base_mode = MAV_MODE_FLAG_GUIDED_ENABLED |
                     MAV_MODE_FLAG_STABILIZE_ENABLED;
        break;
    case Mode::Number::INITIALISING:
        break;
    }

    // 检查训练模式下的手动控制
    if (!plane.training_manual_pitch || !plane.training_manual_roll) {
        _base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;        
    }

    // 检查是否启用了稳定器
    if (plane.control_mode != &plane.mode_manual && plane.control_mode != &plane.mode_initializing) {
        _base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    }

    // 检查摇杆混合模式
    if (plane.g.stick_mixing != StickMixing::NONE && plane.control_mode != &plane.mode_initializing) {
        if ((plane.g.stick_mixing != StickMixing::VTOL_YAW) || (plane.control_mode == &plane.mode_auto)) {
            _base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        }
    }

    // 检查是否已解锁
    if (plane.control_mode != &plane.mode_initializing && plane.arming.is_armed()) {
        _base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // 设置自定义模式标志
    _base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    return (MAV_MODE)_base_mode;
}

// 返回自定义模式
uint32_t GCS_Plane::custom_mode() const
{
    return plane.control_mode->mode_number();
}

// 返回飞行器系统状态
MAV_STATE GCS_MAVLINK_Plane::vehicle_system_status() const
{
    if (plane.control_mode == &plane.mode_initializing) {
        return MAV_STATE_CALIBRATING;
    }
    if (plane.any_failsafe_triggered()) {
        return MAV_STATE_CRITICAL;
    }
    if (plane.crash_state.is_crashed) {
        return MAV_STATE_EMERGENCY;
    }
    if (plane.is_flying()) {
        return MAV_STATE_ACTIVE;
    }

    return MAV_STATE_STANDBY;
}

// 发送姿态信息
void GCS_MAVLINK_Plane::send_attitude() const
{
    const AP_AHRS &ahrs = AP::ahrs();

    float r = ahrs.get_roll();
    float p = ahrs.get_pitch();
    float y = ahrs.get_yaw();

    // 根据飞行选项调整俯仰角
    if (!(plane.flight_option_enabled(FlightOptions::GCS_REMOVE_TRIM_PITCH))) {
        p -= radians(plane.g.pitch_trim);
    }

#if HAL_QUADPLANE_ENABLED
    // 四旋翼模式下使用特定视图
    if (plane.quadplane.show_vtol_view()) {
        r = plane.quadplane.ahrs_view->roll;
        p = plane.quadplane.ahrs_view->pitch;
        y = plane.quadplane.ahrs_view->yaw;
    }
#endif

    const Vector3f &omega = ahrs.get_gyro();
    mavlink_msg_attitude_send(
        chan,
        millis(),
        r,
        p,
        y,
        omega.x,
        omega.y,
        omega.z);
}

// 发送姿态目标
void GCS_MAVLINK_Plane::send_attitude_target() 
{
#if HAL_QUADPLANE_ENABLED
    // 检查姿态目标是否有效
    const uint32_t now = AP_HAL::millis();
    if (now  - plane.quadplane.last_att_control_ms > 100) {
        return;
    }

    const Quaternion quat  = plane.quadplane.attitude_control->get_attitude_target_quat();
    const Vector3f& ang_vel = plane.quadplane.attitude_control->get_attitude_target_ang_vel();
    const float throttle = plane.quadplane.attitude_control->get_throttle_in();

    const float quat_out[4] {quat.q1, quat.q2, quat.q3, quat.q4};

    const uint16_t typemask = 0; 

    mavlink_msg_attitude_target_send(
        chan,
        now,
        typemask,
        quat_out,
        ang_vel.x,
        ang_vel.y,
        ang_vel.z,
        throttle);

#endif // HAL_QUADPLANE_ENABLED 
}

// 发送迎角和侧滑角
void GCS_MAVLINK_Plane::send_aoa_ssa()
{
    AP_AHRS &ahrs = AP::ahrs();

    mavlink_msg_aoa_ssa_send(
        chan,
        micros(),
        ahrs.getAOA(),
        ahrs.getSSA());
}

// 发送导航控制器输出
void GCS_MAVLINK_Plane::send_nav_controller_output() const
{
    if (plane.control_mode == &plane.mode_manual) {
        return;
    }
#if HAL_QUADPLANE_ENABLED
    const QuadPlane &quadplane = plane.quadplane;
    if (quadplane.show_vtol_view() && quadplane.using_wp_nav()) {
        const Vector3f &targets = quadplane.attitude_control->get_att_target_euler_cd();

        const Vector2f& curr_pos = quadplane.inertial_nav.get_position_xy_cm();
        const Vector2f& target_pos = quadplane.pos_control->get_pos_target_cm().xy().tofloat();
        const Vector2f error = (target_pos - curr_pos) * 0.01;

        mavlink_msg_nav_controller_output_send(
            chan,
            targets.x * 0.01,
            targets.y * 0.01,
            targets.z * 0.01,
            degrees(error.angle()),
            MIN(error.length(), UINT16_MAX),
            (plane.control_mode != &plane.mode_qstabilize) ? quadplane.pos_control->get_pos_error_z_cm() * 0.01 : 0,
            plane.airspeed_error * 100,
            quadplane.wp_nav->crosstrack_error());
        return;
    }
#endif
    {
        const AP_Navigation *nav_controller = plane.nav_controller;
        mavlink_msg_nav_controller_output_send(
            chan,
            plane.nav_roll_cd * 0.01,
            plane.nav_pitch_cd * 0.01,
            nav_controller->nav_bearing_cd() * 0.01,
            nav_controller->target_bearing_cd() * 0.01,
            MIN(plane.auto_state.wp_distance, UINT16_MAX),
            plane.calc_altitude_error_cm() * 0.01,
            plane.airspeed_error * 100,
            nav_controller->crosstrack_error());
    }
}

// 发送全球位置目标
void GCS_MAVLINK_Plane::send_position_target_global_int()
{
    if (plane.control_mode == &plane.mode_manual) {
        return;
    }
    Location &next_WP_loc = plane.next_WP_loc;
    static constexpr uint16_t POSITION_TARGET_TYPEMASK_LAST_BYTE = 0xF000;
    static constexpr uint16_t TYPE_MASK = POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                                          POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                                          POSITION_TARGET_TYPEMASK_YAW_IGNORE | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE | POSITION_TARGET_TYPEMASK_LAST_BYTE;
    int32_t alt = 0;
    if (!next_WP_loc.is_zero()) {
        UNUSED_RESULT(next_WP_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt));
    }

    mavlink_msg_position_target_global_int_send(
        chan,
        AP_HAL::millis(),
        MAV_FRAME_GLOBAL,
        TYPE_MASK,
        next_WP_loc.lat,
        next_WP_loc.lng,
        alt * 0.01,
        0.0f, // vx
        0.0f, // vy
        0.0f, // vz
        0.0f, // afx
        0.0f, // afy
        0.0f, // afz
        0.0f, // yaw
        0.0f); // yaw_rate
}

// 返回VFR HUD的空速
float GCS_MAVLINK_Plane::vfr_hud_airspeed() const
{
    // 空速传感器是最佳选择。虽然AHRS的空速估计会使用空速传感器，
    // 但该值受地速限制。在报告时，我们应尽可能发送真实的空速值：
#if AP_AIRSPEED_ENABLED
    if (plane.airspeed.enabled() && plane.airspeed.healthy()) {
        return plane.airspeed.get_airspeed();
    }
#endif

    // 空速估计也可以接受：
    float aspeed;
    if (AP::ahrs().airspeed_estimate(aspeed)) {
        return aspeed;
    }

    // 最差的情况是返回0：
    return 0;
}

// 返回VFR HUD的油门百分比
int16_t GCS_MAVLINK_Plane::vfr_hud_throttle() const
{
    return plane.throttle_percentage();
}

// 返回VFR HUD的爬升率
float GCS_MAVLINK_Plane::vfr_hud_climbrate() const
{
#if HAL_SOARING_ENABLED
    if (plane.g2.soaring_controller.is_active()) {
        return plane.g2.soaring_controller.get_vario_reading();
    }
#endif
    return GCS_MAVLINK::vfr_hud_climbrate();
}

// 发送风速信息
void GCS_MAVLINK_Plane::send_wind() const
{
    const Vector3f wind = AP::ahrs().wind_estimate();
    mavlink_msg_wind_send(
        chan,
        degrees(atan2f(-wind.y, -wind.x)), // 使用负值，给出风的来向
        wind.length(),
        wind.z);
}

// 通过提供的通道发送单个PID信息
void GCS_MAVLINK_Plane::send_pid_info(const AP_PIDInfo *pid_info,
                          const uint8_t axis, const float achieved)
{
    if (pid_info == nullptr) {
        return;
    }
    if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
        return;
    }
     mavlink_msg_pid_tuning_send(chan, axis,
                                 pid_info->target,
                                 achieved,
                                 pid_info->FF,
                                 pid_info->P,
                                 pid_info->I,
                                 pid_info->D,
                                 pid_info->slew_rate,
                                 pid_info->Dmod);
}

// 发送PID调谐消息
void GCS_MAVLINK_Plane::send_pid_tuning()
{
    if (plane.control_mode == &plane.mode_manual) {
        // 手动模式下不应使用PID
        return;
    }

    const Parameters &g = plane.g;

    const AP_PIDInfo *pid_info;
    // 发送滚转PID信息
    if (g.gcs_pid_mask & TUNING_BITS_ROLL) {
        pid_info = &plane.rollController.get_pid_info();
#if HAL_QUADPLANE_ENABLED
        if (plane.quadplane.in_vtol_mode()) {
            pid_info = &plane.quadplane.attitude_control->get_rate_roll_pid().get_pid_info();
        }
#endif
        send_pid_info(pid_info, PID_TUNING_ROLL, pid_info->actual);
    }
    // 发送俯仰PID信息
    if (g.gcs_pid_mask & TUNING_BITS_PITCH) {
        pid_info = &plane.pitchController.get_pid_info();
#if HAL_QUADPLANE_ENABLED
        if (plane.quadplane.in_vtol_mode()) {
            pid_info = &plane.quadplane.attitude_control->get_rate_pitch_pid().get_pid_info();
        }
#endif
        send_pid_info(pid_info, PID_TUNING_PITCH, pid_info->actual);
    }
    // 发送偏航PID信息
    if (g.gcs_pid_mask & TUNING_BITS_YAW) {
        pid_info = &plane.yawController.get_pid_info();
#if HAL_QUADPLANE_ENABLED
        if (plane.quadplane.in_vtol_mode()) {
            pid_info = &plane.quadplane.attitude_control->get_rate_yaw_pid().get_pid_info();
        }
#endif
        send_pid_info(pid_info, PID_TUNING_YAW, pid_info->actual);
    }
    // 发送转向PID信息
    if (g.gcs_pid_mask & TUNING_BITS_STEER) {
        pid_info = &plane.steerController.get_pid_info();
        send_pid_info(pid_info, PID_TUNING_STEER, pid_info->actual);
    }
    // 发送着陆PID信息
    if ((g.gcs_pid_mask & TUNING_BITS_LAND) && (plane.flight_stage == AP_FixedWing::FlightStage::LAND)) {
        AP_AHRS &ahrs = AP::ahrs();
        const Vector3f &gyro = ahrs.get_gyro();
        send_pid_info(plane.landing.get_pid_info(), PID_TUNING_LANDING, degrees(gyro.z));
    }
#if HAL_QUADPLANE_ENABLED
    // 发送垂直加速度PID信息（仅在VTOL模式下）
    if (g.gcs_pid_mask & TUNING_BITS_ACCZ && plane.quadplane.in_vtol_mode()) {
        pid_info = &plane.quadplane.pos_control->get_accel_z_pid().get_pid_info();
        send_pid_info(pid_info, PID_TUNING_ACCZ, pid_info->actual);
    }
#endif
 }

// 返回地面站的系统ID
uint8_t GCS_MAVLINK_Plane::sysid_my_gcs() const
{
    return plane.g.sysid_my_gcs;
}

// 返回是否强制执行系统ID
bool GCS_MAVLINK_Plane::sysid_enforce() const
{
    return plane.g2.sysid_enforce;
}

// 返回遥测延迟
uint32_t GCS_MAVLINK_Plane::telem_delay() const
{
    return (uint32_t)(plane.g.telem_delay);
}

// 尝试发送消息，如果无法放入串行发送缓冲区则返回false
bool GCS_MAVLINK_Plane::try_send_message(enum ap_message id)
{
    switch (id) {

    case MSG_SERVO_OUT:
        // 未使用
        break;

    case MSG_TERRAIN:
#if AP_TERRAIN_AVAILABLE
        CHECK_PAYLOAD_SIZE(TERRAIN_REQUEST);
        plane.terrain.send_request(chan);
#endif
        break;

    case MSG_WIND:
        CHECK_PAYLOAD_SIZE(WIND);
        send_wind();
        break;

    case MSG_ADSB_VEHICLE:
#if HAL_ADSB_ENABLED
        CHECK_PAYLOAD_SIZE(ADSB_VEHICLE);
        plane.adsb.send_adsb_vehicle(chan);
#endif
        break;

    case MSG_AOA_SSA:
        CHECK_PAYLOAD_SIZE(AOA_SSA);
        send_aoa_ssa();
        break;
    case MSG_LANDING:
        plane.landing.send_landing_message(chan);
        break;

    case MSG_HYGROMETER:
#if AP_AIRSPEED_HYGROMETER_ENABLE
        CHECK_PAYLOAD_SIZE(HYGROMETER_SENSOR);
        send_hygrometer();
#endif
        break;

    default:
        return GCS_MAVLINK::try_send_message(id);
    }
    return true;
}

#if AP_AIRSPEED_HYGROMETER_ENABLE
// 发送湿度计数据
void GCS_MAVLINK_Plane::send_hygrometer()
{
    if (!HAVE_PAYLOAD_SPACE(chan, HYGROMETER_SENSOR)) {
        return;
    }

    const auto *airspeed = AP::airspeed();
    if (airspeed == nullptr) {
        return;
    } 
    const uint32_t now = AP_HAL::millis();

    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        uint8_t idx = (i+last_hygrometer_send_idx+1) % AIRSPEED_MAX_SENSORS;
        float temperature, humidity;
        uint32_t last_sample_ms;
        if (!airspeed->get_hygrometer(idx, last_sample_ms, temperature, humidity)) {
            continue;
        }
        if (now - last_sample_ms > 2000) {
            // 数据未更新，停止发送
            continue;
        }
        if (!HAVE_PAYLOAD_SPACE(chan, HYGROMETER_SENSOR)) {
            return;
        }

        mavlink_msg_hygrometer_sensor_send(
            chan,
            idx,
            int16_t(temperature*100),
            uint16_t(humidity*100));
        last_hygrometer_send_idx = idx;
    }
}
#endif // AP_AIRSPEED_HYGROMETER_ENABLE
/*
  默认流速率设置为1Hz
 */
const AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
    // @Param: RAW_SENS
    // @DisplayName: 原始传感器流速率
    // @Description: RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 和 AIRSPEED 的 MAVLink 流速率
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK_Parameters, streamRates[0],  1),

    // @Param: EXT_STAT
    // @DisplayName: 扩展状态流速率
    // @Description: SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (如可用), GPS2_RAW_INT (如可用), GPS2_RTK (如可用), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, 和 GLOBAL_TARGET_POS_INT 的 MAVLink 流速率
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK_Parameters, streamRates[1],  1),

    // @Param: RC_CHAN
    // @DisplayName: RC 通道流速率
    // @Description: SERVO_OUTPUT_RAW 和 RC_CHANNELS 的 MAVLink 流速率
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("RC_CHAN",  2, GCS_MAVLINK_Parameters, streamRates[2],  1),

    // @Param: RAW_CTRL
    // @DisplayName: 原始控制流速率
    // @Description: SERVO_OUT 的 MAVLink 原始控制流速率
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("RAW_CTRL", 3, GCS_MAVLINK_Parameters, streamRates[3],  1),

    // @Param: POSITION
    // @DisplayName: 位置流速率
    // @Description: GLOBAL_POSITION_INT 和 LOCAL_POSITION_NED 的 MAVLink 流速率
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("POSITION", 4, GCS_MAVLINK_Parameters, streamRates[4],  1),

    // @Param: EXTRA1
    // @DisplayName: 额外数据类型1流速率
    // @Description: ATTITUDE, SIMSTATE (仅限SIM), AHRS2, RPM, AOA_SSA, LANDING, ESC_TELEMETRY, EFI_STATUS 和 PID_TUNING 的 MAVLink 流速率
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK_Parameters, streamRates[5],  1),

    // @Param: EXTRA2
    // @DisplayName: 额外数据类型2流速率
    // @Description: VFR_HUD 的 MAVLink 流速率
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("EXTRA2",   6, GCS_MAVLINK_Parameters, streamRates[6],  1),

    // @Param: EXTRA3
    // @DisplayName: 额外数据类型3流速率
    // @Description: AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, BATTERY2, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION 和 BATTERY_STATUS 的 MAVLink 流速率
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK_Parameters, streamRates[7],  1),

    // @Param: PARAMS
    // @DisplayName: 参数流速率
    // @Description: PARAM_VALUE 的 MAVLink 流速率
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("PARAMS",   8, GCS_MAVLINK_Parameters, streamRates[8],  10),

    // @Param: ADSB
    // @DisplayName: ADSB 流速率
    // @Description: MAVLink ADSB 流速率
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("ADSB",   9, GCS_MAVLINK_Parameters, streamRates[9],  5),
    AP_GROUPEND
};

// 原始传感器流消息
static const ap_message STREAM_RAW_SENSORS_msgs[] = {
    MSG_RAW_IMU,
    MSG_SCALED_IMU2,
    MSG_SCALED_IMU3,
    MSG_SCALED_PRESSURE,
    MSG_SCALED_PRESSURE2,
    MSG_SCALED_PRESSURE3,
#if AP_AIRSPEED_ENABLED
    MSG_AIRSPEED,
#endif
};

// 扩展状态流消息
static const ap_message STREAM_EXTENDED_STATUS_msgs[] = {
    MSG_SYS_STATUS,
    MSG_POWER_STATUS,
#if HAL_WITH_MCU_MONITORING
    MSG_MCU_STATUS,
#endif
    MSG_MEMINFO,
    MSG_CURRENT_WAYPOINT,
    MSG_GPS_RAW,
    MSG_GPS_RTK,
#if GPS_MAX_RECEIVERS > 1
    MSG_GPS2_RAW,
    MSG_GPS2_RTK,
#endif
    MSG_NAV_CONTROLLER_OUTPUT,
#if AP_FENCE_ENABLED
    MSG_FENCE_STATUS,
#endif
    MSG_POSITION_TARGET_GLOBAL_INT,
};

// 位置流消息
static const ap_message STREAM_POSITION_msgs[] = {
    MSG_LOCATION,
    MSG_LOCAL_POSITION
};

// 原始控制器流消息
static const ap_message STREAM_RAW_CONTROLLER_msgs[] = {
    MSG_SERVO_OUT,
};

// RC通道流消息
static const ap_message STREAM_RC_CHANNELS_msgs[] = {
    MSG_SERVO_OUTPUT_RAW,
    MSG_RC_CHANNELS,
#if AP_MAVLINK_MSG_RC_CHANNELS_RAW_ENABLED
    MSG_RC_CHANNELS_RAW, // 仅在mavlink1连接时发送
#endif
};

// 额外数据类型1流消息
static const ap_message STREAM_EXTRA1_msgs[] = {
    MSG_ATTITUDE,
#if AP_SIM_ENABLED
    MSG_SIMSTATE,
#endif
    MSG_AHRS2,
#if AP_RPM_ENABLED
    MSG_RPM,
#endif
    MSG_AOA_SSA,
    MSG_PID_TUNING,
    MSG_LANDING,
#if HAL_WITH_ESC_TELEM
    MSG_ESC_TELEMETRY,
#endif
#if HAL_EFI_ENABLED
    MSG_EFI_STATUS,
#endif
#if AP_AIRSPEED_HYGROMETER_ENABLE
    MSG_HYGROMETER,
#endif
};

// 额外数据类型2流消息
static const ap_message STREAM_EXTRA2_msgs[] = {
    MSG_VFR_HUD
};

// 额外数据类型3流消息
static const ap_message STREAM_EXTRA3_msgs[] = {
    MSG_AHRS,
    MSG_WIND,
#if AP_RANGEFINDER_ENABLED
    MSG_RANGEFINDER,
#endif
    MSG_DISTANCE_SENSOR,
    MSG_SYSTEM_TIME,
#if AP_TERRAIN_AVAILABLE
    MSG_TERRAIN,
#endif
#if AP_BATTERY_ENABLED
    MSG_BATTERY_STATUS,
#endif
#if HAL_MOUNT_ENABLED
    MSG_GIMBAL_DEVICE_ATTITUDE_STATUS,
#endif
#if AP_OPTICALFLOW_ENABLED
    MSG_OPTICAL_FLOW,
#endif
#if COMPASS_CAL_ENABLED
    MSG_MAG_CAL_REPORT,
    MSG_MAG_CAL_PROGRESS,
#endif
    MSG_EKF_STATUS_REPORT,
    MSG_VIBRATION,
};

// 参数流消息
static const ap_message STREAM_PARAMS_msgs[] = {
    MSG_NEXT_PARAM
};

// ADSB流消息
static const ap_message STREAM_ADSB_msgs[] = {
    MSG_ADSB_VEHICLE,
#if AP_AIS_ENABLED
    MSG_AIS_VESSEL,
#endif
};

// 所有流条目
const struct GCS_MAVLINK::stream_entries GCS_MAVLINK::all_stream_entries[] = {
    MAV_STREAM_ENTRY(STREAM_RAW_SENSORS),
    MAV_STREAM_ENTRY(STREAM_EXTENDED_STATUS),
    MAV_STREAM_ENTRY(STREAM_POSITION),
    MAV_STREAM_ENTRY(STREAM_RAW_CONTROLLER),
    MAV_STREAM_ENTRY(STREAM_RC_CHANNELS),
    MAV_STREAM_ENTRY(STREAM_EXTRA1),
    MAV_STREAM_ENTRY(STREAM_EXTRA2),
    MAV_STREAM_ENTRY(STREAM_EXTRA3),
    MAV_STREAM_ENTRY(STREAM_PARAMS),
    MAV_STREAM_ENTRY(STREAM_ADSB),
    MAV_STREAM_TERMINATOR // 必须在stream_entries末尾
};

/*
  处理切换到引导模式的请求。这通过handle_mission_item()的回调发生
 */
bool GCS_MAVLINK_Plane::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
    return plane.control_mode->handle_guided_request(cmd.content.location);
}

/*
  处理更改当前WP高度的请求。这通过handle_mission_item()的回调发生
 */
void GCS_MAVLINK_Plane::handle_change_alt_request(AP_Mission::Mission_Command &cmd)
{
    plane.next_WP_loc.alt = cmd.content.location.alt;
    if (cmd.content.location.relative_alt) {
        plane.next_WP_loc.alt += plane.home.alt;
    }
    plane.next_WP_loc.relative_alt = false;
    plane.next_WP_loc.terrain_alt = cmd.content.location.terrain_alt;
    plane.reset_offset_altitude();
}

/*
  处理LANDING_TARGET命令。时间戳已经进行了抖动校正
*/
void GCS_MAVLINK_Plane::handle_landing_target(const mavlink_landing_target_t &packet, uint32_t timestamp_ms)
{
#if AC_PRECLAND_ENABLED
    plane.g2.precland.handle_msg(packet, timestamp_ms);
#endif
}

// 处理飞行前校准命令
MAV_RESULT GCS_MAVLINK_Plane::handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    plane.in_calibration = true;
    MAV_RESULT ret = GCS_MAVLINK::handle_command_preflight_calibration(packet, msg);
    plane.in_calibration = false;

    return ret;
}

// 处理接收到的数据包
void GCS_MAVLINK_Plane::packetReceived(const mavlink_status_t &status,
                                       const mavlink_message_t &msg)
{
#if HAL_ADSB_ENABLED
    plane.avoidance_adsb.handle_msg(msg);
#endif
#if AP_SCRIPTING_ENABLED && AP_FOLLOW_ENABLED
    // 将消息传递给follow库
    plane.g2.follow.handle_msg(msg);
#endif
    GCS_MAVLINK::packetReceived(status, msg);
}
// 将当前位置设置为新的Home点
bool Plane::set_home_to_current_location(bool _lock)
{
    // 尝试将当前GPS位置设置为新的Home点
    if (!set_home_persistently(AP::gps().location())) {
        return false;
    }
    // 如果需要，锁定Home点
    if (_lock) {
        AP::ahrs().lock_home();
    }
    // 如果当前模式是RTL或QRTL，重新进入该模式以更新目标位置
    if ((control_mode == &mode_rtl)
#if HAL_QUADPLANE_ENABLED
            || (control_mode == &mode_qrtl)
#endif
                                                        ) {
        control_mode->enter();
    }
    return true;
}

// 将指定位置设置为新的Home点
bool Plane::set_home(const Location& loc, bool _lock)
{
    // 尝试设置新的Home点
    if (!AP::ahrs().set_home(loc)) {
        return false;
    }
    // 如果需要，锁定Home点
    if (_lock) {
        AP::ahrs().lock_home();
    }
    // 如果当前模式是RTL或QRTL，重新进入该模式以更新目标位置
    if ((control_mode == &mode_rtl)
#if HAL_QUADPLANE_ENABLED
            || (control_mode == &mode_qrtl)
#endif
                                                        ) {
        control_mode->enter();
    }
    return true;
}

// 处理MAV_CMD_DO_REPOSITION命令
MAV_RESULT GCS_MAVLINK_Plane::handle_command_int_do_reposition(const mavlink_command_int_t &packet)
{
    // 检查经纬度是否有效
    if (!check_latlng(packet.x, packet.y)) {
        return MAV_RESULT_DENIED;
    }

    Location requested_position;
    // 从命令包中提取位置信息
    if (!location_from_command_t(packet, requested_position)) {
        return MAV_RESULT_DENIED;
    }

    // 设置盘旋方向
    if (isnan(packet.param4) || is_zero(packet.param4)) {
        requested_position.loiter_ccw = 0;
    } else {
        requested_position.loiter_ccw = 1;
    }

    // 检查位置是否合理
    if (requested_position.sanitize(plane.current_loc)) {
        return MAV_RESULT_DENIED;
    }

#if AP_FENCE_ENABLED
    // 检查目标位置是否在围栏内
    if (!plane.fence.check_destination_within_fence(requested_position)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        return MAV_RESULT_DENIED;
    }
#endif

    // 如果需要改变模式或当前已经是GUIDED模式，则设置新的引导目标
    if (((int32_t)packet.param2 & MAV_DO_REPOSITION_FLAGS_CHANGE_MODE) ||
        (plane.control_mode == &plane.mode_guided)) {
        plane.set_mode(plane.mode_guided, ModeReason::GCS_COMMAND);

        // 如果是相对高度，加上Home点高度
        if (requested_position.relative_alt) {
            requested_position.alt += plane.home.alt;
            requested_position.relative_alt = 0;
        }

        // 设置引导模式的目标航点
        plane.set_guided_WP(requested_position);

        // 设置盘旋半径（如果提供）
        if (!isnan(packet.param3) && packet.param3 > 0) {
            plane.mode_guided.set_radius_and_direction(packet.param3, requested_position.loiter_ccw);
        }

        return MAV_RESULT_ACCEPTED;
    }
    return MAV_RESULT_FAILED;
}

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
// 处理GUIDED模式下的速度、高度和航向渐变命令
MAV_RESULT GCS_MAVLINK_Plane::handle_command_int_guided_slew_commands(const mavlink_command_int_t &packet)
{
  switch(packet.command) {
    case MAV_CMD_GUIDED_CHANGE_SPEED: {
        // 仅在GUIDED模式下有效
        if (plane.control_mode != &plane.mode_guided) {
            return MAV_RESULT_FAILED;
        }

        // 目前只支持空速命令
        if (int(packet.param1) != SPEED_TYPE_AIRSPEED) {
            return MAV_RESULT_DENIED;
        }

        // 拒绝超出调整范围的空速
        if (packet.param2 > plane.aparm.airspeed_max || packet.param2 < plane.aparm.airspeed_min) {
            return MAV_RESULT_DENIED;
        }

        // 如果目标空速未变，无需处理
        float new_target_airspeed_cm = packet.param2 * 100;
        if (is_equal(new_target_airspeed_cm, plane.guided_state.target_airspeed_cm)) { 
            return MAV_RESULT_ACCEPTED;
        }
        plane.guided_state.target_airspeed_cm = new_target_airspeed_cm;
        plane.guided_state.target_airspeed_time_ms = AP_HAL::millis();

        // 设置加速度
        if (is_zero(packet.param3)) {
            plane.guided_state.target_airspeed_accel = 1000.0f;
        } else {
            plane.guided_state.target_airspeed_accel = fabsf(packet.param3);
        }

        // 设置加速度方向
        if (plane.guided_state.target_airspeed_cm < plane.target_airspeed_cm) {
            plane.guided_state.target_airspeed_accel *= -1.0f;
        }
        return MAV_RESULT_ACCEPTED;
    }

    case MAV_CMD_GUIDED_CHANGE_ALTITUDE: {
        // 仅在GUIDED模式下有效
        if (plane.control_mode != &plane.mode_guided) {
            return MAV_RESULT_FAILED;
        }

        // 拒绝无效的高度值
        if (is_equal(packet.z, -1.0f) || is_equal(packet.z, 0.0f)){
            return MAV_RESULT_DENIED;
        }

        // 转换高度框架
        Location::AltFrame new_target_alt_frame;
        if (!mavlink_coordinate_frame_to_location_alt_frame((MAV_FRAME)packet.frame, new_target_alt_frame)) {
            return MAV_RESULT_DENIED;
        }
        plane.guided_state.target_mav_frame = packet.frame;

        // 设置目标高度
        const int32_t new_target_alt_cm = packet.z * 100;
        plane.guided_state.target_location.set_alt_cm(new_target_alt_cm, new_target_alt_frame); 
        plane.guided_state.target_alt_time_ms = AP_HAL::millis();

        // 设置垂直速率
        if (is_zero(packet.param3)) {
            plane.guided_state.target_alt_rate = 1000.0;
        } else {
            plane.guided_state.target_alt_rate = fabsf(packet.param3);
        }

        return MAV_RESULT_ACCEPTED;
    }

    case MAV_CMD_GUIDED_CHANGE_HEADING: {
        // 仅在GUIDED模式下有效
        if (plane.control_mode != &plane.mode_guided) {
            return MAV_RESULT_FAILED;
        }

        // 拒绝超出[0-360]度范围的航向
        if (packet.param2 < 0.0f || packet.param2 >= 360.0f) {
            return MAV_RESULT_DENIED;
        }

        float new_target_heading = radians(wrap_180(packet.param2));

        // 设置航向类型（地面航向或飞机航向）
        if (int(packet.param1) == HEADING_TYPE_COURSE_OVER_GROUND) {
            plane.guided_state.target_heading_type = GUIDED_HEADING_COG;
            plane.prev_WP_loc = plane.current_loc;
        } else if (int(packet.param1) == HEADING_TYPE_HEADING) {
            plane.guided_state.target_heading_type = GUIDED_HEADING_HEADING;
        } else {
            return MAV_RESULT_DENIED;
        }

        // 重置积分器
        plane.g2.guidedHeading.reset_I();

        // 设置目标航向和相关参数
        plane.guided_state.target_heading = new_target_heading;
        plane.guided_state.target_heading_accel_limit = MAX(packet.param3, 0.05f);
        plane.guided_state.target_heading_time_ms = AP_HAL::millis();
        return MAV_RESULT_ACCEPTED;
    }
  }
  // 不支持的命令
  return MAV_RESULT_UNSUPPORTED;
}
#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
MAV_RESULT GCS_MAVLINK_Plane::handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    switch(packet.command) {

    case MAV_CMD_DO_AUTOTUNE_ENABLE:
        return handle_MAV_CMD_DO_AUTOTUNE_ENABLE(packet);

    case MAV_CMD_DO_REPOSITION:
        return handle_command_int_do_reposition(packet);

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // 特殊的'滑移启用'引导命令，用于速度、高度和方向命令
    case MAV_CMD_GUIDED_CHANGE_SPEED:
    case MAV_CMD_GUIDED_CHANGE_ALTITUDE:
    case MAV_CMD_GUIDED_CHANGE_HEADING:
        return handle_command_int_guided_slew_commands(packet);
#endif

#if AP_SCRIPTING_ENABLED && AP_FOLLOW_ENABLED
    case MAV_CMD_DO_FOLLOW:
        // param1: 要跟随的目标的系统ID
        if ((packet.param1 > 0) && (packet.param1 <= 255)) {
            plane.g2.follow.set_target_sysid((uint8_t)packet.param1);
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_DENIED;
#endif

#if AP_ICENGINE_ENABLED
    case MAV_CMD_DO_ENGINE_CONTROL:
        if (!plane.g2.ice_control.engine_control(packet.param1, packet.param2, packet.param3, (uint32_t)packet.param4)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;
#endif

    case MAV_CMD_DO_CHANGE_SPEED:
        return handle_command_DO_CHANGE_SPEED(packet);

#if HAL_PARACHUTE_ENABLED
    case MAV_CMD_DO_PARACHUTE:
        return handle_MAV_CMD_DO_PARACHUTE(packet);
#endif

#if HAL_QUADPLANE_ENABLED
    case MAV_CMD_DO_MOTOR_TEST:
        return handle_MAV_CMD_DO_MOTOR_TEST(packet);

    case MAV_CMD_DO_VTOL_TRANSITION:
        return handle_command_DO_VTOL_TRANSITION(packet);

    case MAV_CMD_NAV_TAKEOFF:
        return handle_command_MAV_CMD_NAV_TAKEOFF(packet);
#endif

    case MAV_CMD_DO_GO_AROUND:
        return plane.trigger_land_abort(packet.param1) ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;

    case MAV_CMD_DO_LAND_START:
        // 尝试切换到任务中的下一个DO_LAND_START命令
        if (plane.have_position && plane.mission.jump_to_landing_sequence(plane.current_loc)) {
            plane.set_mode(plane.mode_auto, ModeReason::GCS_COMMAND);
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;

    case MAV_CMD_MISSION_START:
        if (!is_zero(packet.param1) || !is_zero(packet.param2)) {
            // 不支持第一项/最后一项
            return MAV_RESULT_DENIED;
        }
        plane.set_mode(plane.mode_auto, ModeReason::GCS_COMMAND);
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_NAV_LOITER_UNLIM:
        plane.set_mode(plane.mode_loiter, ModeReason::GCS_COMMAND);
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        plane.set_mode(plane.mode_rtl, ModeReason::GCS_COMMAND);
        return MAV_RESULT_ACCEPTED;

#if AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED
    case MAV_CMD_SET_HAGL:
        plane.handle_external_hagl(packet);
        return MAV_RESULT_ACCEPTED;
#endif
        
    default:
        return GCS_MAVLINK::handle_command_int_packet(packet, msg);
    }
}

MAV_RESULT GCS_MAVLINK_Plane::handle_command_DO_CHANGE_SPEED(const mavlink_command_int_t &packet)
{
    // 如果我们处于故障保护模式（例如，RTL，LOITER）或飞行员控制模式（例如，MANUAL，TRAINING）
    // 这个命令应该被忽略，因为它来自GCS或伴随计算机：
    if ((!plane.control_mode->is_guided_mode()) &&
        (plane.control_mode != &plane.mode_auto)) {
        // 失败
        return MAV_RESULT_FAILED;
    }

    if (plane.do_change_speed(packet.param1, packet.param2, packet.param3)) {
        return MAV_RESULT_ACCEPTED;
    }
    return MAV_RESULT_FAILED;
}

#if HAL_QUADPLANE_ENABLED
#if AP_MAVLINK_COMMAND_LONG_ENABLED
void GCS_MAVLINK_Plane::convert_MAV_CMD_NAV_TAKEOFF_to_COMMAND_INT(const mavlink_command_long_t &in, mavlink_command_int_t &out)
{
    // 转换为MAV_FRAME_LOCAL_OFFSET_NED，"以车辆为原点的NED局部切线坐标系"
    out = {};
    out.target_system = in.target_system;
    out.target_component = in.target_component;
    out.frame = MAV_FRAME_LOCAL_OFFSET_NED;
    out.command = in.command;
    // out.current = 0;
    // out.autocontinue = 0;
    // out.param1 = in.param1;  // 我们在这个命令中只使用"z"参数：
    // out.param2 = in.param2;
    // out.param3 = in.param3;
    // out.param4 = in.param4;
    // out.x = 0;  // 我们在起飞时不处理定位
    // out.y = 0;
    out.z = -in.param7;  // 上 -> 下
}

void GCS_MAVLINK_Plane::convert_COMMAND_LONG_to_COMMAND_INT(const mavlink_command_long_t &in, mavlink_command_int_t &out, MAV_FRAME frame)
{
    switch (in.command) {
    case MAV_CMD_NAV_TAKEOFF:
        convert_MAV_CMD_NAV_TAKEOFF_to_COMMAND_INT(in, out);
        return;
    }
    return GCS_MAVLINK::convert_COMMAND_LONG_to_COMMAND_INT(in, out, frame);
}
#endif  // AP_MAVLINK_COMMAND_LONG_ENABLED

MAV_RESULT GCS_MAVLINK_Plane::handle_command_MAV_CMD_NAV_TAKEOFF(const mavlink_command_int_t &packet)
{
    float takeoff_alt = packet.z;
    switch (packet.frame) {
    case MAV_FRAME_LOCAL_OFFSET_NED:  // "以车辆为原点的NED局部切线坐标系"
        takeoff_alt = -takeoff_alt;  // 下 -> 上
        break;
    default:
        return MAV_RESULT_DENIED; // "支持但参数无效"
    }
    if (!plane.quadplane.available()) {
        return MAV_RESULT_FAILED;
    }
    if (!plane.quadplane.do_user_takeoff(takeoff_alt)) {
        return MAV_RESULT_FAILED;
    }
    return MAV_RESULT_ACCEPTED;
}
#endif

MAV_RESULT GCS_MAVLINK_Plane::handle_MAV_CMD_DO_AUTOTUNE_ENABLE(const mavlink_command_int_t &packet)
{
    // param1 : 启用/禁用
    plane.autotune_enable(!is_zero(packet.param1));
    return MAV_RESULT_ACCEPTED;
}

#if HAL_PARACHUTE_ENABLED
MAV_RESULT GCS_MAVLINK_Plane::handle_MAV_CMD_DO_PARACHUTE(const mavlink_command_int_t &packet)
{
    // 配置或释放降落伞
    switch ((uint16_t)packet.param1) {
    case PARACHUTE_DISABLE:
        plane.parachute.enabled(false);
        return MAV_RESULT_ACCEPTED;
    case PARACHUTE_ENABLE:
        plane.parachute.enabled(true);
        return MAV_RESULT_ACCEPTED;
    case PARACHUTE_RELEASE:
        // 视为手动释放，执行一些额外的高度检查
        if (plane.parachute.released()) {
            gcs().send_text(MAV_SEVERITY_NOTICE, "降落伞已释放");
            return MAV_RESULT_FAILED;
        }
        if (!plane.parachute.enabled()) {
            gcs().send_text(MAV_SEVERITY_NOTICE, "降落伞未启用");
            return MAV_RESULT_FAILED;
        }
        if (!plane.parachute_manual_release()) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;
    default:
        break;
    }
    return MAV_RESULT_FAILED;
}
#endif
#if HAL_QUADPLANE_ENABLED
MAV_RESULT GCS_MAVLINK_Plane::handle_MAV_CMD_DO_MOTOR_TEST(const mavlink_command_int_t &packet)
{
    // param1 : 电机序列号（从1到车辆最大电机数的数字）
    // param2 : 油门类型（0=油门百分比，1=PWM，2=飞行员油门通道直通。参见MOTOR_TEST_THROTTLE_TYPE枚举）
    // param3 : 油门（范围取决于param2）
    // param4 : 超时（以秒为单位）
    // param5 : 电机数量（要按顺序测试的电机数量）
    return plane.quadplane.mavlink_motor_test_start(chan,
                                                    (uint8_t)packet.param1,
                                                    (uint8_t)packet.param2,
                                                    (uint16_t)packet.param3,
                                                    packet.param4,
                                                    (uint8_t)packet.x);
}

MAV_RESULT GCS_MAVLINK_Plane::handle_command_DO_VTOL_TRANSITION(const mavlink_command_int_t &packet)
{
    // 处理VTOL转换命令
    if (!plane.quadplane.handle_do_vtol_transition((enum MAV_VTOL_STATE)packet.param1)) {
        return MAV_RESULT_FAILED;
    }
    return MAV_RESULT_ACCEPTED;
}
#endif

// 这个函数在收到MANUAL_CONTROL数据包时被调用，
// 预期会调用manual_override来覆盖所需轴上的RC输入。
void GCS_MAVLINK_Plane::handle_manual_control_axes(const mavlink_manual_control_t &packet, const uint32_t tnow)
{
    manual_override(plane.channel_roll, packet.y, 1000, 2000, tnow);
    manual_override(plane.channel_pitch, packet.x, 1000, 2000, tnow, true);
    manual_override(plane.channel_throttle, packet.z, 0, 1000, tnow);
    manual_override(plane.channel_rudder, packet.r, 1000, 2000, tnow);
}

void GCS_MAVLINK_Plane::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {

    case MAVLINK_MSG_ID_TERRAIN_DATA:
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
#if AP_TERRAIN_AVAILABLE
        plane.terrain.handle_data(chan, msg);
#endif
        break;

    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
        handle_set_attitude_target(msg);
        break;

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
        handle_set_position_target_local_ned(msg);
        break;

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
        handle_set_position_target_global_int(msg);
        break;

    default:
        GCS_MAVLINK::handle_message(msg);
        break;
    } // end switch
} // end handle mavlink

void GCS_MAVLINK_Plane::handle_set_attitude_target(const mavlink_message_t &msg)
{
    // 只允许伴随计算机（或其他外部控制器）在GUIDED模式下控制姿态。
    // 我们不希望在例如RTL、CICLE等模式下进行外部控制。
    // 为伴随计算机控制指定单一模式更安全（在使用FENCE_ACTION = 4进行地理围栏失败时更是如此）。
    if (plane.control_mode != &plane.mode_guided) { // 不要破坏故障保护
        return;
    }

    mavlink_set_attitude_target_t att_target;
    mavlink_msg_set_attitude_target_decode(&msg, &att_target);

    // 映射：如果设置了这些位中的任何一个，相应的输入应被忽略。
    // 注意，在解析位时我们将它们反转以便更容易解释，但传输时它们是反转的
    // bit 1: 机体横滚速率
    // bit 2: 机体俯仰速率
    // bit 3: 机体偏航速率
    // bit 4: 未知
    // bit 5: 未知
    // bit 6: 保留
    // bit 7: 油门
    // bit 8: 姿态

    // 如果不设置所有四元数值，使用_rate标志来指示哪些字段。

    // 从四元数中提取欧拉横滚角。
    Quaternion q(att_target.q[0], att_target.q[1],
            att_target.q[2], att_target.q[3]);

    // 注意：att_target.type_mask被反转以便更容易解释
    att_target.type_mask = att_target.type_mask ^ 0xFF;

    uint8_t attitude_mask = att_target.type_mask & 0b10000111; // q加上rpy

    uint32_t now = AP_HAL::millis();
    if ((attitude_mask & 0b10000001) ||    // 部分，包括横滚
            (attitude_mask == 0b10000000)) { // 所有角度
        plane.guided_state.forced_rpy_cd.x = degrees(q.get_euler_roll()) * 100.0f;

        // 更新外部横滚到导航控制的计时器
        plane.guided_state.last_forced_rpy_ms.x = now;
    }

    if ((attitude_mask & 0b10000010) ||    // 部分，包括俯仰
            (attitude_mask == 0b10000000)) { // 所有角度
        plane.guided_state.forced_rpy_cd.y = degrees(q.get_euler_pitch()) * 100.0f;

        // 更新外部俯仰到导航控制的计时器
        plane.guided_state.last_forced_rpy_ms.y = now;
    }

    if ((attitude_mask & 0b10000100) ||    // 部分，包括偏航
            (attitude_mask == 0b10000000)) { // 所有角度
        plane.guided_state.forced_rpy_cd.z = degrees(q.get_euler_yaw()) * 100.0f;

        // 更新外部偏航到导航控制的计时器
        plane.guided_state.last_forced_rpy_ms.z = now;
    }
    if (att_target.type_mask & 0b01000000) { // 油门
        plane.guided_state.forced_throttle = att_target.thrust * 100.0f;

        // 更新外部油门的计时器
        plane.guided_state.last_forced_throttle_ms = now;
    }
}

void GCS_MAVLINK_Plane::handle_set_position_target_local_ned(const mavlink_message_t &msg)
{
    // 解码数据包
    mavlink_set_position_target_local_ned_t packet;
    mavlink_msg_set_position_target_local_ned_decode(&msg, &packet);

    // 如果车辆不在Guided模式，则退出
    if (plane.control_mode != &plane.mode_guided) {
        return;
    }

    // 目前只处理本地移动
    if (packet.coordinate_frame != MAV_FRAME_LOCAL_OFFSET_NED) {
        return;
    }

    // 目前只处理高度
    plane.next_WP_loc.alt += -packet.z*100.0;
    gcs().send_text(MAV_SEVERITY_INFO, "Change alt to %.1f",
                    (double)((plane.next_WP_loc.alt - plane.home.alt)*0.01));
}

void GCS_MAVLINK_Plane::handle_set_position_target_global_int(const mavlink_message_t &msg)
{
    // 只希望在特定模式下允许伴随计算机进行位置控制，
    // 以避免在自动驾驶仪响应RTL、CIRCLE等模式中的问题时
    // 无意中发送这些类型的命令。
    // 为伴随计算机控制指定只有一种模式更安全
    // （前提是使用FENCE_ACTION = 4（RTL）处理地理围栏故障）。
    if (plane.control_mode != &plane.mode_guided) {
        // 不要破坏故障保护
        return;
    }

    mavlink_set_position_target_global_int_t pos_target;
    mavlink_msg_set_position_target_global_int_decode(&msg, &pos_target);
    // 出乎意料的是，掩码期望对应该被忽略而不是包含的维度使用"1"。
    // 参见SET_POSITION_TARGET_GLOBAL_INT消息的mavlink文档中的type_mask字段。
    const uint16_t alt_mask = 0b1111111111111011; // （z掩码在第3位）
        
    bool msg_valid = true;
    AP_Mission::Mission_Command cmd = {0};
    
    if (pos_target.type_mask & alt_mask)
    {
        cmd.content.location.alt = pos_target.alt * 100;
        cmd.content.location.relative_alt = false;
        cmd.content.location.terrain_alt = false;
        switch (pos_target.coordinate_frame) 
        {
            case MAV_FRAME_GLOBAL:
            case MAV_FRAME_GLOBAL_INT:
                break; // 默认为MSL高度
            case MAV_FRAME_GLOBAL_RELATIVE_ALT:
            case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
                cmd.content.location.relative_alt = true;
                break;
            case MAV_FRAME_GLOBAL_TERRAIN_ALT:
            case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
                cmd.content.location.relative_alt = true;
                cmd.content.location.terrain_alt = true;
                break;
            default:
                gcs().send_text(MAV_SEVERITY_WARNING, "Invalid coord frame in SET_POSTION_TARGET_GLOBAL_INT");
                msg_valid = false;
                break;
        }    

        if (msg_valid) {
            handle_change_alt_request(cmd);
        }
    } // end if alt_mask       
}

MAV_RESULT GCS_MAVLINK_Plane::handle_command_do_set_mission_current(const mavlink_command_int_t &packet)
{
    const MAV_RESULT result = GCS_MAVLINK::handle_command_do_set_mission_current(packet);
    if (result != MAV_RESULT_ACCEPTED) {
        return result;
    }

    // 如果你改变这里，你必须改变handle_mission_set_current
    plane.auto_state.next_wp_crosstrack = false;
    if (plane.control_mode == &plane.mode_auto && plane.mission.state() == AP_Mission::MISSION_STOPPED) {
        plane.mission.resume();
    }

    return result;
}

#if AP_MAVLINK_MISSION_SET_CURRENT_ENABLED
// 处理设置当前任务的消息
void GCS_MAVLINK_Plane::handle_mission_set_current(AP_Mission &mission, const mavlink_message_t &msg)
{
    // 如果你改变这里，你必须改变handle_command_do_set_mission_current
    plane.auto_state.next_wp_crosstrack = false;
    GCS_MAVLINK::handle_mission_set_current(mission, msg);
    if (plane.control_mode == &plane.mode_auto && plane.mission.state() == AP_Mission::MISSION_STOPPED) {
        plane.mission.resume();
    }
}
#endif

// 返回飞机的能力
uint64_t GCS_MAVLINK_Plane::capabilities() const
{
    return (MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
            MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
            MAV_PROTOCOL_CAPABILITY_MISSION_INT |
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
            MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
#if AP_TERRAIN_AVAILABLE
            (plane.terrain.enabled() ? MAV_PROTOCOL_CAPABILITY_TERRAIN : 0) |
#endif
            GCS_MAVLINK::capabilities());
}

#if HAL_HIGH_LATENCY2_ENABLED
// 返回高延迟模式下的目标高度
int16_t GCS_MAVLINK_Plane::high_latency_target_altitude() const
{
    AP_AHRS &ahrs = AP::ahrs();
    Location global_position_current;
    UNUSED_RESULT(ahrs.get_location(global_position_current));

#if HAL_QUADPLANE_ENABLED
    const QuadPlane &quadplane = plane.quadplane;
    // 返回单位为米
    if (quadplane.show_vtol_view()) {
        return (plane.control_mode != &plane.mode_qstabilize) ? 0.01 * (global_position_current.alt + quadplane.pos_control->get_pos_error_z_cm()) : 0;
    }
#endif
    return 0.01 * (global_position_current.alt + plane.calc_altitude_error_cm());
}

// 返回高延迟模式下的目标航向
uint8_t GCS_MAVLINK_Plane::high_latency_tgt_heading() const
{
    // 返回单位为度/2
#if HAL_QUADPLANE_ENABLED
    const QuadPlane &quadplane = plane.quadplane;
    if (quadplane.show_vtol_view()) {
        const Vector3f &targets = quadplane.attitude_control->get_att_target_euler_cd();
        return ((uint16_t)(targets.z * 0.01)) / 2;
    }
#endif
        const AP_Navigation *nav_controller = plane.nav_controller;
        // 需要将-18000->18000转换为0->360/2
        return wrap_360_cd(nav_controller->target_bearing_cd() ) / 200;
}

// 返回高延迟模式下的目标距离，单位为分米
uint16_t GCS_MAVLINK_Plane::high_latency_tgt_dist() const
{
#if HAL_QUADPLANE_ENABLED
    const QuadPlane &quadplane = plane.quadplane;
    if (quadplane.show_vtol_view()) {
        bool wp_nav_valid = quadplane.using_wp_nav();
        return (wp_nav_valid ? MIN(quadplane.wp_nav->get_wp_distance_to_destination(), UINT16_MAX) : 0) / 10;
    }
    #endif

    return MIN(plane.auto_state.wp_distance, UINT16_MAX) / 10;
}

// 返回高延迟模式下的目标空速
uint8_t GCS_MAVLINK_Plane::high_latency_tgt_airspeed() const
{
    // 返回单位为m/s*5
    return plane.target_airspeed_cm * 0.05;
}

// 返回高延迟模式下的风速
uint8_t GCS_MAVLINK_Plane::high_latency_wind_speed() const
{
    Vector3f wind;
    wind = AP::ahrs().wind_estimate();

    // 返回单位为m/s*5
    return MIN(wind.length() * 5, UINT8_MAX);
}

// 返回高延迟模式下的风向
uint8_t GCS_MAVLINK_Plane::high_latency_wind_direction() const
{
    const Vector3f wind = AP::ahrs().wind_estimate();

    // 返回单位为度/2
    // 需要将-180->180转换为0->360/2
    return wrap_360(degrees(atan2f(-wind.y, -wind.x))) / 2;
}
#endif // HAL_HIGH_LATENCY2_ENABLED

// 返回VTOL状态
MAV_VTOL_STATE GCS_MAVLINK_Plane::vtol_state() const
{
#if !HAL_QUADPLANE_ENABLED
    return MAV_VTOL_STATE_UNDEFINED;
#else
    if (!plane.quadplane.available()) {
        return MAV_VTOL_STATE_UNDEFINED;
    }

    return plane.quadplane.transition->get_mav_vtol_state();
#endif
};

// 返回着陆状态
MAV_LANDED_STATE GCS_MAVLINK_Plane::landed_state() const
{
    if (plane.is_flying()) {
        // 注意Q模式几乎总是认为自己在飞行
        return MAV_LANDED_STATE_IN_AIR;
    }

    return MAV_LANDED_STATE_ON_GROUND;
}
