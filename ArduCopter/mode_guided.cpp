/*
ArduCopter的GUIDED模式介绍
GUIDED模式是ArduCopter飞控系统中的一个重要功能，主要用于自动导航或通过地面控制站实时控制无人机。
该模式允许用户通过地面控制站（如Mission Planner或QGroundControl）发送实时命令，控制无人机飞往特定的GPS坐标，
或者执行其他动作（如悬停、改变高度、跟随目标等）。

功能特点：
实时控制：用户可以通过地面控制站或者外部API（如MAVLink协议）实时发送位置、速度、方向等命令，控制无人机执行相应的飞行动作。
导航到特定位置：用户可以发送一个指定的GPS坐标，无人机会自动导航到该坐标并悬停。
速度控制：除了导航到目标位置，用户还可以设置飞行速度。
目标跟踪：支持基于MAVLink发送的目标数据进行目标跟踪（如移动物体的追踪）。
高度控制：在GUIDED模式下，用户可以指定无人机飞行到特定的高度，并保持稳定飞行。
自动悬停：到达指定位置或停止命令后，无人机会自动悬停在当前位置。
GUIDED模式的实用方法
1. 启动GUIDED模式
通过地面站（Mission Planner、QGroundControl等）或遥控器，将飞控模式切换为GUIDED模式。也可以通过MAVLink命令实现切换，例如：

cpp
复制代码
mavlink_set_mode_message(sys_id, comp_id, GUIDED);
地面站会显示无人机当前的模式状态，并确认已经成功切换到GUIDED模式。

2. 发送导航命令
在GUIDED模式下，用户可以通过MAVLink命令或地面站GUI发送导航命令，控制无人机飞往指定的GPS位置。
常用的MAVLink命令是MAV_CMD_NAV_WAYPOINT或MAV_CMD_NAV_LOITER_TIME。

例如，使用MAVLink协议发送一个导航到目标点（例如经纬度和高度）的命令：

cpp
复制代码
mavlink_command_long_send(
    system_id, component_id,
    MAV_CMD_NAV_WAYPOINT,  // 命令类型为导航到航点
    0,                     // 确认
    latitude,              // 目标点的纬度
    longitude,             // 目标点的经度
    altitude,              // 目标点的高度
    0, 0, 0, 0);           // 其他参数可以根据需要设置
3. 控制无人机的速度
GUIDED模式支持通过MAVLink命令设置无人机的速度。MAV_CMD_DO_CHANGE_SPEED命令允许用户动态调整飞行速度。例如：

cpp
复制代码
mavlink_command_long_send(
    system_id, component_id,
    MAV_CMD_DO_CHANGE_SPEED,
    0,
    1,       // 设置速度类型（1表示地速）
    speed,   // 目标速度（m/s）
    0, 0, 0, 0, 0, 0);
4. 目标跟踪
GUIDED模式支持使用外部系统（如无人机目标追踪系统）发送目标坐标，使无人机跟随一个动态目标。
例如，发送MAVLink命令MAV_CMD_DO_SET_GUIDED_SUBMODE来激活跟随目标模式。

此外，通过发送不断更新的GPS坐标，GUIDED模式中的无人机可以实时跟踪目标的移动。

5. 高度控制
在GUIDED模式下，用户可以发送一个目标高度，无人机会自动调整高度至目标值并保持。例如，使用MAV_CMD_NAV_TAKEOFF命令可以指示无人机起飞到特定高度：

cpp
复制代码
mavlink_command_long_send(
    system_id, component_id,
    MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0,
    latitude,  // 起飞位置的纬度
    longitude, // 起飞位置的经度
    altitude   // 目标高度
);
6. 自动悬停
当无人机到达目标位置后，GUIDED模式会自动让无人机悬停。如果用户需要在特定时间点或执行完某个命令后让无人机保持悬停状态，
可以发送零速度指令或者直接停止更新位置命令，无人机会在当前位置保持悬停。

实践中的常见使用场景
实时飞行控制：通过GUIDED模式，地面控制人员可以随时改变无人机的飞行路径，用于探索或任务需求的临时调整。
自主导航任务：例如无人机的包裹递送，用户可以实时发送目标位置来控制无人机按需求到达不同地点。
目标追踪：用于自动跟踪移动目标（如救援场景中的人员定位，或者动态物体的监控任务）。
精准降落：结合其他传感器（如视觉或激光雷达），GUIDED模式可以用于无人机的精准着陆或在狭小空间中的导航。
需要注意的事项
GPS信号质量：GUIDED模式下主要依赖GPS导航，因此在弱GPS信号或无信号的情况下，不建议使用GUIDED模式。
通信链路稳定性：GUIDED模式需要与地面控制站保持通信，因此确保通信链路的稳定性十分重要。
安全性：确保无人机的飞行范围在安全区域内，避免误操作导致无人机飞入危险区域。
电池管理：长时间的自动导航任务会消耗大量电力，建议在GUIDED模式执行任务时密切关注电池电量。
总结
GUIDED模式为ArduCopter提供了强大的实时控制和导航能力，尤其适用于需要远程操作和自主导航的场景。
通过与MAVLink协议配合，GUIDED模式可以实现精确的飞行控制、目标追踪和任务执行，是无人机任务执行中不可或缺的功能。 */




#include "Copter.h"

#if MODE_GUIDED_ENABLED

/*
 * Init and run calls for guided flight mode
 * 引导飞行模式的初始化和运行调用
 */

// 位置目标(仅用于posvel控制器)
static Vector3p guided_pos_target_cm;       // position target (used by posvel controller only)
// guided_pos_target_cm.z是否为地形高度
bool guided_pos_terrain_alt;                // true if guided_pos_target_cm.z is an alt above terrain
// 速度目标(用于pos_vel_accel控制器和vel_accel控制器)
static Vector3f guided_vel_target_cms;      // velocity target (used by pos_vel_accel controller and vel_accel controller)
// 加速度目标(用于pos_vel_accel控制器、vel_accel控制器和accel控制器)
static Vector3f guided_accel_target_cmss;   // acceleration target (used by pos_vel_accel controller vel_accel controller and accel controller)
// pos_vel_accel、vel_accel或accel控制器的最后目标更新时间
static uint32_t update_time_ms;             // system time of last target update to pos_vel_accel, vel_accel or accel controller

// 引导角度状态结构体
struct {
    uint32_t update_time_ms;        // 更新时间
    Quaternion attitude_quat;       // 姿态四元数
    Vector3f ang_vel_body;          // 机体坐标系下的角速度
    float yaw_rate_cds;             // 偏航角速度(厘度/秒)
    float climb_rate_cms;           // 爬升速率(厘米/秒)。当use_thrust为false时使用
    float thrust;                   // 推力，范围从-1到1。当use_thrust为true时使用
    bool use_yaw_rate;              // 是否使用偏航角速度
    bool use_thrust;                // 是否使用推力
} static guided_angle_state;

// 引导限制结构体
struct Guided_Limit {
    uint32_t timeout_ms;  // 从引导模式被调用时开始的超时时间(秒)
    float alt_min_cm;     // 相对于home点的最低高度限制(厘米)(0 = 无限制)
    float alt_max_cm;     // 相对于home点的最高高度限制(厘米)(0 = 无限制)
    float horiz_max_cm;   // 从引导模式开始位置的水平位置限制(厘米)(0 = 无限制)
    uint32_t start_time;  // 控制权交给外部计算机的系统时间(毫秒)
    Vector3f start_pos;   // 开始位置，相对于home点的距离(厘米)。用于检查horiz_max限制
} guided_limit;

// init - 初始化引导控制器
bool ModeGuided::init(bool ignore_checks)
{
    // 以velaccel控制模式开始
    velaccel_control_start();
    guided_vel_target_cms.zero();
    guided_accel_target_cmss.zero();
    send_notification = false;

    // 进入引导模式时清除暂停状态
    _paused = false;

    return true;
}

// run - 运行引导控制器
// 应该以100Hz或更高的频率调用
void ModeGuided::run()
{
    // 如果飞行器处于暂停状态，运行暂停控制
    if (_paused) {
        pause_control_run();
        return;
    }

    // 调用正确的自动控制器
    switch (guided_mode) {

    case SubMode::TakeOff:
        // 运行起飞控制器
        takeoff_run();
        break;

    case SubMode::WP:
        // 运行航点控制器
        wp_control_run();
        if (send_notification && wp_nav->reached_wp_destination()) {
            send_notification = false;
            gcs().send_mission_item_reached_message(0);
        }
        break;

    case SubMode::Pos:
        // 运行位置控制器
        pos_control_run();
        break;

    case SubMode::Accel:
        accel_control_run();
        break;

    case SubMode::VelAccel:
        velaccel_control_run();
        break;

    case SubMode::PosVelAccel:
        posvelaccel_control_run();
        break;

    case SubMode::Angle:
        angle_control_run();
        break;
    }
 }

// 返回Guided模式选项是否启用(参见GUID_OPTIONS)
bool ModeGuided::option_is_enabled(Option option) const
{
    return (copter.g2.guided_options.get() & (uint32_t)option) != 0;
}

// 检查是否允许解锁
bool ModeGuided::allows_arming(AP_Arming::Method method) const
{
    // 始终允许从地面站或脚本解锁
    if (AP_Arming::method_is_GCS(method) || method == AP_Arming::Method::SCRIPTING) {
        return true;
    }

    // 可选择性地允许从发射器解锁
    return option_is_enabled(Option::AllowArmingFromTX);
};

#if WEATHERVANE_ENABLED
// 检查是否允许风向标功能
bool ModeGuided::allows_weathervaning() const
{
    return option_is_enabled(Option::AllowWeatherVaning);
}
#endif

// 初始化位置控制器以实现起飞
// takeoff_alt_cm被解释为相对于home点的高度(厘米)，如果有测距仪可用则为相对于地形的高度
bool ModeGuided::do_user_takeoff_start(float takeoff_alt_cm)
{
    // 计算目标高度和参考框架(相对于EKF原点的高度或相对于地形的高度)
    int32_t alt_target_cm;
    bool alt_target_terrain = false;
#if AP_RANGEFINDER_ENABLED
    if (wp_nav->rangefinder_used_and_healthy() &&
        wp_nav->get_terrain_source() == AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER &&
        takeoff_alt_cm < copter.rangefinder.max_distance_cm_orient(ROTATION_PITCH_270)) {
        // 不能向下起飞
        if (takeoff_alt_cm <= copter.rangefinder_state.alt_cm) {
            return false;
        }
        // 提供相对于地形的目标高度
        alt_target_cm = takeoff_alt_cm;
        alt_target_terrain = true;
    } else
#endif
    {
        // 将高度解释为相对于home点的高度
        Location target_loc = copter.current_loc;
        target_loc.set_alt_cm(takeoff_alt_cm, Location::AltFrame::ABOVE_HOME);

        // 提供相对于EKF原点的目标高度
        if (!target_loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, alt_target_cm)) {
            // 这种情况不应该发生，但我们以防万一拒绝该命令
            return false;
        }
    }

    guided_mode = SubMode::TakeOff;

    // 初始化偏航
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

    // 起飞时清除I项
    pos_control->init_z_controller();

    // 初始化WP_NAVALT_MIN的高度并设置完成高度
    auto_takeoff.start(alt_target_cm, alt_target_terrain);

    // 记录起飞未完成
    takeoff_complete = false;

    return true;
}

// 初始化引导模式的航点导航控制器
void ModeGuided::wp_control_start()
{
    // 设置为位置控制模式
    guided_mode = SubMode::WP;

    // 初始化航点和样条控制器
    wp_nav->wp_and_spline_init();

    // 初始化wpnav到停止点
    Vector3f stopping_point;
    wp_nav->get_wp_stopping_point(stopping_point);
    if (!wp_nav->set_wp_destination(stopping_point, false)) {
        // 这种情况不应该发生，因为不使用地形数据
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    // 初始化偏航
    auto_yaw.set_mode_to_default(false);
}

// 运行引导模式的航点导航控制器
void ModeGuided::wp_control_run()
{
    // 如果未解锁则将油门设为零并立即退出
    if (is_disarmed_or_landed()) {
        // 当在地面上且电机联锁启用时，不要降低传统直升机的转速
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // 将电机设置为全范围
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // 运行航点控制器
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // 调用z轴位置控制器(wpnav应该已经更新了它的高度目标)
    pos_control->update_z_controller();

    // 使用自动偏航调用姿态控制器
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

// 初始化位置控制器
void ModeGuided::pva_control_start()
{
    // 初始化水平速度、加速度
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // 初始化垂直速度和加速度
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // 初始化速度控制器
    pos_control->init_z_controller();
    pos_control->init_xy_controller();

    // 初始化偏航
    auto_yaw.set_mode_to_default(false);

    // 初始化地形高度
    guided_pos_terrain_alt = false;
}

// 初始化引导模式的位置控制器
void ModeGuided::pos_control_start()
{
    // 设置为位置控制模式
    guided_mode = SubMode::Pos;

    // 初始化位置控制器
    pva_control_start();
}

// 初始化引导模式的加速度控制器
void ModeGuided::accel_control_start()
{
    // 设置guided_mode为加速度控制器
    guided_mode = SubMode::Accel;

    // 初始化位置控制器
    pva_control_start();
}

// 初始化引导模式的速度和加速度控制器
void ModeGuided::velaccel_control_start()
{
    // 设置guided_mode为速度和加速度控制器
    guided_mode = SubMode::VelAccel;

    // 初始化位置控制器
    pva_control_start();
}

// 初始化引导模式的位置、速度和加速度控制器
void ModeGuided::posvelaccel_control_start()
{
    // 设置guided_mode为位置、速度和加速度控制器
    guided_mode = SubMode::PosVelAccel;

    // 初始化位置控制器
    pva_control_start();
}

// 检查是否正在起飞
bool ModeGuided::is_taking_off() const
{
    return guided_mode == SubMode::TakeOff && !takeoff_complete;
}

// 设置水平速度
bool ModeGuided::set_speed_xy(float speed_xy_cms)
{
    // 初始化水平速度、加速度
    pos_control->set_max_speed_accel_xy(speed_xy_cms, wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(speed_xy_cms, wp_nav->get_wp_acceleration());
    return true;
}

// 设置上升速度
bool ModeGuided::set_speed_up(float speed_up_cms)
{
    // 初始化垂直速度和加速度
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), speed_up_cms, wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), speed_up_cms, wp_nav->get_accel_z());
    return true;
}

// 设置下降速度
bool ModeGuided::set_speed_down(float speed_down_cms)
{
    // 初始化垂直速度和加速度
    pos_control->set_max_speed_accel_z(speed_down_cms, wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(speed_down_cms, wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    return true;
}

// 初始化引导模式的角度控制器
void ModeGuided::angle_control_start()
{
    // 将引导模式设置为角度控制器
    guided_mode = SubMode::Angle;

    // 设置垂直速度和加速度限制
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // 初始化垂直位置控制器
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // 初始化目标值
    guided_angle_state.update_time_ms = millis();  // 更新时间戳
    guided_angle_state.attitude_quat.from_euler(Vector3f(0.0, 0.0, attitude_control->get_att_target_euler_rad().z));  // 从欧拉角设置姿态四元数
    guided_angle_state.ang_vel_body.zero();  // 将角速度设置为零
    guided_angle_state.climb_rate_cms = 0.0f;  // 将爬升速率设置为零
    guided_angle_state.yaw_rate_cds = 0.0f;  // 将偏航速率设置为零
    guided_angle_state.use_yaw_rate = false;  // 不使用偏航速率
}

// set_destination - 设置引导模式的目标目的地
// 如果围栏启用且引导航点在围栏内，则返回true
// 如果航点在围栏外，则返回false
bool ModeGuided::set_destination(const Vector3f& destination, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw, bool terrain_alt)
{
#if AP_FENCE_ENABLED
    // 如果目的地在围栏外，则拒绝
    const Location dest_loc(destination, terrain_alt ? Location::AltFrame::ABOVE_TERRAIN : Location::AltFrame::ABOVE_ORIGIN);
    if (!copter.fence.check_destination_within_fence(dest_loc)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // 失败信息通过NAK传播到GCS
        return false;
    }
#endif

    // 如果配置为使用wpnav进行位置控制
    if (use_wpnav_for_position_control()) {
        // 确保我们处于位置控制模式
        if (guided_mode != SubMode::WP) {
            wp_control_start();
        }

        // 设置偏航状态
        set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

        // 设置航点目的地（不需要检查返回状态，因为不使用地形数据）
        wp_nav->set_wp_destination(destination, terrain_alt);

#if HAL_LOGGING_ENABLED
        // 记录目标
        copter.Log_Write_Guided_Position_Target(guided_mode, destination, terrain_alt, Vector3f(), Vector3f());
#endif
        send_notification = true;
        return true;
    }

    // 如果配置为使用位置控制器进行位置控制
    // 确保我们处于位置控制模式
    if (guided_mode != SubMode::Pos) {
        pos_control_start();
    }

    // 如果需要，初始化地形跟随
    if (terrain_alt) {
        // 获取当前地形高度
        float origin_terr_offset;
        if (!wp_nav->get_terrain_offset(origin_terr_offset)) {
            // 如果我们没有地形高度数据，则停止
            init(true);
            return false;
        }
        // 如果必要，将原点转换为地形高度
        if (!guided_pos_terrain_alt) {
            // 新目的地是相对地形的高度，之前的目的地是相对EKF原点的高度
            pos_control->set_pos_offset_z_cm(origin_terr_offset);
        }
    } else {
        pos_control->set_pos_offset_z_cm(0.0);
    }

    // 设置偏航状态
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // 设置位置目标，并将速度和加速度设为零
    guided_pos_target_cm = destination.topostype();
    guided_pos_terrain_alt = terrain_alt;
    guided_vel_target_cms.zero();
    guided_accel_target_cmss.zero();
    update_time_ms = millis();

#if HAL_LOGGING_ENABLED
    // 记录目标
    copter.Log_Write_Guided_Position_Target(guided_mode, guided_pos_target_cm.tofloat(), guided_pos_terrain_alt, guided_vel_target_cms, guided_accel_target_cmss);
#endif

    send_notification = true;

    return true;
}

// 获取当前航点
bool ModeGuided::get_wp(Location& destination) const
{
    switch (guided_mode) {
    case SubMode::WP:
        return wp_nav->get_oa_wp_destination(destination);
    case SubMode::Pos:
        destination = Location(guided_pos_target_cm.tofloat(), guided_pos_terrain_alt ? Location::AltFrame::ABOVE_TERRAIN : Location::AltFrame::ABOVE_ORIGIN);
        return true;
    case SubMode::Angle:
    case SubMode::TakeOff:
    case SubMode::Accel:
    case SubMode::VelAccel:
    case SubMode::PosVelAccel:
        break;
    }

    return false;
}

// 从Location对象设置引导模式的目标
// 如果无法设置目的地（可能是由于缺少地形数据）或围栏启用且引导航点在围栏外，则返回false
bool ModeGuided::set_destination(const Location& dest_loc, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
#if AP_FENCE_ENABLED
    // 拒绝围栏外的目的地
    // 注意：如果指定为地形高度的目标转换为相对家的高度失败，可能存在不被检查的危险
    if (!copter.fence.check_destination_within_fence(dest_loc)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // 失败信息通过NAK传播到GCS
        return false;
    }
#endif

    // 如果使用wpnav进行位置控制
    if (use_wpnav_for_position_control()) {
        if (guided_mode != SubMode::WP) {
            wp_control_start();
        }

        if (!wp_nav->set_wp_destination_loc(dest_loc)) {
            // 设置目的地失败只可能是因为缺少地形数据
            LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
            // 失败信息通过NAK传播到GCS
            return false;
        }

        // 设置偏航状态
        set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

#if HAL_LOGGING_ENABLED
        // 记录目标
        copter.Log_Write_Guided_Position_Target(guided_mode, Vector3f(dest_loc.lat, dest_loc.lng, dest_loc.alt), (dest_loc.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN), Vector3f(), Vector3f());
#endif

        send_notification = true;
        return true;
    }

    // 设置位置目标，并将速度和加速度设为零
    Vector3f pos_target_f;
    bool terrain_alt;
    if (!wp_nav->get_vector_NEU(dest_loc, pos_target_f, terrain_alt)) {
        return false;
    }

    // 如果配置为使用位置控制器进行位置控制
    // 确保我们处于位置控制模式
    if (guided_mode != SubMode::Pos) {
        pos_control_start();
    }

    // 设置偏航状态
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // 如果需要，初始化地形跟随
    if (terrain_alt) {
        // 获取当前地形高度
        float origin_terr_offset;
        if (!wp_nav->get_terrain_offset(origin_terr_offset)) {
            // 如果我们没有地形高度数据，则停止
            init(true);
            return false;
        }
        // 如果必要，将原点转换为地形高度
        if (!guided_pos_terrain_alt) {
            // 新目的地是相对地形的高度，之前的目的地是相对EKF原点的高度
            pos_control->set_pos_offset_z_cm(origin_terr_offset);
        }
    } else {
        pos_control->set_pos_offset_z_cm(0.0);
    }

    guided_pos_target_cm = pos_target_f.topostype();
    guided_pos_terrain_alt = terrain_alt;
    guided_vel_target_cms.zero();
    guided_accel_target_cmss.zero();
    update_time_ms = millis();

    // 记录目标
#if HAL_LOGGING_ENABLED
    copter.Log_Write_Guided_Position_Target(guided_mode, Vector3f(dest_loc.lat, dest_loc.lng, dest_loc.alt), guided_pos_terrain_alt, guided_vel_target_cms, guided_accel_target_cmss);
#endif

    send_notification = true;

    return true;
}

// set_velaccel - 设置引导模式的目标速度和加速度
void ModeGuided::set_accel(const Vector3f& acceleration, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw, bool log_request)
{
    // 检查我们是否处于加速度控制模式
    if (guided_mode != SubMode::Accel) {
        accel_control_start();
    }

    // 设置偏航状态
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // 设置速度和加速度目标，并将位置设为零
    guided_pos_target_cm.zero();
    guided_pos_terrain_alt = false;
    guided_vel_target_cms.zero();
    guided_accel_target_cmss = acceleration;
    update_time_ms = millis();

#if HAL_LOGGING_ENABLED
    // 记录目标
    if (log_request) {
        copter.Log_Write_Guided_Position_Target(guided_mode, guided_pos_target_cm.tofloat(), guided_pos_terrain_alt, guided_vel_target_cms, guided_accel_target_cmss);
    }
#endif
}

// set_velocity - 设置引导模式的目标速度
void ModeGuided::set_velocity(const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw, bool log_request)
{
    set_velaccel(velocity, Vector3f(), use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw, log_request);
}

// set_velaccel - 设置引导模式的目标速度和加速度
void ModeGuided::set_velaccel(const Vector3f& velocity, const Vector3f& acceleration, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw, bool log_request)
{
    // 检查我们是否处于速度和加速度控制模式
    if (guided_mode != SubMode::VelAccel) {
        velaccel_control_start();
    }

    // 设置偏航状态
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // 设置速度和加速度目标，并将位置设为零
    guided_pos_target_cm.zero();
    guided_pos_terrain_alt = false;
    guided_vel_target_cms = velocity;
    guided_accel_target_cmss = acceleration;
    update_time_ms = millis();

#if HAL_LOGGING_ENABLED
    // 记录目标
    if (log_request) {
        copter.Log_Write_Guided_Position_Target(guided_mode, guided_pos_target_cm.tofloat(), guided_pos_terrain_alt, guided_vel_target_cms, guided_accel_target_cmss);
    }
#endif
}

// set_destination_posvel - 设置引导模式的位置和速度目标
// 参数:
//   destination: 目标位置(Vector3f)
//   velocity: 目标速度(Vector3f)
//   use_yaw: 是否使用偏航角
//   yaw_cd: 目标偏航角(百分之一度)
//   use_yaw_rate: 是否使用偏航角速率
//   yaw_rate_cds: 目标偏航角速率(百分之一度/秒)
//   relative_yaw: 偏航角是否为相对值
bool ModeGuided::set_destination_posvel(const Vector3f& destination, const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
    return set_destination_posvelaccel(destination, velocity, Vector3f(), use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);
}

// set_destination_posvelaccel - 设置引导模式的位置、速度和加速度目标
// 参数:
//   destination: 目标位置(Vector3f)
//   velocity: 目标速度(Vector3f)
//   acceleration: 目标加速度(Vector3f)
//   use_yaw: 是否使用偏航角
//   yaw_cd: 目标偏航角(百分之一度)
//   use_yaw_rate: 是否使用偏航角速率
//   yaw_rate_cds: 目标偏航角速率(百分之一度/秒)
//   relative_yaw: 偏航角是否为相对值
bool ModeGuided::set_destination_posvelaccel(const Vector3f& destination, const Vector3f& velocity, const Vector3f& acceleration, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
#if AP_FENCE_ENABLED
    // 如果目标位置在围栏外，拒绝该目标
    const Location dest_loc(destination, Location::AltFrame::ABOVE_ORIGIN);
    if (!copter.fence.check_destination_within_fence(dest_loc)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // 向地面站发送NAK表示失败
        return false;
    }
#endif

    // 检查我们是否处于位置、速度和加速度控制模式
    if (guided_mode != SubMode::PosVelAccel) {
        posvelaccel_control_start();
    }

    // 设置偏航状态
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    update_time_ms = millis();
    guided_pos_target_cm = destination.topostype();
    guided_pos_terrain_alt = false;
    guided_vel_target_cms = velocity;
    guided_accel_target_cmss = acceleration;

#if HAL_LOGGING_ENABLED
    // 记录目标
    copter.Log_Write_Guided_Position_Target(guided_mode, guided_pos_target_cm.tofloat(), guided_pos_terrain_alt, guided_vel_target_cms, guided_accel_target_cmss);
#endif
    return true;
}

// 如果GUIDED_OPTIONS参数建议SET_ATTITUDE_TARGET的"thrust"字段应该被解释为推力而不是爬升率，则返回true
bool ModeGuided::set_attitude_target_provides_thrust() const
{
    return option_is_enabled(Option::SetAttitudeTarget_ThrustAsThrust);
}

// 如果GUIDED_OPTIONS参数指定应该控制位置(当速度和/或加速度控制处于活动状态时)，则返回true
bool ModeGuided::stabilizing_pos_xy() const
{
    return !option_is_enabled(Option::DoNotStabilizePositionXY);
}

// 如果GUIDED_OPTIONS参数指定应该控制速度(当加速度控制处于活动状态时)，则返回true
bool ModeGuided::stabilizing_vel_xy() const
{
    return !option_is_enabled(Option::DoNotStabilizeVelocityXY);
}

// 如果GUIDED_OPTIONS参数指定应该使用航点导航进行位置控制(允许使用路径规划，但更新必须较慢)，则返回true
bool ModeGuided::use_wpnav_for_position_control() const
{
    return option_is_enabled(Option::WPNavUsedForPosControl);
}

// 设置引导的角度目标子模式：使用旋转四元数、角速度和爬升率或推力(取决于用户选项)
// 参数:
//   attitude_quat: 如果为零：必须提供ang_vel_body(机体坐标系角速度)，即使全为零
//                  如果非零：使用姿态四元数和机体坐标系角速度进行姿态控制
//   ang_vel_body: 机体坐标系角速度(rad/s)
//   climb_rate_cms_or_thrust: 表示爬升率(cm/s)或缩放到[0, 1]的推力，无单位
//   use_thrust: 如果为true：climb_rate_cms_or_thrust表示推力
//               如果为false：climb_rate_cms_or_thrust表示爬升率(cm/s)
void ModeGuided::set_angle(const Quaternion &attitude_quat, const Vector3f &ang_vel_body, float climb_rate_cms_or_thrust, bool use_thrust)
{
    // 检查我们是否处于速度控制模式
    if (guided_mode != SubMode::Angle) {
        angle_control_start();
    } else if (!use_thrust && guided_angle_state.use_thrust) {
        // 已经处于角度控制模式，但从推力切换到爬升率
        pos_control->init_z_controller();
    }

    guided_angle_state.attitude_quat = attitude_quat;
    guided_angle_state.ang_vel_body = ang_vel_body;

    guided_angle_state.use_thrust = use_thrust;
    if (use_thrust) {
        guided_angle_state.thrust = climb_rate_cms_or_thrust;
        guided_angle_state.climb_rate_cms = 0.0f;
    } else {
        guided_angle_state.thrust = 0.0f;
        guided_angle_state.climb_rate_cms = climb_rate_cms_or_thrust;
    }

    guided_angle_state.update_time_ms = millis();

    // 将四元数转换为欧拉角
    float roll_rad, pitch_rad, yaw_rad;
    attitude_quat.to_euler(roll_rad, pitch_rad, yaw_rad);

#if HAL_LOGGING_ENABLED
    // 记录目标
    copter.Log_Write_Guided_Attitude_Target(guided_mode, roll_rad, pitch_rad, yaw_rad, ang_vel_body, guided_angle_state.thrust, guided_angle_state.climb_rate_cms * 0.01);
#endif
}

// takeoff_run - 在引导模式下执行起飞
//      由guided_run以100Hz或更高频率调用
void ModeGuided::takeoff_run()
{
    auto_takeoff.run();
    if (auto_takeoff.complete && !takeoff_complete) {
        takeoff_complete = true;
#if AP_FENCE_ENABLED
        copter.fence.auto_enable_fence_after_takeoff();
#endif
#if AP_LANDINGGEAR_ENABLED
        // 可选地收起起落架
        copter.landinggear.retract_after_takeoff();
#endif
    }
}

// pos_control_run - 运行引导位置控制器
// 由guided_run调用
void ModeGuided::pos_control_run()
{
    // 如果未解锁，将油门设为零并立即退出
    if (is_disarmed_or_landed()) {
        // 当传统直升机在地面上且电机联锁启用时，不要降低电机转速
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // 计算地形调整
    float terr_offset = 0.0f;
    if (guided_pos_terrain_alt && !wp_nav->get_terrain_offset(terr_offset)) {
        // 设置目标失败只可能是因为缺少地形数据
        copter.failsafe_terrain_on_event();
        return;
    }

    // 将电机设置为全范围
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // 向位置控制器发送位置和速度目标
    guided_accel_target_cmss.zero();
    guided_vel_target_cms.zero();

    // 如果在超时时间内没有收到更新，停止旋转
    if (millis() - update_time_ms > get_timeout_ms()) {
        if ((auto_yaw.mode() == AutoYaw::Mode::RATE) || (auto_yaw.mode() == AutoYaw::Mode::ANGLE_RATE)) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);
        }
    }

    float pos_offset_z_buffer = 0.0; // 垂直缓冲区大小(m)
    if (guided_pos_terrain_alt) {
        pos_offset_z_buffer = MIN(copter.wp_nav->get_terrain_margin() * 100.0, 0.5 * fabsF(guided_pos_target_cm.z));
    }
    pos_control->input_pos_xyz(guided_pos_target_cm, terr_offset, pos_offset_z_buffer);

    // 运行位置控制器
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // 调用姿态控制器，使用自动偏航
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

// velaccel_control_run - 运行引导速度控制器
// 由guided_run调用
void ModeGuided::accel_control_run()
{
    // 如果未解锁，将油门设为零并立即退出
    if (is_disarmed_or_landed()) {
        // 当传统直升机在地面上且电机联锁启用时，不要降低电机转速
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // 将电机设置为全范围
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // 如果3秒内没有收到更新，将速度设为零并停止旋转
    uint32_t tnow = millis();
    if (tnow - update_time_ms > get_timeout_ms()) {
        guided_vel_target_cms.zero();
        guided_accel_target_cmss.zero();
        if ((auto_yaw.mode() == AutoYaw::Mode::RATE) || (auto_yaw.mode() == AutoYaw::Mode::ANGLE_RATE)) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);
        }
        pos_control->input_vel_accel_xy(guided_vel_target_cms.xy(), guided_accel_target_cmss.xy(), false);
        pos_control->input_vel_accel_z(guided_vel_target_cms.z, guided_accel_target_cmss.z, false);
    } else {
        // 用新目标更新位置控制器
        pos_control->input_accel_xy(guided_accel_target_cmss);
        if (!stabilizing_vel_xy()) {
            // 将位置和速度误差设为零
            pos_control->stop_vel_xy_stabilisation();
        } else if (!stabilizing_pos_xy()) {
            // 将位置误差设为零
            pos_control->stop_pos_xy_stabilisation();
        }
        pos_control->input_accel_z(guided_accel_target_cmss.z);
    }

    // 调用速度控制器，包括z轴控制器
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // 调用姿态控制器，使用自动偏航
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

// velaccel_control_run - 运行引导速度和加速度控制器
// 由guided_run调用
void ModeGuided::velaccel_control_run()
{
    // 如果未解锁，将油门设为零并立即退出
    if (is_disarmed_or_landed()) {
        // 当传统直升机在地面上且电机联锁启用时，不要降低电机转速
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // 将电机设置为全范围
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // 如果3秒内没有收到更新，将速度设为零并停止旋转
    uint32_t tnow = millis();
    if (tnow - update_time_ms > get_timeout_ms()) {
        guided_vel_target_cms.zero();
        guided_accel_target_cmss.zero();
        if ((auto_yaw.mode() == AutoYaw::Mode::RATE) || (auto_yaw.mode() == AutoYaw::Mode::ANGLE_RATE)) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);
        }
    }

    bool do_avoid = false;
#if AP_AVOIDANCE_ENABLED
    // 为障碍物/围栏避障限制速度
    copter.avoid.adjust_velocity(guided_vel_target_cms, pos_control->get_pos_xy_p().kP(), pos_control->get_max_accel_xy_cmss(), pos_control->get_pos_z_p().kP(), pos_control->get_max_accel_z_cmss(), G_Dt);
    do_avoid = copter.avoid.limits_active();
#endif

    // 用新目标更新位置控制器

    if (!stabilizing_vel_xy() && !do_avoid) {
        // 将当前命令的xy速度设为期望速度
        guided_vel_target_cms.x = pos_control->get_vel_desired_cms().x;
        guided_vel_target_cms.y = pos_control->get_vel_desired_cms().y;
    }
    pos_control->input_vel_accel_xy(guided_vel_target_cms.xy(), guided_accel_target_cmss.xy(), false);
    if (!stabilizing_vel_xy() && !do_avoid) {
        // 将位置和速度误差设为零
        pos_control->stop_vel_xy_stabilisation();
    } else if (!stabilizing_pos_xy() && !do_avoid) {
        // 将位置误差设为零
        pos_control->stop_pos_xy_stabilisation();
    }
    pos_control->input_vel_accel_z(guided_vel_target_cms.z, guided_accel_target_cmss.z, false);

    // 调用速度控制器，包括z轴控制器
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // 调用姿态控制器，使用自动偏航
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

// pause_control_run - 运行引导模式暂停控制器
// 由guided_run调用
void ModeGuided::pause_control_run()
{
    // 如果未解锁，将油门设为零并立即退出
    if (is_disarmed_or_landed()) {
        // 当传统直升机在地面上且电机联锁启用时，不要降低电机转速
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // 将电机设置为全范围
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // 将水平速度和加速度目标设为零
    Vector2f vel_xy, accel_xy;
    pos_control->input_vel_accel_xy(vel_xy, accel_xy, false);

    // 将垂直速度和加速度目标设为零
    float vel_z = 0.0;
    pos_control->input_vel_accel_z(vel_z, 0.0, false);

    // 调用速度控制器，包括z轴控制器
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // 调用姿态控制器
    attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), 0.0);
}

// posvelaccel_control_run - 运行引导位置、速度和加速度控制器
// 由guided_run调用
void ModeGuided::posvelaccel_control_run()
{
    // 如果未解锁，将油门设为零并立即退出
    if (is_disarmed_or_landed()) {
        // 当传统直升机在地面上且电机联锁启用时，不要降低电机转速
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // 将电机设置为全范围
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // 如果3秒内没有收到更新，将速度设为零并停止旋转
    uint32_t tnow = millis();
    if (tnow - update_time_ms > get_timeout_ms()) {
        guided_vel_target_cms.zero();
        guided_accel_target_cmss.zero();
        if ((auto_yaw.mode() == AutoYaw::Mode::RATE) || (auto_yaw.mode() == AutoYaw::Mode::ANGLE_RATE)) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);
        }
    }

    // 向位置控制器发送位置和速度目标
    if (!stabilizing_vel_xy()) {
        // 将当前命令的xy位置设为目标位置，xy速度设为期望速度
        guided_pos_target_cm.x = pos_control->get_pos_target_cm().x;
        guided_pos_target_cm.y = pos_control->get_pos_target_cm().y;
        guided_vel_target_cms.x = pos_control->get_vel_desired_cms().x;
        guided_vel_target_cms.y = pos_control->get_vel_desired_cms().y;
    } else if (!stabilizing_pos_xy()) {
        // 将当前命令的xy位置设为目标位置
        guided_pos_target_cm.x = pos_control->get_pos_target_cm().x;
        guided_pos_target_cm.y = pos_control->get_pos_target_cm().y;
    }
    pos_control->input_pos_vel_accel_xy(guided_pos_target_cm.xy(), guided_vel_target_cms.xy(), guided_accel_target_cmss.xy(), false);
    if (!stabilizing_vel_xy()) {
        // 将位置和速度误差设为零
        pos_control->stop_vel_xy_stabilisation();
    } else if (!stabilizing_pos_xy()) {
        // 将位置误差设为零
        pos_control->stop_pos_xy_stabilisation();
    }

    // guided_pos_target z轴不应该是地形高度
    if (guided_pos_terrain_alt) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    float pz = guided_pos_target_cm.z;
    pos_control->input_pos_vel_accel_z(pz, guided_vel_target_cms.z, guided_accel_target_cmss.z, false);
    guided_pos_target_cm.z = pz;

    // 运行位置控制器
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // 调用姿态控制器，使用自动偏航
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

// angle_control_run - 运行引导角度控制器
// 由guided_run调用
void ModeGuided::angle_control_run()
{
    float climb_rate_cms = 0.0f;
    if (!guided_angle_state.use_thrust) {
        // 限制爬升率
        climb_rate_cms = constrain_float(guided_angle_state.climb_rate_cms, -wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up());

        // 获取避障调整后的爬升率
        climb_rate_cms = get_avoidance_adjusted_climbrate(climb_rate_cms);
    }

    // 检查超时 - 如果3秒内没有收到更新，将倾斜角和爬升率设为零
    uint32_t tnow = millis();
    if (tnow - guided_angle_state.update_time_ms > get_timeout_ms()) {
        guided_angle_state.attitude_quat.from_euler(Vector3f(0.0, 0.0, attitude_control->get_att_target_euler_rad().z));
        guided_angle_state.ang_vel_body.zero();
        climb_rate_cms = 0.0f;
        if (guided_angle_state.use_thrust) {
            // 初始化垂直速度控制器
            pos_control->init_z_controller();
            guided_angle_state.use_thrust = false;
        }
    }

    // 将正的爬升率或推力解释为触发起飞
    const bool positive_thrust_or_climbrate = is_positive(guided_angle_state.use_thrust ? guided_angle_state.thrust : climb_rate_cms);
    if (motors->armed() && positive_thrust_or_climbrate) {
        copter.set_auto_armed(true);
    }

    // 如果未解锁或未自动解锁，或者已着陆且没有正的期望爬升率或推力，将油门设为零并立即退出
    if (!motors->armed() || !copter.ap.auto_armed || (copter.ap.land_complete && !positive_thrust_or_climbrate)) {
        // 当传统直升机在地面上且电机联锁启用时，不要降低电机转速
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // TODO: 使用get_alt_hold_state
    // 已着陆且有正的期望爬升率或推力，起飞
    if (copter.ap.land_complete && positive_thrust_or_climbrate) {
        zero_throttle_and_relax_ac();
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        if (motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
            set_land_complete(false);
            pos_control->init_z_controller();
        }
        return;
    }

    // 将电机设置为全范围
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // 调用姿态控制器
    if (guided_angle_state.attitude_quat.is_零()) {
        // 如果姿态四元数为零，使用角速度输入
        attitude_control->input_rate_bf_roll_pitch_yaw(
            ToDeg(guided_angle_state.ang_vel_body.x) * 100.0f,  // 将roll角速度转换为度/秒并乘以100
            ToDeg(guided_angle_state.ang_vel_body.y) * 100.0f,  // 将pitch角速度转换为度/秒并乘以100
            ToDeg(guided_angle_state.ang_vel_body.z) * 100.0f   // 将yaw角速度转换为度/秒并乘以100
        );
    } else {
        // 否则，使用姿态四元数和角速度输入
        attitude_control->input_quaternion(guided_angle_state.attitude_quat, guided_angle_state.ang_vel_body);
    }

    // 调用位置控制器
    if (guided_angle_state.use_thrust) {
        // 如果使用推力控制，直接设置油门输出
        attitude_control->set_throttle_out(guided_angle_state.thrust, true, copter.g.throttle_filt);
    } else {
        // 否则，使用爬升率控制高度
        pos_control->set_pos_target_z_from_climb_rate_cm(climb_rate_cms);
        pos_control->update_z_controller();
    }
}

// 辅助函数，用于设置偏航状态和目标
void ModeGuided::set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle)
{
    if (use_yaw && relative_angle) {
        // 设置相对固定偏航角
        auto_yaw.set_fixed_yaw(yaw_cd * 0.01f, 0.0f, 0, relative_angle);
    } else if (use_yaw && use_yaw_rate) {
        // 设置偏航角和偏航角速率
        auto_yaw.set_yaw_angle_rate(yaw_cd * 0.01f, yaw_rate_cds * 0.01f);
    } else if (use_yaw && !use_yaw_rate) {
        // 仅设置偏航角
        auto_yaw.set_yaw_angle_rate(yaw_cd * 0.01f, 0.0f);
    } else if (use_yaw_rate) {
        // 仅设置偏航角速率
        auto_yaw.set_rate(yaw_rate_cds);
    } else {
        // 使用默认偏航模式
        auto_yaw.set_mode_to_default(false);
    }
}

// 返回是否应使用飞行员的偏航输入来调整飞行器的航向
bool ModeGuided::use_pilot_yaw(void) const
{
    return !option_is_enabled(Option::IgnorePilotYaw);
}

// Guided限制代码

// limit_clear - 清除/关闭引导限制
void ModeGuided::limit_clear()
{
    guided_limit.timeout_ms = 0;
    guided_limit.alt_min_cm = 0.0f;
    guided_limit.alt_max_cm = 0.0f;
    guided_limit.horiz_max_cm = 0.0f;
}

// limit_set - 设置引导超时和移动限制
void ModeGuided::limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm)
{
    guided_limit.timeout_ms = timeout_ms;
    guided_limit.alt_min_cm = alt_min_cm;
    guided_limit.alt_max_cm = alt_max_cm;
    guided_limit.horiz_max_cm = horiz_max_cm;
}

// limit_init_time_and_pos - 初始化引导开始时间和位置作为限制检查的参考
// 仅由AUTO模式的auto_nav_guided_start函数调用
void ModeGuided::limit_init_time_and_pos()
{
    // 初始化开始时间
    guided_limit.start_time = AP_HAL::millis();

    // 从当前位置初始化开始位置
    guided_limit.start_pos = inertial_nav.get_position_neu_cm();
}

// limit_check - 如果引导模式超出限制则返回true
// 在从NAV_GUIDED_ENABLE任务命令调用引导时使用
bool ModeGuided::limit_check()
{
    // 检查是否超过超时时间
    if ((guided_limit.timeout_ms > 0) && (millis() - guided_limit.start_time >= guided_limit.timeout_ms)) {
        return true;
    }

    // 获取当前位置
    const Vector3f& curr_pos = inertial_nav.get_position_neu_cm();

    // 检查是否低于最小高度
    if (!is_zero(guided_limit.alt_min_cm) && (curr_pos.z < guided_limit.alt_min_cm)) {
        return true;
    }

    // 检查是否高于最大高度
    if (!is_zero(guided_limit.alt_max_cm) && (curr_pos.z > guided_limit.alt_max_cm)) {
        return true;
    }

    // 检查是否超出水平限制
    if (guided_limit.horiz_max_cm > 0.0f) {
        const float horiz_move = get_horizontal_distance_cm(guided_limit.start_pos.xy(), curr_pos.xy());
        if (horiz_move > guided_limit.horiz_max_cm) {
            return true;
        }
    }

    // 如果到达这里，说明在限制范围内
    return false;
}

// 获取目标位置
const Vector3p &ModeGuided::get_target_pos() const
{
    return guided_pos_target_cm;
}

// 获取目标速度
const Vector3f& ModeGuided::get_target_vel() const
{
    return guided_vel_target_cms;
}

// 获取目标加速度
const Vector3f& ModeGuided::get_target_accel() const
{
    return guided_accel_target_cmss;
}

// 获取到航点的距离（厘米）
uint32_t ModeGuided::wp_distance() const
{
    switch(guided_mode) {
    case SubMode::WP:
        return wp_nav->get_wp_distance_to_destination();
    case SubMode::Pos:
        return get_horizontal_distance_cm(inertial_nav.get_position_xy_cm(), guided_pos_target_cm.tofloat().xy());
    case SubMode::PosVelAccel:
        return pos_control->get_pos_error_xy_cm();
    default:
        return 0;
    }
}

// 获取到航点的方位角（厘度）
int32_t ModeGuided::wp_bearing() const
{
    switch(guided_mode) {
    case SubMode::WP:
        return wp_nav->get_wp_bearing_to_destination();
    case SubMode::Pos:
        return get_bearing_cd(inertial_nav.get_position_xy_cm(), guided_pos_target_cm.tofloat().xy());
    case SubMode::PosVelAccel:
        return pos_control->get_bearing_to_target_cd();
    case SubMode::TakeOff:
    case SubMode::Accel:
    case SubMode::VelAccel:
    case SubMode::Angle:
        // 这些模式没有方位角
        return 0;
    }
    // 编译器保证我们不会到达这里
    return 0.0;
}

// 获取横向跟踪误差
float ModeGuided::crosstrack_error() const
{
    switch (guided_mode) {
    case SubMode::WP:
        return wp_nav->crosstrack_error();
    case SubMode::Pos:
    case SubMode::TakeOff:
    case SubMode::Accel:
    case SubMode::VelAccel:
    case SubMode::PosVelAccel:
        return pos_control->crosstrack_error();
    case SubMode::Angle:
        // 角度模式没有轨迹，因此没有横向跟踪误差
        return 0;
    }
    // 编译器保证我们不会到达这里
    return 0;
}

// 返回引导模式超时时间（毫秒）。仅用于速度、加速度、角度控制和角速率控制
uint32_t ModeGuided::get_timeout_ms() const
{
    return MAX(copter.g2.guided_timeout, 0.1) * 1000;
}

// 暂停引导模式
bool ModeGuided::pause()
{
    _paused = true;
    return true;
}

// 恢复引导模式
bool ModeGuided::resume()
{
    _paused = false;
    return true;
}

#endif
