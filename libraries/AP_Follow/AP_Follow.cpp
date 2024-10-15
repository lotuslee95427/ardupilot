/*
   这个程序是自由软件:你可以在遵守GNU通用公共许可证的前提下对其进行再分发和/或修改,
   许可证的版本应为第三版或(你可以选择)任何更新的版本。

   发布这个程序是希望它能有用,但并不提供任何保证;甚至不保证它的可销售性或对某一特定用途的适用性。
   更多细节请参见GNU通用公共许可证。

   你应该已经收到了一份GNU通用公共许可证的副本。如果没有,请查看<http://www.gnu.org/licenses/>。
 */

#include "AP_Follow_config.h"

#if AP_FOLLOW_ENABLED // 如果启用了AP_FOLLOW功能

#include <AP_HAL/AP_HAL.h>
#include "AP_Follow.h"
#include <ctype.h>
#include <stdio.h>

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL& hal;

#define AP_FOLLOW_TIMEOUT_MS    3000    // 位置估计超时时间为3秒
#define AP_FOLLOW_SYSID_TIMEOUT_MS 10000 // 如果10秒内没有收到被跟随目标的消息,则忘记该系统ID

#define AP_FOLLOW_OFFSET_TYPE_NED       0   // 偏移量以北东下(NED)坐标系表示
#define AP_FOLLOW_OFFSET_TYPE_RELATIVE  1   // 偏移量相对于被跟随车辆的航向

#define AP_FOLLOW_ALTITUDE_TYPE_RELATIVE  1 // 默认使用相对高度   

#define AP_FOLLOW_POS_P_DEFAULT 0.1f    // 位置误差增益默认值

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#define AP_FOLLOW_ALT_TYPE_DEFAULT 0
#else
#define AP_FOLLOW_ALT_TYPE_DEFAULT AP_FOLLOW_ALTITUDE_TYPE_RELATIVE
#endif

AP_Follow *AP_Follow::_singleton;

// 用户可设置参数表
const AP_Param::GroupInfo AP_Follow::var_info[] = {

    // @Param: _ENABLE
    // @DisplayName: 启用/禁用跟随功能
    // @Description: 启用/禁用跟随目标
    // @Values: 0:禁用,1:启用
    // @User: Standard
    AP_GROUPINFO_FLAGS("_ENABLE", 1, AP_Follow, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // 2 保留给TYPE参数

    // @Param: _SYSID
    // @DisplayName: 被跟随目标的mavlink系统ID
    // @Description: 被跟随目标的mavlink系统ID
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("_SYSID", 3, AP_Follow, _sysid, 0),

    // 4 保留给MARGIN参数

    // @Param: _DIST_MAX
    // @DisplayName: 最大跟随距离
    // @Description: 最大跟随距离。超过此距离的目标将被忽略
    // @Units: m
    // @Range: 1 1000
    // @User: Standard
    AP_GROUPINFO("_DIST_MAX", 5, AP_Follow, _dist_max, 100),

    // @Param: _OFS_TYPE
    // @DisplayName: 跟随偏移类型
    // @Description: 跟随偏移类型
    // @Values: 0:北东下, 1:相对于被跟随车辆航向
    // @User: Standard
    AP_GROUPINFO("_OFS_TYPE", 6, AP_Follow, _offset_type, AP_FOLLOW_OFFSET_TYPE_NED),

    // @Param: _OFS_X
    // @DisplayName: 北向/前向偏移量(米)
    // @Description: 北向/前向偏移量(米)。如果为正,本机将飞行在被跟随车辆的前方或北方。取决于FOLL_OFS_TYPE
    // @Range: -100 100
    // @Units: m
    // @Increment: 1
    // @User: Standard

    // @Param: _OFS_Y
    // @DisplayName: 东向/右向偏移量(米)
    // @Description: 东向/右向偏移量(米)。如果为正,本机将飞行在被跟随车辆的右侧或东侧。取决于FOLL_OFS_TYPE
    // @Range: -100 100
    // @Units: m
    // @Increment: 1
    // @User: Standard

    // @Param: _OFS_Z
    // @DisplayName: 下向偏移量(米)
    // @Description: 下向偏移量(米)。如果为正,本机将飞行在被跟随车辆的下方
    // @Range: -100 100
    // @Units: m
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_OFS", 7, AP_Follow, _offset, 0),

#if !(APM_BUILD_TYPE(APM_BUILD_Rover))
    // @Param: _YAW_BEHAVE
    // @DisplayName: 跟随偏航行为
    // @Description: 跟随偏航行为
    // @Values: 0:无,1:面向被跟随车辆,2:与被跟随车辆相同,3:飞行方向
    // @User: Standard
    AP_GROUPINFO("_YAW_BEHAVE", 8, AP_Follow, _yaw_behave, 1),
#endif

    // @Param: _POS_P
    // @DisplayName: 跟随位置误差P增益
    // @Description: 跟随位置误差P增益。将期望垂直速度与实际速度之间的差异转换为传递给油门加速度控制器的期望加速度
    // @Range: 0.01 1.00
    // @Increment: 0.01
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos, "_POS_", 9, AP_Follow, AC_P),

#if !(APM_BUILD_TYPE(APM_BUILD_Rover)) 
    // @Param: _ALT_TYPE
    // @DisplayName: 跟随高度类型
    // @Description: 跟随高度类型
    // @Values: 0:绝对高度, 1:相对高度
    // @User: Standard
    AP_GROUPINFO("_ALT_TYPE", 10, AP_Follow, _alt_type, AP_FOLLOW_ALT_TYPE_DEFAULT),
#endif

    // @Param: _OPTIONS
    // @DisplayName: 跟随选项
    // @Description: 跟随选项位掩码
    // @Values: 0:无,1: 进入模式时云台跟随被跟随车辆
    // @User: Standard
    AP_GROUPINFO("_OPTIONS", 11, AP_Follow, _options, 0),

    AP_GROUPEND
};

/* 
   构造函数同时初始化接近传感器。注意这个构造函数直到detect()返回true才会被调用,
   所以我们已经知道应该设置接近传感器
*/
AP_Follow::AP_Follow() :
        _p_pos(AP_FOLLOW_POS_P_DEFAULT)
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// 如有必要,将偏移量恢复为零,应在车辆退出跟随模式时调用
void AP_Follow::clear_offsets_if_required()
{
    if (_offsets_were_zero) {
        _offset.set(Vector3f());
    }
    _offsets_were_zero = false;
}

// 获取目标的估计位置
bool AP_Follow::get_target_location_and_velocity(Location &loc, Vector3f &vel_ned) const
{
    // 如果未启用,立即退出
    if (!_enabled) {
        return false;
    }

    // 检查超时
    if ((_last_location_update_ms == 0) || (AP_HAL::millis() - _last_location_update_ms > AP_FOLLOW_TIMEOUT_MS)) {
        return false;
    }

    // 计算自上次实际位置更新以来的时间
    const float dt = (AP_HAL::millis() - _last_location_update_ms) * 0.001f;

    // 获取速度估计
    if (!get_velocity_ned(vel_ned, dt)) {
        return false;
    }

    // 推算车辆位置
    Location last_loc = _target_location;
    last_loc.offset(vel_ned.x * dt, vel_ned.y * dt);
    last_loc.alt -= vel_ned.z * 100.0f * dt; // 将m/s转换为cm/s,乘以dt。减法是因为NED坐标系

    // 返回最新的位置估计
    loc = last_loc;
    return true;
}

// 获取到目标的距离向量(以米为单位)和目标的速度,均在NED坐标系中
bool AP_Follow::get_target_dist_and_vel_ned(Vector3f &dist_ned, Vector3f &dist_with_offs, Vector3f &vel_ned)
{
    // 获取我们的位置
    Location current_loc;
    if (!AP::ahrs().get_location(current_loc)) {
        clear_dist_and_bearing_to_target();
         return false;
    }

    // 获取目标位置和速度
    Location target_loc;
    Vector3f veh_vel;
    if (!get_target_location_and_velocity(target_loc, veh_vel)) {
        clear_dist_and_bearing_to_target();
        return false;
    }

    // 改变为相对于home点的高度
    if (target_loc.relative_alt == 1) {
        current_loc.change_alt_frame(Location::AltFrame::ABOVE_HOME);
    }

    // 计算差异
    const Vector3f dist_vec = current_loc.get_distance_NED(target_loc);

    // 如果太远则失败
    if (is_positive(_dist_max.get()) && (dist_vec.length() > _dist_max)) {
        clear_dist_and_bearing_to_target();
        return false;
    }

    // 如果需要,从距离向量初始化偏移量
    init_offsets_if_required(dist_vec);

    // 获取偏移量
    Vector3f offsets;
    if (!get_offsets_ned(offsets)) {
        clear_dist_and_bearing_to_target();
        return false;
    }

    // 计算结果
    dist_ned = dist_vec;
    dist_with_offs = dist_vec + offsets;
    vel_ned = veh_vel;

    // 记录距离和方位以用于报告
    if (is_zero(dist_with_offs.x) && is_zero(dist_with_offs.y)) {
        clear_dist_and_bearing_to_target();
    } else {
        _dist_to_target = safe_sqrt(sq(dist_with_offs.x) + sq(dist_with_offs.y));
        _bearing_to_target = degrees(atan2f(dist_with_offs.y, dist_with_offs.x));
    }

    return true;
}

// 获取目标的航向(度)(0 = 北, 90 = 东)
bool AP_Follow::get_target_heading_deg(float &heading) const
{
    // 如果未启用,立即退出
    if (!_enabled) {
        return false;
    }

    // 检查超时
    if ((_last_heading_update_ms == 0) || (AP_HAL::millis() - _last_heading_update_ms > AP_FOLLOW_TIMEOUT_MS)) {
        return false;
    }

    // 返回最新的航向估计
    heading = _target_heading;
    return true;
}

// 返回是否应从消息中提取信息
bool AP_Follow::should_handle_message(const mavlink_message_t &msg) const
{
    // 如果未启用,立即退出
    if (!_enabled) {
        return false;
    }

    // 跳过我们自己的消息
    if (msg.sysid == mavlink_system.sysid) {
        return false;
    }

    // 如果不是来自我们的目标,跳过消息
    if (_sysid != 0 && msg.sysid != _sysid) {
        return false;
    }

    return true;
}

// 处理 MAVLINK DISTANCE_SENSOR 消息
void AP_Follow::handle_msg(const mavlink_message_t &msg)
{
    // 此方法应该从"update()"方法中调用:
    if (_automatic_sysid) {
        // 可能需要超时我们正在跟随的对象...
        if ((_last_location_update_ms == 0) ||
            (AP_HAL::millis() - _last_location_update_ms > AP_FOLLOW_SYSID_TIMEOUT_MS)) {
            _sysid.set(0);
        }
    }

    // 检查是否应处理此消息
    if (!should_handle_message(msg)) {
        return;
    }

    // 解码全球位置信息消息
    bool updated = false;

    // 根据消息ID处理不同类型的消息
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        // 处理全球位置信息消息
        updated = handle_global_position_int_message(msg);
        break;
    }
    case MAVLINK_MSG_ID_FOLLOW_TARGET: {
        // 处理跟随目标消息
        updated = handle_follow_target_message(msg);
        break;
    }
    }

    // 如果信息已更新,记录日志
    if (updated) {
#if HAL_LOGGING_ENABLED
        Log_Write_FOLL();
#endif
    }
}

// 处理全球位置信息消息
bool AP_Follow::handle_global_position_int_message(const mavlink_message_t &msg)
{
        // 解码消息
        mavlink_global_position_int_t packet;
        mavlink_msg_global_position_int_decode(&msg, &packet);

        // 如果经纬度都为(精确的)零,则忽略此消息
        if ((packet.lat == 0 && packet.lon == 0)) {
            return false;
        }

        // 更新目标位置
        _target_location.lat = packet.lat;
        _target_location.lng = packet.lon;

        // 根据 FOLL_ALT_TYPE 参数选择高度源 
        if (_alt_type == AP_FOLLOW_ALTITUDE_TYPE_RELATIVE) {
            // 相对于家的高度
            _target_location.set_alt_cm(packet.relative_alt / 10, Location::AltFrame::ABOVE_HOME);
        } else {
            // 绝对高度
            _target_location.set_alt_cm(packet.alt / 10, Location::AltFrame::ABSOLUTE);
        }

        // 更新目标速度
        _target_velocity_ned.x = packet.vx * 0.01f; // 北向速度
        _target_velocity_ned.y = packet.vy * 0.01f; // 东向速度
        _target_velocity_ned.z = packet.vz * 0.01f; // 向下速度

        // 获取经过传输抖动校正的本地时间戳
        _last_location_update_ms = _jitter.correct_offboard_timestamp_msec(packet.time_boot_ms, AP_HAL::millis());
        
        // 更新目标航向(如果可用)
        if (packet.hdg <= 36000) {                  // 航向 (如果未知则为 UINT16_MAX)
            _target_heading = packet.hdg * 0.01f;   // 将百分之一度转换为度
            _last_heading_update_ms = _last_location_update_ms;
        }
        
        // 如果 _sysid 为零,则将其初始化为发送者的 ID
        if (_sysid == 0) {
            _sysid.set(msg.sysid);
            _automatic_sysid = true;
        }
        return true;
}

// 处理跟随目标消息
bool AP_Follow::handle_follow_target_message(const mavlink_message_t &msg)
{
        // 解码消息
        mavlink_follow_target_t packet;
        mavlink_msg_follow_target_decode(&msg, &packet);

        // 如果经纬度都为(精确的)零,则忽略此消息
        if ((packet.lat == 0 && packet.lon == 0)) {
            return false;
        }
        // 至少需要位置信息
        if ((packet.est_capabilities & (1<<0)) == 0) {
            return false;
        }

        // 更新目标位置
        Location new_loc = _target_location;
        new_loc.lat = packet.lat;
        new_loc.lng = packet.lon;
        new_loc.set_alt_cm(packet.alt*100, Location::AltFrame::ABSOLUTE);

        // FOLLOW_TARGET 总是使用 AMSL,如果我们配置为相对高度,则需要将提供的高度更改为相对于家的高度
        if (_alt_type == AP_FOLLOW_ALTITUDE_TYPE_RELATIVE &&
            !new_loc.change_alt_frame(Location::AltFrame::ABOVE_HOME)) {
            return false;
        }
        _target_location = new_loc;

        // 更新目标速度(如果可用)
        if (packet.est_capabilities & (1<<1)) {
            _target_velocity_ned.x = packet.vel[0]; // 北向速度
            _target_velocity_ned.y = packet.vel[1]; // 东向速度
            _target_velocity_ned.z = packet.vel[2]; // 向下速度
        } else {
            _target_velocity_ned.zero();
        }

        // 获取经过传输抖动校正的本地时间戳
        _last_location_update_ms = _jitter.correct_offboard_timestamp_msec(packet.timestamp, AP_HAL::millis());

        // 更新目标航向(如果可用)
        if (packet.est_capabilities & (1<<3)) {
            Quaternion q{packet.attitude_q[0], packet.attitude_q[1], packet.attitude_q[2], packet.attitude_q[3]};
            float r, p, y;
            q.to_euler(r,p,y);
            _target_heading = degrees(y);
            _last_heading_update_ms = _last_location_update_ms;
        }

        // 如果 _sysid 为零,则将其初始化为发送者的 ID
        if (_sysid == 0) {
            _sysid.set(msg.sysid);
            _automatic_sysid = true;
        }

        return true;
}

// 写入板载日志消息以帮助诊断跟随问题:
#if HAL_LOGGING_ENABLED
void AP_Follow::Log_Write_FOLL()
{
        // 获取估计的位置和速度
        Location loc_estimate{};
        Vector3f vel_estimate;
        UNUSED_RESULT(get_target_location_and_velocity(loc_estimate, vel_estimate));

        // 记录目标的估计位置与报告位置
// @LoggerMessage: FOLL
// @Description: 跟随库诊断数据
// @Field: TimeUS: 系统启动以来的时间
// @Field: Lat: 目标纬度
// @Field: Lon: 目标经度
// @Field: Alt: 目标绝对高度
// @Field: VelN: 目标地球坐标系北向速度
// @Field: VelE: 目标地球坐标系东向速度
// @Field: VelD: 目标地球坐标系向下速度
// @Field: LatE: 飞行器纬度
// @Field: LonE: 飞行器经度
// @Field: AltE: 飞行器绝对高度
        AP::logger().WriteStreaming("FOLL",
                                               "TimeUS,Lat,Lon,Alt,VelN,VelE,VelD,LatE,LonE,AltE",  // 标签
                                               "sDUmnnnDUm",    // 单位
                                               "F--B000--B",    // 乘数
                                               "QLLifffLLi",    // 格式
                                               AP_HAL::micros64(),
                                               _target_location.lat,
                                               _target_location.lng,
                                               _target_location.alt,
                                               (double)_target_velocity_ned.x,
                                               (double)_target_velocity_ned.y,
                                               (double)_target_velocity_ned.z,
                                               loc_estimate.lat,
                                               loc_estimate.lng,
                                               loc_estimate.alt
                                               );
}
#endif  // HAL_LOGGING_ENABLED

// 使用自上次更新以来的 dt 获取 NED 坐标系中的速度估计(单位:m/s)
bool AP_Follow::get_velocity_ned(Vector3f &vel_ned, float dt) const
{
    vel_ned = _target_velocity_ned + (_target_accel_ned * dt);
    return true;
}

// 如果需要,使用提供的到其他车辆的距离向量(NED 坐标系中的米)初始化偏移量
void AP_Follow::init_offsets_if_required(const Vector3f &dist_vec_ned)
{
    // 如果偏移量已经设置,立即返回
    if (!_offset.get().is_zero()) {
        return;
    }
    _offsets_were_zero = true;

    float target_heading_deg;
    if ((_offset_type == AP_FOLLOW_OFFSET_TYPE_RELATIVE) && get_target_heading_deg(target_heading_deg)) {
        // 将偏移量从北向旋转到车辆的视角
        _offset.set(rotate_vector(-dist_vec_ned, -target_heading_deg));
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "已加载相对跟随偏移量");
    } else {
        // 在 NED 坐标系中初始化偏移量
        _offset.set(-dist_vec_ned);
        // 确保使用的 offset_type 与保存的偏移量帧匹配
        _offset_type.set(AP_FOLLOW_OFFSET_TYPE_NED);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "已加载 N-E-D 跟随偏移量");
    }
}

// 获取 NED 坐标系中的偏移量(单位:米)
bool AP_Follow::get_offsets_ned(Vector3f &offset) const
{
    const Vector3f &off = _offset.get();

    // 如果偏移量为零或类型为 NED,直接返回偏移向量
    if (off.is_zero() || (_offset_type == AP_FOLLOW_OFFSET_TYPE_NED)) {
        offset = off;
        return true;
    }

    // 偏移类型为相对,如果无法获取车辆航向则退出
    float target_heading_deg;
    if (!get_target_heading_deg(target_heading_deg)) {
        return false;
    }

    // 将偏移量从车辆视角旋转到 NED 坐标系
    offset = rotate_vector(off, target_heading_deg);
    return true;
}

// 将 3D 向量顺时针旋转指定角度(单位:度)
Vector3f AP_Follow::rotate_vector(const Vector3f &vec, float angle_deg) const
{
    // 将北向的滚转、俯仰输入旋转到车辆的视角
    Vector3f ret = vec;
    ret.xy().rotate(radians(angle_deg));

    return ret;
}

// 将记录的到目标的距离和方位设置为零
void AP_Follow::clear_dist_and_bearing_to_target()
{
    _dist_to_target = 0.0f;
    _bearing_to_target = 0.0f;
}

// 获取目标的估计位置和速度(NED 坐标系),并添加偏移量
bool AP_Follow::get_target_location_and_velocity_ofs(Location &loc, Vector3f &vel_ned) const
{
    Vector3f ofs;
    if (!get_offsets_ned(ofs) ||
        !get_target_location_and_velocity(loc, vel_ned)) {
        return false;
    }
    // 应用偏移量
    loc.offset(ofs.x, ofs.y);
    loc.alt -= ofs.z*100;
    return true;
}

// 如果我们有目标,则返回 true
bool AP_Follow::have_target(void) const
{
    if (!_enabled) {
        return false;
    }

    // 检查超时
    if ((_last_location_update_ms == 0) || (AP_HAL::millis() - _last_location_update_ms > AP_FOLLOW_TIMEOUT_MS)) {
        return false;
    }
    return true;
}

namespace AP {

AP_Follow &follow()
{
    return *AP_Follow::get_singleton();
}

}

#endif  // AP_FOLLOW_ENABLED
