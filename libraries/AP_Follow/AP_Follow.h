/*
   本程序是自由软件：您可以根据自由软件基金会发布的GNU通用公共许可证的条款重新分发和/或修改它，
   可以选择使用许可证的第3版，或（由您选择）任何更高版本。

   发布此程序是希望它能有用，但不提供任何保证；甚至不提供针对特定用途的适销性或适用性的暗示保证。
   有关更多详细信息，请参阅GNU通用公共许可证。

   您应该已经收到了GNU通用公共许可证的副本。如果没有，请参阅<http://www.gnu.org/licenses/>。
 */
#pragma once
#include "AP_Follow_config.h"

#if AP_FOLLOW_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AC_PID/AC_P.h>
#include <AP_RTC/JitterCorrection.h>

class AP_Follow
{

public:

    // FOLLOW_OPTIONS参数的枚举
    enum class Option {
        MOUNT_FOLLOW_ON_ENTER = 1  // 进入跟随模式时启用云台跟随
    };

    // YAW_BEHAVE参数的枚举
    enum YawBehave {
        YAW_BEHAVE_NONE = 0,                 // 不控制偏航
        YAW_BEHAVE_FACE_LEAD_VEHICLE = 1,    // 面向领航车辆
        YAW_BEHAVE_SAME_AS_LEAD_VEHICLE = 2, // 与领航车辆保持相同偏航
        YAW_BEHAVE_DIR_OF_FLIGHT = 3         // 朝向飞行方向
    };

    // 构造函数
    AP_Follow();

    // 启用单例模式
    static AP_Follow *get_singleton(void) {
        return _singleton;
    }

    // 返回库是否启用
    bool enabled() const { return _enabled; }

    // 设置要跟随的目标
    void set_target_sysid(uint8_t sysid) { _sysid.set(sysid); }

    // 如有必要，将偏移量恢复为零，应在车辆退出跟随模式时调用
    void clear_offsets_if_required();

    //
    // 位置跟踪相关方法
    //

    // 如果我们有有效的目标位置估计，则返回true
    bool have_target() const;

    // 获取目标的估计位置和速度（以NED坐标系表示）
    bool get_target_location_and_velocity(Location &loc, Vector3f &vel_ned) const;

    // 获取目标的估计位置和速度（以NED坐标系表示），并添加偏移量
    bool get_target_location_and_velocity_ofs(Location &loc, Vector3f &vel_ned) const;
    
    // 获取到目标的距离向量（以米为单位），目标加偏移量，以及目标的速度，全部以NED坐标系表示
    bool get_target_dist_and_vel_ned(Vector3f &dist_ned, Vector3f &dist_with_ofs, Vector3f &vel_ned);

    // 获取目标系统ID
    uint8_t get_target_sysid() const { return _sysid.get(); }

    // 获取位置控制器。此控制器在本库中未使用，但在此处保存很方便
    const AC_P& get_pos_p() const { return _p_pos; }

    //
    // 偏航/航向相关方法
    //

    // 获取用户定义的偏航行为
    YawBehave get_yaw_behave() const { return (YawBehave)_yaw_behave.get(); }

    // 获取目标的航向（以度为单位，0 = 北，90 = 东）
    bool get_target_heading_deg(float &heading) const;

    // 解析可能包含目标位置、速度和姿态的mavlink消息
    void handle_msg(const mavlink_message_t &msg);

    //
    // GCS报告功能
    //

    // 获取到目标的水平距离（包括偏移量），以米为单位（用于报告目的）
    float get_distance_to_target() const { return _dist_to_target; }

    // 获取到目标的方位角（包括偏移量），以度为单位（用于报告目的）
    float get_bearing_to_target() const { return _bearing_to_target; }

    // 获取最后位置更新的系统时间
    uint32_t get_last_update_ms() const { return _last_location_update_ms; }

    // 如果启用了跟随选项，则返回true
    bool option_is_enabled(Option option) const { return (_options.get() & (uint16_t)option) != 0; }

    // 参数列表
    static const struct AP_Param::GroupInfo var_info[];

private:
    static AP_Follow *_singleton;

    // 如果我们应该从消息中提取信息，则返回true
    bool should_handle_message(const mavlink_message_t &msg) const;

    // 使用自上次更新以来的dt获取以m/s为单位的NED坐标系速度估计
    bool get_velocity_ned(Vector3f &vel_ned, float dt) const;

    // 如果需要，将偏移量初始化为提供的到其他车辆的距离向量（以NED坐标系中的米为单位）
    void init_offsets_if_required(const Vector3f &dist_vec_ned);

    // 获取NED坐标系中的偏移量（以米为单位）
    bool get_offsets_ned(Vector3f &offsets) const;

    // 将3D向量按指定角度（以度为单位）顺时针旋转
    Vector3f rotate_vector(const Vector3f &vec, float angle_deg) const;

    // 将记录的到目标的距离和方位设置为零
    void clear_dist_and_bearing_to_target();

    // 处理提供位置的各种mavlink消息：
    bool handle_global_position_int_message(const mavlink_message_t &msg);
    bool handle_follow_target_message(const mavlink_message_t &msg);

    // 写入机载日志消息以帮助诊断跟随问题：
    void Log_Write_FOLL();

    // 参数
    AP_Int8     _enabled;           // 如果此子系统启用则为1
    AP_Int16    _sysid;             // 目标的mavlink系统ID（0表示使用看到的第一个系统ID）
    AP_Float    _dist_max;          // 到目标的最大距离。超过此距离的目标将被忽略
    AP_Int8     _offset_type;       // 偏移框架类型（0:北东下，1:相对于领航车辆航向）
    AP_Vector3f _offset;            // 相对于领航车辆的偏移量（以米为单位）
    AP_Int8     _yaw_behave;        // 跟随车辆的偏航/航向行为（参见YAW_BEHAVE枚举）
    AP_Int8     _alt_type;          // 跟随模式的高度来源
    AC_P        _p_pos;             // 位置误差P控制器
    AP_Int16    _options;           // 跟随模式的云台行为选项

    // 局部变量
    uint32_t _last_location_update_ms;  // 最后位置更新的系统时间
    Location _target_location;      // 目标的最后已知位置
    Vector3f _target_velocity_ned;  // 目标在NED坐标系中的最后已知速度（m/s）
    Vector3f _target_accel_ned;     // 目标在NED坐标系中的最后已知加速度（m/s/s）
    uint32_t _last_heading_update_ms;   // 最后航向更新的系统时间
    float _target_heading;          // 航向（以度为单位）
    bool _automatic_sysid;          // 我们是否自动锁定到一个系统ID？
    float _dist_to_target;          // 到目标的最新距离（以米为单位，用于报告目的）
    float _bearing_to_target;       // 到目标的最新方位（以度为单位，用于报告目的）
    bool _offsets_were_zero;        // 如果偏移量最初为零，然后初始化为与领航车辆的偏移量，则为true

    // 设置抖动校正，最大传输延迟为3秒
    JitterCorrection _jitter{3000};
};

namespace AP {
    AP_Follow &follow();
};

#endif
