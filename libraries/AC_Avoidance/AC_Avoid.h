#pragma once

#include "AC_Avoidance_config.h"

#if AP_AVOIDANCE_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h> // 用于sqrt控制器的姿态控制器库

#define AC_AVOID_ACCEL_CMSS_MAX         100.0f  // 用于避免撞击围栏的最大加速度/减速度(厘米/秒/秒)

// 启用围栏类型的位掩码
#define AC_AVOID_DISABLED               0       // 避障禁用
#define AC_AVOID_STOP_AT_FENCE          1       // 在围栏处停止
#define AC_AVOID_USE_PROXIMITY_SENSOR   2       // 基于近距离传感器输出停止
#define AC_AVOID_STOP_AT_BEACON_FENCE   4       // 基于信标周边停止
#define AC_AVOID_DEFAULT                (AC_AVOID_STOP_AT_FENCE | AC_AVOID_USE_PROXIMITY_SENSOR)

// 非GPS避障的定义
#define AC_AVOID_NONGPS_DIST_MAX_DEFAULT    5.0f    // 超过5米的物体将被忽略(DIST_MAX参数的默认值)
#define AC_AVOID_ANGLE_MAX_PERCENT          0.75f   // 物体避障最大倾斜角度占总车辆最大倾斜角度的百分比(以0~1范围表示)

#define AC_AVOID_ACTIVE_LIMIT_TIMEOUT_MS    500     // 如果在最后x毫秒内发生限制,则限制处于活动状态
#define AC_AVOID_ACCEL_TIMEOUT_MS           200     // 如果避障在此毫秒数后仍处于活动状态,用于计算加速度的存储速度将被重置

/*
 * 此类防止车辆离开多边形围栏或撞击基于近距离的障碍物
 * 此外,如果超出与障碍物的边距,车辆可能会后退
 */
class AC_Avoid {
public:
    AC_Avoid();

    /* 不允许复制 */
    CLASS_NO_COPY(AC_Avoid);

    // 获取单例实例
    static AC_Avoid *get_singleton() {
        return _singleton;
    }

    // 如果启用了任何避障功能,则返回true
    bool enabled() const { return _enabled != AC_AVOID_DISABLED; }

    // 调整期望速度,使车辆能够在围栏/物体前停止
    // kP, accel_cmss用于水平轴
    // kP_z, accel_cmss_z用于垂直轴
    void adjust_velocity(Vector3f &desired_vel_cms, bool &backing_up, float kP, float accel_cmss, float kP_z, float accel_cmss_z, float dt);
    void adjust_velocity(Vector3f &desired_vel_cms, float kP, float accel_cmss, float kP_z, float accel_cmss_z, float dt) {
        bool backing_up = false;
        adjust_velocity(desired_vel_cms, backing_up, kP, accel_cmss, kP_z, accel_cmss_z, dt);
    }

    // 此方法限制速度并计算来自各种支持的围栏的后退速度
    // 还使用adjust_velocity_z方法限制垂直速度
    void adjust_velocity_fence(float kP, float accel_cmss, Vector3f &desired_vel_cms, Vector3f &backup_vel, float kP_z, float accel_cmss_z, float dt);

    // 调整期望的水平速度,使车辆在围栏或物体前停止
    // accel(最大加速度/减速度)单位为米/秒/秒
    // heading单位为弧度
    // speed单位为米/秒
    // kP应为零以获得线性响应,非零以获得非线性响应
    // dt是自上次调用以来的时间(秒)
    void adjust_speed(float kP, float accel, float heading, float &speed, float dt);

    // 调整垂直爬升率,使车辆不会突破垂直围栏
    void adjust_velocity_z(float kP, float accel_cmss, float& climb_rate_cms, float& backup_speed, float dt);
    void adjust_velocity_z(float kP, float accel_cmss, float& climb_rate_cms, float dt);

    // 调整横滚-俯仰以推动车辆远离物体
    // 横滚和俯仰值以厘度为单位
    // angle_max是用户定义的车辆最大倾斜角度(以厘度为单位)
    void adjust_roll_pitch(float &roll, float &pitch, float angle_max);

    // 启用/禁用基于近距离的避障
    void proximity_avoidance_enable(bool on_off) { _proximity_enabled = on_off; }
    bool proximity_avoidance_enabled() const { return (_proximity_enabled && (_enabled & AC_AVOID_USE_PROXIMITY_SENSOR) > 0); }
    void proximity_alt_avoidance_enable(bool on_off) { _proximity_alt_enabled = on_off; }

    // 辅助函数

    // 限制desired_vel_cms在单位向量limit_direction方向上的分量
    // 使其最多为limit_distance_cm允许的最大速度
    // 使用Randy在此线程中第二封电子邮件中的速度调整想法:
    //   https://groups.google.com/forum/#!searchin/drones-discuss/obstacle/drones-discuss/QwUXz__WuqY/qo3G8iTLSJAJ
    void limit_velocity_2D(float kP, float accel_cmss, Vector2f &desired_vel_cms, const Vector2f& limit_direction, float limit_distance_cm, float dt);
    
    // 注意:此方法用于根据3D期望速度向量限制水平和垂直速度
    // 基于传入的"margin"值限制desired_vel_cms在obstacle_vector方向上的分量
    void limit_velocity_3D(float kP, float accel_cmss, Vector3f &desired_vel_cms, const Vector3f& limit_direction, float limit_distance_cm, float kP_z, float accel_cmss_z ,float dt);
    
     // 计算使车辆停止距离恰好等于输入距离的速度
     // 对于具有非线性响应的Copter,kP应为非零
    float get_max_speed(float kP, float accel_cmss, float distance_cm, float dt) const;

    // 返回车辆应与物体保持的边距(以米为单位)
    float get_margin() const { return _margin; }

    // 返回避障将处于活动状态的最小高度(以米为单位)
    float get_min_alt() const { return _alt_min; }

    // 如果限制处于活动状态则返回true
    bool limits_active() const {return (AP_HAL::millis() - _last_limit_time) < AC_AVOID_ACTIVE_LIMIT_TIMEOUT_MS;};

    static const struct AP_Param::GroupInfo var_info[];

private:
    // 行为类型(参见BEHAVE参数)
    enum BehaviourType {
        BEHAVIOR_SLIDE = 0,  // 滑动
        BEHAVIOR_STOP = 1    // 停止
    };

    /*
     * 限制加速度,以控制避障库输出的速度变化
     * 这有助于减少车辆的抖动和突然移动
     */
    void limit_accel(const Vector3f &original_vel, Vector3f &modified_vel, float dt);

    /*
     * 调整圆形围栏的期望速度
     */
    void adjust_velocity_circle_fence(float kP, float accel_cmss, Vector2f &desired_vel_cms, Vector2f &backup_vel, float dt);

    /*
     * 调整包含和排除多边形围栏的期望速度
     */
    void adjust_velocity_inclusion_and_exclusion_polygons(float kP, float accel_cmss, Vector2f &desired_vel_cms, Vector2f &backup_vel, float dt);

    /*
     * 调整包含和排除圆的期望速度
     */
    void adjust_velocity_inclusion_circles(float kP, float accel_cmss, Vector2f &desired_vel_cms, Vector2f &backup_vel, float dt);
    void adjust_velocity_exclusion_circles(float kP, float accel_cmss, Vector2f &desired_vel_cms, Vector2f &backup_vel, float dt);

    /*
     * 调整信标围栏的期望速度
     */
    void adjust_velocity_beacon_fence(float kP, float accel_cmss, Vector2f &desired_vel_cms, Vector2f &backup_vel, float dt);

    /*
     * 基于近距离传感器输出调整期望速度
     */
    void adjust_velocity_proximity(float kP, float accel_cmss, Vector3f &desired_vel_cms, Vector3f &backup_vel, float kP_z, float accel_cmss_z, float dt);

    /*
     * 根据边界点数组调整期望速度
     * 边界必须在地球坐标系中
     * margin是车辆应在多边形前停止的距离(以米为单位)
     * stay_inside对于围栏应为true,对于排除多边形应为false
     */
    void adjust_velocity_polygon(float kP, float accel_cmss, Vector2f &desired_vel_cms, Vector2f &backup_vel, const Vector2f* boundary, uint16_t num_points, float margin, float dt, bool stay_inside);

    /*
     * 计算给定当前速度所需的停止距离
     */
    float get_stopping_distance(float kP, float accel_cmss, float speed_cms) const;

   /*
    * 计算避免超出边距所需的后退速度
    * 输入:此方法需要边距超出距离(back_distance_cm)、朝向超出的方向(limit_direction)
    *      然后计算所需的后退速度并将其传递给"find_max_quadrant_velocity"方法以将速度向量分配到相应象限
    * 输出:该方法然后输出四个速度(quad1/2/3/4_back_vel_cms),对应于每个象限中的最终所需后退速度
    */
    void calc_backup_velocity_2D(float kP, float accel_cmss, Vector2f &quad1_back_vel_cms, Vector2f &qua2_back_vel_cms, Vector2f &quad3_back_vel_cms, Vector2f &quad4_back_vel_cms, float back_distance_cm, Vector2f limit_direction, float dt);
    
    /*
    * 计算避免超出边距所需的后退速度,包括垂直分量
    * min_z_vel <= 0,存储向下方向的最大速度
    * max_z_vel >= 0,存储向上方向的最大速度
    * 最终max_z_vel + min_z_vel将给出最终所需的Z后退速度
    */
    void calc_backup_velocity_3D(float kP, float accel_cmss, Vector2f &quad1_back_vel_cms, Vector2f &quad2_back_vel_cms, Vector2f &quad3_back_vel_cms, Vector2f &quad4_back_vel_cms, float back_distance_cms, Vector3f limit_direction, float kp_z, float accel_cmss_z, float back_distance_z, float& min_z_vel, float& max_z_vel, float dt);
   
   /*
    * 计算可以在每个象限形成的最大速度向量
    * 此方法接受所需的后退速度和对应于每个象限的其他四个速度
    * 然后根据其分量的符号将所需速度拟合到4个象限速度之一中
    * 这确保我们有多个后退速度,我们可以在每个象限中获得所有这些速度的最大值
    */
    void find_max_quadrant_velocity(Vector2f &desired_vel, Vector2f &quad1_vel, Vector2f &quad2_vel, Vector2f &quad3_vel, Vector2f &quad4_vel);

    /*
    * 计算可以在每个象限形成的最大速度向量,并分别存储垂直分量的最大值和最小值
    */
    void find_max_quadrant_velocity_3D(Vector3f &desired_vel, Vector2f &quad1_vel, Vector2f &quad2_vel, Vector2f &quad3_vel, Vector2f &quad4_vel, float &max_z_vel, float &min_z_vel);

    /*
     * 非GPS飞行模式下的避障方法
     */

    // 将距离(以米为单位)转换为倾斜百分比(0~1范围)以用于手动飞行模式
    float distance_to_lean_pct(float dist_m);

    // 根据近距离传感器返回最大正负横滚和俯仰百分比(-1~+1范围)
    void get_proximity_roll_pitch_pct(float &roll_positive, float &roll_negative, float &pitch_positive, float &pitch_negative);

    // 日志记录函数
    void Write_SimpleAvoidance(const uint8_t state, const Vector3f& desired_vel, const Vector3f& modified_vel, const bool back_up) const;

    // 参数
    AP_Int8 _enabled;                // 启用状态
    AP_Int16 _angle_max;            // 避障的最大倾斜角度(仅在非GPS飞行模式下使用)
    AP_Float _dist_max;             // 在非GPS模式下开始避障的与物体的距离(以米为单位)
    AP_Float _margin;               // 车辆在GPS模式下将尝试与物体保持此距离(以米为单位)
    AP_Int8 _behavior;              // 避障行为(滑动或停止)
    AP_Float _backup_speed_xy_max;  // 用于水平后退的最大速度(以米/秒为单位)
    AP_Float _backup_speed_z_max;   // 用于垂直后退的最大速度(以米/秒为单位)
    AP_Float _alt_min;              // 低于此高度时关闭基于近距离的避障
    AP_Float _accel_max;            // 简单避障活动时的最大加速度
    AP_Float _backup_deadzone;      // AVOID_MARGIN参数之外的距离,超过此距离后车辆将远离障碍物

    bool _proximity_enabled = true;  // 如果启用基于近距离传感器的避障则为true(用于允许飞行员启用/禁用)
    bool _proximity_alt_enabled = true; // 如果基于高度启用基于近距离传感器的避障则为true
    uint32_t _last_limit_time;       // 限制处于活动状态的最后时间
    uint32_t _last_log_ms;           // 上次记录简单避障的时间
    Vector3f _prev_avoid_vel;        // 避障调整速度的副本

    static AC_Avoid *_singleton;
};

namespace AP {
    AC_Avoid *ac_avoid();
};

#endif  // AP_AVOIDANCE_ENABLED
