#pragma once

// 包含必要的头文件
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>
#include <AP_GPS/AP_GPS_config.h>
#include <AP_BoardConfig/AP_BoardConfig_config.h>

#include "AP_Arming_config.h"
#include "AP_InertialSensor/AP_InertialSensor_config.h"
#include "AP_Proximity/AP_Proximity_config.h"

// AP_Arming类用于处理飞行器解锁相关功能
class AP_Arming {
public:

    // 构造函数
    AP_Arming();

    // 禁止拷贝构造
    CLASS_NO_COPY(AP_Arming);  /* Do not allow copies */

    // 获取单例对象
    static AP_Arming *get_singleton();

    // 更新函数
    void update();

    // 解锁检查项枚举
    enum ArmingChecks {
        ARMING_CHECK_ALL         = (1U << 0),  // 全部检查
        ARMING_CHECK_BARO        = (1U << 1),  // 气压计检查
        ARMING_CHECK_COMPASS     = (1U << 2),  // 罗盘检查
        ARMING_CHECK_GPS         = (1U << 3),  // GPS检查
        ARMING_CHECK_INS         = (1U << 4),  // 惯性导航检查
        ARMING_CHECK_PARAMETERS  = (1U << 5),  // 参数检查
        ARMING_CHECK_RC          = (1U << 6),  // 遥控器检查
        ARMING_CHECK_VOLTAGE     = (1U << 7),  // 电压检查
        ARMING_CHECK_BATTERY     = (1U << 8),  // 电池检查
        ARMING_CHECK_AIRSPEED    = (1U << 9),  // 空速检查
        ARMING_CHECK_LOGGING     = (1U << 10), // 日志检查
        ARMING_CHECK_SWITCH      = (1U << 11), // 开关检查
        ARMING_CHECK_GPS_CONFIG  = (1U << 12), // GPS配置检查
        ARMING_CHECK_SYSTEM      = (1U << 13), // 系统检查
        ARMING_CHECK_MISSION     = (1U << 14), // 任务检查
        ARMING_CHECK_RANGEFINDER = (1U << 15), // 测距仪检查
        ARMING_CHECK_CAMERA      = (1U << 16), // 相机检查
        ARMING_CHECK_AUX_AUTH    = (1U << 17), // 辅助认证检查
        ARMING_CHECK_VISION      = (1U << 18), // 视觉检查
        ARMING_CHECK_FFT         = (1U << 19), // FFT检查
        ARMING_CHECK_OSD         = (1U << 20), // OSD检查
    };

    // 解锁方式枚举
    enum class Method {
        RUDDER = 0,                    // 方向舵解锁
        MAVLINK = 1,                   // MAVLink解锁
        AUXSWITCH = 2,                 // 辅助开关解锁
        MOTORTEST = 3,                 // 电机测试解锁
        SCRIPTING = 4,                 // 脚本解锁
        TERMINATION = 5,               // 终止
        CPUFAILSAFE = 6,              // CPU故障保护
        BATTERYFAILSAFE = 7,          // 电池故障保护
        SOLOPAUSEWHENLANDED = 8,      // Solo着陆暂停
        AFS = 9,                       // 自动飞行系统
        ADSBCOLLISIONACTION = 10,      // ADS-B防撞动作
        PARACHUTE_RELEASE = 11,        // 降落伞释放
        CRASH = 12,                    // 坠毁
        LANDED = 13,                   // 已着陆
        MISSIONEXIT = 14,              // 任务退出
        FENCEBREACH = 15,              // 围栏突破
        RADIOFAILSAFE = 16,            // 遥控器故障保护
        DISARMDELAY = 17,              // 解锁延迟
        GCSFAILSAFE = 18,              // 地面站故障保护
        TERRRAINFAILSAFE = 19,         // 地形故障保护
        FAILSAFE_ACTION_TERMINATE = 20, // 故障保护终止动作
        TERRAINFAILSAFE = 21,          // 地形故障保护
        MOTORDETECTDONE = 22,          // 电机检测完成
        BADFLOWOFCONTROL = 23,         // 控制流异常
        EKFFAILSAFE = 24,              // EKF故障保护
        GCS_FAILSAFE_SURFACEFAILED = 25, // 地面站故障保护-表面失败
        GCS_FAILSAFE_HOLDFAILED = 26,    // 地面站故障保护-保持失败
        TAKEOFFTIMEOUT = 27,           // 起飞超时
        AUTOLANDED = 28,               // 自动着陆
        PILOT_INPUT_FAILSAFE = 29,     // 飞手输入故障保护
        TOYMODELANDTHROTTLE = 30,      // 玩具模式和油门
        TOYMODELANDFORCE = 31,         // 玩具模式和力
        LANDING = 32,                  // 着陆
        DEADRECKON_FAILSAFE = 33,      // 航位推算故障保护
        BLACKBOX = 34,                 // 黑匣子
        DDS = 35,                      // DDS
        UNKNOWN = 100,                 // 未知
    };

    // 解锁要求枚举
    enum class Required {
        NO           = 0,              // 不需要
        YES_MIN_PWM  = 1,              // 需要最小PWM
        YES_ZERO_PWM = 2               // 需要零PWM
    };

    // 初始化函数
    void init(void);

    // 获取解锁要求
    Required arming_required() const;
    // 解锁函数
    virtual bool arm(AP_Arming::Method method, bool do_arming_checks=true);
    // 强制解锁函数
    virtual bool arm_force(AP_Arming::Method method) { return arm(method, false); }
    // 上锁函数
    virtual bool disarm(AP_Arming::Method method, bool do_disarm_checks=true);
    // 检查是否已解锁
    bool is_armed() const;
    // 检查是否已解锁且安全开关关闭
    bool is_armed_and_safety_off() const;

    // 获取启用的检查项位掩码
    uint32_t get_enabled_checks() const;

    // 解锁前检查函数
    virtual bool pre_arm_checks(bool report);

    // 解锁检查函数
    virtual bool arm_checks(AP_Arming::Method method);

    // 获取预期的磁场强度
    uint16_t compass_magfield_expected() const;

    // 方向舵解锁支持枚举
    enum class RudderArming {
        IS_DISABLED  = 0,              // 禁用
        ARMONLY   = 1,                 // 仅解锁
        ARMDISARM = 2                  // 解锁和上锁
    };

    // 获取方向舵解锁类型
    RudderArming get_rudder_arming_type() const { return (RudderArming)_rudder_arming.get(); }

#if AP_ARMING_AUX_AUTH_ENABLED
    // 辅助认证方法
    bool get_aux_auth_id(uint8_t& auth_id);
    void set_aux_auth_passed(uint8_t auth_id);
    void set_aux_auth_failed(uint8_t auth_id, const char* fail_msg);
#endif

    // 参数组信息
    static const struct AP_Param::GroupInfo        var_info[];

    // 获取最后一次上锁方式
    Method last_disarm_method() const { return _last_disarm_method; }

    // 获取最后一次解锁方式
    Method last_arm_method() const { return _last_arm_method; }
    
    // 解锁选项枚举
    enum class Option : int32_t {
        DISABLE_PREARM_DISPLAY             = (1U << 0),  // 禁用解锁前显示
        DISABLE_STATUSTEXT_ON_STATE_CHANGE = (1U << 1),  // 禁用状态改变文本
    };
    // 检查选项是否启用
    bool option_enabled(Option option) const {
        return (_arming_options & uint32_t(option)) != 0;
    }

    // 发送解锁/上锁状态文本
    void send_arm_disarm_statustext(const char *string) const;

    // 检查解锁方式是否为地面站
    static bool method_is_GCS(Method method) {
        return (method == Method::MAVLINK || method == Method::DDS);
    }
protected:

    // 参数
    AP_Enum<Required>       require;                    // 解锁要求
    AP_Int32                checks_to_perform;          // 需要执行的检查项位掩码
    AP_Float                accel_error_threshold;      // 加速度错误阈值
    AP_Int8                 _rudder_arming;            // 方向舵解锁设置
    AP_Int32                _required_mission_items;    // 所需任务项
    AP_Int32                _arming_options;           // 解锁选项
    AP_Int16                magfield_error_threshold;   // 磁场错误阈值

    // 内部成员
    bool                    armed;                      // 解锁状态
    uint32_t                last_accel_pass_ms;        // 最后一次加速度检查通过时间
    uint32_t                last_gyro_pass_ms;         // 最后一次陀螺仪检查通过时间

    // 检查函数
    virtual bool barometer_checks(bool report);         // 气压计检查
    bool airspeed_checks(bool report);                 // 空速检查
    bool logging_checks(bool report);                  // 日志检查

#if AP_INERTIALSENSOR_ENABLED
    // 惯性传感器检查
    virtual bool ins_checks(bool report);
#endif

    // 罗盘检查
    bool compass_checks(bool report);

    // GPS检查
    virtual bool gps_checks(bool report);

    // 电池检查
    bool battery_checks(bool report);

    // 硬件安全开关检查
    bool hardware_safety_check(bool report);

    // 板载电压检查
    virtual bool board_voltage_checks(bool report);

    // 遥控器校准检查
    virtual bool rc_calibration_checks(bool report);

    // 遥控器输入校准检查
    bool rc_in_calibration_check(bool report);

    // 遥控器解锁检查
    bool rc_arm_checks(AP_Arming::Method method);

    // 手动遥控器检查
    bool manual_transmitter_checks(bool report);

    // 任务检查
    virtual bool mission_checks(bool report);

    // 地形检查
    bool terrain_checks(bool report) const;

    // 检查是否需要加载所有地形数据
    // 期望返回true如果需要加载所有地形数据
    virtual bool terrain_database_required() const;

    // 测距仪检查
    bool rangefinder_checks(bool report);

    // 围栏检查
    bool fence_checks(bool report);

#if HAL_HAVE_IMU_HEATER
    // IMU加热器最低温度检查
    bool heater_min_temperature_checks(bool report);
#endif

    // 相机检查
    bool camera_checks(bool display_failure);

    // OSD检查
    bool osd_checks(bool display_failure) const;

    // 云台检查
    bool mount_checks(bool display_failure) const;

#if AP_ARMING_AUX_AUTH_ENABLED
    // 辅助认证检查
    bool aux_auth_checks(bool display_failure);
#endif

    // 发电机检查
    bool generator_checks(bool report) const;

    // OpenDroneID检查
    bool opendroneid_checks(bool display_failure);
    
    // 串口协议检查
    bool serial_protocol_checks(bool display_failure);
    
    // 紧急停止检查
    bool estop_checks(bool display_failure);

#if AP_ARMING_CRASHDUMP_ACK_ENABLED
    // 崩溃转储确认检查
    bool crashdump_checks(bool report);
#endif

    // 系统检查
    virtual bool system_checks(bool report);

    // CAN总线检查
    bool can_checks(bool report);

    // FETtec单线协议检查
    bool fettec_checks(bool display_failure) const;

#if HAL_PROXIMITY_ENABLED
    // 距离传感器检查
    virtual bool proximity_checks(bool report) const;
#endif

    // 舵机检查
    bool servo_checks(bool report) const;
    // 多轴飞行器和潜水器的遥控器检查
    bool rc_checks_copter_sub(bool display_failure, const class RC_Channel *channels[4]) const;

    // 视觉里程计检查
    bool visodom_checks(bool report) const;
    // 解锁开关检查
    bool disarm_switch_checks(bool report) const;

    // 强制检查项,不能被跳过。仅当ARMING_CHECK为0或强制解锁时调用
    virtual bool mandatory_checks(bool report);

    // 检查特定检查项是否启用
    bool check_enabled(const enum AP_Arming::ArmingChecks check) const;
    // 处理检查失败的情况
    void check_failed(const enum AP_Arming::ArmingChecks check, bool report, const char *fmt, ...) const FMT_PRINTF(4, 5);
    void check_failed(bool report, const char *fmt, ...) const FMT_PRINTF(3, 4);

    // 记录解锁日志
    void Log_Write_Arm(bool forced, AP_Arming::Method method);
    // 记录上锁日志
    void Log_Write_Disarm(bool forced, AP_Arming::Method method);

private:

    // 单例指针
    static AP_Arming *_singleton;

#if AP_INERTIALSENSOR_ENABLED
    // 检查加速度计一致性
    bool ins_accels_consistent(const class AP_InertialSensor &ins);
    // 检查陀螺仪一致性
    bool ins_gyros_consistent(const class AP_InertialSensor &ins);
#endif

    // 检查是否需要在解锁后继续记录日志
    void check_forced_logging(const AP_Arming::Method method);

    // 任务项检查枚举
    enum MIS_ITEM_CHECK {
        MIS_ITEM_CHECK_LAND          = (1 << 0),  // 着陆
        MIS_ITEM_CHECK_VTOL_LAND     = (1 << 1),  // 垂直起降着陆
        MIS_ITEM_CHECK_DO_LAND_START = (1 << 2),  // 开始着陆
        MIS_ITEM_CHECK_TAKEOFF       = (1 << 3),  // 起飞
        MIS_ITEM_CHECK_VTOL_TAKEOFF  = (1 << 4),  // 垂直起降起飞
        MIS_ITEM_CHECK_RALLY         = (1 << 5),  // 集结点
        MIS_ITEM_CHECK_RETURN_TO_LAUNCH = (1 << 6), // 返航
        MIS_ITEM_CHECK_MAX
    };

#if AP_ARMING_AUX_AUTH_ENABLED
    // 辅助认证相关
    static const uint8_t aux_auth_count_max = 3;    // 最大辅助认证数量
    static const uint8_t aux_auth_str_len = 42;     // 失败消息最大长度(50-8用于"PreArm: ")
    enum class AuxAuthStates : uint8_t {
        NO_RESPONSE = 0,  // 无响应
        AUTH_FAILED,      // 认证失败
        AUTH_PASSED       // 认证通过
    } aux_auth_state[aux_auth_count_max] = {};  // 每个辅助认证的状态
    uint8_t aux_auth_count;     // 辅助认证数量
    uint8_t aux_auth_fail_msg_source;   // 设置失败消息的认证ID
    char* aux_auth_fail_msg;    // 失败消息缓冲区
    bool aux_auth_error;        // 辅助认证过多时为true
    HAL_Semaphore aux_auth_sem; // 用于访问aux_auth_state和aux_auth_fail_msg的信号量
#endif

    // 最后使用的解锁/上锁方式,仅在飞行器至少解锁过一次后有效
    Method _last_disarm_method = Method::UNKNOWN;
    Method _last_arm_method = Method::UNKNOWN;

    uint32_t last_prearm_display_ms;  // 上次发送解锁前状态文本的时间
    bool running_arming_checks;  // 当前正在执行解锁检查时为true,表示飞行器正在尝试解锁

    bool last_prearm_checks_result; // 上次解锁前检查结果
    bool report_immediately; // 当检查从通过变为失败时设为true,触发立即报告

    // 更新解锁GPIO
    void update_arm_gpio();

#if !AP_GPS_BLENDED_ENABLED
    // GPS混合自动切换检查
    bool blending_auto_switch_checks(bool report);
#endif

#if AP_ARMING_CRASHDUMP_ACK_ENABLED
    struct CrashDump {
        void check_reset();
        AP_Int8  acked;  // 崩溃转储确认标志
    } crashdump_ack;
#endif  // AP_ARMING_CRASHDUMP_ACK_ENABLED

};

namespace AP {
    // 获取解锁单例对象
    AP_Arming &arming();
};
