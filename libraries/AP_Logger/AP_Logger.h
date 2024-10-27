/* ************************************************************ */
/* Test for AP_Logger Log library                               */
/* AP_Logger日志库测试                                          */
/* ************************************************************ */
#pragma once

#include "AP_Logger_config.h"

#if HAL_LOGGING_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Logger/LogStructure.h>
#include <AP_Vehicle/ModeReason.h>

#include <stdint.h>

#include "LoggerMessageWriter.h"

class AP_Logger_Backend;

// do not do anything here apart from add stuff; maintaining older
// entries means log analysis is easier
// 不要在这里做任何修改,只添加新内容;保持旧条目有助于日志分析
enum class LogEvent : uint8_t {
    ARMED = 10,                      // 系统武装
    DISARMED = 11,                   // 系统解除武装
    AUTO_ARMED = 15,                 // 自动武装
    LAND_COMPLETE_MAYBE = 17,        // 可能已着陆
    LAND_COMPLETE = 18,              // 着陆完成
    NOT_LANDED = 28,                 // 未着陆
    LOST_GPS = 19,                   // GPS信号丢失
    FLIP_START = 21,                 // 翻转开始
    FLIP_END = 22,                   // 翻转结束
    SET_HOME = 25,                   // 设置Home点
    SET_SIMPLE_ON = 26,              // 开启简单模式
    SET_SIMPLE_OFF = 27,             // 关闭简单模式
    SET_SUPERSIMPLE_ON = 29,         // 开启超级简单模式
    AUTOTUNE_INITIALISED = 30,       // 自动调参初始化
    AUTOTUNE_OFF = 31,               // 自动调参关闭
    AUTOTUNE_RESTART = 32,           // 自动调参重启
    AUTOTUNE_SUCCESS = 33,           // 自动调参成功
    AUTOTUNE_FAILED = 34,            // 自动调参失败
    AUTOTUNE_REACHED_LIMIT = 35,     // 自动调参达到限制
    AUTOTUNE_PILOT_TESTING = 36,     // 自动调参飞手测试
    AUTOTUNE_SAVEDGAINS = 37,        // 自动调参保存增益
    SAVE_TRIM = 38,                  // 保存微调
    SAVEWP_ADD_WP = 39,             // 保存航点
    FENCE_ENABLE = 41,               // 启用围栏
    FENCE_DISABLE = 42,              // 禁用围栏
    ACRO_TRAINER_OFF = 43,           // 特技训练模式关闭
    ACRO_TRAINER_LEVELING = 44,      // 特技训练模式水平
    ACRO_TRAINER_LIMITED = 45,       // 特技训练模式受限
    GRIPPER_GRAB = 46,               // 抓取器抓取
    GRIPPER_RELEASE = 47,            // 抓取器释放
    PARACHUTE_DISABLED = 49,         // 降落伞禁用
    PARACHUTE_ENABLED = 50,          // 降落伞启用
    PARACHUTE_RELEASED = 51,         // 降落伞释放
    LANDING_GEAR_DEPLOYED = 52,      // 起落架放下
    LANDING_GEAR_RETRACTED = 53,     // 起落架收起
    MOTORS_EMERGENCY_STOPPED = 54,    // 电机紧急停止
    MOTORS_EMERGENCY_STOP_CLEARED = 55, // 电机紧急停止解除
    MOTORS_INTERLOCK_DISABLED = 56,   // 电机联锁禁用
    MOTORS_INTERLOCK_ENABLED = 57,    // 电机联锁启用
    ROTOR_RUNUP_COMPLETE = 58,       // 旋翼预热完成(仅直升机)
    ROTOR_SPEED_BELOW_CRITICAL = 59,  // 旋翼速度低于临界值(仅直升机)
    EKF_ALT_RESET = 60,              // EKF高度重置
    LAND_CANCELLED_BY_PILOT = 61,     // 飞手取消着陆
    EKF_YAW_RESET = 62,              // EKF偏航重置
    AVOIDANCE_ADSB_ENABLE = 63,      // 启用ADSB避障
    AVOIDANCE_ADSB_DISABLE = 64,     // 禁用ADSB避障
    AVOIDANCE_PROXIMITY_ENABLE = 65,  // 启用近距离避障
    AVOIDANCE_PROXIMITY_DISABLE = 66, // 禁用近距离避障
    GPS_PRIMARY_CHANGED = 67,         // GPS主要来源改变
    // 68, 69, 70 were winch events  // 68,69,70是绞车事件
    ZIGZAG_STORE_A = 71,             // 之字形存储A点
    ZIGZAG_STORE_B = 72,             // 之字形存储B点
    LAND_REPO_ACTIVE = 73,           // 着陆重新定位激活
    STANDBY_ENABLE = 74,             // 启用待机
    STANDBY_DISABLE = 75,            // 禁用待机

    // Fence events                   // 围栏事件
    FENCE_ALT_MAX_ENABLE = 76,       // 启用最大高度围栏
    FENCE_ALT_MAX_DISABLE = 77,      // 禁用最大高度围栏
    FENCE_CIRCLE_ENABLE = 78,        // 启用圆形围栏
    FENCE_CIRCLE_DISABLE = 79,       // 禁用圆形围栏
    FENCE_ALT_MIN_ENABLE = 80,       // 启用最小高度围栏
    FENCE_ALT_MIN_DISABLE = 81,      // 禁用最小高度围栏
    FENCE_POLYGON_ENABLE = 82,       // 启用多边形围栏
    FENCE_POLYGON_DISABLE = 83,      // 禁用多边形围栏

    // if the EKF's source input set is changed (e.g. via a switch or
    // a script), we log an event:
    // 如果EKF的输入源发生改变(如通过开关或脚本),记录事件:
    EK3_SOURCES_SET_TO_PRIMARY = 85,   // EKF3设置为主要来源
    EK3_SOURCES_SET_TO_SECONDARY = 86, // EKF3设置为次要来源
    EK3_SOURCES_SET_TO_TERTIARY = 87,  // EKF3设置为第三来源

    AIRSPEED_PRIMARY_CHANGED = 90,     // 空速主要来源改变

    SURFACED = 163,                    // 浮出水面
    NOT_SURFACED = 164,                // 未浮出水面
    BOTTOMED = 165,                    // 触底
    NOT_BOTTOMED = 166,                // 未触底
};

// 日志数据ID枚举
enum class LogDataID : uint8_t {
    AP_STATE = 7,                      // AP状态
// SYSTEM_TIME_SET = 8,                // 系统时间设置
    INIT_SIMPLE_BEARING = 9,           // 初始化简单方位
};

// 日志错误子系统枚举
enum class LogErrorSubsystem : uint8_t {
    MAIN = 1,                          // 主系统
    RADIO = 2,                         // 遥控
    COMPASS = 3,                       // 罗盘
    OPTFLOW = 4,   // not used         // 光流(未使用)
    FAILSAFE_RADIO = 5,                // 遥控故障保护
    FAILSAFE_BATT = 6,                 // 电池故障保护
    FAILSAFE_GPS = 7,   // not used    // GPS故障保护(未使用)
    FAILSAFE_GCS = 8,                  // 地面站故障保护
    FAILSAFE_FENCE = 9,                // 围栏故障保护
    FLIGHT_MODE = 10,                  // 飞行模式
    GPS = 11,                          // GPS
    CRASH_CHECK = 12,                  // 坠机检查
    FLIP = 13,                         // 翻转
    AUTOTUNE = 14,  // not used        // 自动调参(未使用)
    PARACHUTES = 15,                   // 降落伞
    EKFCHECK = 16,                     // EKF检查
    FAILSAFE_EKFINAV = 17,             // EKF导航故障保护
    BARO = 18,                         // 气压计
    CPU = 19,                          // CPU
    FAILSAFE_ADSB = 20,                // ADSB故障保护
    TERRAIN = 21,                      // 地形
    NAVIGATION = 22,                   // 导航
    FAILSAFE_TERRAIN = 23,             // 地形故障保护
    EKF_PRIMARY = 24,                  // EKF主要
    THRUST_LOSS_CHECK = 25,            // 推力损失检查
    FAILSAFE_SENSORS = 26,             // 传感器故障保护
    FAILSAFE_LEAK = 27,                // 泄漏故障保护
    PILOT_INPUT = 28,                  // 飞手输入
    FAILSAFE_VIBE = 29,                // 振动故障保护
    INTERNAL_ERROR = 30,               // 内部错误
    FAILSAFE_DEADRECKON = 31           // 推算导航故障保护
};

// bizarrely this enumeration has lots of duplicate values, offering
// very little in the way of typesafety
// 奇怪的是这个枚举有很多重复值,在类型安全方面提供的保护很少
enum class LogErrorCode : uint8_t {
// general error codes                  // 通用错误代码
    ERROR_RESOLVED  = 0,               // 错误已解决
    FAILED_TO_INITIALISE = 1,          // 初始化失败
    UNHEALTHY = 4,                     // 不健康

// subsystem specific error codes -- radio  // 子系统特定错误代码 -- 遥控
    RADIO_LATE_FRAME = 2,              // 遥控帧延迟

// subsystem specific error codes -- failsafe_thr, batt, gps  // 子系统特定错误代码 -- 油门故障保护,电池,GPS
    FAILSAFE_RESOLVED = 0,             // 故障保护已解决
    FAILSAFE_OCCURRED = 1,             // 发生故障保护

// subsystem specific error codes -- main  // 子系统特定错误代码 -- 主系统
    MAIN_INS_DELAY = 1,                // 主惯性系统延迟

// subsystem specific error codes -- crash checker  // 子系统特定错误代码 -- 坠机检查器
    CRASH_CHECK_CRASH = 1,             // 检测到坠机
    CRASH_CHECK_LOSS_OF_CONTROL = 2,   // 检测到失控

// subsystem specific error codes -- flip  // 子系统特定错误代码 -- 翻转
    FLIP_ABANDONED = 2,                // 翻转已放弃

// subsystem specific error codes -- terrain  // 子系统特定错误代码 -- 地形
    MISSING_TERRAIN_DATA = 2,          // 缺少地形数据

// subsystem specific error codes -- navigation  // 子系统特定错误代码 -- 导航
    FAILED_TO_SET_DESTINATION = 2,     // 设置目的地失败
    RESTARTED_RTL = 3,                 // 重启返航
    FAILED_CIRCLE_INIT = 4,            // 圆形初始化失败
    DEST_OUTSIDE_FENCE = 5,            // 目的地在围栏外
    RTL_MISSING_RNGFND = 6,            // 返航缺少测距仪

// subsystem specific error codes -- internal_error  // 子系统特定错误代码 -- 内部错误
    INTERNAL_ERRORS_DETECTED = 1,      // 检测到内部错误

// parachute failed to deploy because of low altitude or landed
// 由于高度过低或已着陆导致降落伞未能部署
    PARACHUTE_TOO_LOW = 2,             // 降落伞高度过低
    PARACHUTE_LANDED = 3,              // 降落伞已着陆

// EKF check definitions               // EKF检查定义
    EKFCHECK_BAD_VARIANCE = 2,         // EKF方差异常
    EKFCHECK_VARIANCE_CLEARED = 0,     // EKF方差已清除

// Baro specific error codes           // 气压计特定错误代码
    BARO_GLITCH = 2,                   // 气压计故障
    BAD_DEPTH = 3, // sub-only         // 深度异常(仅限潜水器)

// GPS specific error codes            // GPS特定错误代码
    GPS_GLITCH = 2,                    // GPS故障
};

// AP_Logger类定义 - 主要的日志记录类
class AP_Logger
{
    // 允许AP_Logger_Backend访问_num_types
    friend class AP_Logger_Backend; 
    // 允许AP_Logger_RateLimiter访问
    friend class AP_Logger_RateLimiter;

public:
    // 定义vehicle_startup_message_Writer函数类型
    FUNCTOR_TYPEDEF(vehicle_startup_message_Writer, void);

    // 构造函数
    AP_Logger();

    /* 禁止拷贝构造 */
    CLASS_NO_COPY(AP_Logger);

    // 获取单例实例
    static AP_Logger *get_singleton(void) {
        return _singleton;
    }

    // 初始化函数
    void init(const AP_Int32 &log_bitmask, const struct LogStructure *structure, uint8_t num_types);
    // 设置日志类型数量
    void set_num_types(uint8_t num_types) { _num_types = num_types; }

    // 检查SD卡是否插入
    bool CardInserted(void);
    // 日志暂停标志
    bool _log_pause;

    // 如果辅助开关激活且日志速率限制启用,则暂停日志记录
    void log_pause(bool value) {
        _log_pause = value;
    }

    // 擦除所有日志
    void EraseAll();

    /* 在当前偏移量写入数据块 */
    void WriteBlock(const void *pBuffer, uint16_t size);

    /* 在当前偏移量写入数据块,如果第一个后端成功则返回true */
    bool WriteBlock_first_succeed(const void *pBuffer, uint16_t size);

    /* 在当前偏移量写入重要数据块 */
    void WriteCriticalBlock(const void *pBuffer, uint16_t size);

    /* 在当前偏移量写入回放数据块 */
    bool WriteReplayBlock(uint8_t msg_id, const void *pBuffer, uint16_t size);

    // 高级接口
    uint16_t find_last_log() const;
    void get_log_boundaries(uint16_t log_num, uint32_t & start_page, uint32_t & end_page);
    uint16_t get_num_logs(void);
    uint16_t get_max_num_logs();

    // 设置vehicle启动消息写入器
    void setVehicle_Startup_Writer(vehicle_startup_message_Writer writer);

    // 准备解锁
    void PrepForArming();

    // 启用/禁用写入
    void EnableWrites(bool enable) { _writes_enabled = enable; }
    bool WritesEnabled() const { return _writes_enabled; }

    // 停止日志记录
    void StopLogging();

    // 写入各种类型的日志数据
    void Write_Parameter(const char *name, float value);
    void Write_Event(LogEvent id);
    void Write_Error(LogErrorSubsystem sub_system,
                     LogErrorCode error_code);
    void Write_RCIN(void);
    void Write_RCOUT(void);
    void Write_RSSI();
    void Write_Rally();
#if HAL_LOGGER_FENCE_ENABLED
    void Write_Fence();
#endif
    void Write_NamedValueFloat(const char *name, float value);
    void Write_Power(void);
    void Write_Radio(const mavlink_radio_t &packet);
    void Write_Message(const char *message);
    void Write_MessageF(const char *fmt, ...);
    void Write_ServoStatus(uint64_t time_us, uint8_t id, float position, float force, float speed, uint8_t power_pct,
                           float pos_cmd, float voltage, float current, float mot_temp, float pcb_temp, uint8_t error);
    void Write_Compass();
    void Write_Mode(uint8_t mode, const ModeReason reason);

    // 写入完整任务
    void Write_EntireMission();
    void Write_Command(const mavlink_command_int_t &packet,
                       uint8_t source_system,
                       uint8_t source_component,
                       MAV_RESULT result,
                       bool was_command_long=false);
    void Write_Mission_Cmd(const AP_Mission &mission,
                               const AP_Mission::Mission_Command &cmd);
    void Write_RallyPoint(uint8_t total,
                          uint8_t sequence,
                          const class RallyLocation &rally_point);
    void Write_SRTL(bool active, uint16_t num_points, uint16_t max_points, uint8_t action, const Vector3f& point);
    void Write_Winch(bool healthy, bool thread_end, bool moving, bool clutch, uint8_t mode, float desired_length, float length, float desired_rate, uint16_t tension, float voltage, int8_t temp);

    // 通用写入接口
    void Write(const char *name, const char *labels, const char *fmt, ...);
    void Write(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, ...);
    void WriteStreaming(const char *name, const char *labels, const char *fmt, ...);
    void WriteStreaming(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, ...);
    void WriteCritical(const char *name, const char *labels, const char *fmt, ...);
    void WriteCritical(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, ...);
    void WriteV(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, va_list arg_list, bool is_critical=false, bool is_streaming=false);

    // 写入PID控制器信息
    void Write_PID(uint8_t msg_type, const class AP_PIDInfo &info);

    // 判断是否应该记录某个消息
    bool should_log(uint32_t mask) const;

    // 判断日志是否已开始记录
    bool logging_started(void);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    // 目前只有AP_Logger_File支持此功能:
    void flush(void);
#endif

    // 处理MAVLink消息
    void handle_mavlink_msg(class GCS_MAVLINK &, const mavlink_message_t &msg);

    // 周期性任务
    void periodic_tasks(); // 可能需要分为GCS和非GCS任务

    // 在启动EKF前确保数据可记录
    bool allow_start_ekf() const;

    // 获取丢弃的数据块数量
    uint32_t num_dropped(void) const;

    // 访问公共参数
    void set_force_log_disarmed(bool force_logging) { _force_log_disarmed = force_logging; }
    void set_long_log_persist(bool b) { _force_long_log_persist = b; }
    bool log_while_disarmed(void) const;
    bool in_log_persistance(void) const;
    uint8_t log_replay(void) const { return _params.log_replay; }

    // 飞行器启动消息写入器
    vehicle_startup_message_Writer _vehicle_messages;

    // 解锁日志记录枚举
    enum class LogDisarmed : uint8_t {
        NONE = 0,                          // 不记录
        LOG_WHILE_DISARMED = 1,           // 解锁时记录
        LOG_WHILE_DISARMED_NOT_USB = 2,   // 解锁时记录(非USB)
        LOG_WHILE_DISARMED_DISCARD = 3,   // 解锁时记录(丢弃)
    };

    // 参数支持
    static const struct AP_Param::GroupInfo        var_info[];
    struct {
        AP_Int8 backend_types;             // 后端类型
        AP_Int16 file_bufsize;             // 文件缓冲区大小(KB)
        AP_Int8 file_disarm_rot;           // 文件解锁旋转
        AP_Enum<LogDisarmed> log_disarmed; // 解锁日志选项
        AP_Int8 log_replay;                // 日志回放
        AP_Int8 mav_bufsize;               // MAVLink缓冲区大小(KB)
        AP_Int16 file_timeout;             // 文件超时(秒)
        AP_Int16 min_MB_free;              // 最小可用空间(MB)
        AP_Float file_ratemax;             // 文件最大速率
        AP_Float mav_ratemax;              // MAVLink最大速率
        AP_Float blk_ratemax;              // 块最大速率
        AP_Float disarm_ratemax;           // 解锁最大速率
        AP_Int16 max_log_files;            // 最大日志文件数
    } _params;

    // 获取日志结构、单位和乘数信息
    const struct LogStructure *structure(uint16_t num) const;
    const struct UnitStructure *unit(uint16_t num) const;
    const struct MultiplierStructure *multiplier(uint16_t num) const;

    // 用于MAVLink SYS_STATUS消息的方法(send_sys_status)
    // 这些方法只覆盖第一个使用的日志后端 - 通常是AP_Logger_File
    bool logging_present() const;
    bool logging_enabled() const;
    bool logging_failed() const;

    // 通知日志系统解锁失败。这会触发HAL_LOGGER_ARM_PERSIST秒的日志记录
    void arming_failure() {
        _last_arming_failure_ms = AP_HAL::millis();
#if HAL_LOGGER_FILE_CONTENTS_ENABLED
        file_content_prepare_for_arming = true;
#endif
    }

    // 设置飞行器解锁状态
    void set_vehicle_armed(bool armed_state);
    bool vehicle_is_armed() const { return _armed; }

    // 处理日志发送
    void handle_log_send();
    bool in_log_download() const;

    // 返回NaN值
    float quiet_nanf() const { return NaNf; } // "AR"
    double quiet_nan() const { return nan("0x4152445550490a"); } // "ARDUPI"

    // 判断msg_type是否与消息关联
    bool msg_type_in_use(uint8_t msg_type) const;

    // 计算使用fmt中指定字段的消息长度,包括消息头
    int16_t Write_calc_msg_len(const char *fmt) const;

    // 日志写入格式结构体
    struct log_write_fmt {
        struct log_write_fmt *next;
        uint8_t msg_type;
        uint8_t msg_len;
        const char *name;
        const char *fmt;
        const char *labels;
        const char *units;
        const char *mults;
    } *log_write_fmts;

    // 为名称返回(可能分配)log_write_fmt
    struct log_write_fmt *msg_fmt_for_name(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, const bool direct_comp = false, const bool copy_strings = false);

    // 如果尚未为每个后端输出FMT消息
    void Safe_Write_Emit_FMT(log_write_fmt *f);

    // 获取开始日志记录的次数
    uint8_t get_log_start_count(void) const {
        return _log_start_count;
    }

    // 将文件名添加到要记录的文件列表中。名称必须是常量字符串,不能分配
    void log_file_content(const char *name);

protected:

    // 日志结构、单位和乘数数组
    const struct LogStructure *_structures;
    uint8_t _num_types;
    const struct UnitStructure *_units = log_Units;
    const struct MultiplierStructure *_multipliers = log_Multipliers;
    const uint8_t _num_units = (sizeof(log_Units) / sizeof(log_Units[0]));
    const uint8_t _num_multipliers = (sizeof(log_Multipliers) / sizeof(log_Multipliers[0]));

    /* 写入具有指定重要性的数据块 */
    /* 如果有布尔值表示消息很重要可能会有用... */
    void WritePrioritisedBlock(const void *pBuffer, uint16_t size,
                               bool is_critical);

private:
    #define LOGGER_MAX_BACKENDS 2
    uint8_t _next_backend;
    AP_Logger_Backend *backends[LOGGER_MAX_BACKENDS];
    const AP_Int32 *_log_bitmask;

    // 后端类型枚举
    enum class Backend_Type : uint8_t {
        NONE       = 0,
        FILESYSTEM = (1<<0),
        MAVLINK    = (1<<1),
        BLOCK      = (1<<2),
    };

    // RC日志标志枚举
    enum class RCLoggingFlags : uint8_t {
        HAS_VALID_INPUT = 1U<<0,  // 系统接收到有效RC值时为true
        IN_RC_FAILSAFE =  1U<<1,  // 系统当前处于RC故障保护状态时为true
    };

    /*
     * 支持动态Write;用户在单个函数调用中提供名称、格式、
     * 标签和值。
     */
    HAL_Semaphore log_write_fmts_sem;

    // 为msg_type返回(可能分配)log_write_fmt
    const struct log_write_fmt *log_write_fmt_for_msg_type(uint8_t msg_type) const;

    // 为msg_type返回LogStructure
    const struct LogStructure *structure_for_msg_type(uint8_t msg_type) const;

    // 返回当前未使用的msg_type(如果没有可用则返回-1)
    int16_t find_free_msg_type() const;

    // 用msg_type的信息填充LogStructure
    bool fill_logstructure(struct LogStructure &logstruct, const uint8_t msg_type) const;

    // 解锁状态
    bool _armed;

    // 帮助我们不记录不必要的RCIN值的状态:
    bool should_log_rcin2;

    // 写入指南针实例数据
    void Write_Compass_instance(uint64_t time_us, uint8_t mag_instance);

    // 后端开始新日志
    void backend_starting_new_log(const AP_Logger_Backend *backend);

    // 单例指针
    static AP_Logger *_singleton;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    bool validate_structure(const struct LogStructure *logstructure, int16_t offset);
    void validate_structures(const struct LogStructure *logstructures, const uint8_t num_types);
    void dump_structure_field(const struct LogStructure *logstructure, const char *label, const uint8_t fieldnum);
    void dump_structures(const struct LogStructure *logstructures, const uint8_t num_types);
    bool assert_same_fmt_for_name(const log_write_fmt *f,
                                  const char *name,
                                  const char *labels,
                                  const char *units,
                                  const char *mults,
                                  const char *fmt) const;
    const char* unit_name(const uint8_t unit_id);
    double multiplier_name(const uint8_t multiplier_id);
    bool seen_ids[256] = { };
    bool labels_string_is_good(const char *labels) const;
#endif

    // 标志位
    bool _writes_enabled:1;
    bool _force_log_disarmed:1;
    bool _force_long_log_persist:1;

    // 日志写入格式字符串结构体
    struct log_write_fmt_strings {
        char name[LS_NAME_SIZE];
        char format[LS_FORMAT_SIZE];
        char labels[LS_LABELS_SIZE];
        char units[LS_UNITS_SIZE];
        char multipliers[LS_MULTIPLIERS_SIZE];
    };

    // 为回放记住格式
    void save_format_Replay(const void *pBuffer);

    // IO线程支持
    bool _io_thread_started;

    // 启动IO线程
    void start_io_thread(void);
    void io_thread();
    bool check_crash_dump_save(void);

#if HAL_LOGGER_FILE_CONTENTS_ENABLED
    // 支持记录文件内容
    struct file_list {
        struct file_list *next;
        const char *filename;
        char log_filename[16];
    };
    struct FileContent {
        void reset();
        void remove_and_free(file_list *victim);
        struct file_list *head, *tail;
        int fd{-1};
        uint32_t offset;
        bool fast;
        uint8_t counter;
        HAL_Semaphore sem;
    };
    FileContent normal_file_content;
    FileContent at_arm_file_content;

    // 用信号量保护?
    bool file_content_prepare_for_arming;

    // 更新文件内容
    void file_content_update(void);

    // 准备解锁时的系统文件日志记录
    void prepare_at_arming_sys_file_logging();

#endif
    
    /* 支持通过mavlink检索日志: */

    // 传输活动枚举
    enum class TransferActivity {
        IDLE,    // 空闲,所有文件描述符关闭
        LISTING, // 正在主动发送log_entry数据包
        SENDING, // 正在主动发送log_sending数据包
    } transfer_activity = TransferActivity::IDLE;

    // 上次处理日志传输mavlink消息的时间:
    uint32_t _last_mavlink_log_transfer_message_handled_ms;
    bool _warned_log_disarm; // 如果我们已发送解锁日志记录警告消息则为true

    // 下一个要发送的日志列表条目
    uint16_t _log_next_list_entry;

    // 最后要发送的日志列表条目
    uint16_t _log_last_list_entry;

    // 日志文件数量
    uint16_t _log_num_logs;

    // 数据发送的日志编号
    uint16_t _log_num_data;

    // 日志偏移量
    uint32_t _log_data_offset;

    // 日志文件大小
    uint32_t _log_data_size;

    // 剩余要发送的字节数
    uint32_t _log_data_remaining;

    // 日志数据起始页
    uint32_t _log_data_page;

    // 日志发送链路
    GCS_MAVLINK *_log_sending_link;
    HAL_Semaphore _log_send_sem;

    // 后端最后解锁失败时间
    uint32_t _last_arming_failure_ms;

    // 开始日志记录的次数计数
    // 可用于其他子系统检测是否应该记录数据
    uint8_t _log_start_count;

    // 处理日志消息
    void handle_log_message(class GCS_MAVLINK &, const mavlink_message_t &msg);

    // 处理各种日志请求
    void handle_log_request_list(class GCS_MAVLINK &, const mavlink_message_t &msg);
    void handle_log_request_data(class GCS_MAVLINK &, const mavlink_message_t &msg);
    void handle_log_request_erase(class GCS_MAVLINK &, const mavlink_message_t &msg);
    void handle_log_request_end(class GCS_MAVLINK &, const mavlink_message_t &msg);
    void end_log_transfer();
    void handle_log_send_listing(); // 处理LISTING状态
    void handle_log_sending(); // 处理SENDING状态
    bool handle_log_send_data(); // 向客户端发送数据块

    // 获取日志信息
    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc);

    // 获取日志数据
    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data);

    /* 结束通过mavlink检索日志的支持 */

#if HAL_LOGGER_FILE_CONTENTS_ENABLED
    // 记录文件内容到日志中
    // Log file contents to the logger
    void log_file_content(FileContent &file_content, const char *filename);
    // 更新文件内容记录状态
    // Update file content logging status
    void file_content_update(FileContent &file_content);
#endif
};

namespace AP {
    // 获取AP_Logger单例对象
    // Get AP_Logger singleton instance
    AP_Logger &logger();
};

// 记录子系统错误的宏定义
// Macro for logging subsystem errors
#define LOGGER_WRITE_ERROR(subsys, err) AP::logger().Write_Error(subsys, err)
// 记录事件的宏定义
// Macro for logging events
#define LOGGER_WRITE_EVENT(evt) AP::logger().Write_Event(evt)

#else

// 如果日志功能未启用,定义空的错误记录宏
// Define empty error logging macro if logging is disabled
#define LOGGER_WRITE_ERROR(subsys, err)
// 如果日志功能未启用,定义空的事件记录宏
// Define empty event logging macro if logging is disabled
#define LOGGER_WRITE_EVENT(evt)

#endif  // HAL_LOGGING_ENABLED
