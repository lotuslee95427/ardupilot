#pragma once

// 内部定义，请勿编辑，否则可能会影响功能
// -------------------------------------------------------

#define SERVO_MAX 4500.0  // 此值代表45度，是舵机最大行程的任意表示

#define MIN_AIRSPEED_MIN 5 // 最小空速，单位m/s，用于解锁检查和速度缩放

#define TAKEOFF_RUDDER_WARNING_TIMEOUT 3000 // GCS警告未将解锁方向舵回中的重复时间，单位ms

// 故障保护
// ----------------------
enum failsafe_state {
    FAILSAFE_NONE=0,   // 无故障保护
    FAILSAFE_SHORT=1,  // 短期故障保护
    FAILSAFE_LONG=2,   // 长期故障保护
    FAILSAFE_GCS=3     // 地面站故障保护
};

// GCS故障保护类型，用于FS_GCS_ENABL参数
enum gcs_failsafe {
    GCS_FAILSAFE_OFF        = 0, // 无GCS故障保护
    GCS_FAILSAFE_HEARTBEAT  = 1, // 停止接收心跳时触发故障保护
    GCS_FAILSAFE_HB_RSSI    = 2, // 停止接收心跳或RADIO.remrssi降至0时触发故障保护
    GCS_FAILSAFE_HB_AUTO    = 3  // 在AUTO模式下停止接收心跳时触发故障保护
};

enum failsafe_action_short {
    FS_ACTION_SHORT_BESTGUESS = 0,      // CIRCLE模式/不改变(如果已经在AUTO|GUIDED|LOITER模式)
    FS_ACTION_SHORT_CIRCLE = 1,         // 圆周飞行
    FS_ACTION_SHORT_FBWA = 2,           // 切换到FBWA模式
    FS_ACTION_SHORT_DISABLED = 3,       // 禁用
    FS_ACTION_SHORT_FBWB = 4,           // 切换到FBWB模式
};

enum failsafe_action_long {
    FS_ACTION_LONG_CONTINUE = 0,        // 继续当前模式
    FS_ACTION_LONG_RTL = 1,             // 返航
    FS_ACTION_LONG_GLIDE = 2,           // 滑翔
    FS_ACTION_LONG_PARACHUTE = 3,       // 释放降落伞
    FS_ACTION_LONG_AUTO = 4,            // 切换到AUTO模式
};

// 启用的摇杆混控类型
enum class StickMixing {
    NONE     = 0,                       // 无混控
    FBW      = 1,                       // 飞行控制线混控
    DIRECT_REMOVED = 2,                 // 直接控制（已移除）
    VTOL_YAW = 3,                       // 垂直起降偏航混控
};

// RTL_AUTOLAND的值
enum class RtlAutoland {
    RTL_DISABLE = 0,                    // 禁用RTL自动着陆
    RTL_THEN_DO_LAND_START = 1,         // RTL后开始着陆
    RTL_IMMEDIATE_DO_LAND_START = 2,    // 立即开始RTL和着陆
    NO_RTL_GO_AROUND = 3,               // 无RTL复飞
};

enum ChannelMixing {
    MIXING_DISABLED = 0,                // 禁用混控
    MIXING_UPUP     = 1,                // 上-上混控
    MIXING_UPDN     = 2,                // 上-下混控
    MIXING_DNUP     = 3,                // 下-上混控
    MIXING_DNDN     = 4,                // 下-下混控
    MIXING_UPUP_SWP = 5,                // 上-上混控（交换）
    MIXING_UPDN_SWP = 6,                // 上-下混控（交换）
    MIXING_DNUP_SWP = 7,                // 下-上混控（交换）
    MIXING_DNDN_SWP = 8,                // 下-下混控（交换）
};

// PID广播位掩码
enum tuning_pid_bits {
    TUNING_BITS_ROLL  = (1 <<  0),      // 横滚PID调谐
    TUNING_BITS_PITCH = (1 <<  1),      // 俯仰PID调谐
    TUNING_BITS_YAW   = (1 <<  2),      // 偏航PID调谐
    TUNING_BITS_STEER = (1 <<  3),      // 转向PID调谐
    TUNING_BITS_LAND  = (1 <<  4),      // 着陆PID调谐
    TUNING_BITS_ACCZ  = (1 <<  5),      // 垂直加速度PID调谐
    TUNING_BITS_END                     // 哑元，用于静态检查
};

static_assert(TUNING_BITS_END <= (1 << 24) + 1, "调谐位掩码过大，无法通过MAVLink设置");

// 日志消息类型 - 飞行器仅有32种可用消息
enum log_messages {
    LOG_CTUN_MSG,                       // 控制调谐日志
    LOG_NTUN_MSG,                       // 导航调谐日志
    LOG_STATUS_MSG,                     // 状态日志
    LOG_QTUN_MSG,                       // 四旋翼调谐日志
    LOG_PIQR_MSG,                       // 横滚PI控制器日志
    LOG_PIQP_MSG,                       // 俯仰PI控制器日志
    LOG_PIQY_MSG,                       // 偏航PI控制器日志
    LOG_PIQA_MSG,                       // 高度PI控制器日志
    LOG_PIDG_MSG,                       // PID增益日志
    LOG_AETR_MSG,                       // 副翼、升降舵、油门、方向舵日志
    LOG_OFG_MSG,                        // 光流日志
    LOG_TSIT_MSG,                       // 倾转座椅日志
    LOG_TILT_MSG,                       // 倾斜日志
};

#define MASK_LOG_ATTITUDE_FAST          (1<<0)  // 快速姿态日志掩码
#define MASK_LOG_ATTITUDE_MED           (1<<1)  // 中速姿态日志掩码
#define MASK_LOG_GPS                    (1<<2)  // GPS日志掩码
#define MASK_LOG_PM                     (1<<3)  // 电源管理日志掩码
#define MASK_LOG_CTUN                   (1<<4)  // 控制调谐日志掩码
#define MASK_LOG_NTUN                   (1<<5)  // 导航调谐日志掩码
//#define MASK_LOG_MODE                 (1<<6)  // 不再使用
#define MASK_LOG_IMU                    (1<<7)  // IMU日志掩码
#define MASK_LOG_CMD                    (1<<8)  // 命令日志掩码
#define MASK_LOG_CURRENT                (1<<9)  // 电流日志掩码
#define MASK_LOG_COMPASS                (1<<10) // 罗盘日志掩码
#define MASK_LOG_TECS                   (1<<11) // TECS日志掩码
#define MASK_LOG_CAMERA                 (1<<12) // 相机日志掩码
#define MASK_LOG_RC                     (1<<13) // 遥控器日志掩码
#define MASK_LOG_SONAR                  (1<<14) // 声呐日志掩码
// #define MASK_LOG_ARM_DISARM             (1<<15) // 解锁/上锁日志掩码
#define MASK_LOG_IMU_RAW                (1UL<<19) // 原始IMU数据日志掩码
#define MASK_LOG_ATTITUDE_FULLRATE      (1U<<20)  // 全速率姿态日志掩码
#define MASK_LOG_VIDEO_STABILISATION    (1UL<<21) // 视频稳定日志掩码
#define MASK_LOG_NOTCH_FULLRATE         (1UL<<22) // 全速率陷波器日志掩码

enum {
    CRASH_DETECT_ACTION_BITMASK_DISABLED = 0,     // 禁用碰撞检测动作
    CRASH_DETECT_ACTION_BITMASK_DISARM = (1<<0),  // 碰撞检测时解除武装
    // 注意：下一个枚举将是(1<<1)，然后是(1<<2)，(1<<3)
};

enum {
    USE_REVERSE_THRUST_NEVER                    = 0,     // 从不使用反向推力
    USE_REVERSE_THRUST_AUTO_ALWAYS              = (1<<0),  // 自动模式下始终使用反向推力
    USE_REVERSE_THRUST_AUTO_LAND_APPROACH       = (1<<1),  // 自动着陆进场时使用反向推力
    USE_REVERSE_THRUST_AUTO_LOITER_TO_ALT       = (1<<2),  // 自动模式下盘旋至高度时使用反向推力
    USE_REVERSE_THRUST_AUTO_LOITER_ALL          = (1<<3),  // 自动模式下所有盘旋时使用反向推力
    USE_REVERSE_THRUST_AUTO_WAYPOINT            = (1<<4),  // 自动航点模式下使用反向推力
    USE_REVERSE_THRUST_LOITER                   = (1<<5),  // 盘旋模式下使用反向推力
    USE_REVERSE_THRUST_RTL                      = (1<<6),  // 返航模式下使用反向推力
    USE_REVERSE_THRUST_CIRCLE                   = (1<<7),  // 圆周飞行模式下使用反向推力
    USE_REVERSE_THRUST_CRUISE                   = (1<<8),  // 巡航模式下使用反向推力
    USE_REVERSE_THRUST_FBWB                     = (1<<9),  // FBWB模式下使用反向推力
    USE_REVERSE_THRUST_GUIDED                   = (1<<10), // 引导模式下使用反向推力
    USE_REVERSE_THRUST_AUTO_LANDING_PATTERN     = (1<<11), // 自动着陆模式下使用反向推力
    USE_REVERSE_THRUST_FBWA                   = (1<<12),   // FBWA模式下使用反向推力
    USE_REVERSE_THRUST_ACRO                   = (1<<13),   // 特技模式下使用反向推力
    USE_REVERSE_THRUST_STABILIZE            = (1<<14),     // 自稳模式下使用反向推力
    USE_REVERSE_THRUST_THERMAL             = (1<<15),      // 热气流模式下使用反向推力
};

enum FlightOptions {
    DIRECT_RUDDER_ONLY   = (1 << 0),               // 仅使用直接方向舵控制
    CRUISE_TRIM_THROTTLE = (1 << 1),               // 巡航时调整油门
    DISABLE_TOFF_ATTITUDE_CHK = (1 << 2),          // 禁用起飞姿态检查
    CRUISE_TRIM_AIRSPEED = (1 << 3),               // 巡航时调整空速
    CLIMB_BEFORE_TURN = (1 << 4),                  // 转弯前爬升
    ACRO_YAW_DAMPER = (1 << 5),                    // 特技模式下使用偏航阻尼
    SURPRESS_TKOFF_SCALING = (1<<6),               // 抑制起飞缩放
    ENABLE_DEFAULT_AIRSPEED = (1<<7),              // 启用默认空速
    GCS_REMOVE_TRIM_PITCH = (1 << 8),              // GCS移除俯仰微调
    OSD_REMOVE_TRIM_PITCH = (1 << 9),              // OSD移除俯仰微调
    CENTER_THROTTLE_TRIM = (1<<10),                // 居中油门微调
    DISABLE_GROUND_PID_SUPPRESSION = (1<<11),      // 禁用地面PID抑制
    ENABLE_LOITER_ALT_CONTROL = (1<<12),           // 启用盘旋高度控制
    INDICATE_WAITING_FOR_RUDDER_NEUTRAL = (1<<13), // 指示等待方向舵回中
    IMMEDIATE_CLIMB_IN_AUTO = (1<<14),             // 自动模式下立即爬升
};

enum CrowFlapOptions {
    FLYINGWING       = (1 << 0),                   // 飞翼
    FULLSPAN         = (1 << 1),                   // 全展
    PROGRESSIVE_CROW = (1 << 2),                   // 渐进式乌鸦襟翼
}; 

enum guided_heading_type_t {
    GUIDED_HEADING_NONE = 0, // 无航向跟踪
    GUIDED_HEADING_COG,      // 保持地面航迹
    GUIDED_HEADING_HEADING,  // 保持航向
};

enum class AirMode {
    OFF,                    // 关闭
    ON,                     // 开启
    ASSISTED_FLIGHT_ONLY,   // 仅在辅助飞行时开启
};

enum class FenceAutoEnable : uint8_t {
    OFF=0,                  // 关闭
    Auto=1,                 // 自动
    AutoDisableFloorOnly=2, // 自动（仅禁用地板）
    WhenArmed=3             // 解锁时启用
};
