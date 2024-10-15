#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// 机架类型定义
#define UNDEFINED_FRAME 0  // 未定义机架
#define MULTICOPTER_FRAME 1  // 多旋翼机架
#define HELI_FRAME 2  // 直升机机架

// 调参枚举
enum tuning_func {
    TUNING_NONE =                        0, // 无调参
    TUNING_STABILIZE_ROLL_PITCH_KP =     1, // 稳定模式下横滚/俯仰角控制器的P参数
    TUNING_STABILIZE_YAW_KP =            3, // 稳定模式下偏航角控制器的P参数
    TUNING_RATE_ROLL_PITCH_KP =          4, // 机体坐标系下横滚/俯仰角速率控制器的P参数
    TUNING_RATE_ROLL_PITCH_KI =          5, // 机体坐标系下横滚/俯仰角速率控制器的I参数
    TUNING_YAW_RATE_KP =                 6, // 机体坐标系下偏航角速率控制器的P参数
    TUNING_THROTTLE_RATE_KP =            7, // 油门速率控制器的P参数(期望速率到加速度或电机输出)
    TUNING_WP_SPEED =                   10, // 航点间最大速度(0到10m/s)
    TUNING_LOITER_POSITION_KP =         12, // 悬停位置控制器的P参数(位置误差到速度)
    TUNING_HELI_EXTERNAL_GYRO =         13, // 传统直升机特有的外部尾桨陀螺仪增益
    TUNING_ALTITUDE_HOLD_KP =           14, // 定高控制器的P参数(高度误差到期望速率)
    TUNING_RATE_ROLL_PITCH_KD =         21, // 机体坐标系下横滚/俯仰角速率控制器的D参数
    TUNING_VEL_XY_KP =                  22, // 悬停速率控制器的P参数(速度误差到倾斜角度)
    TUNING_ACRO_RP_RATE =               25, // 特技模式下期望的横滚和俯仰角速率(度/秒)
    TUNING_YAW_RATE_KD =                26, // 机体坐标系下偏航角速率控制器的D参数
    TUNING_VEL_XY_KI =                  28, // 悬停速率控制器的I参数(速度误差到倾斜角度)
    TUNING_AHRS_YAW_KP =                30, // AHRS中罗盘对偏航角的影响(0 = 很低, 1 = 很高)
    TUNING_AHRS_KP =                    31, // 加速度计对横滚/俯仰角的影响(0=低)
    TUNING_ACCEL_Z_KP =                 34, // 基于加速度的油门控制器的P参数
    TUNING_ACCEL_Z_KI =                 35, // 基于加速度的油门控制器的I参数
    TUNING_ACCEL_Z_KD =                 36, // 基于加速度的油门控制器的D参数
    TUNING_DECLINATION =                38, // 罗盘磁偏角(弧度)
    TUNING_CIRCLE_RATE =                39, // 圆形轨迹转弯速率(度)(硬编码为约45度)
    TUNING_ACRO_YAW_RATE =              40, // 特技模式下期望的偏航角速率(度/秒)
    TUNING_RANGEFINDER_GAIN =           41, // 未使用
    TUNING_EKF_VERTICAL_POS =           42, // 未使用
    TUNING_EKF_HORIZONTAL_POS =         43, // 未使用
    TUNING_EKF_ACCEL_NOISE =            44, // 未使用
    TUNING_RC_FEEL_RP =                 45, // 横滚-俯仰输入平滑
    TUNING_RATE_PITCH_KP =              46, // 机体坐标系下俯仰角速率控制器的P参数
    TUNING_RATE_PITCH_KI =              47, // 机体坐标系下俯仰角速率控制器的I参数
    TUNING_RATE_PITCH_KD =              48, // 机体坐标系下俯仰角速率控制器的D参数
    TUNING_RATE_ROLL_KP =               49, // 机体坐标系下横滚角速率控制器的P参数
    TUNING_RATE_ROLL_KI =               50, // 机体坐标系下横滚角速率控制器的I参数
    TUNING_RATE_ROLL_KD =               51, // 机体坐标系下横滚角速率控制器的D参数
    TUNING_RATE_PITCH_FF =              52, // 机体坐标系下俯仰角速率控制器的前馈项
    TUNING_RATE_ROLL_FF =               53, // 机体坐标系下横滚角速率控制器的前馈项
    TUNING_RATE_YAW_FF =                54, // 机体坐标系下偏航角速率控制器的前馈项
    TUNING_RATE_MOT_YAW_HEADROOM =      55, // 电机偏航余量最小值
    TUNING_RATE_YAW_FILT =              56, // 偏航角速率输入滤波
    UNUSED =                            57, // 曾用于绞车控制,现已弃用
    TUNING_SYSTEM_ID_MAGNITUDE =        58, // 系统识别信号幅值
    TUNING_POS_CONTROL_ANGLE_MAX =      59  // 位置控制器最大角度
};

// 任务中的偏航行为 - WP_YAW_BEHAVIOR参数的可能值
#define WP_YAW_BEHAVIOR_NONE                          0   // 自动驾驶仪在任务或返航过程中永不控制偏航(除非收到DO_CONDITIONAL_YAW命令)
#define WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP               1   // 自动驾驶仪在任务中面向下一个航点,返航时面向家
#define WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL    2   // 自动驾驶仪在任务中面向下一个航点,但返航时保持最后的偏航角
#define WP_YAW_BEHAVIOR_LOOK_AHEAD                    3   // 自动驾驶仪在任务和返航过程中向前看(主要用于传统直升机)

// 空中模式
enum class AirMode {
    AIRMODE_NONE,       // 无空中模式
    AIRMODE_DISABLED,   // 空中模式禁用
    AIRMODE_ENABLED,    // 空中模式启用
};

// DEV_OPTIONS参数的位选项
enum DevOptions {
    DevOptionADSBMAVLink = 1,            // ADSB MAVLink选项
    DevOptionVFR_HUDRelativeAlt = 2,     // VFR HUD相对高度选项
};

// 日志记录参数 - 飞行器这里只有32个消息可用
enum LoggingParameters {
     LOG_CONTROL_TUNING_MSG,             // 控制调优消息
     LOG_DATA_INT16_MSG,                 // 16位整数数据消息
     LOG_DATA_UINT16_MSG,                // 16位无符号整数数据消息
     LOG_DATA_INT32_MSG,                 // 32位整数数据消息
     LOG_DATA_UINT32_MSG,                // 32位无符号整数数据消息
     LOG_DATA_FLOAT_MSG,                 // 浮点数数据消息
     LOG_PARAMTUNE_MSG,                  // 参数调优消息
     LOG_HELI_MSG,                       // 直升机消息
     LOG_GUIDED_POSITION_TARGET_MSG,     // 引导模式位置目标消息
     LOG_SYSIDD_MSG,                     // 系统识别D消息
     LOG_SYSIDS_MSG,                     // 系统识别S消息
     LOG_GUIDED_ATTITUDE_TARGET_MSG      // 引导模式姿态目标消息
};

// 日志记录掩码定义
#define MASK_LOG_ATTITUDE_FAST          (1<<0)   // 快速姿态日志
#define MASK_LOG_ATTITUDE_MED           (1<<1)   // 中速姿态日志
#define MASK_LOG_GPS                    (1<<2)   // GPS日志
#define MASK_LOG_PM                     (1<<3)   // 电源管理日志
#define MASK_LOG_CTUN                   (1<<4)   // 控制调优日志
#define MASK_LOG_NTUN                   (1<<5)   // 导航调优日志
#define MASK_LOG_RCIN                   (1<<6)   // 遥控输入日志
#define MASK_LOG_IMU                    (1<<7)   // IMU日志
#define MASK_LOG_CMD                    (1<<8)   // 命令日志
#define MASK_LOG_CURRENT                (1<<9)   // 电流日志
#define MASK_LOG_RCOUT                  (1<<10)  // 遥控输出日志
#define MASK_LOG_OPTFLOW                (1<<11)  // 光流日志
#define MASK_LOG_PID                    (1<<12)  // PID日志
#define MASK_LOG_COMPASS                (1<<13)  // 罗盘日志
#define MASK_LOG_INAV                   (1<<14)  // 惯性导航日志(已弃用)
#define MASK_LOG_CAMERA                 (1<<15)  // 相机日志
#define MASK_LOG_MOTBATT                (1UL<<17) // 电机电池日志
#define MASK_LOG_IMU_FAST               (1UL<<18) // 快速IMU日志
#define MASK_LOG_IMU_RAW                (1UL<<19) // 原始IMU日志
#define MASK_LOG_VIDEO_STABILISATION    (1UL<<20) // 视频稳定日志
#define MASK_LOG_FTN_FAST               (1UL<<21) // 快速FTN日志
#define MASK_LOG_ANY                    0xFFFF    // 任意日志

// 遥控器失效保护定义(FS_THR参数)
#define FS_THR_DISABLED                            0  // 禁用
#define FS_THR_ENABLED_ALWAYS_RTL                  1  // 启用,总是返航
#define FS_THR_ENABLED_CONTINUE_MISSION            2  // 4.0+版本已移除,现使用fs_options
#define FS_THR_ENABLED_ALWAYS_LAND                 3  // 启用,总是降落
#define FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_RTL      4  // 启用,总是智能返航或普通返航
#define FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_LAND     5  // 启用,总是智能返航或降落
#define FS_THR_ENABLED_AUTO_RTL_OR_RTL             6  // 启用,自动返航或普通返航
#define FS_THR_ENABLED_BRAKE_OR_LAND               7  // 启用,刹车或降落

// 地面站失效保护定义(FS_GCS_ENABLE参数)
#define FS_GCS_DISABLED                        0  // 禁用
#define FS_GCS_ENABLED_ALWAYS_RTL              1  // 启用,总是返航
#define FS_GCS_ENABLED_CONTINUE_MISSION        2  // 4.0+版本已移除,现使用fs_options
#define FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_RTL  3  // 启用,总是智能返航或普通返航
#define FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_LAND 4  // 启用,总是智能返航或降落
#define FS_GCS_ENABLED_ALWAYS_LAND             5  // 启用,总是降落
#define FS_GCS_ENABLED_AUTO_RTL_OR_RTL         6  // 启用,自动返航或普通返航
#define FS_GCS_ENABLED_BRAKE_OR_LAND           7  // 启用,刹车或降落

// EKF失效保护定义(FS_EKF_ACTION参数)
#define FS_EKF_ACTION_LAND                  1       // EKF失效时切换到降落模式
#define FS_EKF_ACTION_ALTHOLD               2       // EKF失效时切换到定高模式
#define FS_EKF_ACTION_LAND_EVEN_STABILIZE   3       // EKF失效时切换到降落模式,即使在手动飞行模式如自稳模式下也是如此

// PILOT_THR_BHV参数的位定义
#define THR_BEHAVE_FEEDBACK_FROM_MID_STICK (1<<0)   // 从中位油门获取反馈
#define THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND (1<<1) // 高油门取消降落
#define THR_BEHAVE_DISARM_ON_LAND_DETECT (1<<2)     // 检测到着陆时自动解锁
