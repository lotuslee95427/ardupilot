//
#pragma once

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//
// 警告 警告 警告 警告 警告 警告 警告 警告 警告
//
//  请不要直接编辑此文件来调整你的配置。创建你自己的
//  APM_Config.h 文件,并使用 APM_Config.h.example 作为参考。
//
// 警告 警告 警告 警告 警告 警告 警告 警告 警告
///
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//
// 默认和自动配置详情。
//
// 维护者注意事项:
//
// - 尽量保持此文件的组织顺序与 APM_Config.h.example 相同
//
#include "defines.h"

///
/// 请不要编辑这个包含 - 如果你想做本地更改,请在你本地的
/// APM_Config.h 副本中进行更改。
///
#include "APM_Config.h"
#include <AP_ADSB/AP_ADSB_config.h>
#include <AP_Follow/AP_Follow_config.h>
#include <AC_Avoidance/AC_Avoidance_config.h>

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// 硬件配置和连接
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifndef CONFIG_HAL_BOARD
#error 必须定义 CONFIG_HAL_BOARD 才能构建 ArduCopter
#endif

#ifndef ARMING_DELAY_SEC
    # define ARMING_DELAY_SEC 2.0f  // 解锁延迟秒数
#endif

//////////////////////////////////////////////////////////////////////////////
// FRAME_CONFIG
//
#ifndef FRAME_CONFIG
 # define FRAME_CONFIG   MULTICOPTER_FRAME  // 默认机架配置为多旋翼
#endif

/////////////////////////////////////////////////////////////////////////////////
// TradHeli 默认值
#if FRAME_CONFIG == HELI_FRAME
  # define RC_FAST_SPEED                        125  // 传统直升机的遥控快速速度
  # define WP_YAW_BEHAVIOR_DEFAULT              WP_YAW_BEHAVIOR_LOOK_AHEAD  // 传统直升机的默认航点偏航行为
#endif

//////////////////////////////////////////////////////////////////////////////
// PWM 控制
// 默认遥控速度(Hz)
#ifndef RC_FAST_SPEED
   #   define RC_FAST_SPEED 490
#endif

//////////////////////////////////////////////////////////////////////////////
// 测距仪
//

#ifndef RANGEFINDER_FILT_DEFAULT
 # define RANGEFINDER_FILT_DEFAULT 0.5f     // 测距仪距离的默认滤波值
#endif

#ifndef SURFACE_TRACKING_TIMEOUT_MS
 # define SURFACE_TRACKING_TIMEOUT_MS  1000 // 表面跟踪目标高度将在这么多毫秒后重置为当前测距仪高度(如果没有获得良好的测距仪高度)
#endif

#ifndef MAV_SYSTEM_ID
 # define MAV_SYSTEM_ID          1  // 默认 MAVLink 系统 ID
#endif

// 预解锁 GPS HDOP 检查
#ifndef GPS_HDOP_GOOD_DEFAULT
 # define GPS_HDOP_GOOD_DEFAULT         140     // 表示良好位置的最小 HDOP 值。在启用地理围栏时用于预解锁检查
#endif

// 缺失地形数据故障保护
#ifndef FS_TERRAIN_TIMEOUT_MS
 #define FS_TERRAIN_TIMEOUT_MS          5000     // 5秒的地形数据缺失将触发故障保护(RTL)
#endif

// 预解锁气压计与惯性导航最大高度差异
#ifndef PREARM_MAX_ALT_DISPARITY_CM
 # define PREARM_MAX_ALT_DISPARITY_CM       100     // 气压计和惯性导航高度必须在这么多厘米之内
#endif

//////////////////////////////////////////////////////////////////////////////
//  EKF 故障保护
#ifndef FS_EKF_ACTION_DEFAULT
 # define FS_EKF_ACTION_DEFAULT         FS_EKF_ACTION_LAND  // EKF 故障保护默认触发降落
#endif
#ifndef FS_EKF_THRESHOLD_DEFAULT
 # define FS_EKF_THRESHOLD_DEFAULT      0.8f    // EKF 故障保护的默认罗盘和速度方差阈值,超过此值将触发 EKF 故障保护
#endif

#ifndef EKF_ORIGIN_MAX_ALT_KM
 # define EKF_ORIGIN_MAX_ALT_KM         50   // EKF 原点和家必须在垂直方向上在 50km 之内
#endif

#ifndef FS_EKF_FILT_DEFAULT
# define FS_EKF_FILT_DEFAULT     5.0f    // EKF 方差滤波器的频率截止
#endif

//////////////////////////////////////////////////////////////////////////////
//  自动调参
#ifndef AUTOTUNE_ENABLED
 # define AUTOTUNE_ENABLED  1  // 启用自动调参功能
#endif

//////////////////////////////////////////////////////////////////////////////
// Nav-Guided - 允许外部导航计算机控制飞行器
#ifndef AC_NAV_GUIDED
 # define AC_NAV_GUIDED    1  // 启用导航引导功能
#endif

//////////////////////////////////////////////////////////////////////////////
// Acro - 特技模式,允许飞行器进行特技飞行
#ifndef MODE_ACRO_ENABLED
# define MODE_ACRO_ENABLED 1  // 启用特技模式
#endif

//////////////////////////////////////////////////////////////////////////////
// Auto 模式 - 允许飞行器追踪航点并执行自动动作
#ifndef MODE_AUTO_ENABLED
# define MODE_AUTO_ENABLED 1  // 启用自动模式
#endif

//////////////////////////////////////////////////////////////////////////////
// Brake 模式 - 使飞行器停止
#ifndef MODE_BRAKE_ENABLED
# define MODE_BRAKE_ENABLED 1  // 启用刹车模式
#endif

//////////////////////////////////////////////////////////////////////////////
// Circle - 使飞行器围绕中心点飞行
#ifndef MODE_CIRCLE_ENABLED
# define MODE_CIRCLE_ENABLED 1  // 启用环绕模式
#endif

//////////////////////////////////////////////////////////////////////////////
// Drift - 使飞行器在保持高度的同时进行协调转弯
#ifndef MODE_DRIFT_ENABLED
# define MODE_DRIFT_ENABLED 1  // 启用漂移模式
#endif

//////////////////////////////////////////////////////////////////////////////
// flip - 使飞行器在俯仰和横滚方向翻转
#ifndef MODE_FLIP_ENABLED
# define MODE_FLIP_ENABLED 1  // 启用翻转模式
#endif

//////////////////////////////////////////////////////////////////////////////
// Follow - 跟随另一个飞行器或地面站
#ifndef MODE_FOLLOW_ENABLED
#if AP_FOLLOW_ENABLED && AP_AVOIDANCE_ENABLED
#define MODE_FOLLOW_ENABLED 1  // 如果启用了跟随和避障功能,则启用跟随模式
#else
#define MODE_FOLLOW_ENABLED 0  // 否则禁用跟随模式
#endif
#endif

//////////////////////////////////////////////////////////////////////////////
// Guided 模式 - 从地面站控制飞行器的位置或角度
#ifndef MODE_GUIDED_ENABLED
# define MODE_GUIDED_ENABLED 1  // 启用引导模式
#endif

//////////////////////////////////////////////////////////////////////////////
// GuidedNoGPS 模式 - 从地面站控制飞行器的角度
#ifndef MODE_GUIDED_NOGPS_ENABLED
# define MODE_GUIDED_NOGPS_ENABLED 1  // 启用无 GPS 引导模式
#endif

//////////////////////////////////////////////////////////////////////////////
// Loiter 模式 - 允许飞行器保持全球位置
#ifndef MODE_LOITER_ENABLED
# define MODE_LOITER_ENABLED 1  // 启用悬停模式
#endif

//////////////////////////////////////////////////////////////////////////////
// Position Hold - 启用保持全球位置
#ifndef MODE_POSHOLD_ENABLED
# define MODE_POSHOLD_ENABLED 1  // 启用位置保持模式
#endif

//////////////////////////////////////////////////////////////////////////////
// RTL - 返航
#ifndef MODE_RTL_ENABLED
# define MODE_RTL_ENABLED 1  // 启用返航模式
#endif

//////////////////////////////////////////////////////////////////////////////
// SmartRTL - 允许飞行器沿(消除循环的)面包屑轨迹返回
#ifndef MODE_SMARTRTL_ENABLED
# define MODE_SMARTRTL_ENABLED 1  // 启用智能返航模式
#endif

//////////////////////////////////////////////////////////////////////////////
// Sport - 在速率控制(地球坐标系)模式下飞行
#ifndef MODE_SPORT_ENABLED
# define MODE_SPORT_ENABLED 0  // 禁用运动模式
#endif

//////////////////////////////////////////////////////////////////////////////
// System ID - 在飞行器上进行系统识别测试
#ifndef MODE_SYSTEMID_ENABLED
# define MODE_SYSTEMID_ENABLED HAL_LOGGING_ENABLED  // 如果启用了日志记录,则启用系统识别模式
#endif

//////////////////////////////////////////////////////////////////////////////
// Throw - 将飞行器抛到空中后飞行
#ifndef MODE_THROW_ENABLED
# define MODE_THROW_ENABLED 1  // 启用抛飞模式
#endif

//////////////////////////////////////////////////////////////////////////////
// ZigZag - 允许飞行器以预定义的 A B 点之间的之字形方式飞行
#ifndef MODE_ZIGZAG_ENABLED
# define MODE_ZIGZAG_ENABLED 1  // 启用之字形模式
#endif

//////////////////////////////////////////////////////////////////////////////
// Turtle - 允许在坠毁后翻转飞行器
#ifndef MODE_TURTLE_ENABLED
# define MODE_TURTLE_ENABLED HAL_DSHOT_ENABLED && FRAME_CONFIG != HELI_FRAME  // 如果启用了 DSHOT 且不是直升机机架,则启用龟壳模式
#endif

//////////////////////////////////////////////////////////////////////////////
// Flowhold - 使用光流传感器保持位置
#ifndef MODE_FLOWHOLD_ENABLED
# define MODE_FLOWHOLD_ENABLED AP_OPTICALFLOW_ENABLED  // 如果启用了光流,则启用光流保持模式
#endif

//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Weathervane - 允许飞行器偏航对准风向
#ifndef WEATHERVANE_ENABLED
# define WEATHERVANE_ENABLED 1  // 启用风向标功能
#endif

//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Autorotate - 自主自转 - 仅适用于直升机
#ifndef MODE_AUTOROTATE_ENABLED
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    #if FRAME_CONFIG == HELI_FRAME
        #ifndef MODE_AUTOROTATE_ENABLED
        # define MODE_AUTOROTATE_ENABLED 1  // 在 SITL 中为直升机启用自转模式
        #endif
    #else
        # define MODE_AUTOROTATE_ENABLED 0  // 在 SITL 中为非直升机禁用自转模式
    #endif
#else
    # define MODE_AUTOROTATE_ENABLED 0  // 在非 SITL 板上禁用自转模式
#endif
#endif

//////////////////////////////////////////////////////////////////////////////
// 遥控器配置
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
// FLIGHT_MODE
//

#ifndef FLIGHT_MODE_1
 # define FLIGHT_MODE_1                  Mode::Number::STABILIZE  // 默认飞行模式 1 为自稳模式
#endif
#ifndef FLIGHT_MODE_2
 # define FLIGHT_MODE_2                  Mode::Number::STABILIZE  // 默认飞行模式 2 为自稳模式
#endif
#ifndef FLIGHT_MODE_3
 # define FLIGHT_MODE_3                  Mode::Number::STABILIZE  // 默认飞行模式 3 为自稳模式
#endif
#ifndef FLIGHT_MODE_4
 # define FLIGHT_MODE_4                  Mode::Number::STABILIZE  // 默认飞行模式 4 为自稳模式
#endif
#ifndef FLIGHT_MODE_5
 # define FLIGHT_MODE_5                  Mode::Number::STABILIZE  // 默认飞行模式 5 为自稳模式
#endif
#ifndef FLIGHT_MODE_6
 # define FLIGHT_MODE_6                  Mode::Number::STABILIZE  // 默认飞行模式 6 为自稳模式
#endif

//////////////////////////////////////////////////////////////////////////////
// 油门失效保护
//
#ifndef FS_THR_VALUE_DEFAULT
 # define FS_THR_VALUE_DEFAULT             975    // 油门失效保护的默认PWM值
#endif

//////////////////////////////////////////////////////////////////////////////
// 起飞
//
#ifndef PILOT_TKOFF_ALT_DEFAULT
 # define PILOT_TKOFF_ALT_DEFAULT           0     // 飞行员发起起飞时,相对于home点的默认最终高度(单位:厘米)
#endif


//////////////////////////////////////////////////////////////////////////////
// 降落
//
#ifndef LAND_SPEED
 # define LAND_SPEED    50          // 降落最后阶段的下降速度(单位:厘米/秒)
#endif
#ifndef LAND_REPOSITION_DEFAULT
 # define LAND_REPOSITION_DEFAULT   1   // 默认允许飞行员在降落过程中控制横滚/俯仰
#endif
#ifndef LAND_WITH_DELAY_MS
 # define LAND_WITH_DELAY_MS        4000    // 失效保护事件触发延迟降落时的默认延迟时间(单位:毫秒)
#endif
#ifndef LAND_CANCEL_TRIGGER_THR
 # define LAND_CANCEL_TRIGGER_THR   700     // 当输入油门高于700时取消降落
#endif
#ifndef LAND_RANGEFINDER_MIN_ALT_CM
#define LAND_RANGEFINDER_MIN_ALT_CM 200     // 使用测距仪辅助降落的最小高度(单位:厘米)
#endif

//////////////////////////////////////////////////////////////////////////////
// 着陆检测器
//
#ifndef LAND_DETECTOR_TRIGGER_SEC
 # define LAND_DETECTOR_TRIGGER_SEC         1.0f    // 检测到着陆的秒数
#endif
#ifndef LAND_AIRMODE_DETECTOR_TRIGGER_SEC
 # define LAND_AIRMODE_DETECTOR_TRIGGER_SEC 3.0f    // 空中模式下检测到着陆的秒数
#endif
#ifndef LAND_DETECTOR_MAYBE_TRIGGER_SEC
 # define LAND_DETECTOR_MAYBE_TRIGGER_SEC   0.2f    // 可能已着陆的秒数(用于重置水平位置目标以防止倾翻)
#endif
#ifndef LAND_DETECTOR_ACCEL_LPF_CUTOFF
# define LAND_DETECTOR_ACCEL_LPF_CUTOFF     1.0f    // 着陆检测器加速度滤波器的截止频率
#endif
#ifndef LAND_DETECTOR_ACCEL_MAX
# define LAND_DETECTOR_ACCEL_MAX            1.0f    // 飞行器加速度必须小于1m/s/s
#endif
#ifndef LAND_DETECTOR_VEL_Z_MAX
# define LAND_DETECTOR_VEL_Z_MAX              1.0f    // 飞行器垂直速度必须小于1m/s
#endif

//////////////////////////////////////////////////////////////////////////////
// 飞行模式定义
//

// 特技模式(Acro Mode)
#ifndef ACRO_LEVEL_MAX_ANGLE
 # define ACRO_LEVEL_MAX_ANGLE      3000 // 训练模式下最大倾斜角度(单位:0.01度)
#endif

#ifndef ACRO_LEVEL_MAX_OVERSHOOT
 # define ACRO_LEVEL_MAX_OVERSHOOT  1000 // 训练模式下保持最大横滚或俯仰摇杆时的最大超调角度(单位:0.01度)
#endif

#ifndef ACRO_BALANCE_ROLL
 #define ACRO_BALANCE_ROLL          1.0f // 特技模式下横滚平衡因子
#endif

#ifndef ACRO_BALANCE_PITCH
 #define ACRO_BALANCE_PITCH         1.0f // 特技模式下俯仰平衡因子
#endif

#ifndef ACRO_RP_EXPO_DEFAULT
 #define ACRO_RP_EXPO_DEFAULT       0.3f    // 特技模式下横滚和俯仰指数参数默认值
#endif

#ifndef ACRO_Y_EXPO_DEFAULT
 #define ACRO_Y_EXPO_DEFAULT        0.0f    // 特技模式下偏航指数参数默认值
#endif

#ifndef ACRO_THR_MID_DEFAULT
 #define ACRO_THR_MID_DEFAULT       0.0f    // 特技模式下油门中点默认值
#endif

#ifndef ACRO_RP_RATE_DEFAULT
 #define ACRO_RP_RATE_DEFAULT      360      // 特技模式下横滚和俯仰旋转速率参数默认值(单位:度/秒)
#endif

#ifndef ACRO_Y_RATE_DEFAULT
 #define ACRO_Y_RATE_DEFAULT       202.5    // 特技模式下偏航旋转速率参数默认值(单位:度/秒)
#endif

// 返航模式(RTL Mode)
#ifndef RTL_ALT_FINAL
 # define RTL_ALT_FINAL             0       // 返航最后阶段的高度(单位:厘米),设为0表示降落
#endif

#ifndef RTL_ALT
 # define RTL_ALT                   1500    // 返航默认高度(单位:厘米),0表示保持当前高度
#endif

#ifndef RTL_ALT_MIN
 # define RTL_ALT_MIN               30     // 返航最小高度(单位:厘米)
#endif

#ifndef RTL_CLIMB_MIN_DEFAULT
 # define RTL_CLIMB_MIN_DEFAULT     0       // 返航第一阶段总是爬升的最小高度(单位:厘米)
#endif

#ifndef RTL_CONE_SLOPE_DEFAULT
 # define RTL_CONE_SLOPE_DEFAULT    3.0f    // 返航锥形斜率(高度/距离),0表示无锥形
#endif

#ifndef RTL_MIN_CONE_SLOPE
 # define RTL_MIN_CONE_SLOPE        0.5f    // 返航锥形最小斜率
#endif

#ifndef RTL_LOITER_TIME
 # define RTL_LOITER_TIME           5000    // 返航时在home点上方盘旋的时间(单位:毫秒)
#endif

// 自动模式(AUTO Mode)
#ifndef WP_YAW_BEHAVIOR_DEFAULT
 # define WP_YAW_BEHAVIOR_DEFAULT   WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL  // 默认航点偏航行为
#endif

#ifndef YAW_LOOK_AHEAD_MIN_SPEED
 # define YAW_LOOK_AHEAD_MIN_SPEED  100             // 飞行器朝向地面航向所需的最小地速(单位:厘米/秒)
#endif

// 超级简单模式(Super Simple mode)
#ifndef SUPER_SIMPLE_RADIUS
 # define SUPER_SIMPLE_RADIUS       1000    // 超级简单模式的半径(单位:厘米)
#endif

//////////////////////////////////////////////////////////////////////////////
// 稳定速率控制
//
#ifndef ROLL_PITCH_YAW_INPUT_MAX
 # define ROLL_PITCH_YAW_INPUT_MAX      4500        // 横滚、俯仰和偏航输入范围
#endif
#ifndef DEFAULT_ANGLE_MAX
 # define DEFAULT_ANGLE_MAX         3000            // ANGLE_MAX参数的默认值(单位:0.01度)
#endif

//////////////////////////////////////////////////////////////////////////////
// 停止模式默认值
//
#ifndef BRAKE_MODE_SPEED_Z
 # define BRAKE_MODE_SPEED_Z     250 // 刹车模式下Z轴速度(单位:厘米/秒)
#endif
#ifndef BRAKE_MODE_DECEL_RATE
 # define BRAKE_MODE_DECEL_RATE  750 // 刹车模式下减速率(单位:厘米/秒/秒)
#endif

//////////////////////////////////////////////////////////////////////////////
// 位置保持(PosHold)参数默认值
//
#ifndef POSHOLD_BRAKE_RATE_DEFAULT
 # define POSHOLD_BRAKE_RATE_DEFAULT    8       // 默认POSHOLD_BRAKE_RATE参数值,刹车过程中的旋转速率(单位:度/秒)
#endif
#ifndef POSHOLD_BRAKE_ANGLE_DEFAULT
 # define POSHOLD_BRAKE_ANGLE_DEFAULT   3000    // 默认POSHOLD_BRAKE_ANGLE参数值,刹车过程中的最大倾斜角度(单位:0.01度)
#endif

//////////////////////////////////////////////////////////////////////////////
// 飞行员控制默认值
//

#ifndef THR_DZ_DEFAULT
# define THR_DZ_DEFAULT         100             // 定高或悬停模式下,油门中点上下的死区
#endif

// 飞行员可请求的默认最大垂直速度和加速度
#ifndef PILOT_VELZ_MAX
 # define PILOT_VELZ_MAX    250     // 最大垂直速度(单位:厘米/秒)
#endif
#ifndef PILOT_ACCEL_Z_DEFAULT
 # define PILOT_ACCEL_Z_DEFAULT 250 // 高度由飞行员控制时的垂直加速度(单位:厘米/秒/秒)
#endif

#ifndef PILOT_Y_RATE_DEFAULT
 # define PILOT_Y_RATE_DEFAULT  202.5   // 除特技模式外所有模式的偏航旋转速率参数默认值(单位:度/秒)
#endif
#ifndef PILOT_Y_EXPO_DEFAULT
 # define PILOT_Y_EXPO_DEFAULT  0.0     // 除特技模式外所有模式的偏航指数参数默认值
#endif

#ifndef AUTO_DISARMING_DELAY
# define AUTO_DISARMING_DELAY  10       // 自动解锁延迟(单位:秒)
#endif

//////////////////////////////////////////////////////////////////////////////
// 抛飞模式配置
//
#ifndef THROW_HIGH_SPEED
# define THROW_HIGH_SPEED       500.0f  // 飞行器必须达到的总3D速度(单位:厘米/秒)(或自由落体)
#endif
#ifndef THROW_VERTICAL_SPEED
# define THROW_VERTICAL_SPEED   50.0f   // 电机启动时飞行器必须达到的总3D速度(单位:厘米/秒)
#endif

//////////////////////////////////////////////////////////////////////////////
// 日志控制
//

// 默认日志位掩码
#ifndef DEFAULT_LOG_BITMASK
 # define DEFAULT_LOG_BITMASK \
    MASK_LOG_ATTITUDE_MED | \
    MASK_LOG_GPS | \
    MASK_LOG_PM | \
    MASK_LOG_CTUN | \
    MASK_LOG_NTUN | \
    MASK_LOG_RCIN | \
    MASK_LOG_IMU | \
    MASK_LOG_CMD | \
    MASK_LOG_CURRENT | \
    MASK_LOG_RCOUT | \
    MASK_LOG_OPTFLOW | \
    MASK_LOG_COMPASS | \
    MASK_LOG_CAMERA | \
    MASK_LOG_MOTBATT
#endif

//////////////////////////////////////////////////////////////////////////////
// 围栏、集结点、地形和避障默认值
//

#if MODE_FOLLOW_ENABLED && !AP_AVOIDANCE_ENABLED
  #error 跟随模式依赖于AP_AVOIDANCE_ENABLED,但后者被禁用
#endif

#if MODE_AUTO_ENABLED && !MODE_GUIDED_ENABLED
  #error 自动模式需要引导模式,但后者被禁用
#endif

#if MODE_AUTO_ENABLED && !MODE_CIRCLE_ENABLED
  #error 自动模式需要环绕模式,但后者被禁用
#endif

#if MODE_AUTO_ENABLED && !MODE_RTL_ENABLED
  #error 自动模式需要返航模式,但后者被禁用
#endif

#if FRAME_CONFIG == HELI_FRAME && !MODE_ACRO_ENABLED
  #error 直升机机架需要特技模式支持,但后者被禁用
#endif

#if MODE_SMARTRTL_ENABLED && !MODE_RTL_ENABLED
  #error 智能返航需要返航模式,但后者被禁用
#endif

#if HAL_ADSB_ENABLED && !MODE_GUIDED_ENABLED
  #error ADSB需要引导模式,但后者被禁用
#endif

#if MODE_FOLLOW_ENABLED && !MODE_GUIDED_ENABLED
  #error 跟随模式需要引导模式,但后者被禁用
#endif

#if MODE_GUIDED_NOGPS_ENABLED && !MODE_GUIDED_ENABLED
  #error 无GPS引导模式需要引导模式,但后者被禁用
#endif

//////////////////////////////////////////////////////////////////////////////
// 开发者选项
//

#ifndef ADVANCED_FAILSAFE
# define ADVANCED_FAILSAFE 0         // 高级失效保护
#endif

#ifndef CH_MODE_DEFAULT
 # define CH_MODE_DEFAULT   5        // 默认模式切换通道
#endif

#ifndef TOY_MODE_ENABLED
#define TOY_MODE_ENABLED 0           // 玩具模式使能
#endif

#if TOY_MODE_ENABLED && FRAME_CONFIG == HELI_FRAME
  #error 玩具模式不适用于直升机
#endif

#ifndef HAL_FRAME_TYPE_DEFAULT
#define HAL_FRAME_TYPE_DEFAULT AP_Motors::MOTOR_FRAME_TYPE_X  // 默认机架类型为X型
#endif

#ifndef AC_CUSTOMCONTROL_MULTI_ENABLED
#define AC_CUSTOMCONTROL_MULTI_ENABLED FRAME_CONFIG == MULTICOPTER_FRAME && AP_CUSTOMCONTROL_ENABLED  // 多旋翼自定义控制使能
#endif

#ifndef AC_PAYLOAD_PLACE_ENABLED
#define AC_PAYLOAD_PLACE_ENABLED 1   // 负载放置功能使能
#endif

#ifndef USER_PARAMS_ENABLED
  #define USER_PARAMS_ENABLED 0      // 用户参数使能
#endif
