/*
   本程序是自由软件:你可以在遵守由自由软件基金会发布的GNU通用公共许可证(版本3或更新版本)条款的前提下对其进行再分发和/或修改。

   本程序的发布是希望它能够有用,但不负任何担保责任;甚至没有适销性或特定用途适用性的暗示担保。详情请参阅GNU通用公共许可证。

   你应该已经收到一份GNU通用公共许可证的副本。如果没有,请查看<http://www.gnu.org/licenses/>。
 */
#pragma once
/*
  这是Copter的主类
 */

////////////////////////////////////////////////////////////////////////////////
// 头文件包含
////////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include <stdio.h>
#include <stdarg.h>

#include <AP_HAL/AP_HAL.h>

// 通用依赖
#include <AP_Common/AP_Common.h>            // ArduPilot库的通用定义和实用程序
#include <AP_Common/Location.h>             // 实现位置类的库         
#include <AP_Param/AP_Param.h>              // 用于管理和存储对系统普遍感兴趣的变量的系统
#include <StorageManager/StorageManager.h>  // 用于管理hal.storage的库,允许向后兼容的存储偏移量映射到可用存储

// 应用依赖
#include <AP_Logger/AP_Logger.h>            // ArduPilot Mega闪存存储库
#include <AP_Math/AP_Math.h>                // ArduPilot Mega向量/矩阵数学库
#include <AP_AccelCal/AP_AccelCal.h>        // 加速度计校准的接口和数学
#include <AP_InertialSensor/AP_InertialSensor.h>                // ArduPilot Mega惯性传感器(加速度计和陀螺仪)库
#include <AP_AHRS/AP_AHRS.h>                                    // ArduPilot的AHRS(姿态航向参考系统)接口库
#include <AP_Mission/AP_Mission.h>                              // 任务命令库
#include <AP_Mission/AP_Mission_ChangeDetector.h>               // 任务命令变更检测库
#include <AC_AttitudeControl/AC_AttitudeControl_Multi.h>        // 多旋翼姿态控制库
#include <AC_AttitudeControl/AC_AttitudeControl_Multi_6DoF.h>   // 6自由度姿态控制库
#include <AC_AttitudeControl/AC_AttitudeControl_Heli.h>         // 传统直升机姿态控制库
#include <AC_AttitudeControl/AC_PosControl.h>                   // 位置控制库
#include <AC_AttitudeControl/AC_CommandModel.h>                 // 命令模型库
#include <AP_Motors/AP_Motors.h>            // AP电机库
#include <Filter/Filter.h>                  // 滤波器库
#include <AP_Vehicle/AP_Vehicle.h>          // AHRS构建所需
#include <AP_InertialNav/AP_InertialNav.h>  // 惯性导航库
#include <AC_WPNav/AC_WPNav.h>              // ArduCopter航点导航库
#include <AC_WPNav/AC_Loiter.h>             // ArduCopter盘旋模式库
#include <AC_WPNav/AC_Circle.h>             // 圆形导航库
#include <AP_Declination/AP_Declination.h>  // ArduPilot Mega磁偏角辅助库
#include <AP_RCMapper/AP_RCMapper.h>        // 遥控输入映射库
#include <AP_BattMonitor/AP_BattMonitor.h>  // 电池监控库
#include <AP_LandingGear/AP_LandingGear.h>  // 起落架库
#include <AC_InputManager/AC_InputManager.h>        // 飞行员输入处理库
#include <AC_InputManager/AC_InputManager_Heli.h>   // 直升机特定飞行员输入处理库
#include <AP_Arming/AP_Arming.h>            // ArduPilot电机解锁库
#include <AP_SmartRTL/AP_SmartRTL.h>        // ArduPilot智能返航模式(SRTL)库
#include <AP_TempCalibration/AP_TempCalibration.h>  // 温度校准库
#include <AC_AutoTune/AC_AutoTune_Multi.h>  // ArduCopter自动调谐库。支持多旋翼的自动调谐。
#include <AC_AutoTune/AC_AutoTune_Heli.h>   // ArduCopter自动调谐库。支持直升机的自动调谐。
#include <AP_Parachute/AP_Parachute.h>      // ArduPilot降落伞释放库
#include <AC_Sprayer/AC_Sprayer.h>          // 农药喷洒库
#include <AP_ADSB/AP_ADSB.h>                // 基于ADS-B RF的碰撞避免模块库
#include <AP_Proximity/AP_Proximity.h>      // ArduPilot接近传感器库
#include <AC_PrecLand/AC_PrecLand_config.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>
#include <AP_Winch/AP_Winch_config.h>
#include <AP_SurfaceDistance/AP_SurfaceDistance.h>

// 配置
#include "defines.h"
#include "config.h"

#if FRAME_CONFIG == HELI_FRAME
 #define MOTOR_CLASS AP_MotorsHeli
#else
 #define MOTOR_CLASS AP_MotorsMulticopter
#endif

#if MODE_AUTOROTATE_ENABLED
 #include <AC_Autorotation/AC_Autorotation.h> // 自转控制器
#endif

#include "RC_Channel.h"         // 遥控通道库

#include "GCS_Mavlink.h"
#include "GCS_Copter.h"
#include "AP_Rally.h"           // 集结点库
#include "AP_Arming.h"

#include <AP_ExternalControl/AP_ExternalControl_config.h>
#if AP_EXTERNAL_CONTROL_ENABLED
#include "AP_ExternalControl_Copter.h"
#endif

#include <AP_Beacon/AP_Beacon_config.h>
#if AP_BEACON_ENABLED
 #include <AP_Beacon/AP_Beacon.h>
#endif

#if AP_AVOIDANCE_ENABLED
 #include <AC_Avoidance/AC_Avoid.h>
#endif
#if AP_OAPATHPLANNER_ENABLED
 #include <AC_WPNav/AC_WPNav_OA.h>
 #include <AC_Avoidance/AP_OAPathPlanner.h>
#endif
#if AC_PRECLAND_ENABLED
 # include <AC_PrecLand/AC_PrecLand.h>
 # include <AC_PrecLand/AC_PrecLand_StateMachine.h>
#endif
#if MODE_FOLLOW_ENABLED
 # include <AP_Follow/AP_Follow.h>
#endif
#if AP_TERRAIN_AVAILABLE
 # include <AP_Terrain/AP_Terrain.h>
#endif
#if AP_RANGEFINDER_ENABLED
 # include <AP_RangeFinder/AP_RangeFinder.h>
#endif

#include <AP_Mount/AP_Mount.h>

#include <AP_Camera/AP_Camera.h>

#if HAL_BUTTON_ENABLED
 # include <AP_Button/AP_Button.h>
#endif

#if OSD_ENABLED || OSD_PARAM_ENABLED
 #include <AP_OSD/AP_OSD.h>
#endif

#if ADVANCED_FAILSAFE
 # include "afs_copter.h"
#endif
#if TOY_MODE_ENABLED
 # include "toy_mode.h"
#endif
#if AP_WINCH_ENABLED
 # include <AP_Winch/AP_Winch.h>
#endif
#include <AP_RPM/AP_RPM.h>

#if AP_SCRIPTING_ENABLED
#include <AP_Scripting/AP_Scripting.h>
#endif

#if AC_CUSTOMCONTROL_MULTI_ENABLED
#include <AC_CustomControl/AC_CustomControl.h>                  // 自定义控制库
#endif

#if AP_AVOIDANCE_ENABLED && !AP_FENCE_ENABLED
  #error AC_Avoidance依赖于AP_FENCE_ENABLED,但它被禁用了
#endif

#if AP_OAPATHPLANNER_ENABLED && !AP_FENCE_ENABLED
  #error AP_OAPathPlanner依赖于AP_FENCE_ENABLED,但它被禁用了
#endif

#if HAL_ADSB_ENABLED
#include "avoidance_adsb.h"
#endif
// 本地模块
#include "Parameters.h"
#if USER_PARAMS_ENABLED
#include "UserParameters.h"
#endif
#include "mode.h"

class Copter : public AP_Vehicle {
public:
    friend class GCS_MAVLINK_Copter;
    friend class GCS_Copter;
    friend class AP_Rally_Copter;
    friend class Parameters;
    friend class ParametersG2;
    friend class AP_Avoidance_Copter;

#if ADVANCED_FAILSAFE
    friend class AP_AdvancedFailsafe_Copter;
#endif
    friend class AP_Arming_Copter;
#if AP_EXTERNAL_CONTROL_ENABLED
    friend class AP_ExternalControl_Copter;
#endif
    friend class ToyMode;
    friend class RC_Channel_Copter;
    friend class RC_Channels_Copter;

    friend class AutoTune;

    friend class Mode;
    friend class ModeAcro;
    friend class ModeAcro_Heli;
    friend class ModeAltHold;
    friend class ModeAuto;
    friend class ModeAutoTune;
    friend class ModeAvoidADSB;
    friend class ModeBrake;
    friend class ModeCircle;
    friend class ModeDrift;
    friend class ModeFlip;
    friend class ModeFlowHold;
    friend class ModeFollow;
    friend class ModeGuided;
    friend class ModeLand;
    friend class ModeLoiter;
    friend class ModePosHold;
    friend class ModeRTL;
    friend class ModeSmartRTL;
    friend class ModeSport;
    friend class ModeStabilize;
    friend class ModeStabilize_Heli;
    friend class ModeSystemId;
    friend class ModeThrow;
    friend class ModeZigZag;
    friend class ModeAutorotate;
    friend class ModeTurtle;

    friend class _AutoTakeoff;

    friend class PayloadPlace;

    Copter(void);

private:

    // 传递给多个库的关键飞行器参数
    AP_MultiCopter aparm;

    // 全局参数都包含在'g'类中
    Parameters g;
    ParametersG2 g2;

    // 用于检测来自GCS的MAVLink确认以停止罗盘校准
    uint8_t command_ack_counter;

    // 主要输入控制通道
    RC_Channel *channel_roll;
    RC_Channel *channel_pitch;
    RC_Channel *channel_throttle;
    RC_Channel *channel_yaw;

    // 飞行模式便利数组
    AP_Int8 *flight_modes;
    const uint8_t num_flight_modes = 6;

    AP_SurfaceDistance rangefinder_state {ROTATION_PITCH_270, inertial_nav, 0U};
    AP_SurfaceDistance rangefinder_up_state {ROTATION_PITCH_90, inertial_nav, 1U};

    // 辅助函数,用于获取惯性插值的测距仪高度
    bool get_rangefinder_height_interpolated_cm(int32_t& ret) const;

#if AP_RANGEFINDER_ENABLED
    class SurfaceTracking {
    public:

        // update_surface_offset - 管理位置控制器的垂直偏移,以跟随使用测距仪测量的地面或天花板水平
        void update_surface_offset();

        // 获取/设置地面以上的目标高度(以厘米为单位)
        bool get_target_alt_cm(float &target_alt_cm) const;
        void set_target_alt_cm(float target_alt_cm);

        // 获取目标和实际距离(以米为单位)用于日志记录
        bool get_target_dist_for_logging(float &target_dist) const;
        float get_dist_for_logging() const;
        void invalidate_for_logging() { valid_for_logging = false; }

        // 表面跟踪表面
        enum class Surface {
            NONE = 0,
            GROUND = 1,
            CEILING = 2
        };
        // 设置要跟踪的表面
        void set_surface(Surface new_surface);
        // 初始化表面跟踪
        void init(Surface surf) { surface = surf; }

    private:
        Surface surface;                // 当前跟踪的表面类型
        uint32_t last_update_ms;        // 上次更新目标高度的系统时间(毫秒)
        uint32_t last_glitch_cleared_ms;// 上次处理故障恢复的系统时间(毫秒)
        bool valid_for_logging;         // 如果我们有有效的目标高度,则为true
        bool reset_target;              // 如果由于跟踪表面变化需要重置目标,则为true

    } surface_tracking;
#endif

#if AP_RPM_ENABLED
    AP_RPM rpm_sensor;                  // RPM传感器对象
#endif

    // 惯性导航EKF - 不同视角
    AP_AHRS_View *ahrs_view;

    // 武装/解除武装管理类
    AP_Arming_Copter arming;

    // 光流传感器
#if AP_OPTICALFLOW_ENABLED
    AP_OpticalFlow optflow;
#endif

    // 外部控制库
#if AP_EXTERNAL_CONTROL_ENABLED
    AP_ExternalControl_Copter external_control;
#endif


    // EKF最后记录的偏航重置的系统时间(毫秒)
    uint32_t ekfYawReset_ms;
    int8_t ekf_primary_core;            // EKF主核心索引

    // 振动检查
    struct {
        bool high_vibes;    // 检测到高振动时为true
        uint32_t start_ms;  // 上次检测到高振动的系统时间
        uint32_t clear_ms;  // 高振动停止的系统时间
    } vibration_check;

    // EKF方差是未经过滤的,设计用于在可能的情况下快速恢复
    // 因此故障保护应该在过滤后的值上触发,以避免瞬时错误
    LowPassFilterFloat pos_variance_filt;   // 位置方差低通滤波器
    LowPassFilterFloat vel_variance_filt;   // 速度方差低通滤波器
    LowPassFilterFloat hgt_variance_filt;   // 高度方差低通滤波器
    bool variances_valid;                   // 方差是否有效
    uint32_t last_ekf_check_us;             // 上次EKF检查的微秒时间

    // 起飞检查
    uint32_t takeoff_check_warning_ms;  // 上次警告用户起飞检查失败的系统时间

    // GCS选择
    GCS_Copter _gcs; // 避免使用这个; 使用gcs()
    GCS_Copter &gcs() { return _gcs; }

    // 用户变量
#ifdef USERHOOK_VARIABLES
# include USERHOOK_VARIABLES
#endif

    // 全局变量文档:
    typedef union {
        struct {
            uint8_t unused1                 : 1; // 0
            uint8_t unused_was_simple_mode  : 2; // 1,2
            uint8_t pre_arm_rc_check        : 1; // 3       // 如果RC输入预武装检查已成功完成则为true
            uint8_t pre_arm_check           : 1; // 4       // 如果所有预武装检查(rc,加速度计校准,GPS锁定)已执行则为true
            uint8_t auto_armed              : 1; // 5       // 防止自动任务在油门抬高前开始
            uint8_t logging_started         : 1; // 6       // 如果日志记录已开始则为true
            uint8_t land_complete           : 1; // 7       // 如果我们检测到着陆则为true
            uint8_t new_radio_frame         : 1; // 8       // 如果我们有新的PWM数据要处理则设置为true
            uint8_t usb_connected_unused    : 1; // 9       // 未使用
            uint8_t rc_receiver_present_unused : 1; // 10      // 未使用
            uint8_t compass_mot             : 1; // 11      // 如果我们当前正在执行指南针电机校准则为true
            uint8_t motor_test              : 1; // 12      // 如果我们当前正在执行电机测试则为true
            uint8_t initialised             : 1; // 13      // init_ardupilot函数完成后为true。在此完成之前不会向GCS发送扩展状态
            uint8_t land_complete_maybe     : 1; // 14      // 如果我们可能已着陆则为true(比land_complete宽松的版本)
            uint8_t throttle_zero           : 1; // 15      // 如果油门摇杆在零位(去抖动),确定飞行员在不使用电机联锁时是否打算关机
            uint8_t system_time_set_unused  : 1; // 16      // 如果系统时间已从GPS设置则为true
            uint8_t gps_glitching           : 1; // 17      // 如果GPS故障影响导航精度则为true
            uint8_t using_interlock         : 1; // 20      // 辅助开关电机联锁功能正在使用
            uint8_t land_repo_active        : 1; // 21      // 如果飞行员正在覆盖着陆位置则为true
            uint8_t motor_interlock_switch  : 1; // 22      // 如果飞行员请求启用电机联锁则为true
            uint8_t in_arming_delay         : 1; // 23      // 如果我们已武装但正在等待旋转电机则为true
            uint8_t initialised_params      : 1; // 24      // 当所有参数都已初始化时为true。在此之前我们不能向GCS发送参数
            uint8_t unused3                 : 1; // 25      // 曾用于compass_init_location; 当指南针的初始位置已设置时为true
            uint8_t unused2                 : 1; // 26      // 允许辅助开关rc_override
            uint8_t armed_with_airmode_switch : 1; // 27      // 我们使用武装开关武装
            uint8_t prec_land_active        : 1; // 28      // 如果精确着陆处于活动状态则为true
        };
        uint32_t value;
    } ap_t;

    ap_t ap;

    AirMode air_mode; // 空中模式为0 = 未配置; 1 = 禁用; 2 = 启用;
    bool force_flying; // 当为true时强制飞行启用;

    static_assert(sizeof(uint32_t) == sizeof(ap), "ap_t必须是uint32_t");

    // 这是飞行控制系统的状态
    // 定义了多个状态,如STABILIZE, ACRO,
    Mode *flightmode;
    Mode::Number prev_control_mode;

    RCMapper rcmap;

    // 武装时的惯性导航高度
    float arming_altitude_m;

    // 故障保护
    struct {
        uint32_t terrain_first_failure_ms;  // 地形数据访问首次失败的时间 - 用于计算失败持续时间
        uint32_t terrain_last_failure_ms;   // 地形数据访问最近一次失败的时间

        int8_t radio_counter;            // 油门低于throttle_fs_value的迭代次数

        uint8_t radio               : 1; // 无线电故障保护的状态标志
        uint8_t gcs                 : 1; // 地面站故障保护的状态标志
        uint8_t ekf                 : 1; // 如果ekf故障保护已发生则为true
        uint8_t terrain             : 1; // 如果缺失地形数据故障保护已发生则为true
        uint8_t adsb                : 1; // 如果与adsb相关的故障保护已发生则为true
        uint8_t deadreckon          : 1; // 如果死推算故障保护已触发则为true
    } failsafe;

    bool any_failsafe_triggered() const {
        return failsafe.radio || battery.has_failsafed() || failsafe.gcs || failsafe.ekf || failsafe.terrain || failsafe.adsb || failsafe.deadreckon;
    }

    // 死推算状态
    struct {
        bool active;        // 如果死推算活动(使用估计的空速进行位置估计,没有位置或速度源)则为true
        bool timeout;       // 如果死推算已超时,EKF的位置和速度估计不应再被信任则为true
        uint32_t start_ms;  // EKF开始死推算的系统时间
    } dead_reckoning;

    // 电机输出
    MOTOR_CLASS *motors;
    const struct AP_Param::GroupInfo *motors_var_info;

    int32_t _home_bearing;
    uint32_t _home_distance;

    // 简单模式
    // 用于跟踪简单模式下车辆的方向。此值在每次武装时重置
    // 或在超级简单模式下,当车辆离开距离家20m半径时重置。
    enum class SimpleMode {
        NONE = 0,
        SIMPLE = 1,
        SUPERSIMPLE = 2,
    } simple_mode;

    float simple_cos_yaw;
    float simple_sin_yaw;
    int32_t super_simple_last_bearing;
    float super_simple_cos_yaw;
    float super_simple_sin_yaw;

    // 存储武装时的初始方位 - 初始简单方位在超级简单模式下被修改,因此不适用
    int32_t initial_armed_bearing;

    // 电池传感器
    AP_BattMonitor battery{MASK_LOG_CURRENT,
                           FUNCTOR_BIND_MEMBER(&Copter::handle_battery_failsafe, void, const char*, const int8_t),
                           _failsafe_priorities};

#if OSD_ENABLED || OSD_PARAM_ENABLED
    AP_OSD osd;
#endif

    // 高度
    int32_t baro_alt;            // 气压计高度,相对于家的厘米
    LowPassFilterVector3f land_accel_ef_filter; // 用于着陆和坠机检测测试的加速度

    // 过滤后的飞行员油门输入,用于在油门保持高时取消着陆
    LowPassFilterFloat rc_throttle_control_in_filter;

    // 3D位置向量
    // 车辆当前位置(高度相对于家)
    Location current_loc;

    // 惯性导航
    AP_InertialNav inertial_nav;

    // 姿态、位置和航点导航对象
    // 待办:将惯性导航上移或其他导航变量下移到这里
    AC_AttitudeControl *attitude_control;
    const struct AP_Param::GroupInfo *attitude_control_var_info;
    AC_PosControl *pos_control;
    AC_WPNav *wp_nav;
    AC_Loiter *loiter_nav;

#if AC_CUSTOMCONTROL_MULTI_ENABLED
    AC_CustomControl custom_control{ahrs_view, attitude_control, motors, scheduler.get_loop_period_s()};
#endif

#if MODE_CIRCLE_ENABLED
    AC_Circle *circle_nav;
#endif

    // 系统计时器
    // --------------
    // arm_time_ms - 记录车辆武装的时间。如果我们解除武装,则为零。
    uint32_t arm_time_ms;

    // 用于退出横滚和俯仰自动微调功能
    uint8_t auto_trim_counter;
    bool auto_trim_started = false;

    // 相机
#if AP_CAMERA_ENABLED
    AP_Camera camera{MASK_LOG_CAMERA};
#endif

    // 相机/天线云台跟踪和稳定
#if HAL_MOUNT_ENABLED
    AP_Mount camera_mount;
#endif

#if AP_AVOIDANCE_ENABLED
    AC_Avoid avoid;
#endif

    // 集结点库
#if HAL_RALLY_ENABLED
    AP_Rally_Copter rally;
#endif

    // 作物喷洒器
#if HAL_SPRAYER_ENABLED
    AC_Sprayer sprayer;
#endif

    // 降落伞释放
#if HAL_PARACHUTE_ENABLED
    AP_Parachute parachute;
#endif

    // 起落架控制器
#if AP_LANDINGGEAR_ENABLED
    AP_LandingGear landinggear;
#endif

    // 地形处理
#if AP_TERRAIN_AVAILABLE
    AP_Terrain terrain;
#endif

    // 精确着陆
#if AC_PRECLAND_ENABLED
    AC_PrecLand precland;
    AC_PrecLand_StateMachine precland_statemachine;
#endif

    // 飞行员输入管理库
    // 目前仅用于直升机
#if FRAME_CONFIG == HELI_FRAME
    AC_InputManager_Heli input_manager;
#endif

#if HAL_ADSB_ENABLED
    AP_ADSB adsb;

    // 避开启用adsb的车辆(通常是载人车辆)
    AP_Avoidance_Copter avoidance_adsb{adsb};
#endif

    // 最后有效的RC输入时间
    uint32_t last_radio_update_ms;

    // 最后ESC校准通知更新
    uint32_t esc_calibration_notify_update_ms;

    // 顶层逻辑
    // 设置var_info表
    AP_Param param_loader;

#if FRAME_CONFIG == HELI_FRAME
    // 模式滤波器,用于拒绝RC输入毛刺。滤波器大小为5,它提取第4个元素,因此可以拒绝3个低毛刺,
    // 和1个高毛刺。这是因为对于运行ESC调速器的直升机来说,任何"关闭"毛刺都可能非常有问题。
    // 即使是单个"关闭"帧也可能导致转子显著减速并需要很长时间才能重新启动。
    ModeFilterInt16_Size5 rotor_speed_deglitch_filter {4};

    // 传统直升机标志
    typedef struct {
        uint8_t dynamic_flight          : 1;    // 0   // 如果我们以显著速度移动则为true(用于打开/关闭漏积分项)
        bool coll_stk_low                  ;    // 1   // 当总距杆在下限时为true
    } heli_flags_t;
    heli_flags_t heli_flags;

    int16_t hover_roll_trim_scalar_slew;
#endif

    // 地面效应检测器
    struct {
        bool takeoff_expected;
        bool touchdown_expected;
        uint32_t takeoff_time_ms;
        float takeoff_alt_cm;
    } gndeffect_state;

    bool standby_active;

    static const AP_Scheduler::Task scheduler_tasks[];
    static const AP_Param::Info var_info[];
    static const struct LogStructure log_structure[];

    // ESC校准的枚举
    enum ESCCalibrationModes : uint8_t {
        ESCCAL_NONE = 0,
        ESCCAL_PASSTHROUGH_IF_THROTTLE_HIGH = 1,
        ESCCAL_PASSTHROUGH_ALWAYS = 2,
        ESCCAL_AUTO = 3,
        ESCCAL_DISABLED = 9,
    };

    enum class FailsafeAction : uint8_t {
        NONE               = 0,
        LAND               = 1,
        RTL                = 2,
        SMARTRTL           = 3,
        SMARTRTL_LAND      = 4,
        TERMINATE          = 5,
        AUTO_DO_LAND_START = 6,
        BRAKE_LAND         = 7
    };

    enum class FailsafeOption {
        RC_CONTINUE_IF_AUTO             = (1<<0),   // 1
        GCS_CONTINUE_IF_AUTO            = (1<<1),   // 2
        RC_CONTINUE_IF_GUIDED           = (1<<2),   // 4
        CONTINUE_IF_LANDING             = (1<<3),   // 8
        GCS_CONTINUE_IF_PILOT_CONTROL   = (1<<4),   // 16
        RELEASE_GRIPPER                 = (1<<5),   // 32
    };


    enum class FlightOption : uint32_t {
        DISABLE_THRUST_LOSS_CHECK     = (1<<0),   // 1
        DISABLE_YAW_IMBALANCE_WARNING = (1<<1),   // 2
        RELEASE_GRIPPER_ON_THRUST_LOSS = (1<<2),  // 4
        REQUIRE_POSITION_FOR_ARMING =   (1<<3),   // 8
    };
    // 如果此车辆启用了选项则返回true
    bool option_is_enabled(FlightOption option) const {
        return (g2.flight_options & uint32_t(option)) != 0;
    }

    static constexpr int8_t _failsafe_priorities[] = {
                                                      (int8_t)FailsafeAction::TERMINATE,
                                                      (int8_t)FailsafeAction::LAND,
                                                      (int8_t)FailsafeAction::RTL,
                                                      (int8_t)FailsafeAction::SMARTRTL_LAND,
                                                      (int8_t)FailsafeAction::SMARTRTL,
                                                      (int8_t)FailsafeAction::NONE,
                                                      -1 // 优先级列表必须以-1作为哨兵结束
                                                     };

    #define FAILSAFE_LAND_PRIORITY 1
    static_assert(_failsafe_priorities[FAILSAFE_LAND_PRIORITY] == (int8_t)FailsafeAction::LAND,
                  "FAILSAFE_LAND_PRIORITY必须与_failsafe_priorities中的条目匹配");
    static_assert(_failsafe_priorities[ARRAY_SIZE(_failsafe_priorities) - 1] == -1,
                  "_failsafe_priorities缺少哨兵");


    // AP_State.cpp
    void set_auto_armed(bool b);  // 设置自动解锁状态
    void set_simple_mode(SimpleMode b);  // 设置简单模式
    void set_failsafe_radio(bool b);  // 设置遥控器失效保护
    void set_failsafe_gcs(bool b);  // 设置地面站失效保护
    void update_using_interlock();  // 更新互锁使用状态

    // Copter.cpp
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                             uint8_t &task_count,
                             uint32_t &log_bit) override;  // 获取调度器任务
#if AP_SCRIPTING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED
#if MODE_GUIDED_ENABLED
    bool set_target_location(const Location& target_loc) override;  // 设置目标位置
#endif // MODE_GUIDED_ENABLED
#endif // AP_SCRIPTING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED

#if AP_SCRIPTING_ENABLED
#if MODE_GUIDED_ENABLED
    bool start_takeoff(float alt) override;  // 开始起飞
    bool get_target_location(Location& target_loc) override;  // 获取目标位置
    bool update_target_location(const Location &old_loc, const Location &new_loc) override;  // 更新目标位置
    bool set_target_pos_NED(const Vector3f& target_pos, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool yaw_relative, bool terrain_alt) override;  // 设置NED坐标系下的目标位置
    bool set_target_posvel_NED(const Vector3f& target_pos, const Vector3f& target_vel) override;  // 设置NED坐标系下的目标位置和速度
    bool set_target_posvelaccel_NED(const Vector3f& target_pos, const Vector3f& target_vel, const Vector3f& target_accel, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool yaw_relative) override;  // 设置NED坐标系下的目标位置、速度和加速度
    bool set_target_velocity_NED(const Vector3f& vel_ned) override;  // 设置NED坐标系下的目标速度
    bool set_target_velaccel_NED(const Vector3f& target_vel, const Vector3f& target_accel, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool relative_yaw) override;  // 设置NED坐标系下的目标速度和加速度
    bool set_target_angle_and_climbrate(float roll_deg, float pitch_deg, float yaw_deg, float climb_rate_ms, bool use_yaw_rate, float yaw_rate_degs) override;  // 设置目标角度和爬升率
    bool set_target_rate_and_throttle(float roll_rate_dps, float pitch_rate_dps, float yaw_rate_dps, float throttle) override;  // 设置目标角速率和油门

#endif
#if MODE_CIRCLE_ENABLED
    bool get_circle_radius(float &radius_m) override;  // 获取圆形模式半径
    bool set_circle_rate(float rate_dps) override;  // 设置圆形模式角速度
#endif
    bool set_desired_speed(float speed) override;  // 设置期望速度
#if MODE_AUTO_ENABLED
    bool nav_scripting_enable(uint8_t mode) override;  // 启用导航脚本
    bool nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2, int16_t &arg3, int16_t &arg4) override;  // 导航脚本时间
    void nav_script_time_done(uint16_t id) override;  // 导航脚本时间完成
#endif
    // lua脚本使用此函数获取EKF失效保护状态
    // 如果EKF失效保护触发则返回true
    bool has_ekf_failsafed() const override;
#endif // AP_SCRIPTING_ENABLED
    bool is_landing() const override;  // 是否正在着陆
    bool is_taking_off() const override;  // 是否正在起飞
    void rc_loop();  // 遥控循环
    void throttle_loop();  // 油门循环
    void update_batt_compass(void);  // 更新电池和罗盘
    void loop_rate_logging();  // 循环速率日志记录
    void ten_hz_logging_loop();  // 10Hz日志记录循环
    void twentyfive_hz_logging();  // 25Hz日志记录
    void three_hz_loop();  // 3Hz循环
    void one_hz_loop();  // 1Hz循环
    void init_simple_bearing();  // 初始化简单方位
    void update_simple_mode(void);  // 更新简单模式
    void update_super_simple_bearing(bool force_update);  // 更新超级简单方位
    void read_AHRS(void);  // 读取AHRS(姿态航向参考系统)
    void update_altitude();  // 更新高度
    bool get_wp_distance_m(float &distance) const override;  // 获取到航点的距离
    bool get_wp_bearing_deg(float &bearing) const override;  // 获取到航点的方位角
    bool get_wp_crosstrack_error_m(float &xtrack_error) const override;  // 获取航点的横向跟踪误差
    bool get_rate_ef_targets(Vector3f& rate_ef_targets) const override;  // 获取地球坐标系下的目标角速率

    // Attitude.cpp
    void update_throttle_hover();  // 更新悬停油门
    float get_pilot_desired_climb_rate(float throttle_control);  // 获取飞行员期望的爬升率
    float get_non_takeoff_throttle();  // 获取非起飞油门
    void set_accel_throttle_I_from_pilot_throttle();  // 根据飞行员油门设置加速度油门积分项
    void rotate_body_frame_to_NE(float &x, float &y);  // 将机体坐标系旋转到NE坐标系
    uint16_t get_pilot_speed_dn() const;  // 获取飞行员下降速度
    void run_rate_controller();  // 运行角速率控制器

#if AC_CUSTOMCONTROL_MULTI_ENABLED
    void run_custom_controller() { custom_control.update(); }  // 运行自定义控制器
#endif

    // avoidance.cpp
    void low_alt_avoidance();  // 低空避障

#if HAL_ADSB_ENABLED
    // avoidance_adsb.cpp
    void avoidance_adsb_update(void);  // 更新ADS-B避障
#endif

    // baro_ground_effect.cpp
    void update_ground_effect_detector(void);  // 更新地面效应检测器
    void update_ekf_terrain_height_stable();  // 更新EKF地形高度稳定性

    // commands.cpp
    void update_home_from_EKF();  // 从EKF更新Home点
    void set_home_to_current_location_inflight();  // 在飞行中将当前位置设为Home点
    bool set_home_to_current_location(bool lock) override WARN_IF_UNUSED;  // 将当前位置设为Home点
    bool set_home(const Location& loc, bool lock) override WARN_IF_UNUSED;  // 设置Home点

    // compassmot.cpp
    MAV_RESULT mavlink_compassmot(const GCS_MAVLINK &gcs_chan);  // MAVLink罗盘电机校准

    // crash_check.cpp
    void crash_check();  // 坠机检查
    void thrust_loss_check();  // 推力损失检查
    void yaw_imbalance_check();  // 偏航不平衡检查
    LowPassFilterFloat yaw_I_filt{0.05f};  // 偏航积分项低通滤波器
    uint32_t last_yaw_warn_ms;  // 上次偏航警告时间
    void parachute_check();  // 降落伞检查
    void parachute_release();  // 释放降落伞
    void parachute_manual_release();  // 手动释放降落伞

    // ekf_check.cpp
    void ekf_check();  // EKF检查
    bool ekf_over_threshold();  // EKF是否超过阈值
    void failsafe_ekf_event();  // EKF失效保护事件
    void failsafe_ekf_off_event(void);  // EKF失效保护关闭事件
    void failsafe_ekf_recheck();  // 重新检查EKF失效保护
    void check_ekf_reset();  // 检查EKF重置
    void check_vibration();  // 检查振动

    // esc_calibration.cpp
    void esc_calibration_startup_check();  // ESC校准启动检查
    void esc_calibration_passthrough();  // ESC校准直通模式
    void esc_calibration_auto();  // ESC自动校准
    void esc_calibration_notify();  // ESC校准通知
    void esc_calibration_setup();  // ESC校准设置

    // events.cpp
    bool failsafe_option(FailsafeOption opt) const;  // 失效保护选项
    void failsafe_radio_on_event();  // 遥控器失效保护开启事件
    void failsafe_radio_off_event();  // 遥控器失效保护关闭事件
    void handle_battery_failsafe(const char* type_str, const int8_t action);  // 处理电池失效保护
    void failsafe_gcs_check();  // 地面站失效保护检查
    void failsafe_gcs_on_event(void);  // 地面站失效保护开启事件
    void failsafe_gcs_off_event(void);  // 地面站失效保护关闭事件
    void failsafe_terrain_check();  // 地形失效保护检查
    void failsafe_terrain_set_status(bool data_ok);  // 设置地形失效保护状态
    void failsafe_terrain_on_event();  // 地形失效保护开启事件
    void gpsglitch_check();  // GPS故障检查
    void failsafe_deadreckon_check();  // 航位推算失效保护检查
    void set_mode_RTL_or_land_with_pause(ModeReason reason);  // 设置RTL或带暂停的着陆模式
    void set_mode_SmartRTL_or_RTL(ModeReason reason);  // 设置智能RTL或RTL模式
    void set_mode_SmartRTL_or_land_with_pause(ModeReason reason);  // 设置智能RTL或带暂停的着陆模式
    void set_mode_auto_do_land_start_or_RTL(ModeReason reason);  // 设置自动着陆开始或RTL模式
    void set_mode_brake_or_land_with_pause(ModeReason reason);  // 设置刹车或带暂停的着陆模式
    bool should_disarm_on_failsafe();  // 是否应在失效保护时解锁
    void do_failsafe_action(FailsafeAction action, ModeReason reason);  // 执行失效保护动作
    void announce_failsafe(const char *type, const char *action_undertaken=nullptr);  // 宣布失效保护

    // failsafe.cpp
    void failsafe_enable();  // 启用失效保护
    void failsafe_disable();  // 禁用失效保护
#if ADVANCED_FAILSAFE
    void afs_fs_check(void);  // 高级失效保护检查
#endif

    // fence.cpp
#if AP_FENCE_ENABLED
    void fence_check();  // 地理围栏检查
#endif

    // heli.cpp
    void heli_init();  // 直升机初始化
    void check_dynamic_flight(void);  // 检查动态飞行
    bool should_use_landing_swash() const;  // 是否应使用着陆斜盘
    void update_heli_control_dynamics(void);  // 更新直升机控制动态
    void heli_update_landing_swash();  // 更新直升机着陆斜盘
    float get_pilot_desired_rotor_speed() const;  // 获取飞行员期望的旋翼速度
    void heli_update_rotor_speed_targets();  // 更新直升机旋翼速度目标
    void heli_update_autorotation();  // 更新直升机自转
    void update_collective_low_flag(int16_t throttle_control);  // 更新集体桨距低标志

    // inertia.cpp
    void read_inertia();  // 读取惯性

    // landing_detector.cpp
    void update_land_and_crash_detectors();  // 更新着陆和坠机检测器
    void update_land_detector();  // 更新着陆检测器
    void set_land_complete(bool b);  // 设置着陆完成
    void set_land_complete_maybe(bool b);  // 设置可能着陆完成
    void update_throttle_mix();  // 更新油门混合
    bool get_force_flying() const;  // 获取强制飞行状态
#if HAL_LOGGING_ENABLED
    enum class LandDetectorLoggingFlag : uint16_t {
        LANDED               = 1U <<  0,  // 已着陆
        LANDED_MAYBE         = 1U <<  1,  // 可能已着陆
        LANDING              = 1U <<  2,  // 正在着陆
        STANDBY_ACTIVE       = 1U <<  3,  // 待机激活
        WOW                  = 1U <<  4,  // 重量在轮上
        RANGEFINDER_BELOW_2M = 1U <<  5,  // 测距仪低于2米
        DESCENT_RATE_LOW     = 1U <<  6,  // 下降率低
        ACCEL_STATIONARY     = 1U <<  7,  // 加速度静止
        LARGE_ANGLE_ERROR    = 1U <<  8,  // 大角度误差
        LARGE_ANGLE_REQUEST  = 1U <<  8,  // 大角度请求
        MOTOR_AT_LOWER_LIMIT = 1U <<  9,  // 电机在下限
        THROTTLE_MIX_AT_MIN  = 1U << 10,  // 油门混合在最小值
    };
    struct {
        uint32_t last_logged_ms;  // 上次记录时间
        uint32_t last_logged_count;  // 上次记录计数
        uint16_t last_logged_flags;  // 上次记录标志
    } land_detector;
    void Log_LDET(uint16_t logging_flags, uint32_t land_detector_count);  // 记录着陆检测器日志
#endif

#if AP_LANDINGGEAR_ENABLED
    // landing_gear.cpp
    void landinggear_update();  // 更新起落架
#endif

    // standby.cpp
    void standby_update();  // 更新待机状态

#if HAL_LOGGING_ENABLED
    // methods for AP_Vehicle:
    const AP_Int32 &get_log_bitmask() override { return g.log_bitmask; }  // 获取日志位掩码
    const struct LogStructure *get_log_structures() const override {
        return log_structure;
    }  // 获取日志结构
    uint8_t get_num_log_structures() const override;  // 获取日志结构数量

    // Log.cpp
    void Log_Write_Control_Tuning();  // 记录控制调谐日志
    void Log_Write_Attitude();  // 记录姿态日志
    void Log_Write_EKF_POS();  // 记录EKF位置日志
    void Log_Write_PIDS();  // 记录PID日志
    void Log_Write_Data(LogDataID id, int32_t value);  // 记录数据日志(int32_t)
    void Log_Write_Data(LogDataID id, uint32_t value);  // 记录数据日志(uint32_t)
    void Log_Write_Data(LogDataID id, int16_t value);  // 记录数据日志(int16_t)
    void Log_Write_Data(LogDataID id, uint16_t value);  // 记录数据日志(uint16_t)
    void Log_Write_Data(LogDataID id, float value);  // 记录数据日志(float)
    void Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, float tune_min, float tune_max);  // 记录参数调谐日志
    void Log_Video_Stabilisation();  // 记录视频稳定日志
    void Log_Write_Guided_Position_Target(ModeGuided::SubMode submode, const Vector3f& pos_target, bool terrain_alt, const Vector3f& vel_target, const Vector3f& accel_target);  // 记录引导模式位置目标日志
    void Log_Write_Guided_Attitude_Target(ModeGuided::SubMode target_type, float roll, float pitch, float yaw, const Vector3f &ang_vel, float thrust, float climb_rate);  // 记录引导模式姿态目标日志
    void Log_Write_SysID_Setup(uint8_t systemID_axis, float waveform_magnitude, float frequency_start, float frequency_stop, float time_fade_in, float time_const_freq, float time_record, float time_fade_out);  // 记录系统识别设置日志
    void Log_Write_SysID_Data(float waveform_time, float waveform_sample, float waveform_freq, float angle_x, float angle_y, float angle_z, float accel_x, float accel_y, float accel_z);  // 记录系统识别数据日志
    void Log_Write_Vehicle_Startup_Messages();  // 记录车辆启动消息日志
#endif  // HAL_LOGGING_ENABLED

    // mode.cpp
    bool set_mode(Mode::Number mode, ModeReason reason);  // 设置飞行模式
    bool set_mode(const uint8_t new_mode, const ModeReason reason) override;  // 设置飞行模式
    ModeReason _last_reason;  // 上次模式切换原因
    // 当尝试切换到一个模式失败时调用:
    void mode_change_failed(const Mode *mode, const char *reason);  // 模式切换失败
    uint8_t get_mode() const override { return (uint8_t)flightmode->mode_number(); }  // 获取当前飞行模式
    bool current_mode_requires_mission() const override;  // 当前模式是否需要任务
    void update_flight_mode();  // 更新飞行模式
    void notify_flight_mode();  // 通知飞行模式

    // 检查此模式是否可以从GCS进入
    bool gcs_mode_enabled(const Mode::Number mode_num);  // 检查GCS是否启用了该模式

    // mode_land.cpp
    void set_mode_land_with_pause(ModeReason reason);  // 设置带暂停的着陆模式
    bool landing_with_GPS();  // 是否使用GPS着陆

    // motor_test.cpp
    void motor_test_output();  // 电机测试输出
    bool mavlink_motor_control_check(const GCS_MAVLINK &gcs_chan, bool check_rc, const char* mode);  // MAVLink电机控制检查
    MAV_RESULT mavlink_motor_test_start(const GCS_MAVLINK &gcs_chan, uint8_t motor_seq, uint8_t throttle_type, float throttle_value, float timeout_sec, uint8_t motor_count);  // 开始MAVLink电机测试
    void motor_test_stop();  // 停止电机测试

    // motors.cpp
    void arm_motors_check();  // 检查电机解锁
    void auto_disarm_check();  // 自动解锁检查
    void motors_output();  // 电机输出
    void lost_vehicle_check();  // 丢失车辆检查

    // navigation.cpp
    void run_nav_updates(void);  // 运行导航更新
    int32_t home_bearing();  // Home点方位
    uint32_t home_distance();  // Home点距离

    // Parameters.cpp
    void load_parameters(void) override;  // 加载参数
    void convert_pid_parameters(void);  // 转换PID参数
#if HAL_PROXIMITY_ENABLED
    void convert_prx_parameters();  // 转换接近传感器参数
#endif
    void convert_lgr_parameters(void);  // 转换起落架参数
    void convert_tradheli_parameters(void) const;  // 转换传统直升机参数

    // precision_landing.cpp
    void init_precland();  // 初始化精确着陆
    void update_precland();  // 更新精确着陆

    // radio.cpp
    void default_dead_zones();  // 默认死区
    void init_rc_in();  // 初始化遥控输入
    void init_rc_out();  // 初始化遥控输出
    void read_radio();  // 读取遥控器
    void set_throttle_and_failsafe(uint16_t throttle_pwm);  // 设置油门和失效保护
    void set_throttle_zero_flag(int16_t throttle_control);  // 设置油门零标志
    void radio_passthrough_to_motors();  // 遥控器直通到电机
    int16_t get_throttle_mid(void);  // 获取油门中点

    // sensors.cpp
    void read_barometer(void);  // 读取气压计
    void init_rangefinder(void);  // 初始化测距仪
    void read_rangefinder(void);  // 读取测距仪
    bool rangefinder_alt_ok() const;  // 测距仪高度是否正常
    bool rangefinder_up_ok() const;  // 向上测距仪是否正常
    void update_rangefinder_terrain_offset();  // 更新测距仪地形偏移
    void update_optical_flow(void);  // 更新光流

    // takeoff_check.cpp
    void takeoff_check();  // 起飞检查

    // RC_Channel.cpp
    void save_trim();  // 保存微调
    void auto_trim();  // 自动微调
    void auto_trim_cancel();  // 取消自动微调

    // system.cpp
    void init_ardupilot() override;  // 初始化ArduPilot
    void startup_INS_ground();  // 地面INS启动
    bool position_ok() const;  // 位置是否正常
    bool ekf_has_absolute_position() const;  // EKF是否有绝对位置
    bool ekf_has_relative_position() const;  // EKF是否有相对位置
    bool ekf_alt_ok() const;  // EKF高度是否正常
    void update_auto_armed();  // 更新自动解锁
    bool should_log(uint32_t mask);  // 是否应该记录日志
    const char* get_frame_string() const;  // 获取机架字符串
    void allocate_motors(void);  // 分配电机
    bool is_tradheli() const;  // 是否为传统直升机

    // terrain.cpp
    void terrain_update();  // 更新地形
    void terrain_logging();  // 地形日志记录

    // tuning.cpp
    void tuning();  // 执行飞行中调整

    // UserCode.cpp
    void userhook_init();  // 用户自定义初始化钩子函数
    void userhook_FastLoop();  // 用户自定义快速循环钩子函数
    void userhook_50Hz();  // 用户自定义50Hz循环钩子函数
    void userhook_MediumLoop();  // 用户自定义中速循环钩子函数
    void userhook_SlowLoop();  // 用户自定义慢速循环钩子函数
    void userhook_SuperSlowLoop();  // 用户自定义超慢速循环钩子函数
    void userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag);  // 用户自定义辅助开关1钩子函数
    void userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag);  // 用户自定义辅助开关2钩子函数
    void userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag);  // 用户自定义辅助开关3钩子函数

    // 各种飞行模式的定义
#if MODE_ACRO_ENABLED
#if FRAME_CONFIG == HELI_FRAME
    ModeAcro_Heli mode_acro;  // 直升机特定的特技模式
#else
    ModeAcro mode_acro;  // 标准特技模式
#endif
#endif
    ModeAltHold mode_althold;  // 定高模式
#if MODE_AUTO_ENABLED
    ModeAuto mode_auto;  // 自动模式
#endif
#if AUTOTUNE_ENABLED
    ModeAutoTune mode_autotune;  // 自动调参模式
#endif
#if MODE_BRAKE_ENABLED
    ModeBrake mode_brake;  // 刹车模式
#endif
#if MODE_CIRCLE_ENABLED
    ModeCircle mode_circle;  // 绕圈模式
#endif
#if MODE_DRIFT_ENABLED
    ModeDrift mode_drift;  // 漂移模式
#endif
#if MODE_FLIP_ENABLED
    ModeFlip mode_flip;  // 翻转模式
#endif
#if MODE_FOLLOW_ENABLED
    ModeFollow mode_follow;  // 跟随模式
#endif
#if MODE_GUIDED_ENABLED
    ModeGuided mode_guided;  // 引导模式
#endif
    ModeLand mode_land;  // 着陆模式
#if MODE_LOITER_ENABLED
    ModeLoiter mode_loiter;  // 悬停模式
#endif
#if MODE_POSHOLD_ENABLED
    ModePosHold mode_poshold;  // 定点模式
#endif
#if MODE_RTL_ENABLED
    ModeRTL mode_rtl;  // 返航模式
#endif
#if FRAME_CONFIG == HELI_FRAME
    ModeStabilize_Heli mode_stabilize;  // 直升机特定的自稳模式
#else
    ModeStabilize mode_stabilize;  // 标准自稳模式
#endif
#if MODE_SPORT_ENABLED
    ModeSport mode_sport;  // 运动模式
#endif
#if MODE_SYSTEMID_ENABLED
    ModeSystemId mode_systemid;  // 系统识别模式
#endif
#if HAL_ADSB_ENABLED
    ModeAvoidADSB mode_avoid_adsb;  // ADS-B避障模式
#endif
#if MODE_THROW_ENABLED
    ModeThrow mode_throw;  // 抛飞模式
#endif
#if MODE_GUIDED_NOGPS_ENABLED
    ModeGuidedNoGPS mode_guided_nogps;  // 无GPS引导模式
#endif
#if MODE_SMARTRTL_ENABLED
    ModeSmartRTL mode_smartrtl;  // 智能返航模式
#endif
#if MODE_FLOWHOLD_ENABLED
    ModeFlowHold mode_flowhold;  // 光流定点模式
#endif
#if MODE_ZIGZAG_ENABLED
    ModeZigZag mode_zigzag;  // 之字形模式
#endif
#if MODE_AUTOROTATE_ENABLED
    ModeAutorotate mode_autorotate;  // 自旋模式
#endif
#if MODE_TURTLE_ENABLED
    ModeTurtle mode_turtle;  // 倒置恢复模式
#endif

    // mode.cpp
    Mode *mode_from_mode_num(const Mode::Number mode);  // 根据模式编号获取模式指针
    void exit_mode(Mode *&old_flightmode, Mode *&new_flightmode);  // 退出当前模式并进入新模式

public:
    void failsafe_check();      // failsafe.cpp  // 执行失效保护检查

};

extern Copter copter;  // 声明全局Copter对象

using AP_HAL::millis;  // 使用AP_HAL库的毫秒计时函数
using AP_HAL::micros;  // 使用AP_HAL库的微秒计时函数
