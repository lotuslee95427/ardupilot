/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// 包含配置头文件
#include "AP_Arming_config.h"

#if AP_ARMING_ENABLED

// 包含所需的头文件
#include "AP_Arming.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Notify/AP_Notify.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Proximity/AP_Proximity.h>
#include <AP_Rally/AP_Rally.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AC_Fence/AC_Fence.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Generator/AP_Generator.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_ADSB/AP_ADSB.h>
#include <AP_Scripting/AP_Scripting.h>
#include <AP_Camera/AP_RunCam.h>
#include <AP_GyroFFT/AP_GyroFFT.h>
#include <AP_VisualOdom/AP_VisualOdom.h>
#include <AP_Parachute/AP_Parachute.h>
#include <AP_OSD/AP_OSD.h>
#include <AP_Relay/AP_Relay.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_Button/AP_Button.h>
#include <AP_FETtecOneWire/AP_FETtecOneWire.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_Mount/AP_Mount.h>
#include <AP_OpenDroneID/AP_OpenDroneID.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_KDECAN/AP_KDECAN.h>
#include <AP_Vehicle/AP_Vehicle.h>

// 如果启用了CAN协议驱动
#if HAL_MAX_CAN_PROTOCOL_DRIVERS
  #include <AP_CANManager/AP_CANManager.h>
  #include <AP_Common/AP_Common.h>
  #include <AP_Vehicle/AP_Vehicle_Type.h>

  #include <AP_PiccoloCAN/AP_PiccoloCAN.h>
  #include <AP_DroneCAN/AP_DroneCAN.h>
#endif

#include <AP_Logger/AP_Logger.h>

// 定义一些常量
#define AP_ARMING_COMPASS_MAGFIELD_EXPECTED 530  // 预期的罗盘磁场强度
#define AP_ARMING_COMPASS_MAGFIELD_MIN  185     // 最小磁场强度 (0.35 * 530 毫高斯)
#define AP_ARMING_COMPASS_MAGFIELD_MAX  875     // 最大磁场强度 (1.65 * 530 毫高斯)
#define AP_ARMING_BOARD_VOLTAGE_MAX     5.8f    // 最大板载电压
#define AP_ARMING_ACCEL_ERROR_THRESHOLD 0.75f   // 加速度计误差阈值
#define AP_ARMING_MAGFIELD_ERROR_THRESHOLD 100  // 磁场误差阈值
#define AP_ARMING_AHRS_GPS_ERROR_MAX    10      // AHRS和GPS之间允许的最大差异(10米)

// 根据不同的飞行器类型设置默认的方向舵解锁模式
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
  #define ARMING_RUDDER_DEFAULT         (uint8_t)RudderArming::ARMONLY
#else
  #define ARMING_RUDDER_DEFAULT         (uint8_t)RudderArming::ARMDISARM
#endif

#ifndef PREARM_DISPLAY_PERIOD
# define PREARM_DISPLAY_PERIOD 30  // 预解锁显示周期(秒)
#endif

extern const AP_HAL::HAL& hal;

// 参数定义
const AP_Param::GroupInfo AP_Arming::var_info[] = {

    // @Param{Plane, Rover}: REQUIRE
    // @DisplayName: Require Arming Motors 
    // @Description: 解锁电机的要求。0:无要求(立即解锁)。1:解锁时向油门通道发送最小PWM值。2:解锁时向油门通道发送0 PWM(无信号)。
    // @Values: 0:禁用,1:解锁时最小PWM,2:解锁时0 PWM
    // @User: Advanced
    AP_GROUPINFO_FLAGS_FRAME("REQUIRE",     0,      AP_Arming,  require, float(Required::YES_MIN_PWM),
                             AP_PARAM_FLAG_NO_SHIFT,
                             AP_PARAM_FRAME_PLANE | AP_PARAM_FRAME_ROVER),

    // 2 was the CHECK paramter stored in a AP_Int16

    // @Param: ACCTHRESH
    // @DisplayName: 加速度计误差阈值
    // @Description: 用于判断加速度计不一致的误差阈值。将此误差范围与其他加速度计进行比较以检测硬件或校准错误。
    // @Units: m/s/s
    // @Range: 0.25 3.0
    // @User: Advanced
    AP_GROUPINFO("ACCTHRESH",    3,     AP_Arming,  accel_error_threshold,  AP_ARMING_ACCEL_ERROR_THRESHOLD),

    // index 4 was VOLT_MIN, moved to AP_BattMonitor
    // index 5 was VOLT2_MIN, moved to AP_BattMonitor

    // @Param{Plane,Rover,Copter,Blimp}: RUDDER
    // @DisplayName: 方向舵解锁使能/禁用
    // @Description: 允许通过方向舵输入解锁/锁定。启用时,右方向舵解锁,左方向舵锁定。
    // @Values: 0:禁用,1:仅解锁,2:解锁和锁定
    // @User: Advanced
    AP_GROUPINFO_FRAME("RUDDER",  6,     AP_Arming, _rudder_arming, ARMING_RUDDER_DEFAULT, AP_PARAM_FRAME_PLANE |
                                                                                           AP_PARAM_FRAME_ROVER |
                                                                                           AP_PARAM_FRAME_COPTER |
                                                                                           AP_PARAM_FRAME_TRICOPTER |
                                                                                           AP_PARAM_FRAME_HELI |
                                                                                           AP_PARAM_FRAME_BLIMP),

    // @Param: MIS_ITEMS
    // @DisplayName: 必需的任务项
    // @Description: 解锁飞行器所需的任务项位掩码
    // @Bitmask: 0:着陆,1:VTOL着陆,2:DO_LAND_START,3:起飞,4:VTOL起飞,5:集结点,6:返航
    // @User: Advanced
    AP_GROUPINFO("MIS_ITEMS",    7,     AP_Arming, _required_mission_items, 0),

    // @Param: CHECK
    // @DisplayName: 执行解锁检查(位掩码)
    // @Description: 解锁前的检查项。这是一个位掩码,表示在允许解锁前将执行的检查。
    // @Bitmask: 0:全部,1:气压计,2:罗盘,3:GPS锁定,4:INS,5:参数,6:RC通道,7:板载电压,8:电池电量,10:日志可用,11:硬件安全开关,12:GPS配置,13:系统,14:任务,15:测距仪,16:相机,17:辅助认证,18:视觉里程计,19:FFT
    // @User: Standard
    AP_GROUPINFO("CHECK",        8,     AP_Arming,  checks_to_perform,       ARMING_CHECK_ALL),

    // @Param: OPTIONS
    // @DisplayName: 解锁选项
    // @Description: 可以应用以改变解锁行为的选项
    // @Bitmask: 0:禁用预解锁显示,1:状态改变时不发送状态文本
    // @User: Advanced
    AP_GROUPINFO("OPTIONS", 9,   AP_Arming, _arming_options, 0),

    // @Param: MAGTHRESH
    // @DisplayName: 罗盘磁场强度误差阈值与地球磁场模型
    // @Description: 罗盘磁场强度误差阈值与地球磁场模型。X和Y轴使用此阈值进行比较,Z轴使用2倍此阈值。0表示禁用检查
    // @Units: mGauss
    // @Range: 0 500
    // @User: Advanced
    AP_GROUPINFO("MAGTHRESH", 10, AP_Arming, magfield_error_threshold,  AP_ARMING_MAGFIELD_ERROR_THRESHOLD),

#if AP_ARMING_CRASHDUMP_ACK_ENABLED
    // @Param: CRSDP_IGN
    // @DisplayName: 禁用崩溃转储解锁检查
    // @Description: 如果系统存在崩溃转储数据,必须将此值设为"1",否则会引发预解锁失败。
    // @Values: 0:崩溃转储解锁检查激活, 1:崩溃转储解锁检查停用
    // @User: Advanced
    AP_GROUPINFO("CRSDP_IGN", 11, AP_Arming, crashdump_ack.acked, 0),
#endif  // AP_ARMING_CRASHDUMP_ACK_ENABLED

    AP_GROUPEND
};

#if HAL_WITH_IO_MCU
#include <AP_IOMCU/AP_IOMCU.h>
extern AP_IOMCU iomcu;
#endif

#pragma GCC diagnostic push
#if defined (__clang__)
#pragma GCC diagnostic ignored "-Wbitwise-instead-of-logical"
#endif

// 构造函数
AP_Arming::AP_Arming()
{
    if (_singleton) {
        AP_HAL::panic("Too many AP_Arming instances");
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

// 执行预解锁检查。预期以1Hz的频率调用。
void AP_Arming::update(void)
{
#if AP_ARMING_CRASHDUMP_ACK_ENABLED
    // 如果启动时没有崩溃转储数据,重置"忽略"参数,
    // 这样用户在将来出现崩溃时需要重新确认
    crashdump_ack.check_reset();
#endif

    const uint32_t now_ms = AP_HAL::millis();
    // 每30秒执行预解锁检查并显示失败
    // 是否显示预解锁检查失败信息的标志
    bool display_fail = false;
    // 如果需要立即报告且距离上次显示超过4秒,或者距离上次显示超过PREARM_DISPLAY_PERIOD秒
    if ((report_immediately && (now_ms - last_prearm_display_ms > 4000)) ||
        (now_ms - last_prearm_display_ms > PREARM_DISPLAY_PERIOD*1000)) {
        // 清除立即报告标志
        report_immediately = false;
        // 设置显示失败标志
        display_fail = true;
        // 更新上次显示时间
        last_prearm_display_ms = now_ms;
    }
    // 另一方面,用户可能永远不想显示它们:
    if (option_enabled(Option::DISABLE_PREARM_DISPLAY)) {
        display_fail = false;
    }

    pre_arm_checks(display_fail);
}

#if AP_ARMING_CRASHDUMP_ACK_ENABLED
void AP_Arming::CrashDump::check_reset()
{
    // 如果没有崩溃转储数据则清除崩溃转储确认。
    // 这意味着在后续出现崩溃转储时用户必须重新确认。
    if (hal.util->last_crash_dump_size() == 0) {
        // 无崩溃转储数据
        acked.set_and_save_ifchanged(0);
    }
}
#endif  // AP_ARMING_CRASHDUMP_ACK_ENABLED

// 返回预期的罗盘磁场强度
uint16_t AP_Arming::compass_magfield_expected() const
{
    return AP_ARMING_COMPASS_MAGFIELD_EXPECTED;
}

// 检查是否已解锁
bool AP_Arming::is_armed() const
{
    return armed || arming_required() == Required::NO;
}

/*
  检查是否已解锁且安全开关已关闭
 */
bool AP_Arming::is_armed_and_safety_off() const
{
    return is_armed() && hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED;
}

/*
  获取已启用的检查项
 */
uint32_t AP_Arming::get_enabled_checks() const
{
    return checks_to_perform;
}

/*
  检查指定的检查项是否已启用
 */
bool AP_Arming::check_enabled(const enum AP_Arming::ArmingChecks check) const
{
    if (checks_to_perform & ARMING_CHECK_ALL) {
        return true;
    }
    return (checks_to_perform & check);
}

/*
  检查失败时的处理,带检查项参数
 */
void AP_Arming::check_failed(const enum AP_Arming::ArmingChecks check, bool report, const char *fmt, ...) const
{
    if (!report) {
        return;
    }
    char taggedfmt[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];

    // metafmt 包装传入的格式字符串,根据当前检查类型添加"PreArm"或"Arm"前缀
    const char *metafmt = "PreArm: %s";  // 格式化一直向下传递
    if (running_arming_checks) {
        metafmt = "Arm: %s";
    }
    hal.util->snprintf(taggedfmt, sizeof(taggedfmt), metafmt, fmt);

#if HAL_GCS_ENABLED
    MAV_SEVERITY severity = MAV_SEVERITY_CRITICAL;
    if (!check_enabled(check)) {
        // 技术上应该是 NOTICE,但在该级别会打扰用户
        severity = MAV_SEVERITY_DEBUG;
    }
    va_list arg_list;
    va_start(arg_list, fmt);
    gcs().send_textv(severity, taggedfmt, arg_list);
    va_end(arg_list);
#endif  // HAL_GCS_ENABLED
}

/*
  检查失败时的处理,不带检查项参数
 */
void AP_Arming::check_failed(bool report, const char *fmt, ...) const
{
#if HAL_GCS_ENABLED
    if (!report) {
        return;
    }
    char taggedfmt[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];

    // metafmt 包装传入的格式字符串,根据当前检查类型添加"PreArm"或"Arm"前缀
    const char *metafmt = "PreArm: %s";  // 格式化一直向下传递
    if (running_arming_checks) {
        metafmt = "Arm: %s";
    }
    hal.util->snprintf(taggedfmt, sizeof(taggedfmt), metafmt, fmt);

    va_list arg_list;
    va_start(arg_list, fmt);
    gcs().send_textv(MAV_SEVERITY_CRITICAL, taggedfmt, arg_list);
    va_end(arg_list);
#endif  // HAL_GCS_ENABLED
}

/*
  执行气压计检查
 */
bool AP_Arming::barometer_checks(bool report)
{
#ifdef HAL_BARO_ALLOW_INIT_NO_BARO
    return true;
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (AP::sitl()->baro_count == 0) {
        // 模拟没有气压计板
        return true;
    }
#endif
    if (check_enabled(ARMING_CHECK_BARO)) {
        char buffer[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1] {};
        if (!AP::baro().arming_checks(sizeof(buffer), buffer)) {
            check_failed(ARMING_CHECK_BARO, report, "Baro: %s", buffer);
            return false;
        }
    }

    return true;
}

#if AP_AIRSPEED_ENABLED
/*
  执行空速计检查
 */
bool AP_Arming::airspeed_checks(bool report)
{
    if (check_enabled(ARMING_CHECK_AIRSPEED)) {
        const AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
        if (airspeed == nullptr) {
            // 不是具有空速计功能的飞行器
            return true;
        }
        for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
            if (airspeed->enabled(i) && airspeed->use(i) && !airspeed->healthy(i)) {
                check_failed(ARMING_CHECK_AIRSPEED, report, "Airspeed %d not healthy", i + 1);
                return false;
            }
        }
    }

    return true;
}
#endif  // AP_AIRSPEED_ENABLED

#if HAL_LOGGING_ENABLED
/*
  执行日志记录检查
 */
bool AP_Arming::logging_checks(bool report)
{
    if (check_enabled(ARMING_CHECK_LOGGING)) {
        if (!AP::logger().logging_present()) {
            // 日志记录已禁用,无需检查
            return true;
        }
        if (AP::logger().logging_failed()) {
            check_failed(ARMING_CHECK_LOGGING, report, "Logging failed");
            return false;
        }
        if (!AP::logger().CardInserted()) {
            check_failed(ARMING_CHECK_LOGGING, report, "No SD card");
            return false;
        }
        if (AP::logger().in_log_download()) {
            check_failed(ARMING_CHECK_LOGGING, report, "Downloading logs");
            return false;
        }
    }
    return true;
}
#endif  // HAL_LOGGING_ENABLED

#if AP_INERTIALSENSOR_ENABLED
/*
  检查加速度计数据是否一致
 */
bool AP_Arming::ins_accels_consistent(const AP_InertialSensor &ins)
{
    const uint32_t now = AP_HAL::millis();
    if (!ins.accels_consistent(accel_error_threshold)) {
        // 加速度计数据不一致
        last_accel_pass_ms = 0;
        return false;
    }

    if (last_accel_pass_ms == 0) {
        // 上面没有返回 false,说明传感器当前是一致的
        last_accel_pass_ms = now;
    }

    // 如果加速度计理论上可能不一致,
    // 必须连续通过至少10秒才能被认为是一致的:
    if (ins.get_accel_count() > 1 && now - last_accel_pass_ms < 10000) {
        return false;
    }

    return true;
}

// 检查陀螺仪数据是否一致
bool AP_Arming::ins_gyros_consistent(const AP_InertialSensor &ins)
{
    const uint32_t now = AP_HAL::millis();
    // 允许最多5度/秒的差异
    if (!ins.gyros_consistent(5)) {
        // 陀螺仪数据不一致:
        last_gyro_pass_ms = 0;
        return false;
    }

    // 上面没有返回false,说明传感器当前是一致的:
    if (last_gyro_pass_ms == 0) {
        last_gyro_pass_ms = now;
    }

    // 如果陀螺仪理论上可能不一致,
    // 必须连续通过至少10秒才能被认为是一致的:
    if (ins.get_gyro_count() > 1 && now - last_gyro_pass_ms < 10000) {
        return false;
    }

    return true;
}

// 执行IMU检查
bool AP_Arming::ins_checks(bool report)
{
    if (check_enabled(ARMING_CHECK_INS)) {
        const AP_InertialSensor &ins = AP::ins();
        if (!ins.get_gyro_health_all()) {
            check_failed(ARMING_CHECK_INS, report, "Gyros not healthy");
            return false;
        }
        if (!ins.gyro_calibrated_ok_all()) {
            check_failed(ARMING_CHECK_INS, report, "Gyros not calibrated");
            return false;
        }
        if (!ins.get_accel_health_all()) {
            check_failed(ARMING_CHECK_INS, report, "Accels not healthy");
            return false;
        }
        if (!ins.accel_calibrated_ok_all()) {
            check_failed(ARMING_CHECK_INS, report, "3D Accel calibration needed");
            return false;
        }
        
        // 检查加速度计是否已校准并需要重启
        if (ins.accel_cal_requires_reboot()) {
            check_failed(ARMING_CHECK_INS, report, "Accels calibrated requires reboot");
            return false;
        }

        // 检查所有加速度计是否指向大致相同的方向
        if (!ins_accels_consistent(ins)) {
            check_failed(ARMING_CHECK_INS, report, "Accels inconsistent");
            return false;
        }

        // 检查所有陀螺仪是否给出一致的读数
        if (!ins_gyros_consistent(ins)) {
            check_failed(ARMING_CHECK_INS, report, "Gyros inconsistent");
            return false;
        }

        // 温度校准运行时不允许解锁
        if (ins.temperature_cal_running()) {
            check_failed(ARMING_CHECK_INS, report, "temperature cal running");
            return false;
        }

#if AP_INERTIALSENSOR_BATCHSAMPLER_ENABLED
        // 如果启用了批量采样,必须先初始化
        if (ins.batchsampler.enabled() && !ins.batchsampler.is_initialised()) {
            check_failed(ARMING_CHECK_INS, report, "Batch sampling requires reboot");
            return false;
        }
#endif

    }

#if HAL_GYROFFT_ENABLED
    // 陀螺仪正常后检查FFT
    if (check_enabled(ARMING_CHECK_FFT)) {
        // 检查噪声分析器是否工作正常
        AP_GyroFFT *fft = AP::fft();

        char fail_msg[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
        if (fft != nullptr && !fft->pre_arm_check(fail_msg, ARRAY_SIZE(fail_msg))) {
            check_failed(ARMING_CHECK_INS, report, "%s", fail_msg);
            return false;
        }
    }
#endif

    return true;
}
#endif // AP_INERTIALSENSOR_ENABLED

// 执行罗盘检查
bool AP_Arming::compass_checks(bool report)
{
    Compass &_compass = AP::compass();

#if COMPASS_CAL_ENABLED
    // 检查罗盘是否正在校准
    if (_compass.is_calibrating()) {
        check_failed(report, "Compass calibration running");
        return false;
    }

    // 检查罗盘是否已校准并需要重启
    if (_compass.compass_cal_requires_reboot()) {
        check_failed(report, "Compass calibrated requires reboot");
        return false;
    }
#endif

    if (check_enabled(ARMING_CHECK_COMPASS)) {

        // 避免使用Compass::use_for_yaw(void),因为它会隐式调用healthy()
        // 这可能会错误地跳过剩余检查,直接传递主实例
        if (!_compass.use_for_yaw(0)) {
            // 罗盘使用已禁用
            return true;
        }

        if (!_compass.healthy()) {
            check_failed(ARMING_CHECK_COMPASS, report, "Compass not healthy");
            return false;
        }
        // 检查罗盘学习是否开启或偏移量已设置
#if !APM_BUILD_COPTER_OR_HELI && !APM_BUILD_TYPE(APM_BUILD_Blimp)
        // 如果学习关闭则检查罗盘偏移量是否已设置
        // 直升机和飞艇始终需要配置罗盘
        if (!_compass.learn_offsets_enabled())
#endif
        {
            char failure_msg[100] = {};
            if (!_compass.configured(failure_msg, ARRAY_SIZE(failure_msg))) {
                check_failed(ARMING_CHECK_COMPASS, report, "%s", failure_msg);
                return false;
            }
        }

        // 检查罗盘偏移量是否不合理
        const Vector3f offsets = _compass.get_offsets();
        if (offsets.length() > _compass.get_offsets_max()) {
            check_failed(ARMING_CHECK_COMPASS, report, "Compass offsets too high");
            return false;
        }

        // 检查磁场强度是否不合理
        const float mag_field = _compass.get_field().length();
        if (mag_field > AP_ARMING_COMPASS_MAGFIELD_MAX || mag_field < AP_ARMING_COMPASS_MAGFIELD_MIN) {
            check_failed(ARMING_CHECK_COMPASS, report, "Check mag field: %4.0f, max %d, min %d", (double)mag_field, AP_ARMING_COMPASS_MAGFIELD_MAX, AP_ARMING_COMPASS_MAGFIELD_MIN);
            return false;
        }

        // 检查所有罗盘是否指向大致相同的方向
        if (!_compass.consistent()) {
            check_failed(ARMING_CHECK_COMPASS, report, "Compasses inconsistent");
            return false;
        }

#if AP_AHRS_ENABLED
        // 如果AHRS使用罗盘且我们有位置,检查磁场与预期的地球磁场模型是否一致
        Location ahrs_loc;
        AP_AHRS &ahrs = AP::ahrs();
        if ((magfield_error_threshold > 0) && ahrs.use_compass() && ahrs.get_location(ahrs_loc)) {
            const Vector3f veh_mag_field_ef = ahrs.get_rotation_body_to_ned() * _compass.get_field();
            const Vector3f earth_field_mgauss = AP_Declination::get_earth_field_ga(ahrs_loc) * 1000.0;
            const Vector3f diff_mgauss = veh_mag_field_ef - earth_field_mgauss;
            if (MAX(fabsf(diff_mgauss.x), fabsf(diff_mgauss.y)) > magfield_error_threshold) {
                check_failed(ARMING_CHECK_COMPASS, report, "Check mag field (xy diff:%.0f>%d)",
                             (double)MAX(fabsf(diff_mgauss.x), (double)fabsf(diff_mgauss.y)), (int)magfield_error_threshold);
                return false;
            }
            if (fabsf(diff_mgauss.x) > magfield_error_threshold*2.0) {
                check_failed(ARMING_CHECK_COMPASS, report, "Check mag field (z diff:%.0f>%d)",
                             (double)fabsf(diff_mgauss.z), (int)magfield_error_threshold*2);
                return false;
            }           
        }
#endif  // AP_AHRS_ENABLED
    }

    return true;
}

#if AP_GPS_ENABLED
// 执行GPS检查
bool AP_Arming::gps_checks(bool report)
{
    const AP_GPS &gps = AP::gps();
    if (check_enabled(ARMING_CHECK_GPS)) {

        // GPS后端的任何失败消息
        char failure_msg[100] = {};
        if (!AP::gps().pre_arm_checks(failure_msg, ARRAY_SIZE(failure_msg))) {
            if (failure_msg[0] != '\0') {
                check_failed(ARMING_CHECK_GPS, report, "%s", failure_msg);
            }
            return false;
        }

        for (uint8_t i = 0; i < gps.num_sensors(); i++) {
#if AP_GPS_BLENDED_ENABLED
            if ((i != GPS_BLENDED_INSTANCE) &&
#else
            if (
#endif
                    (gps.get_type(i) == AP_GPS::GPS_Type::GPS_TYPE_NONE)) {
                if (gps.primary_sensor() == i) {
                    check_failed(ARMING_CHECK_GPS, report, "GPS %i: primary but TYPE 0", i+1);
                    return false;
                }
                continue;
            }

            // GPS状态正常?
            if (gps.status(i) < AP_GPS::GPS_OK_FIX_3D) {
                check_failed(ARMING_CHECK_GPS, report, "GPS %i: Bad fix", i+1);
                return false;
            }

            // GPS更新率可接受
            if (!gps.is_healthy(i)) {
                check_failed(ARMING_CHECK_GPS, report, "GPS %i: not healthy", i+1);
                return false;
            }
        }

        if (!AP::ahrs().home_is_set()) {
            check_failed(ARMING_CHECK_GPS, report, "AHRS: waiting for home");
            return false;
        }

        // 检查GPS之间的距离是否在50m以内,且混合状态正常
        float distance_m;
        if (!gps.all_consistent(distance_m)) {
            check_failed(ARMING_CHECK_GPS, report, "GPS positions differ by %4.1fm",
                         (double)distance_m);
            return false;
        }

        // 检查AHRS和GPS之间的距离是否在10m以内
        if (gps.num_sensors() > 0) {
            const Location gps_loc = gps.location();
            Location ahrs_loc;
            if (AP::ahrs().get_location(ahrs_loc)) {
                const float distance = gps_loc.get_distance(ahrs_loc);
                if (distance > AP_ARMING_AHRS_GPS_ERROR_MAX) {
                    check_failed(ARMING_CHECK_GPS, report, "GPS and AHRS differ by %4.1fm", (double)distance);
                    return false;
                }
            }
        }
    }

    if (check_enabled(ARMING_CHECK_GPS_CONFIG)) {
        uint8_t first_unconfigured;
        if (gps.first_unconfigured_gps(first_unconfigured)) {
            check_failed(ARMING_CHECK_GPS_CONFIG,
                         report,
                         "GPS %d still configuring this GPS",
                         first_unconfigured + 1);
            if (report) {
                gps.broadcast_first_configuration_failure_reason();
            }
            return false;
        }
    }

    return true;
}
#endif  // AP_GPS_ENABLED

#if AP_BATTERY_ENABLED
// 执行电池检查
bool AP_Arming::battery_checks(bool report)
{
    if (check_enabled(ARMING_CHECK_BATTERY)) {

        char buffer[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1] {};
        if (!AP::battery().arming_checks(sizeof(buffer), buffer)) {
            check_failed(ARMING_CHECK_BATTERY, report, "%s", buffer);
            return false;
        }
     }
    return true;
}
#endif  // AP_BATTERY_ENABLED

// 检查硬件安全开关状态
bool AP_Arming::hardware_safety_check(bool report) 
{
    if (check_enabled(ARMING_CHECK_SWITCH)) {

      // 检查安全开关是否已按下
      if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
          check_failed(ARMING_CHECK_SWITCH, report, "Hardware safety switch");
          return false;
      }
    }

    return true;
}

#if AP_RC_CHANNEL_ENABLED
// 执行遥控器解锁检查
bool AP_Arming::rc_arm_checks(AP_Arming::Method method)
{
    // 如果处于故障保护状态,不检查微调
    if (!rc().has_valid_input()) {
        return true;
    }

    // 仅在最近1秒内收到输入时才检查
    // 这是为了防止从未启用输入的车辆
    uint32_t last_input_ms = rc().last_input_ms();
    if ((last_input_ms == 0) || ((AP_HAL::millis() - last_input_ms) > 1000)) {
        return true;
    }

    bool check_passed = true;
    // 确保所有遥控通道具有不同的功能
    if (rc().duplicate_options_exist()) {
        check_failed(ARMING_CHECK_PARAMETERS, true, "Duplicate Aux Switch Options");
        check_passed = false;
    }
    // 检查飞行模式通道是否与RC选项冲突
    if (rc().flight_mode_channel_conflicts_with_rc_option()) {
        check_failed(ARMING_CHECK_PARAMETERS, true, "Mode channel and RC%d_OPTION conflict", rc().flight_mode_channel_number());
        check_passed = false;
    }
    {
        // 如果未启用跳过RPY检查选项
        if (!rc().option_is_enabled(RC_Channels::Option::ARMING_SKIP_CHECK_RPY)) {
            const struct {
                const char *name;
                const RC_Channel *channel;
            } channels_to_check[] {
                { "Roll", &rc().get_roll_channel(), },    // 横滚通道
                { "Pitch", &rc().get_pitch_channel(), },  // 俯仰通道
                { "Yaw", &rc().get_yaw_channel(), },      // 偏航通道
            };
            // 检查每个通道的中位值
            for (const auto &channel_to_check : channels_to_check) {
                const auto *c = channel_to_check.channel;
                if (c->get_control_in() != 0) {
                    if ((method != Method::RUDDER) || (c != rc().get_arming_channel())) { // 如果是方向舵解锁则忽略偏航输入通道
                        check_failed(ARMING_CHECK_RC, true, "%s (RC%d) is not neutral", channel_to_check.name, c->ch());
                        check_passed = false;
                    }
                }
            }
        }

        // 如果启用了油门检查,要求输入为零
        if (rc().arming_check_throttle()) {
            const RC_Channel *c = &rc().get_throttle_channel();
                if (c->get_control_in() != 0) {
                    check_failed(ARMING_CHECK_RC, true, "%s (RC%d) is not neutral", "Throttle", c->ch());
                    check_passed = false;
                }
            // 检查前向推力通道
            c = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FWD_THR);
            if (c != nullptr) {
                uint8_t fwd_thr = c->percent_input();
                // 要求通道输入在最小值2%范围内
                if (fwd_thr > 2) {
                    check_failed(ARMING_CHECK_RC, true, "VTOL Fwd Throttle is not zero");
                    check_passed = false;
                }
            }
        }
    }
    return check_passed;
}

// 执行遥控器校准检查
bool AP_Arming::rc_calibration_checks(bool report)
{
    bool check_passed = true;
    const uint8_t num_channels = RC_Channels::get_valid_channel_count();
    // 检查每个通道的校准值
    for (uint8_t i = 0; i < NUM_RC_CHANNELS; i++) {
        const RC_Channel *c = rc().channel(i);
        if (c == nullptr) {
            continue;
        }
        if (i >= num_channels && !(c->has_override())) {
            continue;
        }
        const uint16_t trim = c->get_radio_trim();
        // 检查最小值是否小于中位值
        if (c->get_radio_min() > trim) {
            check_failed(ARMING_CHECK_RC, report, "RC%d_MIN is greater than RC%d_TRIM", i + 1, i + 1);
            check_passed = false;
        }
        // 检查最大值是否大于中位值
        if (c->get_radio_max() < trim) {
            check_failed(ARMING_CHECK_RC, report, "RC%d_MAX is less than RC%d_TRIM", i + 1, i + 1);
            check_passed = false;
        }
    }

    return check_passed;
}

// 检查遥控器是否正在校准
bool AP_Arming::rc_in_calibration_check(bool report)
{
    if (rc().calibrating()) {
        check_failed(ARMING_CHECK_RC, report, "RC calibrating");
        return false;
    }
    return true;
}

// 执行手动发射器检查
bool AP_Arming::manual_transmitter_checks(bool report)
{
    if (check_enabled(ARMING_CHECK_RC)) {

        // 检查遥控器故障保护
        if (AP_Notify::flags.failsafe_radio) {
            check_failed(ARMING_CHECK_RC, report, "Radio failsafe on");
            return false;
        }

        // 执行遥控器校准检查
        if (!rc_calibration_checks(report)) {
            return false;
        }
    }

    return rc_in_calibration_check(report);
}
#endif  // AP_RC_CHANNEL_ENABLED

#if AP_MISSION_ENABLED
// 执行任务检查
bool AP_Arming::mission_checks(bool report)
{
    AP_Mission *mission = AP::mission();
    if (check_enabled(ARMING_CHECK_MISSION) && _required_mission_items) {
        if (mission == nullptr) {
            check_failed(ARMING_CHECK_MISSION, report, "No mission library present");
            return false;
        }

        // 定义需要检查的任务项类型
        const struct MisItemTable {
          MIS_ITEM_CHECK check;
          MAV_CMD mis_item_type;
          const char *type;
        } misChecks[] = {
          {MIS_ITEM_CHECK_LAND,          MAV_CMD_NAV_LAND,           "land"},           // 着陆
          {MIS_ITEM_CHECK_VTOL_LAND,     MAV_CMD_NAV_VTOL_LAND,      "vtol land"},      // 垂直起降着陆
          {MIS_ITEM_CHECK_DO_LAND_START, MAV_CMD_DO_LAND_START,      "do land start"},   // 开始着陆
          {MIS_ITEM_CHECK_TAKEOFF,       MAV_CMD_NAV_TAKEOFF,        "takeoff"},         // 起飞
          {MIS_ITEM_CHECK_VTOL_TAKEOFF,  MAV_CMD_NAV_VTOL_TAKEOFF,   "vtol takeoff"},    // 垂直起降起飞
          {MIS_ITEM_CHECK_RETURN_TO_LAUNCH,  MAV_CMD_NAV_RETURN_TO_LAUNCH,   "RTL"},     // 返航
        };
        // 检查每个必需的任务项
        for (uint8_t i = 0; i < ARRAY_SIZE(misChecks); i++) {
            if (_required_mission_items & misChecks[i].check) {
                if (!mission->contains_item(misChecks[i].mis_item_type)) {
                    check_failed(ARMING_CHECK_MISSION, report, "Missing mission item: %s", misChecks[i].type);
                    return false;
                }
            }
        }
        // 检查集结点
        if (_required_mission_items & MIS_ITEM_CHECK_RALLY) {
#if HAL_RALLY_ENABLED
            AP_Rally *rally = AP::rally();
            if (rally == nullptr) {
                check_failed(ARMING_CHECK_MISSION, report, "No rally library present");
                return false;
            }
            Location ahrs_loc;
            if (!AP::ahrs().get_location(ahrs_loc)) {
                check_failed(ARMING_CHECK_MISSION, report, "Can't check rally without position");
                return false;
            }
            RallyLocation rally_loc = {};
            if (!rally->find_nearest_rally_point(ahrs_loc, rally_loc)) {
                check_failed(ARMING_CHECK_MISSION, report, "No sufficiently close rally point located");
                return false;
            }
#else
            check_failed(ARMING_CHECK_MISSION, report, "No rally library present");
            return false;
#endif
        }
    }

#if AP_SDCARD_STORAGE_ENABLED
    // 检查SD卡存储
    if (check_enabled(ARMING_CHECK_MISSION) &&
        mission != nullptr &&
        (mission->failed_sdcard_storage() || StorageManager::storage_failed())) {
        check_failed(ARMING_CHECK_MISSION, report, "Failed to open %s", AP_MISSION_SDCARD_FILENAME);
        return false;
    }
#endif

#if AP_VEHICLE_ENABLED
    // 如果当前模式需要任务但没有任务项,不允许解锁
    if (AP::vehicle()->current_mode_requires_mission() &&
        (mission == nullptr || mission->num_commands() <= 1)) {
        check_failed(ARMING_CHECK_MISSION, report, "Mode requires mission");
        return false;
    }
#endif

    return true;
}
#endif  // AP_MISSION_ENABLED

// 执行测距仪检查
bool AP_Arming::rangefinder_checks(bool report)
{
#if AP_RANGEFINDER_ENABLED
    if (check_enabled(ARMING_CHECK_RANGEFINDER)) {
        RangeFinder *range = RangeFinder::get_singleton();
        if (range == nullptr) {
            return true;
        }

        char buffer[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
        if (!range->prearm_healthy(buffer, ARRAY_SIZE(buffer))) {
            check_failed(ARMING_CHECK_RANGEFINDER, report, "%s", buffer);
            return false;
        }
    }
#endif

    return true;
}

// 执行舵机检查
bool AP_Arming::servo_checks(bool report) const
{
#if NUM_SERVO_CHANNELS
    bool check_passed = true;
    // 遍历所有舵机通道
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        const SRV_Channel *c = SRV_Channels::srv_channel(i);
        // 跳过未使用的通道
        if (c == nullptr || c->get_function() <= SRV_Channel::k_none) {
            continue;
        }

        // 获取舵机中点值
        const uint16_t trim = c->get_trim();
        // 检查最小值是否大于中点值
        if (c->get_output_min() > trim) {
            check_failed(report, "SERVO%d_MIN is greater than SERVO%d_TRIM", i + 1, i + 1);
            check_passed = false;
        }
        // 检查最大值是否小于中点值
        if (c->get_output_max() < trim) {
            check_failed(report, "SERVO%d_MAX is less than SERVO%d_TRIM", i + 1, i + 1);
            check_passed = false;
        }

        // 检查使用PWM的功能是否启用
        if (SRV_Channels::get_disabled_channel_mask() & 1U<<i) {
            const SRV_Channel::Aux_servo_function_t ch_function = c->get_function();

            // 以下功能可以使用数字输出,因此可以被禁用:
            // - 电机
            // - 可以紧急停止的功能
            // - neopixel和ProfiLED
            // - 脚本可以使用其功能作为LED设置的标签
            const bool disabled_ok = SRV_Channel::is_motor(ch_function) ||
                                     SRV_Channel::should_e_stop(ch_function) ||
                                     (ch_function >= SRV_Channel::k_LED_neopixel1 && ch_function <= SRV_Channel::k_LED_neopixel4) ||
                                     (ch_function >= SRV_Channel::k_ProfiLED_1 && ch_function <= SRV_Channel::k_ProfiLED_Clock) ||
                                     (ch_function >= SRV_Channel::k_scripting1 && ch_function <= SRV_Channel::k_scripting16);

            // 对于所有其他功能,报告预解锁检查失败
            if (!disabled_ok) {
                check_failed(report, "SERVO%u_FUNCTION=%u on disabled channel", i + 1, (unsigned)ch_function);
                check_passed = false;
            }
        }
    }

#if HAL_WITH_IO_MCU
    // 检查IO MCU是否正常
    if (!iomcu.healthy() && AP_BoardConfig::io_enabled()) {
        check_failed(report, "IOMCU is unhealthy");
        check_passed = false;
    }
#endif

    return check_passed;
#else
    return false;
#endif
}

// 执行板载电压检查
bool AP_Arming::board_voltage_checks(bool report)
{
    // 检查板载电压
    if (check_enabled(ARMING_CHECK_VOLTAGE)) {
#if HAL_HAVE_BOARD_VOLTAGE
        // 获取总线电压
        const float bus_voltage =  hal.analogin->board_voltage();
        const float vbus_min = AP_BoardConfig::get_minimum_board_voltage();
        // 检查电压是否在允许范围内
        if(((bus_voltage < vbus_min) || (bus_voltage > AP_ARMING_BOARD_VOLTAGE_MAX))) {
            check_failed(ARMING_CHECK_VOLTAGE, report, "Board (%1.1fv) out of range %1.1f-%1.1fv", (double)bus_voltage, (double)vbus_min, (double)AP_ARMING_BOARD_VOLTAGE_MAX);
            return false;
        }
#endif // HAL_HAVE_BOARD_VOLTAGE

#if HAL_HAVE_SERVO_VOLTAGE
       // 获取舵机最小电压要求
       const float vservo_min = AP_BoardConfig::get_minimum_servo_voltage();
        if (is_positive(vservo_min)) {
            // 获取舵机电压
            const float servo_voltage =  hal.analogin->servorail_voltage();
            // 检查舵机电压是否过低
            if (servo_voltage < vservo_min) {
                check_failed(ARMING_CHECK_VOLTAGE, report, "Servo voltage to low (%1.2fv < %1.2fv)", (double)servo_voltage, (double)vservo_min);
                return false;
            }
        }
#endif // HAL_HAVE_SERVO_VOLTAGE
    }

    return true;
}

#if HAL_HAVE_IMU_HEATER
// 执行加热器最低温度检查
bool AP_Arming::heater_min_temperature_checks(bool report)
{
    if (checks_to_perform & ARMING_CHECK_ALL) {
        AP_BoardConfig *board = AP::boardConfig();
        if (board) {
            float temperature;
            int8_t min_temperature;
            // 获取当前温度和最低温度要求
            if (board->get_board_heater_temperature(temperature) &&
                board->get_board_heater_arming_temperature(min_temperature) &&
                (temperature < min_temperature)) {
                check_failed(ARMING_CHECK_SYSTEM, report, "heater temp low (%0.1f < %i)", temperature, min_temperature);
                return false;
            }
        }
    }
    return true;
}
#endif // HAL_HAVE_IMU_HEATER

/*
  检查基本系统操作
 */
bool AP_Arming::system_checks(bool report)
{
    char buffer[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1] {};

    if (check_enabled(ARMING_CHECK_SYSTEM)) {
        // 检查存储是否正常
        if (!hal.storage->healthy()) {
            check_failed(ARMING_CHECK_SYSTEM, report, "Param storage failed");
            return false;
        }

        // 检查参数存储是否已满
        if (AP_Param::get_eeprom_full()) {
            check_failed(ARMING_CHECK_PARAMETERS, report, "parameter storage full");
            return false;
        }
        
        // 检查主循环速率是否至少达到预期值的90%
        const float actual_loop_rate = AP::scheduler().get_filtered_loop_rate_hz();
        const uint16_t expected_loop_rate = AP::scheduler().get_loop_rate_hz();
        const float loop_rate_pct =  actual_loop_rate / expected_loop_rate;
        if (loop_rate_pct < 0.90) {
            check_failed(ARMING_CHECK_SYSTEM, report, "Main loop slow (%uHz < %uHz)", (unsigned)actual_loop_rate, (unsigned)expected_loop_rate);
            return false;
        }

#if AP_TERRAIN_AVAILABLE
        // 检查地形数据库是否内存不足
        const AP_Terrain *terrain = AP_Terrain::get_singleton();
        if ((terrain != nullptr) && terrain->init_failed()) {
            check_failed(ARMING_CHECK_SYSTEM, report, "Terrain out of memory");
            return false;
        }
#endif
#if AP_SCRIPTING_ENABLED
        // 执行脚本预解锁检查
        const AP_Scripting *scripting = AP_Scripting::get_singleton();
        if ((scripting != nullptr) && !scripting->arming_checks(sizeof(buffer), buffer)) {
            check_failed(ARMING_CHECK_SYSTEM, report, "%s", buffer);
            return false;
        }
#endif
#if HAL_ADSB_ENABLED
        // 检查ADS-B是否内存不足
        AP_ADSB *adsb = AP::ADSB();
        if ((adsb != nullptr) && adsb->enabled() && adsb->init_failed()) {
            check_failed(ARMING_CHECK_SYSTEM, report, "ADSB out of memory");
            return false;
        }
#endif
    }
    // 检查内部错误
    if (AP::internalerror().errors() != 0) {
        AP::internalerror().errors_as_string((uint8_t*)buffer, ARRAY_SIZE(buffer));
        check_failed(report, "Internal errors 0x%x l:%u %s", (unsigned int)AP::internalerror().errors(), AP::internalerror().last_error_line(), buffer);
        return false;
    }

    // 执行GPIO预解锁检查
    if (!hal.gpio->arming_checks(sizeof(buffer), buffer)) {
        check_failed(report, "%s", buffer);
        return false;
    }

    if (check_enabled(ARMING_CHECK_PARAMETERS)) {
#if !AP_GPS_BLENDED_ENABLED
        // 执行GPS混合切换检查
        if (!blending_auto_switch_checks(report)) {
            return false;
        }
#endif
#if AP_RPM_ENABLED
        // 执行RPM预解锁检查
        auto *rpm = AP::rpm();
        if (rpm && !rpm->arming_checks(sizeof(buffer), buffer)) {
            check_failed(ARMING_CHECK_PARAMETERS, report, "%s", buffer);
            return false;
        }
#endif
#if AP_RELAY_ENABLED
        // 执行继电器预解锁检查
        auto *relay = AP::relay();
        if (relay && !relay->arming_checks(sizeof(buffer), buffer)) {
            check_failed(ARMING_CHECK_PARAMETERS, report, "%s", buffer);
            return false;
        }
#endif
#if HAL_PARACHUTE_ENABLED
        // 执行降落伞预解锁检查
        auto *chute = AP::parachute();
        if (chute && !chute->arming_checks(sizeof(buffer), buffer)) {
            check_failed(ARMING_CHECK_PARAMETERS, report, "%s", buffer);
            return false;
        }
#endif
#if HAL_BUTTON_ENABLED
        // 执行按钮预解锁检查
        const auto &button = AP::button();
        if (!button.arming_checks(sizeof(buffer), buffer)) {
            check_failed(ARMING_CHECK_PARAMETERS, report, "%s", buffer);
            return false;
        }
#endif
    }

    return true;
}

// 检查是否需要地形数据库
bool AP_Arming::terrain_database_required() const
{
#if AP_MISSION_ENABLED
    AP_Mission *mission = AP::mission();
    if (mission == nullptr) {
        // 没有任务支持
        return false;
    }
    // 检查任务中是否包含地形高度项
    if (mission->contains_terrain_alt_items()) {
        return true;
    }
#endif
    return false;
}

// 检查地形数据库是否可用
bool AP_Arming::terrain_checks(bool report) const
{
    if (!check_enabled(ARMING_CHECK_PARAMETERS)) {
        return true;
    }

    // 如果不需要地形数据库则返回true
    if (!terrain_database_required()) {
        return true;
    }

#if AP_TERRAIN_AVAILABLE

    const AP_Terrain *terrain = AP_Terrain::get_singleton();
    if (terrain == nullptr) {
        // 这也是一个系统错误,已经报告过了
        return false;
    }

    // 检查地形功能是否启用
    if (!terrain->enabled()) {
        check_failed(ARMING_CHECK_PARAMETERS, report, "terrain disabled");
        return false;
    }

    // 执行地形预解锁检查
    char fail_msg[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
    if (!terrain->pre_arm_checks(fail_msg, sizeof(fail_msg))) {
        check_failed(ARMING_CHECK_PARAMETERS, report, "%s", fail_msg);
        return false;
    }

    return true;

#else
    check_failed(ARMING_CHECK_PARAMETERS, report, "terrain required but disabled");
    return false;
#endif
}


#if HAL_PROXIMITY_ENABLED
// 检查是否有物体太靠近飞行器
bool AP_Arming::proximity_checks(bool report) const
{
    const AP_Proximity *proximity = AP::proximity();
    // 如果没有传感器则直接返回true
    if (proximity == nullptr) {
        return true;
    }
    // 执行接近传感器预解锁检查
    char buffer[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
    if (!proximity->prearm_healthy(buffer, ARRAY_SIZE(buffer))) {
        check_failed(report, "%s", buffer);
        return false;
    }
    return true;
}
#endif  // HAL_PROXIMITY_ENABLED

#if HAL_MAX_CAN_PROTOCOL_DRIVERS && HAL_CANMANAGER_ENABLED
// 执行CAN总线检查
bool AP_Arming::can_checks(bool report)
{
    if (check_enabled(ARMING_CHECK_SYSTEM)) {
        char fail_msg[100] = {};
        (void)fail_msg; // 可能未使用
        // 获取CAN驱动数量
        uint8_t num_drivers = AP::can().get_num_drivers();

        // 遍历所有CAN驱动
        for (uint8_t i = 0; i < num_drivers; i++) {
            switch (AP::can().get_driver_type(i)) {
                case AP_CAN::Protocol::PiccoloCAN: {
#if HAL_PICCOLO_CAN_ENABLE
                    // 执行PiccoloCAN预解锁检查
                    AP_PiccoloCAN *ap_pcan = AP_PiccoloCAN::get_pcan(i);

                    if (ap_pcan != nullptr && !ap_pcan->pre_arm_check(fail_msg, ARRAY_SIZE(fail_msg))) {
                        check_failed(ARMING_CHECK_SYSTEM, report, "PiccoloCAN: %s", fail_msg);
                        return false;
                    }

#else
                    check_failed(ARMING_CHECK_SYSTEM, report, "PiccoloCAN not enabled");
                    return false;
#endif
                    break;
                }
                case AP_CAN::Protocol::DroneCAN:
                {
#if HAL_ENABLE_DRONECAN_DRIVERS
                    // 执行DroneCAN预解锁检查
                    AP_DroneCAN *ap_dronecan = AP_DroneCAN::get_dronecan(i);
                    if (ap_dronecan != nullptr && !ap_dronecan->prearm_check(fail_msg, ARRAY_SIZE(fail_msg))) {
                        check_failed(ARMING_CHECK_SYSTEM, report, "DroneCAN: %s", fail_msg);
                        return false;
                    }
#endif
                    break;
                }
                case AP_CAN::Protocol::USD1:
                case AP_CAN::Protocol::TOFSenseP:
                case AP_CAN::Protocol::NanoRadar:
                case AP_CAN::Protocol::Benewake:
                {
                    // 检查相同的测距仪是否在不同的CAN端口上
                    for (uint8_t j = i; j; j--) {
                        if (AP::can().get_driver_type(i) == AP::can().get_driver_type(j-1)) {
                            check_failed(ARMING_CHECK_SYSTEM, report, "Same rfnd on different CAN ports");
                            return false;
                        }
                    }
                    break;
                }
                case AP_CAN::Protocol::EFI_NWPMU:
                case AP_CAN::Protocol::None:
                case AP_CAN::Protocol::Scripting:
                case AP_CAN::Protocol::Scripting2:
                case AP_CAN::Protocol::KDECAN:

                    break;
            }
        }
    }
    return true;
}
#endif  // HAL_MAX_CAN_PROTOCOL_DRIVERS && HAL_CANMANAGER_ENABLED


#if AP_FENCE_ENABLED
// 执行地理围栏检查
bool AP_Arming::fence_checks(bool display_failure)
{
    const AC_Fence *fence = AP::fence();
    if (fence == nullptr) {
        return true;
    }

    // 检查围栏是否就绪
    char fail_msg[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
    if (fence->pre_arm_check(fail_msg, ARRAY_SIZE(fail_msg))) {
        return true;
    }

    check_failed(display_failure, "%s", fail_msg);

#if AP_SDCARD_STORAGE_ENABLED
    // 检查SD卡存储是否失败
    if (fence->failed_sdcard_storage() || StorageManager::storage_failed()) {
        check_failed(display_failure, "Failed to open fence storage");
        return false;
    }
#endif
    
    return false;
}
#endif  // AP_FENCE_ENABLED

#if HAL_RUNCAM_ENABLED
// 执行相机检查
bool AP_Arming::camera_checks(bool display_failure)
{
    // 如果启用了相机检查
    if (check_enabled(ARMING_CHECK_CAMERA)) {
        AP_RunCam *runcam = AP::runcam();
        // 如果没有RunCam对象则通过检查
        if (runcam == nullptr) {
            return true;
        }

        // 检查相机是否就绪
        char fail_msg[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
        if (!runcam->pre_arm_check(fail_msg, ARRAY_SIZE(fail_msg))) {
            check_failed(ARMING_CHECK_CAMERA, display_failure, "%s", fail_msg);
            return false;
        }
    }
    return true;
}
#endif  // HAL_RUNCAM_ENABLED

#if OSD_ENABLED
// 执行OSD检查
bool AP_Arming::osd_checks(bool display_failure) const
{
    // 如果启用了OSD检查
    if (check_enabled(ARMING_CHECK_OSD)) {
        // 如果没有OSD对象则通过检查
        const AP_OSD *osd = AP::osd();
        if (osd == nullptr) {
            return true;
        }
        // 执行OSD配置检查
        char fail_msg[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
        if (!osd->pre_arm_check(fail_msg, ARRAY_SIZE(fail_msg))) {
            check_failed(ARMING_CHECK_OSD, display_failure, "%s", fail_msg);
            return false;
        }
   }
    return true;
}
#endif  // OSD_ENABLED

#if HAL_MOUNT_ENABLED
// 执行云台检查
bool AP_Arming::mount_checks(bool display_failure) const
{
    // 如果启用了相机检查
    if (check_enabled(ARMING_CHECK_CAMERA)) {
        AP_Mount *mount = AP::mount();
        // 如果没有云台对象则通过检查
        if (mount == nullptr) {
            return true;
        }
        char fail_msg[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1] = {};
        // 执行云台预解锁检查
        if (!mount->pre_arm_checks(fail_msg, sizeof(fail_msg))) {
            check_failed(ARMING_CHECK_CAMERA, display_failure, "Mount: %s", fail_msg);
            return false;
        }
    }
    return true;
}
#endif  // HAL_MOUNT_ENABLED

#if AP_FETTEC_ONEWIRE_ENABLED
// 执行FETtec电调检查
bool AP_Arming::fettec_checks(bool display_failure) const
{
    const AP_FETtecOneWire *f = AP_FETtecOneWire::get_singleton();
    // 如果没有FETtec对象则通过检查
    if (f == nullptr) {
        return true;
    }

    // 检查电调是否就绪
    char fail_msg[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
    if (!f->pre_arm_check(fail_msg, ARRAY_SIZE(fail_msg))) {
        check_failed(ARMING_CHECK_ALL, display_failure, "FETtec: %s", fail_msg);
        return false;
    }
    return true;
}
#endif  // AP_FETTEC_ONEWIRE_ENABLED

#if AP_ARMING_AUX_AUTH_ENABLED
// 请求一个辅助授权ID。此ID应在后续调用set_aux_auth_passed/failed时使用
// 成功返回true
bool AP_Arming::get_aux_auth_id(uint8_t& auth_id)
{
    WITH_SEMAPHORE(aux_auth_sem);

    // 检查是否有足够空间分配新ID
    if (aux_auth_count >= aux_auth_count_max) {
        aux_auth_error = true;
        return false;
    }

    // 为失败消息分配缓冲区
    if (aux_auth_fail_msg == nullptr) {
        aux_auth_fail_msg = (char *)calloc(aux_auth_str_len, sizeof(char));
        if (aux_auth_fail_msg == nullptr) {
            aux_auth_error = true;
            return false;
        }
    }
    auth_id = aux_auth_count;
    aux_auth_count++;
    return true;
}

// 设置辅助授权通过
void AP_Arming::set_aux_auth_passed(uint8_t auth_id)
{
    WITH_SEMAPHORE(aux_auth_sem);

    // 检查auth_id是否有效
    if (auth_id >= aux_auth_count) {
        return;
    }

    aux_auth_state[auth_id] = AuxAuthStates::AUTH_PASSED;
}

// 设置辅助授权失败并提供失败消息
void AP_Arming::set_aux_auth_failed(uint8_t auth_id, const char* fail_msg)
{
    WITH_SEMAPHORE(aux_auth_sem);

    // 检查auth_id是否有效
    if (auth_id >= aux_auth_count) {
        return;
    }

    // 更新状态
    aux_auth_state[auth_id] = AuxAuthStates::AUTH_FAILED;

    // 如果此授权者具有最低的auth_id,则存储失败消息
    for (uint8_t i = 0; i < auth_id; i++) {
        if (aux_auth_state[i] == AuxAuthStates::AUTH_FAILED) {
            return;
        }
    }
    if (aux_auth_fail_msg != nullptr) {
        if (fail_msg == nullptr) {
            strncpy(aux_auth_fail_msg, "Auxiliary authorisation refused", aux_auth_str_len);
        } else {
            strncpy(aux_auth_fail_msg, fail_msg, aux_auth_str_len);
        }
        aux_auth_fail_msg_source = auth_id;
    }
}

// 执行辅助授权检查
bool AP_Arming::aux_auth_checks(bool display_failure)
{
    // 处理错误情况
    if (aux_auth_error) {
        if (aux_auth_fail_msg == nullptr) {
            check_failed(ARMING_CHECK_AUX_AUTH, display_failure, "memory low for auxiliary authorisation");
        } else {
            check_failed(ARMING_CHECK_AUX_AUTH, display_failure, "Too many auxiliary authorisers");
        }
        return false;
    }

    WITH_SEMAPHORE(aux_auth_sem);

    // 检查每个辅助授权ID的结果
    bool some_failures = false;
    bool failure_msg_sent = false;
    bool waiting_for_responses = false;
    for (uint8_t i = 0; i < aux_auth_count; i++) {
        switch (aux_auth_state[i]) {
        case AuxAuthStates::NO_RESPONSE:
            waiting_for_responses = true;
            break;
        case AuxAuthStates::AUTH_FAILED:
            some_failures = true;
            if (i == aux_auth_fail_msg_source) {
                check_failed(ARMING_CHECK_AUX_AUTH, display_failure, "%s", aux_auth_fail_msg);
                failure_msg_sent = true;
            }
            break;
        case AuxAuthStates::AUTH_PASSED:
            break;
        }
    }

    // 发送失败或等待消息
    if (some_failures) {
        if (!failure_msg_sent) {
            check_failed(ARMING_CHECK_AUX_AUTH, display_failure, "Auxiliary authorisation refused");
        }
        return false;
    } else if (waiting_for_responses) {
        check_failed(ARMING_CHECK_AUX_AUTH, display_failure, "Waiting for auxiliary authorisation");
        return false;
    }

    // 如果执行到这里说明所有辅助检查都通过了
    return true;
}
#endif  // AP_ARMING_AUX_AUTH_ENABLED

#if HAL_GENERATOR_ENABLED
// 执行发电机检查
bool AP_Arming::generator_checks(bool display_failure) const
{
    const AP_Generator *generator = AP::generator();
    if (generator == nullptr) {
        return true;
    }
    char failure_msg[100] = {};
    if (!generator->pre_arm_check(failure_msg, sizeof(failure_msg))) {
        check_failed(display_failure, "Generator: %s", failure_msg);
        return false;
    }
    return true;
}
#endif  // HAL_GENERATOR_ENABLED

#if AP_OPENDRONEID_ENABLED
// 执行OpenDroneID检查
bool AP_Arming::opendroneid_checks(bool display_failure)
{
    auto &opendroneid = AP::opendroneid();

    char failure_msg[100] {};
    if (!opendroneid.pre_arm_check(failure_msg, sizeof(failure_msg))) {
        check_failed(display_failure, "OpenDroneID: %s", failure_msg);
        return false;
    }
    return true;
}
#endif  // AP_OPENDRONEID_ENABLED

// 检查是否有多个串口配置为RC输入
bool AP_Arming::serial_protocol_checks(bool display_failure)
{
    if (AP::serialmanager().have_serial(AP_SerialManager::SerialProtocol_RCIN, 1)) {
       check_failed(display_failure, "Multiple SERIAL ports configured for RC input");
       return false;
    }
    return true;
}

// 检查紧急停止状态
bool AP_Arming::estop_checks(bool display_failure)
{
    // 如果没有处于紧急停止状态,则通过检查
    if (!SRV_Channels::get_emergency_stop()) {
       return true;
    }
#if AP_RC_CHANNEL_ENABLED
    // 如果通过开关触发了紧急停止,则不会导致预解锁检查失败
    const RC_Channel *chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::ARM_EMERGENCY_STOP);
    if (chan != nullptr) {
        // 如果配置了紧急停止开关
        if (chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::LOW) {
            // 开关在紧急停止位置,可能是导致紧急停止的原因,所以不会导致预解锁检查失败
            return true;  
        }
    }
#endif  // AP_RC_CHANNEL_ENABLED
    // 其他情况下报告紧急停止错误
    check_failed(display_failure,"Motors Emergency Stopped");
    return false;
}

// 执行预解锁检查
bool AP_Arming::pre_arm_checks(bool report)
{
#if !APM_BUILD_COPTER_OR_HELI
    // 如果已经解锁或不需要解锁检查,则跳过检查
    if (armed || arming_required() == Required::NO) {
        return true;
    }
#endif

    // 执行所有预解锁检查项目
    bool checks_result = hardware_safety_check(report)
#if HAL_HAVE_IMU_HEATER
        &  heater_min_temperature_checks(report)
#endif
#if AP_BARO_ENABLED
        &  barometer_checks(report)
#endif
#if AP_INERTIALSENSOR_ENABLED
        &  ins_checks(report)
#endif
#if AP_COMPASS_ENABLED
        &  compass_checks(report)
#endif
#if AP_GPS_ENABLED
        &  gps_checks(report)
#endif
#if AP_BATTERY_ENABLED
        &  battery_checks(report)
#endif
#if HAL_LOGGING_ENABLED
        &  logging_checks(report)
#endif
#if AP_RC_CHANNEL_ENABLED
        &  manual_transmitter_checks(report)
#endif
#if AP_MISSION_ENABLED
        &  mission_checks(report)
#endif
#if AP_RANGEFINDER_ENABLED
        &  rangefinder_checks(report)
#endif
        &  servo_checks(report)
        &  board_voltage_checks(report)
        &  system_checks(report)
        &  terrain_checks(report)
#if HAL_MAX_CAN_PROTOCOL_DRIVERS && HAL_CANMANAGER_ENABLED
        &  can_checks(report)
#endif
#if HAL_GENERATOR_ENABLED
        &  generator_checks(report)
#endif
#if HAL_PROXIMITY_ENABLED
        &  proximity_checks(report)
#endif
#if HAL_RUNCAM_ENABLED
        &  camera_checks(report)
#endif
#if OSD_ENABLED
        &  osd_checks(report)
#endif
#if HAL_MOUNT_ENABLED
        &  mount_checks(report)
#endif
#if AP_FETTEC_ONEWIRE_ENABLED
        &  fettec_checks(report)
#endif
#if HAL_VISUALODOM_ENABLED
        &  visodom_checks(report)
#endif
#if AP_ARMING_AUX_AUTH_ENABLED
        &  aux_auth_checks(report)
#endif
#if AP_RC_CHANNEL_ENABLED
        &  disarm_switch_checks(report)
#endif
#if AP_FENCE_ENABLED
        &  fence_checks(report)
#endif
#if AP_OPENDRONEID_ENABLED
        &  opendroneid_checks(report)
#endif
#if AP_ARMING_CRASHDUMP_ACK_ENABLED
        & crashdump_checks(report)
#endif
        &  serial_protocol_checks(report)
        &  estop_checks(report);

    // 如果检查结果从通过变为失败,立即报告
    if (!checks_result && last_prearm_checks_result) { 
        report_immediately = true;
    }
    last_prearm_checks_result = checks_result;

    return checks_result;
}

// 执行解锁检查
bool AP_Arming::arm_checks(AP_Arming::Method method)
{
#if AP_RC_CHANNEL_ENABLED
    // 检查遥控器
    if (check_enabled(ARMING_CHECK_RC)) {
        if (!rc_arm_checks(method)) {
            return false;
        }
    }
#endif

    // 确保GPS驱动准备就绪
    if (check_enabled(ARMING_CHECK_GPS_CONFIG)) {
        if (!AP::gps().prepare_for_arming()) {
            return false;
        }
    }

    // 准备日志记录器开始记录
    // 应该是解锁前的最后一个检查

    // 注意即使禁用了解锁检查,也需要PrepForArming()
    // 禁用解锁检查不应该阻止日志记录工作

#if HAL_LOGGING_ENABLED
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger->logging_present()) {
        // 如果配置了日志记录,准备解锁
        logger->PrepForArming();
        if (!logger->logging_started() &&
            check_enabled(ARMING_CHECK_LOGGING)) {
            check_failed(ARMING_CHECK_LOGGING, true, "Logging not started");
            return false;
        }
    }
#endif  // HAL_LOGGING_ENABLED

    return true;
}

#if !AP_GPS_BLENDED_ENABLED
// 检查GPS混合自动切换
bool AP_Arming::blending_auto_switch_checks(bool report)
{
    if (AP::gps().get_auto_switch_type() == 2) {
        if (report) {
            check_failed(ARMING_CHECK_GPS, true, "GPS_AUTO_SWITCH==2 but no blending");
        }
        return false;
    }
    return true;
}
#endif

#if AP_ARMING_CRASHDUMP_ACK_ENABLED
// 检查崩溃转储
bool AP_Arming::crashdump_checks(bool report)
{
    // 如果没有崩溃转储数据,通过检查
    if (hal.util->last_crash_dump_size() == 0) {
        return true;
    }

    // 检查用户是否确认了故障并希望继续飞行
    if (crashdump_ack.acked) {
        // 即使用户确认了问题,我们仍然继续警告他们处于危险状态
        if (report) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CrashDump data detected");
        }
        return true;
    }

    check_failed(ARMING_CHECK_PARAMETERS, true, "CrashDump data detected");

    return false;
}
#endif  // AP_ARMING_CRASHDUMP_ACK_ENABLED

// 执行强制性检查
bool AP_Arming::mandatory_checks(bool report)
{
    bool ret = true;
#if AP_OPENDRONEID_ENABLED
    ret &= opendroneid_checks(report);
#endif
    ret &= rc_in_calibration_check(report);
    ret &= serial_protocol_checks(report);
    return ret;
}

// 尝试解锁,成功返回true
bool AP_Arming::arm(AP_Arming::Method method, const bool do_arming_checks)
{
    // 如果已经解锁则返回false
    if (armed) { 
        return false;
    }

    running_arming_checks = true;  // 在消息中显示Arm而不是Disarm

    // 执行解锁检查
    if ((!do_arming_checks && mandatory_checks(true)) || (pre_arm_checks(true) && arm_checks(method))) {
        armed = true;

        _last_arm_method = method;

#if HAL_LOGGING_ENABLED
        Log_Write_Arm(!do_arming_checks, method); // 注意Log_Write_Armed接受forced而不是do_arming_checks
#endif

    } else {
#if HAL_LOGGING_ENABLED
        AP::logger().arming_failure();
#endif
        armed = false;
    }

    running_arming_checks = false;

    // 如果解锁检查被禁用,显示警告
    if (armed && do_arming_checks && checks_to_perform == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Warning: Arming Checks Disabled");
    }
    
#if HAL_GYROFFT_ENABLED
    // 如果解锁检查被禁用,确保FFT子系统被启用
    AP_GyroFFT *fft = AP::fft();
    if (fft != nullptr) {
        fft->prepare_for_arming();
    }
#endif

#if AP_TERRAIN_AVAILABLE
    // 如果已解锁,告诉地形系统设置参考位置
    if (armed) {
        auto *terrain = AP::terrain();
        if (terrain != nullptr) {
            terrain->set_reference_location();
        }
    }
#endif

#if AP_FENCE_ENABLED
    // 如果已解锁,自动启用地理围栏
    if (armed) {
        auto *fence = AP::fence();
        if (fence != nullptr) {
            fence->auto_enable_fence_on_arming();
        }
    }
#endif
#if defined(HAL_ARM_GPIO_PIN)
    update_arm_gpio();
#endif
    return armed;
}

// 尝试加锁,成功返回true
bool AP_Arming::disarm(const AP_Arming::Method method, bool do_disarm_checks)
{
    // 如果已经加锁则返回false
    if (!armed) { 
        return false;
    }
    armed = false;
    _last_disarm_method = method;

#if HAL_LOGGING_ENABLED
    Log_Write_Disarm(!do_disarm_checks, method);  // Log_Write_Disarm接受"force"参数

    check_forced_logging(method);
#endif

#if HAL_HAVE_SAFETY_SWITCH
    // 如果配置了加锁时启用安全开关,则强制启用安全开关
    AP_BoardConfig *board_cfg = AP_BoardConfig::get_singleton();
    if ((board_cfg != nullptr) &&
        (board_cfg->get_safety_button_options() & AP_BoardConfig::BOARD_SAFETY_OPTION_SAFETY_ON_DISARM)) {
        hal.rcout->force_safety_on();
    }
#endif // HAL_HAVE_SAFETY_SWITCH

#if HAL_GYROFFT_ENABLED
    // 加锁时保存FFT参数
    AP_GyroFFT *fft = AP::fft();
    if (fft != nullptr) {
        fft->save_params_on_disarm();
    }
#endif

#if AP_FENCE_ENABLED
    // 加锁时自动禁用地理围栏
    AC_Fence *fence = AP::fence();
    if (fence != nullptr) {
        fence->auto_disable_fence_on_disarming();
    }
#endif
#if defined(HAL_ARM_GPIO_PIN)
    // 更新GPIO引脚状态
    update_arm_gpio();
#endif
    return true;
}

#if defined(HAL_ARM_GPIO_PIN)
// 更新GPIO引脚状态以反映当前的解锁状态
void AP_Arming::update_arm_gpio()
{
    if (!AP_BoardConfig::arming_gpio_disabled()) {
        // 根据解锁状态和极性设置GPIO引脚电平
        hal.gpio->write(HAL_ARM_GPIO_PIN, HAL_ARM_GPIO_POL_INVERT ? !armed : armed);
    }
}
#endif

// 发送解锁/加锁状态变化的消息
void AP_Arming::send_arm_disarm_statustext(const char *str) const
{
    // 如果禁用了状态变化消息则直接返回
    if (option_enabled(AP_Arming::Option::DISABLE_STATUSTEXT_ON_STATE_CHANGE)) {
        return;
    }
    // 发送状态变化消息
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s", str);
}

// 获取是否需要解锁
AP_Arming::Required AP_Arming::arming_required() const
{
#if AP_OPENDRONEID_ENABLED
    // 如果启用了OpenDroneID,则不能禁用解锁要求
    if (AP_OpenDroneID::get_singleton() != nullptr && AP::opendroneid().enabled()) {
        if (require != Required::YES_MIN_PWM && require != Required::YES_ZERO_PWM) {
            return Required::YES_MIN_PWM;
        }
    }
#endif
    return require;
}

#if AP_RC_CHANNEL_ENABLED
// 执行Copter和Sub共用的遥控器输入限制检查
// Copter默认检查最小和最大值是否已配置,Sub不检查
bool AP_Arming::rc_checks_copter_sub(const bool display_failure, const RC_Channel *channels[4]) const
{
    // 如果禁用了遥控器检查则返回成功
    if (!check_enabled(ARMING_CHECK_RC)) {
        return true;
    }

    bool ret = true;

    // 通道名称数组
    const char *channel_names[] = { "Roll", "Pitch", "Throttle", "Yaw" };

    // 检查每个通道
    for (uint8_t i=0; i<ARRAY_SIZE(channel_names);i++) {
        const RC_Channel *channel = channels[i];
        const char *channel_name = channel_names[i];
        // 检查遥控器是否已校准
        if (channel->get_radio_min() > RC_Channel::RC_CALIB_MIN_LIMIT_PWM) {
            check_failed(ARMING_CHECK_RC, display_failure, "%s radio min too high", channel_name);
            ret = false;
        }
        if (channel->get_radio_max() < RC_Channel::RC_CALIB_MAX_LIMIT_PWM) {
            check_failed(ARMING_CHECK_RC, display_failure, "%s radio max too low", channel_name);
            ret = false;
        }
    }
    return ret;
}
#endif  // AP_RC_CHANNEL_ENABLED

#if HAL_VISUALODOM_ENABLED
// 检查视觉里程计是否工作正常
bool AP_Arming::visodom_checks(bool display_failure) const
{
    // 如果禁用了视觉检查则返回成功
    if (!check_enabled(ARMING_CHECK_VISION)) {
        return true;
    }

    AP_VisualOdom *visual_odom = AP::visualodom();
    if (visual_odom != nullptr) {
        char fail_msg[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
        if (!visual_odom->pre_arm_check(fail_msg, ARRAY_SIZE(fail_msg))) {
            check_failed(ARMING_CHECK_VISION, display_failure, "VisOdom: %s", fail_msg);
            return false;
        }
    }

    return true;
}
#endif

#if AP_RC_CHANNEL_ENABLED
// 检查加锁开关是否已按下
// 检查加锁开关状态
bool AP_Arming::disarm_switch_checks(bool display_failure) const
{
    // 查找配置为加锁功能的遥控通道
    const RC_Channel *chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::DISARM);
    // 如果找到加锁通道且开关位于高位
    if (chan != nullptr &&
        chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH) {
        // 报告加锁开关打开错误
        check_failed(display_failure, "Disarm Switch on");
        // 返回检查失败
        return false;
    }

    // 加锁开关检查通过
    return true;
}
#endif  // AP_RC_CHANNEL_ENABLED

#if HAL_LOGGING_ENABLED
// 记录解锁事件
void AP_Arming::Log_Write_Arm(const bool forced, const AP_Arming::Method method)
{
    const struct log_Arm_Disarm pkt {
        LOG_PACKET_HEADER_INIT(LOG_ARM_DISARM_MSG),
        time_us                 : AP_HAL::micros64(),
        arm_state               : is_armed(),
        arm_checks              : get_enabled_checks(),
        forced                  : forced,
        method                  : (uint8_t)method,
    };
    AP::logger().WriteCriticalBlock(&pkt, sizeof(pkt));
    AP::logger().Write_Event(LogEvent::ARMED);
}

// 记录加锁事件
void AP_Arming::Log_Write_Disarm(const bool forced, const AP_Arming::Method method)
{
    const struct log_Arm_Disarm pkt {
        LOG_PACKET_HEADER_INIT(LOG_ARM_DISARM_MSG),
        time_us                 : AP_HAL::micros64(),
        arm_state               : is_armed(),
        arm_checks              : 0,
        forced                  : forced,
        method                  : (uint8_t)method
    };
    AP::logger().WriteCriticalBlock(&pkt, sizeof(pkt));
    AP::logger().Write_Event(LogEvent::DISARMED);
}

// 检查加锁后是否应该继续记录日志
void AP_Arming::check_forced_logging(const AP_Arming::Method method)
{
    // 如果是因为故障而加锁则继续记录日志
    switch(method) {
        case Method::TERMINATION:
        case Method::CPUFAILSAFE:
        case Method::BATTERYFAILSAFE:
        case Method::AFS:
        case Method::ADSBCOLLISIONACTION:
        case Method::PARACHUTE_RELEASE:
        case Method::CRASH:
        case Method::FENCEBREACH:
        case Method::RADIOFAILSAFE:
        case Method::GCSFAILSAFE:
        case Method::TERRRAINFAILSAFE:
        case Method::FAILSAFE_ACTION_TERMINATE:
        case Method::TERRAINFAILSAFE:
        case Method::BADFLOWOFCONTROL:
        case Method::EKFFAILSAFE:
        case Method::GCS_FAILSAFE_SURFACEFAILED:
        case Method::GCS_FAILSAFE_HOLDFAILED:
        case Method::PILOT_INPUT_FAILSAFE:
        case Method::DEADRECKON_FAILSAFE:
        case Method::BLACKBOX:
            // 如果是因为故障而加锁,则继续记录日志
            AP::logger().set_long_log_persist(true);
            return;

        case Method::RUDDER:
        case Method::MAVLINK:
        case Method::AUXSWITCH:
        case Method::MOTORTEST:
        case Method::SCRIPTING:
        case Method::SOLOPAUSEWHENLANDED:
        case Method::LANDED:
        case Method::MISSIONEXIT:
        case Method::DISARMDELAY:
        case Method::MOTORDETECTDONE:
        case Method::TAKEOFFTIMEOUT:
        case Method::AUTOLANDED:
        case Method::TOYMODELANDTHROTTLE:
        case Method::TOYMODELANDFORCE:
        case Method::LANDING:
        case Method::DDS:
        case Method::UNKNOWN:
            // 如果是正常加锁,则停止记录日志
            AP::logger().set_long_log_persist(false);
            return;
    }
}
#endif  // HAL_LOGGING_ENABLED

// AP_Arming单例指针
AP_Arming *AP_Arming::_singleton = nullptr;

// 获取AP_Arming单例
AP_Arming *AP_Arming::get_singleton()
{
    return AP_Arming::_singleton;
}

namespace AP {

// 获取AP_Arming单例的引用
AP_Arming &arming()
{
    return *AP_Arming::get_singleton();
}

};

#pragma GCC diagnostic pop

#endif  // AP_ARMING_ENABLED
