#include "Copter.h"

#pragma GCC diagnostic push
#if defined(__clang__)
#pragma GCC diagnostic ignored "-Wbitwise-instead-of-logical"
#endif

// 执行预起飞检查并设置检查结果
bool AP_Arming_Copter::pre_arm_checks(bool display_failure)
{
    const bool passed = run_pre_arm_checks(display_failure);
    set_pre_arm_check(passed);
    return passed;
}

// 执行预起飞检查
// 如果检查通过则返回true
bool AP_Arming_Copter::run_pre_arm_checks(bool display_failure)
{
    // 如果已经解锁则直接返回true
    if (copter.motors->armed()) {
        return true;
    }

    // 检查电机联锁和紧急停止开关是否同时使用
    // 这是不允许的
    bool passed = true;
    if (rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOTOR_INTERLOCK) &&
        (rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOTOR_ESTOP) || 
        rc().find_channel_for_option(RC_Channel::AUX_FUNC::ARM_EMERGENCY_STOP))){
        check_failed(display_failure, "Interlock/E-Stop Conflict");
        passed = false;
    }

    // 检查电机联锁辅助开关是否在使用
    // 如果在使用,需要处于禁用位置才能解锁
    // 否则直接退出
    if (copter.ap.using_interlock && copter.ap.motor_interlock_switch) {
        check_failed(display_failure, "Motor Interlock Enabled");
        passed = false;
    }

    // 执行解锁开关检查
    if (!disarm_switch_checks(display_failure)) {
        passed = false;
    }

    // 始终检查电机状态
    char failure_msg[100] {};
    if (!copter.motors->arming_checks(ARRAY_SIZE(failure_msg), failure_msg)) {
        check_failed(display_failure, "Motors: %s", failure_msg);
        passed = false;
    }

    // 如果未通过所有检查则返回false
    if (!passed) {
        return false;
    }

    // 如果预起飞检查被禁用,则只运行必要的检查
    if (checks_to_perform == 0) {
        return mandatory_checks(display_failure);
    }

    // 使用位与运算确保执行所有检查
    return parameter_checks(display_failure)
        & oa_checks(display_failure)
        & gcs_failsafe_check(display_failure)
        & winch_checks(display_failure)
        & rc_throttle_failsafe_checks(display_failure)
        & alt_checks(display_failure)
#if AP_AIRSPEED_ENABLED
        & AP_Arming::airspeed_checks(display_failure)
#endif
        & AP_Arming::pre_arm_checks(display_failure);
}

// 检查遥控器油门失效保护
bool AP_Arming_Copter::rc_throttle_failsafe_checks(bool display_failure) const
{
    if (!check_enabled(ARMING_CHECK_RC)) {
        // 此检查已被禁用
        return true;
    }

    // 油门失效保护。在这种情况下,参数也控制着无脉冲RC失效情况
    // radio-in值可能为零是因为根本没有收到任何RC脉冲
    // 注意,如果我们曾经看到过RC信号然后*丢失*了RC信号,那么如果用户依赖无脉冲来检测RC失效,
    // 这些检查可能会通过。但是,在这种情况下,由于处于RC失效状态而无法解锁。
    if (copter.g.failsafe_throttle == FS_THR_DISABLED) {
        return true;
    }

#if FRAME_CONFIG == HELI_FRAME
    const char *rc_item = "Collective";
#else
    const char *rc_item = "Throttle";
#endif

    // 检查是否有RC接收机或RC覆盖
    if (!rc().has_had_rc_receiver() && !rc().has_had_rc_override()) {
        check_failed(ARMING_CHECK_RC, display_failure, "RC not found");
        return false;
    }

    // 检查油门是否太低 - 必须高于失效保护油门值
    if (copter.channel_throttle->get_radio_in() < copter.g.failsafe_throttle_value) {
        check_failed(ARMING_CHECK_RC, display_failure, "%s below failsafe", rc_item);
        return false;
    }

    return true;
}

// 气压计检查
bool AP_Arming_Copter::barometer_checks(bool display_failure)
{
    if (!AP_Arming::barometer_checks(display_failure)) {
        return false;
    }

    bool ret = true;
    // 检查气压计
    if (check_enabled(ARMING_CHECK_BARO)) {
        // 如果EKF在绝对位置模式下运行,检查气压计和惯性导航高度是否在1m以内
        // 如果打算在地面相对高度模式下运行,则不检查,因为由于气压计漂移,
        // EKF输出的地面相对高度可能与气压计高度不同
        nav_filter_status filt_status = copter.inertial_nav.get_filter_status();
        bool using_baro_ref = (!filt_status.flags.pred_horiz_pos_rel && filt_status.flags.pred_horiz_pos_abs);
        if (using_baro_ref) {
            if (fabsf(copter.inertial_nav.get_position_z_up_cm() - copter.baro_alt) > PREARM_MAX_ALT_DISPARITY_CM) {
                check_failed(ARMING_CHECK_BARO, display_failure, "Altitude disparity");
                ret = false;
            }
        }
    }
    return ret;
}

// 惯性导航系统检查
bool AP_Arming_Copter::ins_checks(bool display_failure)
{
    bool ret = AP_Arming::ins_checks(display_failure);

    if (check_enabled(ARMING_CHECK_INS)) {

        // 获取EKF姿态(如果不好,通常是陀螺仪偏差问题)
        if (!pre_arm_ekf_attitude_check()) {
            check_failed(ARMING_CHECK_INS, display_failure, "EKF attitude is bad");
            ret = false;
        }
    }

    return ret;
}

// 板载电压检查
bool AP_Arming_Copter::board_voltage_checks(bool display_failure)
{
    if (!AP_Arming::board_voltage_checks(display_failure)) {
        return false;
    }

    // 检查电池电压
    if (check_enabled(ARMING_CHECK_VOLTAGE)) {
        if (copter.battery.has_failsafed()) {
            check_failed(ARMING_CHECK_VOLTAGE, display_failure, "Battery failsafe");
            return false;
        }
    }

    return true;
}

// 检查是否需要加载所有地形数据库数据
bool AP_Arming_Copter::terrain_database_required() const
{

    if (copter.wp_nav->get_terrain_source() == AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER) {
        // 主要地形源来自测距仪,允许在没有地形数据库的情况下解锁
        return false;
    }

    if (copter.wp_nav->get_terrain_source() == AC_WPNav::TerrainSource::TERRAIN_FROM_TERRAINDATABASE &&
        copter.mode_rtl.get_alt_type() == ModeRTL::RTLAltType::TERRAIN) {
        return true;
    }
    return AP_Arming::terrain_database_required();
}

// 参数检查
bool AP_Arming_Copter::parameter_checks(bool display_failure)
{
    // 检查各种参数值
    if (check_enabled(ARMING_CHECK_PARAMETERS)) {

        // 失效保护参数检查
        if (copter.g.failsafe_throttle) {
            // 检查油门最小值是否高于油门失效保护触发值,且触发值是否高于PPM编码器的信号丢失值900
            if (copter.channel_throttle->get_radio_min() <= copter.g.failsafe_throttle_value+10 || copter.g.failsafe_throttle_value < 910) {
                check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Check FS_THR_VALUE");
                return false;
            }
        }
        if (copter.g.failsafe_gcs == FS_GCS_ENABLED_CONTINUE_MISSION) {
            // FS_GCS_ENABLE == 2已被移除
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "FS_GCS_ENABLE=2 removed, see FS_OPTIONS");
        }

        // 倾斜角度参数检查
        if (copter.aparm.angle_max < 1000 || copter.aparm.angle_max > 8000) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Check ANGLE_MAX");
            return false;
        }

        // 特技飞行平衡参数检查
#if MODE_ACRO_ENABLED || MODE_SPORT_ENABLED
        if ((copter.g.acro_balance_roll > copter.attitude_control->get_angle_roll_p().kP()) || (copter.g.acro_balance_pitch > copter.attitude_control->get_angle_pitch_p().kP())) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Check ACRO_BAL_ROLL/PITCH");
            return false;
        }
#endif

        // 飞行员速度增加参数检查
        if (copter.g.pilot_speed_up <= 0) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Check PILOT_SPEED_UP");
            return false;
        }

        #if FRAME_CONFIG == HELI_FRAME
        char fail_msg[100]{};
        // 检查输入管理器参数
        if (!copter.input_manager.parameter_check(fail_msg, ARRAY_SIZE(fail_msg))) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "%s", fail_msg);
            return false;
        }

        // 确保为电机联锁配置了一个辅助通道
        if (rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOTOR_INTERLOCK) == nullptr) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Motor Interlock not configured");
            return false;
        }

        #else
        switch (copter.g2.frame_class.get()) {
        case AP_Motors::MOTOR_FRAME_HELI_QUAD:
        case AP_Motors::MOTOR_FRAME_HELI_DUAL:
        case AP_Motors::MOTOR_FRAME_HELI:
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Invalid MultiCopter FRAME_CLASS");
            return false;

        default:
            break;
        }
        #endif // HELI_FRAME

        // 使用测距仪进行RTL时的检查
#if MODE_RTL_ENABLED
        if (copter.mode_rtl.get_alt_type() == ModeRTL::RTLAltType::TERRAIN) {
            // 从wpnav获取地形源
            const char *failure_template = "RTL_ALT_TYPE is above-terrain but %s";
            switch (copter.wp_nav->get_terrain_source()) {
            case AC_WPNav::TerrainSource::TERRAIN_UNAVAILABLE:
                check_failed(ARMING_CHECK_PARAMETERS, display_failure, failure_template, "no terrain data");
                return false;
                break;
            case AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER:
#if AP_RANGEFINDER_ENABLED
                if (!copter.rangefinder_state.enabled || !copter.rangefinder.has_orientation(ROTATION_PITCH_270)) {
                    check_failed(ARMING_CHECK_PARAMETERS, display_failure, failure_template, "no rangefinder");
                    return false;
                }
                // 检查RTL_ALT是否高于测距仪的最大范围
                if (copter.g.rtl_altitude > copter.rangefinder.max_distance_cm_orient(ROTATION_PITCH_270)) {
                    check_failed(ARMING_CHECK_PARAMETERS, display_failure, failure_template, "RTL_ALT>RNGFND_MAX_CM");
                    return false;
                }
#else
                check_failed(ARMING_CHECK_PARAMETERS, display_failure, failure_template, "rangefinder not in firmware");
#endif
                break;
            case AC_WPNav::TerrainSource::TERRAIN_FROM_TERRAINDATABASE:
                // 这些检查在AP_Arming中完成
                break;
            }
        }
#endif

        // 检查ADSB避障失效保护
#if HAL_ADSB_ENABLED
        if (copter.failsafe.adsb) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "ADSB threat detected");
            return false;
        }
#endif

        // 确保控制器允许我们解锁:
        char failure_msg[100] = {};
        if (!copter.pos_control->pre_arm_checks("PSC", failure_msg, ARRAY_SIZE(failure_msg))) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Bad parameter: %s", failure_msg);
            return false;
        }
        if (!copter.attitude_control->pre_arm_checks("ATC", failure_msg, ARRAY_SIZE(failure_msg))) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Bad parameter: %s", failure_msg);
            return false;
        }
    }

    return true;
}

// 执行避障系统检查
bool AP_Arming_Copter::oa_checks(bool display_failure)
{
#if AP_OAPATHPLANNER_ENABLED
    char failure_msg[100] = {};
    // 调用避障系统的预解锁检查
    if (copter.g2.oa.pre_arm_check(failure_msg, ARRAY_SIZE(failure_msg))) {
        return true;
    }
    // 显示失败信息
    if (strlen(failure_msg) == 0) {
        check_failed(display_failure, "%s", "Check Object Avoidance");
    } else {
        check_failed(display_failure, "%s", failure_msg);
    }
    return false;
#else
    return true;
#endif
}

// 执行遥控器校准检查
bool AP_Arming_Copter::rc_calibration_checks(bool display_failure)
{
    // 定义需要检查的遥控器通道
    const RC_Channel *channels[] = {
        copter.channel_roll,
        copter.channel_pitch,
        copter.channel_throttle,
        copter.channel_yaw
    };

    // 使用位与运算确保所有检查都被执行
    copter.ap.pre_arm_rc_check = rc_checks_copter_sub(display_failure, channels)
        & AP_Arming::rc_calibration_checks(display_failure);

    return copter.ap.pre_arm_rc_check;
}

// 执行GPS相关的预解锁检查，如果通过则返回true
bool AP_Arming_Copter::gps_checks(bool display_failure)
{
    // 检查围栏是否需要GPS
    bool fence_requires_gps = false;
#if AP_FENCE_ENABLED
    // 如果启用了圆形或多边形围栏，则需要GPS
    fence_requires_gps = (copter.fence.get_enabled_fences() & (AC_FENCE_TYPE_CIRCLE | AC_FENCE_TYPE_POLYGON)) > 0;
#endif

    // 检查飞行模式是否需要GPS
    bool mode_requires_gps = copter.flightmode->requires_GPS() || fence_requires_gps || (copter.simple_mode == Copter::SimpleMode::SUPERSIMPLE);

    // 调用父类的GPS检查
    if (mode_requires_gps) {
        if (!AP_Arming::gps_checks(display_failure)) {
            AP_Notify::flags.pre_arm_gps_check = false;
            return false;
        }
    }

    // 首先运行必需的GPS检查
    if (!mandatory_gps_checks(display_failure)) {
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // 如果不需要GPS则返回true
    if (!mode_requires_gps) {
        AP_Notify::flags.pre_arm_gps_check = true;
        return true;
    }

    // 如果GPS检查被禁用则立即返回true
    if (!check_enabled(ARMING_CHECK_GPS)) {
        AP_Notify::flags.pre_arm_gps_check = true;
        return true;
    }

    // 单独警告HDOP问题 - 防止用户与无GPS锁定混淆
    if ((copter.gps.num_sensors() > 0) && (copter.gps.get_hdop() > copter.g.gps_hdop_good)) {
        check_failed(ARMING_CHECK_GPS, display_failure, "High GPS HDOP");
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // 如果到达这里说明一切正常
    AP_Notify::flags.pre_arm_gps_check = true;
    return true;
}

// 检查EKF姿态是否可接受
bool AP_Arming_Copter::pre_arm_ekf_attitude_check()
{
    // 获取ekf滤波器状态
    nav_filter_status filt_status = copter.inertial_nav.get_filter_status();

    return filt_status.flags.attitude;
}

#if HAL_PROXIMITY_ENABLED
// 检查是否有物体太靠近飞行器
bool AP_Arming_Copter::proximity_checks(bool display_failure) const
{

    if (!AP_Arming::proximity_checks(display_failure)) {
        return false;
    }

    if (!check_enabled(ARMING_CHECK_PARAMETERS)) {
        // 检查被禁用
        return true;
    }

    // 如果可能用于避障，则获取最近的物体
#if AP_AVOIDANCE_ENABLED
    float angle_deg, distance;
    if (copter.avoid.proximity_avoidance_enabled() && copter.g2.proximity.get_closest_object(angle_deg, distance)) {
        // 如果有物体在60cm内则显示错误
        const float tolerance = 0.6f;
        if (distance <= tolerance) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Proximity %d deg, %4.2fm (want > %0.1fm)", (int)angle_deg, (double)distance, (double)tolerance);
            return false;
        }
    }
#endif

    return true;
}
#endif  // HAL_PROXIMITY_ENABLED

// 执行必需的GPS检查。如果通过则返回true
bool AP_Arming_Copter::mandatory_gps_checks(bool display_failure)
{
    // 检查飞行模式是否需要GPS
    bool mode_requires_gps = copter.flightmode->requires_GPS();

    // 始终检查惯性导航是否已启动并准备就绪
    const auto &ahrs = AP::ahrs();
    char failure_msg[100] = {};
    if (!ahrs.pre_arm_check(mode_requires_gps, failure_msg, sizeof(failure_msg))) {
        check_failed(display_failure, "AHRS: %s", failure_msg);
        return false;
    }

    // 检查围栏是否需要GPS
    bool fence_requires_gps = false;
#if AP_FENCE_ENABLED
    // 如果启用了圆形或多边形围栏则需要GPS
    fence_requires_gps = (copter.fence.get_enabled_fences() & (AC_FENCE_TYPE_CIRCLE | AC_FENCE_TYPE_POLYGON)) > 0;
#endif

    if (mode_requires_gps || copter.option_is_enabled(Copter::FlightOption::REQUIRE_POSITION_FOR_ARMING)) {
        if (!copter.position_ok()) {
            // 飞行器水平位置估计检查
            check_failed(display_failure, "Need Position Estimate");
            return false;
        }
    } else if (fence_requires_gps) {
        if (!copter.position_ok()) {
            // 向用户说明为什么在非GPS飞行模式下需要GPS
            check_failed(display_failure, "Fence enabled, need position estimate");
            return false;
        }
    } else {
        // 如果不需要GPS则返回true
        return true;
    }

    // 检查GPS故障(由EKF报告)
    nav_filter_status filt_status;
    if (ahrs.get_filter_status(filt_status)) {
        if (filt_status.flags.gps_glitching) {
            check_failed(display_failure, "GPS glitching");
            return false;
        }
    }

    // 检查EKF的罗盘、位置、高度和速度方差是否低于故障保护阈值
    if (copter.g.fs_ekf_thresh > 0.0f) {
        float vel_variance, pos_variance, hgt_variance, tas_variance;
        Vector3f mag_variance;
        ahrs.get_variances(vel_variance, pos_variance, hgt_variance, mag_variance, tas_variance);
        if (mag_variance.length() >= copter.g.fs_ekf_thresh) {
            check_failed(display_failure, "EKF compass variance");
            return false;
        }
        if (pos_variance >= copter.g.fs_ekf_thresh) {
            check_failed(display_failure, "EKF position variance");
            return false;
        }
        if (vel_variance >= copter.g.fs_ekf_thresh) {
            check_failed(display_failure, "EKF velocity variance");
            return false;
        }
        if (hgt_variance >= copter.g.fs_ekf_thresh) {
            check_failed(display_failure, "EKF height variance");
            return false;
        }
    }

    // 如果到达这里说明一切正常
    return true;
}

// 检查地面站故障保护
bool AP_Arming_Copter::gcs_failsafe_check(bool display_failure)
{
    if (copter.failsafe.gcs) {
        check_failed(display_failure, "GCS failsafe on");
        return false;
    }
    return true;
}

// 检查绞车
bool AP_Arming_Copter::winch_checks(bool display_failure) const
{
#if AP_WINCH_ENABLED
    // 如果参数或所有解锁检查被禁用则通过
    if (!check_enabled(ARMING_CHECK_PARAMETERS)) {
        return true;
    }

    const AP_Winch *winch = AP::winch();
    if (winch == nullptr) {
        return true;
    }
    char failure_msg[100] = {};
    if (!winch->pre_arm_check(failure_msg, sizeof(failure_msg))) {
        check_failed(display_failure, "%s", failure_msg);
        return false;
    }
#endif
    return true;
}

// 执行高度检查。如果通过则返回true
bool AP_Arming_Copter::alt_checks(bool display_failure)
{
    // 始终检查EKF高度估计
    if (!copter.flightmode->has_manual_throttle() && !copter.ekf_alt_ok()) {
        check_failed(display_failure, "Need Alt Estimate");
        return false;
    }

    return true;
}

// arm_checks - 在解锁前执行最终检查
// 总是在解锁前调用。如果可以解锁则返回true
// 有启动日志记录的副作用
bool AP_Arming_Copter::arm_checks(AP_Arming::Method method)
{
    const auto &ahrs = AP::ahrs();

    // 始终检查惯性导航是否已启动并准备就绪
    if (!ahrs.healthy()) {
        check_failed(true, "AHRS not healthy");
        return false;
    }

#ifndef ALLOW_ARM_NO_COMPASS
    // 如果航向来源不是罗盘，我们可以跳过罗盘健康检查
    if (!ahrs.using_noncompass_for_yaw()) {
        const Compass &_compass = AP::compass();
        // 检查罗盘健康状况
        if (!_compass.healthy()) {
            check_failed(true, "Compass not healthy");
            return false;
        }
    }
#endif

    // 始终检查当前模式是否允许解锁
    if (!copter.flightmode->allows_arming(method)) {
        check_failed(true, "%s mode not armable", copter.flightmode->name());
        return false;
    }

    // 如果解锁检查被禁用则成功
    if (checks_to_perform == 0) {
        return true;
    }

    // 检查倾斜角度
    if (check_enabled(ARMING_CHECK_INS)) {
        if (degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch()))*100.0f > copter.aparm.angle_max) {
            check_failed(ARMING_CHECK_INS, true, "Leaning");
            return false;
        }
    }

    // 检查ADSB
#if HAL_ADSB_ENABLED
    if (check_enabled(ARMING_CHECK_PARAMETERS)) {
        if (copter.failsafe.adsb) {
            check_failed(ARMING_CHECK_PARAMETERS, true, "ADSB threat detected");
            return false;
        }
    }
#endif

    // 检查油门
    if (check_enabled(ARMING_CHECK_RC)) {
#if FRAME_CONFIG == HELI_FRAME
        const char *rc_item = "Collective";
#else
        const char *rc_item = "Throttle";
#endif
        // 检查油门是否太高 - 如果从GCS/脚本在引导、无GPS引导或自动模式下解锁则跳过检查
        if (!((AP_Arming::method_is_GCS(method) || method == AP_Arming::Method::SCRIPTING) && copter.flightmode->allows_GCS_or_SCR_arming_with_throttle_high())) {
            // 死区以上总是太高
            if (copter.get_pilot_desired_climb_rate(copter.channel_throttle->get_control_in()) > 0.0f) {
                check_failed(ARMING_CHECK_RC, true, "%s too high", rc_item);
                return false;
            }
            // 在手动模式下油门必须为零
#if FRAME_CONFIG != HELI_FRAME
            if ((copter.flightmode->has_manual_throttle() || copter.flightmode->mode_number() == Mode::Number::DRIFT) && copter.channel_throttle->get_control_in() > 0) {
                check_failed(ARMING_CHECK_RC, true, "%s too high", rc_item);
                return false;
            }
#endif
        }
    }

    // 检查安全开关是否已按下
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        check_failed(true, "Safety Switch");
        return false;
    }

    // 父类方法应该总是最后调用;
    // 因为如果我们的解锁检查失败,它的副作用需要被清理
    return AP_Arming::arm_checks(method);
}

// 如果ARMING_CHECK为零或强制解锁时将运行的强制检查
bool AP_Arming_Copter::mandatory_checks(bool display_failure)
{
    // 调用强制GPS检查并更新通知状态,因为常规GPS检查不会运行
    bool result = mandatory_gps_checks(display_failure);
    AP_Notify::flags.pre_arm_gps_check = result;

    // 调用强制高度检查
    if (!alt_checks(display_failure)) {
        result = false;
    }

    return result & AP_Arming::mandatory_checks(display_failure);
}

// 设置预解锁检查标志
void AP_Arming_Copter::set_pre_arm_check(bool b)
{
    copter.ap.pre_arm_check = b;
    AP_Notify::flags.pre_arm_check = b;
}

// 解锁电机
bool AP_Arming_Copter::arm(const AP_Arming::Method method, const bool do_arming_checks)
{
    static bool in_arm_motors = false;

    // 如果已经在此函数中则立即退出
    if (in_arm_motors) {
        return false;
    }
    in_arm_motors = true;

    // 如果已经解锁则返回true
    if (copter.motors->armed()) {
        in_arm_motors = false;
        return true;
    }

    if (!AP_Arming::arm(method, do_arming_checks)) {
        AP_Notify::events.arming_failed = true;
        in_arm_motors = false;
        return false;
    }

#if HAL_LOGGING_ENABLED
    // 让日志记录器知道我们已解锁(它可能会打开日志等)
    AP::logger().set_vehicle_armed(true);
#endif

    // 禁用CPU故障保护,因为初始化所有内容需要一段时间
    copter.failsafe_disable();

    // 通知即将解锁(我们提前这样做以提供充分的警告)
    AP_Notify::flags.armed = true;
    // 多次调用通知更新以确保消息发出
    for (uint8_t i=0; i<=10; i++) {
        AP::notify().update();
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    send_arm_disarm_statustext("Arming motors");
#endif

    // 记住方向
    // --------------------
    copter.init_simple_bearing();

    auto &ahrs = AP::ahrs();

    copter.initial_armed_bearing = ahrs.yaw_sensor;

    if (!ahrs.home_is_set()) {
        // 如果还没有设置home点,则重置EKF高度(我们使用EKF高度作为home点以上高度的替代)
        ahrs.resetHeightDatum();
        LOGGER_WRITE_EVENT(LogEvent::EKF_ALT_RESET);

        // 我们已重置高度,所以解锁高度为零
        copter.arming_altitude_m = 0;
    } else if (!ahrs.home_is_locked()) {
        // 如果已经设置了home点但未锁定,则重置home点位置
        if (!copter.set_home_to_current_location(false)) {
            // 忽略失败
        }

        // 记住我们解锁时的高度
        copter.arming_altitude_m = copter.inertial_nav.get_position_z_up_cm() * 0.01;
    }
    copter.update_super_simple_bearing(false);

    // 重置SmartRTL返回位置。如果激活,SmartRTL最终会尝试在此点着陆
#if MODE_SMARTRTL_ENABLED
    copter.g2.smart_rtl.set_home(copter.position_ok());
#endif

    hal.util->set_soft_armed(true);

#if HAL_SPRAYER_ENABLED
    // 关闭喷雾器的测试(如果开启)
    copter.sprayer.test_pump(false);
#endif

    // 输出最低可能值到电机
    copter.motors->output_min();

    // 最后实际解锁电机
    copter.motors->armed(true);

#if HAL_LOGGING_ENABLED
    // 记录飞行模式,以防在飞机未解锁时更改了模式
    AP::logger().Write_Mode((uint8_t)copter.flightmode->mode_number(), copter.control_mode_reason);
#endif

    // 重新启用故障保护
    copter.failsafe_enable();

    // 性能监视器忽略由于解锁导致的延迟
    AP::scheduler().perf_info.ignore_this_loop();

    // 标记退出此函数
    in_arm_motors = false;

    // 记录解锁事件的时间戳
    copter.arm_time_ms = millis();

    // 开始解锁延迟
    copter.ap.in_arming_delay = true;

    // 假设在没有解锁开关的情况下解锁。在switches.cpp中被覆盖
    copter.ap.armed_with_airmode_switch = false;

    // 返回成功
    return true;
}

// arming.disarm - 锁定电机
bool AP_Arming_Copter::disarm(const AP_Arming::Method method, bool do_disarm_checks)
{
    // 如果我们已经锁定则立即返回
    if (!copter.motors->armed()) {
        return true;
    }

    // 如果我们认为正在飞行,则不允许通过mavlink锁定:
    if (do_disarm_checks &&
        AP_Arming::method_is_GCS(method) &&
        !copter.ap.land_complete) {
        return false;
    }

    if (!AP_Arming::disarm(method, do_disarm_checks)) {
        return false;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    send_arm_disarm_statustext("Disarming motors");
#endif

    auto &ahrs = AP::ahrs();

    // 如果启用,保存EKF学习的罗盘偏移
    Compass &compass = AP::compass();
    if (ahrs.use_compass() && compass.get_learn_type() == Compass::LEARN_EKF) {
        for(uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
            Vector3f magOffsets;
            if (ahrs.getMagOffsets(i, magOffsets)) {
                compass.set_and_save_offsets(i, magOffsets);
            }
        }
    }

    // 我们不在空中
    copter.set_land_complete(true);
    copter.set_land_complete_maybe(true);

    // 发送锁定命令到电机
    copter.motors->armed(false);

#if MODE_AUTO_ENABLED
    // 重置任务
    copter.mode_auto.mission.reset();
#endif

#if HAL_LOGGING_ENABLED
    AP::logger().set_vehicle_armed(false);
#endif

    hal.util->set_soft_armed(false);

    copter.ap.in_arming_delay = false;

#if AUTOTUNE_ENABLED
    // 可能保存自动调参参数
    copter.mode_autotune.autotune.disarmed(copter.flightmode == &copter.mode_autotune);
#endif

    return true;
}

#pragma GCC diagnostic pop
