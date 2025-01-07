/*
   主要开发者: Andrew Tridgell
 
   作者:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Amilcar Lucas, Gregory Fletcher, Paul Riseborough, Brandon Jones, Jon Challinger, Tom Pittenger
   鸣谢:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier, Yury MonZon

   请贡献您的想法! 详情请见 https://ardupilot.org/dev

   本程序是自由软件：您可以根据自由软件基金会发布的GNU通用公共许可证的条款，即许可证的第3版或（您选择的）任何后来的版本重新发布它和/或修改它。

   本程序的发布是希望它能起到作用。但没有任何保证；甚至没有隐含的保证。本程序的分发是希望它是有用的，但没有任何保证，甚至没有隐含的适销对路或适合某一特定目的的保证。参见GNU通用公共许可证了解更多细节。

   您应该已经收到一份GNU通用公共许可证的副本。如果没有，请参阅<http://www.gnu.org/licenses/>。
 */

#include "Plane.h"

// 定义调度任务宏
#define SCHED_TASK(func, rate_hz, max_time_micros, priority) SCHED_TASK_CLASS(Plane, &plane, func, rate_hz, max_time_micros, priority)
#define FAST_TASK(func) FAST_TASK_CLASS(Plane, &plane, func)

// 调度器任务表 - 所有常规任务都应列在此处
const AP_Scheduler::Task Plane::scheduler_tasks[] = {
    // 单位:   Hz      us
    FAST_TASK(ahrs_update),                    // AHRS更新
    FAST_TASK(update_control_mode),            // 更新控制模式
    FAST_TASK(stabilize),                      // 稳定
    FAST_TASK(set_servos),                     // 设置舵机
    SCHED_TASK(read_radio,             50,    100,   6),  // 读取遥控信号
    SCHED_TASK(check_short_failsafe,   50,    100,   9),  // 检查短期故障保护
    SCHED_TASK(update_speed_height,    50,    200,  12),  // 更新速度和高度
    SCHED_TASK(update_throttle_hover, 100,     90,  24),  // 更新悬停油门
    SCHED_TASK_CLASS(RC_Channels,     (RC_Channels*)&plane.g2.rc_channels, read_mode_switch,           7,    100, 27),  // 读取模式开关
    SCHED_TASK(update_GPS_50Hz,        50,    300,  30),  // 50Hz GPS更新
    SCHED_TASK(update_GPS_10Hz,        10,    400,  33),  // 10Hz GPS更新
    SCHED_TASK(navigate,               10,    150,  36),  // 导航
    SCHED_TASK(update_compass,         10,    200,  39),  // 更新指南针
    SCHED_TASK(calc_airspeed_errors,   10,    100,  42),  // 计算空速误差
    SCHED_TASK(update_alt,             10,    200,  45),  // 更新高度
    SCHED_TASK(adjust_altitude_target, 10,    200,  48),  // 调整目标高度
#if AP_ADVANCEDFAILSAFE_ENABLED
    SCHED_TASK(afs_fs_check,           10,    100,  51),  // 高级故障保护检查
#endif
    SCHED_TASK(ekf_check,              10,     75,  54),  // EKF检查
    SCHED_TASK_CLASS(GCS,            (GCS*)&plane._gcs,       update_receive,   300,  500,  57),  // GCS接收更新
    SCHED_TASK_CLASS(GCS,            (GCS*)&plane._gcs,       update_send,      300,  750,  60),  // GCS发送更新
#if AP_SERVORELAYEVENTS_ENABLED
    SCHED_TASK_CLASS(AP_ServoRelayEvents, &plane.ServoRelayEvents, update_events, 50, 150,  63),  // 舵机继电器事件更新
#endif
    SCHED_TASK_CLASS(AP_BattMonitor, &plane.battery, read,   10, 300,  66),  // 电池监测读取
#if AP_RANGEFINDER_ENABLED
    SCHED_TASK(read_rangefinder,       50,    100, 78),  // 读取测距仪
#endif
#if AP_ICENGINE_ENABLED
    SCHED_TASK_CLASS(AP_ICEngine,      &plane.g2.ice_control, update,     10, 100,  81),  // 内燃机控制更新
#endif
#if AP_OPTICALFLOW_ENABLED
    SCHED_TASK_CLASS(AP_OpticalFlow, &plane.optflow, update,    50,    50,  87),  // 光流更新
#endif
    SCHED_TASK(one_second_loop,         1,    400,  90),  // 1秒循环
    SCHED_TASK(three_hz_loop,           3,     75,  93),  // 3Hz循环
    SCHED_TASK(check_long_failsafe,     3,    400,  96),  // 检查长期故障保护
#if AP_RPM_ENABLED
    SCHED_TASK_CLASS(AP_RPM,           &plane.rpm_sensor,     update,     10, 100,  99),  // RPM传感器更新
#endif
#if AP_AIRSPEED_AUTOCAL_ENABLE
    SCHED_TASK(airspeed_ratio_update,   1,    100,  102),  // 空速比更新
#endif // AP_AIRSPEED_AUTOCAL_ENABLE
#if HAL_MOUNT_ENABLED
    SCHED_TASK_CLASS(AP_Mount, &plane.camera_mount, update, 50, 100, 105),  // 云台更新
#endif // HAL_MOUNT_ENABLED
#if AP_CAMERA_ENABLED
    SCHED_TASK_CLASS(AP_Camera, &plane.camera, update,      50, 100, 108),  // 相机更新
#endif // CAMERA == ENABLED
#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(AP_Scheduler, &plane.scheduler, update_logging,         0.2,    100, 111),  // 调度器日志更新
#endif
    SCHED_TASK(compass_save,          0.1,    200, 114),  // 保存指南针校准
#if HAL_LOGGING_ENABLED
    SCHED_TASK(Log_Write_FullRate,        400,    300, 117),  // 全速率日志写入
    SCHED_TASK(update_logging10,        10,    300, 120),  // 10Hz日志更新
    SCHED_TASK(update_logging25,        25,    300, 123),  // 25Hz日志更新
#endif
#if HAL_SOARING_ENABLED
    SCHED_TASK(update_soaring,         50,    400, 126),  // 滑翔更新
#endif
    SCHED_TASK(parachute_check,        10,    200, 129),  // 降落伞检查
#if AP_TERRAIN_AVAILABLE
    SCHED_TASK_CLASS(AP_Terrain, &plane.terrain, update, 10, 200, 132),  // 地形更新
#endif // AP_TERRAIN_AVAILABLE
    SCHED_TASK(update_is_flying_5Hz,    5,    100, 135),  // 5Hz飞行状态更新
#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(AP_Logger,         &plane.logger, periodic_tasks, 50, 400, 138),  // 日志周期性任务
#endif
    SCHED_TASK_CLASS(AP_InertialSensor, &plane.ins,    periodic,       50,  50, 141),  // 惯性传感器周期性任务
#if HAL_ADSB_ENABLED
    SCHED_TASK(avoidance_adsb_update,  10,    100, 144),  // ADS-B避障更新
#endif
    SCHED_TASK_CLASS(RC_Channels,       (RC_Channels*)&plane.g2.rc_channels, read_aux_all,           10,    200, 147),  // 读取所有辅助通道
#if HAL_BUTTON_ENABLED
    SCHED_TASK_CLASS(AP_Button, &plane.button, update, 5, 100, 150),  // 按钮更新
#endif
#if AP_LANDINGGEAR_ENABLED
    SCHED_TASK(landing_gear_update, 5, 50, 159),  // 起落架更新
#endif
#if AC_PRECLAND_ENABLED
    SCHED_TASK(precland_update, 400, 50, 160),  // 精确着陆更新
#endif
};

// 获取调度器任务
void Plane::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                uint8_t &task_count,
                                uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}

#if HAL_QUADPLANE_ENABLED
constexpr int8_t Plane::_failsafe_priorities[7];
#else
constexpr int8_t Plane::_failsafe_priorities[6];
#endif

// 更新AHRS系统
void Plane::ahrs_update()
{
    arming.update_soft_armed();

    ahrs.update();

#if HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_IMU)) {
        AP::ins().Write_IMU();
    }
#endif

    // 根据当前俯仰角计算缩放后的横滚限制
    roll_limit_cd = aparm.roll_limit*100;
    pitch_limit_min = aparm.pitch_limit_min;

    bool rotate_limits = true;
#if HAL_QUADPLANE_ENABLED
    if (quadplane.tailsitter.active()) {
        rotate_limits = false;
    }
#endif
    if (rotate_limits) {
        roll_limit_cd *= ahrs.cos_pitch();
        pitch_limit_min *= fabsf(ahrs.cos_roll());
    }

    // 更新用于地面转向和自动起飞的陀螺仪总和
    // DCM.c与陀螺仪向量的点积给出地球坐标系的偏航率
    steer_state.locked_course_err += ahrs.get_yaw_rate_earth() * G_Dt;
    steer_state.locked_course_err = wrap_PI(steer_state.locked_course_err);

#if HAL_QUADPLANE_ENABLED
    // 检查EKF是否有偏航重置
    quadplane.check_yaw_reset();

    // 更新四旋翼的惯性导航
    quadplane.inertial_nav.update();
#endif

#if HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_VIDEO_STABILISATION)) {
        ahrs.write_video_stabilisation();
    }
#endif
}

// 更新50Hz速度/高度控制器
void Plane::update_speed_height(void)
{
    bool should_run_tecs = control_mode->does_auto_throttle();
#if HAL_QUADPLANE_ENABLED
    if (quadplane.should_disable_TECS()) {
        should_run_tecs = false;
    }
#endif

    if (auto_state.idle_mode) {
        should_run_tecs = false;
    }

#if AP_PLANE_GLIDER_PULLUP_ENABLED
    if (mode_auto.in_pullup()) {
        should_run_tecs = false;
    }
#endif

    if (should_run_tecs) {
        // 调用TECS 50Hz更新。注意，无论油门是否被抑制，我们都会调用此函数，因为这需要运行以进行起飞检测
        TECS_controller.update_50hz();
    }

#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode() ||
        quadplane.in_assisted_flight()) {
        quadplane.update_throttle_mix();
    }
#endif
}

// 读取和更新指南针
void Plane::update_compass(void)
{
    compass.read();
}

#if HAL_LOGGING_ENABLED
// 执行10Hz日志记录
void Plane::update_logging10(void)
{
    bool log_faster = (should_log(MASK_LOG_ATTITUDE_FULLRATE) || should_log(MASK_LOG_ATTITUDE_FAST));
    if (should_log(MASK_LOG_ATTITUDE_MED) && !log_faster) {
        Log_Write_Attitude();
        ahrs.Write_AOA_SSA();
    } else if (log_faster) {
        ahrs.Write_AOA_SSA();
    }
#if HAL_MOUNT_ENABLED
    if (should_log(MASK_LOG_CAMERA)) {
        camera_mount.write_log();
    }
#endif
}

/*
  执行25Hz日志记录
 */
void Plane::update_logging25(void)
{
    // MASK_LOG_ATTITUDE_FULLRATE 以400Hz记录，MASK_LOG_ATTITUDE_FAST 以25Hz记录，MASK_LOG_ATTIUDE_MED 以10Hz记录
    // 选择最高频率
    bool log_faster = should_log(MASK_LOG_ATTITUDE_FULLRATE);
    if (should_log(MASK_LOG_ATTITUDE_FAST) && !log_faster) {
        Log_Write_Attitude();
    }

    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
        if (!should_log(MASK_LOG_NOTCH_FULLRATE)) {
            AP::ins().write_notch_log_messages();
        }
#endif
#if HAL_GYROFFT_ENABLED
        gyro_fft.write_log_messages();
#endif
    }

    if (should_log(MASK_LOG_NTUN)) {
        Log_Write_Nav_Tuning();
        Log_Write_Guided();
    }

    if (should_log(MASK_LOG_RC))
        Log_Write_RC();

    if (should_log(MASK_LOG_IMU))
        AP::ins().Write_Vibration();
}
#endif  // HAL_LOGGING_ENABLED

/*
  检查AFS故障保护
 */
#if AP_ADVANCEDFAILSAFE_ENABLED
void Plane::afs_fs_check(void)
{
    afs.check(failsafe.AFS_last_valid_rc_ms);
}
#endif

#if HAL_WITH_IO_MCU
#include <AP_IOMCU/AP_IOMCU.h>
extern AP_IOMCU iomcu;
#endif

void Plane::one_second_loop()
{
    // 使运行时更改控制通道顺序成为可能
    set_control_channels();

#if HAL_WITH_IO_MCU
    iomcu.setup_mixing(&rcmap, g.override_channel.get(), g.mixing_gain, g2.manual_rc_mask);
#endif

#if HAL_ADSB_ENABLED
    adsb.set_stall_speed_cm(aparm.airspeed_min * 100); // 将m/s转换为cm/s
    adsb.set_max_speed(aparm.airspeed_max);
#endif

    if (flight_option_enabled(FlightOptions::ENABLE_DEFAULT_AIRSPEED)) {
        // 使用最小和最大空速的平均值作为默认空速融合，具有高方差
        ahrs.writeDefaultAirSpeed((float)((aparm.airspeed_min + aparm.airspeed_max)/2),
                                  (float)((aparm.airspeed_max - aparm.airspeed_min)/2));
    }

    // 同步MAVLink系统ID
    mavlink_system.sysid = g.sysid_this_mav;

    SRV_Channels::enable_aux_servos();

    // 更新通知标志
    AP_Notify::flags.pre_arm_check = arming.pre_arm_checks(false);
    AP_Notify::flags.pre_arm_gps_check = true;
    AP_Notify::flags.armed = arming.is_armed() || arming.arming_required() == AP_Arming::Required::NO;

#if AP_TERRAIN_AVAILABLE && HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_GPS)) {
        terrain.log_terrain_data();
    }
#endif

    // 如果未解锁且GPS位置已更改，则更新家位置。最多每5秒更新一次
    if (!arming.is_armed() &&
        gps.last_message_time_ms() - last_home_update_ms > 5000 &&
        gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
            last_home_update_ms = gps.last_message_time_ms();
            update_home();
            
            // 重置着陆高度校正
            landing.alt_offset = 0;
    }

    // 确保G_Dt正确，捕获构造函数调用调度程序方法时的启动问题
    if (!is_equal(1.0f/scheduler.get_loop_rate_hz(), scheduler.get_loop_period_s()) ||
        !is_equal(G_Dt, scheduler.get_loop_period_s())) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    const float loop_rate = AP::scheduler().get_filtered_loop_rate_hz();
#if HAL_QUADPLANE_ENABLED
    if (quadplane.available()) {
        quadplane.attitude_control->set_notch_sample_rate(loop_rate);
    }
#endif
    rollController.set_notch_sample_rate(loop_rate);
    pitchController.set_notch_sample_rate(loop_rate);
    yawController.set_notch_sample_rate(loop_rate);
}

void Plane::three_hz_loop()
{
#if AP_FENCE_ENABLED
    fence_check();
#endif
}

void Plane::compass_save()
{
    if (AP::compass().available() &&
        compass.get_learn_type() >= Compass::LEARN_INTERNAL &&
        !hal.util->get_soft_armed()) {
        /*
          仅在解除武装时保存偏移量
         */
        compass.save_offsets();
    }
}

#if AP_AIRSPEED_AUTOCAL_ENABLE
/*
  每秒更新一次空速校准比率
 */
void Plane::airspeed_ratio_update(void)
{
    if (!airspeed.enabled() ||
        gps.status() < AP_GPS::GPS_OK_FIX_3D ||
        gps.ground_speed() < 4) {
        // 不移动时不校准
        return;        
    }
    if (airspeed.get_airspeed() < aparm.airspeed_min && 
        gps.ground_speed() < (uint32_t)aparm.airspeed_min) {
        // 飞行速度低于最小空速时不校准。我们
        // 同时检查空速和地速，以捕获空速比
        // 过低的情况，这可能导致它
        // 永远不会再上升
        return;
    }
    if (labs(ahrs.roll_sensor) > roll_limit_cd ||
        ahrs.pitch_sensor > aparm.pitch_limit_max*100 ||
        ahrs.pitch_sensor < pitch_limit_min*100) {
        // 超出正常飞行包络时不校准
        return;
    }
    const Vector3f &vg = gps.velocity();
    airspeed.update_calibration(vg, aparm.airspeed_max);
}
#endif // AP_AIRSPEED_AUTOCAL_ENABLE
/*
  读取GPS并更新位置
 */
void Plane::update_GPS_50Hz(void)
{
    gps.update();

    update_current_loc();
}

/*
  读取更新GPS位置 - 10Hz更新
 */
void Plane::update_GPS_10Hz(void)
{
    static uint32_t last_gps_msg_ms;
    if (gps.last_message_time_ms() != last_gps_msg_ms && gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        last_gps_msg_ms = gps.last_message_time_ms();

        if (ground_start_count > 1) {
            ground_start_count--;
        } else if (ground_start_count == 1) {
            // 我们倒计时N次良好的GPS定位
            // 以使高度更准确
            // -------------------------------------
            if (current_loc.lat == 0 && current_loc.lng == 0) {
                ground_start_count = 5;

            } else if (!hal.util->was_watchdog_reset()) {
                if (!set_home_persistently(gps.location())) {
                    // 静默忽略失败...
                }

                next_WP_loc = prev_WP_loc = home;

                ground_start_count = 0;
            }
        }

        // 更新风速估计
        ahrs.estimate_wind();
    } else if (gps.status() < AP_GPS::GPS_OK_FIX_3D && ground_start_count != 0) {
        // 失去3D定位，重新开始
        ground_start_count = 5;
    }

    calc_gndspeed_undershoot();
}

/*
  主控制模式依赖的更新代码
 */
void Plane::update_control_mode(void)
{
    if (control_mode != &mode_auto) {
        // hold_course仅在起飞和着陆时使用
        steer_state.hold_course_cd = -1;
    }

    update_fly_forward();

    control_mode->update();
}


void Plane::update_fly_forward(void)
{
    // 确保我们在作为纯固定翼飞行器飞行时保持前飞
    // 这有助于EKF产生更好的状态估计，因为它可以做出更强的假设
#if HAL_QUADPLANE_ENABLED
    if (quadplane.available() &&
        quadplane.tailsitter.is_in_fw_flight()) {
        ahrs.set_fly_forward(true);
        return;
    }

    if (quadplane.in_vtol_mode() ||
        quadplane.in_assisted_flight()) {
        ahrs.set_fly_forward(false);
        return;
    }
#endif

    if (auto_state.idle_mode) {
        // 在气球升力模式下不融合空速
        ahrs.set_fly_forward(false);
        return;
    }

    if (flight_stage == AP_FixedWing::FlightStage::LAND) {
        ahrs.set_fly_forward(landing.is_flying_forward());
        return;
    }

    ahrs.set_fly_forward(true);
}

/*
  设置飞行阶段
 */
void Plane::set_flight_stage(AP_FixedWing::FlightStage fs)
{
    if (fs == flight_stage) {
        return;
    }

    landing.handle_flight_stage_change(fs == AP_FixedWing::FlightStage::LAND);

    if (fs == AP_FixedWing::FlightStage::ABORT_LANDING) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "Landing aborted, climbing to %dm",
                        int(auto_state.takeoff_altitude_rel_cm/100));
    }

    flight_stage = fs;
#if HAL_LOGGING_ENABLED
    Log_Write_Status();
#endif
}

void Plane::update_alt()
{
    barometer.update();

    // 计算下沉率
    float sink_rate;
    Vector3f vel;
    if (ahrs.get_velocity_NED(vel)) {
        sink_rate = vel.z;
    } else if (gps.status() >= AP_GPS::GPS_OK_FIX_3D && gps.have_vertical_velocity()) {
        sink_rate = gps.velocity().z;
    } else {
        sink_rate = -barometer.get_climb_rate();        
    }

    // 对下沉率进行低通滤波以消除一些噪声
    auto_state.sink_rate = 0.8f * auto_state.sink_rate + 0.2f*sink_rate;
#if HAL_PARACHUTE_ENABLED
    parachute.set_sink_rate(auto_state.sink_rate);
#endif

    update_flight_stage();

#if AP_SCRIPTING_ENABLED
    if (nav_scripting_active()) {
        // 在执行特技动作时不调用TECS
        return;
    }
#endif

    bool should_run_tecs = control_mode->does_auto_throttle();
#if HAL_QUADPLANE_ENABLED
    if (quadplane.should_disable_TECS()) {
        should_run_tecs = false;
    }
#endif

    if (auto_state.idle_mode) {
        should_run_tecs = false;
    }

#if AP_PLANE_GLIDER_PULLUP_ENABLED
    if (mode_auto.in_pullup()) {
        should_run_tecs = false;
    }
#endif
    
    if (should_run_tecs && !throttle_suppressed) {

        float distance_beyond_land_wp = 0;
        if (flight_stage == AP_FixedWing::FlightStage::LAND &&
            current_loc.past_interval_finish_line(prev_WP_loc, next_WP_loc)) {
            distance_beyond_land_wp = current_loc.get_distance(next_WP_loc);
        }

        tecs_target_alt_cm = relative_target_altitude_cm();

        if (control_mode == &mode_rtl && !rtl.done_climb && (g2.rtl_climb_min > 0 || (plane.flight_option_enabled(FlightOptions::CLIMB_BEFORE_TURN)))) {
            // 确保我们在RTL模式下进行初始爬升。我们在要求的高度上再增加10m
            // 以推动TECS快速爬升
            tecs_target_alt_cm = MAX(tecs_target_alt_cm, prev_WP_loc.alt - home.alt) + (g2.rtl_climb_min+10)*100;
        }

        TECS_controller.update_pitch_throttle(tecs_target_alt_cm,
                                                 target_airspeed_cm,
                                                 flight_stage,
                                                 distance_beyond_land_wp,
                                                 get_takeoff_pitch_min_cd(),
                                                 throttle_nudge,
                                                 tecs_hgt_afe(),
                                                 aerodynamic_load_factor,
                                                 g.pitch_trim.get());
    }
}
/*
  重新计算飞行阶段
 */
void Plane::update_flight_stage(void)
{
    // 更新速度和高度控制器状态
    if (control_mode->does_auto_throttle() && !throttle_suppressed) {
        if (control_mode == &mode_auto) {
#if HAL_QUADPLANE_ENABLED
            if (quadplane.in_vtol_auto()) {
                set_flight_stage(AP_FixedWing::FlightStage::VTOL);
                return;
            }
#endif
            if (auto_state.takeoff_complete == false) {
                set_flight_stage(AP_FixedWing::FlightStage::TAKEOFF);
                return;
            } else if (mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND) {
                if (landing.is_commanded_go_around() || flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING) {
                    // 中止模式是粘性的，必须在执行NAV_LAND时完成
                    set_flight_stage(AP_FixedWing::FlightStage::ABORT_LANDING);
                } else if (landing.get_abort_throttle_enable() && get_throttle_input() >= 90 &&
                           landing.request_go_around()) {
                    gcs().send_text(MAV_SEVERITY_INFO,"通过油门中止着陆");
                    set_flight_stage(AP_FixedWing::FlightStage::ABORT_LANDING);
                } else {
                    set_flight_stage(AP_FixedWing::FlightStage::LAND);
                }
                return;
            }
#if HAL_QUADPLANE_ENABLED
            if (quadplane.in_assisted_flight()) {
                set_flight_stage(AP_FixedWing::FlightStage::VTOL);
                return;
            }
#endif
            set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
        } else if (control_mode != &mode_takeoff) {
            // 如果不在AUTO模式，则假设正常操作以进行正常TECS操作。
            // 这可以防止TECS在错误的阶段卡住，比如在着陆、中止着陆或起飞时从AUTO切换到FBWB。
            set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
        }
        return;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode() ||
        quadplane.in_assisted_flight()) {
        set_flight_stage(AP_FixedWing::FlightStage::VTOL);
        return;
    }
#endif
    set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
}

/*
    如果启用了land_DisarmDelay（非零），检查着陆后在时间到期后自动解除武装

    仅在着陆库准备解除武装时由AP_Landing调用
 */
void Plane::disarm_if_autoland_complete()
{
    if (landing.get_disarm_delay() > 0 &&
        !is_flying() &&
        arming.arming_required() != AP_Arming::Required::NO &&
        arming.is_armed()) {
        /* 我们启用了自动解除武装。检查是否已经过去足够的时间 */
        if (millis() - auto_state.last_flying_ms >= landing.get_disarm_delay()*1000UL) {
            if (arming.disarm(AP_Arming::Method::AUTOLANDED)) {
                gcs().send_text(MAV_SEVERITY_INFO,"自动解除武装");
            }
        }
    }
}

bool Plane::trigger_land_abort(const float climb_to_alt_m)
{
    if (plane.control_mode != &plane.mode_auto) {
        return false;
    }
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.in_vtol_auto()) {
        return quadplane.abort_landing();
    }
#endif

    uint16_t mission_id = plane.mission.get_current_nav_cmd().id;
    bool is_in_landing = (plane.flight_stage == AP_FixedWing::FlightStage::LAND) ||
        plane.is_land_command(mission_id);
    if (is_in_landing) {
        // 如果可用，执行用户计划的中止模式
        if (plane.have_position && plane.mission.jump_to_abort_landing_sequence(plane.current_loc)) {
            return true;
        }

        // 只有在不执行四旋翼操作或可能进行四旋翼进场时才执行固定翼中止
#if HAL_QUADPLANE_ENABLED
        const bool attempt_go_around =
            (!plane.quadplane.available()) ||
            ((!plane.quadplane.in_vtol_auto()) &&
                (!plane.quadplane.landing_with_fixed_wing_spiral_approach()));
#else
        const bool attempt_go_around = true;
#endif
        if (attempt_go_around) {
            // 初始化中止着陆。这将触发抬头和爬升到安全高度保持航向，然后执行以下操作之一，按此顺序检查：
            // - 如果任务中的下一个命令是MAV_CMD_CONTINUE_AND_CHANGE_ALT，
            //   增加任务索引以执行它
            // - 否则如果DO_LAND_START可用，跳转到它
            // - 否则减少任务索引以重复着陆进场

            if (!is_zero(climb_to_alt_m)) {
                plane.auto_state.takeoff_altitude_rel_cm = climb_to_alt_m * 100;
            }
            if (plane.landing.request_go_around()) {
                plane.auto_state.next_wp_crosstrack = false;
                return true;
            }
        }
    }
    return false;
}

/*
  传递给TECS的相对于场地高度的高度
 */
float Plane::tecs_hgt_afe(void)
{
    /*
      在着陆时，将相对于场地高度的高度作为相对于地面的高度传递，
      这意味着TECS获取测距仪信息，因此可以知道何时开始拉平。
    */
    float hgt_afe;
    if (flight_stage == AP_FixedWing::FlightStage::LAND) {
        hgt_afe = height_above_target();
#if AP_RANGEFINDER_ENABLED
        hgt_afe -= rangefinder_correction();
#endif
    } else {
        // 在正常飞行中，我们将hgt_afe作为相对于家的高度传递
        hgt_afe = relative_altitude;
    }
    return hgt_afe;
}

// 特定于车辆的航点信息辅助函数
bool Plane::get_wp_distance_m(float &distance) const
{
    // 参见 GCS_MAVLINK_Plane::send_nav_controller_output()
    if (control_mode == &mode_manual) {
        return false;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode()) {
        distance = quadplane.using_wp_nav() ? quadplane.wp_nav->get_wp_distance_to_destination() * 0.01 : 0;
        return true;
    }
#endif
    distance = auto_state.wp_distance;
    return true;
}

bool Plane::get_wp_bearing_deg(float &bearing) const
{
    // 参见 GCS_MAVLINK_Plane::send_nav_controller_output()
    if (control_mode == &mode_manual) {
        return false;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode()) {
        bearing = quadplane.using_wp_nav() ? quadplane.wp_nav->get_wp_bearing_to_destination() : 0;
        return true;
    }
#endif
    bearing = nav_controller->target_bearing_cd() * 0.01;
    return true;
}

bool Plane::get_wp_crosstrack_error_m(float &xtrack_error) const
{
    // 参见 GCS_MAVLINK_Plane::send_nav_controller_output()
    if (control_mode == &mode_manual) {
        return false;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode()) {
        xtrack_error = quadplane.using_wp_nav() ? quadplane.wp_nav->crosstrack_error() : 0;
        return true;
    }
#endif
    xtrack_error = nav_controller->crosstrack_error();
    return true;
}
#if AP_SCRIPTING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED
// 设置目标位置（用于外部控制和脚本）
bool Plane::set_target_location(const Location &target_loc)
{
    Location loc{target_loc};

    if (plane.control_mode != &plane.mode_guided) {
        // 仅在引导模式下接受位置更新
        return false;
    }
    // 如果需要，添加家的高度
    if (loc.relative_alt) {
        loc.alt += plane.home.alt;
        loc.relative_alt = 0;
    }
    plane.set_guided_WP(loc);
    return true;
}
#endif //AP_SCRIPTING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED

#if AP_SCRIPTING_ENABLED
// 获取目标位置（用于脚本）
bool Plane::get_target_location(Location& target_loc)
{
    switch (control_mode->mode_number()) {
    case Mode::Number::RTL:
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
    case Mode::Number::AUTO:
    case Mode::Number::LOITER:
    case Mode::Number::TAKEOFF:
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QLOITER:
    case Mode::Number::QLAND:
    case Mode::Number::QRTL:
#endif
        target_loc = next_WP_loc;
        return true;
        break;
    default:
        break;
    }
    return false;
}

// 更新目标位置（适用于所有自动导航模式）
bool Plane::update_target_location(const Location &old_loc, const Location &new_loc)
{
    // 通过检查调用者提供的旧目标位置是否正确，
    // 防止用户在控制lua脚本中更改模式或命令不同目标时的竞态条件
    if (!old_loc.same_loc_as(next_WP_loc) ||
        old_loc.get_alt_frame() != new_loc.get_alt_frame()) {
        return false;
    }
    next_WP_loc = new_loc;

#if HAL_QUADPLANE_ENABLED
    if (control_mode == &mode_qland || control_mode == &mode_qloiter) {
        mode_qloiter.last_target_loc_set_ms = AP_HAL::millis();
    }
#endif

    return true;
}

// 允许在VTOL模式下进行速度匹配
bool Plane::set_velocity_match(const Vector2f &velocity)
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode() || quadplane.in_vtol_land_sequence()) {
        quadplane.poscontrol.velocity_match = velocity;
        quadplane.poscontrol.last_velocity_match_ms = AP_HAL::millis();
        return true;
    }
#endif
    return false;
}

// 允许覆盖着陆下降率
bool Plane::set_land_descent_rate(float descent_rate)
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_land_descent() ||
        control_mode == &mode_qland) {
        quadplane.poscontrol.override_descent_rate = descent_rate;
        quadplane.poscontrol.last_override_descent_ms = AP_HAL::millis();
        return true;
    }
#endif
    return false;
}

// 允许脚本控制退出和恢复任务或引导飞行时的交叉跟踪
// Lua脚本负责确保提供的位置是合理的
bool Plane::set_crosstrack_start(const Location &new_start_location)
{        
    prev_WP_loc = new_start_location;
    auto_state.crosstrack = true;
    return true;
}

#endif // AP_SCRIPTING_ENABLED

// 返回飞机是否正在着陆
bool Plane::is_landing() const
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.in_vtol_land_descent()) {
        return true;
    }
#endif
    return control_mode->is_landing();
}

// 返回飞机是否正在起飞
bool Plane::is_taking_off() const
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.in_vtol_takeoff()) {
        return true;
    }
#endif
    return control_mode->is_taking_off();
}

// 在非VTOL模式下修正AHRS俯仰角以适应PTCH_TRIM_DEG，并在VTOL模式下返回VTOL视图
void Plane::get_osd_roll_pitch_rad(float &roll, float &pitch) const
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.show_vtol_view()) {
        pitch = quadplane.ahrs_view->pitch;
        roll = quadplane.ahrs_view->roll;
        return;
    }
#endif
    pitch = ahrs.get_pitch();
    roll = ahrs.get_roll();
    if (!(flight_option_enabled(FlightOptions::OSD_REMOVE_TRIM_PITCH))) {  // 修正PTCH_TRIM_DEG
        pitch -= g.pitch_trim * DEG_TO_RAD;
    }
}

// 更新当前位置
void Plane::update_current_loc(void)
{
    have_position = plane.ahrs.get_location(plane.current_loc);

    // 重新计算相对高度
    ahrs.get_relative_position_D_home(plane.relative_altitude);
    relative_altitude *= -1.0f;
}

// 检查FLIGHT_OPTION是否启用
bool Plane::flight_option_enabled(FlightOptions flight_option) const
{
    return g2.flight_options & flight_option;
}

#if AC_PRECLAND_ENABLED
// 更新精确着陆
void Plane::precland_update(void)
{
    // 如果我们将false作为第二个参数传递，alt将不会被使用：
#if AP_RANGEFINDER_ENABLED
    return g2.precland.update(rangefinder_state.height_estimate*100, rangefinder_state.in_range);
#else
    return g2.precland.update(0, false);
#endif
}
#endif

AP_HAL_MAIN_CALLBACKS(&plane);
