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

/*
 *  ArduCopter (也被称为 APM, APM:Copter 或简称 Copter)
 *  Wiki:           copter.ardupilot.org
 *  创建者:        Jason Short
 *  首席开发者: Randy Mackay
 *  首席测试员:    Marco Robustini
 *  基于 Arducopter 团队的代码和想法: Leonard Hall, Andrew Tridgell, Robert Lefebvre, Pat Hickey, Michael Oborne, Jani Hirvinen,
                                                      Olivier Adler, Kevin Hester, Arthur Benemann, Jonathan Challinger, John Arne Birkeland,
                                                      Jean-Louis Naudin, Mike Smith, 等等
 *  感谢: Chris Anderson, Jordi Munoz, Jason Short, Doug Weibel, Jose Julio
 *
 *  特别感谢贡献者 (按名字字母顺序排列):
 *
 *  Adam M Rivera       :自动罗盘偏角
 *  Amilcar Lucas       :相机云台库
 *  Andrew Tridgell     :总体开发, Mavlink 支持
 *  Andy Piper          :谐波陷波滤波器, 飞行中 FFT, 双向 DShot, 各种驱动程序
 *  Angel Fernandez     :Alpha 测试
 *  AndreasAntonopoulous:地理围栏
 *  Arthur Benemann     :DroidPlanner 地面站
 *  Benjamin Pelletier  :库
 *  Bill King           :单旋翼直升机
 *  Christof Schmid     :Alpha 测试
 *  Craig Elder         :发布管理, 支持
 *  Dani Saez           :V 型八旋翼支持
 *  Doug Weibel         :DCM, 库, 控制律建议
 *  Emile Castelnuovo   :VRBrain 移植, bug 修复
 *  Gregory Fletcher    :相机云台方向数学
 *  Guntars             :解锁安全建议
 *  HappyKillmore       :Mavlink 地面站
 *  Hein Hollander      :八旋翼支持, 直升机测试
 *  Igor van Airde      :控制律优化
 *  Jack Dunkle         :Alpha 测试
 *  James Goppert       :Mavlink 支持
 *  Jani Hiriven        :测试反馈
 *  Jean-Louis Naudin   :自动着陆
 *  John Arne Birkeland :PPM 编码器
 *  Jose Julio          :稳定控制律, MPU6k 驱动
 *  Julien Dubois       :PosHold 飞行模式
 *  Julian Oes          :Pixhawk
 *  Jonathan Challinger :惯性导航, CompassMot, 解锁时旋转
 *  Kevin Hester        :Andropilot 地面站
 *  Max Levine          :三旋翼支持, 图形
 *  Leonard Hall        :飞行动力学, 油门, 悬停和导航控制器
 *  Marco Robustini     :首席测试员
 *  Michael Oborne      :Mission Planner 地面站
 *  Mike Smith          :Pixhawk 驱动, 编码支持
 *  Olivier Adler       :PPM 编码器, 压电蜂鸣器
 *  Pat Hickey          :硬件抽象层 (HAL)
 *  Robert Lefebvre     :直升机支持, Copter LED
 *  Roberto Navoni      :库测试, 移植到 VRBrain
 *  Sandro Benigno      :相机支持, MinimOSD
 *  Sandro Tognana      :PosHold 飞行模式
 *  Sebastian Quilter   :SmartRTL
 *  ..以及更多人.
 *
 *  代码提交统计可以在这里找到: https://github.com/ArduPilot/ardupilot/graphs/contributors
 *  Wiki: https://copter.ardupilot.org/
 *
 */

#include "Copter.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define SCHED_TASK(func, _interval_ticks, _max_time_micros, _prio) SCHED_TASK_CLASS(Copter, &copter, func, _interval_ticks, _max_time_micros, _prio)
#define FAST_TASK(func) FAST_TASK_CLASS(Copter, &copter, func)

/*
  调度器表 - 所有任务都应该列在这里。

  此表中的所有条目必须按优先级排序。

  此表与 AP_Vehicle 中的表交错，以确定任务运行的顺序。
  提供了便利方法 SCHED_TASK 和 SCHED_TASK_CLASS 来构建此结构中的条目:

SCHED_TASK 参数:
 - 要调用的静态函数的名称
 - 应该调用该函数的频率 (以赫兹为单位)
 - 函数运行应该花费的预期时间 (以微秒为单位)
 - 优先级 (0 到 255，数字越小表示优先级越高)

SCHED_TASK_CLASS 参数:
 - 要调用的方法的类名
 - 要在其上调用方法的实例
 - 要在该实例上调用的方法
 - 应该调用该方法的频率 (以赫兹为单位)
 - 方法运行应该花费的预期时间 (以微秒为单位)
 - 优先级 (0 到 255，数字越小表示优先级越高)

 */
const AP_Scheduler::Task Copter::scheduler_tasks[] = {
    // 立即更新 INS 以获取当前陀螺仪数据
    FAST_TASK_CLASS(AP_InertialSensor, &copter.ins, update),
    // 运行只需要 IMU 数据的低级速率控制器
    FAST_TASK(run_rate_controller),
#if AC_CUSTOMCONTROL_MULTI_ENABLED
    FAST_TASK(run_custom_controller),
#endif
#if FRAME_CONFIG == HELI_FRAME
    FAST_TASK(heli_update_autorotation),
#endif //HELI_FRAME
    // 立即向电机库发送输出
    FAST_TASK(motors_output),
     // 运行 EKF 状态估计器 (耗时)
    FAST_TASK(read_AHRS),
#if FRAME_CONFIG == HELI_FRAME
    FAST_TASK(update_heli_control_dynamics),
#endif //HELI_FRAME
    // 惯性导航
    FAST_TASK(read_inertia),
    // 检查 ekf 是否重置了目标航向或位置
    FAST_TASK(check_ekf_reset),
    // 运行姿态控制器
    FAST_TASK(update_flight_mode),
    // 如有必要，从 EKF 更新 home 位置
    FAST_TASK(update_home_from_EKF),
    // 检查我们是否已着陆或坠毁
    FAST_TASK(update_land_and_crash_detectors),
    // 表面跟踪更新
    FAST_TASK(update_rangefinder_terrain_offset),
#if HAL_MOUNT_ENABLED
    // 相机云台的快速更新
    FAST_TASK_CLASS(AP_Mount, &copter.camera_mount, update_fast),
#endif
#if HAL_LOGGING_ENABLED
    FAST_TASK(Log_Video_Stabilisation),
#endif

    SCHED_TASK(rc_loop,              250,    130,  3),
    SCHED_TASK(throttle_loop,         50,     75,  6),
#if AP_FENCE_ENABLED
    SCHED_TASK(fence_check,           25,    100,  7),
#endif
    SCHED_TASK_CLASS(AP_GPS,               &copter.gps,                 update,          50, 200,   9),
#if AP_OPTICALFLOW_ENABLED
    SCHED_TASK_CLASS(AP_OpticalFlow,          &copter.optflow,             update,         200, 160,  12),
#endif
    SCHED_TASK(update_batt_compass,   10,    120, 15),
    SCHED_TASK_CLASS(RC_Channels, (RC_Channels*)&copter.g2.rc_channels, read_aux_all,    10,  50,  18),
    SCHED_TASK(arm_motors_check,      10,     50, 21),
#if TOY_MODE_ENABLED
    SCHED_TASK_CLASS(ToyMode,              &copter.g2.toy_mode,         update,          10,  50,  24),
#endif
    SCHED_TASK(auto_disarm_check,     10,     50,  27),
    SCHED_TASK(auto_trim,             10,     75,  30),
#if AP_RANGEFINDER_ENABLED
    SCHED_TASK(read_rangefinder,      20,    100,  33),
#endif
#if HAL_PROXIMITY_ENABLED
    SCHED_TASK_CLASS(AP_Proximity,         &copter.g2.proximity,        update,         200,  50,  36),
#endif
#if AP_BEACON_ENABLED
    SCHED_TASK_CLASS(AP_Beacon,            &copter.g2.beacon,           update,         400,  50,  39),
#endif
    SCHED_TASK(update_altitude,       10,    100,  42),
    SCHED_TASK(run_nav_updates,       50,    100,  45),
    SCHED_TASK(update_throttle_hover,100,     90,  48),
#if MODE_SMARTRTL_ENABLED
    SCHED_TASK_CLASS(ModeSmartRTL,         &copter.mode_smartrtl,       save_position,    3, 100,  51),
#endif
#if HAL_SPRAYER_ENABLED
    SCHED_TASK_CLASS(AC_Sprayer,           &copter.sprayer,               update,         3,  90,  54),
#endif
    SCHED_TASK(three_hz_loop,          3,     75, 57),
#if AP_SERVORELAYEVENTS_ENABLED
    SCHED_TASK_CLASS(AP_ServoRelayEvents,  &copter.ServoRelayEvents,      update_events, 50,  75,  60),
#endif
#if AC_PRECLAND_ENABLED
    SCHED_TASK(update_precland,      400,     50,  69),
#endif
#if FRAME_CONFIG == HELI_FRAME
    SCHED_TASK(check_dynamic_flight,  50,     75,  72),
#endif
#if HAL_LOGGING_ENABLED
    SCHED_TASK(loop_rate_logging, LOOP_RATE,    50,  75),
#endif
    SCHED_TASK(one_hz_loop,            1,    100,  81),
    SCHED_TASK(ekf_check,             10,     75,  84),
    SCHED_TASK(check_vibration,       10,     50,  87),
    SCHED_TASK(gpsglitch_check,       10,     50,  90),
    SCHED_TASK(takeoff_check,         50,     50,  91),
#if AP_LANDINGGEAR_ENABLED
    SCHED_TASK(landinggear_update,    10,     75,  93),
#endif
    SCHED_TASK(standby_update,        100,    75,  96),
    SCHED_TASK(lost_vehicle_check,    10,     50,  99),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&copter._gcs,          update_receive, 400, 180, 102),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&copter._gcs,          update_send,    400, 550, 105),
#if HAL_MOUNT_ENABLED
    SCHED_TASK_CLASS(AP_Mount,             &copter.camera_mount,        update,          50,  75, 108),
#endif
#if AP_CAMERA_ENABLED
    SCHED_TASK_CLASS(AP_Camera,            &copter.camera,              update,          50,  75, 111),
#endif
#if HAL_LOGGING_ENABLED
    SCHED_TASK(ten_hz_logging_loop,   10,    350, 114),
    SCHED_TASK(twentyfive_hz_logging, 25,    110, 117),
    SCHED_TASK_CLASS(AP_Logger,            &copter.logger,              periodic_tasks, 400, 300, 120),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor,    &copter.ins,                 periodic,       400,  50, 123),

#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(AP_Scheduler,         &copter.scheduler,           update_logging, 0.1,  75, 126),
#endif
#if AP_RPM_ENABLED
    SCHED_TASK_CLASS(AP_RPM,               &copter.rpm_sensor,          update,          40, 200, 129),
#endif
#if AP_TEMPCALIBRATION_ENABLED
    SCHED_TASK_CLASS(AP_TempCalibration,   &copter.g2.temp_calibration, update,          10, 100, 135),
#endif
#if HAL_ADSB_ENABLED
    SCHED_TASK(avoidance_adsb_update, 10,    100, 138),
#endif
#if ADVANCED_FAILSAFE
    SCHED_TASK(afs_fs_check,          10,    100, 141),
#endif
#if AP_TERRAIN_AVAILABLE
    SCHED_TASK(terrain_update,        10,    100, 144),
#endif
#if AP_WINCH_ENABLED
    SCHED_TASK_CLASS(AP_Winch,             &copter.g2.winch,            update,          50,  50, 150),
#endif
#ifdef USERHOOK_FASTLOOP
    SCHED_TASK(userhook_FastLoop,    100,     75, 153),
#endif
#ifdef USERHOOK_50HZLOOP
    SCHED_TASK(userhook_50Hz,         50,     75, 156),
#endif
#ifdef USERHOOK_MEDIUMLOOP
    SCHED_TASK(userhook_MediumLoop,   10,     75, 159),
#endif
#ifdef USERHOOK_SLOWLOOP
    SCHED_TASK(userhook_SlowLoop,      3.3,   75, 162),
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    SCHED_TASK(userhook_SuperSlowLoop, 1,     75, 165),
#endif
#if HAL_BUTTON_ENABLED
    SCHED_TASK_CLASS(AP_Button,            &copter.button,              update,           5, 100, 168),
#endif
};

// 获取调度器任务
void Copter::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                 uint8_t &task_count,
                                 uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}

constexpr int8_t Copter::_failsafe_priorities[7];


#if AP_SCRIPTING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED
#if MODE_GUIDED_ENABLED
// 设置目标位置 (用于外部控制和脚本)
bool Copter::set_target_location(const Location& target_loc)
{
    // 如果车辆不在引导模式或自动引导模式，则退出
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    return mode_guided.set_destination(target_loc);
}
#endif //MODE_GUIDED_ENABLED
#endif //AP_SCRIPTING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED

#if AP_SCRIPTING_ENABLED
#if MODE_GUIDED_ENABLED
// 开始起飞到给定高度 (用于脚本)
bool Copter::start_takeoff(float alt)
{
    // 如果车辆不在引导模式或自动引导模式，则退出
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    if (mode_guided.do_user_takeoff_start(alt * 100.0f)) {
        copter.set_auto_armed(true);
        return true;
    }
    return false;
}

// 设置目标位置 (用于脚本)
bool Copter::set_target_pos_NED(const Vector3f& target_pos, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool yaw_relative, bool terrain_alt)
{
    // 如果车辆不在引导模式或自动引导模式，则退出
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3f pos_neu_cm(target_pos.x * 100.0f, target_pos.y * 100.0f, -target_pos.z * 100.0f);

    return mode_guided.set_destination(pos_neu_cm, use_yaw, yaw_deg * 100.0, use_yaw_rate, yaw_rate_degs * 100.0, yaw_relative, terrain_alt);
}

// 设置目标位置和速度 (用于脚本)
bool Copter::set_target_posvel_NED(const Vector3f& target_pos, const Vector3f& target_vel)
{
    // 如果车辆不在引导模式或自动引导模式，则退出
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3f pos_neu_cm(target_pos.x * 100.0f, target_pos.y * 100.0f, -target_pos.z * 100.0f);
    const Vector3f vel_neu_cms(target_vel.x * 100.0f, target_vel.y * 100.0f, -target_vel.z * 100.0f);

    return mode_guided.set_destination_posvelaccel(pos_neu_cm, vel_neu_cms, Vector3f());
}

// 设置目标位置、速度和加速度 (用于脚本)
bool Copter::set_target_posvelaccel_NED(const Vector3f& target_pos, const Vector3f& target_vel, const Vector3f& target_accel, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool yaw_relative)
{
    // 如果车辆不在引导模式或自动引导模式，则退出
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3f pos_neu_cm(target_pos.x * 100.0f, target_pos.y * 100.0f, -target_pos.z * 100.0f);
    const Vector3f vel_neu_cms(target_vel.x * 100.0f, target_vel.y * 100.0f, -target_vel.z * 100.0f);
    const Vector3f accel_neu_cms(target_accel.x * 100.0f, target_accel.y * 100.0f, -target_accel.z * 100.0f);

    return mode_guided.set_destination_posvelaccel(pos_neu_cm, vel_neu_cms, accel_neu_cms, use_yaw, yaw_deg * 100.0, use_yaw_rate, yaw_rate_degs * 100.0, yaw_relative);
}

// 设置目标速度 (用于脚本)
bool Copter::set_target_velocity_NED(const Vector3f& vel_ned)
{
    // 如果车辆不在引导模式或自动引导模式，则退出
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    // 将向量转换为 neu，单位为 cm/s
    const Vector3f vel_neu_cms(vel_ned.x * 100.0f, vel_ned.y * 100.0f, -vel_ned.z * 100.0f);
    mode_guided.set_velocity(vel_neu_cms);
    return true;
}

// 设置目标速度和加速度 (用于脚本)
bool Copter::set_target_velaccel_NED(const Vector3f& target_vel, const Vector3f& target_accel, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool relative_yaw)
{
    // 如果车辆不在引导模式或自动引导模式，则退出
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    // 将向量转换为 NEU 坐标系，单位为 cm/s 和 cm/s/s
    const Vector3f vel_neu_cms(target_vel.x * 100.0f, target_vel.y * 100.0f, -target_vel.z * 100.0f);
    const Vector3f accel_neu_cms(target_accel.x * 100.0f, target_accel.y * 100.0f, -target_accel.z * 100.0f);

    // 调用引导模式的 set_velaccel 函数设置目标速度和加速度
    mode_guided.set_velaccel(vel_neu_cms, accel_neu_cms, use_yaw, yaw_deg * 100.0, use_yaw_rate, yaw_rate_degs * 100.0, relative_yaw);
    return true;
}

// 设置目标滚转、俯仰和偏航角度以及爬升率 (用于脚本)
bool Copter::set_target_angle_and_climbrate(float roll_deg, float pitch_deg, float yaw_deg, float climb_rate_ms, bool use_yaw_rate, float yaw_rate_degs)
{
    // 如果车辆不在引导模式或自动引导模式，则退出
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    // 创建四元数并从欧拉角转换
    Quaternion q;
    q.from_euler(radians(roll_deg),radians(pitch_deg),radians(yaw_deg));

    // 调用引导模式的 set_angle 函数设置目标姿态和爬升率
    mode_guided.set_angle(q, Vector3f{}, climb_rate_ms*100, false);
    return true;
}

// 设置目标滚转、俯仰和偏航角速率以及油门 (用于脚本)
bool Copter::set_target_rate_and_throttle(float roll_rate_dps, float pitch_rate_dps, float yaw_rate_dps, float throttle)
{
    // 如果车辆不在引导模式或自动引导模式，则退出
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    // 创建零四元数表示角速率控制
    Quaternion q;
    q.zero();

    // 将角速率从度/秒转换为弧度/秒
    Vector3f ang_vel_body { roll_rate_dps, pitch_rate_dps, yaw_rate_dps };
    ang_vel_body *= DEG_TO_RAD;

    // 调用引导模式的 set_angle 函数设置目标角速率和油门
    mode_guided.set_angle(q, ang_vel_body, throttle, true);
    return true;
}
#endif

#if MODE_CIRCLE_ENABLED
// 圆形模式控制

// 获取圆形半径
bool Copter::get_circle_radius(float &radius_m)
{
    radius_m = circle_nav->get_radius() * 0.01f;
    return true;
}

// 设置圆形速率
bool Copter::set_circle_rate(float rate_dps)
{
    circle_nav->set_rate(rate_dps);
    return true;
}
#endif

// 设置期望速度 (m/s)。用于脚本。
bool Copter::set_desired_speed(float speed)
{
    return flightmode->set_speed_xy(speed * 100.0f);
}

#if MODE_AUTO_ENABLED
// 返回模式是否支持 NAV_SCRIPT_TIME 任务命令
bool Copter::nav_scripting_enable(uint8_t mode)
{
    return mode == (uint8_t)mode_auto.mode_number();
}

// Lua 脚本使用此函数检索活动命令的内容
bool Copter::nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2, int16_t &arg3, int16_t &arg4)
{
    if (flightmode != &mode_auto) {
        return false;
    }

    return mode_auto.nav_script_time(id, cmd, arg1, arg2, arg3, arg4);
}

// Lua 脚本使用此函数指示命令已完成
void Copter::nav_script_time_done(uint16_t id)
{
    if (flightmode != &mode_auto) {
        return;
    }

    return mode_auto.nav_script_time_done(id);
}
#endif

// 返回 EKF 故障保护是否已触发。仅供 Lua 脚本使用
bool Copter::has_ekf_failsafed() const
{
    return failsafe.ekf;
}

// 获取目标位置 (供脚本使用)
bool Copter::get_target_location(Location& target_loc)
{
    return flightmode->get_wp(target_loc);
}

/*
  update_target_location() 作为 set_target_location 的包装器
 */
bool Copter::update_target_location(const Location &old_loc, const Location &new_loc)
{
    /*
      通过检查调用者是否提供了正确的旧目标位置，
      我们可以防止用户在控制 Lua 脚本中更改模式或命令不同目标时出现竞态条件
    */
    Location next_WP_loc;
    flightmode->get_wp(next_WP_loc);
    if (!old_loc.same_loc_as(next_WP_loc) ||
         old_loc.get_alt_frame() != new_loc.get_alt_frame()) {
        return false;
    }

    return set_target_location(new_loc);
}

#endif // AP_SCRIPTING_ENABLED

// 返回车辆是否正在着陆
bool Copter::is_landing() const
{
    return flightmode->is_landing();
}

// 返回车辆是否正在起飞
bool Copter::is_taking_off() const
{
    return flightmode->is_taking_off();
}

// 返回当前模式是否需要任务
bool Copter::current_mode_requires_mission() const
{
#if MODE_AUTO_ENABLED
        return flightmode == &mode_auto;
#else
        return false;
#endif
}

// rc_loops - 读取用户从发射机/接收机的输入
// 以 100Hz 的频率调用
void Copter::rc_loop()
{
    // 读取无线电和无线电上的三位开关
    // -----------------------------------------
    read_radio();
    rc().read_mode_switch();
}

// throttle_loop - 应以 50 Hz 运行
// ---------------------------
void Copter::throttle_loop()
{
    // 更新 throttle_low_comp 值 (控制油门与姿态控制的优先级)
    update_throttle_mix();

    // 检查 auto_armed 状态
    update_auto_armed();

#if FRAME_CONFIG == HELI_FRAME
    // 更新旋翼速度
    heli_update_rotor_speed_targets();

    // 更新传统直升机斜盘运动
    heli_update_landing_swash();
#endif

    // 补偿地面效应 (如果启用)
    update_ground_effect_detector();
    update_ekf_terrain_height_stable();
}

// update_batt_compass - 读取电池和罗盘
// 应以 10Hz 的频率调用
void Copter::update_batt_compass(void)
{
    // 在罗盘之前读取电池，因为它可能用于电机干扰补偿
    battery.read();

    if(AP::compass().available()) {
        // 用油门值更新罗盘 - 用于 compassmot
        compass.set_throttle(motors->get_throttle());
        compass.set_voltage(battery.voltage());
        compass.read();
    }
}

// 25Hz 日志记录 - 应以 25Hz 的频率运行
void Copter::twentyfive_hz_logging()
{
    // 如果需要记录快速姿态日志
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_EKF_POS();  // 记录 EKF 位置信息
    }

    // 如果需要记录 IMU 日志,但不需要快速 IMU 日志
    if (should_log(MASK_LOG_IMU) && !(should_log(MASK_LOG_IMU_FAST))) {
        AP::ins().Write_IMU();  // 记录 IMU 数据
    }

#if MODE_AUTOROTATE_ENABLED
    // 如果需要记录中等或快速姿态日志
    if (should_log(MASK_LOG_ATTITUDE_MED) || should_log(MASK_LOG_ATTITUDE_FAST)) {
        // 更新自转日志
        g2.arot.Log_Write_Autorotation();
    }
#endif
#if HAL_GYROFFT_ENABLED
    // 如果需要记录快速 FFT 日志
    if (should_log(MASK_LOG_FTN_FAST)) {
        gyro_fft.write_log_messages();  // 记录陀螺仪 FFT 数据
    }
#endif
}
#endif  // HAL_LOGGING_ENABLED

// 3Hz 循环 - 每秒运行 3 次
void Copter::three_hz_loop()
{
    // 检查是否失去与地面站的联系
    failsafe_gcs_check();

    // 检查是否失去地形数据
    failsafe_terrain_check();

    // 检查航位推算失效保护
    failsafe_deadreckon_check();

    // 更新基于发射机的飞行中调整
    tuning();

    // 根据高度检查是否应启用避障
    low_alt_avoidance();
}

// 1Hz 循环 - 每秒运行一次
void Copter::one_hz_loop()
{
#if HAL_LOGGING_ENABLED
    // 如果需要记录任何日志
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(LogDataID::AP_STATE, ap.value);  // 记录 AP 状态
    }
#endif

    // 如果电机未解锁
    if (!motors->armed()) {
        // 更新互锁使用状态
        update_using_interlock();

        // 检查用户是否更新了机架类别或类型
        motors->set_frame_class_and_type((AP_Motors::motor_frame_class)g2.frame_class.get(), (AP_Motors::motor_frame_type)g.frame_type.get());

#if FRAME_CONFIG != HELI_FRAME
        // 设置所有油门通道设置
        motors->update_throttle_range();
#endif
    }

    // 更新分配的功能并启用辅助舵机
    SRV_Channels::enable_aux_servos();

#if HAL_LOGGING_ENABLED
    // 记录地形数据
    terrain_logging();
#endif

#if HAL_ADSB_ENABLED
    // 设置 ADSB 是否在飞行状态
    adsb.set_is_flying(!ap.land_complete);
#endif

    // 更新飞行状态标志
    AP_Notify::flags.flying = !ap.land_complete;

    // 使用平均循环速率缓慢更新 PID 陷波器
    attitude_control->set_notch_sample_rate(AP::scheduler().get_filtered_loop_rate_hz());
    pos_control->get_accel_z_pid().set_notch_sample_rate(AP::scheduler().get_filtered_loop_rate_hz());
#if AC_CUSTOMCONTROL_MULTI_ENABLED
    custom_control.set_notch_sample_rate(AP::scheduler().get_filtered_loop_rate_hz());
#endif
}

// 初始化简单模式方位
void Copter::init_simple_bearing()
{
    // 捕获当前的 cos_yaw 和 sin_yaw 值
    simple_cos_yaw = ahrs.cos_yaw();
    simple_sin_yaw = ahrs.sin_yaw();

    // 初始化超级简单模式航向(即朝向家的航向)为简单模式航向的反方向
    super_simple_last_bearing = wrap_360_cd(ahrs.yaw_sensor+18000);
    super_simple_cos_yaw = simple_cos_yaw;
    super_simple_sin_yaw = simple_sin_yaw;

#if HAL_LOGGING_ENABLED
    // 记录简单模式方位
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(LogDataID::INIT_SIMPLE_BEARING, ahrs.yaw_sensor);
    }
#endif
}

// 更新简单模式 - 如果我们处于简单模式,则旋转飞行员输入
void Copter::update_simple_mode(void)
{
    float rollx, pitchx;

    // 如果没有新的无线电帧或不在简单模式,立即退出
    if (simple_mode == SimpleMode::NONE || !ap.new_radio_frame) {
        return;
    }

    // 标记无线电帧已被消耗
    ap.new_radio_frame = false;

    if (simple_mode == SimpleMode::SIMPLE) {
        // 通过初始简单航向(即朝北)旋转横滚、俯仰输入
        rollx = channel_roll->get_control_in()*simple_cos_yaw - channel_pitch->get_control_in()*simple_sin_yaw;
        pitchx = channel_roll->get_control_in()*simple_sin_yaw + channel_pitch->get_control_in()*simple_cos_yaw;
    }else{
        // 通过超级简单航向(朝向家的反方向)旋转横滚、俯仰输入
        rollx = channel_roll->get_control_in()*super_simple_cos_yaw - channel_pitch->get_control_in()*super_simple_sin_yaw;
        pitchx = channel_roll->get_control_in()*super_simple_sin_yaw + channel_pitch->get_control_in()*super_simple_cos_yaw;
    }

    // 将横滚、俯仰输入从朝北旋转到飞行器的视角
    channel_roll->set_control_in(rollx*ahrs.cos_yaw() + pitchx*ahrs.sin_yaw());
    channel_pitch->set_control_in(-rollx*ahrs.sin_yaw() + pitchx*ahrs.cos_yaw());
}

// 更新超级简单模式方位 - 根据位置调整简单方位
// 应在更新 home_bearing 后调用
void Copter::update_super_simple_bearing(bool force_update)
{
    if (!force_update) {
        if (simple_mode != SimpleMode::SUPERSIMPLE) {
            return;
        }
        if (home_distance() < SUPER_SIMPLE_RADIUS) {
            return;
        }
    }

    const int32_t bearing = home_bearing();

    // 检查朝向家的方位是否至少改变了 5 度
    if (labs(super_simple_last_bearing - bearing) < 500) {
        return;
    }

    super_simple_last_bearing = bearing;
    const float angle_rad = radians((super_simple_last_bearing+18000)/100);
    super_simple_cos_yaw = cosf(angle_rad);
    super_simple_sin_yaw = sinf(angle_rad);
}

// 读取 AHRS (姿态航向参考系统)
void Copter::read_AHRS(void)
{
    // 我们告诉 AHRS 跳过 INS 更新,因为我们已经在 FAST_TASK 中完成了
    ahrs.update(true);
}

// 读取气压计并记录控制调整
void Copter::update_altitude()
{
    // 读取气压高度
    read_barometer();

#if HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
        if (!should_log(MASK_LOG_FTN_FAST)) {
#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
            AP::ins().write_notch_log_messages();
#endif
#if HAL_GYROFFT_ENABLED
            gyro_fft.write_log_messages();
#endif
        }
    }
#endif
}

// 飞行器特定的航点信息辅助函数
bool Copter::get_wp_distance_m(float &distance) const
{
    // 参见 GCS_MAVLINK_Copter::send_nav_controller_output()
    distance = flightmode->wp_distance() * 0.01;
    return true;
}

// 飞行器特定的航点信息辅助函数
bool Copter::get_wp_bearing_deg(float &bearing) const
{
    // 参见 GCS_MAVLINK_Copter::send_nav_controller_output()
    bearing = flightmode->wp_bearing() * 0.01;
    return true;
}

// 飞行器特定的航点信息辅助函数
bool Copter::get_wp_crosstrack_error_m(float &xtrack_error) const
{
    // 参见 GCS_MAVLINK_Copter::send_nav_controller_output()
    xtrack_error = flightmode->crosstrack_error() * 0.01;
    return true;
}

// 获取目标地球坐标系角速度(rad/s)(Z轴分量被某些云台使用)
bool Copter::get_rate_ef_targets(Vector3f& rate_ef_targets) const
{
    // 如果已着陆或未解锁,始终返回零向量
    if (copter.ap.land_complete) {
        rate_ef_targets.zero();
    } else {
        rate_ef_targets = attitude_control->get_rate_ef_targets();
    }
    return true;
}

/*
  Copter 主类的构造函数
 */
Copter::Copter(void)
    :
    flight_modes(&g.flight_mode1),
    pos_variance_filt(FS_EKF_FILT_DEFAULT),
    vel_variance_filt(FS_EKF_FILT_DEFAULT),
    hgt_variance_filt(FS_EKF_FILT_DEFAULT),
    flightmode(&mode_stabilize),
    simple_cos_yaw(1.0f),
    super_simple_cos_yaw(1.0),
    land_accel_ef_filter(LAND_DETECTOR_ACCEL_LPF_CUTOFF),
    rc_throttle_control_in_filter(1.0f),
    inertial_nav(ahrs),
    param_loader(var_info)
{
}

Copter copter;
AP_Vehicle& vehicle = copter;

AP_HAL_MAIN_CALLBACKS(&copter);
