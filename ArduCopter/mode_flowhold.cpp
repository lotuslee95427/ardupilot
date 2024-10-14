#include "Copter.h"
#include <utility>

#if MODE_FLOWHOLD_ENABLED

/*
  implement FLOWHOLD mode, for position hold using optical flow
  without rangefinder
  实现FLOWHOLD模式,使用光流传感器进行位置保持,无需测距仪
 */

const AP_Param::GroupInfo ModeFlowHold::var_info[] = {
    // @Param: _XY_P
    // @DisplayName: FlowHold P gain
    // @Description: FlowHold (horizontal) P gain.
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced
    // FlowHold水平方向P增益

    // @Param: _XY_I
    // @DisplayName: FlowHold I gain
    // @Description: FlowHold (horizontal) I gain
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced
    // FlowHold水平方向I增益

    // @Param: _XY_IMAX
    // @DisplayName: FlowHold Integrator Max
    // @Description: FlowHold (horizontal) integrator maximum
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cdeg
    // @User: Advanced
    // FlowHold水平方向积分器最大值

    // @Param: _XY_FILT_HZ
    // @DisplayName: FlowHold filter on input to control
    // @Description: FlowHold (horizontal) filter on input to control
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced
    // FlowHold水平方向控制输入滤波器
    AP_SUBGROUPINFO(flow_pi_xy, "_XY_",  1, ModeFlowHold, AC_PI_2D),

    // @Param: _FLOW_MAX
    // @DisplayName: FlowHold Flow Rate Max
    // @Description: Controls maximum apparent flow rate in flowhold
    // @Range: 0.1 2.5
    // @User: Standard
    // 控制FlowHold模式下的最大视觉流速率
    AP_GROUPINFO("_FLOW_MAX", 2, ModeFlowHold, flow_max, 0.6),

    // @Param: _FILT_HZ
    // @DisplayName: FlowHold Filter Frequency
    // @Description: Filter frequency for flow data
    // @Range: 1 100
    // @Units: Hz
    // @User: Standard
    // 光流数据的滤波频率
    AP_GROUPINFO("_FILT_HZ", 3, ModeFlowHold, flow_filter_hz, 5),

    // @Param: _QUAL_MIN
    // @DisplayName: FlowHold Flow quality minimum
    // @Description: Minimum flow quality to use flow position hold
    // @Range: 0 255
    // @User: Standard
    // 使用光流进行位置保持的最小光流质量
    AP_GROUPINFO("_QUAL_MIN", 4, ModeFlowHold, flow_min_quality, 10),

    // 5 was FLOW_SPEED

    // @Param: _BRAKE_RATE
    // @DisplayName: FlowHold Braking rate
    // @Description: Controls deceleration rate on stick release
    // @Range: 1 30
    // @User: Standard
    // @Units: deg/s
    // 控制摇杆释放时的减速率
    AP_GROUPINFO("_BRAKE_RATE", 6, ModeFlowHold, brake_rate_dps, 8),

    AP_GROUPEND
};

ModeFlowHold::ModeFlowHold(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

#define CONTROL_FLOWHOLD_EARTH_FRAME 0

// flowhold_init - 初始化flowhold控制器
bool ModeFlowHold::init(bool ignore_checks)
{
    if (!copter.optflow.enabled() || !copter.optflow.healthy()) {
        return false;
    }

    // 设置垂直速度和加速度限制
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // 初始化垂直位置控制器
    if (!copter.pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // 设置光流滤波器的截止频率
    flow_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), flow_filter_hz.get());

    quality_filtered = 0;
    flow_pi_xy.reset_I();
    limited = false;

    // 设置PI控制器的dt
    flow_pi_xy.set_dt(1.0/copter.scheduler.get_loop_rate_hz());

    // 从INS获取初始高度
    last_ins_height = copter.inertial_nav.get_position_z_up_cm() * 0.01;
    height_offset = 0;

    return true;
}

/*
  计算从光流传感器得到的期望姿态。当光流传感器健康时调用
 */
void ModeFlowHold::flowhold_flow_to_angle(Vector2f &bf_angles, bool stick_input)
{
    uint32_t now = AP_HAL::millis();

    // 获取校正后的原始光流速率
    Vector2f raw_flow = copter.optflow.flowRate() - copter.optflow.bodyRate();

    // 限制传感器光流,防止低高度时的振荡
    raw_flow.x = constrain_float(raw_flow.x, -flow_max, flow_max);
    raw_flow.y = constrain_float(raw_flow.y, -flow_max, flow_max);

    // 滤波光流速率
    Vector2f sensor_flow = flow_filter.apply(raw_flow);

    // 根据高度估计缩放,限制高度在height_min到height_max之间
    float ins_height = copter.inertial_nav.get_position_z_up_cm() * 0.01;
    float height_estimate = ins_height + height_offset;

    // 补偿高度,将单位转换为(近似)m/s
    sensor_flow *= constrain_float(height_estimate, height_min, height_max);

    // 将控制器输入旋转到地球坐标系
    Vector2f input_ef = copter.ahrs.body_to_earth2D(sensor_flow);

    // 运行PI控制器
    flow_pi_xy.set_input(input_ef);

    // 获取地球坐标系下的控制器姿态(单位:厘度)
    Vector2f ef_output;

    // 获取P项
    ef_output = flow_pi_xy.get_p();

    if (stick_input) {
        last_stick_input_ms = now;
        braking = true;
    }
    if (!stick_input && braking) {
        // 如果3秒过去或速度低于0.3m/s,停止制动
        if (now - last_stick_input_ms > 3000 || sensor_flow.length() < 0.3) {
            braking = false;
#if 0
            printf("braking done at %u vel=%f\n", now - last_stick_input_ms,
                   (double)sensor_flow.length());
#endif
        }
    }

    if (!stick_input && !braking) {
        // 获取I项
        if (limited) {
            // 只允许I项长度缩小
            xy_I = flow_pi_xy.get_i_shrink();
        } else {
            // 正常I项操作
            xy_I = flow_pi_xy.get_pi();
        }
    }

    if (!stick_input && braking) {
        // 分别计算每个轴的制动角度
        for (uint8_t i=0; i<2; i++) {
            float &velocity = sensor_flow[i];
            float abs_vel_cms = fabsf(velocity)*100;
            const float brake_gain = (15.0f * brake_rate_dps.get() + 95.0f) * 0.01f;
            float lean_angle_cd = brake_gain * abs_vel_cms * (1.0f+500.0f/(abs_vel_cms+60.0f));
            if (velocity < 0) {
                lean_angle_cd = -lean_angle_cd;
            }
            bf_angles[i] = lean_angle_cd;
        }
        ef_output.zero();
    }

    ef_output += xy_I;
    ef_output *= copter.aparm.angle_max;

    // 转换到机体坐标系
    bf_angles += copter.ahrs.earth_to_body2D(ef_output);

    // 设置限制标志以防止积分器饱和
    limited = fabsf(bf_angles.x) > copter.aparm.angle_max || fabsf(bf_angles.y) > copter.aparm.angle_max;

    // 限制到最大角度
    bf_angles.x = constrain_float(bf_angles.x, -copter.aparm.angle_max, copter.aparm.angle_max);
    bf_angles.y = constrain_float(bf_angles.y, -copter.aparm.angle_max, copter.aparm.angle_max);

#if HAL_LOGGING_ENABLED
// @LoggerMessage: FHLD
// @Description: FlowHold mode messages
// @URL: https://ardupilot.org/copter/docs/flowhold-mode.html
// @Field: TimeUS: Time since system startup
// @Field: SFx: Filtered flow rate, X-Axis
// @Field: SFy: Filtered flow rate, Y-Axis
// @Field: Ax: Target lean angle, X-Axis
// @Field: Ay: Target lean angle, Y-Axis
// @Field: Qual: Flow sensor quality. If this value falls below FHLD_QUAL_MIN parameter, FlowHold will act just like AltHold.
// @Field: Ix: Integral part of PI controller, X-Axis
// @Field: Iy: Integral part of PI controller, Y-Axis

    if (log_counter++ % 20 == 0) {
        AP::logger().WriteStreaming("FHLD", "TimeUS,SFx,SFy,Ax,Ay,Qual,Ix,Iy", "Qfffffff",
                                               AP_HAL::micros64(),
                                               (double)sensor_flow.x, (double)sensor_flow.y,
                                               (double)bf_angles.x, (double)bf_angles.y,
                                               (double)quality_filtered,
                                               (double)xy_I.x, (double)xy_I.y);
    }
#endif  // HAL_LOGGING_ENABLED
}

// flowhold_run - 运行flowhold控制器
// 应该以100Hz或更高的频率调用
void ModeFlowHold::run()
{
    update_height_estimate();

    // 设置垂直速度和加速度限制
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // 应用SIMPLE模式变换到飞行员输入
    update_simple_mode();

    // 检查滤波器变化
    if (!is_equal(flow_filter.get_cutoff_freq(), flow_filter_hz.get())) {
        flow_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), flow_filter_hz.get());
    }

    // 获取飞行员期望的爬升速率
    float target_climb_rate = copter.get_pilot_desired_climb_rate(copter.channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), copter.g.pilot_speed_up);

    // 获取飞行员期望的偏航速率
    float target_yaw_rate = get_pilot_desired_yaw_rate();

    // 确定FlowHold状态机状态
    AltHoldModeState flowhold_state = get_alt_hold_state(target_climb_rate);

    if (copter.optflow.healthy()) {
        const float filter_constant = 0.95;
        quality_filtered = filter_constant * quality_filtered + (1-filter_constant) * copter.optflow.quality();
    } else {
        quality_filtered = 0;
    }

    // FlowHold状态机
    switch (flowhold_state) {

    case AltHoldModeState::MotorStopped:
        copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        copter.attitude_control->reset_rate_controller_I_terms();
        copter.attitude_control->reset_yaw_target_and_rate();
        copter.pos_control->relax_z_controller(0.0f);   // 强制油门输出衰减到零
        flow_pi_xy.reset_I();
        break;

    case AltHoldModeState::Takeoff:
        // 将电机设置为全范围
        copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // 初始化起飞
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // 获取经过避障调整的爬升速率
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // 设置根据飞行员输入调整的位置控制器目标
        takeoff.do_pilot_takeoff(target_climb_rate);
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_z_controller(0.0f);   // 强制油门输出衰减到零
        break;

    case AltHoldModeState::Flying:
        copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // 获取经过避障调整的爬升速率
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

#if AP_RANGEFINDER_ENABLED
        // 根据表面测量更新垂直偏移
        copter.surface_tracking.update_surface_offset();
#endif

        // 将命令的爬升速率发送到位置控制器
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        break;
    }

    // flowhold姿态目标计算
    Vector2f bf_angles;

    // 计算定高角度
    int16_t roll_in = copter.channel_roll->get_control_in();
    int16_t pitch_in = copter.channel_pitch->get_control_in();
    float angle_max = copter.aparm.angle_max;
    get_pilot_desired_lean_angles(bf_angles.x, bf_angles.y, angle_max, attitude_control->get_althold_lean_angle_max_cd());

    if (quality_filtered >= flow_min_quality &&
        AP_HAL::millis() - copter.arm_time_ms > 3000) {
        // 在起飞后的前3秒内不使用
        Vector2f flow_angles;

        flowhold_flow_to_angle(flow_angles, (roll_in != 0) || (pitch_in != 0));
        flow_angles.x = constrain_float(flow_angles.x, -angle_max/2, angle_max/2);
        flow_angles.y = constrain_float(flow_angles.y, -angle_max/2, angle_max/2);
        bf_angles += flow_angles;
    }
    bf_angles.x = constrain_float(bf_angles.x, -angle_max, angle_max);
    bf_angles.y = constrain_float(bf_angles.y, -angle_max, angle_max);

#if AP_AVOIDANCE_ENABLED
    // 应用避障
    copter.avoid.adjust_roll_pitch(bf_angles.x, bf_angles.y, copter.aparm.angle_max);
#endif

    // 调用姿态控制器
    copter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(bf_angles.x, bf_angles.y, target_yaw_rate);

    // 运行垂直位置控制器并设置输出油门
    pos_control->update_z_controller();
}

/*
  使用与光流集成的加速度计比率更新高度估计
 */
void ModeFlowHold::update_height_estimate(void)
{
    float ins_height = copter.inertial_nav.get_position_z_up_cm() * 0.01;

#if 1
    // 当解除武装或刚开始启动电机时,假设在地面上
    if (!hal.util->get_soft_armed() ||
        copter.motors->get_desired_spool_state() != AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED ||
        AP_HAL::millis() - copter.arm_time_ms < 1500) {
        height_offset = -ins_height;
        last_ins_height = ins_height;
        return;
    }
#endif

    // 获取机体坐标系下的速度增量
    Vector3f delta_vel;
    float delta_vel_dt;
    if (!copter.ins.get_delta_velocity(delta_vel, delta_vel_dt)) {
        return;
    }

    // 将速度增量积分到地球坐标系
    const Matrix3f &rotMat = copter.ahrs.get_rotation_body_to_ned();
    delta_vel = rotMat * delta_vel;
    delta_velocity_ne.x += delta_vel.x;
    delta_velocity_ne.y += delta_vel.y;

    if (!copter.optflow.healthy()) {
        // 没有光流传感器时无法更新高度模型
        last_flow_ms = AP_HAL::millis();
        delta_velocity_ne.zero();
        return;
    }

    if (last_flow_ms == 0) {
        // 刚开始启动
        last_flow_ms = copter.optflow.last_update();
        delta_velocity_ne.zero();
        height_offset = 0;
        return;
    }

    if (copter.optflow.last_update() == last_flow_ms) {
        // 没有新的光流数据
        return;
    }

    // 将速度增量转换回机体坐标系以匹配光流传感器
    Vector2f delta_vel_bf = copter.ahrs.earth_to_body2D(delta_velocity_ne);

    // 并转换为等效速率,以便与光流可比
    Vector2f delta_vel_rate(-delta_vel_bf.y, delta_vel_bf.x);

    // 获取机体坐标系下的光流速率(弧度/秒)
    Vector2f flow_rate_rps = copter.optflow.flowRate() - copter.optflow.bodyRate();

    uint32_t dt_ms = copter.optflow.last_update() - last_flow_ms;
    if (dt_ms > 500) {
        // 更新间隔太长,忽略
        last_flow_ms = copter.optflow.last_update();
        delta_velocity_ne.zero();
        last_flow_rate_rps = flow_rate_rps;
        last_ins_height = ins_height;
        height_offset = 0;
        return;        
    }

    /*
      基本方程是:
      height_m = delta_velocity_mps / delta_flowrate_rps;
     */

    // 获取delta_flowrate_rps
    Vector2f delta_flowrate = flow_rate_rps - last_flow_rate_rps;
    last_flow_rate_rps = flow_rate_rps;
    last_flow_ms = copter.optflow.last_update();

    /*
      更新高度估计
     */
    const float min_velocity_change = 0.04;
    const float min_flow_change = 0.04;
    const float height_delta_max = 0.25;

    /*
      对每个轴更新高度估计
     */
    float delta_height = 0;
    uint8_t total_weight = 0;
    float height_estimate = ins_height + height_offset;

    for (uint8_t i=0; i<2; i++) {
        // 只在有显著速度增量和显著光流增量时使用高度估计
        float abs_flow = fabsf(delta_flowrate[i]);
        if (abs_flow < min_flow_change ||
            fabsf(delta_vel_rate[i]) < min_velocity_change) {
            continue;
        }
        // 获取瞬时高度估计
        float height = delta_vel_rate[i] / delta_flowrate[i];
        if (height <= 0) {
            // 丢弃负高度
            continue;
        }
        delta_height += (height - height_estimate) * abs_flow;
        total_weight += abs_flow;
    }
    if (total_weight > 0) {
        delta_height /= total_weight;
    }

    if (delta_height < 0) {
        // 偏向较低高度,因为我们宁愿增益过低也不要出现振荡
        // 这也部分补偿了上面丢弃负高度的影响
        delta_height *= 2;
    }

    // 高度变化不超过height_delta_max,这是一种简单的噪声抑制方法
    float new_offset = height_offset + constrain_float(delta_height, -height_delta_max, height_delta_max);

    // 应用简单滤波
    height_offset = 0.8 * height_offset + 0.2 * new_offset;

    if (ins_height + height_offset < height_min) {
        // 高度估计永远不允许低于最小值
        height_offset = height_min - ins_height;
    }

    // 用于记录的新高度估计
    height_estimate = ins_height + height_offset;

#if HAL_LOGGING_ENABLED
// @LoggerMessage: FHXY
// @Description: Height estimation using optical flow sensor 
// @Field: TimeUS: Time since system startup
// @Field: DFx: Delta flow rate, X-Axis
// @Field: DFy: Delta flow rate, Y-Axis
// @Field: DVx: Integrated delta velocity rate, X-Axis
// @Field: DVy: Integrated delta velocity rate, Y-Axis
// @Field: Hest: Estimated Height
// @Field: DH: Delta Height
// @Field: Hofs: Height offset
// @Field: InsH: Height estimate from inertial navigation library
// @Field: LastInsH: Last used INS height in optical flow sensor height estimation calculations 
// @Field: DTms: Time between optical flow sensor updates. This should be less than 500ms for performing the height estimation calculations

    AP::logger().WriteStreaming("FHXY", "TimeUS,DFx,DFy,DVx,DVy,Hest,DH,Hofs,InsH,LastInsH,DTms", "QfffffffffI",
                                           AP_HAL::micros64(),
                                           (double)delta_flowrate.x,
                                           (double)delta_flowrate.y,
                                           (double)delta_vel_rate.x,
                                           (double)delta_vel_rate.y,
                                           (double)height_estimate,
                                           (double)delta_height,
                                           (double)height_offset,
                                           (double)ins_height,
                                           (double)last_ins_height,
                                           dt_ms);
#endif

    gcs().send_named_float("HEST", height_estimate);
    delta_velocity_ne.zero();
    last_ins_height = ins_height;
}

#endif // MODE_FLOWHOLD_ENABLED
