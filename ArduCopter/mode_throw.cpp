#include "Copter.h"

#if MODE_THROW_ENABLED

// throw_init - 初始化抛投控制器
bool ModeThrow::init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
    // 不允许直升机使用抛投启动
    return false;
#endif

    // 当已经解锁或正在飞行时不进入该模式
    if (motors->armed()) {
        return false;
    }

    // 初始化状态
    stage = Throw_Disarmed;
    nextmode_attempted = false;

    // 初始化位置控制器的速度和加速度
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), BRAKE_MODE_DECEL_RATE);
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), BRAKE_MODE_DECEL_RATE);

    // 设置垂直速度和加速度限制
    pos_control->set_max_speed_accel_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);
    pos_control->set_correction_speed_accel_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);

    return true;
}

// 运行抛投启动控制器
// 应该以100Hz或更高的频率调用
void ModeThrow::run()
{
    /* 抛投状态机
    Throw_Disarmed - 电机关闭
    Throw_Detecting - 电机开启，等待抛投
    Throw_Uprighting - 检测到抛投，飞行器正在调整姿态
    Throw_HgtStabilise - 飞行器保持水平，高度稳定在目标高度
    Throw_PosHold - 飞行器保持在固定位置和高度
    */

    if (!motors->armed()) {
        // 状态机入口始终是从解除武装状态开始
        stage = Throw_Disarmed;

    } else if (stage == Throw_Disarmed && motors->armed()) {
        gcs().send_text(MAV_SEVERITY_INFO,"等待抛投");
        stage = Throw_Detecting;

    } else if (stage == Throw_Detecting && throw_detected()){
        gcs().send_text(MAV_SEVERITY_INFO,"检测到抛投 - 电机加速中");
        copter.set_land_complete(false);
        stage = Throw_Wait_Throttle_Unlimited;

        // 取消等待抛投的音调序列
        AP_Notify::flags.waiting_for_throw = false;

    } else if (stage == Throw_Wait_Throttle_Unlimited &&
               motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        gcs().send_text(MAV_SEVERITY_INFO,"油门已解锁 - 正在调整姿态");
        stage = Throw_Uprighting;
    } else if (stage == Throw_Uprighting && throw_attitude_good()) {
        gcs().send_text(MAV_SEVERITY_INFO,"姿态已调整 - 控制高度中");
        stage = Throw_HgtStabilise;

        // 初始化Z轴控制器
        pos_control->init_z_controller_no_descent();

        // 初始化目标高度为抛投高度上方3米
        // 我们希望快速清除周围障碍物
        if (g2.throw_type == ThrowType::Drop) {
            pos_control->set_pos_target_z_cm(inertial_nav.get_position_z_up_cm() - 100);
        } else {
            pos_control->set_pos_target_z_cm(inertial_nav.get_position_z_up_cm() + 300);
        }

        // 将auto_arm状态设置为true，以避免选择自动模式时油门最小导致的可能的自动解除武装
        copter.set_auto_armed(true);

    } else if (stage == Throw_HgtStabilise && throw_height_good()) {
        gcs().send_text(MAV_SEVERITY_INFO,"已达到目标高度 - 控制位置中");
        stage = Throw_PosHold;

        // 初始化位置控制器
        pos_control->init_xy_controller();

        // 将auto_arm状态设置为true，以避免选择自动模式时油门最小导致的可能的自动解除武装
        copter.set_auto_armed(true);
    } else if (stage == Throw_PosHold && throw_position_good()) {
        if (!nextmode_attempted) {
            switch ((Mode::Number)g2.throw_nextmode.get()) {
                case Mode::Number::AUTO:
                case Mode::Number::GUIDED:
                case Mode::Number::RTL:
                case Mode::Number::LAND:
                case Mode::Number::BRAKE:
                case Mode::Number::LOITER:
                    set_mode((Mode::Number)g2.throw_nextmode.get(), ModeReason::THROW_COMPLETE);
                    break;
                default:
                    // 不做任何操作
                    break;
            }
            nextmode_attempted = true;
        }
    }

    // 抛投状态处理
    switch (stage) {

    case Throw_Disarmed:

        // 除非用户启用，否则在检测到抛投之前防止电机旋转
        if (g.throw_motor_start == PreThrowMotorState::RUNNING) {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        } else {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        }

        // 要求零油门（电机无论如何都会停止）并持续重置姿态控制器
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        break;

    case Throw_Detecting:

        // 除非用户启用，否则在检测到抛投之前防止电机旋转
        if (g.throw_motor_start == PreThrowMotorState::RUNNING) {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        } else {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        }

        // 在抛投过程中保持油门为零，并持续重置姿态控制器
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_throttle_out(0,true,g.throttle_filt);

        // 播放等待抛投的音调序列以提醒用户
        AP_Notify::flags.waiting_for_throw = true;

        break;

    case Throw_Wait_Throttle_Unlimited:

        // 将电机设置为全范围
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        break;

    case Throw_Uprighting:

        // 将电机设置为全范围
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // 要求水平的横滚/俯仰姿态，偏航速率为零
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);

        // 输出50%油门并关闭角度增强以最大化调整力矩
        attitude_control->set_throttle_out(0.5f, false, g.throttle_filt);

        break;

    case Throw_HgtStabilise:

        // 将电机设置为全范围
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // 调用姿态控制器
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);

        // 调用高度控制器
        pos_control->set_pos_target_z_from_climb_rate_cm(0.0f);
        pos_control->update_z_controller();

        break;

    case Throw_PosHold:

        // 将电机设置为全范围
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // 使用位置控制器停止
        Vector2f vel;
        Vector2f accel;
        pos_control->input_vel_accel_xy(vel, accel);
        pos_control->update_xy_controller();

        // 调用姿态控制器
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), 0.0f);

        // 调用高度控制器
        pos_control->set_pos_target_z_from_climb_rate_cm(0.0f);
        pos_control->update_z_controller();

        break;
    }

#if HAL_LOGGING_ENABLED
    // 每10Hz或状态改变时记录日志
    uint32_t now = AP_HAL::millis();
    if ((stage != prev_stage) || (now - last_log_ms) > 100) {
        prev_stage = stage;
        last_log_ms = now;
        const float velocity = inertial_nav.get_velocity_neu_cms().length();
        const float velocity_z = inertial_nav.get_velocity_z_up_cms();
        const float accel = copter.ins.get_accel().length();
        const float ef_accel_z = ahrs.get_accel_ef().z;
        const bool throw_detect = (stage > Throw_Detecting) || throw_detected();
        const bool attitude_ok = (stage > Throw_Uprighting) || throw_attitude_good();
        const bool height_ok = (stage > Throw_HgtStabilise) || throw_height_good();
        const bool pos_ok = (stage > Throw_PosHold) || throw_position_good();

// @LoggerMessage: THRO
// @Description: 抛投模式消息
// @URL: https://ardupilot.org/copter/docs/throw-mode.html
// @Field: TimeUS: 系统启动以来的时间
// @Field: Stage: 抛投模式的当前阶段
// @Field: Vel: 速度向量的大小
// @Field: VelZ: 垂直速度
// @Field: Acc: 当前加速度向量的大小
// @Field: AccEfZ: 地球坐标系下的垂直加速度值
// @Field: Throw: 如果在进入此模式后检测到抛投，则为True
// @Field: AttOk: 如果飞行器处于直立状态，则为True
// @Field: HgtOk: 如果飞行器在要求高度的50cm范围内，则为True
// @Field: PosOk: 如果飞行器在要求水平位置的50cm范围内，则为True

        // 使用WriteStreaming()方法记录抛投模式的状态数据
        // 参数说明:
        // "THRO" - 日志消息的标识符
        // "TimeUS,Stage,Vel..." - 记录的字段名称列表
        // "s-nnoo----" - 单位类型:s=秒,n=米/秒,o=米/秒^2,-=无单位
        // "F-0000----" - 乘数:F=微秒,0=无乘数,-=无乘数
        // "QBffffbbbb" - 数据类型:Q=uint64_t,B=uint8_t,f=float,b=bool
        AP::logger().WriteStreaming(
            "THRO",                                          // 消息标识符
            "TimeUS,Stage,Vel,VelZ,Acc,AccEfZ,Throw,AttOk,HgtOk,PosOk", // 字段名称
            "s-nnoo----",                                    // 单位类型
            "F-0000----",                                    // 乘数
            "QBffffbbbb",                                    // 数据类型
            AP_HAL::micros64(),                             // 当前系统时间(微秒)
            (uint8_t)stage,                                 // 当前抛投阶段
            (double)velocity,                               // 总速度(m/s)
            (double)velocity_z,                             // 垂直速度(m/s)
            (double)accel,                                  // 总加速度(m/s^2)
            (double)ef_accel_z,                            // 地球坐标系下的垂直加速度(m/s^2)
            throw_detect,                                   // 是否检测到抛投
            attitude_ok,                                    // 姿态是否正常
            height_ok,                                      // 高度是否在范围内
            pos_ok);                                        // 位置是否在范围内
    }
#endif  // HAL_LOGGING_ENABLED
}

bool ModeThrow::throw_detected()
{
    // 检查我们是否有有效的导航解决方案
    nav_filter_status filt_status = inertial_nav.get_filter_status();
    if (!filt_status.flags.attitude || !filt_status.flags.horiz_pos_abs || !filt_status.flags.vert_pos) {
        return false;
    }

    // 检查高速（>500 cm/s）
    bool high_speed = inertial_nav.get_velocity_neu_cms().length_squared() > (THROW_HIGH_SPEED * THROW_HIGH_SPEED);

    // 检查向上或向下的轨迹（空投）是否为50cm/s
    bool changing_height;
    if (g2.throw_type == ThrowType::Drop) {
        changing_height = inertial_nav.get_velocity_z_up_cms() < -THROW_VERTICAL_SPEED;
    } else {
        changing_height = inertial_nav.get_velocity_z_up_cms() > THROW_VERTICAL_SPEED;
    }

    // 检查垂直加速度是否大于0.25g
    bool free_falling = ahrs.get_accel_ef().z > -0.25 * GRAVITY_MSS;

    // 检查加速度长度是否<1.0g，表明任何抛投动作已完成，飞行器已被释放
    bool no_throw_action = copter.ins.get_accel().length() < 1.0f * GRAVITY_MSS;

    // 获取相对于家的高度
    float altitude_above_home;  // 如果设置了家的位置，则使用相对于家的高度，否则使用相对于EKF原点的高度
    if (ahrs.home_is_set()) {
        ahrs.get_relative_position_D_home(altitude_above_home);
        altitude_above_home = -altitude_above_home; // 相对于家的高度返回为负值
    } else {
        altitude_above_home = inertial_nav.get_position_z_up_cm() * 0.01f; // 厘米转换为米
    }

    // 检查高度是否在用户定义的限制范围内
    const bool height_within_params = (g.throw_altitude_min == 0 || altitude_above_home > g.throw_altitude_min) && (g.throw_altitude_max == 0 || (altitude_above_home < g.throw_altitude_max));

    // 高速或自由落体与高度增加相结合表明可能存在空投或抛投释放  
    bool possible_throw_detected = (free_falling || high_speed) && changing_height && no_throw_action && height_within_params;


    // 当我们检测到可能的抛投时，记录时间和垂直速度
    if (possible_throw_detected && ((AP_HAL::millis() - free_fall_start_ms) > 500)) {
        free_fall_start_ms = AP_HAL::millis();
        free_fall_start_velz = inertial_nav.get_velocity_z_up_cms();
    }

    // 一旦检测到可能的抛投条件，我们检查在不到0.5秒内是否有2.5 m/s的向下速度变化以确认
    bool throw_condition_confirmed = ((AP_HAL::millis() - free_fall_start_ms < 500) && ((inertial_nav.get_velocity_z_up_cms() - free_fall_start_velz) < -250.0f));

    // 如果我们处于持续自由落体状态，则启动电机并进入控制模式
    return throw_condition_confirmed;
}

bool ModeThrow::throw_attitude_good() const
{
    // 检查我们是否已经使飞行器直立
    const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
    return (rotMat.c.z > 0.866f); // is_upright
}

bool ModeThrow::throw_height_good() const
{
    // 检查我们是否在要求高度的0.5m范围内
    return (pos_control->get_pos_error_z_cm() < 50.0f);
}

bool ModeThrow::throw_position_good() const
{
    // 检查我们的水平位置误差是否在50cm内
    return (pos_control->get_pos_error_xy_cm() < 50.0f);
}

#endif
