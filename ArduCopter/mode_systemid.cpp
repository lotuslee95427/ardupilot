#include "Copter.h"
#include <AP_Math/control.h>

#if MODE_SYSTEMID_ENABLED

/*
 * 系统识别飞行模式的初始化和运行调用
 */

const AP_Param::GroupInfo ModeSystemId::var_info[] = {

    // @Param: _AXIS
    // @DisplayName: System identification axis
    // @Description: Controls which axis are being excited.  Set to non-zero to see more parameters
    // @User: Standard
    // @Values: 0:None, 1:Input Roll Angle, 2:Input Pitch Angle, 3:Input Yaw Angle, 4:Recovery Roll Angle, 5:Recovery Pitch Angle, 6:Recovery Yaw Angle, 7:Rate Roll, 8:Rate Pitch, 9:Rate Yaw, 10:Mixer Roll, 11:Mixer Pitch, 12:Mixer Yaw, 13:Mixer Thrust, 14:Measured Lateral Position, 15:Measured Longitudinal Position, 16:Measured Lateral Velocity, 17:Measured Longitudinal Velocity, 18:Input Lateral Velocity, 19:Input Longitudinal Velocity
    // 系统识别轴参数，控制哪个轴被激励。设置为非零值以查看更多参数
    AP_GROUPINFO_FLAGS("_AXIS", 1, ModeSystemId, axis, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _MAGNITUDE
    // @DisplayName: System identification Chirp Magnitude
    // @Description: Magnitude of sweep in deg, deg/s and 0-1 for mixer outputs.
    // @User: Standard
    // 系统识别啁啾信号幅度，单位为度、度/秒，混合器输出为0-1
    AP_GROUPINFO("_MAGNITUDE", 2, ModeSystemId, waveform_magnitude, 15),

    // @Param: _F_START_HZ
    // @DisplayName: System identification Start Frequency
    // @Description: Frequency at the start of the sweep
    // @Range: 0.01 100
    // @Units: Hz
    // @User: Standard
    // 系统识别起始频率，单位为Hz
    AP_GROUPINFO("_F_START_HZ", 3, ModeSystemId, frequency_start, 0.5f),

    // @Param: _F_STOP_HZ
    // @DisplayName: System identification Stop Frequency
    // @Description: Frequency at the end of the sweep
    // @Range: 0.01 100
    // @Units: Hz
    // @User: Standard
    // 系统识别结束频率，单位为Hz
    AP_GROUPINFO("_F_STOP_HZ", 4, ModeSystemId, frequency_stop, 40),

    // @Param: _T_FADE_IN
    // @DisplayName: System identification Fade in time
    // @Description: Time to reach maximum amplitude of sweep
    // @Range: 0 20
    // @Units: s
    // @User: Standard
    // 系统识别渐入时间，达到最大幅度所需的时间，单位为秒
    AP_GROUPINFO("_T_FADE_IN", 5, ModeSystemId, time_fade_in, 15),

    // @Param: _T_REC
    // @DisplayName: System identification Total Sweep length
    // @Description: Time taken to complete the sweep
    // @Range: 0 255
    // @Units: s
    // @User: Standard
    // 系统识别总扫描时长，完成整个扫描所需的时间，单位为秒
    AP_GROUPINFO("_T_REC", 6, ModeSystemId, time_record, 70),

    // @Param: _T_FADE_OUT
    // @DisplayName: System identification Fade out time
    // @Description: Time to reach zero amplitude at the end of the sweep
    // @Range: 0 5
    // @Units: s
    // @User: Standard
    // 系统识别渐出时间，在扫描结束时达到零幅度所需的时间，单位为秒
    AP_GROUPINFO("_T_FADE_OUT", 7, ModeSystemId, time_fade_out, 2),

    AP_GROUPEND
};

ModeSystemId::ModeSystemId(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

#define SYSTEM_ID_DELAY     1.0f      // 系统识别模式改变后等待的时间（秒），用于频率扫描注入

// systemId_init - 初始化系统识别控制器
bool ModeSystemId::init(bool ignore_checks)
{
    // 检查是否启用
    if (axis == 0) {
        gcs().send_text(MAV_SEVERITY_WARNING, "No axis selected, SID_AXIS = 0");
        return false;
    }

    // 确保飞机正在飞行
    if (!copter.motors->armed() || !copter.ap.auto_armed || copter.ap.land_complete) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Aircraft must be flying");
        return false;
    }

    if (!is_poscontrol_axis_type()) {

        // 系统识别正在对姿态控制回路进行

        // 只能在具有手动油门的飞行模式下切换到系统识别轴1-13
        if (!copter.flightmode->has_manual_throttle()) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Axis requires manual throttle");
            return false;
        }

#if FRAME_CONFIG == HELI_FRAME
        copter.input_manager.set_use_stab_col(true);
#endif

    } else {

        // 系统识别正在对位置控制回路进行

        // 只能从悬停飞行模式切换到系统识别轴14-19
        if (copter.flightmode->mode_number() != Mode::Number::LOITER) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Axis requires switch from Loiter");
            return false;
        }

        // 设置水平速度和加速度限制
        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

        // 初始化水平位置控制器
        if (!pos_control->is_active_xy()) {
            pos_control->init_xy_controller();
        }

        // 设置垂直速度和加速度限制
        pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
        pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

        // 初始化垂直位置控制器
        if (!pos_control->is_active_z()) {
            pos_control->init_z_controller();
        }
        Vector3f curr_pos;
        curr_pos = inertial_nav.get_position_neu_cm();
        target_pos = curr_pos.xy();
    }

    att_bf_feedforward = attitude_control->get_bf_feedforward();
    waveform_time = 0.0f;
    time_const_freq = 2.0f / frequency_start; // 起始频率的两个完整周期
    systemid_state = SystemIDModeState::SYSTEMID_STATE_TESTING;
    log_subsample = 0;

    chirp_input.init(time_record, frequency_start, frequency_stop, time_fade_in, time_fade_out, time_const_freq);

    gcs().send_text(MAV_SEVERITY_INFO, "SystemID Starting: axis=%d", (unsigned)axis);

#if HAL_LOGGING_ENABLED
    copter.Log_Write_SysID_Setup(axis, waveform_magnitude, frequency_start, frequency_stop, time_fade_in, time_const_freq, time_record, time_fade_out);
#endif

    return true;
}

// systemId_exit - 退出系统识别控制器前进行清理
void ModeSystemId::exit()
{
    // 将前馈启用参数重置为初始化状态
    attitude_control->bf_feedforward(att_bf_feedforward);
}

// systemId_run - 运行系统识别控制器
// 应以100Hz或更高的频率调用
void ModeSystemId::run()
{
    float target_roll, target_pitch;
    float target_yaw_rate = 0.0f;
    float pilot_throttle_scaled = 0.0f;
    float target_climb_rate = 0.0f;
    Vector2f input_vel;

    if (!is_poscontrol_axis_type()) {

        // 对飞行员输入应用简单模式转换
        update_simple_mode();

        // 将飞行员输入转换为倾斜角度
        get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

        // 获取飞行员期望的偏航速率
        target_yaw_rate = get_pilot_desired_yaw_rate();

        if (!motors->armed()) {
            // 电机应该停止
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        // 传统直升机在油门摇杆为零时不将电机状态设置为地面怠速。只有在电机联锁禁用时才设置地面怠速。
        } else if (copter.ap.throttle_zero && !copter.is_tradheli()) {
            // 尝试着陆
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        } else {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        }

        switch (motors->get_spool_state()) {
        case AP_Motors::SpoolState::SHUT_DOWN:
            // 电机停止
            attitude_control->reset_yaw_target_and_rate();
            attitude_control->reset_rate_controller_I_terms();
            break;

        case AP_Motors::SpoolState::GROUND_IDLE:
            // 已着陆
            // 传统直升机在从解除武装到武装状态时初始化目标。
            // 对于多旋翼飞行器，init_targets_on_arming始终设置为true。
            if (motors->init_targets_on_arming()) {
                attitude_control->reset_yaw_target_and_rate();
                attitude_control->reset_rate_controller_I_terms_smoothly();
            }
            break;

        case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
            // 在油门大于零时清除着陆标志
            if (!motors->limit.throttle_lower) {
                set_land_complete(false);
            }
            break;

        case AP_Motors::SpoolState::SPOOLING_UP:
        case AP_Motors::SpoolState::SPOOLING_DOWN:
            // 不做任何操作
            break;
        }

        // 获取飞行员期望的油门
#if FRAME_CONFIG == HELI_FRAME
        // 对于直升机配置，获取飞行员期望的集体螺距
        pilot_throttle_scaled = copter.input_manager.get_pilot_desired_collective(channel_throttle->get_control_in());
#else
        // 对于其他配置，获取飞行员期望的油门
        pilot_throttle_scaled = get_pilot_desired_throttle();
#endif

    }

    // 检查系统识别参数是否有效
    if ((systemid_state == SystemIDModeState::SYSTEMID_STATE_TESTING) &&
        (!is_positive(frequency_start) || !is_positive(frequency_stop) || is_negative(time_fade_in) || !is_positive(time_record) || is_negative(time_fade_out) || (time_record <= time_const_freq))) {
        // 如果参数无效，停止系统识别
        systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
        gcs().send_text(MAV_SEVERITY_INFO, "SystemID Parameter Error");
    }

    // 更新波形时间和样本
    waveform_time += G_Dt;
    waveform_sample = chirp_input.update(waveform_time - SYSTEM_ID_DELAY, waveform_magnitude);
    waveform_freq_rads = chirp_input.get_frequency_rads();
    Vector2f disturb_state;
    switch (systemid_state) {
        case SystemIDModeState::SYSTEMID_STATE_STOPPED:
            // 系统识别停止状态，启用姿态前馈
            attitude_control->bf_feedforward(att_bf_feedforward);
            break;
        case SystemIDModeState::SYSTEMID_STATE_TESTING:

            // 检查是否已着陆
            if (copter.ap.land_complete) {
                systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
                gcs().send_text(MAV_SEVERITY_INFO, "SystemID Stopped: Landed");
                break;
            }
            // 检查倾斜角度是否超过最大限制
            if (attitude_control->lean_angle_deg()*100 > attitude_control->lean_angle_max_cd()) {
                systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
                gcs().send_text(MAV_SEVERITY_INFO, "SystemID Stopped: lean=%f max=%f", (double)attitude_control->lean_angle_deg(), (double)attitude_control->lean_angle_max_cd());
                break;
            }
            // 检查系统识别是否完成
            if (waveform_time > SYSTEM_ID_DELAY + time_fade_in + time_const_freq + time_record + time_fade_out) {
                systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
                gcs().send_text(MAV_SEVERITY_INFO, "SystemID Finished");
                break;
            }

            // 根据选择的轴类型执行相应的操作
            switch ((AxisType)axis.get()) {
                case AxisType::NONE:
                    systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
                    gcs().send_text(MAV_SEVERITY_INFO, "SystemID Stopped: axis = 0");
                    break;
                case AxisType::INPUT_ROLL:
                    // 在滚转角度上添加波形样本
                    target_roll += waveform_sample*100.0f;
                    break;
                case AxisType::INPUT_PITCH:
                    // 在俯仰角度上添加波形样本
                    target_pitch += waveform_sample*100.0f;
                    break;
                case AxisType::INPUT_YAW:
                    // 在偏航速率上添加波形样本
                    target_yaw_rate += waveform_sample*100.0f;
                    break;
                case AxisType::RECOVER_ROLL:
                    // 在滚转角度上添加波形样本并禁用前馈
                    target_roll += waveform_sample*100.0f;
                    attitude_control->bf_feedforward(false);
                    break;
                case AxisType::RECOVER_PITCH:
                    // 在俯仰角度上添加波形样本并禁用前馈
                    target_pitch += waveform_sample*100.0f;
                    attitude_control->bf_feedforward(false);
                    break;
                case AxisType::RECOVER_YAW:
                    // 在偏航速率上添加波形样本并禁用前馈
                    target_yaw_rate += waveform_sample*100.0f;
                    attitude_control->bf_feedforward(false);
                    break;
                case AxisType::RATE_ROLL:
                    // 设置滚转角速度
                    attitude_control->rate_bf_roll_sysid(radians(waveform_sample));
                    break;
                case AxisType::RATE_PITCH:
                    // 设置俯仰角速度
                    attitude_control->rate_bf_pitch_sysid(radians(waveform_sample));
                    break;
                case AxisType::RATE_YAW:
                    // 设置偏航角速度
                    attitude_control->rate_bf_yaw_sysid(radians(waveform_sample));
                    break;
                case AxisType::MIX_ROLL:
                    // 设置滚转执行器输入
                    attitude_control->actuator_roll_sysid(waveform_sample);
                    break;
                case AxisType::MIX_PITCH:
                    // 设置俯仰执行器输入
                    attitude_control->actuator_pitch_sysid(waveform_sample);
                    break;
                case AxisType::MIX_YAW:
                    // 设置偏航执行器输入
                    attitude_control->actuator_yaw_sysid(waveform_sample);
                    break;
                case AxisType::MIX_THROTTLE:
                    // 在油门上添加波形样本
                    pilot_throttle_scaled += waveform_sample;
                    break;
                case AxisType::DISTURB_POS_LAT:
                    // 设置横向位置扰动
                    disturb_state.x = 0.0f;
                    disturb_state.y = waveform_sample * 100.0f;
                    disturb_state.rotate(attitude_control->get_att_target_euler_rad().z);
                    pos_control->set_disturb_pos_cm(disturb_state);
                    break;
                case AxisType::DISTURB_POS_LONG:
                    // 设置纵向位置扰动
                    disturb_state.x = waveform_sample * 100.0f;
                    disturb_state.y = 0.0f;
                    disturb_state.rotate(attitude_control->get_att_target_euler_rad().z);
                    pos_control->set_disturb_pos_cm(disturb_state);
                    break;
                case AxisType::DISTURB_VEL_LAT:
                    // 设置横向速度扰动
                    disturb_state.x = 0.0f;
                    disturb_state.y = waveform_sample * 100.0f;
                    disturb_state.rotate(attitude_control->get_att_target_euler_rad().z);
                    pos_control->set_disturb_vel_cms(disturb_state);
                    break;
                case AxisType::DISTURB_VEL_LONG:
                    // 设置纵向速度扰动
                    disturb_state.x = waveform_sample * 100.0f;
                    disturb_state.y = 0.0f;
                    disturb_state.rotate(attitude_control->get_att_target_euler_rad().z);
                    pos_control->set_disturb_vel_cms(disturb_state);
                    break;
                case AxisType::INPUT_VEL_LAT:
                    // 设置横向速度输入
                    input_vel.x = 0.0f;
                    input_vel.y = waveform_sample * 100.0f;
                    input_vel.rotate(attitude_control->get_att_target_euler_rad().z);
                    break;
                case AxisType::INPUT_VEL_LONG:
                    // 设置纵向速度输入
                    input_vel.x = waveform_sample * 100.0f;
                    input_vel.y = 0.0f;
                    input_vel.rotate(attitude_control->get_att_target_euler_rad().z);
                    break;
            }
            break;
    }

    if (!is_poscontrol_axis_type()) {

        // 调用姿态控制器
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

        // 输出飞行员的油门
        attitude_control->set_throttle_out(pilot_throttle_scaled, !copter.is_tradheli(), g.throttle_filt);
        
    } else {

        // 如果可能已着陆，放松悬停目标
        if (copter.ap.land_complete_maybe) {
            pos_control->soften_for_landing_xy();
        }

        // 更新位置、速度和加速度目标
        Vector2f accel;
        target_pos += input_vel * G_Dt;
        if (is_positive(G_Dt)) {
            accel = (input_vel - input_vel_last) / G_Dt;
            input_vel_last = input_vel;
        }
        pos_control->set_pos_vel_accel_xy(target_pos.topostype(), input_vel, accel);

        // 运行位置控制器
        pos_control->update_xy_controller();

        // 调用姿态控制器
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), target_yaw_rate, false);

        // 将命令的爬升率发送到位置控制器
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);

        // 运行垂直位置控制器并设置输出油门
        pos_control->update_z_controller();
    }

    // 控制日志记录频率
    if (log_subsample <= 0) {
        log_data();
        if (copter.should_log(MASK_LOG_ATTITUDE_FAST) && copter.should_log(MASK_LOG_ATTITUDE_MED)) {
            log_subsample = 1;
        } else if (copter.should_log(MASK_LOG_ATTITUDE_FAST)) {
            log_subsample = 2;
        } else if (copter.should_log(MASK_LOG_ATTITUDE_MED)) {
            log_subsample = 4;
        } else {
            log_subsample = 8;
        }
    }
    log_subsample -= 1;
}

// 记录系统识别和姿态数据
void ModeSystemId::log_data() const
{
    // 获取角度增量和时间间隔
    Vector3f delta_angle;
    float delta_angle_dt;
    copter.ins.get_delta_angle(delta_angle, delta_angle_dt);

    // 获取速度增量和时间间隔
    Vector3f delta_velocity;
    float delta_velocity_dt;
    copter.ins.get_delta_velocity(delta_velocity, delta_velocity_dt);

    // 记录系统识别数据
    if (is_positive(delta_angle_dt) && is_positive(delta_velocity_dt)) {
        copter.Log_Write_SysID_Data(waveform_time, waveform_sample, waveform_freq_rads / (2 * M_PI), degrees(delta_angle.x / delta_angle_dt), degrees(delta_angle.y / delta_angle_dt), degrees(delta_angle.z / delta_angle_dt), delta_velocity.x / delta_velocity_dt, delta_velocity.y / delta_velocity_dt, delta_velocity.z / delta_velocity_dt);
    }

    // 记录姿态和PID数据
    copter.Log_Write_Attitude();
    copter.Log_Write_PIDS();

    // 如果是位置控制轴类型，记录位置控制数据
    if (is_poscontrol_axis_type()) {
        pos_control->write_log();
        copter.logger.Write_PID(LOG_PIDN_MSG, pos_control->get_vel_xy_pid().get_pid_info_x());
        copter.logger.Write_PID(LOG_PIDE_MSG, pos_control->get_vel_xy_pid().get_pid_info_y());

    }
}

// 检查是否为位置控制轴类型
bool ModeSystemId::is_poscontrol_axis_type() const
{
    bool ret = false;

    switch ((AxisType)axis.get()) {
        case AxisType::DISTURB_POS_LAT:
        case AxisType::DISTURB_POS_LONG:
        case AxisType::DISTURB_VEL_LAT:
        case AxisType::DISTURB_VEL_LONG:
        case AxisType::INPUT_VEL_LAT:
        case AxisType::INPUT_VEL_LONG:
            ret = true;
            break;
        default:
            break;
        }

    return ret;
}

#endif
