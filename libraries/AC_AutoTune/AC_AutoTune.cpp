#include "AC_AutoTune_config.h"

#if AC_AUTOTUNE_ENABLED

#include "AC_AutoTune.h"

#include <AP_Logger/AP_Logger.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Notify/AP_Notify.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

// 飞手覆盖超时时间(毫秒),如果飞手将摇杆保持在中间位置2秒则重新开始调参
#define AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS  500         
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
 // 判定为水平的角度(固定翼使用更宽松的5度)
 # define AUTOTUNE_LEVEL_ANGLE_CD           500         
 // 判定为水平的横滚和俯仰速率(固定翼使用更宽松的10度/秒)
 # define AUTOTUNE_LEVEL_RATE_RP_CD         1000        
#else
 // 判定为水平的角度
 # define AUTOTUNE_LEVEL_ANGLE_CD           250         
 // 判定为水平的横滚和俯仰速率
 # define AUTOTUNE_LEVEL_RATE_RP_CD         500         
#endif
// 判定为水平的偏航速率
#define AUTOTUNE_LEVEL_RATE_Y_CD            750         
// 开始下一次测试前需要保持水平的时间(毫秒)
#define AUTOTUNE_REQUIRED_LEVEL_TIME_MS     250         
// 水平超时时间(毫秒)
#define AUTOTUNE_LEVEL_TIMEOUT_MS           2000        
// 水平失败警告消息发送间隔(毫秒)
#define AUTOTUNE_LEVEL_WARNING_INTERVAL_MS  5000        

// 构造函数
AC_AutoTune::AC_AutoTune()
{
}

// 自动调参初始化 - 选择自动调参模式时应调用
bool AC_AutoTune::init_internals(bool _use_poshold,
                                 AC_AttitudeControl *_attitude_control,
                                 AC_PosControl *_pos_control,
                                 AP_AHRS_View *_ahrs_view,
                                 AP_InertialNav *_inertial_nav)
{
    use_poshold = _use_poshold;
    attitude_control = _attitude_control;
    pos_control = _pos_control;
    ahrs_view = _ahrs_view;
    inertial_nav = _inertial_nav;
    motors = AP_Motors::get_singleton();
    const uint32_t now = AP_HAL::millis();

    // 如果电机未解锁则立即退出
    if ((motors == nullptr) || !motors->armed()) {
        return false;
    }

    // 初始化位置控制器
    init_position_controller();

    switch (mode) {
    case FAILED:
        // 继续重新开始调参
        FALLTHROUGH;

    case UNINITIALISED:
        // 自动调参从未运行过
        // 将当前增益存储为原始增益
        backup_gains_and_initialise();
        // 将模式切换到调参
        mode = TUNING;
        // 向地面站发送开始调参消息
        update_gcs(AUTOTUNE_MESSAGE_STARTED);
        break;

    case TUNING:
        // 重置每个车辆的测试变量
        reset_vehicle_test_variables();

        // 我们正在重新开始调参,所以从上次停止的地方继续
        step = WAITING_FOR_LEVEL;
        step_start_time_ms = now;
        level_start_time_ms = now;
        // 重置增益为调参开始时的增益(即较低的I项)
        load_gains(GAIN_INTRA_TEST);
        LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_RESTART);
        update_gcs(AUTOTUNE_MESSAGE_STARTED);
        break;

    case SUCCESS:
        // 我们已完成调参,飞手希望测试新的增益
        load_gains(GAIN_TUNED);
        update_gcs(AUTOTUNE_MESSAGE_TESTING);
        LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_PILOT_TESTING);
        break;
    }

    have_position = false;

    return true;
}

// 停止 - 当ch7/ch8开关关闭时应调用
void AC_AutoTune::stop()
{
    // 将增益设置为原始值
    load_gains(GAIN_ORIGINAL);

    // 重新启用角度到速率请求限制
    attitude_control->use_sqrt_controller(true);

    update_gcs(AUTOTUNE_MESSAGE_STOPPED);

    LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_OFF);

    // 注意:我们保持模式不变,以便知道自动调参如何结束
    // 我们期望调用者将飞行模式切换回飞行模式开关指示的飞行模式
}

// 自动调参辅助功能触发器
void AC_AutoTune::do_aux_function(const RC_Channel::AuxSwitchPos ch_flag)
{
    if (mode != TuneMode::SUCCESS) {
        if (ch_flag == RC_Channel::AuxSwitchPos::HIGH) {
            gcs().send_text(MAV_SEVERITY_NOTICE,"AutoTune: 必须完成才能测试增益");
        }
        return;
    }

    switch(ch_flag) {
        case RC_Channel::AuxSwitchPos::LOW:
            // 加载原始增益
            load_gains(GainType::GAIN_ORIGINAL);
            update_gcs(AUTOTUNE_MESSAGE_TESTING_END);
            break;
        case RC_Channel::AuxSwitchPos::MIDDLE:
            // 中间位置暂时未使用
            break;
        case RC_Channel::AuxSwitchPos::HIGH:
            // 加载调参增益
            load_gains(GainType::GAIN_TUNED);
            update_gcs(AUTOTUNE_MESSAGE_TESTING);
            break;
    }

    have_pilot_testing_command = true;
}

// 可能保存增益,解除武装时调用
void AC_AutoTune::disarmed(const bool in_autotune_mode)
{
    // 如果飞手正在测试调参增益则为true
    const bool testing_tuned = have_pilot_testing_command && (loaded_gains == GainType::GAIN_TUNED);

    // 如果在自动调参模式且未收到飞手测试命令则为true
    const bool tune_complete_no_testing = !have_pilot_testing_command && in_autotune_mode;

    if (tune_complete_no_testing || testing_tuned) {
        save_tuning_gains();
    } else {
        reset();
    }
}

// 初始化位置控制器
bool AC_AutoTune::init_position_controller(void)
{
    // 初始化垂直最大速度和加速度
    init_z_limits();

    // 初始化垂直位置控制器
    pos_control->init_z_controller();

    return true;
}

// 发送步骤字符串
void AC_AutoTune::send_step_string()
{
    if (pilot_override) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: 已暂停: 飞手覆盖激活");
        return;
    }
    switch (step) {
    case WAITING_FOR_LEVEL:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: 正在调平");
        return;
    case UPDATE_GAINS:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: 正在更新增益");
        return;
    case ABORT:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: 正在中止测试");
        return;
    case TESTING:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: 正在测试");
        return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: 未知步骤");
}

// 返回类型字符串
const char *AC_AutoTune::type_string() const
{
    switch (tune_type) {
    case RD_UP:
        return "速率D增加";
    case RD_DOWN:
        return "速率D减小";
    case RP_UP:
        return "速率P增加";
    case RFF_UP:
        return "速率FF增加";
    case SP_UP:
        return "角度P增加";
    case SP_DOWN:
        return "角度P减小";
    case MAX_GAINS:
        return "寻找最大增益";
    case TUNE_CHECK:
        return "检查调参频率响应";
    case TUNE_COMPLETE:
        return "调参完成";
    }
    return "";
    // 这种情况不应该发生
    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
}

// 返回当前轴向字符串
const char *AC_AutoTune::axis_string() const
{
    switch (axis) {
    case AxisType::ROLL:
        return "横滚";
    case AxisType::PITCH:
        return "俯仰";
    case AxisType::YAW:
        return "偏航(E)";
    case AxisType::YAW_D:
        return "偏航(D)";
    }
    return "";
}

// 运行 - 运行自动调参飞行模式
// 应以100Hz或更高频率调用
void AC_AutoTune::run()
{
    // 初始化垂直速度和加速度
    init_z_limits();

    // 如果未自动解锁或电机联锁未启用则将油门设为零并立即退出
    // 由于init()检查,这实际上不应该发生
    if (!motors->armed() || !motors->get_interlock()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0.0f, true, 0.0f);
        pos_control->relax_z_controller(0.0f);
        return;
    }

    float target_roll_cd, target_pitch_cd, target_yaw_rate_cds;
    get_pilot_desired_rp_yrate_cd(target_roll_cd, target_pitch_cd, target_yaw_rate_cds);

    // 获取飞手期望的爬升速率
    const float target_climb_rate_cms = get_pilot_desired_climb_rate_cms();

    const bool zero_rp_input = is_zero(target_roll_cd) && is_zero(target_pitch_cd);

    const uint32_t now = AP_HAL::millis();

    if (mode != SUCCESS) {
        if (!zero_rp_input || !is_zero(target_yaw_rate_cds) || !is_zero(target_climb_rate_cms)) {
            if (!pilot_override) {
                pilot_override = true;
                // 将增益设置为原始值
                load_gains(GAIN_ORIGINAL);
                attitude_control->use_sqrt_controller(true);
            }
            // 重置飞手覆盖时间
            override_time = now;
            if (!zero_rp_input) {
                // 仅在横滚或俯仰输入时重置位置
                have_position = false;
            }
        } else if (pilot_override) {
            // 检查飞手覆盖后是否应恢复调参
            if (now - override_time > AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS) {
                pilot_override = false;             // 关闭飞手覆盖
                // 将增益设置为测试间值(与原始增益非常接近)
                // load_gains(GAIN_INTRA_TEST); //我认为我们应该在这里保持原始值以让I项快速稳定
                step = WAITING_FOR_LEVEL; // 将调参步骤设回开始
                step_start_time_ms = now;
                level_start_time_ms = now;
                desired_yaw_cd = ahrs_view->yaw_sensor;
            }
        }
    }
    if (pilot_override) {
        if (now - last_pilot_override_warning > 1000) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: 飞手覆盖激活");
            last_pilot_override_warning = now;
        }
    }
    if (zero_rp_input) {
        // 飞手在油门和偏航上的输入如果启用了位置保持仍将使用位置保持
        get_poshold_attitude(target_roll_cd, target_pitch_cd, desired_yaw_cd);
    }

    // 将电机设置为全范围
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // 如果飞手覆盖则调用姿态控制器
    if (pilot_override || mode != TUNING) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll_cd, target_pitch_cd, target_yaw_rate_cds);
    } else {
        // 从自动调参获取姿态请求
        control_attitude();
        // 告诉用户正在发生什么
        do_gcs_announcements();
    }

    // 调用位置控制器
    pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate_cms);
    pos_control->update_z_controller();

}

// 如果飞机接近水平则返回true
bool AC_AutoTune::currently_level()
{
    // 如果超过2 * AUTOTUNE_LEVEL_TIMEOUT_MS则中止自动调参
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - level_start_time_ms > 3 * AUTOTUNE_LEVEL_TIMEOUT_MS) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "AutoTune: 调平失败,请手动调参");
        mode = FAILED;
        LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_FAILED);
    }

    // 斜率阈值以确保无法获得小阈值的飞机有足够的稳定时间
    // 如果超过AUTOTUNE_LEVEL_TIMEOUT_MS则放宽阈值
    const float threshold_mul = constrain_float((float)(now_ms - level_start_time_ms) / (float)AUTOTUNE_LEVEL_TIMEOUT_MS, 0.0, 2.0);

    if (fabsf(ahrs_view->roll_sensor - roll_cd) > threshold_mul * AUTOTUNE_LEVEL_ANGLE_CD) {
        return false;
    }

    if (fabsf(ahrs_view->pitch_sensor - pitch_cd) > threshold_mul * AUTOTUNE_LEVEL_ANGLE_CD) {
        return false;
    }
    if (fabsf(wrap_180_cd(ahrs_view->yaw_sensor - desired_yaw_cd)) > threshold_mul * AUTOTUNE_LEVEL_ANGLE_CD) {
        return false;
    }
    if ((ToDeg(ahrs_view->get_gyro().x) * 100.0f) > threshold_mul * AUTOTUNE_LEVEL_RATE_RP_CD) {
        return false;
    }
    if ((ToDeg(ahrs_view->get_gyro().y) * 100.0f) > threshold_mul * AUTOTUNE_LEVEL_RATE_RP_CD) {
        return false;
    }
    if ((ToDeg(ahrs_view->get_gyro().z) * 100.0f) > threshold_mul * AUTOTUNE_LEVEL_RATE_Y_CD) {
        return false;
    }
    return true;
}

// 主状态机用于调平飞机、执行测试和更新增益
// 直接用目标更新姿态控制器
void AC_AutoTune::control_attitude()
{
    rotation_rate = 0.0f;        // 旋转速率(弧度/秒)
    lean_angle = 0.0f;
    const float direction_sign = positive_direction ? 1.0f : -1.0f;
    const uint32_t now = AP_HAL::millis();

    // 检查调参步骤
    switch (step) {

    case WAITING_FOR_LEVEL: {

        // 注意:我们应该使用测试间增益(与原始增益非常接近但I项较低)
        // 重新启用速率限制
        attitude_control->use_sqrt_controller(true);

        get_poshold_attitude(roll_cd, pitch_cd, desired_yaw_cd);

        // 保持水平姿态
        attitude_control->input_euler_angle_roll_pitch_yaw(roll_cd, pitch_cd, desired_yaw_cd, true);

        // 在开始抖动前保持飞机水平0.5秒
        // 如果不再水平则重置计数器
        if (!currently_level()) {
            step_start_time_ms = now;
        }

        // 如果我们已经保持水平足够长时间(0.5秒)则进入调参步骤
        if (now - step_start_time_ms > AUTOTUNE_REQUIRED_LEVEL_TIME_MS) {
            // 初始化下一步的变量
            step = TESTING;
            step_start_time_ms = now;
            step_time_limit_ms = get_testing_step_timeout_ms();
            // 将增益设置为待测试值
            load_gains(GAIN_TEST);
        } else {
            // 等待水平时使用测试间增益
            load_gains(GAIN_INTRA_TEST);
        }

        // 初始化特定于测试的变量
        switch (axis) {
        case AxisType::ROLL:
            start_rate = ToDeg(ahrs_view->get_gyro().x) * 100.0f;
            start_angle = ahrs_view->roll_sensor;
            break;
        case AxisType::PITCH:
            start_rate = ToDeg(ahrs_view->get_gyro().y) * 100.0f;
            start_angle = ahrs_view->pitch_sensor;
            break;
        case AxisType::YAW:
        case AxisType::YAW_D:
            start_rate = ToDeg(ahrs_view->get_gyro().z) * 100.0f;
            start_angle = ahrs_view->yaw_sensor;
            break;
        }

        // 测试必须最后初始化,因为有些依赖于上面的变量
        test_init();

        break;
    }

    case TESTING: {
        // 运行抖动步骤
        load_gains(GAIN_TEST);

        // 运行测试
        test_run(axis, direction_sign);

        // 检查导致反向响应的故障
        if (lean_angle <= -angle_lim_neg_rpy_cd()) {
            step = WAITING_FOR_LEVEL;
            positive_direction = twitch_reverse_direction();
            step_start_time_ms = now;
            level_start_time_ms = now;
        }

        // 防止翻转
        if (attitude_control->lean_angle_deg() * 100 > angle_lim_max_rp_cd()) {
            step = WAITING_FOR_LEVEL;
            positive_direction = twitch_reverse_direction();
            step_start_time_ms = now;
            level_start_time_ms = now;
        }

#if HAL_LOGGING_ENABLED
        // 记录本次迭代的倾斜角度和旋转速率
        Log_AutoTuneDetails();
        attitude_control->Write_Rate(*pos_control);
        log_pids();
#endif

        if (axis == AxisType::YAW || axis == AxisType::YAW_D) {
            desired_yaw_cd = ahrs_view->yaw_sensor;
        }
        break;
    }

    case UPDATE_GAINS:

        // 重新启用速率限制
        attitude_control->use_sqrt_controller(true);

#if HAL_LOGGING_ENABLED
        // 记录最新增益
        Log_AutoTune();
#endif

        // 宣布调参类型测试结果
        // 必须在更新方法之前完成,因为此方法会更改下一次测试的参数
        do_post_test_gcs_announcements();

        switch (tune_type) {
        // 检查小步骤后增加速率D增益的结果
        case RD_UP:
            updating_rate_d_up_all(axis);
            break;
        // 检查小步骤后减小速率D增益的结果
        case RD_DOWN:
            updating_rate_d_down_all(axis);
            break;
        // 检查小步骤后增加速率P增益的结果
        case RP_UP:
            updating_rate_p_up_all(axis);
            break;
        // 检查小步骤后增加稳定P增益的结果
        case SP_DOWN:
            updating_angle_p_down_all(axis);
            break;
        // 检查小步骤后增加稳定P增益的结果
        case SP_UP:
            updating_angle_p_up_all(axis);
            break;
        case RFF_UP:
            updating_rate_ff_up_all(axis);
            break;
        case MAX_GAINS:
            updating_max_gains_all(axis);
            break;
        case TUNE_CHECK:
            counter = AUTOTUNE_SUCCESS_COUNT;
            FALLTHROUGH;
        case TUNE_COMPLETE:
            break;
        }

        // 我们已完成此步骤,最终确定pid并移至下一步
        if (counter >= AUTOTUNE_SUCCESS_COUNT) {

            // 重置计数器
            counter = 0;

            // 重置缩放因子
            step_scaler = 1.0f;


            // 在移至下一个调参类型之前设置调参后的增益
            set_gains_post_tune(axis);

            // 将调参类型递增到调参序列中的下一个
            next_tune_type(tune_type, false);

            if (tune_type == TUNE_COMPLETE) {
                // 我们已到达D-up-down PI-up-down调参类型循环的末尾
                next_tune_type(tune_type, true);

                report_final_gains(axis);

                // 进入下一个轴向
                bool complete = false;
                switch (axis) {
                case AxisType::ROLL:
                    axes_completed |= AUTOTUNE_AXIS_BITMASK_ROLL;
                    if (pitch_enabled()) {
                        axis = AxisType::PITCH;
                    } else if (yaw_enabled()) {
                        axis = AxisType::YAW;
                    } else if (yaw_d_enabled()) {
                        axis = AxisType::YAW_D;
                    } else {
                        complete = true;
                    }
                    break;
                case AxisType::PITCH:
                    axes_completed |= AUTOTUNE_AXIS_BITMASK_PITCH;
                    if (yaw_enabled()) {
                        axis = AxisType::YAW;
                    } else if (yaw_d_enabled()) {
                        axis = AxisType::YAW_D;
                    } else {
                        complete = true;
                    }
                    break;
                case AxisType::YAW:
                    axes_completed |= AUTOTUNE_AXIS_BITMASK_YAW;
                    if (yaw_d_enabled()) {
                        axis = AxisType::YAW_D;
                    } else {
                        complete = true;
                    }
                    break;
                case AxisType::YAW_D:
                    axes_completed |= AUTOTUNE_AXIS_BITMASK_YAW_D;
                    complete = true;
                    break;
                }

                // 如果我们刚完成所有轴向,我们已成功完成自动调参
                // 切换到TESTING模式以允许用户使用新增益飞行
                if (complete) {
                    mode = SUCCESS;
                    update_gcs(AUTOTUNE_MESSAGE_SUCCESS);
                    LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_SUCCESS);
                    AP_Notify::events.autotune_complete = true;

                    // 返回原始增益以进行着陆
                    load_gains(GainType::GAIN_ORIGINAL);
                } else {
                    AP_Notify::events.autotune_next_axis = true;
                    reset_update_gain_variables();
                }
            }
        }
        FALLTHROUGH;

    case ABORT:
        if (axis == AxisType::YAW || axis == AxisType::YAW_D) {
            // todo: 检查我们是否需要这个
            attitude_control->input_euler_angle_roll_pitch_yaw(0.0f, 0.0f, ahrs_view->yaw_sensor, false);
        }

        // 将增益设置为测试间值(与原始增益非常接近)
        load_gains(GAIN_INTRA_TEST);

        // 重置测试步骤
        step = WAITING_FOR_LEVEL;
        positive_direction = twitch_reverse_direction();
        step_start_time_ms = now;
        level_start_time_ms = now;
        step_time_limit_ms = AUTOTUNE_REQUIRED_LEVEL_TIME_MS;
        break;
    }
}

// 备份增益并初始化 - 在调参开始前存储当前增益作为原始增益
void AC_AutoTune::backup_gains_and_initialise()
{
    const uint32_t now = AP_HAL::millis();
    
    // 初始化状态因为这是我们的第一次
    if (roll_enabled()) {
        axis = AxisType::ROLL;
    } else if (pitch_enabled()) {
        axis = AxisType::PITCH;
    } else if (yaw_enabled()) {
        axis = AxisType::YAW;
    } else if (yaw_d_enabled()) {
        axis = AxisType::YAW_D;
    }
    // 没有完成的轴向
    axes_completed = 0;

    // 重置每个车辆的更新增益变量
    reset_update_gain_variables();

    // 从调参序列开始
    next_tune_type(tune_type, true);

    step = WAITING_FOR_LEVEL;
    positive_direction = false;
    step_start_time_ms = now;
    level_start_time_ms = now;
    step_scaler = 1.0f;

    desired_yaw_cd = ahrs_view->yaw_sensor;
}

/*
  加载指定的增益集
 */
void AC_AutoTune::load_gains(enum GainType gain_type)
{
    if (loaded_gains == gain_type) {
        // 已加载的增益已经是正确类型
        return;
    }
    loaded_gains = gain_type;

    switch (gain_type) {
    case GAIN_ORIGINAL:
        load_orig_gains();
        break;
    case GAIN_INTRA_TEST:
        load_intra_test_gains();
        break;
    case GAIN_TEST:
        load_test_gains();
        break;
    case GAIN_TUNED:
        load_tuned_gains();
        break;
    }
}

// 更新gcs - 向地面站发送消息
void AC_AutoTune::update_gcs(uint8_t message_id) const
{
    switch (message_id) {
    case AUTOTUNE_MESSAGE_STARTED:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"AutoTune: 已启动");
        break;
    case AUTOTUNE_MESSAGE_STOPPED:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"AutoTune: 已停止");
        break;
    case AUTOTUNE_MESSAGE_SUCCESS:
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE,"AutoTune: 成功");
        break;
    case AUTOTUNE_MESSAGE_FAILED:
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE,"AutoTune: 失败");
        break;
    case AUTOTUNE_MESSAGE_TESTING:
    case AUTOTUNE_MESSAGE_SAVED_GAINS:
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE,"AutoTune: %s gains for %s%s%s%s",
                        (message_id == AUTOTUNE_MESSAGE_SAVED_GAINS) ? "Saved" : "Pilot Testing",
                        (axes_completed&AUTOTUNE_AXIS_BITMASK_ROLL)?"Roll ":"",
                        (axes_completed&AUTOTUNE_AXIS_BITMASK_PITCH)?"Pitch ":"",
                        (axes_completed&AUTOTUNE_AXIS_BITMASK_YAW)?"Yaw(E)":"",
                        (axes_completed&AUTOTUNE_AXIS_BITMASK_YAW_D)?"Yaw(D)":"");
        break;
    case AUTOTUNE_MESSAGE_TESTING_END:
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE,"AutoTune: original gains restored");
        break;
    }
}

// 轴向辅助函数 - 检查是否启用了各个轴向的自动调参

// 检查是否启用了横滚轴自动调参
bool AC_AutoTune::roll_enabled() const
{
    return get_axis_bitmask() & AUTOTUNE_AXIS_BITMASK_ROLL;
}

// 检查是否启用了俯仰轴自动调参
bool AC_AutoTune::pitch_enabled() const
{
    return get_axis_bitmask() & AUTOTUNE_AXIS_BITMASK_PITCH;
}

// 检查是否启用了偏航轴自动调参
bool AC_AutoTune::yaw_enabled() const
{
    return get_axis_bitmask() & AUTOTUNE_AXIS_BITMASK_YAW;
}

// 检查是否启用了偏航D参数自动调参
// 对于直升机,始终返回false
bool AC_AutoTune::yaw_d_enabled() const
{
#if APM_BUILD_TYPE(APM_BUILD_Heli)
    return false;
#else
    return get_axis_bitmask() & AUTOTUNE_AXIS_BITMASK_YAW_D;
#endif
}

/*
  检查是否有良好的位置估计
  返回true表示有可靠的位置估计,false表示位置估计不可靠
 */
bool AC_AutoTune::position_ok(void)
{
    // 如果没有惯性导航系统,不允许导航
    if (!AP::ahrs().have_inertial_nav()) {
        return false;
    }

    // 使用EKF滤波器状态进行检查
    nav_filter_status filt_status = inertial_nav->get_filter_status();

    // 要求有良好的绝对位置,且EKF不能处于const_pos_mode
    return (filt_status.flags.horiz_pos_abs && !filt_status.flags.const_pos_mode);
}

// 获取自动调参模式下的位置保持姿态
// roll_cd_out: 输出的横滚角(厘度)
// pitch_cd_out: 输出的俯仰角(厘度)
// yaw_cd_out: 输出的偏航角(厘度)
void AC_AutoTune::get_poshold_attitude(float &roll_cd_out, float &pitch_cd_out, float &yaw_cd_out)
{
    // 初始化输出角度为0
    roll_cd_out = pitch_cd_out = 0;

    // 如果未启用位置保持,直接返回
    if (!use_poshold) {
        return;
    }

    // 如果位置估计不可靠,不进行位置保持
    if (!position_ok()) {
        return;
    }

    // 如果还没有记录起始位置,记录当前位置
    if (!have_position) {
        have_position = true;
        start_position = inertial_nav->get_position_neu_cm();
    }

    // 限制最大倾斜角为10度,以免影响自动调参效果
    const float angle_max_cd = 1000;

    // 在20米位置误差时达到10度倾斜限制
    const float dist_limit_cm = 2000;

    // 只有当位置误差超过5米(对应2.5度倾斜角)时才开始调整偏航
    const float yaw_dist_limit_cm = 500;

    // 计算当前位置与起始位置的偏差向量(不考虑高度)
    Vector3f pdiff = inertial_nav->get_position_neu_cm() - start_position;
    pdiff.z = 0;
    float dist_cm = pdiff.length();
    
    // 如果位置偏差小于10厘米,不做任何调整
    if (dist_cm < 10) {
        return;
    }

    /*
      简单的线性控制器:
      根据位置偏差计算所需倾斜角度
     */
    float scaling = constrain_float(angle_max_cd * dist_cm / dist_limit_cm, 0, angle_max_cd);
    Vector2f angle_ne(pdiff.x, pdiff.y);
    angle_ne *= scaling / dist_cm;

    // 将角度从地理坐标系转换到机体坐标系
    pitch_cd_out = angle_ne.x * ahrs_view->cos_yaw() + angle_ne.y * ahrs_view->sin_yaw();
    roll_cd_out  = angle_ne.x * ahrs_view->sin_yaw() - angle_ne.y * ahrs_view->cos_yaw();

    // 如果位置偏差小于yaw_dist_limit_cm,不调整偏航
    if (dist_cm < yaw_dist_limit_cm) {
        return;
    }

    /*
      如果飞机偏离期望位置超过yaw_dist_limit_cm,
      调整机头使抖动方向垂直于风向。
      这确保自动调参在调整某一轴时,其他轴的姿态偏差不超过2.5度
     */
    float target_yaw_cd = degrees(atan2f(pdiff.y, pdiff.x)) * 100;
    if (axis == AxisType::PITCH) {
        // 调参俯仰轴时机头垂直于风向,调参横滚和偏航轴时机头对准风向
        target_yaw_cd += 9000;
    }
    // 选择最近的180度标记,留5度余量防止振荡
    if (fabsf(yaw_cd_out - target_yaw_cd) > 9500) {
        target_yaw_cd += 18000;
    }

    yaw_cd_out = target_yaw_cd;
}

// 获取下一个调参类型
// curr_tune_type: 当前调参类型
// reset: 是否重置调参序列
void AC_AutoTune::next_tune_type(TuneType &curr_tune_type, bool reset)
{
    if (reset) {
        // 重置时设置调参序列并从头开始
        set_tune_sequence();
        tune_seq_curr = 0;
    } else if (curr_tune_type == TUNE_COMPLETE) {
        // 如果当前调参完成,保持TUNE_COMPLETE状态以开始下一轴或退出自动调参
        return;
    } else {
        // 进入序列中的下一个调参类型
        tune_seq_curr++;
    }

    curr_tune_type = tune_seq[tune_seq_curr];
}

#endif  // AC_AUTOTUNE_ENABLED
