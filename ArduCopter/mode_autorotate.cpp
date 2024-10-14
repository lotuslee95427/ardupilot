/* -----------------------------------------------------------------------------------------
    This is currently a SITL only function until the project is complete.
    To trial this in SITL you will need to use Real Flight 8.
    Instructions for how to set this up in SITL can be found here:
    https://discuss.ardupilot.org/t/autonomous-autorotation-gsoc-project-blog/42139
 -----------------------------------------------------------------------------------------*/

// 这是一个仅用于SITL(软件在环仿真)的功能,直到项目完成
// 要在SITL中试用此功能,你需要使用Real Flight 8
// 关于如何在SITL中设置此功能的说明可以在以下链接找到:
// https://discuss.ardupilot.org/t/autonomous-autorotation-gsoc-project-blog/42139

#include "Copter.h"
#include <AC_Autorotation/AC_Autorotation.h>
#include "mode.h"

#include <utility>

#if MODE_AUTOROTATE_ENABLED

#define AUTOROTATE_ENTRY_TIME          2.0f    // (s) number of seconds that the entry phase operates for
                                               // (秒) 进入阶段持续的秒数
#define BAILOUT_MOTOR_RAMP_TIME        1.0f    // (s) time set on bailout ramp up timer for motors - See AC_MotorsHeli_Single
                                               // (秒) 设置在电机紧急中止爬升定时器上的时间 - 参见 AC_MotorsHeli_Single
#define HEAD_SPEED_TARGET_RATIO        1.0f    // Normalised target main rotor head speed (unit: -)
                                               // 归一化的主旋翼头部速度目标 (单位: -)

bool ModeAutorotate::init(bool ignore_checks)
{
#if FRAME_CONFIG != HELI_FRAME
    // Only allow trad heli to use autorotation mode
    // 只允许传统直升机使用自转模式
    return false;
#endif

    // Check that mode is enabled
    // 检查模式是否启用
    if (!g2.arot.is_enable()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Autorot Mode Not Enabled");
        return false;
    }

    // Check that interlock is disengaged
    // 检查联锁是否已解除
    if (motors->get_interlock()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Autorot Mode Change Fail: Interlock Engaged");
        return false;
    }

    // Initialise controllers
    // This must be done before RPM value is fetched
    // 初始化控制器
    // 这必须在获取RPM值之前完成
    g2.arot.init_hs_controller();
    g2.arot.init_fwd_spd_controller();

    // Retrieve rpm and start rpm sensor health checks
    // 获取rpm并开始rpm传感器健康检查
    _initial_rpm = g2.arot.get_rpm(true);

    // Display message 
    // 显示消息
    gcs().send_text(MAV_SEVERITY_INFO, "Autorotation initiated");

     // Set all intial flags to on
     // 将所有初始标志设置为开启
    _flags.entry_initial = true;
    _flags.ss_glide_initial = true;
    _flags.flare_initial = true;
    _flags.touch_down_initial = true;
    _flags.level_initial = true;
    _flags.break_initial = true;
    _flags.straight_ahead_initial = true;
    _flags.bail_out_initial = true;
    _msg_flags.bad_rpm = true;

    // Setting default starting switches
    // 设置默认起始开关
    phase_switch = Autorotation_Phase::ENTRY;

    // Set entry timer
    // 设置进入计时器
    _entry_time_start_ms = millis();

    // The decay rate to reduce the head speed from the current to the target
    // 从当前头部速度降低到目标速度的衰减率
    _hs_decay = ((_initial_rpm/g2.arot.get_hs_set_point()) - HEAD_SPEED_TARGET_RATIO) / AUTOROTATE_ENTRY_TIME;

    return true;
}



void ModeAutorotate::run()
{
    // Check if interlock becomes engaged
    // 检查联锁是否已接合
    if (motors->get_interlock() && !copter.ap.land_complete) {
        phase_switch = Autorotation_Phase::BAIL_OUT;
    } else if (motors->get_interlock() && copter.ap.land_complete) {
        // Aircraft is landed and no need to bail out
        // 飞机已着陆,无需紧急中止
        set_mode(copter.prev_control_mode, ModeReason::AUTOROTATION_BAILOUT);
    }

    // Current time
    // 当前时间
    uint32_t now = millis(); //milliseconds

    // Initialise internal variables
    // 初始化内部变量
    float curr_vel_z = inertial_nav.get_velocity_z_up_cms();   // Current vertical descent
                                                               // 当前垂直下降速度

    //----------------------------------------------------------------
    //                  State machine logic
    //                  状态机逻辑
    //----------------------------------------------------------------

     // Setting default phase switch positions
     // 设置默认阶段开关位置
     nav_pos_switch = Navigation_Decision::USER_CONTROL_STABILISED;

    // Timer from entry phase to progress to glide phase
    // 从进入阶段到滑翔阶段的计时器
    if (phase_switch == Autorotation_Phase::ENTRY){

        if ((now - _entry_time_start_ms)/1000.0f > AUTOROTATE_ENTRY_TIME) {
            // Flight phase can be progressed to steady state glide
            // 飞行阶段可以进展到稳态滑翔
            phase_switch = Autorotation_Phase::SS_GLIDE;
        }

    }


    //----------------------------------------------------------------
    //                  State machine actions
    //                  状态机动作
    //----------------------------------------------------------------
    switch (phase_switch) {

        case Autorotation_Phase::ENTRY:
        {
            // Entry phase functions to be run only once
            // 仅运行一次的进入阶段函数
            if (_flags.entry_initial == true) {

                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    gcs().send_text(MAV_SEVERITY_INFO, "Entry Phase");
                #endif

                // Set following trim low pass cut off frequency
                // 设置跟随微调低通截止频率
                g2.arot.set_col_cutoff_freq(g2.arot.get_col_entry_freq());

                // Target head speed is set to rpm at initiation to prevent unwanted changes in attitude
                // 目标头部速度设置为初始化时的rpm,以防止姿态发生不必要的变化
                _target_head_speed = _initial_rpm/g2.arot.get_hs_set_point();

                // Set desired forward speed target
                // 设置期望的前向速度目标
                g2.arot.set_desired_fwd_speed();

                // Prevent running the initial entry functions again
                // 防止再次运行初始进入函数
                _flags.entry_initial = false;

            }

            // Slowly change the target head speed until the target head speed matches the parameter defined value
            // 缓慢改变目标头部速度,直到目标头部速度与参数定义的值匹配
            if (g2.arot.get_rpm() > HEAD_SPEED_TARGET_RATIO*1.005f  ||  g2.arot.get_rpm() < HEAD_SPEED_TARGET_RATIO*0.995f) {
                _target_head_speed -= _hs_decay*G_Dt;
            } else {
                _target_head_speed = HEAD_SPEED_TARGET_RATIO;
            }

            // Set target head speed in head speed controller
            // 在头部速度控制器中设置目标头部速度
            g2.arot.set_target_head_speed(_target_head_speed);

            // Run airspeed/attitude controller
            // 运行空速/姿态控制器
            g2.arot.set_dt(G_Dt);
            g2.arot.update_forward_speed_controller();

            // Retrieve pitch target
            // 获取俯仰目标
            _pitch_target = g2.arot.get_pitch();

            // Update controllers
            // 更新控制器
            _flags.bad_rpm = g2.arot.update_hs_glide_controller(G_Dt); //run head speed/ collective controller
                                                                       //运行头部速度/总距控制器

            break;
        }

        case Autorotation_Phase::SS_GLIDE:
        {
            // Steady state glide functions to be run only once
            // 仅运行一次的稳态滑翔函数
            if (_flags.ss_glide_initial == true) {

                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    gcs().send_text(MAV_SEVERITY_INFO, "SS Glide Phase");
                #endif

                // Set following trim low pass cut off frequency
                // 设置跟随微调低通截止频率
                g2.arot.set_col_cutoff_freq(g2.arot.get_col_glide_freq());

                // Set desired forward speed target
                // 设置期望的前向速度目标
                g2.arot.set_desired_fwd_speed();

                // Set target head speed in head speed controller
                // 在头部速度控制器中设置目标头部速度
                _target_head_speed = HEAD_SPEED_TARGET_RATIO;  //Ensure target hs is set to glide in case hs has not reached target for glide
                                                               //确保目标头部速度设置为滑翔,以防头部速度尚未达到滑翔目标
                g2.arot.set_target_head_speed(_target_head_speed);

                // Prevent running the initial glide functions again
                // 防止再次运行初始滑翔函数
                _flags.ss_glide_initial = false;
            }

            // Run airspeed/attitude controller
            // 运行空速/姿态控制器
            g2.arot.set_dt(G_Dt);
            g2.arot.update_forward_speed_controller();

            // Retrieve pitch target 
            // 获取俯仰目标
            _pitch_target = g2.arot.get_pitch();

            // Update head speed/ collective controller
            // 更新头部速度/总距控制器
            _flags.bad_rpm = g2.arot.update_hs_glide_controller(G_Dt); 
            // Attitude controller is updated in navigation switch-case statements
            // 姿态控制器在导航switch-case语句中更新

            break;
        }

        case Autorotation_Phase::FLARE:
        case Autorotation_Phase::TOUCH_DOWN:
        {
            break;
        }

        case Autorotation_Phase::BAIL_OUT:
        {
        if (_flags.bail_out_initial == true) {
                // Functions and settings to be done once are done here.
                // 这里执行只需要执行一次的函数和设置。

                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    gcs().send_text(MAV_SEVERITY_INFO, "Bailing Out of Autorotation");
                #endif

                // Set bail out timer remaining equal to the parameter value, bailout time 
                // cannot be less than the motor spool-up time: BAILOUT_MOTOR_RAMP_TIME.
                // 设置剩余的紧急中止计时器等于参数值,紧急中止时间
                // 不能小于电机加速时间: BAILOUT_MOTOR_RAMP_TIME。
                _bail_time = MAX(g2.arot.get_bail_time(),BAILOUT_MOTOR_RAMP_TIME+0.1f);

                // Set bail out start time
                // 设置紧急中止开始时间
                _bail_time_start_ms = now;

                // Set initial target vertical speed
                // 设置初始目标垂直速度
                _desired_v_z = curr_vel_z;

                // Initialise position and desired velocity
                // 初始化位置和期望速度
                if (!pos_control->is_active_z()) {
                    pos_control->relax_z_controller(g2.arot.get_last_collective());
                }

                // Get pilot parameter limits
                // 获取飞行员参数限制
                const float pilot_spd_dn = -get_pilot_speed_dn();
                const float pilot_spd_up = g.pilot_speed_up;

                float pilot_des_v_z = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
                pilot_des_v_z = constrain_float(pilot_des_v_z, pilot_spd_dn, pilot_spd_up);

                // Calculate target climb rate adjustment to transition from bail out descent speed to requested climb rate on stick.
                // 计算目标爬升率调整,以从紧急中止下降速度过渡到摇杆请求的爬升率。
                _target_climb_rate_adjust = (curr_vel_z - pilot_des_v_z)/(_bail_time - BAILOUT_MOTOR_RAMP_TIME); //accounting for 0.5s motor spool time
                                                                                                                 //考虑0.5秒电机加速时间

                // Calculate pitch target adjustment rate to return to level
                // 计算俯仰目标调整率以返回水平
                _target_pitch_adjust = _pitch_target/_bail_time;

                // set vertical speed and acceleration limits
                // 设置垂直速度和加速度限制
                pos_control->set_max_speed_accel_z(curr_vel_z, pilot_spd_up, fabsf(_target_climb_rate_adjust));
                pos_control->set_correction_speed_accel_z(curr_vel_z, pilot_spd_up, fabsf(_target_climb_rate_adjust));

                motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

                _flags.bail_out_initial = false;
            }

        if ((now - _bail_time_start_ms)/1000.0f >= BAILOUT_MOTOR_RAMP_TIME) {
            // Update desired vertical speed and pitch target after the bailout motor ramp timer has completed
            // 在紧急中止电机加速计时器完成后更新期望的垂直速度和俯仰目标
            _desired_v_z -= _target_climb_rate_adjust*G_Dt;
            _pitch_target -= _target_pitch_adjust*G_Dt;
        }
        // Set position controller
        // 设置位置控制器
        pos_control->set_pos_target_z_from_climb_rate_cm(_desired_v_z);

        // Update controllers
        // 更新控制器
        pos_control->update_z_controller();

        if ((now - _bail_time_start_ms)/1000.0f >= _bail_time) {
            // Bail out timer complete.  Change flight mode. Do not revert back to auto. Prevent aircraft
            // from continuing mission and potentially flying further away after a power failure.
            // 紧急中止计时器完成。改变飞行模式。不要恢复到自动模式。防止飞机
            // 在电源故障后继续任务并可能飞得更远。
            if (copter.prev_control_mode == Mode::Number::AUTO) {
                set_mode(Mode::Number::ALT_HOLD, ModeReason::AUTOROTATION_BAILOUT);
            } else {
                set_mode(copter.prev_control_mode, ModeReason::AUTOROTATION_BAILOUT);
            }
        }

        break;
        }
    }


    switch (nav_pos_switch) {

        case Navigation_Decision::USER_CONTROL_STABILISED:
        {
            // Operator is in control of roll and yaw.  Controls act as if in stabilise flight mode.  Pitch 
            // is controlled by speed-height controller.
            // 操作员控制横滚和偏航。控制行为就像在稳定飞行模式中一样。俯仰
            // 由速度-高度控制器控制。
            float pilot_roll, pilot_pitch;
            get_pilot_desired_lean_angles(pilot_roll, pilot_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

            // Get pilot's desired yaw rate
            // 获取飞行员期望的偏航速率
            float pilot_yaw_rate = get_pilot_desired_yaw_rate();

            // Pitch target is calculated in autorotation phase switch above
            // 俯仰目标在上面的自转阶段开关中计算
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pilot_roll, _pitch_target, pilot_yaw_rate);
            break;
        }

        case Navigation_Decision::STRAIGHT_AHEAD:
        case Navigation_Decision::INTO_WIND:
        case Navigation_Decision::NEAREST_RALLY:
        {
            break;
        }
    }

    // Output warning messaged if rpm signal is bad
    // 如果rpm信号不好,输出警告消息
    if (_flags.bad_rpm) {
        warning_message(1);
    }

} // End function run()

void ModeAutorotate::warning_message(uint8_t message_n)
{
    switch (message_n) {
        case 1:
        {
            if (_msg_flags.bad_rpm) {
                // Bad rpm sensor health.
                // rpm传感器健康状况不佳。
                gcs().send_text(MAV_SEVERITY_INFO, "Warning: Poor RPM Sensor Health");
                gcs().send_text(MAV_SEVERITY_INFO, "Action: Minimum Collective Applied");
                _msg_flags.bad_rpm = false;
            }
            break;
        }
    }
}

#endif
