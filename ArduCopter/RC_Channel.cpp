// 包含Copter头文件
#include "Copter.h"

// 包含RC_Channel头文件
#include "RC_Channel.h"


// 定义这两个宏并包含RC_Channels_VarInfo头文件,定义了所有车辆类型通用的参数信息
#define RC_CHANNELS_SUBCLASS RC_Channels_Copter
#define RC_CHANNEL_SUBCLASS RC_Channel_Copter

#include <RC_Channel/RC_Channels_VarInfo.h>

// 返回飞行模式通道号
int8_t RC_Channels_Copter::flight_mode_channel_number() const
{
    return copter.g.flight_mode_chan.get();
}

// 处理模式开关变化
void RC_Channel_Copter::mode_switch_changed(modeswitch_pos_t new_pos)
{
    // 检查新位置是否有效
    if (new_pos < 0 || (uint8_t)new_pos > copter.num_flight_modes) {
        // 如果无效则返回
        return;
    }

    // 尝试设置新的飞行模式
    if (!copter.set_mode((Mode::Number)copter.flight_modes[new_pos].get(), ModeReason::RC_COMMAND)) {
        return;
    }

    // 如果没有设置简单模式或超级简单模式的辅助开关
    if (!rc().find_channel_for_option(AUX_FUNC::SIMPLE_MODE) &&
        !rc().find_channel_for_option(AUX_FUNC::SUPERSIMPLE_MODE)) {
        // 则使用EEPROM中存储的参数设置简单模式
        if (BIT_IS_SET(copter.g.super_simple, new_pos)) {
            copter.set_simple_mode(Copter::SimpleMode::SUPERSIMPLE);
        } else {
            copter.set_simple_mode(BIT_IS_SET(copter.g.simple_modes, new_pos) ? Copter::SimpleMode::SIMPLE : Copter::SimpleMode::NONE);
        }
    }
}

// 检查是否处于遥控器故障保护状态
bool RC_Channels_Copter::in_rc_failsafe() const
{
    return copter.failsafe.radio;
}

// 检查是否有有效的遥控器输入
bool RC_Channels_Copter::has_valid_input() const
{
    if (in_rc_failsafe()) {
        return false;
    }
    if (copter.failsafe.radio_counter != 0) {
        return false;
    }
    return true;
}

// 检查是否应该运行油门解锁检查
bool RC_Channels_Copter::arming_check_throttle() const {
    if ((copter.g.throttle_behavior & THR_BEHAVE_FEEDBACK_FROM_MID_STICK) != 0) {
        // 如果配置了中位回弹油门,不运行AP_Arming检查
        // Copter在自己的解锁检查中已经检查了这种情况
        return false;
    }
    return RC_Channels::arming_check_throttle();
}

// 获取解锁通道
RC_Channel * RC_Channels_Copter::get_arming_channel(void) const
{
    return copter.channel_yaw;
}

// 初始化辅助功能
void RC_Channel_Copter::init_aux_function(const AUX_FUNC ch_option, const AuxSwitchPos ch_flag)
{
    // 初始化通道选项
    switch(ch_option) {
    // 以下功能不需要初始化:
    case AUX_FUNC::ALTHOLD:
    case AUX_FUNC::AUTO:
    case AUX_FUNC::AUTOTUNE_MODE:
    case AUX_FUNC::AUTOTUNE_TEST_GAINS:
    case AUX_FUNC::BRAKE:
    case AUX_FUNC::CIRCLE:
    case AUX_FUNC::DRIFT:
    case AUX_FUNC::FLIP:
    case AUX_FUNC::FLOWHOLD:
    case AUX_FUNC::FOLLOW:
    case AUX_FUNC::GUIDED:
    case AUX_FUNC::LAND:
    case AUX_FUNC::LOITER:
#if HAL_PARACHUTE_ENABLED
    case AUX_FUNC::PARACHUTE_RELEASE:
#endif
    case AUX_FUNC::POSHOLD:
    case AUX_FUNC::RESETTOARMEDYAW:
    case AUX_FUNC::RTL:
    case AUX_FUNC::SAVE_TRIM:
    case AUX_FUNC::SAVE_WP:
    case AUX_FUNC::SMART_RTL:
    case AUX_FUNC::STABILIZE:
    case AUX_FUNC::THROW:
    case AUX_FUNC::USER_FUNC1:
    case AUX_FUNC::USER_FUNC2:
    case AUX_FUNC::USER_FUNC3:
#if AP_WINCH_ENABLED
    case AUX_FUNC::WINCH_CONTROL:
#endif
    case AUX_FUNC::ZIGZAG:
    case AUX_FUNC::ZIGZAG_Auto:
    case AUX_FUNC::ZIGZAG_SaveWP:
    case AUX_FUNC::ACRO:
    case AUX_FUNC::AUTO_RTL:
    case AUX_FUNC::TURTLE:
    case AUX_FUNC::SIMPLE_HEADING_RESET:
    case AUX_FUNC::ARMDISARM_AIRMODE:
    case AUX_FUNC::TURBINE_START:
    case AUX_FUNC::FLIGHTMODE_PAUSE:
        break;
    // 需要初始化的功能:
    case AUX_FUNC::ACRO_TRAINER:
    case AUX_FUNC::ATTCON_ACCEL_LIM:
    case AUX_FUNC::ATTCON_FEEDFWD:
    case AUX_FUNC::INVERTED:
    case AUX_FUNC::MOTOR_INTERLOCK:
#if HAL_PARACHUTE_ENABLED
    case AUX_FUNC::PARACHUTE_3POS:      // 我们相信飞机会被锁定,所以即使开关在释放位置降落伞也不会释放
    case AUX_FUNC::PARACHUTE_ENABLE:
#endif
    case AUX_FUNC::PRECISION_LOITER:
#if AP_RANGEFINDER_ENABLED
    case AUX_FUNC::RANGEFINDER:
#endif
    case AUX_FUNC::SIMPLE_MODE:
    case AUX_FUNC::STANDBY:
    case AUX_FUNC::SUPERSIMPLE_MODE:
    case AUX_FUNC::SURFACE_TRACKING:
#if AP_WINCH_ENABLED
    case AUX_FUNC::WINCH_ENABLE:
#endif
    case AUX_FUNC::AIRMODE:
    case AUX_FUNC::FORCEFLYING:
    case AUX_FUNC::CUSTOM_CONTROLLER:
    case AUX_FUNC::WEATHER_VANE_ENABLE:
    case AUX_FUNC::TRANSMITTER_TUNING:
        run_aux_function(ch_option, ch_flag, AuxFuncTriggerSource::INIT);
        break;
    default:
        RC_Channel::init_aux_function(ch_option, ch_flag);
        break;
    }
}

// 根据辅助开关变化改变模式
void RC_Channel_Copter::do_aux_function_change_mode(const Mode::Number mode,
                                                    const AuxSwitchPos ch_flag)
{
    switch(ch_flag) {
    case AuxSwitchPos::HIGH: {
        // 启用模式(如果不可能则保持当前飞行模式)
        copter.set_mode(mode, ModeReason::AUX_FUNCTION);
        break;
    }
    default:
        // 如果我们当前在这个模式,则返回到飞行模式开关的飞行模式
        if (copter.flightmode->mode_number() == mode) {
            rc().reset_mode_switch();
        }
    }
}

// 执行辅助功能 - 实现由辅助开关调用的功能
bool RC_Channel_Copter::do_aux_function(const AUX_FUNC ch_option, const AuxSwitchPos ch_flag)
{
    switch(ch_option) {
        case AUX_FUNC::FLIP:
            // 如果开关打开,油门为正且实际在飞行,则执行翻转
            if (ch_flag == AuxSwitchPos::HIGH) {
                copter.set_mode(Mode::Number::FLIP, ModeReason::AUX_FUNCTION);
            }
            break;

        case AUX_FUNC::SIMPLE_MODE:
            // 低位=简单模式关闭,中位或高位打开简单模式
            copter.set_simple_mode((ch_flag == AuxSwitchPos::LOW) ? Copter::SimpleMode::NONE : Copter::SimpleMode::SIMPLE);
            break;

        case AUX_FUNC::SUPERSIMPLE_MODE: {
            Copter::SimpleMode newmode = Copter::SimpleMode::NONE;
            switch (ch_flag) {
            case AuxSwitchPos::LOW:
                break;
            case AuxSwitchPos::MIDDLE:
                newmode = Copter::SimpleMode::SIMPLE;
                break;
            case AuxSwitchPos::HIGH:
                newmode = Copter::SimpleMode::SUPERSIMPLE;
                break;
            }
            copter.set_simple_mode(newmode);
            break;
        }

#if MODE_RTL_ENABLED
        case AUX_FUNC::RTL:
            do_aux_function_change_mode(Mode::Number::RTL, ch_flag);
            break;
#endif

        case AUX_FUNC::SAVE_TRIM:
            if ((ch_flag == AuxSwitchPos::HIGH) &&
                (copter.flightmode->allows_save_trim()) &&
                (copter.channel_throttle->get_control_in() == 0)) {
                copter.save_trim();
            }
            break;

#if MODE_AUTO_ENABLED
        case AUX_FUNC::SAVE_WP:
            // 当开关变为高位时保存航点
            if (ch_flag == RC_Channel::AuxSwitchPos::HIGH) {

                // 在自动模式或未解锁时不允许保存新航点
                if (copter.flightmode == &copter.mode_auto || !copter.motors->armed()) {
                    break;
                }

                // 不允许在油门为零时保存第一个航点
                if ((copter.mode_auto.mission.num_commands() == 0) && (copter.channel_throttle->get_control_in() == 0)) {
                    break;
                }

                // 创建新的任务命令
                AP_Mission::Mission_Command cmd  = {};

                // 如果任务为空则保存起飞命令
                if (copter.mode_auto.mission.num_commands() == 0) {
                    // 设置位置ID为16, MAV_CMD_NAV_WAYPOINT
                    cmd.id = MAV_CMD_NAV_TAKEOFF;
                    cmd.content.location.alt = MAX(copter.current_loc.alt,100);

                    // 使用当前高度作为起飞目标高度
                    // 对于起飞来说,只有高度对AP任务脚本有意义
                    if (copter.mode_auto.mission.add_cmd(cmd)) {
                        // 记录事件
                        LOGGER_WRITE_EVENT(LogEvent::SAVEWP_ADD_WP);
                    }
                }

                // 将新航点设置为当前位置
                cmd.content.location = copter.current_loc;

                // 如果油门大于零,创建航点命令
                if (copter.channel_throttle->get_control_in() > 0) {
                    cmd.id = MAV_CMD_NAV_WAYPOINT;
                } else {
                    // 油门为零时,创建降落命令
                    cmd.id = MAV_CMD_NAV_LAND;
                }

                // 保存命令
                if (copter.mode_auto.mission.add_cmd(cmd)) {
                    // 记录事件
                    LOGGER_WRITE_EVENT(LogEvent::SAVEWP_ADD_WP);
                }
            }
            break;

        case AUX_FUNC::AUTO:
            do_aux_function_change_mode(Mode::Number::AUTO, ch_flag);
            break;
#endif

#if AP_RANGEFINDER_ENABLED
        case AUX_FUNC::RANGEFINDER:
            // 启用或禁用测距仪
            if ((ch_flag == AuxSwitchPos::HIGH) &&
                copter.rangefinder.has_orientation(ROTATION_PITCH_270)) {
                copter.rangefinder_state.enabled = true;
            } else {
                copter.rangefinder_state.enabled = false;
            }
            break;
#endif // AP_RANGEFINDER_ENABLED

#if MODE_ACRO_ENABLED
        case AUX_FUNC::ACRO_TRAINER:
            // 根据开关位置设置特技训练模式
            switch(ch_flag) {
                case AuxSwitchPos::LOW:
                    // 关闭特技训练
                    copter.g.acro_trainer.set((uint8_t)ModeAcro::Trainer::OFF);
                    LOGGER_WRITE_EVENT(LogEvent::ACRO_TRAINER_OFF);
                    break;
                case AuxSwitchPos::MIDDLE:
                    // 设置为水平训练模式
                    copter.g.acro_trainer.set((uint8_t)ModeAcro::Trainer::LEVELING);
                    LOGGER_WRITE_EVENT(LogEvent::ACRO_TRAINER_LEVELING);
                    break;
                case AuxSwitchPos::HIGH:
                    // 设置为限制训练模式
                    copter.g.acro_trainer.set((uint8_t)ModeAcro::Trainer::LIMITED);
                    LOGGER_WRITE_EVENT(LogEvent::ACRO_TRAINER_LIMITED);
                    break;
            }
            break;
#endif

#if AUTOTUNE_ENABLED
        case AUX_FUNC::AUTOTUNE_MODE:
            // 切换到自动调参模式
            do_aux_function_change_mode(Mode::Number::AUTOTUNE, ch_flag);
            break;
        case AUX_FUNC::AUTOTUNE_TEST_GAINS:
            // 测试自动调参的增益值
            copter.mode_autotune.autotune.do_aux_function(ch_flag);
            break;
#endif

        case AUX_FUNC::LAND:
            // 切换到降落模式
            do_aux_function_change_mode(Mode::Number::LAND, ch_flag);
            break;

        case AUX_FUNC::GUIDED:
            // 切换到引导模式
            do_aux_function_change_mode(Mode::Number::GUIDED, ch_flag);
            break;

        case AUX_FUNC::LOITER:
            // 切换到悬停模式
            do_aux_function_change_mode(Mode::Number::LOITER, ch_flag);
            break;

        case AUX_FUNC::FOLLOW:
            // 切换到跟随模式
            do_aux_function_change_mode(Mode::Number::FOLLOW, ch_flag);
            break;

#if HAL_PARACHUTE_ENABLED
        case AUX_FUNC::PARACHUTE_ENABLE:
            // 启用/禁用降落伞
            copter.parachute.enabled(ch_flag == AuxSwitchPos::HIGH);
            break;

        case AUX_FUNC::PARACHUTE_RELEASE:
            // 手动释放降落伞
            if (ch_flag == AuxSwitchPos::HIGH) {
                copter.parachute_manual_release();
            }
            break;

        case AUX_FUNC::PARACHUTE_3POS:
            // 使用三位开关控制降落伞:禁用、启用、释放
            switch (ch_flag) {
                case AuxSwitchPos::LOW:
                    copter.parachute.enabled(false);
                    break;
                case AuxSwitchPos::MIDDLE:
                    copter.parachute.enabled(true);
                    break;
                case AuxSwitchPos::HIGH:
                    copter.parachute.enabled(true);
                    copter.parachute_manual_release();
                    break;
            }
            break;
#endif  // HAL_PARACHUTE_ENABLED

        case AUX_FUNC::ATTCON_FEEDFWD:
            // 启用或禁用前馈控制
            copter.attitude_control->bf_feedforward(ch_flag == AuxSwitchPos::HIGH);
            break;

        case AUX_FUNC::ATTCON_ACCEL_LIM:
            // 通过恢复默认值来启用或禁用加速度限制
            copter.attitude_control->accel_limiting(ch_flag == AuxSwitchPos::HIGH);
            break;

        case AUX_FUNC::MOTOR_INTERLOCK:
#if FRAME_CONFIG == HELI_FRAME
            // 直升机框架下的电机联锁逻辑
            // ROTOR_CONTROL_MODE_PASSTHROUGH模式下的联锁逻辑在heli_update_rotor_speed_targets中处理
            if (copter.motors->get_rsc_mode() != ROTOR_CONTROL_MODE_PASSTHROUGH) {
                copter.ap.motor_interlock_switch = (ch_flag == AuxSwitchPos::HIGH || ch_flag == AuxSwitchPos::MIDDLE);
            }
#else
            // 其他框架下的电机联锁逻辑
            copter.ap.motor_interlock_switch = (ch_flag == AuxSwitchPos::HIGH || ch_flag == AuxSwitchPos::MIDDLE);
#endif
            break;

#if FRAME_CONFIG == HELI_FRAME
        case AUX_FUNC::TURBINE_START:
           // 涡轮启动控制
           switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    copter.motors->set_turb_start(true);
                    break;
                case AuxSwitchPos::MIDDLE:
                    // 中间位置不做任何操作
                    break;
                case AuxSwitchPos::LOW:
                    copter.motors->set_turb_start(false);
                    break;
           }
           break;
#endif

#if MODE_BRAKE_ENABLED
        case AUX_FUNC::BRAKE:
            // 切换到刹车模式
            do_aux_function_change_mode(Mode::Number::BRAKE, ch_flag);
            break;
#endif

#if MODE_THROW_ENABLED
        case AUX_FUNC::THROW:
            // 切换到抛飞模式
            do_aux_function_change_mode(Mode::Number::THROW, ch_flag);
            break;
#endif

#if AC_PRECLAND_ENABLED && MODE_LOITER_ENABLED
        case AUX_FUNC::PRECISION_LOITER:
            // 精确悬停控制
            switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    copter.mode_loiter.set_precision_loiter_enabled(true);
                    break;
                case AuxSwitchPos::MIDDLE:
                    // 中间位置不做任何操作
                    break;
                case AuxSwitchPos::LOW:
                    copter.mode_loiter.set_precision_loiter_enabled(false);
                    break;
            }
            break;
#endif

#if MODE_SMARTRTL_ENABLED
        case AUX_FUNC::SMART_RTL:
            // 切换到智能返航模式
            do_aux_function_change_mode(Mode::Number::SMART_RTL, ch_flag);
            break;
#endif

#if FRAME_CONFIG == HELI_FRAME
        case AUX_FUNC::INVERTED:
            // 倒飞控制
            switch (ch_flag) {
            case AuxSwitchPos::HIGH:
                if (copter.flightmode->allows_inverted()) {
                    copter.attitude_control->set_inverted_flight(true);
                } else {
                    gcs().send_text(MAV_SEVERITY_WARNING, "Inverted flight not available in %s mode", copter.flightmode->name());
                }
                break;
            case AuxSwitchPos::MIDDLE:
                // 中间位置不做任何操作
                break;
            case AuxSwitchPos::LOW:
                copter.attitude_control->set_inverted_flight(false);
                break;
            }
            break;
#endif

#if AP_WINCH_ENABLED
        case AUX_FUNC::WINCH_ENABLE:
            // 绞车控制
            switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    // 高位停止绞车(使用速率控制)
                    copter.g2.winch.set_desired_rate(0.0f);
                    break;
                case AuxSwitchPos::MIDDLE:
                case AuxSwitchPos::LOW:
                    // 其他位置放松绞车
                    copter.g2.winch.relax();
                    break;
                }
            break;

        case AUX_FUNC::WINCH_CONTROL:
            // 绞车速率控制(在AP_Winch中处理)
            break;
#endif  // AP_WINCH_ENABLED

#ifdef USERHOOK_AUXSWITCH
        case AUX_FUNC::USER_FUNC1:
            // 用户自定义功能1
            copter.userhook_auxSwitch1(ch_flag);
            break;

        case AUX_FUNC::USER_FUNC2:
            // 用户自定义功能2
            copter.userhook_auxSwitch2(ch_flag);
            break;

        case AUX_FUNC::USER_FUNC3:
            // 用户自定义功能3
            copter.userhook_auxSwitch3(ch_flag);
            break;
#endif

#if MODE_ZIGZAG_ENABLED
        case AUX_FUNC::ZIGZAG:
            // 切换到之字形模式
            do_aux_function_change_mode(Mode::Number::ZIGZAG, ch_flag);
            break;

        case AUX_FUNC::ZIGZAG_SaveWP:
            // 之字形航点保存
            if (copter.flightmode == &copter.mode_zigzag) {
                // 初始化之字形自动模式
                copter.mode_zigzag.init_auto();
                switch (ch_flag) {
                    case AuxSwitchPos::LOW:
                        copter.mode_zigzag.save_or_move_to_destination(ModeZigZag::Destination::A);
                        break;
                    case AuxSwitchPos::MIDDLE:
                        copter.mode_zigzag.return_to_manual_control(false);
                        break;
                    case AuxSwitchPos::HIGH:
                        copter.mode_zigzag.save_or_move_to_destination(ModeZigZag::Destination::B);
                        break;
                }
            }
            break;
#endif

        case AUX_FUNC::STABILIZE:
            // 切换到自稳模式
            do_aux_function_change_mode(Mode::Number::STABILIZE, ch_flag);
            break;

#if MODE_POSHOLD_ENABLED
        case AUX_FUNC::POSHOLD:
            // 切换到定点模式
            do_aux_function_change_mode(Mode::Number::POSHOLD, ch_flag);
            break;
#endif

        case AUX_FUNC::ALTHOLD:
            // 切换到定高模式
            do_aux_function_change_mode(Mode::Number::ALT_HOLD, ch_flag);
            break;

#if MODE_ACRO_ENABLED
        case AUX_FUNC::ACRO:
            // 切换到特技模式
            do_aux_function_change_mode(Mode::Number::ACRO, ch_flag);
            break;
#endif

#if MODE_FLOWHOLD_ENABLED
        case AUX_FUNC::FLOWHOLD:
            // 切换到光流定点模式
            do_aux_function_change_mode(Mode::Number::FLOWHOLD, ch_flag);
            break;
#endif

#if MODE_CIRCLE_ENABLED
        case AUX_FUNC::CIRCLE:
            // 切换到绕圈模式
            do_aux_function_change_mode(Mode::Number::CIRCLE, ch_flag);
            break;
#endif

#if MODE_DRIFT_ENABLED
        case AUX_FUNC::DRIFT:
            // 切换到漂移模式
            do_aux_function_change_mode(Mode::Number::DRIFT, ch_flag);
            break;
#endif

        case AUX_FUNC::STANDBY: {
            switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    // 启用待机模式
                    copter.standby_active = true;
                    LOGGER_WRITE_EVENT(LogEvent::STANDBY_ENABLE);
                    gcs().send_text(MAV_SEVERITY_INFO, "Stand By Enabled");
                    break;
                default:
                    // 禁用待机模式
                    copter.standby_active = false;
                    LOGGER_WRITE_EVENT(LogEvent::STANDBY_DISABLE);
                    gcs().send_text(MAV_SEVERITY_INFO, "Stand By Disabled");
                    break;
                }
            break;
        }

#if AP_RANGEFINDER_ENABLED
        case AUX_FUNC::SURFACE_TRACKING:
            switch (ch_flag) {
            case AuxSwitchPos::LOW:
                // 设置为地面跟踪
                copter.surface_tracking.set_surface(Copter::SurfaceTracking::Surface::GROUND);
                break;
            case AuxSwitchPos::MIDDLE:
                // 关闭表面跟踪
                copter.surface_tracking.set_surface(Copter::SurfaceTracking::Surface::NONE);
                break;
            case AuxSwitchPos::HIGH:
                // 设置为天花板跟踪
                copter.surface_tracking.set_surface(Copter::SurfaceTracking::Surface::CEILING);
                break;
            }
            break;
#endif

        case AUX_FUNC::FLIGHTMODE_PAUSE:
            switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    // 尝试暂停当前飞行模式
                    if (!copter.flightmode->pause()) {
                        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Flight Mode Pause failed");
                    }
                    break;
                case AuxSwitchPos::MIDDLE:
                    break;
                case AuxSwitchPos::LOW:
                    // 恢复飞行模式
                    copter.flightmode->resume();
                    break;
            }
            break;

#if MODE_ZIGZAG_ENABLED
        case AUX_FUNC::ZIGZAG_Auto:
            // 如果当前是Z字形模式
            if (copter.flightmode == &copter.mode_zigzag) {
                switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    // 运行自动Z字形
                    copter.mode_zigzag.run_auto();
                    break;
                default:
                    // 暂停自动Z字形
                    copter.mode_zigzag.suspend_auto();
                    break;
                }
            }
            break;
#endif

        case AUX_FUNC::AIRMODE:
            // 改变空中模式状态
            do_aux_function_change_air_mode(ch_flag);
#if MODE_ACRO_ENABLED && FRAME_CONFIG != HELI_FRAME
            copter.mode_acro.air_mode_aux_changed();
#endif
            break;

        case AUX_FUNC::FORCEFLYING:
            // 改变强制飞行状态
            do_aux_function_change_force_flying(ch_flag);
            break;

#if MODE_AUTO_ENABLED
        case AUX_FUNC::AUTO_RTL:
            // 切换到自动返航模式
            do_aux_function_change_mode(Mode::Number::AUTO_RTL, ch_flag);
            break;
#endif

#if MODE_TURTLE_ENABLED
        case AUX_FUNC::TURTLE:
            // 切换到乌龟模式
            do_aux_function_change_mode(Mode::Number::TURTLE, ch_flag);
            break;
#endif

        case AUX_FUNC::SIMPLE_HEADING_RESET:
            if (ch_flag == AuxSwitchPos::HIGH) {
                // 重置简单模式航向
                copter.init_simple_bearing();
                gcs().send_text(MAV_SEVERITY_INFO, "Simple heading reset");
            }
            break;

        case AUX_FUNC::ARMDISARM_AIRMODE:
            // 执行解锁/锁定操作
            RC_Channel::do_aux_function_armdisarm(ch_flag);
            if (copter.arming.is_armed()) {
                copter.ap.armed_with_airmode_switch = true;
            }
            break;

#if AC_CUSTOMCONTROL_MULTI_ENABLED
        case AUX_FUNC::CUSTOM_CONTROLLER:
            // 设置自定义控制器状态
            copter.custom_control.set_custom_controller(ch_flag == AuxSwitchPos::HIGH);
            break;
#endif

#if WEATHERVANE_ENABLED
    case AUX_FUNC::WEATHER_VANE_ENABLE: {
            switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    // 启用风向标功能
                    copter.g2.weathervane.allow_weathervaning(true);
                    break;
                case AuxSwitchPos::MIDDLE:
                    break;
                case AuxSwitchPos::LOW:
                    // 禁用风向标功能
                    copter.g2.weathervane.allow_weathervaning(false);
                    break;
            }
            break;
        }
#endif
    case AUX_FUNC::TRANSMITTER_TUNING:
        // 用于发射机调参,在tuning.cpp中处理
        break;

    default:
        // 处理其他辅助功能
        return RC_Channel::do_aux_function(ch_option, ch_flag);
    }
    return true;
}

// 改变空中模式状态
void RC_Channel_Copter::do_aux_function_change_air_mode(const AuxSwitchPos ch_flag)
{
    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        // 启用空中模式
        copter.air_mode = AirMode::AIRMODE_ENABLED;
        break;
    case AuxSwitchPos::MIDDLE:
        break;
    case AuxSwitchPos::LOW:
        // 禁用空中模式
        copter.air_mode = AirMode::AIRMODE_DISABLED;
        break;
    }
}

// 改变强制飞行状态
void RC_Channel_Copter::do_aux_function_change_force_flying(const AuxSwitchPos ch_flag)
{
    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        // 启用强制飞行
        copter.force_flying = true;
        break;
    case AuxSwitchPos::MIDDLE:
        break;
    case AuxSwitchPos::LOW:
        // 禁用强制飞行
        copter.force_flying = false;
        break;
    }
}

// 保存微调 - 将遥控器的横滚和俯仰微调添加到AHRS中
void Copter::save_trim()
{
    // 保存横滚和俯仰微调
    float roll_trim = ToRad((float)channel_roll->get_control_in()/100.0f);
    float pitch_trim = ToRad((float)channel_pitch->get_control_in()/100.0f);
    ahrs.add_trim(roll_trim, pitch_trim);
    LOGGER_WRITE_EVENT(LogEvent::SAVE_TRIM);
    gcs().send_text(MAV_SEVERITY_INFO, "Trim saved");
}

// 取消自动微调
void Copter::auto_trim_cancel()
{
    auto_trim_counter = 0;
    AP_Notify::flags.save_trim = false;
    gcs().send_text(MAV_SEVERITY_INFO, "AutoTrim cancelled");
}

// 自动微调 - 根据当前摇杆位置稍微调整ahrs.roll_trim和ahrs.pitch_trim
// 在飞行员试图保持飞机水平时持续调用
void Copter::auto_trim()
{
    if (auto_trim_counter > 0) {
        // 检查是否在自稳模式且电机已解锁
        if (copter.flightmode != &copter.mode_stabilize ||
            !copter.motors->armed()) {
            auto_trim_cancel();
            return;
        }

        // 闪烁LED指示灯
        AP_Notify::flags.save_trim = true;

        if (!auto_trim_started) {
            if (ap.land_complete) {
                // 还未起飞
                return;
            }
            auto_trim_started = true;
        }

        if (ap.land_complete) {
            // 已经着陆
            auto_trim_cancel();
            return;
        }

        auto_trim_counter--;

        // 计算横滚微调调整量
        float roll_trim_adjustment = ToRad((float)channel_roll->get_control_in() / 4000.0f);

        // 计算俯仰微调调整量
        float pitch_trim_adjustment = ToRad((float)channel_pitch->get_control_in() / 4000.0f);

        // 将微调添加到AHRS对象
        // 在最后一次迭代时保存到EEPROM
        ahrs.add_trim(roll_trim_adjustment, pitch_trim_adjustment, (auto_trim_counter == 0));

        // 在最后一次迭代时恢复LED和加速度计增益到正常状态
        if (auto_trim_counter == 0) {
            AP_Notify::flags.save_trim = false;
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTrim: Trims saved");
        }
    }
}
