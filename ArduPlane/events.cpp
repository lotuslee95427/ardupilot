#include "Plane.h"

// 返回车辆是否处于着陆序列中。仅用于故障保护代码。
bool Plane::failsafe_in_landing_sequence() const
{
    if (flight_stage == AP_FixedWing::FlightStage::LAND) {
        return true;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_land_sequence()) {
        return true;
    }
#endif
    if (mission.get_in_landing_sequence_flag()) {
        return true;
    }
    return false;
}

// 处理短时间失控信号故障保护事件
void Plane::failsafe_short_on_event(enum failsafe_state fstype, ModeReason reason)
{
    // 这是如何处理短时间失去控制信号的故障保护
    failsafe.state = fstype;
    failsafe.short_timer_ms = millis();
    failsafe.saved_mode_number = control_mode->mode_number();
    switch (control_mode->mode_number())
    {
    case Mode::Number::MANUAL:
    case Mode::Number::STABILIZE:
    case Mode::Number::ACRO:
    case Mode::Number::FLY_BY_WIRE_A:
    case Mode::Number::AUTOTUNE:
    case Mode::Number::FLY_BY_WIRE_B:
    case Mode::Number::CRUISE:
    case Mode::Number::TRAINING:  
        if(plane.emergency_landing) {
            set_mode(mode_fbwa, reason); // 紧急着陆开关覆盖正常操作，允许超出范围着陆
            break;
        }
        if(g.fs_action_short == FS_ACTION_SHORT_FBWA) {
            set_mode(mode_fbwa, reason);
        } else if (g.fs_action_short == FS_ACTION_SHORT_FBWB) {
            set_mode(mode_fbwb, reason);
        } else {
            set_mode(mode_circle, reason); // 如果动作 = 0 或 1，则进入盘旋模式
        }
        break;

#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QSTABILIZE:
    case Mode::Number::QLOITER:
    case Mode::Number::QHOVER:
#if QAUTOTUNE_ENABLED
    case Mode::Number::QAUTOTUNE:
#endif
    case Mode::Number::QACRO:
        if (quadplane.option_is_set(QuadPlane::OPTION::FS_RTL)) {
            set_mode(mode_rtl, reason);
        } else if (quadplane.option_is_set(QuadPlane::OPTION::FS_QRTL)) {
            set_mode(mode_qrtl, reason);
        } else {
            set_mode(mode_qland, reason);
        }
        break;
#endif // HAL_QUADPLANE_ENABLED

    case Mode::Number::AUTO: {
        if (failsafe_in_landing_sequence()) {
            // 在着陆序列中不执行故障保护
            break;
        }
        FALLTHROUGH;
    }
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
    case Mode::Number::LOITER:
    case Mode::Number::THERMAL:
        if (g.fs_action_short != FS_ACTION_SHORT_BESTGUESS) { // 如果动作 = 0(BESTGUESS)，这组模式不采取任何动作
            failsafe.saved_mode_number = control_mode->mode_number();
            if (g.fs_action_short == FS_ACTION_SHORT_FBWA) {
                set_mode(mode_fbwa, reason);
            } else if (g.fs_action_short == FS_ACTION_SHORT_FBWB) {
                set_mode(mode_fbwb, reason);
            } else {
                set_mode(mode_circle, reason);
            }
        }
         break;
    case Mode::Number::CIRCLE:  // 这些模式永远不会采取任何短时故障保护动作，继续当前模式
    case Mode::Number::TAKEOFF:
    case Mode::Number::RTL:
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QLAND:
    case Mode::Number::QRTL:
    case Mode::Number::LOITER_ALT_QLAND:
#endif
    case Mode::Number::INITIALISING:
        break;
    }
    if (failsafe.saved_mode_number != control_mode->mode_number()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "RC短时故障保护：切换到 %s", control_mode->name());
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "RC短时故障保护已启动");
    }
}

// 处理长时间失控信号故障保护事件
void Plane::failsafe_long_on_event(enum failsafe_state fstype, ModeReason reason)
{
    // 这是如何处理长时间失去控制信号的故障保护
    //  如果GCS锁定，我们允许控制恢复到RC
    RC_Channels::clear_overrides();
    failsafe.state = fstype;
    switch (control_mode->mode_number())
    {
    case Mode::Number::MANUAL:
    case Mode::Number::STABILIZE:
    case Mode::Number::ACRO:
    case Mode::Number::FLY_BY_WIRE_A:
    case Mode::Number::AUTOTUNE:
    case Mode::Number::FLY_BY_WIRE_B:
    case Mode::Number::CRUISE:
    case Mode::Number::TRAINING:
    case Mode::Number::CIRCLE:
    case Mode::Number::LOITER:
    case Mode::Number::THERMAL:
    case Mode::Number::TAKEOFF:
        if (plane.flight_stage == AP_FixedWing::FlightStage::TAKEOFF && !(g.fs_action_long == FS_ACTION_LONG_GLIDE || g.fs_action_long == FS_ACTION_LONG_PARACHUTE)) {
            // 如果在TAKEOFF模式的初始爬升阶段且FS动作不是滑翔或降落伞，则不执行故障保护
            // 如果初始爬升后仍处于故障保护状态，将重新调用长时故障保护
            long_failsafe_pending = true;
            break;
        }

        if(plane.emergency_landing) {
            set_mode(mode_fbwa, reason); // 紧急着陆开关覆盖正常操作，允许超出范围着陆
            break;
        }
        if(g.fs_action_long == FS_ACTION_LONG_PARACHUTE) {
#if HAL_PARACHUTE_ENABLED
            parachute_release();
#endif
        } else if (g.fs_action_long == FS_ACTION_LONG_GLIDE) {
            set_mode(mode_fbwa, reason);
        } else if (g.fs_action_long == FS_ACTION_LONG_AUTO) {
            set_mode(mode_auto, reason);
        } else {
            set_mode(mode_rtl, reason);
        }
        break;

#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QSTABILIZE:
    case Mode::Number::QHOVER:
    case Mode::Number::QLOITER:
    case Mode::Number::QACRO:
#if QAUTOTUNE_ENABLED
    case Mode::Number::QAUTOTUNE:
#endif
        if (quadplane.option_is_set(QuadPlane::OPTION::FS_RTL)) {
            set_mode(mode_rtl, reason);
        } else if (quadplane.option_is_set(QuadPlane::OPTION::FS_QRTL)) {
            set_mode(mode_qrtl, reason);
        } else {
            set_mode(mode_qland, reason);
        }
        break;
#endif  // HAL_QUADPLANE_ENABLED

    case Mode::Number::AUTO:
        if (failsafe_in_landing_sequence()) {
            // 在着陆序列中不执行故障保护
            break;
        }

#if HAL_QUADPLANE_ENABLED
        if (quadplane.in_vtol_takeoff()) {
            set_mode(mode_qland, reason);
            // 如果在VTOL起飞中，执行QLAND
            break;
        }
#endif
        FALLTHROUGH;

    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:

        if(g.fs_action_long == FS_ACTION_LONG_PARACHUTE) {
#if HAL_PARACHUTE_ENABLED
            parachute_release();
#endif
        } else if (g.fs_action_long == FS_ACTION_LONG_GLIDE) {
            set_mode(mode_fbwa, reason);
        } else if (g.fs_action_long == FS_ACTION_LONG_AUTO) {
            set_mode(mode_auto, reason);
        } else if (g.fs_action_long == FS_ACTION_LONG_RTL) {
            set_mode(mode_rtl, reason);
        }
        break;

    case Mode::Number::RTL:
        if (g.fs_action_long == FS_ACTION_LONG_AUTO) {
            set_mode(mode_auto, reason);
        }
        break;
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QLAND:
    case Mode::Number::QRTL:
    case Mode::Number::LOITER_ALT_QLAND:
#endif
    case Mode::Number::INITIALISING:
        break;
    }
    gcs().send_text(MAV_SEVERITY_WARNING, "%s 故障保护已启动: %s", (reason == ModeReason:: GCS_FAILSAFE) ? "GCS" : "RC长时", control_mode->name());
}

// 处理短时间故障保护解除事件
void Plane::failsafe_short_off_event(ModeReason reason)
{
    // 我们重新获得了无线电联系
    gcs().send_text(MAV_SEVERITY_WARNING, "短时故障保护已清除");
    failsafe.state = FAILSAFE_NONE;
    // 如果需要，恢复进入模式，但检查我们当前的模式是否仍然由于故障保护
    if (control_mode_reason == ModeReason::RADIO_FAILSAFE) { 
       set_mode_by_number(failsafe.saved_mode_number, ModeReason::RADIO_FAILSAFE_RECOVERY);
       gcs().send_text(MAV_SEVERITY_INFO,"飞行模式 %s 已恢复",control_mode->name());
    }
}

// 处理长时间故障保护解除事件
void Plane::failsafe_long_off_event(ModeReason reason)
{
    long_failsafe_pending = false;
    // 我们重新获得了与RC或GCS的无线电联系
    if (reason == ModeReason:: GCS_FAILSAFE) {
        gcs().send_text(MAV_SEVERITY_WARNING, "GCS故障保护已关闭");
    }
    else {
        gcs().send_text(MAV_SEVERITY_WARNING, "RC长时故障保护已清除");
    }
    failsafe.state = FAILSAFE_NONE;
}
void Plane::handle_battery_failsafe(const char *type_str, const int8_t action)
{
    switch ((Failsafe_Action)action) {
#if HAL_QUADPLANE_ENABLED
        case Failsafe_Action_Loiter_alt_QLand:
            if (quadplane.available()) {
                plane.set_mode(mode_loiter_qland, ModeReason::BATTERY_FAILSAFE);
                break;
            }
            FALLTHROUGH;

        case Failsafe_Action_QLand:
            if (quadplane.available()) {
                plane.set_mode(mode_qland, ModeReason::BATTERY_FAILSAFE);
                break;
            }
            FALLTHROUGH;
#endif // HAL_QUADPLANE_ENABLED
        case Failsafe_Action_Land: {
            bool already_landing = flight_stage == AP_FixedWing::FlightStage::LAND;
#if HAL_QUADPLANE_ENABLED
            if (control_mode == &mode_qland || control_mode == &mode_loiter_qland) {
                already_landing = true;
            }
#endif
            if (!already_landing && plane.have_position) {
                // 如果我们已经开始着陆，永远不要停止
                if (plane.mission.is_best_land_sequence(plane.current_loc)) {
                    // 继续任务，因为它将在更短的距离内达到着陆点
                    plane.mission.set_in_landing_sequence_flag(true);
                    break;
                }
                if (plane.mission.jump_to_landing_sequence(plane.current_loc)) {
                    plane.set_mode(mode_auto, ModeReason::BATTERY_FAILSAFE);
                    break;
                }
            }
            FALLTHROUGH;
        }
        case Failsafe_Action_RTL: {
            bool already_landing = flight_stage == AP_FixedWing::FlightStage::LAND;
#if HAL_QUADPLANE_ENABLED
            if (control_mode == &mode_qland || control_mode == &mode_loiter_qland ||
                quadplane.in_vtol_land_sequence()) {
                already_landing = true;
            }
#endif
            if (!already_landing) {
                // 如果我们已经开始着陆，永远不要停止
                if ((g.rtl_autoland == RtlAutoland::RTL_IMMEDIATE_DO_LAND_START) && plane.have_position && plane.mission.is_best_land_sequence(plane.current_loc)) {
                    // 继续任务，因为它将在更短的距离内达到着陆点
                    plane.mission.set_in_landing_sequence_flag(true);
                    break;
                }
                set_mode(mode_rtl, ModeReason::BATTERY_FAILSAFE);
                aparm.throttle_cruise.load();
            }
            break;
        }

        case Failsafe_Action_Terminate:
#if AP_ADVANCEDFAILSAFE_ENABLED
            char battery_type_str[17];
            snprintf(battery_type_str, 17, "%s battery", type_str);
            afs.gcs_terminate(true, battery_type_str);
#else
            arming.disarm(AP_Arming::Method::FAILSAFE_ACTION_TERMINATE);
#endif
            break;

        case Failsafe_Action_Parachute:
#if HAL_PARACHUTE_ENABLED
            parachute_release();
#endif
            break;

        case Failsafe_Action_None:
            // 实际上不做任何事情，但我们仍应标记系统已触发故障保护
            // 并确保所有适当的标志都向用户发出警告
            break;
    }
}
