#include "Plane.h"

// 集成AC_Fence库与主ArduPlane代码

#if AP_FENCE_ENABLED

// fence_check - 要求fence库检查违规并启动响应
void Plane::fence_check()
{
    const uint8_t orig_breaches = fence.get_breaches();
    const bool armed = arming.is_armed();

    uint16_t mission_id = plane.mission.get_current_nav_cmd().id;
    bool landing_or_landed = plane.flight_stage == AP_FixedWing::FlightStage::LAND
                         || !armed
#if HAL_QUADPLANE_ENABLED
                         || control_mode->mode_number() == Mode::Number::QLAND
                         || quadplane.in_vtol_land_descent()
#endif
                         || (plane.is_land_command(mission_id) && plane.mission.state() == AP_Mission::MISSION_RUNNING);

    // 检查新的违规；new_breaches是违反的围栏类型的位掩码
    const uint8_t new_breaches = fence.check(landing_or_landed);

    /*
      如果我们已解除武装，或者我们当前没有违规且没有飞行，
      则清除与之前模式违规处理相关的状态。这允许围栏状态机
      在围栏违规动作（如RTL和自动着陆）结束时重置
     */
    if (plane.previous_mode_reason == ModeReason::FENCE_BREACHED) {
        if (!armed || ((new_breaches == 0 && orig_breaches == 0) && !plane.is_flying())) {
            plane.previous_mode_reason = ModeReason::UNKNOWN;
        }
    }

    if (!fence.enabled()) {
        // 如果仍在GUIDED模式返回点，则切换回所选的控制模式
        switch(fence.get_action()) {
            case AC_FENCE_ACTION_GUIDED:
            case AC_FENCE_ACTION_GUIDED_THROTTLE_PASS:
            case AC_FENCE_ACTION_RTL_AND_LAND:
                if (plane.control_mode_reason == ModeReason::FENCE_BREACHED &&
                    control_mode->is_guided_mode()) {
                    set_mode(*previous_mode, ModeReason::FENCE_RETURN_PREVIOUS_MODE);
                }
                break;
            default:
                // 除非我们的动作允许，否则不返回到之前的模式
                break;
        }
        return;
    }

    // 当解除武装时我们仍然不做任何事，但我们会检查围栏违规。
    // 围栏预解锁检查实际上会检查是否有任何围栏被违反
    // 如果我们在解除武装时不调用AP_Fence的check，这永远不会为真
    if (!armed) {
        return;
    }

    // 在着陆的最后阶段绝不触发围栏违规
    if (landing.is_expecting_impact()) {
        return;
    }

    if (in_fence_recovery()) {
        // 我们已经触发，在用户使用围栏通道开关禁用/重新启用之前不再触发
        return;
    }

    if (new_breaches) {
        fence.print_fence_message("breached", new_breaches);

        // 如果用户想要某种响应且电机已武装
        const uint8_t fence_act = fence.get_action();
        switch (fence_act) {
        case AC_FENCE_ACTION_REPORT_ONLY:
            break;
        case AC_FENCE_ACTION_GUIDED:
        case AC_FENCE_ACTION_GUIDED_THROTTLE_PASS:
        case AC_FENCE_ACTION_RTL_AND_LAND:
            if (fence_act == AC_FENCE_ACTION_RTL_AND_LAND) {
                if (control_mode == &mode_auto &&
                    mission.get_in_landing_sequence_flag() &&
                    (g.rtl_autoland == RtlAutoland::RTL_THEN_DO_LAND_START ||
                     g.rtl_autoland == RtlAutoland::RTL_IMMEDIATE_DO_LAND_START)) {
                    // 已经在着陆
                    return;
                }
                set_mode(mode_rtl, ModeReason::FENCE_BREACHED);
            } else {
                set_mode(mode_guided, ModeReason::FENCE_BREACHED);
            }

            Location loc;
            if (fence.get_return_rally() != 0 || fence_act == AC_FENCE_ACTION_RTL_AND_LAND) {
                loc = calc_best_rally_or_home_location(current_loc, get_RTL_altitude_cm());
            } else {
                // 返回到围栏返回点，而不是集结点
                if (fence.get_return_altitude() > 0) {
                    // 使用_retalt飞到返回点
                    loc.alt = home.alt + 100.0f * fence.get_return_altitude();
                } else if (fence.get_safe_alt_min() >= fence.get_safe_alt_max()) {
                    // 无效的最小/最大值，使用RTL_altitude
                    loc.alt = home.alt + g.RTL_altitude*100;
                } else {
                    // 飞到返回点，高度在最小和最大之间
                    loc.alt = home.alt + 100.0f * (fence.get_safe_alt_min() + fence.get_safe_alt_max()) / 2;
                }

                Vector2l return_point;
                if(fence.polyfence().get_return_point(return_point)) {
                    loc.lat = return_point[0];
                    loc.lng = return_point[1];
                } else {
                    // 当找不到围栏返回点时（即没有上传包含围栏，但上传了排除围栏）
                    // 我们无法获得有效的围栏返回点。在这种情况下，家被认为是安全的返回点。
                    loc.lat = home.lat;
                    loc.lng = home.lng;
                }
            }

            if (fence.get_action() != AC_FENCE_ACTION_RTL_AND_LAND) {
                setup_terrain_target_alt(loc);
                set_guided_WP(loc);
            }

            if (fence.get_action() == AC_FENCE_ACTION_GUIDED_THROTTLE_PASS) {
                guided_throttle_passthru = true;
            }
            break;
        }

        LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode(new_breaches));
    } else if (orig_breaches && fence.get_breaches() == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Fence breach cleared");
        // 记录违规清除
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode::ERROR_RESOLVED);
    }
}

// 围栏摇杆混合
bool Plane::fence_stickmixing(void) const
{
    if (fence.enabled() &&
        fence.get_breaches() &&
        in_fence_recovery())
    {
        // 不混合用户输入
        return false;
    }
    // 正常混合规则
    return true;
}

// 是否在围栏恢复中
bool Plane::in_fence_recovery() const
{
    const bool current_mode_breach = plane.control_mode_reason == ModeReason::FENCE_BREACHED;
    const bool previous_mode_breach = plane.previous_mode_reason ==  ModeReason::FENCE_BREACHED;
    const bool previous_mode_complete = (plane.control_mode_reason == ModeReason::RTL_COMPLETE_SWITCHING_TO_VTOL_LAND_RTL) ||
                                        (plane.control_mode_reason == ModeReason::RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND) ||
                                        (plane.control_mode_reason == ModeReason::QRTL_INSTEAD_OF_RTL) ||
                                        (plane.control_mode_reason == ModeReason::QLAND_INSTEAD_OF_RTL);

    return current_mode_breach || (previous_mode_breach && previous_mode_complete);
}

#endif
