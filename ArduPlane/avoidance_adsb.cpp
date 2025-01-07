#include <stdio.h>
#include "Plane.h"

#if HAL_ADSB_ENABLED
// 更新ADSB避障功能
void Plane::avoidance_adsb_update(void)
{
    adsb.update();
    avoidance_adsb.update();
}

// 处理避障动作
MAV_COLLISION_ACTION AP_Avoidance_Plane::handle_avoidance(const AP_Avoidance::Obstacle *obstacle, MAV_COLLISION_ACTION requested_action)
{
    MAV_COLLISION_ACTION actual_action = requested_action;
    bool failsafe_state_change = false;

    // 检查故障保护状态变化
    if (!plane.failsafe.adsb) {
        plane.failsafe.adsb = true;
        failsafe_state_change = true;
        // 记录当前飞行模式，以备恢复时使用
        prev_control_mode_number = plane.control_mode->mode_number();
    }

    // 在某些飞行模式下不采取行动
    bool flightmode_prohibits_action = false;
    if (plane.control_mode == &plane.mode_manual ||
        (plane.control_mode == &plane.mode_auto && !plane.auto_state.takeoff_complete) ||
        (plane.flight_stage == AP_FixedWing::FlightStage::LAND) || // TODO: 考虑在接近时允许操作
        plane.control_mode == &plane.mode_autotune) {
        flightmode_prohibits_action = true;
    }
#if HAL_QUADPLANE_ENABLED
    if (plane.control_mode == &plane.mode_qland) {
        flightmode_prohibits_action = true;
    }
#endif
    if (flightmode_prohibits_action) {
        actual_action = MAV_COLLISION_ACTION_NONE;
    }

    // 根据请求的动作采取行动
    switch (actual_action) {
        case MAV_COLLISION_ACTION_RTL:
            if (failsafe_state_change) {
                plane.set_mode(plane.mode_rtl, ModeReason::AVOIDANCE);
            }
            break;

        case MAV_COLLISION_ACTION_HOVER:
            if (failsafe_state_change) {
#if HAL_QUADPLANE_ENABLED
                if (plane.quadplane.is_flying()) {
                    plane.set_mode(plane.mode_qloiter, ModeReason::AVOIDANCE);
                    break;
                }
#endif
                plane.set_mode(plane.mode_loiter, ModeReason::AVOIDANCE);
            }
            break;

        case MAV_COLLISION_ACTION_ASCEND_OR_DESCEND: {
            // 爬升或下降以避开障碍物
            Location loc = plane.next_WP_loc;
            if (handle_avoidance_vertical(obstacle, failsafe_state_change, loc)) {
                plane.set_guided_WP(loc);
            } else {
                actual_action = MAV_COLLISION_ACTION_NONE;
            }
            break;
        }
        case MAV_COLLISION_ACTION_MOVE_HORIZONTALLY: {
            // 水平移动以避开障碍物
            Location loc = plane.next_WP_loc;
            if (handle_avoidance_horizontal(obstacle, failsafe_state_change, loc)) {
                plane.set_guided_WP(loc);
            } else {
                actual_action = MAV_COLLISION_ACTION_NONE;
            }
            break;
        }
        case MAV_COLLISION_ACTION_MOVE_PERPENDICULAR:
        {
            // 水平和垂直移动以避开障碍物
            Location loc = plane.next_WP_loc;
            const bool success_vert = handle_avoidance_vertical(obstacle, failsafe_state_change, loc);
            const bool success_hor = handle_avoidance_horizontal(obstacle, failsafe_state_change, loc);
            if (success_vert || success_hor) {
                plane.set_guided_WP(loc);
            } else {
                actual_action = MAV_COLLISION_ACTION_NONE;
            }
        }
        break;

        // 不支持的动作和不需要响应的动作
        case MAV_COLLISION_ACTION_NONE:
            return actual_action;
        case MAV_COLLISION_ACTION_REPORT:
        default:
            break;
    }

    if (failsafe_state_change) {
        gcs().send_text(MAV_SEVERITY_ALERT, "Avoid: Performing action: %d", actual_action);
    }

    // 返回执行的动作
    return actual_action;
}

// 处理恢复动作
void AP_Avoidance_Plane::handle_recovery(RecoveryAction recovery_action)
{
    // 检查是否正在退出故障保护状态
    if (plane.failsafe.adsb) {
        plane.failsafe.adsb = false;
        gcs().send_text(MAV_SEVERITY_INFO, "Avoid: Resuming with action: %u", (unsigned)recovery_action);

        // 如果请求恢复飞行模式且用户未更改模式，则恢复
        if (plane.control_mode_reason == ModeReason::AVOIDANCE) {
            switch (recovery_action) {
            case RecoveryAction::REMAIN_IN_AVOID_ADSB:
                // 保持在AVOID_ADSB模式
                break;

            case RecoveryAction::RESUME_PREVIOUS_FLIGHTMODE:
                plane.set_mode_by_number(prev_control_mode_number, ModeReason::AVOIDANCE_RECOVERY);
                break;

            case RecoveryAction::RTL:
                plane.set_mode(plane.mode_rtl, ModeReason::AVOIDANCE_RECOVERY);
                break;

            case RecoveryAction::RESUME_IF_AUTO_ELSE_LOITER:
                if (prev_control_mode_number == Mode::Number::AUTO) {
                    plane.set_mode(plane.mode_auto, ModeReason::AVOIDANCE_RECOVERY);
                } else {
                    // 让ModeAvoidADSB继续其引导行为，但重置盘旋位置
                    plane.set_guided_WP(plane.current_loc);
                }
                break;

            default:
                // 用户指定了无效的恢复动作，在当前位置盘旋
                plane.set_guided_WP(plane.current_loc);
                break;
            }
        }
    }
}

// 检查飞行模式是否为avoid_adsb
bool AP_Avoidance_Plane::check_flightmode(bool allow_mode_change)
{
    // 确保飞机处于avoid_adsb模式
    if (allow_mode_change && plane.control_mode != &plane.mode_avoidADSB) {
        plane.set_mode(plane.mode_avoidADSB, ModeReason::AVOIDANCE);
    }

    // 检查飞行模式
    return (plane.control_mode == &plane.mode_avoidADSB);
}

// 处理垂直避障
bool AP_Avoidance_Plane::handle_avoidance_vertical(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change, Location &new_loc)
{
    // 确保飞机处于avoid_adsb模式
    if (!check_flightmode(allow_mode_change)) {
        return false;
    }

    // 获取远离障碍物的最佳向量
    if (plane.current_loc.alt > obstacle->_location.alt) {
        // 应该爬升
        new_loc.alt = plane.current_loc.alt + 1000; // 设置高度要求为当前高度上方10米，爬升率将为TECS_CLMB_MAX
        return true;
    } else if (plane.current_loc.alt > plane.g.RTL_altitude*100) {
        // 当高于RTL高度时应该下降
        // TODO: 考虑使用低于RTL_altitude的高度，因为其默认值（100m）相当高
        new_loc.alt = plane.current_loc.alt - 1000; // 设置高度要求为当前高度下方10米，下降率将为TECS_SINK_MAX
        return true;
    }

    return false;
}

// 处理水平避障
bool AP_Avoidance_Plane::handle_avoidance_horizontal(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change, Location &new_loc)
{
    // 确保飞机处于avoid_adsb模式
    if (!check_flightmode(allow_mode_change)) {
        return false;
    }

    // 获取远离障碍物的最佳向量
    Vector3f velocity_neu;
    if (get_vector_perpendicular(obstacle, velocity_neu)) {
        // 移除垂直分量
        velocity_neu.z = 0.0f;

        // 检查除以零的情况
        if (is_zero(velocity_neu.x) && is_zero(velocity_neu.y)) {
            return false;
        }

        // 重新归一化
        velocity_neu.normalize();

        // 将向量推得更远
        velocity_neu *= 10000;

        // 设置目标
        new_loc.offset(velocity_neu.x, velocity_neu.y);
        return true;
    }

    // 如果到达这里，说明未能设置新目标
    return false;
}

#endif // HAL_ADSB_ENABLED
