#include "Copter.h"

#if MODE_SMARTRTL_ENABLED

/*
 * Init and run calls for Smart_RTL flight mode
 * Smart_RTL 飞行模式的初始化和运行调用
 *
 * This code uses the SmartRTL path that is already in memory, and feeds it into WPNav, one point at a time.
 * 此代码使用已经存储在内存中的 SmartRTL 路径，并一次将一个点输入到 WPNav 中。
 * Once the copter is close to home, it will run a standard land controller.
 * 一旦飞行器接近家的位置，它将运行标准的着陆控制器。
 */

bool ModeSmartRTL::init(bool ignore_checks)
{
    if (g2.smart_rtl.is_active()) {
        // initialise waypoint and spline controller
        // 初始化航点和样条控制器
        wp_nav->wp_and_spline_init();

        // set current target to a reasonable stopping point
        // 将当前目标设置为一个合理的停止点
        Vector3p stopping_point;
        pos_control->get_stopping_point_xy_cm(stopping_point.xy());
        pos_control->get_stopping_point_z_cm(stopping_point.z);
        wp_nav->set_wp_destination(stopping_point.tofloat());

        // initialise yaw to obey user parameter
        // 初始化偏航以遵守用户参数
        auto_yaw.set_mode_to_default(true);

        // wait for cleanup of return path
        // 等待返回路径的清理
        smart_rtl_state = SubMode::WAIT_FOR_PATH_CLEANUP;
        return true;
    }

    return false;
}

// perform cleanup required when leaving smart_rtl
// 执行离开 smart_rtl 时所需的清理工作
void ModeSmartRTL::exit()
{
    // restore last point if we hadn't reached it
    // 如果我们还没有到达最后一个点，则恢复它
    if (smart_rtl_state == SubMode::PATH_FOLLOW && !dest_NED_backup.is_zero()) {
        if (!g2.smart_rtl.add_point(dest_NED_backup)) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SmartRTL: lost one point");
        }
    }
    dest_NED_backup.zero();

    g2.smart_rtl.cancel_request_for_thorough_cleanup();
}

void ModeSmartRTL::run()
{
    switch (smart_rtl_state) {
        case SubMode::WAIT_FOR_PATH_CLEANUP:
            wait_cleanup_run();
            break;
        case SubMode::PATH_FOLLOW:
            path_follow_run();
            break;
        case SubMode::PRELAND_POSITION:
            pre_land_position_run();
            break;
        case SubMode::DESCEND:
            descent_run(); // Re-using the descend method from normal rtl mode.
                           // 重用普通 RTL 模式的下降方法
            break;
        case SubMode::LAND:
            land_run(true); // Re-using the land method from normal rtl mode.
                            // 重用普通 RTL 模式的着陆方法
            break;
    }
}

bool ModeSmartRTL::is_landing() const
{
    return smart_rtl_state == SubMode::LAND;
}

void ModeSmartRTL::wait_cleanup_run()
{
    // hover at current target position
    // 在当前目标位置悬停
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    wp_nav->update_wpnav();
    pos_control->update_z_controller();
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());

    // check if return path is computed and if yes, begin journey home
    // 检查是否已计算返回路径，如果是，则开始返回家的旅程
    if (g2.smart_rtl.request_thorough_cleanup()) {
        path_follow_last_pop_fail_ms = 0;
        smart_rtl_state = SubMode::PATH_FOLLOW;
    }
}

void ModeSmartRTL::path_follow_run()
{
    // if we are close to current target point, switch the next point to be our target.
    // 如果我们接近当前目标点，将下一个点切换为我们的目标
    if (wp_nav->reached_wp_destination()) {

        // clear destination backup so that it cannot be restored
        // 清除目标备份，使其无法恢复
        dest_NED_backup.zero();

        // this pop_point can fail if the IO task currently has the
        // path semaphore.
        // 如果 IO 任务当前持有路径信号量，这个 pop_point 可能会失败
        Vector3f dest_NED;
        if (g2.smart_rtl.pop_point(dest_NED)) {
            // backup destination in case we exit smart_rtl mode and need to restore it to the path
            // 备份目标，以防我们退出 smart_rtl 模式并需要将其恢复到路径中
            dest_NED_backup = dest_NED;
            path_follow_last_pop_fail_ms = 0;
            if (g2.smart_rtl.get_num_points() == 0) {
                // this is the very last point, add 2m to the target alt and move to pre-land state
                // 这是最后一个点，将目标高度增加 2m 并移动到着陆前状态
                dest_NED.z -= 2.0f;
                smart_rtl_state = SubMode::PRELAND_POSITION;
                wp_nav->set_wp_destination_NED(dest_NED);
            } else {
                // peek at the next point.  this can fail if the IO task currently has the path semaphore
                // 查看下一个点。如果 IO 任务当前持有路径信号量，这可能会失败
                Vector3f next_dest_NED;
                if (g2.smart_rtl.peek_point(next_dest_NED)) {
                    wp_nav->set_wp_destination_NED(dest_NED);
                    if (g2.smart_rtl.get_num_points() == 1) {
                        // this is the very last point, add 2m to the target alt
                        // 这是最后一个点，将目标高度增加 2m
                        next_dest_NED.z -= 2.0f;
                    }
                    wp_nav->set_wp_destination_next_NED(next_dest_NED);
                } else {
                    // this can only happen if peek failed to take the semaphore
                    // send next point anyway which will cause the vehicle to slow at the next point
                    // 这只有在 peek 未能获取信号量时才会发生
                    // 无论如何都发送下一个点，这将导致飞行器在下一个点减速
                    wp_nav->set_wp_destination_NED(dest_NED);
                    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                }
            }
        } else if (g2.smart_rtl.get_num_points() == 0) {
            // We should never get here; should always have at least
            // two points and the "zero points left" is handled above.
            // 我们不应该到达这里；应该至少有两个点，而且上面已经处理了"剩余零个点"的情况
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            smart_rtl_state = SubMode::PRELAND_POSITION;
        } else if (path_follow_last_pop_fail_ms == 0) {
            // first time we've failed to pop off (ever, or after a success)
            // 第一次未能弹出（从未发生过，或在成功之后）
            path_follow_last_pop_fail_ms = AP_HAL::millis();
        } else if (AP_HAL::millis() - path_follow_last_pop_fail_ms > 10000) {
            // we failed to pop a point off for 10 seconds.  This is
            // almost certainly a bug.
            // 我们在 10 秒内未能弹出一个点。这几乎肯定是一个 bug。
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            smart_rtl_state = SubMode::PRELAND_POSITION;
        }
    }

    // update controllers
    // 更新控制器
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    wp_nav->update_wpnav();
    pos_control->update_z_controller();

    // call attitude controller with auto yaw
    // 调用带有自动偏航的姿态控制器
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

void ModeSmartRTL::pre_land_position_run()
{
    // if we are close to 2m above start point, we are ready to land.
    // 如果我们接近起始点上方 2m，我们就准备好着陆了
    if (wp_nav->reached_wp_destination()) {
        // choose descend and hold, or land based on user parameter rtl_alt_final
        // 根据用户参数 rtl_alt_final 选择下降并保持，或着陆
        if (g.rtl_alt_final <= 0 || copter.failsafe.radio) {
            land_start();
            smart_rtl_state = SubMode::LAND;
        } else {
            set_descent_target_alt(copter.g.rtl_alt_final);
            descent_start();
            smart_rtl_state = SubMode::DESCEND;
        }
    }

    // update controllers
    // 更新控制器
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    wp_nav->update_wpnav();
    pos_control->update_z_controller();
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

// save current position for use by the smart_rtl flight mode
// 保存当前位置以供 smart_rtl 飞行模式使用
void ModeSmartRTL::save_position()
{
    const bool should_save_position = motors->armed() && (copter.flightmode->mode_number() != Mode::Number::SMART_RTL);

    copter.g2.smart_rtl.update(copter.position_ok(), should_save_position);
}

bool ModeSmartRTL::get_wp(Location& destination) const
{
    // provide target in states which use wp_nav
    // 在使用 wp_nav 的状态下提供目标
    switch (smart_rtl_state) {
    case SubMode::WAIT_FOR_PATH_CLEANUP:
    case SubMode::PATH_FOLLOW:
    case SubMode::PRELAND_POSITION:
    case SubMode::DESCEND:
        return wp_nav->get_wp_destination_loc(destination);
    case SubMode::LAND:
        return false;
    }

    // we should never get here but just in case
    // 我们不应该到达这里，但以防万一
    return false;
}

uint32_t ModeSmartRTL::wp_distance() const
{
    return wp_nav->get_wp_distance_to_destination();
}

int32_t ModeSmartRTL::wp_bearing() const
{
    return wp_nav->get_wp_bearing_to_destination();
}

bool ModeSmartRTL::use_pilot_yaw() const
{
    const bool land_repositioning = g.land_repositioning && (smart_rtl_state == SubMode::DESCEND);
    const bool final_landing = smart_rtl_state == SubMode::LAND;
    return g2.smart_rtl.use_pilot_yaw() || land_repositioning || final_landing;
}

#endif
