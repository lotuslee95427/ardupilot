/*
 *  处理当前任务命令和家位置的逻辑
 */

#include "Plane.h"

/*
 *  set_next_WP - 设置飞行器应该飞往的目标位置
 */
void Plane::set_next_WP(const Location &loc)
{
    if (auto_state.next_wp_crosstrack) {
        // 将当前航点复制到OldWP槽
        prev_WP_loc = next_WP_loc;
        auto_state.crosstrack = true;
    } else {
        // 对于这个航点我们不应该尝试横向跟踪
        prev_WP_loc = current_loc;
        // 为下一个航点使用横向跟踪
        auto_state.next_wp_crosstrack = true;
        auto_state.crosstrack = false;
    }

    // 加载next_WP槽
    // ---------------------
    next_WP_loc = loc;

    // 如果纬度和经度为零，则使用当前纬度/经度
    // 这允许任务包含一个"原地盘旋"命令
    if (next_WP_loc.lat == 0 && next_WP_loc.lng == 0) {
        next_WP_loc.lat = current_loc.lat;
        next_WP_loc.lng = current_loc.lng;
        // 另外，将零高度视为当前高度
        if (next_WP_loc.alt == 0) {
            next_WP_loc.alt = current_loc.alt;
            next_WP_loc.relative_alt = false;
            next_WP_loc.terrain_alt = false;
        }
    }

    // 将相对高度转换为绝对高度
    if (next_WP_loc.relative_alt) {
        next_WP_loc.relative_alt = false;
        next_WP_loc.alt += home.alt;
    }

    // 我们是否已经越过了航点？这种情况发生在我们跳过航点时，
    // 可能导致我们跳过一个航点。如果我们在开始一段航线时已经
    // 越过了航点，那么使用当前位置作为前一个航点，以防立即
    // 认为航点已完成
    if (current_loc.past_interval_finish_line(prev_WP_loc, next_WP_loc)) {
        prev_WP_loc = current_loc;
    }

    // 将我们的盘旋值清零以监视错过的航点
    loiter_angle_reset();

    setup_glide_slope();
    setup_turn_angle();

    // 立即更新plane.target_altitude，否则如果我们太接近盘旋点，
    // 我们可能会在更新之前就认为我们处于正确的高度
    // （这是基于调度器表的顺序，我们在adjust_altitude_target()之前
    // 执行navigate()，而navigate()使用在adjust_altitude_target()
    // 中更新的值）
    adjust_altitude_target();
}

void Plane::set_guided_WP(const Location &loc)
{
    // 设置盘旋方向
    if (aparm.loiter_radius < 0 || loc.loiter_ccw) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }

    // 将当前位置复制到OldWP槽
    // ---------------------------------------
    prev_WP_loc = current_loc;

    // 加载next_WP槽
    // ---------------------
    next_WP_loc = loc;

    // 用于控制FBW和限制爬升率
    // -----------------------------------------------
    set_target_altitude_current();

    setup_glide_slope();
    setup_turn_angle();

    // 禁用横向跟踪，直接飞向目标点
    auto_state.crosstrack = false;

    // 重置盘旋开始时间
    loiter.start_time_ms = 0;

    // 以非VTOL模式开始
    auto_state.vtol_loiter = false;
    
    loiter_angle_reset();

#if HAL_QUADPLANE_ENABLED
    // 取消待定的起飞
    quadplane.guided_takeoff = false;
#endif
}

/*
  从GPS更新家位置
  只要我们有3D锁定且解锁开关未按下，就会调用此函数

  如果家位置改变则返回true
*/
bool Plane::update_home()
{
    if (hal.util->was_watchdog_armed()) {
        return false;
    }
    if ((g2.home_reset_threshold == -1) ||
        ((g2.home_reset_threshold > 0) &&
         (fabsf(barometer.get_altitude()) > g2.home_reset_threshold))) {
        // 如果气压高度变化显著，则不自动更新
        // 这允许我们应对缓慢的气压漂移，但如果我们显著改变了高度，
        // 则不重新设置家和气压计
        return false;
    }
    bool ret = false;
    if (ahrs.home_is_set() && !ahrs.home_is_locked() && gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        Location loc;
        if (ahrs.get_location(loc)) {
            // 我们直接从GPS获取高度，因为我们即将重置气压计校准
            // 我们不能使用AHRS高度，否则可能会永久保持高度偏差，
            // 因为AHRS高度依赖于家高度，这意味着我们会有一个循环依赖
            loc.alt = gps.location().alt;
            ret = AP::ahrs().set_home(loc);
        }
    }

    // 即使家未更新，我们也进行气压计重置以停止解除武装时的气压漂移错误
    barometer.update_calibration();
    ahrs.resetHeightDatum();

    update_current_loc();

    return ret;
}

bool Plane::set_home_persistently(const Location &loc)
{
    if (hal.util->was_watchdog_armed()) {
        return false;
    }
    if (!AP::ahrs().set_home(loc)) {
        return false;
    }

    // 将家位置保存到EEPROM
    mission.write_home_to_storage();

    return true;
}
