#include "mode.h"
#include "Plane.h"

// 进入Loiter模式时的初始化
bool ModeLoiter::_enter()
{
    // 在当前位置执行盘旋
    plane.do_loiter_at_location();
    // 设置地形目标高度
    plane.setup_terrain_target_alt(plane.next_WP_loc);

    // 如果启用了摇杆混合和Loiter高度控制
    if (plane.stick_mixing_enabled() && (plane.flight_option_enabled(FlightOptions::ENABLE_LOITER_ALT_CONTROL))) {
        // 设置当前高度为目标高度
        plane.set_target_altitude_current();
    }

    // 重置盘旋角度
    plane.loiter_angle_reset();

    return true;
}

// 更新Loiter模式
void ModeLoiter::update()
{
    // 计算导航滚转角
    plane.calc_nav_roll();
    if (plane.stick_mixing_enabled() && plane.flight_option_enabled(FlightOptions::ENABLE_LOITER_ALT_CONTROL)) {
        // 更新FBWB速度和高度
        plane.update_fbwb_speed_height();
    } else {
        // 计算导航俯仰角和油门
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }

#if AP_SCRIPTING_ENABLED
    if (plane.nav_scripting_active()) {
        // 如果正在执行脚本导航，重置目标高度
        plane.set_target_altitude_current();
        plane.next_WP_loc.set_alt_cm(plane.target_altitude.amsl_cm, Location::AltFrame::ABSOLUTE);
    }
#endif
}

// 检查航向是否对准目标位置
bool ModeLoiter::isHeadingLinedUp(const Location loiterCenterLoc, const Location targetLoc)
{
    // 计算高度修正后的盘旋半径
    const float loiter_radius = plane.nav_controller->loiter_radius(fabsf(plane.loiter.radius));
    if (!is_positive(loiter_radius)) {
        return true;
    }

    // 计算飞机在盘旋圆上的投影位置
    const Vector2f projected_pos = loiterCenterLoc.get_distance_NE(plane.current_loc).normalized() * loiter_radius;

    // 计算目标位置相对于盘旋中心的位置
    const Vector2f target_pos = loiterCenterLoc.get_distance_NE(targetLoc);

    // 计算目标距离盘旋中心的距离
    const float target_dist = target_pos.length();
    if (!is_positive(target_dist)) {
        return true;
    }

    int32_t target_bearing_cd;

    if (target_dist >= loiter_radius) {
        // 目标在盘旋圆外
        const Vector2f pos_to_target = target_pos - projected_pos;
        target_bearing_cd = degrees(pos_to_target.angle()) * 100;
    } else {
        // 目标在盘旋圆内
        const float a = loiter_radius - target_dist;
        const float segment_angle = 2.0 * asinf(a / (2.0 * loiter_radius));
        target_bearing_cd = degrees(wrap_PI(target_pos.angle() + (M_PI_2 - segment_angle) * plane.loiter.direction)) * 100;
    }

    // 计算当前理想航向
    const int32_t current_heading_cd = degrees(wrap_PI(projected_pos.angle() + M_PI_2 * plane.loiter.direction)) * 100;

    return isHeadingLinedUp_cd(target_bearing_cd, current_heading_cd);
}

// 检查当前航向是否对准给定方位
bool ModeLoiter::isHeadingLinedUp_cd(const int32_t bearing_cd) {
    // 获取当前航向
    const int32_t heading_cd = (wrap_360(degrees(ahrs.groundspeed_vector().angle())))*100;

    return isHeadingLinedUp_cd(bearing_cd, heading_cd);
}

// 检查给定航向是否对准给定方位
bool ModeLoiter::isHeadingLinedUp_cd(const int32_t bearing_cd, const int32_t heading_cd)
{
    // 计算航向误差
    const int32_t heading_err_cd = wrap_180_cd(bearing_cd - heading_cd);

    // 计算扩展接受范围
    const int32_t expanded_acceptance = 1000 * (labs(plane.loiter.sum_cd) / 36000);

    if (labs(heading_err_cd) <= 1000 + expanded_acceptance) {
        // 如果启用了xtrack，更新下一个航点位置
        if (plane.next_WP_loc.loiter_xtrack) {
            plane.next_WP_loc = plane.current_loc;
        }
        return true;
    }
    return false;
}

// 执行导航
void ModeLoiter::navigate()
{
    if (plane.flight_option_enabled(FlightOptions::ENABLE_LOITER_ALT_CONTROL)) {
        // 更新航点高度
        plane.next_WP_loc.set_alt_cm(plane.target_altitude.amsl_cm, Location::AltFrame::ABSOLUTE);
    }

#if AP_SCRIPTING_ENABLED
    if (plane.nav_scripting_active()) {
        // 如果正在执行脚本导航，不执行常规导航
        return;
    }
#endif

    // 更新盘旋
    plane.update_loiter(0);
}

// 更新目标高度
void ModeLoiter::update_target_altitude()
{
    if (plane.stick_mixing_enabled() && (plane.flight_option_enabled(FlightOptions::ENABLE_LOITER_ALT_CONTROL))) {
        return;
    }
    Mode::update_target_altitude();
}
