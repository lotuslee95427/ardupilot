#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

// LoiterAltQLand模式的进入函数
bool ModeLoiterAltQLand::_enter()
{
    // 如果之前的模式是VTOL模式或当前在VTOL模式，直接切换到QLand模式
    if (plane.previous_mode->is_vtol_mode() || plane.quadplane.in_vtol_mode()) {
        plane.set_mode(plane.mode_qland, ModeReason::LOITER_ALT_IN_VTOL);
        return true;
    }

    // 判断是否已经在盘旋中，如果是，使用当前的航点；否则，使用当前位置
    const bool already_in_a_loiter = plane.nav_controller->reached_loiter_target() && !plane.nav_controller->data_is_stale();
    const Location loiter_wp = already_in_a_loiter ? plane.next_WP_loc : plane.current_loc;

    // 调用父类的进入函数
    ModeLoiter::_enter();

    // 处理引导请求
    handle_guided_request(loiter_wp);

    // 尝试切换到QLand模式
    switch_qland();

    return true;
}

// LoiterAltQLand模式的导航函数
void ModeLoiterAltQLand::navigate()
{
    // 尝试切换到QLand模式
    switch_qland();

    // 调用父类的导航函数
    ModeLoiter::navigate();
}

// 尝试切换到QLand模式的函数
void ModeLoiterAltQLand::switch_qland()
{
    ftype dist;
    // 如果达到了目标高度并且到达了盘旋目标点，切换到QLand模式
    if ((!plane.current_loc.get_alt_distance(plane.next_WP_loc, dist) || is_negative(dist)) && plane.nav_controller->reached_loiter_target()) {
        plane.set_mode(plane.mode_qland, ModeReason::LOITER_ALT_REACHED_QLAND);
    }
}

// 处理引导请求的函数
bool ModeLoiterAltQLand::handle_guided_request(Location target_loc)
{
    // 设置目标高度
#if AP_TERRAIN_AVAILABLE
    if (plane.terrain_enabled_in_mode(Mode::Number::QLAND)) {
        // 如果地形功能可用且在QLand模式中启用，设置相对地形的高度
        target_loc.set_alt_cm(quadplane.qrtl_alt*100UL, Location::AltFrame::ABOVE_TERRAIN);
    } else {
        // 否则，设置相对家的高度
        target_loc.set_alt_cm(quadplane.qrtl_alt*100UL, Location::AltFrame::ABOVE_HOME);
    }
#else
    // 如果地形功能不可用，设置相对家的高度
    target_loc.set_alt_cm(quadplane.qrtl_alt*100UL, Location::AltFrame::ABOVE_HOME);
#endif

    // 设置引导航点
    plane.set_guided_WP(target_loc);

    return true;
}

#endif
