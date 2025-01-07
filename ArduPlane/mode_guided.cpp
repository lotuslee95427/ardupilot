#include "mode.h"
#include "Plane.h"

// 进入引导模式
bool ModeGuided::_enter()
{
    plane.guided_throttle_passthru = false;
    // 进入引导模式时，将当前位置设置为目标位置
    // 这与四轴飞行器代码的行为相匹配
    Location loc{plane.current_loc};

#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.guided_mode_enabled()) {
        // 如果使用Q_GUIDED_MODE，则根据停止距离向前投影
        loc.offset_bearing(degrees(ahrs.groundspeed_vector().angle()),
                           plane.quadplane.stopping_distance());
    }
#endif

    // 在模式改变时将引导半径设置为WP_LOITER_RAD
    active_radius_m = 0;

    // 设置引导航点
    plane.set_guided_WP(loc);
    return true;
}

// 更新引导模式
void ModeGuided::update()
{
#if HAL_QUADPLANE_ENABLED
    if (plane.auto_state.vtol_loiter && plane.quadplane.available()) {
        // 如果在VTOL悬停状态，更新四轴飞行器的引导模式
        plane.quadplane.guided_update();
        return;
    }
#endif

    // 检查是否在过去3秒内收到了外部消息来引导滚转
    if (plane.guided_state.last_forced_rpy_ms.x > 0 &&
            millis() - plane.guided_state.last_forced_rpy_ms.x < 3000) {
        // 限制滚转角度
        plane.nav_roll_cd = constrain_int32(plane.guided_state.forced_rpy_cd.x, -plane.roll_limit_cd, plane.roll_limit_cd);
        plane.update_load_factor();

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // 如果在引导模式下且有目标航向
    } else if ((plane.control_mode == &plane.mode_guided) && (plane.guided_state.target_heading_type != GUIDED_HEADING_NONE) ) {
        uint32_t tnow = AP_HAL::millis();
        float delta = (tnow - plane.guided_state.target_heading_time_ms) * 1e-3f;
        plane.guided_state.target_heading_time_ms = tnow;

        float error = 0.0f;
        if (plane.guided_state.target_heading_type == GUIDED_HEADING_HEADING) {
            // 计算航向误差
            error = wrap_PI(plane.guided_state.target_heading - AP::ahrs().get_yaw());
        } else {
            // 计算地速向量误差
            Vector2f groundspeed = AP::ahrs().groundspeed_vector();
            error = wrap_PI(plane.guided_state.target_heading - atan2f(-groundspeed.y, -groundspeed.x) + M_PI);
        }

        // 计算倾斜限制
        float bank_limit = degrees(atanf(plane.guided_state.target_heading_accel_limit/GRAVITY_MSS)) * 1e2f;
        bank_limit = MIN(bank_limit, plane.roll_limit_cd);

        // 使用PID控制器更新误差
        const float desired = plane.g2.guidedHeading.update_error(error, delta, plane.guided_state.target_heading_limit);

        // 检查输出饱和
        plane.guided_state.target_heading_limit = fabsf(desired) >= bank_limit;

        // 限制滚转角度
        plane.nav_roll_cd = constrain_int32(desired, -bank_limit, bank_limit);
        plane.update_load_factor();

#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    } else {
        // 计算导航滚转角度
        plane.calc_nav_roll();
    }

    // 检查是否在过去3秒内收到了外部消息来引导俯仰
    if (plane.guided_state.last_forced_rpy_ms.y > 0 &&
            millis() - plane.guided_state.last_forced_rpy_ms.y < 3000) {
        // 限制俯仰角度
        plane.nav_pitch_cd = constrain_int32(plane.guided_state.forced_rpy_cd.y, plane.pitch_limit_min*100, plane.aparm.pitch_limit_max.get()*100);
    } else {
        // 计算导航俯仰角度
        plane.calc_nav_pitch();
    }

    // 油门输出
    if (plane.guided_throttle_passthru) {
        // 在围栏突破时手动传递油门
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.get_throttle_input(true));

    }  else if (plane.aparm.throttle_cruise > 1 &&
            plane.guided_state.last_forced_throttle_ms > 0 &&
            millis() - plane.guided_state.last_forced_throttle_ms < 3000) {
        // 如果在过去3秒内收到了外部消息来引导油门
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.guided_state.forced_throttle);

    } else {
        // TECS控制
        plane.calc_throttle();
    }
}

// 导航更新
void ModeGuided::navigate()
{
    plane.update_loiter(active_radius_m);
}

// 处理引导请求
bool ModeGuided::handle_guided_request(Location target_loc)
{
    // 如果需要，添加家的高度
    if (target_loc.relative_alt) {
        target_loc.alt += plane.home.alt;
        target_loc.relative_alt = 0;
    }

    // 设置引导航点
    plane.set_guided_WP(target_loc);

    return true;
}

// 设置半径和方向
void ModeGuided::set_radius_and_direction(const float radius, const bool direction_is_ccw)
{
    // 将半径限制在update_loiter()的(uint16_t)范围内
    active_radius_m = constrain_int32(fabsf(radius), 0, UINT16_MAX);
    plane.loiter.direction = direction_is_ccw ? -1 : 1;
}

// 更新目标高度
void ModeGuided::update_target_altitude()
{
#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // 目标高度可以为负（例如，从山顶飞到家的高度以下）
    if (((plane.guided_state.target_alt_time_ms != 0) || plane.guided_state.target_location.alt != -1 )) {
        // 板外高度要求
        uint32_t now = AP_HAL::millis();
        float delta = 1e-3f * (now - plane.guided_state.target_alt_time_ms);
        plane.guided_state.target_alt_time_ms = now;
        // 精确计算delta为浮点数
        float delta_amt_f = delta * plane.guided_state.target_alt_rate;
        // 然后缩放x100以匹配last_target_alt，并转换为有符号int32_t，因为它可能是负的
        int32_t delta_amt_i = (int32_t)(100.0 * delta_amt_f); 
        // 为了计算所需的速度（上升或下降），我们需要目标框架中的目标和当前高度
        const Location::AltFrame target_frame = plane.guided_state.target_location.get_alt_frame();
        int32_t target_alt_previous_cm;
        if (plane.current_loc.initialised() && plane.guided_state.target_location.initialised() && 
            plane.current_loc.get_alt_cm(target_frame, target_alt_previous_cm)) {
            // 创建一个新的临时目标位置，该位置从current_location开始，并在正确的方向上移动delta_amt_i
            int32_t temp_alt_cm = constrain_int32(plane.guided_state.target_location.alt, target_alt_previous_cm - delta_amt_i,  target_alt_previous_cm + delta_amt_i);
            Location temp_location = plane.guided_state.target_location;
            temp_location.set_alt_cm(temp_alt_cm, target_frame);

            // 逐步将高度调整到目标高度            
            plane.set_target_altitude_location(temp_location);
        }
    } else 
#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
        {
        Mode::update_target_altitude();
    }
}
