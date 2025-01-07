#include "mode.h"
#include "Plane.h"

// 进入巡航模式时的初始化函数
bool ModeCruise::_enter()
{
    locked_heading = false;  // 初始化航向锁定状态为false
    lock_timer_ms = 0;  // 初始化锁定计时器为0

#if HAL_SOARING_ENABLED
    // 如果启用了滑翔功能，初始化滑翔控制器
    plane.g2.soaring_controller.init_cruising();
#endif

    plane.set_target_altitude_current();  // 设置目标高度为当前高度

    return true;
}

// 巡航模式的更新函数，每帧调用
void ModeCruise::update()
{
    /*
      在巡航模式下，当航向被锁定时，我们使用导航代码来控制横滚
      任何副翼或方向舵输入都会解锁航向
    */
    if (plane.channel_roll->get_control_in() != 0 || plane.channel_rudder->get_control_in() != 0) {
        locked_heading = false;  // 如果有副翼或方向舵输入，解锁航向
        lock_timer_ms = 0;
    }

#if AP_SCRIPTING_ENABLED
    if (plane.nav_scripting_active()) {
        // 当脚本正在运行时，解锁航向并将高度偏移设为零
        locked_heading = false;
        lock_timer_ms = 0;
        plane.set_target_altitude_current();
    }
#endif
    
    if (!locked_heading) {
        // 如果航向未锁定，使用副翼输入控制横滚
        plane.nav_roll_cd = plane.channel_roll->norm_input() * plane.roll_limit_cd;
        plane.update_load_factor();
    } else {
        // 如果航向已锁定，使用导航系统计算横滚
        plane.calc_nav_roll();
    }
    plane.update_fbwb_speed_height();  // 更新飞行速度和高度
}

/*
  处理巡航模式，当我们有足够的地面速度且没有副翼或方向舵输入时，
  将航向锁定到GPS航向
 */
void ModeCruise::navigate()
{
#if AP_SCRIPTING_ENABLED
    if (plane.nav_scripting_active()) {
        // 如果脚本正在运行，不进行导航
        return;
    }
#endif

    // 检查飞机是否朝着机头方向移动
    const int32_t ground_course_cd = plane.gps.ground_course_cd();
    const bool moving_forwards = fabsf(wrap_PI(radians(ground_course_cd * 0.01) - plane.ahrs.get_yaw())) < M_PI_2;

    // 判断是否开始锁定航向的条件
    if (!locked_heading &&
        plane.channel_roll->get_control_in() == 0 &&
        plane.rudder_input() == 0 &&
        plane.gps.status() >= AP_GPS::GPS_OK_FIX_2D &&
        plane.gps.ground_speed() >= 3 &&
        moving_forwards &&
        lock_timer_ms == 0) {
        // 用户想要锁定航向 - 开始计时
        lock_timer_ms = millis();
    }
    if (lock_timer_ms != 0 &&
        (millis() - lock_timer_ms) > 500) {
        // 0.5秒后锁定航向
        locked_heading = true;
        lock_timer_ms = 0;
        locked_heading_cd = ground_course_cd;
        plane.prev_WP_loc = plane.current_loc;
    }
    if (locked_heading) {
        // 如果航向已锁定，设置下一个航点
        plane.next_WP_loc = plane.prev_WP_loc;
        // 始终看向1公里前方
        plane.next_WP_loc.offset_bearing(locked_heading_cd*0.01f, plane.prev_WP_loc.get_distance(plane.current_loc) + 1000);
        plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
    }
}

// 获取目标航向
bool ModeCruise::get_target_heading_cd(int32_t &target_heading) const
{
    target_heading = locked_heading_cd;
    return locked_heading;
}
