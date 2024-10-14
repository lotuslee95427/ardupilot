#include "Copter.h"
#include <AP_Mount/AP_Mount.h>

#if MODE_CIRCLE_ENABLED

/*
 * Init and run calls for circle flight mode
 * 初始化和运行圆周飞行模式的调用
 */

// circle_init - initialise circle controller flight mode
// circle_init - 初始化圆周控制器飞行模式
bool ModeCircle::init(bool ignore_checks)
{
    speed_changing = false; // 初始化速度变化标志为假

    // set speed and acceleration limits
    // 设置速度和加速度限制
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise circle controller including setting the circle center based on vehicle speed
    // 初始化圆周控制器，包括根据飞行器速度设置圆心
    copter.circle_nav->init();

#if HAL_MOUNT_ENABLED
    // Check if the CIRCLE_OPTIONS parameter have roi_at_center
    // 检查CIRCLE_OPTIONS参数是否有roi_at_center
    if (copter.circle_nav->roi_at_center()) {
        const Vector3p &pos { copter.circle_nav->get_center() }; // 获取圆心位置
        Location circle_center;
        if (!AP::ahrs().get_location_from_origin_offset_NED(circle_center, pos * 0.01)) {
            return false; // 如果获取位置失败，返回假
        }
        // point at the ground:
        // 指向地面
        circle_center.set_alt_cm(0, Location::AltFrame::ABOVE_TERRAIN);
        AP_Mount *s = AP_Mount::get_singleton();
        s->set_roi_target(circle_center); // 设置ROI目标为圆心
    }
#endif

    // set auto yaw circle mode
    // 设置自动偏航圆周模式
    auto_yaw.set_mode(AutoYaw::Mode::CIRCLE);

    return true; // 返回真，表示初始化成功
}

// circle_run - runs the circle flight mode
// circle_run - 运行圆周飞行模式
// should be called at 100hz or more
// 应该以100Hz或更高频率调用
void ModeCircle::run()
{
    // set speed and acceleration limits
    // 设置速度和加速度限制
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // Check for any change in params and update in real time
    // 检查参数是否有变化并实时更新
    copter.circle_nav->check_param_change();

    // pilot changes to circle rate and radius
    // 飞行员改变圆周速率和半径
    // skip if in radio failsafe
    // 如果无线电失效则跳过
    if (!copter.failsafe.radio && copter.circle_nav->pilot_control_enabled()) {
        // update the circle controller's radius target based on pilot pitch stick inputs
        // 根据飞行员俯仰杆输入更新圆周控制器的半径目标
        const float radius_current = copter.circle_nav->get_radius();           // circle controller's radius target, which begins as the circle_radius parameter
        // 圆周控制器的半径目标，初始值为circle_radius参数
        const float pitch_stick = channel_pitch->norm_input_dz();               // pitch stick normalized -1 to 1
        // 俯仰杆归一化为-1到1
        const float nav_speed = copter.wp_nav->get_default_speed_xy();          // copter WP_NAV parameter speed
        // 无人机WP_NAV参数速度
        const float radius_pilot_change = (pitch_stick * nav_speed) * G_Dt;     // rate of change (pitch stick up reduces the radius, as in moving forward)
        // 变化率（俯仰杆上推减小半径，如向前移动）
        const float radius_new = MAX(radius_current + radius_pilot_change,0);   // new radius target
        // 新的半径目标

        if (!is_equal(radius_current, radius_new)) {
            copter.circle_nav->set_radius_cm(radius_new); // 如果当前半径与新半径不同，则设置新半径
        }

        // update the orbicular rate target based on pilot roll stick inputs
        // 根据飞行员横滚杆输入更新圆周速率目标
        // skip if using transmitter based tuning knob for circle rate
        // 如果使用基于发射机的调节旋钮来调节圆周速率则跳过
        if (g.radio_tuning != TUNING_CIRCLE_RATE) {
            const float roll_stick = channel_roll->norm_input_dz();         // roll stick normalized -1 to 1
            // 横滚杆归一化为-1到1

            if (is_zero(roll_stick)) {
                // no speed change, so reset speed changing flag
                // 没有速度变化，重置速度变化标志
                speed_changing = false;
            } else {
                const float rate = copter.circle_nav->get_rate();           // circle controller's rate target, which begins as the circle_rate parameter
                // 圆周控制器的速率目标，初始值为circle_rate参数
                const float rate_current = copter.circle_nav->get_rate_current(); // current adjusted rate target, which is probably different from _rate
                // 当前调整后的速率目标，可能与_rate不同
                const float rate_pilot_change = (roll_stick * G_Dt);        // rate of change from 0 to 1 degrees per second
                // 变化率从0到每秒1度
                float rate_new = rate_current;                              // new rate target
                // 新的速率目标
                if (is_positive(rate)) {
                    // currently moving clockwise, constrain 0 to 90
                    // 当前顺时针移动，限制在0到90度
                    rate_new = constrain_float(rate_current + rate_pilot_change, 0, 90);

                } else if (is_negative(rate)) {
                    // currently moving counterclockwise, constrain -90 to 0
                    // 当前逆时针移动，限制在-90到0度
                    rate_new = constrain_float(rate_current + rate_pilot_change, -90, 0);

                } else if (is_zero(rate) && !speed_changing) {
                    // Stopped, pilot has released the roll stick, and pilot now wants to begin moving with the roll stick
                    // 停止，飞行员已松开横滚杆，现在飞行员想要通过横滚杆开始移动
                    rate_new = rate_pilot_change;
                }

                speed_changing = true; // 设置速度变化标志为真
                copter.circle_nav->set_rate(rate_new); // 设置新的速率目标
            }
        }
    }

    // get pilot desired climb rate (or zero if in radio failsafe)
    // 获取飞行员期望的爬升率（如果无线电失效则为零）
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());

    // get avoidance adjusted climb rate
    // 获取避障调整后的爬升率
    target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

    // if not armed set throttle to zero and exit immediately
    // 如果未解锁则将油门设为零并立即退出
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling(); // 进行安全的地面处理
        return;
    }

    // set motors to full range
    // 将电机设置为全范围
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if AP_RANGEFINDER_ENABLED
    // update the vertical offset based on the surface measurement
    // 根据地表测量更新垂直偏移
    copter.surface_tracking.update_surface_offset();
#endif

    copter.failsafe_terrain_set_status(copter.circle_nav->update(target_climb_rate)); // 更新圆周导航状态
    pos_control->update_z_controller(); // 更新Z轴控制器

    // call attitude controller with auto yaw
    // 调用带自动偏航的姿态控制器
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

uint32_t ModeCircle::wp_distance() const
{
    return copter.circle_nav->get_distance_to_target(); // 获取到目标的距离
}

int32_t ModeCircle::wp_bearing() const
{
    return copter.circle_nav->get_bearing_to_target(); // 获取到目标的方位
}

#endif
