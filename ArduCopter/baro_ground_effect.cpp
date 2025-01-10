#include "Copter.h"

// 更新地面效应检测器
void Copter::update_ground_effect_detector(void)
{
    // 如果地面效应补偿未启用或电机未解锁,则禁用地面效应并返回
    if(!g2.gndeffect_comp_enabled || !motors->armed()) {
        // 禁用起飞和着陆预期状态
        gndeffect_state.takeoff_expected = false;
        gndeffect_state.touchdown_expected = false;
        ahrs.set_takeoff_expected(gndeffect_state.takeoff_expected);
        ahrs.set_touchdown_expected(gndeffect_state.touchdown_expected);
        return;
    }

    // 变量初始化
    uint32_t tnow_ms = millis();                                      // 当前时间(毫秒)
    float xy_des_speed_cms = 0.0f;                                    // 期望水平速度(厘米/秒)
    float xy_speed_cms = 0.0f;                                        // 实际水平速度(厘米/秒)
    float des_climb_rate_cms = pos_control->get_vel_desired_cms().z;  // 期望爬升速率(厘米/秒)

    // 如果位置控制器处于激活状态,获取期望水平速度
    if (pos_control->is_active_xy()) {
        Vector3f vel_target = pos_control->get_vel_target_cms();
        vel_target.z = 0.0f;
        xy_des_speed_cms = vel_target.length();
    }

    // 如果位置有效或EKF有相对位置,获取实际水平速度
    if (position_ok() || ekf_has_relative_position()) {
        Vector3f vel = inertial_nav.get_velocity_neu_cms();
        vel.z = 0.0f;
        xy_speed_cms = vel.length();
    }

    // 起飞逻辑处理

    // 投掷模式下不使用起飞预期
    if (flightmode->mode_number() == Mode::Number::THROW) {
        gndeffect_state.takeoff_expected = false;
    } else if (motors->armed() && ap.land_complete) {
        // 如果已解锁且在地面,则预期即将起飞
        gndeffect_state.takeoff_expected = true;
    }

    // 如果还未起飞,重置起飞计时器、高度和完成标志
    const bool throttle_up = flightmode->has_manual_throttle() && channel_throttle->get_control_in() > 0;
    if (!throttle_up && ap.land_complete) {
        gndeffect_state.takeoff_time_ms = tnow_ms;
        gndeffect_state.takeoff_alt_cm = inertial_nav.get_position_z_up_cm();
    }

    // 如果处于起飞预期状态且满足起飞条件(超过5秒或高度增加超过50厘米),结束起飞预期状态
    if (gndeffect_state.takeoff_expected && (tnow_ms-gndeffect_state.takeoff_time_ms > 5000 || inertial_nav.get_position_z_up_cm()-gndeffect_state.takeoff_alt_cm > 50.0f)) {
        gndeffect_state.takeoff_expected = false;
    }

    // 着陆逻辑处理
    Vector3f angle_target_rad = attitude_control->get_att_target_euler_cd() * radians(0.01f);  // 目标姿态角(弧度)
    bool small_angle_request = cosf(angle_target_rad.x)*cosf(angle_target_rad.y) > cosf(radians(7.5f));  // 小角度请求判断
    bool xy_speed_low = (position_ok() || ekf_has_relative_position()) && xy_speed_cms <= 125.0f;         // 水平速度低
    bool xy_speed_demand_low = pos_control->is_active_xy() && xy_des_speed_cms <= 125.0f;                // 期望水平速度低
    bool slow_horizontal = xy_speed_demand_low || (xy_speed_low && !pos_control->is_active_xy()) || (flightmode->mode_number() == Mode::Number::ALT_HOLD && small_angle_request);  // 水平运动缓慢

    bool descent_demanded = pos_control->is_active_z() && des_climb_rate_cms < 0.0f;           // 要求下降
    bool slow_descent_demanded = descent_demanded && des_climb_rate_cms >= -100.0f;            // 要求缓慢下降
    bool z_speed_low = fabsf(inertial_nav.get_velocity_z_up_cms()) <= 60.0f;                  // 垂直速度低
    bool slow_descent = (slow_descent_demanded || (z_speed_low && descent_demanded));          // 缓慢下降状态

    // 当水平运动和下降都缓慢时,预期着陆
    gndeffect_state.touchdown_expected = slow_horizontal && slow_descent;

    // 如果预期起飞或着陆,为EKF准备地面效应
    ahrs.set_takeoff_expected(gndeffect_state.takeoff_expected);
    ahrs.set_touchdown_expected(gndeffect_state.touchdown_expected);
}

// update ekf terrain height stable setting
// when set to true, this allows the EKF to stabilize the normally barometer based altitude using a rangefinder
// this is not related to terrain following
void Copter::update_ekf_terrain_height_stable()
{
    // set to false if no position estimate
    if (!position_ok() && !ekf_has_relative_position()) {
        ahrs.set_terrain_hgt_stable(false);
        return;
    }

    // consider terrain height stable if vehicle is taking off or landing
    ahrs.set_terrain_hgt_stable(flightmode->is_taking_off() || flightmode->is_landing());
}
