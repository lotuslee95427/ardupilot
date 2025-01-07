#include "mode.h"
#include "Plane.h"
#include <GCS_MAVLink/GCS.h>

/*
  起飞模式参数
 */
const AP_Param::GroupInfo ModeTakeoff::var_info[] = {
    // @Param: ALT
    // @DisplayName: 起飞模式高度
    // @Description: 这是TAKEOFF模式的目标高度
    // @Range: 0 200
    // @Increment: 1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("ALT", 1, ModeTakeoff, target_alt, 50),

    // @Param: LVL_ALT
    // @DisplayName: 起飞模式平飞高度
    // @Description: 这是TAKEOFF和AUTO模式下保持机翼水平的高度。低于此高度时，滚转需求被限制在LEVEL_ROLL_LIMIT内。在TKOFF_LVL_ALT*3或TKOFF_ALT（取较低值）以上恢复正常飞行滚转限制。在TKOFF_LVL_ALT和这些高度之间，滚转限制会进行平滑过渡。
    // @Range: 0 50
    // @Increment: 1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("LVL_ALT", 2, ModeTakeoff, level_alt, 10),

    // @Param: LVL_PITCH
    // @DisplayName: 起飞模式初始俯仰角
    // @Description: 这是起飞过程中的目标俯仰角。
    // @Range: 0 30
    // @Increment: 1
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("LVL_PITCH", 3, ModeTakeoff, level_pitch, 15),

    // @Param: DIST
    // @DisplayName: 起飞模式距离
    // @Description: 这是飞机将盘旋的距离起飞位置的距离。盘旋点将位于起飞方向上（飞机开始起飞时面对的方向）
    // @Range: 0 500
    // @Increment: 1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("DIST", 4, ModeTakeoff, target_dist, 200),

    // @Param: GND_PITCH
    // @DisplayName: 起飞滑跑俯仰角需求
    // @Description: 在速度达到TKOFF_ROTATE_SPD之前，起飞滑跑期间要求的俯仰角度。对于尾轮式飞机，设置为三点着陆的地面俯仰角，并使用TKOFF_TDRAG_ELEV防止机头倾覆。对于前轮转向飞机，设置为地面俯仰角，如果需要随着速度上升减少前轮负载，可以使用TKOFF_GND_PITCH的正偏移，最多可达地面角度以上5度，从测量的俯仰角开始，每次增加1度，同时检查每次增加是否会导致过早旋转和起飞。要增加前轮负载，使用负的TKOFF_TDRAG_ELEV，并在进行调整之前参考TKOFF_TDRAG_ELEV的注释。
    // @Units: deg
    // @Range: -5.0 10.0
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("GND_PITCH", 5, ModeTakeoff, ground_pitch, 5),

    AP_GROUPEND
};

ModeTakeoff::ModeTakeoff() :
    Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeTakeoff::_enter()
{
    takeoff_mode_setup = false;
    have_autoenabled_fences = false;

    return true;
}

void ModeTakeoff::update()
{
    // 如果我们没有有效的位置和家的位置，就不设置航点！
    if (!(plane.current_loc.initialised() && AP::ahrs().home_is_set())) {
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        return;
    }

    const float alt = target_alt;
    const float dist = target_dist;
    if (!takeoff_mode_setup) {
        const uint16_t altitude = plane.relative_ground_altitude(false,true);
        const float direction = degrees(ahrs.get_yaw());
        // 检查是否已经在飞行中，如果是，则跳过起飞
        if (plane.is_flying() && (millis() - plane.started_flying_ms > 10000U) && ahrs.groundspeed() > 3) {
            if (altitude >= alt) {
                gcs().send_text(MAV_SEVERITY_INFO, "高于起飞高度 - 正在盘旋");
                plane.next_WP_loc = plane.current_loc;
                takeoff_mode_setup = true;
                plane.set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "正在爬升到起飞高度然后盘旋");
                plane.next_WP_loc = plane.current_loc;
                plane.next_WP_loc.alt += ((alt - altitude) *100);
                plane.next_WP_loc.offset_bearing(direction, dist);
                takeoff_mode_setup = true;
                plane.set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
            }
        // 如果不是在飞行中，则执行完整的起飞序列
        } else {
            // 设置目标航点和高度，以便在起飞位置的指定距离和高度处盘旋
            start_loc = plane.current_loc;
            plane.prev_WP_loc = plane.current_loc;
            plane.next_WP_loc = plane.current_loc;
            plane.next_WP_loc.alt += alt*100.0;
            plane.next_WP_loc.offset_bearing(direction, dist);

            plane.crash_state.is_crashed = false;

            plane.auto_state.takeoff_pitch_cd = level_pitch * 100;

            plane.set_flight_stage(AP_FixedWing::FlightStage::TAKEOFF);

            if (!plane.throttle_suppressed) {
                gcs().send_text(MAV_SEVERITY_INFO, "起飞到 %.0fm 高度，距离 %.1fm，航向 %.1f 度",
                                alt, dist, direction);
                plane.takeoff_state.start_time_ms = millis();
                takeoff_mode_setup = true;
            }
        }
    }
    // 检查可选的起飞超时
    if (plane.check_takeoff_timeout()) {
        plane.set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
        takeoff_mode_setup = false;
    }
        
    // 如果我们爬升超过TKOFF_LVL_ALT或者通过目标位置，我们就完成初始水平起飞
    // 检查目标位置是为了防止在无法爬升的情况下无限飞行
    // 重置盘旋航点目标，使其与起始位置的正确方位和距离一致
    // 这是为了防止由于EKF重置或最大油门时的罗盘干扰导致的原始偏航角偏差
    const float altitude_cm = plane.current_loc.alt - start_loc.alt;
    if (plane.flight_stage == AP_FixedWing::FlightStage::TAKEOFF &&
        (altitude_cm >= level_alt*100 ||
         start_loc.get_distance(plane.current_loc) >= dist)) {
        // 使用当前偏航角重置目标盘旋航点，这应该接近正确的起始航向
        const float direction = start_loc.get_bearing_to(plane.current_loc) * 0.01;
        plane.next_WP_loc = start_loc;
        plane.next_WP_loc.offset_bearing(direction, dist);
        plane.next_WP_loc.alt += alt*100.0;
        plane.set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
    }

    if (plane.flight_stage == AP_FixedWing::FlightStage::TAKEOFF) {
        // 低于TAKOFF_LVL_ALT
        plane.takeoff_calc_roll();
        plane.takeoff_calc_pitch();
        plane.takeoff_calc_throttle(true);
    } else {
        if ((altitude_cm >= alt * 100 - 200)) { // 在TKOFF_ALT的2m范围内，或者在上方并盘旋
#if AP_FENCE_ENABLED
            if (!have_autoenabled_fences) {
                plane.fence.auto_enable_fence_after_takeoff();
                have_autoenabled_fences = true;
            }
#endif
            plane.calc_nav_roll();
            plane.calc_nav_pitch();
            plane.calc_throttle();
        } else { // 仍在爬升到TAKEOFF_ALT；可能正在盘旋
            plane.takeoff_calc_throttle();
            plane.takeoff_calc_roll();
            plane.takeoff_calc_pitch();
        }
        
        // 检查是否由于处于初始TAKEOFF阶段而处于长时间故障保护状态；如果是，现在通过事件调用重新调用长时间故障保护以获取故障保护操作
        if (plane.long_failsafe_pending) {
            plane.long_failsafe_pending = false;
            plane.failsafe_long_on_event(FAILSAFE_LONG, ModeReason::MODE_TAKEOFF_FAILSAFE);
        }
    }
}

void ModeTakeoff::navigate()
{
    // 零表示使用WP_LOITER_RAD
    plane.update_loiter(0);
}
