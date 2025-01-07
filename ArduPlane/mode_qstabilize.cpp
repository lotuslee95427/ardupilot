#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

// 进入QStabilize模式
bool ModeQStabilize::_enter()
{
    quadplane.throttle_wait = false;
    return true;
}

// 更新QStabilize模式
void ModeQStabilize::update()
{
    // 使用摇杆设置nav_roll和nav_pitch
    // 注意：QuadPlane::tailsitter_check_input（从Plane::read_radio调用）
    // 可能会改变roll和yaw的control_in值，但不会改变相应的
    // radio_in值。这意味着对于尾座式飞机，norm_input的结果不一定
    // 正确，所以必须使用get_control_in()。
    // 将control_input归一化到[-1,1]
    const float roll_input = (float)plane.channel_roll->get_control_in() / plane.channel_roll->get_range();
    const float pitch_input = (float)plane.channel_pitch->get_control_in() / plane.channel_pitch->get_range();

    // 然后缩放到目标角度（以厘度为单位）
    if (plane.quadplane.tailsitter.active()) {
        // 尾座式飞机处理不同
        set_tailsitter_roll_pitch(roll_input, pitch_input);
        return;
    }

    if (!plane.quadplane.option_is_set(QuadPlane::OPTION::INGORE_FW_ANGLE_LIMITS_IN_Q_MODES)) {
        // 默认情况下，角度也受前向飞行限制约束
        set_limited_roll_pitch(roll_input, pitch_input);
    } else {
        // 对滚转和俯仰都使用最大角度
        plane.nav_roll_cd = roll_input * plane.quadplane.aparm.angle_max;
        plane.nav_pitch_cd = pitch_input * plane.quadplane.aparm.angle_max;
    }
}

// 四旋翼稳定模式
void ModeQStabilize::run()
{
    const uint32_t now = AP_HAL::millis();
    if (quadplane.tailsitter.in_vtol_transition(now)) {
        // 在VTOL过渡的前向飞行拉起阶段的尾座式飞机运行前向飞行控制器
        Mode::run();
        return;
    }

    plane.quadplane.assign_tilt_to_fwd_thr();

    // QSTABILIZE模式下的ESC校准特殊检查
    if (quadplane.esc_calibration != 0) {
        quadplane.run_esc_calibration();
        plane.stabilize_roll();
        plane.stabilize_pitch();
        return;
    }

    // 正常QSTABILIZE模式
    float pilot_throttle_scaled = quadplane.get_pilot_throttle();
    quadplane.hold_stabilize(pilot_throttle_scaled);

    // 使用固定翼面进行稳定
    plane.stabilize_roll();
    plane.stabilize_pitch();
}

// 为尾座式飞机设置所需的滚转和俯仰
void ModeQStabilize::set_tailsitter_roll_pitch(const float roll_input, const float pitch_input)
{
    // 如果设置了滚转的单独限制
    if (plane.quadplane.tailsitter.max_roll_angle > 0) {
        // 滚转参数以度为单位，而不是厘度
        plane.nav_roll_cd = plane.quadplane.tailsitter.max_roll_angle * 100.0f * roll_input;
    } else {
        plane.nav_roll_cd = roll_input * plane.quadplane.aparm.angle_max;
    }

    // 尾座式飞机俯仰的最大角度
    plane.nav_pitch_cd = pitch_input * plane.quadplane.aparm.angle_max;

    plane.quadplane.transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd);
}

// 为普通四旋翼设置所需的滚转和俯仰，同时受前向飞行限制约束
void ModeQStabilize::set_limited_roll_pitch(const float roll_input, const float pitch_input)
{
    plane.nav_roll_cd = roll_input * MIN(plane.roll_limit_cd, plane.quadplane.aparm.angle_max);
    // 俯仰进一步受PTCH_LIM_MIN/MAX约束，可能施加
    // 更严格的（可能不对称的）限制，而不是Q_ANGLE_MAX
    if (pitch_input > 0) {
        plane.nav_pitch_cd = pitch_input * MIN(plane.aparm.pitch_limit_max*100, plane.quadplane.aparm.angle_max);
    } else {
        plane.nav_pitch_cd = pitch_input * MIN(-plane.pitch_limit_min*100, plane.quadplane.aparm.angle_max);
    }
}

#endif
