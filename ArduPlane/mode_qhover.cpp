#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

// 进入QHover模式时的初始化函数
bool ModeQHover::_enter()
{
    // 设置垂直速度和加速度限制
    pos_control->set_max_speed_accel_z(-quadplane.get_pilot_velocity_z_max_dn(), quadplane.pilot_speed_z_max_up*100, quadplane.pilot_accel_z*100);
    pos_control->set_correction_speed_accel_z(-quadplane.get_pilot_velocity_z_max_dn(), quadplane.pilot_speed_z_max_up*100, quadplane.pilot_accel_z*100);
    quadplane.set_climb_rate_cms(0);  // 设置初始爬升率为0

    quadplane.init_throttle_wait();  // 初始化油门等待
    return true;
}

// 更新QHover模式
void ModeQHover::update()
{
    plane.mode_qstabilize.update();  // 调用QStabilize模式的更新函数
}

/*
  控制QHOVER模式
 */
void ModeQHover::run()
{
    const uint32_t now = AP_HAL::millis();
    if (quadplane.tailsitter.in_vtol_transition(now)) {
        // 尾座式飞机在VTOL转换的前向飞行拉起阶段运行固定翼控制器
        Mode::run();
        return;
    }

    if (quadplane.throttle_wait) {
        // 如果在油门等待状态
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);  // 设置期望的电机状态为地面怠速
        attitude_control->set_throttle_out(0, true, 0);  // 设置油门输出为0
        quadplane.relax_attitude_control();  // 放松姿态控制
        pos_control->relax_z_controller(0);  // 放松高度控制器
    } else {
        // 正常悬停控制
        plane.quadplane.assign_tilt_to_fwd_thr();  // 分配倾斜到前向推力
        quadplane.hold_hover(quadplane.get_pilot_desired_climb_rate_cms());  // 保持悬停，使用飞行员期望的爬升率
    }

    // 使用固定翼表面进行稳定
    plane.stabilize_roll();  // 稳定横滚
    plane.stabilize_pitch();  // 稳定俯仰
}

#endif
