#include "mode.h"
#include "Plane.h"

// 更新稳定模式的导航参数
void ModeStabilize::update()
{
    // 将导航滚转角度设置为0
    plane.nav_roll_cd = 0;
    // 将导航俯仰角度设置为0
    plane.nav_pitch_cd = 0;
}

// 运行稳定模式的主要逻辑
void ModeStabilize::run()
{
    // 稳定飞机的滚转
    plane.stabilize_roll();
    // 稳定飞机的俯仰
    plane.stabilize_pitch();
    // 执行直接的摇杆混合稳定
    stabilize_stick_mixing_direct();
    // 稳定飞机的偏航
    plane.stabilize_yaw();

    // 输出飞行员的油门控制
    output_pilot_throttle();
}
