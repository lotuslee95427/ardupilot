#include "mode.h"
#include "Plane.h"

// 更新手动模式
void ModeManual::update()
{
    // 设置副翼通道输出
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.roll_in_expo(false));
    // 设置升降舵通道输出
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.pitch_in_expo(false));
    // 输出方向舵和转向
    output_rudder_and_steering(plane.rudder_in_expo(false));

    // 获取油门输入
    const float throttle = plane.get_throttle_input(true);
    // 设置油门通道输出
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);

    // 更新导航滚转角度
    plane.nav_roll_cd = ahrs.roll_sensor;
    // 更新导航俯仰角度
    plane.nav_pitch_cd = ahrs.pitch_sensor;
}

// 运行手动模式
void ModeManual::run()
{
    // 重置控制器
    reset_controllers();
}

// 判断是否应该应用油门限制
bool ModeManual::use_throttle_limits() const
{
#if HAL_QUADPLANE_ENABLED
    // 如果四旋翼可用且启用了手动模式下的怠速调速器选项
    if (quadplane.available() && quadplane.option_is_set(QuadPlane::OPTION::IDLE_GOV_MANUAL)) {
        return true;
    }
#endif
    return false;
}
