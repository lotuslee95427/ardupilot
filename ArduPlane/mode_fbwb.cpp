#include "mode.h"
#include "Plane.h"

// FBWB模式进入函数
bool ModeFBWB::_enter()
{
#if HAL_SOARING_ENABLED
    // 初始化ArduSoar滑翔控制器的巡航状态
    plane.g2.soaring_controller.init_cruising();
#endif

    // 设置当前高度为目标高度
    plane.set_target_altitude_current();

    return true;
}

// FBWB模式更新函数
void ModeFBWB::update()
{
    // 感谢Yury MonZon提供的高度限制代码！

    // 计算目标横滚角度（以厘度为单位）
    // 使用遥控器输入的归一化值乘以最大横滚角度限制
    plane.nav_roll_cd = plane.channel_roll->norm_input() * plane.roll_limit_cd;

    // 更新飞机的载荷系数
    plane.update_load_factor();

    // 更新FBWB模式下的速度和高度控制
    plane.update_fbwb_speed_height();
}
