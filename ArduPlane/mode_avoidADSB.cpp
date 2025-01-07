#include "mode.h"
#include "Plane.h"

#if HAL_ADSB_ENABLED

// 进入避免ADSB模式
bool ModeAvoidADSB::_enter()
{
    // 调用引导模式的enter函数
    return plane.mode_guided.enter();
}

// 更新避免ADSB模式
void ModeAvoidADSB::update()
{
    // 调用引导模式的update函数
    plane.mode_guided.update();
}

// 避免ADSB模式的导航逻辑
void ModeAvoidADSB::navigate()
{
    // 更新飞机的盘旋状态
    // 参数为0表示使用WP_LOITER_RAD作为盘旋半径
    plane.update_loiter(0);
}

#endif // HAL_ADSB_ENABLED
