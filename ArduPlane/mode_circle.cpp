#include "mode.h"
#include "Plane.h"

// 进入圆形模式时的初始化函数
bool ModeCircle::_enter()
{
    // 设置圆形飞行的高度为当前高度
    plane.next_WP_loc.alt = plane.current_loc.alt;

    return true;
}

// 圆形模式的更新函数，每帧调用
void ModeCircle::update()
{
    // 在以下情况下使用:
    // 1. 没有安装GPS且失去无线电联系
    // 2. 想要在没有GPS的情况下进行温和的圆形飞行
    // 保持进入模式时设置的高度

    // 设置导航滚转角为滚转限制的1/3
    plane.nav_roll_cd  = plane.roll_limit_cd / 3;
    
    // 更新载荷系数
    plane.update_load_factor();
    
    // 计算导航俯仰角
    plane.calc_nav_pitch();
    
    // 计算油门值
    plane.calc_throttle();
}
