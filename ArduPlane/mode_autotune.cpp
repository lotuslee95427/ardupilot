#include "mode.h"
#include "Plane.h"

// 进入自动调谐模式
bool ModeAutoTune::_enter()
{
    // 启动飞机的自动调谐功能
    plane.autotune_start();

    return true;
}

// 更新自动调谐模式
void ModeAutoTune::update()
{
    // 调用FBWA模式的更新函数
    plane.mode_fbwa.update();
}

// 运行自动调谐模式
void ModeAutoTune::run()
{
    // 运行基类的run函数
    Mode::run();

    // 输出飞行员的油门控制
    output_pilot_throttle();
}
