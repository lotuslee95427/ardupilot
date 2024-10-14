#include "Copter.h"

#if MODE_GUIDED_NOGPS_ENABLED

/*
 * Init and run calls for guided_nogps flight mode
 * 无GPS引导模式的初始化和运行调用
 */

// initialise guided_nogps controller
// 初始化无GPS引导控制器
bool ModeGuidedNoGPS::init(bool ignore_checks)
{
    // start in angle control mode
    // 以角度控制模式启动
    ModeGuided::angle_control_start();
    return true;
}

// guided_run - runs the guided controller
// should be called at 100hz or more
// guided_run - 运行引导控制器
// 应该以100Hz或更高的频率调用
void ModeGuidedNoGPS::run()
{
    // run angle controller
    // 运行角度控制器
    ModeGuided::angle_control_run();
}

#endif
