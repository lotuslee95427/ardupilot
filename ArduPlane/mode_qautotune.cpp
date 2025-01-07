#include "mode.h"
#include "Plane.h"

#include "qautotune.h"

#if QAUTOTUNE_ENABLED

// 进入QAutotune模式
bool ModeQAutotune::_enter()
{
#if QAUTOTUNE_ENABLED
    // 初始化四旋翼自动调参
    return quadplane.qautotune.init();
#else
    return false;
#endif
}

// 更新QAutotune模式
void ModeQAutotune::update()
{
    // 更新四旋翼稳定模式
    plane.mode_qstabilize.update();
}

// 运行QAutotune模式
void ModeQAutotune::run()
{
    const uint32_t now = AP_HAL::millis();
    if (quadplane.tailsitter.in_vtol_transition(now)) {
        // 尾座式飞机在VTOL转换的前向飞行拉起阶段运行固定翼控制器
        Mode::run();
        return;
    }

#if QAUTOTUNE_ENABLED
    // 运行四旋翼自动调参
    quadplane.qautotune.run();
#endif

    // 使用固定翼面进行稳定
    plane.stabilize_roll();
    plane.stabilize_pitch();
}

// 退出QAutotune模式
void ModeQAutotune::_exit()
{
#if QAUTOTUNE_ENABLED
    // 停止四旋翼自动调参
    plane.quadplane.qautotune.stop();
#endif
}

#endif
