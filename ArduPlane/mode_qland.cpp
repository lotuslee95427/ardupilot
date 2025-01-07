#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

// 进入QLand模式
bool ModeQLand::_enter()
{
    // 初始化QLoiter模式
    plane.mode_qloiter._enter();
    // 禁用油门等待
    quadplane.throttle_wait = false;
    // 设置目标位置
    quadplane.setup_target_position();
    // 设置位置控制状态为降落下降
    poscontrol.set_state(QuadPlane::QPOS_LAND_DESCEND);
    // 重置飞行员修正标志
    poscontrol.pilot_correction_done = false;
    // 记录最后的地面高度
    quadplane.last_land_final_agl = plane.relative_ground_altitude(plane.g.rangefinder_landing);
    // 重置着陆检测计时器
    quadplane.landing_detect.lower_limit_start_ms = 0;
    quadplane.landing_detect.land_start_ms = 0;
#if AP_LANDINGGEAR_ENABLED
    // 部署起落架
    plane.g2.landing_gear.deploy_for_landing();
#endif

    return true;
}

// 更新QLand模式
void ModeQLand::update()
{
    // 调用QStabilize模式的更新函数
    plane.mode_qstabilize.update();
}

// 运行QLand模式
void ModeQLand::run()
{
    /*
      使用QLoiter模式进行主要控制
     */
    plane.mode_qloiter.run();
}

#endif
