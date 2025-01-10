#include "Copter.h"

//
// 故障保护支持
// Andrew Tridgell, 2011年12月
//
// 我们的故障保护策略是检测主循环锁死并停止电机
//

// 故障保护状态变量
static bool failsafe_enabled;         // 故障保护是否启用
static uint16_t failsafe_last_ticks;  // 上次主循环运行的tick计数
static uint32_t failsafe_last_timestamp;  // 上次主循环运行的时间戳
static bool in_failsafe;              // 是否处于故障保护状态

//
// failsafe_enable - 启用故障保护
//
void Copter::failsafe_enable()
{
    failsafe_enabled = true;
    failsafe_last_timestamp = micros();
}

//
// failsafe_disable - 当我们知道主循环会显著延迟时使用
//
void Copter::failsafe_disable()
{
    failsafe_enabled = false;
}

//
// failsafe_check - 此函数由核心定时器中断以1kHz频率调用
//
void Copter::failsafe_check()
{
    uint32_t tnow = AP_HAL::micros();  // 获取当前时间戳

    const uint16_t ticks = scheduler.ticks();  // 获取当前调度器tick计数
    if (ticks != failsafe_last_ticks) {
        // 主循环正在运行,一切正常
        failsafe_last_ticks = ticks;
        failsafe_last_timestamp = tnow;
        if (in_failsafe) {
            // 如果之前处于故障保护状态,现在恢复正常
            in_failsafe = false;
            LOGGER_WRITE_ERROR(LogErrorSubsystem::CPU, LogErrorCode::FAILSAFE_RESOLVED);
        }
        return;
    }

    if (!in_failsafe && failsafe_enabled && tnow - failsafe_last_timestamp > 2000000) {
        // 电机在运行但主循环已经2秒没有运行
        // 这意味着出现问题,应该停止电机
        in_failsafe = true;
        // 将电机输出降至最低(不立即解锁是为了记录故障)
        if (motors->armed()) {
            motors->output_min();
        }

        LOGGER_WRITE_ERROR(LogErrorSubsystem::CPU, LogErrorCode::FAILSAFE_OCCURRED);
    }

    if (failsafe_enabled && in_failsafe && tnow - failsafe_last_timestamp > 1000000) {
        // 每秒解锁电机一次
        failsafe_last_timestamp = tnow;
        if(motors->armed()) {
            motors->armed(false);
            motors->output();
        }
    }
}


#if ADVANCED_FAILSAFE
/*
  检查AFS(高级故障保护系统)故障保护
*/
void Copter::afs_fs_check(void)
{
    // 执行AFS故障保护检查
    g2.afs.check(last_radio_update_ms);
}
#endif
