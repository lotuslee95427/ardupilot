// 包含必要的头文件
#include "Scheduler.h"
#include "AP_HAL.h"
#include <stdio.h>

// 使用 AP_HAL 命名空间
using namespace AP_HAL;

// 声明外部 HAL 实例引用
extern const AP_HAL::HAL& hal;

/**
 * @brief 注册延迟回调函数
 * @param proc 要注册的回调函数
 * @param min_time_ms 最小延迟时间(毫秒)
 */
void Scheduler::register_delay_callback(AP_HAL::Proc proc,
                                        uint16_t min_time_ms)
{
    _delay_cb = proc;
    _min_delay_cb_ms = min_time_ms;
}

/**
 * @brief 执行延迟回调函数
 * 如果回调函数未设置或已在执行中则直接返回,
 * 防止递归调用
 */
void Scheduler::call_delay_cb()
{
    if (_delay_cb == nullptr) {
        return;
    }
    if (_in_delay_callback) {
        // 防止递归调用!
        return;
    }
    _in_delay_callback = true;
    _delay_cb();
    _in_delay_callback = false;
}

/**
 * @brief ExpectDelay类构造函数
 * @param ms 预期延迟时间(毫秒)
 */
ExpectDelay::ExpectDelay(uint32_t ms)
{
    hal.scheduler->expect_delay_ms(ms);
}

/**
 * @brief ExpectDelay类析构函数
 * 重置预期延迟时间为0
 */
ExpectDelay::~ExpectDelay()
{
    hal.scheduler->expect_delay_ms(0);
}

/**
 * @brief TimeCheck类用于支持TIME_CHECK()功能
 * 实现对代码执行时间的监控
 */
TimeCheck::TimeCheck(uint32_t _limit_ms, const char *_file, uint32_t _line) :
    limit_ms(_limit_ms),
    line(_line),
    file(_file)
{
    start_ms = AP_HAL::millis();
}

/**
 * @brief TimeCheck类析构函数
 * 检查代码执行时间是否超过限制,
 * 如果超时则打印警告信息
 */
TimeCheck::~TimeCheck()
{
    const uint32_t end_ms = AP_HAL::millis();
    const uint32_t delta_ms = end_ms - start_ms;
    if (delta_ms > limit_ms) {
        ::printf("Delta %u at %s:%u\n", unsigned(delta_ms), file, unsigned(line));
    }
}
