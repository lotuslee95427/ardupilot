#pragma once

#include <stdint.h>

#include <AP_Common/AP_Common.h>

#include "AP_HAL_Macros.h"

namespace AP_HAL {

// 初始化HAL系统
void init();

// 系统异常处理函数,打印错误信息并终止程序运行
// FMT_PRINTF(1,2)表示第1个参数是格式字符串,从第2个参数开始是可变参数
// NORETURN表示该函数不会返回
void panic(const char *errormsg, ...) FMT_PRINTF(1, 2) NORETURN;

// 获取系统运行时间的微秒数(16位)
uint16_t micros16();
// 获取系统运行时间的微秒数(32位)
uint32_t micros();
// 获取系统运行时间的毫秒数(32位)
uint32_t millis();
// 获取系统运行时间的毫秒数(16位)
uint16_t millis16();
// 获取系统运行时间的微秒数(64位)
uint64_t micros64();
// 获取系统运行时间的毫秒数(64位)
uint64_t millis64();

// 打印堆栈跟踪信息
void dump_stack_trace();
// 生成核心转储文件
void dump_core_file();

// 可靠地判断超时是否已过期的模板函数
// 使用任意无符号时间类型和时间间隔
// 模板确保调用者不会混用不同类型的时间值
// 即使当前时间和过去时间跨越了计数器溢出边界,比较也能正确工作
template <typename T, typename S, typename R>
inline bool timeout_expired(const T past_time, const S now, const R timeout)
{
    // 静态断言确保:
    // 1. 比较的时间值类型必须相同
    static_assert(std::is_same<T, S>::value, "timeout_expired() must compare values of the same unsigned type");
    // 2. 时间类型必须是无符号类型
    static_assert(std::is_unsigned<T>::value, "timeout_expired() must use unsigned times");
    // 3. 超时时间必须是无符号类型
    static_assert(std::is_unsigned<R>::value, "timeout_expired() must use unsigned timeouts");
    const T dt = now - past_time;
    return (dt >= timeout);
}

// 可靠地计算超时剩余时间的模板函数
// 使用任意无符号时间类型和时间间隔
// 模板确保调用者不会混用不同类型的时间值
// 即使当前时间和过去时间跨越了计数器溢出边界,计算也能正确工作
template <typename T, typename S, typename R>
inline T timeout_remaining(const T past_time, const S now, const R timeout)
{
    // 静态断言确保:
    // 1. 比较的时间值类型必须相同
    static_assert(std::is_same<T, S>::value, "timeout_remaining() must compare values of the same unsigned type");
    // 2. 时间类型必须是无符号类型
    static_assert(std::is_unsigned<T>::value, "timeout_remaining() must use unsigned times");
    // 3. 超时时间必须是无符号类型
    static_assert(std::is_unsigned<R>::value, "timeout_remaining() must use unsigned timeouts");
    const T dt = now - past_time;
    return (dt >= timeout) ? T(0) : (timeout - dt);
}

} // namespace AP_HAL
