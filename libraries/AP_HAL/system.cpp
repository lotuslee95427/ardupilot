#include "system.h"

// 获取系统运行时间的毫秒数(16位)
// WEAK表示这是一个弱符号定义,可以被其他实现覆盖
// 通过与0xFFFF进行位与运算,将32位毫秒值截断为16位
uint16_t WEAK AP_HAL::millis16()
{
    return millis() & 0xFFFF;
}

// 获取系统运行时间的微秒数(16位)
// WEAK表示这是一个弱符号定义,可以被其他实现覆盖
// 通过与0xFFFF进行位与运算,将32位微秒值截断为16位
uint16_t WEAK AP_HAL::micros16()
{
    return micros() & 0xFFFF;
}

// 打印堆栈跟踪信息的函数
// WEAK表示这是一个弱符号定义,可以被其他实现覆盖
// 在当前平台上未实现堆栈跟踪功能
void WEAK AP_HAL::dump_stack_trace()
{
    // stack dump not available on this platform
}

// 生成核心转储文件的函数
// WEAK表示这是一个弱符号定义,可以被其他实现覆盖
// 在当前平台上未实现核心转储功能
void WEAK AP_HAL::dump_core_file()
{
    // core dump not available on this platform
}
