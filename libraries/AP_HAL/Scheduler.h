#pragma once

#include <stdint.h>

#include <AP_Common/AP_Common.h>

#include "AP_HAL_Boards.h"
#include "AP_HAL_Namespace.h"

// AP_HAL::Scheduler 类定义了调度器的抽象接口
class AP_HAL::Scheduler {
public:
    Scheduler() {}
    // 初始化调度器
    virtual void     init() = 0;
    // 延时指定的毫秒数
    virtual void     delay(uint16_t ms) = 0;

    /*
      延时指定的微秒数。这个延时需要尽可能精确 - 最好在100微秒以内。
     */
    virtual void     delay_microseconds(uint16_t us) = 0;

    /*
      延时指定的微秒数。在支持的平台上,当延时完成时会短暂提升主线程的优先级。
      目的是确保每个循环开始时主线程运行的优先级高于驱动程序。
     */
    virtual void     delay_microseconds_boost(uint16_t us) { delay_microseconds(us); }

    /*
      通知调度器我们正在从主线程调用一个可能需要较长时间的操作。
      这可以用来防止在预期的长延时期间发生看门狗复位。
      值为零时取消之前的预期延时。
     */
    virtual void     expect_delay_ms(uint32_t ms) { }

    /*
      如果我们处于预期延时期间则返回true。
      这可以用来抑制错误消息。
     */
    virtual bool     in_expected_delay(void) const { return false; }

    /*
      结束delay_microseconds_boost()带来的优先级提升
     */
    virtual void     boost_end(void) {}

    // 注册一个函数,当调度器需要休眠超过min_time_ms时会调用该函数
    virtual void     register_delay_callback(AP_HAL::Proc,
                                             uint16_t min_time_ms);
    // 如果调度器已调用延时回调函数则返回true。
    // 如果你在主线程上,这意味着你的调用栈中某处有调度器。
    virtual bool     in_delay_callback() const { return _in_delay_callback; }

    // 注册一个高优先级定时器任务
    virtual void     register_timer_process(AP_HAL::MemberProc) = 0;

    // 注册一个低优先级IO任务
    virtual void     register_io_process(AP_HAL::MemberProc) = 0;

    // 注册一个故障保护定时器
    virtual void     register_timer_failsafe(AP_HAL::Proc,
                                             uint32_t period_us) = 0;

    // 检查和设置系统初始化状态
    virtual void     set_system_initialized() = 0;
    virtual bool     is_system_initialized() = 0;

    // 重启系统,可选择是否保持在bootloader中
    virtual void     reboot(bool hold_in_bootloader = false) = 0;

    /**
       可选函数,用于在给定时间停止时钟,用于日志回放
     */
    virtual void     stop_clock(uint64_t time_usec) {}

    // 判断当前是否在主线程中
    virtual bool     in_main_thread() const = 0;

    /*
      禁用中断并返回一个可用于恢复中断状态的上下文。
      这可用于保护关键区域。

      警告:可能不是在所有HAL上都实现
     */
    virtual void *disable_interrupts_save(void) { return nullptr; }

    /*
      从disable_interrupts_save()恢复中断状态
     */
    virtual void restore_interrupts(void *) {}

    // 当子类需要延时一段时间时调用
    virtual void call_delay_cb();
    uint16_t _min_delay_cb_ms;

    /*
      priority_base用于选择新线程相对于什么基准的优先级
     */
    enum priority_base {
        PRIORITY_BOOST,     // 提升优先级
        PRIORITY_MAIN,      // 主线程优先级
        PRIORITY_SPI,       // SPI优先级
        PRIORITY_I2C,       // I2C优先级
        PRIORITY_CAN,       // CAN优先级
        PRIORITY_TIMER,     // 定时器优先级
        PRIORITY_RCOUT,     // RC输出优先级
        PRIORITY_LED,       // LED优先级
        PRIORITY_RCIN,      // RC输入优先级
        PRIORITY_IO,        // IO优先级
        PRIORITY_UART,      // UART优先级
        PRIORITY_STORAGE,   // 存储优先级
        PRIORITY_SCRIPTING, // 脚本优先级
        PRIORITY_NET,       // 网络优先级
    };
    
    /*
      创建一个新线程
     */
    virtual bool thread_create(AP_HAL::MemberProc proc, const char *name,
                               uint32_t stack_size, priority_base base, int8_t priority) {
        return false;
    }

private:
    // 延时回调函数指针
    AP_HAL::Proc _delay_cb;
    // 是否在延时回调中的标志位
    bool _in_delay_callback : 1;

};

/*
  辅助宏和类,使expect_delay_ms()的使用更安全和容易
 */
class ExpectDelay {
public:
    ExpectDelay(uint32_t ms);
    ~ExpectDelay();
};

#define EXPECT_DELAY_MS(ms) DELAY_JOIN( ms, __COUNTER__ )
#define DELAY_JOIN( ms, counter) _DO_DELAY_JOIN( ms, counter )
#define _DO_DELAY_JOIN( ms, counter ) ExpectDelay _getdelay ## counter(ms)


/*
  TIME_CHECK()可用于检测意外的长延时。
  将它们分散在可能的位置,任何长延时都会被打印出来
 */

class TimeCheck {
public:
    TimeCheck(uint32_t limit_ms, const char *file, uint32_t line);
    ~TimeCheck();
private:
    const uint32_t limit_ms;  // 时间限制(毫秒)
    const uint32_t line;      // 代码行号
    const char *file;         // 文件名
    uint32_t start_ms;        // 开始时间
};

#define TIME_CHECK(limit_ms) JOIN_TC(limit_ms, __FILE__, __LINE__, __COUNTER__ )
#define JOIN_TC(limit_ms, file, line, counter ) _DO_JOIN_TC( limit_ms, file, line, counter )
#define _DO_JOIN_TC(limit_ms, file, line, counter ) TimeCheck _gettc ## counter(limit_ms, file, line)
