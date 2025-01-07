#include "AP_HAL.h"

// 声明外部HAL实例的引用
extern const AP_HAL::HAL &hal;

/*
  实现WithSemaphore类以支持WITH_SEMAPHORE()宏
  这是一个RAII风格的信号量包装类,用于自动管理信号量的获取和释放
 */

// 指针版本的构造函数,将指针转换为引用后调用另一个构造函数
WithSemaphore::WithSemaphore(AP_HAL::Semaphore *mtx, uint32_t line) :
    WithSemaphore(*mtx, line)
{}

// 引用版本的构造函数
WithSemaphore::WithSemaphore(AP_HAL::Semaphore &mtx, uint32_t line) :
    _mtx(mtx) // 初始化信号量成员
{
#ifndef HAL_BOOTLOADER_BUILD
    // 检查是否在主线程中执行
    bool in_main = hal.scheduler->in_main_thread();
    if (in_main) {
        // 如果在主线程中,记录当前的代码行号
        hal.util->persistent_data.semaphore_line = line;
    }
#endif
    // 阻塞式获取信号量
    _mtx.take_blocking();
#ifndef HAL_BOOTLOADER_BUILD
    if (in_main) {
        // 获取信号量后清除行号
        hal.util->persistent_data.semaphore_line = 0;
    }
#endif
}

// 析构函数 - 自动释放信号量
WithSemaphore::~WithSemaphore()
{
    _mtx.give();  // 释放信号量
}
