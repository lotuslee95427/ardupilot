// 包含断言头文件
#include <assert.h>

// 包含HAL头文件
#include "HAL.h"

// AP_HAL命名空间
namespace AP_HAL {

// FunCallbacks类的构造函数
// @param setup_fun: 初始化函数指针,在系统启动时调用
// @param loop_fun: 循环函数指针,在主循环中重复调用
HAL::FunCallbacks::FunCallbacks(void (*setup_fun)(void), void (*loop_fun)(void))
    // 初始化列表,将传入的函数指针赋值给成员变量
    : _setup(setup_fun)  // 初始化setup函数指针
    , _loop(loop_fun)    // 初始化loop函数指针
{
}

} // 命名空间结束
