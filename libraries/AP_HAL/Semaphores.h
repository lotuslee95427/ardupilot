#pragma once

#include "AP_HAL_Namespace.h"

#include <AP_Common/AP_Common.h>

// 定义永久阻塞的超时时间为0
#define HAL_SEMAPHORE_BLOCK_FOREVER 0

// 信号量基类
class AP_HAL::Semaphore {
public:
    // 构造函数
    Semaphore() {}

    // 禁止拷贝构造和赋值操作
    CLASS_NO_COPY(Semaphore);

    // 尝试获取信号量,带超时时间
    // @param timeout_ms: 超时时间(毫秒)
    // @return: 是否成功获取信号量
    virtual bool take(uint32_t timeout_ms) WARN_IF_UNUSED = 0 ;

    // 尝试非阻塞方式获取信号量
    // @return: 是否成功获取信号量
    virtual bool take_nonblocking() WARN_IF_UNUSED = 0;

    // 阻塞方式获取信号量,永久等待直到获取成功
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-result"
    virtual void take_blocking() { take(HAL_SEMAPHORE_BLOCK_FOREVER); };
    #pragma GCC diagnostic pop
    
    // 释放信号量
    // @return: 是否成功释放信号量
    virtual bool give() = 0;

    // 虚析构函数
    virtual ~Semaphore(void) {}
};

/*
  信号量辅助类,用于减少信号量使用错误
  WITH_SEMAPHORE() 宏会永久阻塞直到获取信号量,并在作用域结束时自动释放信号量

  注意我们有两种类型的信号量:
  1. 普通信号量 - 只能被获取一次
  2. 递归信号量 - 持有信号量的线程可以多次获取,但必须释放相同次数

  WITH_SEMAPHORE() 宏可用于这两种类型的信号量
 */

// 信号量RAII包装类
class WithSemaphore {
public:
    // 构造函数,接受信号量指针
    WithSemaphore(AP_HAL::Semaphore *mtx, uint32_t line);
    // 构造函数,接受信号量引用
    WithSemaphore(AP_HAL::Semaphore &mtx, uint32_t line);

    // 析构函数,自动释放信号量
    ~WithSemaphore();
private:
    AP_HAL::Semaphore &_mtx;
};

// 来自: https://stackoverflow.com/questions/19666142/why-is-a-level-of-indirection-needed-for-this-concatenation-macro
// WITH_SEMAPHORE宏,用于创建信号量RAII对象
#define WITH_SEMAPHORE( sem ) JOIN( sem, __AP_LINE__, __COUNTER__ )

// 宏连接辅助函数
#define JOIN( sem, line, counter ) _DO_JOIN( sem, line, counter )
#define _DO_JOIN( sem, line, counter ) WithSemaphore _getsem ## counter(sem, line)

/*
  二值信号量类
 */
class AP_HAL::BinarySemaphore {
public:
    /*
      创建二值信号量
      @param initial_state: 初始状态
      - true: 创建后首次wait()不会阻塞
      - false: 创建后首次wait()会阻塞
     */
    BinarySemaphore(bool initial_state=false) {}

    // 禁止拷贝构造和赋值操作
    CLASS_NO_COPY(BinarySemaphore);

    // 等待信号量,带超时时间
    // @param timeout_us: 超时时间(微秒)
    // @return: 是否成功获取信号量
    virtual bool wait(uint32_t timeout_us) WARN_IF_UNUSED = 0 ;

    // 阻塞等待信号量
    // @return: 是否成功获取信号量
    virtual bool wait_blocking() = 0;

    // 非阻塞等待信号量
    // @return: 是否成功获取信号量
    virtual bool wait_nonblocking() { return wait(0); }

    // 发送信号
    virtual void signal() = 0;

    // 在中断上下文中发送信号
    virtual void signal_ISR() { signal(); }
    
    // 虚析构函数
    virtual ~BinarySemaphore(void) {}
};
