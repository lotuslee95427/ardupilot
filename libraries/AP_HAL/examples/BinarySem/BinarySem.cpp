/*
  test of HAL_BinarySemaphore
  测试HAL_BinarySemaphore二值信号量
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>

#include <stdio.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class BinarySemTest {
public:
    // 创建两个二值信号量,sem1初始值为1,sem2初始值为0
    HAL_BinarySemaphore sem1{1};
    HAL_BinarySemaphore sem2{0};

    void setup(void);
    void thread1(void);
    void thread2(void);
    void update(bool ok);

    // 统计操作次数和超时次数
    uint32_t ops, timeouts;
    // 记录上次打印时间
    uint32_t last_print_us;
    // 互斥锁用于保护共享数据
    HAL_Semaphore mtx;
};

void BinarySemTest::setup(void)
{
    // 创建两个线程,优先级为IO优先级
    hal.scheduler->thread_create(
        FUNCTOR_BIND_MEMBER(&BinarySemTest::thread1, void), "thd1", 2048, AP_HAL::Scheduler::PRIORITY_IO, 0);
    hal.scheduler->thread_create(
        FUNCTOR_BIND_MEMBER(&BinarySemTest::thread2, void), "thd2", 2048, AP_HAL::Scheduler::PRIORITY_IO, 0);
    ::printf("Setup threads\n");
}

void BinarySemTest::thread2(void)
{
    while (true) {
        // 等待sem2信号量,超时时间50ms
        bool ok = sem2.wait(50000);
        // 释放sem1信号量
        sem1.signal();
        update(ok);
    }
}

void BinarySemTest::thread1(void)
{
    while (true) {
        // 等待sem1信号量,超时时间50ms
        bool ok = sem1.wait(50000);
        // 释放sem2信号量
        sem2.signal();
        update(ok);
    }
}

void BinarySemTest::update(bool ok)
{
    // 使用互斥锁保护共享数据访问
    WITH_SEMAPHORE(mtx);
    if (ok) {
        ops++;
    } else {
        timeouts++;
    }
    uint32_t now_us = AP_HAL::micros();
    float dt = (now_us - last_print_us)*1.0e-6;
    if (dt >= 1.0) {
        // 每秒打印一次统计信息
        last_print_us = now_us;
        ::printf("tick %u %.3f ops/s %.3f timeouts/s\n",
                 unsigned(AP_HAL::millis()),
                 ops/dt,
                 timeouts/dt);
        ops = 0;
        timeouts = 0;
    }
}

static BinarySemTest *ct;

void setup(void)
{
    // 创建并初始化测试对象
    ct = new BinarySemTest;
    ct->setup();
}

void loop(void)
{
    // 主循环每秒延时1000ms
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
