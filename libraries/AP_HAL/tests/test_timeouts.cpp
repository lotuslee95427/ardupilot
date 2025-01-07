// 包含所需的头文件
#include <AP_gtest.h>
#include <AP_HAL/HAL.h>
#include <AP_HAL/Util.h>
#include <AP_Math/AP_Math.h>

// 获取HAL实例
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// 超时测试函数模板
template <typename T>
static void timeout_test(T timenow, T trigger)
{
    T ticks = 2000U;
    T time_past = timenow;
    bool timeout_triggered = false;

    while (ticks-- != 0) {
        timenow++;
        if (AP_HAL::timeout_expired(time_past, timenow, 1000U)) {
            if (!timeout_triggered) {
                ASSERT_EQ(timenow, trigger);
                timeout_triggered = true;
            }
        }
    }

    ASSERT_TRUE(timeout_triggered);
}

// 测试超时计数器溢出情况
TEST(timeout_wrap_Test, Basic)
{
    timeout_test(0xFFFFFFFFU - 499U, 500U);
}

// 测试基本超时功能
TEST(timeout_Test, Basic)
{
    timeout_test(500U, 1500U);
}

// 测试剩余超时时间函数模板
template <typename T>
static void timeout_remaining(T timenow, T incr)
{
    T ticks = 2000U;
    T time_past = timenow;
    T last_remaining = 1000;

    while (ticks-- != 0) {
        timenow+=incr;
        const uint32_t new_remaining = AP_HAL::timeout_remaining(time_past, timenow, 1000U);
        ASSERT_LT(new_remaining, MAX(last_remaining, 1U));   // 剩余时间必须递减
        last_remaining = new_remaining;
        ASSERT_LT(new_remaining, 1001U);
    }

    ASSERT_EQ(AP_HAL::timeout_remaining(time_past, timenow, 1000U), 0U);
}

// 测试剩余时间计数器溢出情况
TEST(timeout_remaining_wrap_Test, Basic)
{
    timeout_remaining(0xFFFFFFFFU - 500U, 1U);
}

// 测试基本剩余时间功能
TEST(timeout_remaining_Test, Basic)
{
    timeout_remaining(500U, 1U);
}

// 测试5微秒增量的剩余时间计数器溢出情况
TEST(timeout_remaining_wrap_Test_5us, Basic)
{
    timeout_remaining(0xFFFFFFFF - 500U, 5U);
}

AP_GTEST_MAIN()
