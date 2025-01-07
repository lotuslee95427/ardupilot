// 包含所需的头文件
#include <AP_gtest.h>
#include <AP_HAL/HAL.h>
#include <AP_HAL/Util.h>

// 获取HAL实例
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// 定义字符串比较宏
#define streq(x, y) (!strcmp(x, y))

// 来自GNU snprintf手册:

       // The functions snprintf() and vsnprintf() do not write  more  than  size
       // bytes  (including the terminating null byte ('\0')).  If the output was
       // truncated due to this limit, then the return value  is  the  number  of
       // characters  (excluding the terminating null byte) which would have been
       // written to the final string if enough space had been available.   Thus,
       // a  return  value  of  size or more means that the output was truncated.

// 测试vsnprintf基本功能
TEST(vsnprintf_Test, Basic)
{
    char output[300];
    {
        // 测试基本格式化输出
        int bytes_required = hal.util->snprintf(output, ARRAY_SIZE(output), "Fred: %u", 37);
        EXPECT_TRUE(streq(output, "Fred: 37"));
        EXPECT_EQ(bytes_required, 8);
    }
    {
        // 测试缓冲区截断
        int bytes_required = hal.util->snprintf(output, 3, "Fred: %u", 37);
        EXPECT_TRUE(streq(output, "Fr"));
        EXPECT_EQ(bytes_required, 8);
    }
    {
        // 测试零长度缓冲区
        snprintf(output, ARRAY_SIZE(output), "0123");
        int bytes_required = hal.util->snprintf(output, 0, "Fred: %u", 37);
        EXPECT_TRUE(streq(output, "0123"));
        EXPECT_EQ(bytes_required, 8);
    }
    { // 确保缓冲区其余部分保持不变
        memset(output, 'A', 10);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-truncation"
        const int bytes_required = snprintf(output, 5, "012345678");
#pragma GCC diagnostic pop
        EXPECT_TRUE(streq(output, "0123"));
        EXPECT_EQ(bytes_required, 9);
        EXPECT_EQ(output[6], 'A');
    }
    { // 测试简单浮点数格式化
        const int bytes_required = hal.util->snprintf(output, ARRAY_SIZE(output), "%f", 1/3.0);
        EXPECT_EQ(bytes_required, 8);
        EXPECT_TRUE(streq(output, "0.333333"));
    }
    { // 测试复杂浮点数格式化
        const int bytes_required = hal.util->snprintf(output, ARRAY_SIZE(output), "%30.9f", 1/3.0);
        EXPECT_EQ(bytes_required, 28);
        EXPECT_TRUE(streq(output, "                   0.3333333"));
    }

    { // 测试字符串格式化
        const int bytes_required = hal.util->snprintf(output, ARRAY_SIZE(output), "%s %s %c", "ABC", "DEF", 'x');
        EXPECT_EQ(bytes_required, 9);
        EXPECT_TRUE(streq(output, "ABC DEF x"));
    }
}

// 主函数入口
AP_GTEST_MAIN()
