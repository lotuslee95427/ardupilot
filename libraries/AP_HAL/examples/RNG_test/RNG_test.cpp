/*
  简单的随机数生成测试程序
  用于测试HAL层的随机数生成功能
 */

// 包含HAL抽象层头文件
#include <AP_HAL/AP_HAL.h>

// 声明setup和loop函数
void setup();
void loop();

// 获取HAL实例的引用
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// 初始化函数
void setup(void) {
    // 打印测试开始信息
    hal.console->printf("Running Random Number Generator Test!\n");
}

// 主循环函数
void loop(void)
{
    // 定义32位无符号整型变量存储随机数
    uint32_t random_number;
    // 尝试获取随机数,如果成功则打印
    if (hal.util->get_random_vals((uint8_t*)&random_number, sizeof(random_number))) {
        hal.console->printf("RNG %lx\n", (unsigned long)random_number);
    } else {
        // 获取随机数失败时打印错误信息
        hal.console->printf("RNG failed\n");
    }
    // 延时1秒
    hal.scheduler->delay(1000);
}

// ArduPilot HAL主程序入口点宏
AP_HAL_MAIN();