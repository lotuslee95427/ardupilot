#include "DataRateLimit.h" // 包含数据速率限制头文件

#include <AP_HAL/AP_HAL.h> // 包含HAL头文件

// 根据给定的字节/秒速率限制返回自上次调用以来可以发送的最大字节数
uint32_t DataRateLimit::max_bytes(const float bytes_per_sec)
{
    // 获取自上次调用以来的时间
    const uint32_t now_us = AP_HAL::micros();
    const float dt = (now_us - last_us) * 1.0e-6; // 转换为秒
    last_us = now_us;

    // 计算在这段时间内可以传输的最大字节数
    float max_bytes = bytes_per_sec * dt;

    // 加上上次调用的余数,这可以防止累积的舍入误差
    max_bytes += remainder;

    // 获取字节数的整数部分并存储余数
    float max_bytes_int;
    remainder = modf(max_bytes, &max_bytes_int);

    // 加0.5确保float正确舍入为整数
    return uint32_t(max_bytes_int + 0.5);
}
