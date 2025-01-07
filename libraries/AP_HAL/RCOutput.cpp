// 包含 AP_HAL 头文件
#include "AP_HAL.h"

// 声明外部 HAL 实例引用
extern const AP_HAL::HAL &hal;

// DSHOT 协议相关的位宽和时序参数定义
uint32_t AP_HAL::RCOutput::DSHOT_BIT_WIDTH_TICKS = DSHOT_BIT_WIDTH_TICKS_DEFAULT;
uint32_t AP_HAL::RCOutput::DSHOT_BIT_0_TICKS = DSHOT_BIT_0_TICKS_DEFAULT;
uint32_t AP_HAL::RCOutput::DSHOT_BIT_1_TICKS = DSHOT_BIT_1_TICKS_DEFAULT;

// 辅助函数:将输出模式转换为字符串描述
// 用于实现 get_output_mode_banner 功能
const char* AP_HAL::RCOutput::get_output_mode_string(enum output_mode out_mode) const
{
    // 根据输出模式返回对应的字符串描述
    switch (out_mode) {
    case MODE_PWM_NONE:
        return "None";    // 无输出模式
    case MODE_PWM_NORMAL:
        return "PWM";     // 标准 PWM 模式
    case MODE_PWM_ONESHOT:
        return "OneS";    // OneShot PWM 模式
    case MODE_PWM_ONESHOT125:
        return "OS125";   // OneShot125 PWM 模式
    case MODE_PWM_BRUSHED:
        return "Brush";   // 有刷电机模式
    case MODE_PWM_DSHOT150:
        return "DS150";   // DShot150 协议
    case MODE_PWM_DSHOT300:
        return "DS300";   // DShot300 协议
    case MODE_PWM_DSHOT600:
        return "DS600";   // DShot600 协议
    case MODE_PWM_DSHOT1200:
        return "DS1200";  // DShot1200 协议
    case MODE_NEOPIXEL:
    case MODE_NEOPIXELRGB:
        return "NeoP";    // NeoPixel LED 模式
    case MODE_PROFILED:
        return "ProfiLED"; // ProfiLED 模式
    }

    // 未知模式的默认返回值
    return "Unknown";
}

// 将输出模式信息添加到状态信息字符串中
// banner_msg: 状态信息字符串
// banner_msg_len: 字符串最大长度
// out_mode: 输出模式
// low_ch: 最低通道号
// high_ch: 最高通道号
void AP_HAL::RCOutput::append_to_banner(char banner_msg[], uint8_t banner_msg_len, output_mode out_mode, uint8_t low_ch, uint8_t high_ch) const
{
    // 获取模式字符串描述
    const char* mode_str = get_output_mode_string(out_mode);

    // 创建临时缓冲区保存原始信息
    char banner_msg_temp[banner_msg_len];
    memcpy(banner_msg_temp, banner_msg, banner_msg_len);

    if (low_ch == high_ch) {
        // 单通道情况的格式化
        hal.util->snprintf(banner_msg, banner_msg_len, "%s %s:%u", banner_msg_temp, mode_str, (unsigned)low_ch);
    } else {
        // 多通道情况的格式化
        hal.util->snprintf(banner_msg, banner_msg_len, "%s %s:%u-%u", banner_msg_temp, mode_str, (unsigned)low_ch, (unsigned)high_ch);
    }
}

// 判断给定的输出模式是否为 DShot 协议
bool AP_HAL::RCOutput::is_dshot_protocol(const enum output_mode mode)
{
    switch (mode) {
    case MODE_PWM_DSHOT150:
    case MODE_PWM_DSHOT300:
    case MODE_PWM_DSHOT600:
    case MODE_PWM_DSHOT1200:
        return true;
    default:
        return false;
    }
}

// 计算达到目标频率所需的预分频值
// timer_clock: 定时器时钟频率
// target_frequency: 目标频率
// at_least_freq: 是否需要至少达到目标频率
uint32_t AP_HAL::RCOutput::calculate_bitrate_prescaler(uint32_t timer_clock, uint32_t target_frequency, bool at_least_freq)
{
    if (target_frequency > timer_clock) {
        // 目标频率超过时钟频率,无法实现
        return 0;
    }

    uint32_t prescaler;

    if (at_least_freq) { 
        // BLHeli_S 需要至少达到目标频率的模式
        prescaler = uint32_t(floorf((float) timer_clock / target_frequency + 0.01f) - 1);
    } else {
        // Betaflight 原始计算方式,选择最接近的频率
        // 双向 DShot 对比特率非常敏感
        prescaler = uint32_t(lrintf((float) timer_clock / target_frequency + 0.01f) - 1);
    }

    return prescaler;
}

// 将 PWM 值按照设定的 ESC 输出范围缩放到 [-1,1] 区间
// pwm: 输入的 PWM 值
float AP_HAL::RCOutput::scale_esc_to_unity(uint16_t pwm) const
{
    return 2.0 * ((float) pwm - _esc_pwm_min) / (_esc_pwm_max - _esc_pwm_min) - 1.0;
}
