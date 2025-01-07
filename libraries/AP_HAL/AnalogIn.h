#pragma once

#include <inttypes.h>

#include "AP_HAL_Namespace.h"
#include "AP_HAL_Boards.h"

// 模拟输入源类
class AP_HAL::AnalogSource {
public:
    // 读取平均值
    virtual float read_average() = 0;
    // 读取最新值
    virtual float read_latest() = 0;
    // 设置引脚号
    virtual bool set_pin(uint8_t p) WARN_IF_UNUSED = 0;

    // 返回一个从0.0到5.0V的电压值,根据参考电压进行缩放
    virtual float voltage_average() = 0;

    // 返回一个从0.0到5.0V的电压值,根据参考电压进行缩放
    virtual float voltage_latest() = 0;

    // 返回一个从0.0到5.0V的电压值,假设是比例传感器
    virtual float voltage_average_ratiometric() = 0;
};

// 模拟输入类
class AP_HAL::AnalogIn {
public:
    // 初始化
    virtual void init() = 0;
    // 获取指定通道
    virtual AP_HAL::AnalogSource* channel(int16_t n) = 0;

    // 获取板载5V电压值(单位:伏特)
    virtual float board_voltage(void) = 0;

    // 获取舵机供电电压(单位:伏特),如果未知则返回0
    virtual float servorail_voltage(void) { return 0; }

    // 电源状态标志,参见 MAV_POWER_STATUS
    virtual uint16_t power_status_flags(void) { return 0; }

    // 所有曾经设置过的_power_flags位的位掩码,用于诊断瞬时故障
    virtual uint16_t accumulated_power_status_flags(void) const { return 0; }

    // 此枚举类与MAVLink的MAV_POWER_STATUS枚举一一对应!
    enum class PowerStatusFlag : uint16_t {
        BRICK_VALID = 1,                  // 主电源供电有效
        SERVO_VALID = 2,                  // FMU的主舵机电源有效
        USB_CONNECTED = 4,                // USB电源已连接
        PERIPH_OVERCURRENT = 8,           // 外设供电过流
        PERIPH_HIPOWER_OVERCURRENT = 16,  // 高功率外设供电过流
        CHANGED = 32,                     // 电源状态自启动以来已改变
    };

#if HAL_WITH_MCU_MONITORING
    // 获取MCU温度
    virtual float mcu_temperature(void) { return 0; }
    // 获取MCU电压
    virtual float mcu_voltage(void) { return 0; }
    // 获取MCU最大电压
    virtual float mcu_voltage_max(void) { return 0; }
    // 获取MCU最小电压
    virtual float mcu_voltage_min(void) { return 0; }
#endif
};

// 板载VCC模拟输入通道号
#define ANALOG_INPUT_BOARD_VCC 254
// 无效的模拟输入通道号
#define ANALOG_INPUT_NONE 255
