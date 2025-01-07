/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */

#pragma once

#include "AP_HAL_ChibiOS.h"

// available ADC channels for allocation
// 可分配的ADC通道最大数量
#define ANALOG_MAX_CHANNELS 16

// physical ADC channels per ADC HAL driver
// This is MCU dependent, currently STM32H7 has the highest number of ADC_INs i.e. 20
// 每个ADC HAL驱动的物理ADC通道数
// 这取决于MCU,目前STM32H7有最多的ADC输入,即20个
#define HAL_MAX_ANALOG_IN_CHANNELS 20

// 定义HAL_NUM_ANALOG_INPUTS的默认值
// 根据是否定义了HAL_ANALOG3_PINS或HAL_WITH_MCU_MONITORING来决定ADC数量
#ifndef HAL_NUM_ANALOG_INPUTS
#if defined(HAL_ANALOG3_PINS) || HAL_WITH_MCU_MONITORING
#define HAL_NUM_ANALOG_INPUTS 3
#elif defined(HAL_ANALOG2_PINS)
#define HAL_NUM_ANALOG_INPUTS 2
#else
#define HAL_NUM_ANALOG_INPUTS 1
#endif
#endif

// number of samples on each channel to gather on each DMA callback
// 每次DMA回调时在每个通道上采集的样本数
#define ADC_DMA_BUF_DEPTH 8

#if HAL_USE_ADC == TRUE && !defined(HAL_DISABLE_ADC_DRIVER)

// 模拟输入源类,继承自AP_HAL::AnalogSource
class ChibiOS::AnalogSource : public AP_HAL::AnalogSource {
public:
    friend class ChibiOS::AnalogIn;
    AnalogSource(int16_t pin);
    float read_average() override;      // 读取平均值
    float read_latest() override;       // 读取最新值
    bool set_pin(uint8_t p) override WARN_IF_UNUSED;  // 设置引脚
    float voltage_average() override;   // 读取平均电压
    float voltage_latest() override;    // 读取最新电压
    float voltage_average_ratiometric() override;  // 读取比率平均电压

private:
    int16_t _pin;                      // 引脚编号
    float _value;                      // 当前值
    float _value_ratiometric;          // 比率值
    float _latest_value;               // 最新值
    uint8_t _sum_count;                // 累加计数
    float _sum_value;                  // 累加值
    float _sum_ratiometric;            // 比率累加值
    void _add_value(float v, float vcc5V);  // 添加新值
    float _pin_scaler();               // 引脚缩放因子
    HAL_Semaphore _semaphore;          // 信号量
};

// 模拟输入类,继承自AP_HAL::AnalogIn
class ChibiOS::AnalogIn : public AP_HAL::AnalogIn {
public:
    friend class AnalogSource;

    void init() override;              // 初始化
    AP_HAL::AnalogSource* channel(int16_t pin) override;  // 获取通道
    void _timer_tick(void);            // 定时器滴答
    void timer_tick_adc(uint8_t index);  // ADC定时器滴答
    float board_voltage(void) override { return _board_voltage; }  // 获取板载电压
    float servorail_voltage(void) override { return _servorail_voltage; }  // 获取舵机电源电压
    uint16_t power_status_flags(void) override { return _power_flags; }  // 获取电源状态标志
    uint16_t accumulated_power_status_flags(void) const override { return _accumulated_power_flags; }  // 获取累积电源状态标志

#if HAL_WITH_MCU_MONITORING
    // MCU监控相关函数
    float mcu_temperature(void) override { return _mcu_temperature; }  // 获取MCU温度
    float mcu_voltage(void) override { return _mcu_voltage; }          // 获取MCU电压
    float mcu_voltage_max(void) override { return _mcu_voltage_max; }  // 获取MCU最大电压
    float mcu_voltage_min(void) override { return _mcu_voltage_min; }  // 获取MCU最小电压
#endif

private:
    void read_adc(uint8_t index, uint32_t *val);  // 读取ADC值
    void update_power_flags(void);      // 更新电源标志
    void setup_adc(uint8_t index);      // 设置ADC
    static void adccallback(ADCDriver *adcp);  // ADC回调函数

    ChibiOS::AnalogSource* _channels[ANALOG_MAX_CHANNELS];  // 模拟通道数组

    uint32_t _last_run;                // 上次运行时间
    float _board_voltage;              // 板载电压
    float _servorail_voltage;          // 舵机电源电压
    float _rssi_voltage;               // RSSI电压
    uint16_t _power_flags;             // 电源标志
    uint16_t _accumulated_power_flags;  // 累积电源标志

    ADCConversionGroup adcgrpcfg[HAL_NUM_ANALOG_INPUTS];  // ADC转换组配置

    // 静态工具函数
    static uint8_t get_num_grp_channels(uint8_t index);  // 获取组通道数
    static uint8_t get_pin_channel(uint8_t adc_index, uint8_t pin_index);  // 获取引脚通道
    static uint8_t get_analog_pin(uint8_t adc_index, uint8_t pin_index);   // 获取模拟引脚
    static float get_pin_scaling(uint8_t adc_index, uint8_t pin_index);    // 获取引脚缩放
    static uint8_t get_adc_index(ADCDriver* adcp);                         // 获取ADC索引

    // 引脚信息结构体
    struct pin_info {
        uint8_t channel;    // 通道号
        uint8_t analog_pin; // 模拟引脚号
        float scaling;      // 缩放因子
    };
    static const pin_info pin_config[];  // 引脚配置数组
#ifdef HAL_ANALOG2_PINS
    static const pin_info pin_config_2[];  // ADC2引脚配置
#endif
#if defined(HAL_ANALOG3_PINS) || HAL_WITH_MCU_MONITORING
    static const pin_info pin_config_3[];  // ADC3引脚配置
#endif

    static adcsample_t *samples[HAL_NUM_ANALOG_INPUTS];        // 采样数据
    static uint32_t *sample_sum[HAL_NUM_ANALOG_INPUTS];        // 采样和
    static uint32_t sample_count[HAL_NUM_ANALOG_INPUTS];       // 采样计数

    HAL_Semaphore _semaphore;  // 信号量

#if HAL_WITH_MCU_MONITORING
    // MCU监控相关变量
    uint16_t _mcu_monitor_sample_count;         // MCU监控采样计数
    uint32_t _mcu_monitor_temperature_accum;    // MCU温度累加值
    uint32_t _mcu_monitor_voltage_accum;        // MCU电压累加值
    uint16_t _mcu_vrefint_min;                 // MCU内部参考最小值
    uint16_t _mcu_vrefint_max;                 // MCU内部参考最大值

    float _mcu_temperature;    // MCU温度
    float _mcu_voltage;        // MCU电压
    float _mcu_voltage_min;    // MCU最小电压
    float _mcu_voltage_max;    // MCU最大电压
#endif
};

#endif // HAL_USE_ADC
