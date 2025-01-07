// 包含所需的头文件
#include "GPIO.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

// 声明HAL实例的外部引用
extern const AP_HAL::HAL& hal;

// PWMSource类的析构函数
AP_HAL::PWMSource::~PWMSource()
{
    // 如果中断已经附加,则需要先分离中断
    if (interrupt_attached) {
        // 假设分离中断总是成功的
        hal.gpio->detach_interrupt(_pin);
        interrupt_attached = false;
    }
}

// 设置PWM输入引脚
// new_pin: 新的引脚编号
// subsystem: 子系统名称,用于错误消息
bool AP_HAL::PWMSource::set_pin(int16_t new_pin, const char *subsystem)
{
    // 如果是同一个引脚,直接返回当前中断状态
    if (new_pin == _pin) {
        return interrupt_attached;
    }

    // 如果已经附加了中断,需要先分离
    if (interrupt_attached) {
        if (!hal.gpio->detach_interrupt(_pin)) {
            // 分离中断失败时发送警告消息
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "%s: Failed to detach interrupt from %d",
                          subsystem,
                          _pin);
        }
        interrupt_attached = false;
    }

    // 更新引脚编号
    _pin = new_pin;

    // 检查引脚编号是否有效
    if (_pin <= 0) {
        return false;
    }

    // 配置引脚为输入模式并安装中断处理程序
    // 在上升沿和下降沿都触发中断
    hal.gpio->pinMode(_pin, HAL_GPIO_INPUT);
    if (!hal.gpio->attach_interrupt(
            _pin,
            FUNCTOR_BIND_MEMBER(&AP_HAL::PWMSource::irq_handler,
                                void,
                                uint8_t,
                                bool,
                                uint32_t),
            AP_HAL::GPIO::INTERRUPT_BOTH)) {
        // 附加中断失败时发送警告消息
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "%s: Failed to attach interrupt to %d",
                      subsystem,
                      _pin);
        return false;
    }

    // 标记中断已成功附加
    interrupt_attached = true;
    return interrupt_attached;
}

// PWM中断处理函数
// a_pin: 触发中断的引脚
// pin_high: 引脚是否为高电平
// timestamp_us: 中断发生的时间戳(微秒)
void AP_HAL::PWMSource::irq_handler(uint8_t a_pin, bool pin_high, uint32_t timestamp_us)
{
    if (pin_high) {
        // 上升沿记录起始时间
        _pulse_start_us = timestamp_us;
        return;
    }
    // 如果没有有效的起始时间,说明可能丢失了中断
    if (_pulse_start_us == 0) {
        return;
    }
    // 计算脉冲宽度
    _irq_value_us = timestamp_us - _pulse_start_us;
    _pulse_start_us = 0;

    // 更新平均值计算所需的字段
    _irq_value_us_sum += _irq_value_us;
    _irq_value_us_count++;
}

// 获取最新的PWM脉冲宽度(微秒)
uint16_t AP_HAL::PWMSource::get_pwm_us()
{
    // 禁用中断并获取当前值
    void *irqstate = hal.scheduler->disable_interrupts_save();
    const uint32_t ret = _irq_value_us;
    _irq_value_us = 0;
    hal.scheduler->restore_interrupts(irqstate);

    return ret;
}

// 获取PWM脉冲宽度的平均值(微秒)
uint16_t AP_HAL::PWMSource::get_pwm_avg_us()
{
    // 禁用中断并计算平均值
    void *irqstate = hal.scheduler->disable_interrupts_save();
    uint32_t ret;
    if (_irq_value_us_count == 0) {
        ret = 0;
    } else {
        ret = _irq_value_us_sum / _irq_value_us_count;
    }
    // 重置累计值
    _irq_value_us_sum = 0;
    _irq_value_us_count = 0;
    hal.scheduler->restore_interrupts(irqstate);

    return ret;
}
