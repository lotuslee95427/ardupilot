#pragma once

#include <stdint.h>

#include "AP_HAL_Namespace.h"

// GPIO输入模式定义
#define HAL_GPIO_INPUT  0
// GPIO输出模式定义  
#define HAL_GPIO_OUTPUT 1
// GPIO复用功能模式定义
#define HAL_GPIO_ALT    2

// 数字IO源类,提供GPIO基本操作接口
class AP_HAL::DigitalSource {
public:
    // 设置引脚模式
    virtual void    mode(uint8_t output) = 0;
    // 读取引脚状态
    virtual uint8_t read() = 0;
    // 写入引脚状态
    virtual void    write(uint8_t value) = 0;
    // 翻转引脚状态
    virtual void    toggle() = 0;
};

// PWM输入源类,用于读取PWM信号
class AP_HAL::PWMSource {
public:
    // 析构函数,解除中断绑定
    ~PWMSource();

    // 设置PWM输入引脚
    bool set_pin(int16_t new_pin, const char *subsystem);
    // 获取当前PWM输入引脚编号
    int16_t pin() const { return _pin; }  

    // 获取最近一次测量的PWM输入值(微秒)
    uint16_t get_pwm_us();            
    // 获取自上次调用以来的平均PWM值(微秒)
    uint16_t get_pwm_avg_us();        

private:
    uint16_t _irq_value_us;         // 最近一次计算的PWM值(中断副本)
    uint32_t _pulse_start_us;       // 脉冲开始时的系统时间
    int16_t _pin = -1;              // PWM输入引脚编号

    uint32_t _irq_value_us_sum;     // PWM值累加,用于计算平均值
    uint32_t _irq_value_us_count;   // PWM值计数,用于计算平均值

    bool interrupt_attached;         // 中断是否已绑定标志

    // PWM输入中断处理函数
    void irq_handler(uint8_t pin,
                     bool pin_state,
                     uint32_t timestamp);
};

// GPIO管理类,提供GPIO相关功能接口
class AP_HAL::GPIO {
public:
    GPIO() {}
    // GPIO初始化
    virtual void    init() = 0;
    // 设置引脚模式
    virtual void    pinMode(uint8_t pin, uint8_t output) = 0;

    // 设置引脚模式(带复用功能),部分板子支持
    virtual void    pinMode(uint8_t pin, uint8_t output, uint8_t alt) {};

    // 读取引脚状态
    virtual uint8_t read(uint8_t pin) = 0;
    // 写入引脚状态
    virtual void    write(uint8_t pin, uint8_t value) = 0;
    // 翻转引脚状态
    virtual void    toggle(uint8_t pin) = 0;
    // 检查引脚是否有效
    virtual bool    valid_pin(uint8_t pin) const { return true; }

    // 获取GPIO引脚对应的舵机通道,成功返回true并填充servo_ch参数
    // servo_ch使用从0开始的索引
    virtual bool    pin_to_servo_channel(uint8_t pin, uint8_t& servo_ch) const { return false; }

    // 用于保存和恢复引脚设置
    virtual bool    get_mode(uint8_t pin, uint32_t &mode) { return false; }
    virtual void    set_mode(uint8_t pin, uint32_t mode) {}

    /* 替代接口: */
    virtual AP_HAL::DigitalSource* channel(uint16_t n) = 0;

    // 中断触发类型枚举
    enum INTERRUPT_TRIGGER_TYPE {
        INTERRUPT_NONE,    // 无中断
        INTERRUPT_FALLING, // 下降沿触发
        INTERRUPT_RISING,  // 上升沿触发
        INTERRUPT_BOTH,    // 双边沿触发
    };

    /* 中断接口: */
    //                                返回值, 引脚  , 状态, 时间戳
    // 参数说明:
    //    返回值表示函数必须返回void
    //    pin是触发中断的引脚
    //    state是引脚的新状态
    //    timestamp是中断发生时的微秒时间戳
    FUNCTOR_TYPEDEF(irq_handler_fn_t, void, uint8_t, bool, uint32_t);
    
    // 绑定中断处理函数(使用函数对象)
    virtual bool    attach_interrupt(uint8_t pin,
                                     irq_handler_fn_t fn,
                                     INTERRUPT_TRIGGER_TYPE mode) {
        return false;
    }

    // 绑定中断处理函数(使用普通函数指针)
    virtual bool    attach_interrupt(uint8_t pin,
                                     AP_HAL::Proc proc,
                                     INTERRUPT_TRIGGER_TYPE mode) {
        return false;
    }
    
    // 解除引脚中断绑定
    bool detach_interrupt(uint8_t pin) {
        if (attach_interrupt(pin, (irq_handler_fn_t)nullptr, AP_HAL::GPIO::INTERRUPT_NONE)) {
            return true;
        }
        return attach_interrupt(pin, (AP_HAL::Proc)nullptr, AP_HAL::GPIO::INTERRUPT_NONE);
    }

    /*
      阻塞等待引脚状态改变。timeout_us为0表示永久等待
      引脚状态改变时返回true,超时返回false
     */
    virtual bool wait_pin(uint8_t pin, INTERRUPT_TRIGGER_TYPE mode, uint32_t timeout_us) { return false; }

    /* 返回USB是否已连接 */
    virtual bool    usb_connected(void) = 0;

    // 可选的定时器滴答函数
    virtual void timer_tick(void) {};

    // 运行解锁检查
    virtual bool arming_checks(size_t buflen, char *buffer) const { return true; }

};
