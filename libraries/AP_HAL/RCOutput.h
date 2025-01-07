#pragma once

#include "AP_HAL_Namespace.h"
#include <stdint.h>

// 定义PWM输出的最小和最大脉宽(单位:微秒)
#define RC_OUTPUT_MIN_PULSEWIDTH 400
#define RC_OUTPUT_MAX_PULSEWIDTH 2100

/* 如果没有定义通道名称,则从1开始定义通道索引 */
#ifndef CH_1
#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7
#define CH_9 8
#define CH_10 9
#define CH_11 10
#define CH_12 11
#define CH_13 12
#define CH_14 13
#define CH_15 14
#define CH_16 15
#define CH_17 16
#define CH_18 17
#define CH_19 18
#define CH_20 19
#define CH_21 20
#define CH_22 21
#define CH_23 22
#define CH_24 23
#define CH_25 24
#define CH_26 25
#define CH_27 26
#define CH_28 27
#define CH_29 28
#define CH_30 29
#define CH_31 30
#define CH_32 31
#define CH_NONE 255  // 表示无效通道
#endif

class ByteBuffer;

class ExpandingString;

// RCOutput类定义了遥控输出的抽象接口
class AP_HAL::RCOutput {
public:
    // 初始化RCOutput
    virtual void init() = 0;

    /* 输出频率控制 */
    // 设置指定通道掩码的输出频率
    virtual void     set_freq(uint32_t chmask, uint16_t freq_hz) = 0;
    // 获取指定通道的输出频率
    virtual uint16_t get_freq(uint8_t chan) = 0;

    /* 输出通道使能/禁用控制 */
    // 使能单个通道
    virtual void     enable_ch(uint8_t chan) = 0;
    // 禁用单个通道
    virtual void     disable_ch(uint8_t chan) = 0;

    /*
     * 输出单个通道的PWM值
     * 如果之前调用了cork(),则会与之前的写入一起分组
     */
    virtual void     write(uint8_t chan, uint16_t period_us) = 0;

    /*
     * 标记通道掩码中的通道为可反转
     * 这对某些ESC类型(如DShot)是必需的,以便正确执行输出缩放
     * 传入的通道掩码会与现有掩码进行OR运算
     * 掩码使用舵机通道编号
     */
    virtual void     set_reversible_mask(uint32_t chanmask) {}
    
    /*
     * 标记通道掩码中的通道为已反转
     * 传入的通道掩码会与现有掩码进行OR运算
     * 掩码使用舵机通道编号
     */
    virtual void     set_reversed_mask(uint32_t chanmask) {}
    virtual uint32_t get_reversed_mask() { return 0; }

    /*
     * 以1Hz的频率更新通道掩码,允许发送如dshot命令等
     */
    virtual void     update_channel_masks() {}

    /*
     * 允许临时暂停通道掩码更新
     */
    virtual void     disable_channel_mask_updates() {}
    virtual void     enable_channel_mask_updates() {}

    /*
     * 延迟后续write()调用写入底层硬件
     * 以便将相关写入分组在一起
     * 当所有需要的写入完成后,调用push()提交更改
     */
    virtual void     cork() = 0;

    /*
     * 将挂起的更改推送到底层硬件
     * cork()和push()之间的所有更改都会在一个事务中一起推送
     */
    virtual void     push() = 0;

    /* 读取当前输出状态,可以是单个通道或通道数组
     * 在具有独立IO控制器的板子上,
     * 这返回IO控制器报告的最新输出值 */
    virtual uint16_t read(uint8_t chan) = 0;
    virtual void     read(uint16_t* period_us, uint8_t len) = 0;

    /* 读取当前输入状态。返回最后写入的值 */
    virtual uint16_t read_last_sent(uint8_t chan) { return read(chan); }
    virtual void     read_last_sent(uint16_t* period_us, uint8_t len) { read(period_us, len); };

    /*
      设置FMU固件死机时发送到一组通道的PWM值
     */
    virtual void     set_failsafe_pwm(uint32_t chmask, uint16_t period_us) {}

    /*
      强制打开安全开关,禁用IO板的PWM输出
      默认返回false(表示失败),这样没有安全开关的板子就不需要实现此方法
     */
    virtual bool     force_safety_on(void) { return false; }

    /*
      强制关闭安全开关,使能IO板的PWM输出
     */
    virtual void     force_safety_off(void) {}

    /*
      为可以输出功率百分比的ESC(如UAVCAN ESC)设置ESC输出缩放
      值以微秒为单位,表示最小和最大PWM值,
      用于将通道写入转换为百分比
     */
    void set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm) {
        _esc_pwm_min = min_pwm;
        _esc_pwm_max = max_pwm;
    }

    /*
      根据set_esc_scaling设置的范围,将pwm值缩放到[-1;1]范围,不受约束
     */
    float scale_esc_to_unity(uint16_t pwm) const;

    /*
      如果可用,返回通道的erpm和错误率
     */
    virtual uint16_t get_erpm(uint8_t chan) const { return 0; }
    virtual float get_erpm_error_rate(uint8_t chan) const { return 100.0f; }
    /*
      允许读取所有erpm值并检测新更新 - 主要用于IOMCU
     */
    virtual bool  new_erpm() { return false; }
    virtual uint32_t  read_erpm(uint16_t* erpm, uint8_t len) { return 0; }

    /*
      以给定速率使能PX4IO SBUS输出
     */
    virtual bool enable_px4io_sbus_out(uint16_t rate_hz) { return false; }

    /*
     * 电机更新的可选方法。如果HAL层需要,
     * 派生类可以实现它
     */
    virtual void timer_tick(void) { }

    /*
      使用给定波特率设置串行输出到ESC
      假设1个起始位,1个停止位,LSB优先和8个数据位
      用于ESC配置和固件刷新的直通模式

      当串行输出激活时,chanmask中所有通道的正常输出将暂停
      根据实现情况,其他通道(如同一通道定时器组中的通道)的输出也可能停止
     */
    virtual bool serial_setup_output(uint8_t chan, uint32_t baudrate, uint32_t chanmask) { return false; }

    /*
      使用serial_setup_output的设置向ESC写入一组字节
      这是一个阻塞调用
     */
    virtual bool serial_write_bytes(const uint8_t *bytes, uint16_t len) { return false; }

    /*
      使用serial_setup_output()的串行参数从端口读取一系列字节
      返回读取的字节数。这是一个阻塞调用
     */
    virtual uint16_t serial_read_bytes(uint8_t *buf, uint16_t len) { return 0; }
    
    /*
      停止串行输出。这会恢复通道和其他被serial_setup_output()
      停止的通道的先前输出模式
     */
    virtual void serial_end(void) {}
    
    /*
      输出模式。支持PWM、oneshot和dshot
    */
    // 此枚举被BLH_OTYPE和AP_Periph上的ESC_PWM_TYPE使用
    // 更改时请仔细检查参数是否正确
    enum output_mode {
        MODE_PWM_NONE,        // 无PWM输出
        MODE_PWM_NORMAL,      // 普通PWM模式
        MODE_PWM_ONESHOT,     // OneShot PWM模式
        MODE_PWM_ONESHOT125,  // OneShot125 PWM模式
        MODE_PWM_BRUSHED,     // 有刷电机PWM模式
        MODE_PWM_DSHOT150,    // DShot150协议
        MODE_PWM_DSHOT300,    // DShot300协议
        MODE_PWM_DSHOT600,    // DShot600协议
        MODE_PWM_DSHOT1200,   // DShot1200协议
        MODE_NEOPIXEL,        // 与800kHz的MODE_PWM_DSHOT相同,但用于LED
        MODE_PROFILED,        // 与MODE_PWM_DSHOT相同,使用独立时钟和数据
        MODE_NEOPIXELRGB,     // 与MODE_NEOPIXEL相同但使用RGB顺序
    };
    // 判断输出模式是否为dshot协议
    // 静态方法以允许在ChibiOS线程中使用
    static bool is_dshot_protocol(const enum output_mode mode);

    // 判断输出模式是否为LED协议
    static bool is_led_protocol(const enum output_mode mode) {
      switch (mode) {
      case MODE_NEOPIXEL:
      case MODE_NEOPIXELRGB:
      case MODE_PROFILED:
        return true;
      default:
        return false;
      }
    }

    // BLHeli32: https://github.com/bitdump/BLHeli/blob/master/BLHeli_32%20ARM/BLHeli_32%20Firmware%20specs/Digital_Cmd_Spec.txt
    // BLHeli_S: https://github.com/bitdump/BLHeli/blob/master/BLHeli_S%20SiLabs/Dshotprog%20spec%20BLHeli_S.txt
    // DShot命令枚举
    enum BLHeliDshotCommand : uint8_t {
      DSHOT_RESET = 0,                           // 重置ESC
      DSHOT_BEEP1 = 1,                          // 蜂鸣器音调1
      DSHOT_BEEP2 = 2,                          // 蜂鸣器音调2
      DSHOT_BEEP3 = 3,                          // 蜂鸣器音调3
      DSHOT_BEEP4 = 4,                          // 蜂鸣器音调4
      DSHOT_BEEP5 = 5,                          // 蜂鸣器音调5
      DSHOT_ESC_INFO = 6,                       // 请求ESC信息
      DSHOT_ROTATE = 7,                         // 电机正转
      DSHOT_ROTATE_ALTERNATE = 8,               // 电机反转
      DSHOT_3D_OFF = 9,                         // 关闭3D模式
      DSHOT_3D_ON = 10,                         // 开启3D模式
      DSHOT_SAVE = 12,                          // 保存设置
      DSHOT_EXTENDED_TELEMETRY_ENABLE = 13,     // 启用扩展遥测
      DSHOT_EXTENDED_TELEMETRY_DISABLE = 14,    // 禁用扩展遥测
      DSHOT_NORMAL = 20,                        // 正常运行模式
      DSHOT_REVERSE = 21,                       // 反向运行模式
      // 以下选项仅在BLHeli32上可用
      DSHOT_LED0_ON = 22,                       // 打开LED0
      DSHOT_LED1_ON = 23,                       // 打开LED1
      DSHOT_LED2_ON = 24,                       // 打开LED2
      DSHOT_LED3_ON = 25,                       // 打开LED3
      DSHOT_LED0_OFF = 26,                      // 关闭LED0
      DSHOT_LED1_OFF = 27,                      // 关闭LED1
      DSHOT_LED2_OFF = 28,                      // 关闭LED2
      DSHOT_LED3_OFF = 29,                      // 关闭LED3
    };

    // DShot零油门值
    const uint8_t DSHOT_ZERO_THROTTLE = 48;

    // ESC类型枚举
    enum DshotEscType {
      DSHOT_ESC_NONE = 0,         // 无ESC
      DSHOT_ESC_BLHELI = 1,       // BLHeli ESC
      DSHOT_ESC_BLHELI_S = 2,     // BLHeli_S ESC
      DSHOT_ESC_BLHELI_EDT = 3,   // BLHeli EDT ESC
      DSHOT_ESC_BLHELI_EDT_S = 4  // BLHeli EDT_S ESC
    };

    // 设置输出模式
    virtual void    set_output_mode(uint32_t mask, enum output_mode mode) {}

    // 获取输出模式
    virtual enum output_mode get_output_mode(uint32_t& mask) {
      mask = 0;
      return MODE_PWM_NORMAL;
    }

    /*
     * 获取输出模式横幅以通知用户输出如何配置
     */
    virtual bool get_output_mode_banner(char banner_msg[], uint8_t banner_msg_len) const { return false; }

    /*
     * 返回必须禁用的通道掩码,因为它们与数字通道共享一个组
     */
    virtual uint32_t get_disabled_channels(uint32_t digital_mask) { return 0; }

    /*
      设置默认更新率
     */
    virtual void    set_default_rate(uint16_t rate_hz) {}

    /*
      为通道掩码启用遥测请求
      这用于DShot获取遥测反馈
     */
    virtual void set_telem_request_mask(uint32_t mask) {}

    /*
      为通道掩码启用双向遥测请求
      这用于DShot获取遥测反馈
     */
    virtual void set_bidir_dshot_mask(uint32_t mask) {}

    /*
      标记ESC为活动状态以发送dshot命令
     */
    virtual void set_active_escs_mask(uint32_t mask) {}

    /*
      将dshot速率设置为循环速率的倍数
     */
    virtual void set_dshot_rate(uint8_t dshot_rate, uint16_t loop_rate_hz) {}

    /*
      设置dshot周期(微秒),仅供IOMCU使用
     */
    virtual void set_dshot_period(uint32_t period_us, uint8_t dshot_rate) {}
    virtual uint32_t get_dshot_period_us() const { return 0; }

    /*
      设置dshot ESC类型
     */
    virtual void set_dshot_esc_type(DshotEscType esc_type) {}

    virtual DshotEscType get_dshot_esc_type() const { return DSHOT_ESC_NONE; }

    // 表示所有通道的常量
    const static uint32_t ALL_CHANNELS = 255;
    /*
      发送dshot命令,如果命令超时为0则发送10次命令
      chan是要发送命令的舵机通道
     */
    virtual void send_dshot_command(uint8_t command, uint8_t chan = ALL_CHANNELS, uint32_t command_timeout_ms = 0, uint16_t repeat_count = 10, bool priority = false) {}

    /*
      设置用于rpm计算的电机极数
     */
    virtual void set_motor_poles(uint8_t poles) {}

    /*
      为给定通道号设置串行LED输出,
      链中LED的最大数量为给定值
     */
    virtual bool set_serial_led_num_LEDs(const uint16_t chan, uint8_t num_leds, output_mode mode = MODE_PWM_NONE, uint32_t clock_mask = 0) { return false; }

    /*
      为给定输出通道和LED编号设置串行LED输出数据
      LED编号为-1表示所有LED。LED 0是第一个LED
     */
    virtual bool set_serial_led_rgb_data(const uint16_t chan, int8_t led, uint8_t red, uint8_t green, uint8_t blue) { return false; }

    /*
      触发发送串行LED
     */
    virtual bool serial_led_send(const uint16_t chan) { return false; }

    // 输出定时器信息
    virtual void timer_info(ExpandingString &str) {}

    /*
      此驱动程序是否可以同时处理gpio和RC
    */
    virtual bool supports_gpio() { return false; };

    /*
      向通道写入gpio状态
    */
    virtual void write_gpio(uint8_t chan, bool active) {};

    /*
     * 计算达到目标比特率所需的预分频器
     */
    static uint32_t calculate_bitrate_prescaler(uint32_t timer_clock, uint32_t target_frequency, bool at_least_freq = false);

    /*
     * 不同协议的位宽值
     */
    /*
     * ESC对DSHOT占空比似乎很敏感
     * 选项是(ticks, percentage):
     * 20/7/14, 35/70
     * 11/4/8, 36/72
     * 8/3/6, 37/75 <-- 这是首选占空比,在网上有一些支持
     */
    // 位宽: 8/3/6 == 37%/75%
    static constexpr uint32_t DSHOT_BIT_WIDTH_TICKS_DEFAULT = 8;
    static constexpr uint32_t DSHOT_BIT_0_TICKS_DEFAULT = 3;
    static constexpr uint32_t DSHOT_BIT_1_TICKS_DEFAULT = 6;
    // 位宽: 11/4/8 == 36%/72%
    static constexpr uint32_t DSHOT_BIT_WIDTH_TICKS_S = 11;
    static constexpr uint32_t DSHOT_BIT_0_TICKS_S = 4;
    static constexpr uint32_t DSHOT_BIT_1_TICKS_S = 8;

    static uint32_t DSHOT_BIT_WIDTH_TICKS;
    static uint32_t DSHOT_BIT_0_TICKS;
    static uint32_t DSHOT_BIT_1_TICKS;

    // 参见WS2812B规格中的预期脉冲宽度
    static constexpr uint32_t NEOP_BIT_WIDTH_TICKS = 8;
    static constexpr uint32_t NEOP_BIT_0_TICKS = 2;
    static constexpr uint32_t NEOP_BIT_1_TICKS = 6;
    // neopixel完全不使用脉冲宽度
    static constexpr uint32_t PROFI_BIT_0_TICKS = 7;
    static constexpr uint32_t PROFI_BIT_1_TICKS = 14;
    static constexpr uint32_t PROFI_BIT_WIDTH_TICKS = 20;

    // 足够长的LED输出周期以支持高LED数量
    static constexpr uint32_t LED_OUTPUT_PERIOD_US = 10000;

protected:

    // get_output_mode_banner实现的辅助函数
    void append_to_banner(char banner_msg[], uint8_t banner_msg_len, output_mode out_mode, uint8_t low_ch, uint8_t high_ch) const;
    const char* get_output_mode_string(enum output_mode out_mode) const;

    // ESC PWM的最小和最大值
    uint16_t _esc_pwm_min = 1000;
    uint16_t _esc_pwm_max = 2000;
};
