// 防止头文件重复包含
#pragma once

#include <stdint.h>

#include "AP_HAL_Namespace.h"
#include "utility/BetterStream.h"
#include <AP_Logger/AP_Logger_config.h>

// 配置UART统计功能是否启用
#ifndef HAL_UART_STATS_ENABLED
#define HAL_UART_STATS_ENABLED !defined(HAL_NO_UARTDRIVER)
#endif

// 配置UART监控功能是否启用
#ifndef AP_UART_MONITOR_ENABLED
#define AP_UART_MONITOR_ENABLED 0
#endif

class ExpandingString;
class ByteBuffer;

/* UART驱动的纯虚基类 */
class AP_HAL::UARTDriver : public AP_HAL::BetterStream {
public:
    UARTDriver() {}
    /* 禁止拷贝构造 */
    CLASS_NO_COPY(UARTDriver);

    /*
      各个HAL需要实现受保护版本的这些调用: _begin(), _write(), _read()
      端口锁定由顶层AP_HAL函数检查
     */
    void begin(uint32_t baud);

    /*
      使用指定的最小发送和接收缓冲区大小初始化
     */
    void begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace);

    /*
      用于端口写入锁定时的初始化。注意这不会锁定端口,
      必须使用来自lock_port()的现有write_key
     */
    void begin_locked(uint32_t baud, uint16_t rxSpace, uint16_t txSpace, uint32_t write_key);

    /*
      单字节和多字节写入方法
     */
    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    size_t write(const char *str) override;

    /*
      单字节和多字节读取方法
     */
    int16_t read(void) override;
    bool read(uint8_t &b) override WARN_IF_UNUSED;
    ssize_t read(uint8_t *buffer, uint16_t count) override;
    
    // 结束串口通信
    void end();
    // 刷新缓冲区
    void flush();

    // 检查是否已初始化
    virtual bool is_initialized() = 0;
    // 检查是否有待发送数据
    virtual bool tx_pending() = 0;

    // 锁定端口以独占使用。使用key=0解锁
    bool lock_port(uint32_t write_key, uint32_t read_key);

    // 检查可读数据,如果没有返回0
    uint32_t available() override;
    uint32_t available_locked(uint32_t key);

    // 丢弃所有待处理的输入
    bool discard_input() override;

    // 向锁定的端口写入。如果端口已锁定且key不正确则返回0并丢弃写入
    size_t write_locked(const uint8_t *buffer, size_t size, uint32_t key);

    // 从锁定的端口读取。如果端口已锁定且key不正确则返回-1
    ssize_t read_locked(uint8_t *buf, size_t count, uint32_t key) WARN_IF_UNUSED;

    // 获取当前校验位设置(用于透传)
    uint8_t get_parity(void);
    
    // 控制可选功能
    virtual bool set_options(uint16_t options) { _last_options = options; return options==0; }
    virtual uint16_t get_options(void) const { return _last_options; }

    // UART可选功能位定义
    enum {
        OPTION_RXINV              = (1U<<0),  // 反转RX线
        OPTION_TXINV              = (1U<<1),  // 反转TX线
        OPTION_HDPLEX             = (1U<<2),  // 半双工(单线)模式
        OPTION_SWAP               = (1U<<3),  // 交换RX和TX引脚
        OPTION_PULLDOWN_RX        = (1U<<4),  // RX引脚下拉
        OPTION_PULLUP_RX          = (1U<<5),  // RX引脚上拉
        OPTION_PULLDOWN_TX        = (1U<<6),  // TX引脚下拉
        OPTION_PULLUP_TX          = (1U<<7),  // TX引脚上拉
        OPTION_NODMA_RX           = (1U<<8),  // 不使用DMA接收
        OPTION_NODMA_TX           = (1U<<9),  // 不使用DMA发送
        OPTION_MAVLINK_NO_FORWARD = (1U<<10), // 不转发此设备的MAVLink数据
        OPTION_NOFIFO             = (1U<<11), // 禁用硬件FIFO
        OPTION_NOSTREAMOVERRIDE   = (1U<<12), // 不允许GCS覆盖数据流速率
    };

    // 流控制模式枚举
    enum flow_control {
        FLOW_CONTROL_DISABLE=0,  // 禁用流控
        FLOW_CONTROL_ENABLE=1,   // 启用流控
        FLOW_CONTROL_AUTO=2,     // 自动流控
        FLOW_CONTROL_RTS_DE=3,   // RTS引脚用作驱动使能(用于RS-485)
    };
    
    // 设置流控制模式
    virtual void set_flow_control(enum flow_control flow_control_setting) {};
    // 获取当前流控制模式
    virtual enum flow_control get_flow_control(void) { return FLOW_CONTROL_DISABLE; }

    // 返回当前是否启用流控制
    bool flow_control_enabled() { return flow_control_enabled(get_flow_control()); }

    // 配置校验位
    virtual void configure_parity(uint8_t v){};
    // 设置停止位
    virtual void set_stop_bits(int n){};

    /* 无缓冲写入绕过环形缓冲区直接写入文件描述符 */
    virtual bool set_unbuffered_writes(bool on){ return false; };

    /*
      等待至少n字节的输入数据,超时时间为timeout_ms毫秒
      如果n字节可用返回true,超时返回false
     */
    virtual bool wait_timeout(uint16_t n, uint32_t timeout_ms) { return false; }

    /*
     * 可选的电机更新控制方法。如果HAL层需要,派生类可以实现它
     */
    virtual void _timer_tick(void) { }

    /*
      返回nbytes数据包开始到达uart的微秒级时间戳估计
      这应该被视为时间约束,而不是精确时间
      保证数据包不会在此时间之后开始接收,但可能在返回时间之前就在系统缓冲区中了
      
      这考虑了链路的波特率。对于没有波特率的传输(如USB)
      时间估计可能不太准确
     */
    virtual uint64_t receive_time_constraint_us(uint16_t nbytes);

    // 返回每秒字节带宽
    virtual uint32_t bw_in_bytes_per_second() const {
        return 5760;
    }

    // 获取波特率
    virtual uint32_t get_baud_rate() const { return 0; }

    /*
      如果此UART在RX和TX上都启用了DMA则返回true
     */
    virtual bool is_dma_enabled() const { return false; }

#if HAL_UART_STATS_ENABLED
    // 用于跟踪上次调用以来的数据使用情况的辅助类
    struct StatsTracker {
        class ByteTracker {
        public:
            // 获取累计字节数并返回自上次调用以来的变化
            uint32_t update(uint32_t bytes);
        private:
            uint32_t last_bytes;
        };
        ByteTracker tx;
        ByteTracker rx;
    };

    // 请求此uart的I/O信息,用于@SYS/uarts.txt
    virtual void uart_info(ExpandingString &str, StatsTracker &stats, const uint32_t dt_ms) {}

#if HAL_LOGGING_ENABLED
    // 记录此实例的统计信息
    void log_stats(const uint8_t inst, StatsTracker &stats, const uint32_t dt_ms);
#endif
#endif // HAL_UART_STATS_ENABLED

    /*
      如果可用,软件控制CTS/RTS引脚。不可用时返回false
     */
    virtual bool set_RTS_pin(bool high) { return false; };
    virtual bool set_CTS_pin(bool high) { return false; };

    // 返回USB端口请求的波特率
    virtual uint32_t get_usb_baud(void) const { return 0; }

    // 返回USB端口请求的校验位
    virtual uint8_t get_usb_parity(void) const { return parity; }

    // 禁用未使用uart的TX/RX引脚
    virtual void disable_rxtx(void) const {}

#if AP_UART_MONITOR_ENABLED
    // 一种监控UART所有读取的方法,将它们放入缓冲区
    // 由AP_Periph用于GPS调试
    bool set_monitor_read_buffer(ByteBuffer *buffer) {
        _monitor_read_buffer = buffer;
        return true;
    }
#endif

    /*
      如果端口当前被锁定用于写入则返回true
     */
    bool is_write_locked(void) const {
        return lock_write_key != 0;
    }

protected:
    // 锁定端口的密钥
    uint32_t lock_write_key;
    uint32_t lock_read_key;

    // 校验位
    uint8_t parity;

    /*
      后端begin方法
     */
    virtual void _begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) = 0;

    /*
      后端write方法
     */
    virtual size_t _write(const uint8_t *buffer, size_t size) = 0;

    /*
      后端read方法
     */
    virtual ssize_t _read(uint8_t *buffer, uint16_t count)  WARN_IF_UNUSED = 0;

    /*
      结束端口控制,释放缓冲区
     */
    virtual void _end() = 0;

    /*
      刷新所有待处理数据
     */
    virtual void _flush() = 0;

    // 检查端口可用数据
    virtual uint32_t _available() = 0;

    // 丢弃端口上的输入数据
    virtual bool _discard_input(void) = 0;

    // 辅助函数,检查给定设置下流控制是否启用
    bool flow_control_enabled(enum flow_control flow_control_setting) const;

#if HAL_UART_STATS_ENABLED
    // 获取累计发送和接收字节数
    virtual uint32_t get_total_tx_bytes() const { return 0; }
    virtual uint32_t get_total_rx_bytes() const { return 0; }
#endif

private:
    // 端口选项位
    uint16_t _last_options;

#if AP_UART_MONITOR_ENABLED
    // 监控读取缓冲区
    ByteBuffer *_monitor_read_buffer;
#endif
};
