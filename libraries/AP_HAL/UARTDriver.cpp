/*
  实现通用的UARTDriver代码，包括端口锁定功能
 */
#include "AP_HAL.h"
#include <AP_Logger/AP_Logger.h>

// 以指定波特率、接收缓冲区大小和发送缓冲区大小初始化串口
void AP_HAL::UARTDriver::begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace)
{
    if (lock_write_key != 0) {
        // 如果端口已被锁定则静默失败
        return;
    }
    return _begin(baud, rxSpace, txSpace);
}

// 以指定波特率初始化串口,使用默认缓冲区大小
void AP_HAL::UARTDriver::begin(uint32_t baud)
{
    return begin(baud, 0, 0);
}

/*
  使用指定的写入密钥和读取密钥锁定串口,用于write_locked()和read_locked()函数的独占访问
 */
bool AP_HAL::UARTDriver::lock_port(uint32_t write_key, uint32_t read_key)
{
    if (lock_write_key != 0 && write_key != lock_write_key && write_key != 0) {
        // 其他进程正在使用写入锁
        return false;
    }
    if (lock_read_key != 0 && read_key != lock_read_key && read_key != 0) {
        // 其他进程正在使用读取锁
        return false;
    }
    lock_write_key = write_key;
    lock_read_key = read_key;
    return true;
}

// 使用密钥锁定并初始化串口
void AP_HAL::UARTDriver::begin_locked(uint32_t baud, uint16_t rxSpace, uint16_t txSpace, uint32_t key)
{
    if (lock_write_key != 0 && key != lock_write_key) {
        // 如果密钥不匹配则静默失败
        return;
    }
    return _begin(baud, rxSpace, txSpace);
}

/*
   向锁定的端口写入数据。如果端口已锁定且密钥不正确,则返回0并丢弃写入数据。
   所有写入操作都是非阻塞的。
*/
size_t AP_HAL::UARTDriver::write_locked(const uint8_t *buffer, size_t size, uint32_t key)
{
    if (lock_write_key != 0 && key != lock_write_key) {
        return 0;
    }
    return _write(buffer, size);
}

/*
   从锁定的端口读取数据。如果端口已锁定且密钥不正确则返回-1
*/
ssize_t AP_HAL::UARTDriver::read_locked(uint8_t *buf, size_t count, uint32_t key)
{
    if (lock_read_key != 0 && key != lock_read_key) {
        return 0;
    }
    ssize_t ret = _read(buf, count);
#if AP_UART_MONITOR_ENABLED
    auto monitor = _monitor_read_buffer;
    if (monitor != nullptr && ret > 0) {
        monitor->write(buf, ret);
    }
#endif
    return ret;
}

// 获取锁定端口中可读取的字节数
uint32_t AP_HAL::UARTDriver::available_locked(uint32_t key)
{
    if (lock_read_key != 0 && lock_read_key != key) {
        return 0;
    }
    return _available();
}

// 向未锁定的端口写入数据
size_t AP_HAL::UARTDriver::write(const uint8_t *buffer, size_t size)
{
    if (lock_write_key != 0) {
        return 0;
    }
    return _write(buffer, size);
}

// 写入单个字节
size_t AP_HAL::UARTDriver::write(uint8_t c)
{
    return write(&c, 1);
}

// 写入字符串
size_t AP_HAL::UARTDriver::write(const char *str)
{
    return write((const uint8_t *)str, strlen(str));
}

// 从未锁定的端口读取数据
ssize_t AP_HAL::UARTDriver::read(uint8_t *buffer, uint16_t count)
{
    return read_locked(buffer, count, 0);
}

// 读取单个字节
bool AP_HAL::UARTDriver::read(uint8_t &b)
{
    ssize_t n = read(&b, 1);
    return n > 0;
}

// 读取一个字节,如果没有数据则返回-1
int16_t AP_HAL::UARTDriver::read(void)
{
    uint8_t b;
    if (!read(b)) {
        return -1;
    }
    return b;
}

// 获取未锁定端口中可读取的字节数
uint32_t AP_HAL::UARTDriver::available()
{
    if (lock_read_key != 0) {
        return 0;
    }
    return _available();
}

// 关闭串口
void AP_HAL::UARTDriver::end()
{
    if (lock_read_key != 0 || lock_write_key != 0) {
        return;
    }
    _end();
}

// 刷新串口缓冲区
void AP_HAL::UARTDriver::flush()
{
    if (lock_read_key != 0 || lock_write_key != 0) {
        return;
    }
    _flush();
}

// 丢弃输入缓冲区中的所有数据
bool AP_HAL::UARTDriver::discard_input()
{
    if (lock_read_key != 0) {
        return false;
    }
    return _discard_input();
}

/*
  receive_time_constraint_us()的默认实现将用于未实现此调用的子类(如网络套接字)。
  由于我们不知道传输延迟,我们能做的最好的就是使用当前时间戳
 */
uint64_t AP_HAL::UARTDriver::receive_time_constraint_us(uint16_t nbytes)
{
    return AP_HAL::micros64();
}

// 根据传入的流控制设置检查流控制是否启用的辅助函数
bool AP_HAL::UARTDriver::flow_control_enabled(enum flow_control flow_control_setting) const
{
    switch(flow_control_setting) {
        case FLOW_CONTROL_ENABLE:
        case FLOW_CONTROL_AUTO:
            return true;
        case FLOW_CONTROL_DISABLE:
        case FLOW_CONTROL_RTS_DE:
            break;
    }
    return false;
}

// 获取校验位设置
uint8_t AP_HAL::UARTDriver::get_parity(void)
{
    return AP_HAL::UARTDriver::parity;
}

#if HAL_UART_STATS_ENABLED
// 获取累计字节数并返回自上次调用以来的变化
uint32_t AP_HAL::UARTDriver::StatsTracker::ByteTracker::update(uint32_t bytes)
{
    const uint32_t change = bytes - last_bytes;
    last_bytes = bytes;
    return change;
}

#if HAL_LOGGING_ENABLED
// 写入UART日志消息
void AP_HAL::UARTDriver::log_stats(const uint8_t inst, StatsTracker &stats, const uint32_t dt_ms)
{
    // 获取总计数
    const uint32_t total_tx_bytes = get_total_tx_bytes();
    const uint32_t total_rx_bytes = get_total_rx_bytes();

    // 如果从未看到数据则不记录日志
    if ((total_tx_bytes == 0) && (total_rx_bytes == 0)) {
        // 如果恰好在同一时刻tx和rx都归零,这可能会出错
        // 在这种极不可能的情况下,一条日志会被遗漏
        return;
    }

    // 更新跟踪数据
    const uint32_t tx_bytes = stats.tx.update(total_tx_bytes);
    const uint32_t rx_bytes = stats.rx.update(total_rx_bytes);

    // 组装结构体并记录日志
    struct log_UART pkt {
        LOG_PACKET_HEADER_INIT(LOG_UART_MSG),
        time_us  : AP_HAL::micros64(),
        instance : inst,
        tx_rate  : float((tx_bytes * 1000) / dt_ms),
        rx_rate  : float((rx_bytes * 1000) / dt_ms),
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}
#endif // HAL_LOGGING_ENABLED
#endif // HAL_UART_STATS_ENABLED
