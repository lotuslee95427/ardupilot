/* 
   AP_Logger日志记录 - 基于文件的变体

   使用posix文件IO在指定目录下创建名为logNN.dat的日志文件
 */
#pragma once

#include <AP_Filesystem/AP_Filesystem.h>

#include <AP_HAL/utility/RingBuffer.h>
#include "AP_Logger_Backend.h"

#if HAL_LOGGING_FILESYSTEM_ENABLED

// 定义日志写入块大小,默认4096字节
#ifndef HAL_LOGGER_WRITE_CHUNK_SIZE
#define HAL_LOGGER_WRITE_CHUNK_SIZE 4096
#endif

// AP_Logger_File类继承自AP_Logger_Backend
class AP_Logger_File : public AP_Logger_Backend
{
public:
    // 构造函数
    AP_Logger_File(AP_Logger &front,
                   LoggerMessageWriter_DFLogStart *);

    // 探测函数,创建AP_Logger_File实例
    static AP_Logger_Backend  *probe(AP_Logger &front,
                                     LoggerMessageWriter_DFLogStart *ls) {
        return NEW_NOTHROW AP_Logger_File(front, ls);
    }

    // 初始化函数
    void Init() override;
    // 检查存储卡是否插入
    bool CardInserted(void) const override;

    // 擦除相关函数
    void EraseAll() override;

    /* 在当前偏移量写入数据块 */
    bool _WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical) override;
    // 获取可用缓冲区空间
    uint32_t bufferspace_available() override;

    // 高级接口函数
    uint16_t find_last_log() override;  // 查找最后一个日志
    void get_log_boundaries(uint16_t log_num, uint32_t & start_page, uint32_t & end_page) override;  // 获取日志边界
    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc) override;  // 获取日志信息
    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) override;  // 获取日志数据
    void end_log_transfer() override;  // 结束日志传输
    uint16_t get_num_logs() override;  // 获取日志数量
    void start_new_log(void) override;  // 开始新日志
    uint16_t find_oldest_log() override;  // 查找最早的日志

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    void flush(void) override;  // 刷新缓冲区
#endif
    void periodic_1Hz() override;  // 1Hz周期任务
    void periodic_fullrate() override;  // 全速率周期任务

    // 用于通过mavlink报告系统状态
    bool logging_failed() const override;

    // 检查日志是否已开始记录
    bool logging_started(void) const override { return _write_fd != -1; }
    void io_timer(void) override;  // IO定时器

protected:
    // 检查写入是否正常
    bool WritesOK() const override;
    // 检查是否可以开始新日志
    bool StartNewLogOK() const override;
    // 准备解锁开始记录日志
    void PrepForArming_start_logging() override;

private:
    int _write_fd = -1;  // 写文件描述符
    char *_write_filename;  // 写文件名
    bool last_log_is_marked_discard;  // 最后日志是否标记为丢弃
    uint32_t _last_write_ms;  // 最后写入时间
#if AP_RTC_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    bool _need_rtc_update;  // 是否需要更新RTC
#endif
    
    int _read_fd = -1;  // 读文件描述符
    uint16_t _read_fd_log_num;  // 读取的日志编号
    uint32_t _read_offset;  // 读偏移量
    uint32_t _write_offset;  // 写偏移量
    volatile uint32_t _open_error_ms;  // 打开错误时间
    const char *_log_directory;  // 日志目录
    bool _last_write_failed;  // 最后写入是否失败

    uint32_t _io_timer_heartbeat;  // IO定时器心跳
    bool io_thread_alive() const;  // 检查IO线程是否存活
    uint8_t io_thread_warning_decimation_counter;  // IO线程警告抑制计数器

    // 检查是否有最近的打开错误
    bool recent_open_error(void) const;

    // 可能耗时的准备工作
    void Prep_MinSpace();  // 准备最小空间
    int64_t disk_space_avail();  // 获取可用磁盘空间
    int64_t disk_space();  // 获取总磁盘空间

    void ensure_log_directory_exists();  // 确保日志目录存在

    bool file_exists(const char *filename) const;  // 检查文件是否存在
    bool log_exists(const uint16_t lognum) const;  // 检查日志是否存在

    bool dirent_to_log_num(const dirent *de, uint16_t &log_num) const;  // 目录项转换为日志编号
    bool write_lastlog_file(uint16_t log_num);  // 写入最后日志文件

    // 写缓冲区
    ByteBuffer _writebuf{0};  // 写缓冲区
    const uint16_t _writebuf_chunk = HAL_LOGGER_WRITE_CHUNK_SIZE;  // 写缓冲区块大小
    uint32_t _last_write_time;  // 最后写入时间

    /* 根据日志编号构造文件名。调用者必须释放内存 */
    char *_log_file_name(const uint16_t log_num) const;
    char *_lastlog_file_name() const;
    uint32_t _get_log_size(const uint16_t log_num);  // 获取日志大小
    uint32_t _get_log_time(const uint16_t log_num);  // 获取日志时间

    void stop_logging(void) override;  // 停止日志记录

    uint32_t last_messagewrite_message_sent;  // 最后消息写入时间

    // 空闲空间检查;在NuttX下填满SD卡会导致文件系统损坏,
    // 造成数据丢失、无法收集数据和启动失败
    uint32_t _free_space_last_check_time;  // 最后检查时间(毫秒)
    const uint32_t _free_space_check_interval = 1000UL;  // 检查间隔(毫秒)
    const uint32_t _free_space_min_avail = 8388608;  // 最小可用空间(字节)

    // 信号量用于访问环形缓冲区
    HAL_Semaphore semaphore;
    // write_fd_semaphore用于访问write_fd,使前端可以打开/关闭文件
    // 而不会导致后端写入错误的文件描述符
    HAL_Semaphore write_fd_semaphore;

    // 异步擦除状态
    struct {
        bool was_logging;  // 是否正在记录日志
        uint16_t log_num;  // 日志编号
    } erase;
    void erase_next(void);  // 擦除下一个

    const char *last_io_operation = "";  // 最后IO操作

    bool start_new_log_pending;  // 是否等待开始新日志
};

#endif // HAL_LOGGING_FILESYSTEM_ENABLED
