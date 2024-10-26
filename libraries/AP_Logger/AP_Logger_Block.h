/*
   AP_Logger日志记录 - 基于块的变体
 */
#pragma once

#include "AP_Logger_Backend.h"

#if HAL_LOGGING_BLOCK_ENABLED

#define BLOCK_LOG_VALIDATE 0

// 基于块的日志记录器类,继承自AP_Logger_Backend
class AP_Logger_Block : public AP_Logger_Backend {
public:
    // 构造函数
    AP_Logger_Block(AP_Logger &front, LoggerMessageWriter_DFLogStart *writer);

    // 初始化函数
    virtual void Init(void) override;
    // 检查存储卡是否插入
    virtual bool CardInserted(void) const override = 0;

    // 擦除相关函数
    void EraseAll() override;  // 擦除所有数据

    // 高级接口函数
    uint16_t find_last_log() override;  // 查找最后一个日志
    void get_log_boundaries(uint16_t list_entry, uint32_t & start_page, uint32_t & end_page) override;  // 获取日志边界
    void get_log_info(uint16_t list_entry, uint32_t &size, uint32_t &time_utc) override;  // 获取日志信息
    int16_t get_log_data(uint16_t list_entry, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) override WARN_IF_UNUSED;  // 获取日志数据
    void end_log_transfer() override { }  // 结束日志传输
    uint16_t get_num_logs() override;  // 获取日志数量
    void start_new_log(void) override;  // 开始新日志
    uint32_t bufferspace_available() override;  // 获取可用缓冲区空间
    void stop_logging(void) override;  // 停止日志记录
    void stop_logging_async(void) override;  // 异步停止日志记录
    bool logging_failed() const override;  // 检查日志记录是否失败
    bool logging_started(void) const override { return log_write_started; }  // 检查日志记录是否已开始
    void io_timer(void) override;  // IO定时器

protected:
    /* 在当前偏移量写入数据块 */
    bool _WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical) override;
    void periodic_1Hz() override;  // 1Hz周期任务
    void periodic_10Hz(const uint32_t now) override;  // 10Hz周期任务
    bool WritesOK() const override;  // 检查写入是否正常

    // 从当前页获取扇区号
    uint32_t get_sector(uint32_t current_page) const {
        return ((current_page - 1) / df_PagePerSector);
    }

    // 从当前页获取块号
    uint32_t get_block(uint32_t current_page) const {
        return ((current_page - 1) / df_PagePerBlock);
    }

    uint32_t df_PageSize;  // 页大小(字节)
    uint16_t df_PagePerBlock;  // 每块的页数(通常64k)
    uint16_t df_PagePerSector;  // 每扇区的页数(通常4k)
    uint32_t df_NumPages;  // 芯片总页数
    volatile bool log_write_started;  // 日志写入是否已开始

    uint8_t *buffer;  // 数据缓冲区
    uint32_t last_messagewrite_message_sent;  // 最后发送消息写入消息的时间
    uint32_t df_Read_PageAdr;  // 当前读取页地址

private:
    /*
      由板级后端实现的函数
     */
    virtual void BufferToPage(uint32_t PageAdr) = 0;  // 缓冲区写入页
    virtual void PageToBuffer(uint32_t PageAdr) = 0;  // 页读入缓冲区
    virtual void SectorErase(uint32_t SectorAdr) = 0;  // 扇区擦除
    virtual void Sector4kErase(uint32_t SectorAdr) = 0;  // 4k扇区擦除
    virtual void StartErase() = 0;  // 开始擦除
    virtual bool InErase() = 0;  // 检查是否在擦除
    void         flash_test(void);  // Flash测试

    // 页头结构
    struct PACKED PageHeader {
        uint32_t FilePage;  // 文件页号
        uint16_t FileNumber;  // 文件号
#if BLOCK_LOG_VALIDATE
        uint32_t crc;  // CRC校验
#endif
    };

    // 文件头结构
    struct PACKED FileHeader {
        uint32_t utc_secs;  // UTC时间戳
    };

    HAL_Semaphore sem;  // 芯片访问信号量
    HAL_Semaphore write_sem;  // 环形缓冲区访问信号量
    ByteBuffer writebuf;  // 写入缓冲区

    // 状态变量
    uint16_t df_Read_BufferIdx;  // 读缓冲区索引
    uint32_t df_PageAdr;  // 当前写入页地址
    uint16_t df_FileNumber;  // 当前文件号
    uint16_t df_Write_FileNumber;  // 写入文件号
    uint32_t df_FileTime;  // 文件时间戳
    uint32_t df_FilePage;  // 当前读/写文件的相对页索引(从1开始)
    uint32_t df_Write_FilePage;  // 写入文件页
    uint32_t df_EraseFrom;  // 发生损坏时擦除起始页

    bool adding_fmt_headers;  // 是否正在添加FMT消息头

    volatile bool erase_started;  // 是否正在等待擦除完成
    volatile bool new_log_pending;  // 擦除前是否正在记录日志
    volatile bool stop_log_pending;  // 是否请求安全停止日志
    volatile bool chip_full;  // 芯片是否已满(仅输出一次完整消息)
    volatile uint32_t io_timer_heartbeat;  // IO线程心跳
    uint8_t warning_decimation_counter;  // 警告抑制计数器

    // 状态消息枚举
    volatile enum class StatusMessage {
        NONE,  // 无消息
        ERASE_COMPLETE,  // 擦除完成
        RECOVERY_COMPLETE,  // 恢复完成
    } status_msg;

    // 读取指定大小的数据到页。调用者必须确保数据在页内,否则会回绕到页首
    bool BlockRead(uint16_t IntPageAdr, void *pBuffer, uint16_t size);

    // 擦除相关函数
    bool NeedErase(void);  // 检查是否需要擦除
    void validate_log_structure();  // 验证日志结构

    // 内部高级函数
    int16_t get_log_data_raw(uint16_t log_num, uint32_t page, uint32_t offset, uint16_t len, uint8_t *data) WARN_IF_UNUSED;
    uint16_t StartRead(uint32_t PageAdr);  // 开始读取指定页地址并返回文件号
    uint16_t ReadHeaders();  // 读取当前读取点的头信息并返回文件号
    uint32_t find_last_page(void);  // 查找最后一页
    uint32_t find_last_page_of_log(uint16_t log_number);  // 查找指定日志的最后一页
    bool is_wrapped(void);  // 检查是否已回绕
    void StartWrite(uint32_t PageAdr);  // 开始写入
    void FinishWrite(void);  // 完成写入

    // 读取方法
    bool ReadBlock(void *pBuffer, uint16_t size);  // 读取数据块

    void StartLogFile(uint16_t FileNumber);  // 开始新日志文件
    uint16_t GetFileNumber() const;  // 获取文件号

    void _print_log_formats(AP_HAL::BetterStream *port);  // 打印日志格式

    // IO线程回调
    bool io_thread_alive() const;  // 检查IO线程是否存活
    void write_log_page();  // 写入日志页
};

#endif  // HAL_LOGGING_BLOCK_ENABLED
