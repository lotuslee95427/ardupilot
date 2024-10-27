/* 
   AP_Logger日志记录 - MAVLink变体

   - 使用MAVLink协议将打开的日志文件以数据块形式传输给客户端
 */
#pragma once

#include "AP_Logger_Backend.h"

#if HAL_LOGGING_MAVLINK_ENABLED

#include <AP_HAL/Semaphores.h>

// 是否禁用中断
#define DF_MAVLINK_DISABLE_INTERRUPTS 0

// AP_Logger_MAVLink类继承自AP_Logger_Backend,实现通过MAVLink传输日志
class AP_Logger_MAVLink : public AP_Logger_Backend
{
public:
    // 构造函数
    AP_Logger_MAVLink(class AP_Logger &front, LoggerMessageWriter_DFLogStart *writer);

    // 探测函数,创建AP_Logger_MAVLink实例
    static AP_Logger_Backend  *probe(AP_Logger &front,
                                     LoggerMessageWriter_DFLogStart *ls) {
        return NEW_NOTHROW AP_Logger_MAVLink(front, ls);
    }

    // 初始化函数
    void Init() override;

    // 在客户端连接前丢弃所有数据
    // 这会阻止车辆调用start_new_log
    bool logging_started() const override { return _initialised; }

    // 停止日志记录
    void stop_logging() override;

    /* 在当前偏移量写入数据块 */
    bool _WritePrioritisedBlock(const void *pBuffer, uint16_t size,
                               bool is_critical) override;

    // 初始化相关函数
    bool CardInserted(void) const override { return true; }

    // 擦除相关函数
    void EraseAll() override {}

    // 准备解锁
    void PrepForArming() override {}

    // 高级接口函数
    uint16_t find_last_log(void) override { return 0; }  // 查找最后一个日志
    void get_log_boundaries(uint16_t log_num, uint32_t & start_page, uint32_t & end_page) override {}  // 获取日志边界
    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc) override {}  // 获取日志信息
    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) override { return 0; }  // 获取日志数据
    void end_log_transfer() override { };  // 结束日志传输
    uint16_t get_num_logs(void) override { return 0; }  // 获取日志数量

    // 处理远程日志块状态消息
    void remote_log_block_status_msg(const GCS_MAVLINK &link, const mavlink_message_t& msg) override;
    // 处理解锁事件
    void vehicle_was_disarmed() override {}

protected:
    // 推送日志块
    void push_log_blocks() override;
    // 检查是否可以写入
    bool WritesOK() const override;

private:
    // 日志数据块结构体
    struct dm_block {
        uint32_t seqno;  // 序列号
        uint8_t buf[MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN];  // 数据缓冲区
        uint32_t last_sent;  // 最后发送时间
        struct dm_block *next;  // 下一个块指针
    };

    // 发送日志块
    bool send_log_block(struct dm_block &block);
    // 处理确认消息
    void handle_ack(const GCS_MAVLINK &link, const mavlink_message_t &msg, uint32_t seqno);
    // 处理重试
    void handle_retry(uint32_t block_num);
    // 执行重发
    void do_resends(uint32_t now);
    // 释放所有块
    void free_all_blocks();

    // 块队列结构体,用于管理待发送、已发送、重试等队列
    struct dm_block_queue {
        uint32_t sent_count;  // 已发送计数
        struct dm_block *oldest;  // 最老的块
        struct dm_block *youngest;  // 最新的块
    };
    typedef struct dm_block_queue dm_block_queue_t;

    // 队列操作函数
    void enqueue_block(dm_block_queue_t &queue, struct dm_block *block);  // 入队
    bool queue_has_block(dm_block_queue_t &queue, struct dm_block *block);  // 检查块是否在队列中
    struct dm_block *dequeue_seqno(dm_block_queue_t &queue, uint32_t seqno);  // 按序列号出队
    bool free_seqno_from_queue(uint32_t seqno, dm_block_queue_t &queue);  // 从队列中释放指定序列号的块
    bool send_log_blocks_from_queue(dm_block_queue_t &queue);  // 从队列发送日志块
    uint8_t stack_size(struct dm_block *stack);  // 获取栈大小
    uint8_t queue_size(dm_block_queue_t queue);  // 获取队列大小

    // 块管理相关变量
    struct dm_block *_blocks_free;  // 空闲块
    dm_block_queue_t _blocks_sent;  // 已发送队列
    dm_block_queue_t _blocks_pending;  // 待发送队列
    dm_block_queue_t _blocks_retry;  // 重试队列

    // 统计信息结构体
    struct _stats {
        uint32_t resends;  // 重发次数
        uint8_t collection_count;  // 收集计数
        uint16_t state_free;  // 空闲状态累计
        uint8_t state_free_min;  // 最小空闲状态
        uint8_t state_free_max;  // 最大空闲状态
        uint16_t state_pending;  // 待发送状态累计
        uint8_t state_pending_min;  // 最小待发送状态
        uint8_t state_pending_max;  // 最大待发送状态
        uint16_t state_retry;  // 重试状态累计
        uint8_t state_retry_min;  // 最小重试状态
        uint8_t state_retry_max;  // 最大重试状态
        uint16_t state_sent;  // 已发送状态累计
        uint8_t state_sent_min;  // 最小已发送状态
        uint8_t state_sent_max;  // 最大已发送状态
    } stats;

    // 用于通过mavlink报告系统状态
    bool logging_enabled() const override { return true; }
    bool logging_failed() const override;

    const GCS_MAVLINK *_link;  // MAVLink通信链路

    uint8_t _target_system_id;  // 目标系统ID
    uint8_t _target_component_id;  // 目标组件ID

    // 控制每次push_log_blocks调用时从pending和sent队列推送的最大块数
    // push_log_blocks由periodic_tasks调用。每个块200字节
    // 在Plane中,50Hz时,_max_blocks_per_send_blocks为2意味着
    // 最多每秒推送2*50*200=20KB的日志
    const uint8_t _max_blocks_per_send_blocks;
    
    uint32_t _next_seq_num;  // 下一个序列号
    uint16_t _latest_block_len;  // 最新块长度
    uint32_t _last_response_time;  // 最后响应时间
    uint32_t _last_send_time;  // 最后发送时间
    uint8_t _next_block_number_to_resend;  // 下一个要重发的块号
    bool _sending_to_client;  // 是否正在向客户端发送

    void Write_logger_MAV(AP_Logger_MAVLink &logger);  // 写入MAVLink日志

    uint32_t bufferspace_available() override;  // 获取可用缓冲区空间(字节)
    uint8_t remaining_space_in_current_block() const;  // 获取当前块剩余空间
    
    // 写缓冲区
    uint8_t _blockcount_free;  // 空闲块计数
    uint8_t _blockcount;  // 总块数
    struct dm_block *_blocks;  // 块数组
    struct dm_block *_current_block;  // 当前块
    struct dm_block *next_block();  // 获取下一个块

    // 周期性任务
    void periodic_10Hz(uint32_t now) override;  // 10Hz任务
    void periodic_1Hz() override;  // 1Hz任务
    
    // 统计相关函数
    void stats_init();  // 初始化统计
    void stats_reset();  // 重置统计
    void stats_collect();  // 收集统计
    void stats_log();  // 记录统计
    uint32_t _stats_last_collected_time;  // 最后统计收集时间
    uint32_t _stats_last_logged_time;  // 最后统计记录时间
    uint8_t mavlink_seq;  // MAVLink序列号

    /* 目前忽略开始新日志的请求
     * 理论上我们可以关闭当前日志会话并期望客户端重新打开一个 */
    void start_new_log(void) override {
        return;
    }

    HAL_Semaphore semaphore;  // 信号量
};

#endif // HAL_LOGGING_MAVLINK_ENABLED
