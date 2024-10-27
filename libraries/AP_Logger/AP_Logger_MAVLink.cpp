/* 
   AP_Logger 通过MAVLink进行远程日志记录
*/

#include "AP_Logger_config.h"

#if HAL_LOGGING_MAVLINK_ENABLED

#include "AP_Logger_MAVLink.h"

#include "LogStructure.h"
#include <AP_Logger/AP_Logger.h>

// 是否启用远程日志调试
#define REMOTE_LOG_DEBUGGING 0

#if REMOTE_LOG_DEBUGGING
#include <stdio.h>
 # define Debug(fmt, args ...)  do {fprintf(stderr, "%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

#include <AP_InternalError/AP_InternalError.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// 构造函数,初始化基类和每次发送的最大块数
AP_Logger_MAVLink::AP_Logger_MAVLink(AP_Logger &front, LoggerMessageWriter_DFLogStart *writer) :
    AP_Logger_Backend(front, writer),
    _max_blocks_per_send_blocks(8)
{
    // 根据缓冲区大小计算块数量
    _blockcount = 1024*((uint8_t)_front._params.mav_bufsize) / sizeof(struct dm_block);
}

// 初始化函数
void AP_Logger_MAVLink::Init()
{
    _blocks = nullptr;
    // 尝试分配内存,如果失败则减半块数量重试
    while (_blockcount >= 8) { // 8是一个魔法数字
        _blocks = (struct dm_block *) calloc(_blockcount, sizeof(struct dm_block));
        if (_blocks != nullptr) {
            break;
        }
        _blockcount /= 2;
    }

    if (_blocks == nullptr) {
        return;
    }

    // 释放所有块并初始化统计信息
    free_all_blocks();
    stats_init();

    _initialised = true;
}

// 检查日志记录是否失败
bool AP_Logger_MAVLink::logging_failed() const
{
    return !_sending_to_client;
}

// 获取可用缓冲区空间大小
uint32_t AP_Logger_MAVLink::bufferspace_available() {
    return (_blockcount_free * 200 + remaining_space_in_current_block());
}

// 获取当前块中剩余空间
uint8_t AP_Logger_MAVLink::remaining_space_in_current_block() const {
    return (MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN - _latest_block_len);
}

// 将块添加到队列末尾
void AP_Logger_MAVLink::enqueue_block(dm_block_queue_t &queue, struct dm_block *block)
{
    if (queue.youngest != nullptr) {
        queue.youngest->next = block;
    } else {
        queue.oldest = block;
    }
    queue.youngest = block;
}

// 从队列中移除指定序号的块
struct AP_Logger_MAVLink::dm_block *AP_Logger_MAVLink::dequeue_seqno(AP_Logger_MAVLink::dm_block_queue_t &queue, uint32_t seqno)
{
    struct dm_block *prev = nullptr;
    for (struct dm_block *block=queue.oldest; block != nullptr; block=block->next) {
        if (block->seqno == seqno) {
            if (prev == nullptr) {
                if (queue.youngest == queue.oldest) {
                    queue.oldest = nullptr;
                    queue.youngest = nullptr;
                } else {
                    queue.oldest = block->next;
                }
            } else {
                if (queue.youngest == block) {
                    queue.youngest = prev;
                }
                prev->next = block->next;
            }
            block->next = nullptr;
            return block;
        }
        prev = block;
    }
    return nullptr;
}

// 从队列中释放指定序号的块
bool AP_Logger_MAVLink::free_seqno_from_queue(uint32_t seqno, dm_block_queue_t &queue)
{
    struct dm_block *block = dequeue_seqno(queue, seqno);
    if (block != nullptr) {
        block->next = _blocks_free;
        _blocks_free = block;
        _blockcount_free++; // 注释掉这行会暴露一个bug!
        return true;
    }
    return false;
}
    
// 检查是否可以写入数据
bool AP_Logger_MAVLink::WritesOK() const
{
    if (!_sending_to_client) {
        return false;
    }
    return true;
}

/* 在当前偏移位置写入一个数据块 */

// DM_write: 70734事件, 0次溢出, 167806us耗时, 平均2us, 最小1us 最大34us 0.620us均方根
bool AP_Logger_MAVLink::_WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical)
{
    // 尝试获取信号量
    if (!semaphore.take_nonblocking()) {
        _dropped++;
        return false;
    }

    // 检查缓冲区空间是否足够
    if (bufferspace_available() < size) {
        if (_startup_messagewriter->finished()) {
            // 不计入启动包的丢弃数
            _dropped++;
        }
        semaphore.give();
        return false;
    }

    uint16_t copied = 0;

    // 循环复制数据到块中
    while (copied < size) {
        if (_current_block == nullptr) {
            _current_block = next_block();
            if (_current_block == nullptr) {
                // 不应该发生 - 上面已经做了完整性检查
                INTERNAL_ERROR(AP_InternalError::error_t::logger_bad_current_block);
                semaphore.give();
                return false;
            }
        }
        uint16_t remaining_to_copy = size - copied;
        uint16_t _curr_remaining = remaining_space_in_current_block();
        uint16_t to_copy = (remaining_to_copy > _curr_remaining) ? _curr_remaining : remaining_to_copy;
        memcpy(&(_current_block->buf[_latest_block_len]), &((const uint8_t *)pBuffer)[copied], to_copy);
        copied += to_copy;
        _latest_block_len += to_copy;
        if (_latest_block_len == MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN) {
            // 块已满,标记为待发送
            enqueue_block(_blocks_pending, _current_block);
            _current_block = next_block();
        }
    }

    semaphore.give();

    return true;
}

// 获取一个空闲块
struct AP_Logger_MAVLink::dm_block *AP_Logger_MAVLink::next_block()
{
    AP_Logger_MAVLink::dm_block *ret = _blocks_free;
    if (ret != nullptr) {
        _blocks_free = ret->next;
        _blockcount_free--;
        ret->seqno = _next_seq_num++;
        ret->last_sent = 0;
        ret->next = nullptr;
        _latest_block_len = 0;
    }
    return ret;
}

// 释放所有块
void AP_Logger_MAVLink::free_all_blocks()
{
    _blocks_free = nullptr;
    _current_block = nullptr;

    // 重置所有队列
    _blocks_pending.sent_count = 0;
    _blocks_pending.oldest = _blocks_pending.youngest = nullptr;
    _blocks_retry.sent_count = 0;
    _blocks_retry.oldest = _blocks_retry.youngest = nullptr;
    _blocks_sent.sent_count = 0;
    _blocks_sent.oldest = _blocks_sent.youngest = nullptr;

    // 将所有块添加到空闲栈
    for(uint8_t i=0; i < _blockcount; i++) {
        _blocks[i].next = _blocks_free;
        _blocks_free = &_blocks[i];
        // 这个值并不重要,但它可以防止valgrind在确认块时报错
        // 另外,当我们收到确认时会检查序号,我们希望确认真正的块0!
        _blocks[i].seqno = 9876543;
    }
    _blockcount_free = _blockcount;

    _latest_block_len = 0;
}

// 停止日志记录
void AP_Logger_MAVLink::stop_logging()
{
    if (_sending_to_client) {
        _sending_to_client = false;
        _last_response_time = AP_HAL::millis();
    }
}

// 处理确认消息
void AP_Logger_MAVLink::handle_ack(const GCS_MAVLINK &link,
                                   const mavlink_message_t &msg,
                                   uint32_t seqno)
{
    if (!_initialised) {
        return;
    }
    // 处理停止日志记录命令
    if(seqno == MAV_REMOTE_LOG_DATA_BLOCK_STOP) {
        Debug("收到停止日志记录数据包");
        stop_logging();
        return;
    }
    // 处理开始日志记录命令
    if(seqno == MAV_REMOTE_LOG_DATA_BLOCK_START) {
        if (!_sending_to_client) {
            Debug("开始新日志");
            free_all_blocks();
            stats_init();
            _sending_to_client = true;
            _target_system_id = msg.sysid;
            _target_component_id = msg.compid;
            _link = &link;
            _next_seq_num = 0;
            start_new_log_reset_variables();
            _last_response_time = AP_HAL::millis();
            Debug("目标: (%u/%u)", _target_system_id, _target_component_id);
        }
        return;
    }

    // 检查已发送块(很可能在列表的第一个)
    if (free_seqno_from_queue(seqno, _blocks_sent)) {
        _last_response_time = AP_HAL::millis();
    } else if(free_seqno_from_queue(seqno, _blocks_retry)) {
        _last_response_time = AP_HAL::millis();
    } else {
        // 可能已经确认并放入空闲列表
    }
}

// 处理远程日志块状态消息
void AP_Logger_MAVLink::remote_log_block_status_msg(const GCS_MAVLINK &link,
                                                    const mavlink_message_t& msg)
{
    mavlink_remote_log_block_status_t packet;
    mavlink_msg_remote_log_block_status_decode(&msg, &packet);
    if (!semaphore.take_nonblocking()) {
        return;
    }
    switch ((MAV_REMOTE_LOG_DATA_BLOCK_STATUSES)packet.status) {
        case MAV_REMOTE_LOG_DATA_BLOCK_NACK:
            handle_retry(packet.seqno);
            break;
        case MAV_REMOTE_LOG_DATA_BLOCK_ACK:
            handle_ack(link, msg, packet.seqno);
            break;
        case MAV_REMOTE_LOG_DATA_BLOCK_STATUSES_ENUM_END:
            break;
    }
    semaphore.give();
}

// 处理重试请求
void AP_Logger_MAVLink::handle_retry(uint32_t seqno)
{
    if (!_initialised || !_sending_to_client) {
        return;
    }

    struct dm_block *victim = dequeue_seqno(_blocks_sent, seqno);
    if (victim != nullptr) {
        _last_response_time = AP_HAL::millis();
        enqueue_block(_blocks_retry, victim);
    }
}

// 初始化统计信息
void AP_Logger_MAVLink::stats_init() {
    _dropped = 0;
    stats.resends = 0;
    stats_reset();
}

// 重置统计信息
void AP_Logger_MAVLink::stats_reset() {
    stats.state_free = 0;
    stats.state_free_min = -1; // 无符号溢出
    stats.state_free_max = 0;
    stats.state_pending = 0;
    stats.state_pending_min = -1; // 无符号溢出
    stats.state_pending_max = 0;
    stats.state_retry = 0;
    stats.state_retry_min = -1; // 无符号溢出
    stats.state_retry_max = 0;
    stats.state_sent = 0;
    stats.state_sent_min = -1; // 无符号溢出
    stats.state_sent_max = 0;
    stats.collection_count = 0;
}

// 写入MAVLink日志统计信息
void AP_Logger_MAVLink::Write_logger_MAV(AP_Logger_MAVLink &logger_mav)
{
    if (logger_mav.stats.collection_count == 0) {
        return;
    }
    const struct log_MAV_Stats pkt{
        LOG_PACKET_HEADER_INIT(LOG_MAV_STATS),
        timestamp         : AP_HAL::micros64(),
        seqno             : logger_mav._next_seq_num-1,
        dropped           : logger_mav._dropped,
        retries           : logger_mav._blocks_retry.sent_count,
        resends           : logger_mav.stats.resends,
        state_free_avg    : (uint8_t)(logger_mav.stats.state_free/logger_mav.stats.collection_count),
        state_free_min    : logger_mav.stats.state_free_min,
        state_free_max    : logger_mav.stats.state_free_max,
        state_pending_avg : (uint8_t)(logger_mav.stats.state_pending/logger_mav.stats.collection_count),
        state_pending_min : logger_mav.stats.state_pending_min,
        state_pending_max : logger_mav.stats.state_pending_max,
        state_sent_avg    : (uint8_t)(logger_mav.stats.state_sent/logger_mav.stats.collection_count),
        state_sent_min    : logger_mav.stats.state_sent_min,
        state_sent_max    : logger_mav.stats.state_sent_max,
    };
    WriteBlock(&pkt,sizeof(pkt));
}

// 记录统计信息
void AP_Logger_MAVLink::stats_log()
{
    if (!_initialised) {
        return;
    }
    if (stats.collection_count == 0) {
        return;
    }
    Write_logger_MAV(*this);
#if REMOTE_LOG_DEBUGGING
    printf("D:%d Retry:%d Resent:%d SF:%d/%d/%d SP:%d/%d/%d SS:%d/%d/%d SR:%d/%d/%d\n",
           _dropped,
           _blocks_retry.sent_count,
           stats.resends,
           stats.state_free_min,
           stats.state_free_max,
           stats.state_free/stats.collection_count,
           stats.state_pending_min,
           stats.state_pending_max,
           stats.state_pending/stats.collection_count,
           stats.state_sent_min,
           stats.state_sent_max,
           stats.state_sent/stats.collection_count,
           stats.state_retry_min,
           stats.state_retry_max,
           stats.state_retry/stats.collection_count
        );
#endif
    stats_reset();
}

// 获取栈中块的数量
uint8_t AP_Logger_MAVLink::stack_size(struct dm_block *stack)
{
    uint8_t ret = 0;
    for (struct dm_block *block=stack; block != nullptr; block=block->next) {
        ret++;
    }
    return ret;
}

// 获取队列中块的数量
uint8_t AP_Logger_MAVLink::queue_size(dm_block_queue_t queue)
{
    return stack_size(queue.oldest);
}

// 收集统计信息
void AP_Logger_MAVLink::stats_collect()
{
    if (!_initialised) {
        return;
    }
    if (!semaphore.take_nonblocking()) {
        return;
    }
    uint8_t pending = queue_size(_blocks_pending);
    uint8_t sent = queue_size(_blocks_sent);
    uint8_t retry = queue_size(_blocks_retry);
    uint8_t sfree = stack_size(_blocks_free);

    if (sfree != _blockcount_free) {
        INTERNAL_ERROR(AP_InternalError::error_t::logger_blockcount_mismatch);
    }
    semaphore.give();

    stats.state_pending += pending;
    stats.state_sent += sent;
    stats.state_free += sfree;
    stats.state_retry += retry;

    if (pending < stats.state_pending_min) {
        stats.state_pending_min = pending;
    }
    if (pending > stats.state_pending_max) {
        stats.state_pending_max = pending;
    }
    if (retry < stats.state_retry_min) {
        stats.state_retry_min = retry;
    }
    if (retry > stats.state_retry_max) {
        stats.state_retry_max = retry;
    }
    if (sent < stats.state_sent_min) {
        stats.state_sent_min = sent;
    }
    if (sent > stats.state_sent_max) {
        stats.state_sent_max = sent;
    }
    if (sfree < stats.state_free_min) {
        stats.state_free_min = sfree;
    }
    if (sfree > stats.state_free_max) {
        stats.state_free_max = sfree;
    }
    
    stats.collection_count++;
}

/* 当我们"成功"从队列发送日志块时,将它们移到已发送列表中。
   不要对已发送的块调用此函数! */
bool AP_Logger_MAVLink::send_log_blocks_from_queue(dm_block_queue_t &queue)
{
    uint8_t sent_count = 0;
    while (queue.oldest != nullptr) {
        if (sent_count++ > _max_blocks_per_send_blocks) {
            return false;
        }
        if (! send_log_block(*queue.oldest)) {
            return false;
        }
        queue.sent_count++;
        struct AP_Logger_MAVLink::dm_block *tmp = dequeue_seqno(queue,queue.oldest->seqno);
        if (tmp != nullptr) { // 不应该为nullptr
            enqueue_block(_blocks_sent, tmp);
        } else {
            INTERNAL_ERROR(AP_InternalError::error_t::logger_dequeue_failure);
        }
    }
    return true;
}

// 推送日志块
void AP_Logger_MAVLink::push_log_blocks()
{
    if (!_initialised || !_sending_to_client) {
        return;
    }

    AP_Logger_Backend::WriteMoreStartupMessages();

    if (!semaphore.take_nonblocking()) {
        return;
    }

    if (! send_log_blocks_from_queue(_blocks_retry)) {
        semaphore.give();
        return;
    }

    if (! send_log_blocks_from_queue(_blocks_pending)) {
        semaphore.give();
        return;
    }
    semaphore.give();
}

// 重发数据
void AP_Logger_MAVLink::do_resends(uint32_t now)
{
    if (!_initialised || !_sending_to_client) {
        return;
    }

    uint8_t count_to_send = 5;
    if (_blockcount < count_to_send) {
        count_to_send = _blockcount;
    }
    uint32_t oldest = now - 100; // 重发前等待100毫秒
    while (count_to_send-- > 0) {
        if (!semaphore.take_nonblocking()) {
            return;
        }
        for (struct dm_block *block=_blocks_sent.oldest; block != nullptr; block=block->next) {
            // 只想每隔一段时间发送块:
            if (block->last_sent < oldest) {
                if (! send_log_block(*block)) {
                    // 发送块失败;稍后重试....
                    semaphore.give();
                    return;
                }
                stats.resends++;
            }
        }
        semaphore.give();
    }
}

// 注意:从这些周期性函数调用的任何函数都必须通过适当地获取信号量来处理块结构的锁定!
void AP_Logger_MAVLink::periodic_10Hz(const uint32_t now)
{
    do_resends(now);
    stats_collect();
}

void AP_Logger_MAVLink::periodic_1Hz()
{
    if (rate_limiter == nullptr &&
        (_front._params.mav_ratemax > 0 ||
         _front._params.disarm_ratemax > 0 ||
         _front._log_pause)) {
        // 如果日志速率最大值>0Hz或请求暂停流式条目的日志,则设置速率限制
        rate_limiter = NEW_NOTHROW AP_Logger_RateLimiter(_front, _front._params.mav_ratemax, _front._params.disarm_ratemax);
    }

    if (_sending_to_client &&
        _last_response_time + 10000 < _last_send_time) {
        // 另一端似乎超时了!
        Debug("客户端超时");
        _sending_to_client = false;
        return;
    }
    stats_log();
}

//TODO: 正确处理完整的txspace
bool AP_Logger_MAVLink::send_log_block(struct dm_block &block)
{
    if (!_initialised) {
       return false;
    }
    if (_link == nullptr) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return false;
    }
    // 不要完全填满缓冲区 - 并确保有足够的空间至少发送一个数据包:
    const uint16_t min_payload_space = 500;
    static_assert(MAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK_LEN <= min_payload_space,
                  "最小分配空间小于有效载荷长度");
    if (_link->txspace() < min_payload_space) {
        return false;
    }

#if DF_MAVLINK_DISABLE_INTERRUPTS
    void *istate = hal.scheduler->disable_interrupts_save();
#endif

// DM_packing: 267039事件, 0次溢出, 8440834us耗时, 平均31us, 最小31us 最大32us 0.488us均方根

    mavlink_message_t msg;
    mavlink_status_t *chan_status = mavlink_get_channel_status(_link->get_chan());
    uint8_t saved_seq = chan_status->current_tx_seq;
    chan_status->current_tx_seq = mavlink_seq++;
    // Debug("发送块 (%d)", block.seqno);
    mavlink_msg_remote_log_data_block_pack(mavlink_system.sysid,
                                           MAV_COMP_ID_LOG,
                                           &msg,
                                           _target_system_id,
                                           _target_component_id,
                                           block.seqno,
                                           block.buf);

#if DF_MAVLINK_DISABLE_INTERRUPTS
    hal.scheduler->restore_interrupts(istate);
#endif

    block.last_sent = AP_HAL::millis();
    chan_status->current_tx_seq = saved_seq;

    // 即使我们未能发送数据包也要设置_last_send_time;
    // 如果txspace重复堵塞,我们不应该增加问题并停止尝试记录
    _last_send_time = AP_HAL::millis();

    _mavlink_resend_uart(_link->get_chan(), &msg);

    return true;
}
#endif
