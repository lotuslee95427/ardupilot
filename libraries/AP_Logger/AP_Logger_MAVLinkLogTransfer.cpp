/*
  MAVLink日志文件传输功能
 */

/*
   本程序是自由软件:你可以在遵循GNU通用公共许可证(GPL)第3版或更高版本的条件下
   重新分发和/或修改它。

   本程序的发布是希望它能有用,但不提供任何保证;甚至不提供适销性或特定用途适用性的
   暗示保证。详见GNU通用公共许可证。

   你应该随程序获得一份GNU通用公共许可证的副本。如果没有,
   请访问<http://www.gnu.org/licenses/>。
 */

#include <GCS_MAVLink/GCS_config.h>
#include <AP_Logger/AP_Logger_config.h>

#if HAL_LOGGING_ENABLED && HAL_GCS_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h> // 用于LOG_ENTRY

extern const AP_HAL::HAL& hal;

/**
   处理来自地面站(GCS)的所有类型的日志下载请求
 */
void AP_Logger::handle_log_message(GCS_MAVLINK &link, const mavlink_message_t &msg)
{
    if (!WritesEnabled()) {
        // 当前用作"in_mavlink_delay"的代理
        return;
    }
    if (vehicle_is_armed()) {
        // 如果飞机处于解锁状态,不允许下载日志
        if (!_warned_log_disarm) {
            link.send_text(MAV_SEVERITY_ERROR, "需要上锁才能下载日志");
            _warned_log_disarm = true;
        }
        return;
    }
    _warned_log_disarm = false;
    _last_mavlink_log_transfer_message_handled_ms = AP_HAL::millis();
    
    // 根据消息ID处理不同类型的日志请求
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:  // 请求日志列表
        handle_log_request_list(link, msg);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:  // 请求日志数据
        handle_log_request_data(link, msg);
        break;
    case MAVLINK_MSG_ID_LOG_ERASE:  // 请求擦除日志
        handle_log_request_erase(link, msg);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_END:  // 请求结束日志传输
        handle_log_request_end(link, msg);
        break;
    }
}

/**
   处理地面站的日志列表请求
 */
void AP_Logger::handle_log_request_list(GCS_MAVLINK &link, const mavlink_message_t &msg)
{
    WITH_SEMAPHORE(_log_send_sem);  // 获取信号量

    // 检查是否已有日志传输在进行
    if (_log_sending_link != nullptr) {
        link.send_text(MAV_SEVERITY_INFO, "日志下载正在进行中");
        return;
    }

    // 解码日志请求列表消息
    mavlink_log_request_list_t packet;
    mavlink_msg_log_request_list_decode(&msg, &packet);

    _log_num_logs = get_num_logs();  // 获取日志总数

    // 设置日志列表的起始和结束索引
    if (_log_num_logs == 0) {
        _log_next_list_entry = 0;
        _log_last_list_entry = 0;        
    } else {
        _log_next_list_entry = packet.start;
        _log_last_list_entry = packet.end;

        // 确保索引在有效范围内
        if (_log_last_list_entry > _log_num_logs) {
            _log_last_list_entry = _log_num_logs;
        }
        if (_log_next_list_entry < 1) {
            _log_next_list_entry = 1;
        }
    }

    // 设置传输状态为列表传输
    transfer_activity = TransferActivity::LISTING;
    _log_sending_link = &link;

    handle_log_send_listing();  // 开始发送日志列表
}


/**
   处理日志数据请求
 */
void AP_Logger::handle_log_request_data(GCS_MAVLINK &link, const mavlink_message_t &msg)
{
    WITH_SEMAPHORE(_log_send_sem);  // 获取信号量

    // 检查是否已有日志传输在进行
    if (_log_sending_link != nullptr) {
        // 某些地面站(如MAVProxy)在填补下载日志的空缺时会尝试流式发送request_data消息
        // 这个通道检查可以避免向它们发出警告,代价是静默丢弃任何重复的日志开始尝试
        if (_log_sending_link->get_chan() != link.get_chan()) {
            link.send_text(MAV_SEVERITY_INFO, "日志下载正在进行中");
        }
        return;
    }

    // 解码日志数据请求消息
    mavlink_log_request_data_t packet;
    mavlink_msg_log_request_data_decode(&msg, &packet);

    // 考虑打开或切换日志:
    if (transfer_activity != TransferActivity::SENDING || _log_num_data != packet.id) {
        uint16_t num_logs = get_num_logs();
        // 检查请求的日志ID是否有效
        if (packet.id > num_logs || packet.id < 1) {
            // 无效日志请求;取消当前下载
            end_log_transfer();
            return;
        }

        // 获取日志信息
        uint32_t time_utc, size;
        get_log_info(packet.id, size, time_utc);
        _log_num_data = packet.id;
        _log_data_size = size;

        // 获取日志边界
        uint32_t end;
        get_log_boundaries(packet.id, _log_data_page, end);
    }

    // 设置数据偏移量和剩余数据大小
    _log_data_offset = packet.ofs;
    if (_log_data_offset >= _log_data_size) {
        _log_data_remaining = 0;
    } else {
        _log_data_remaining = _log_data_size - _log_data_offset;
    }
    if (_log_data_remaining > packet.count) {
        _log_data_remaining = packet.count;
    }

    // 设置传输状态为数据发送
    transfer_activity = TransferActivity::SENDING;
    _log_sending_link = &link;

    handle_log_send();  // 开始发送日志数据
}

/**
   处理擦除日志数据请求
 */
void AP_Logger::handle_log_request_erase(GCS_MAVLINK &link, const mavlink_message_t &msg)
{
    // mavlink_log_erase_t packet;
    // mavlink_msg_log_erase_decode(&msg, &packet);

    EraseAll();  // 擦除所有日志
}

/**
   处理停止传输并恢复正常日志记录的请求
 */
void AP_Logger::handle_log_request_end(GCS_MAVLINK &link, const mavlink_message_t &msg)
{
    WITH_SEMAPHORE(_log_send_sem);
    // mavlink_log_request_end_t packet;
    // mavlink_msg_log_request_end_decode(&msg, &packet);
    end_log_transfer();  // 结束日志传输
}

/**
 * 结束日志传输
 */
void AP_Logger::end_log_transfer()
{
    transfer_activity = TransferActivity::IDLE;  // 设置传输状态为空闲
    _log_sending_link = nullptr;  // 清除发送链接
    backends[0]->end_log_transfer();  // 通知后端结束传输
}

/**
   如果有待发送的日志消息,触发发送
 */
void AP_Logger::handle_log_send()
{
    WITH_SEMAPHORE(_log_send_sem);

    if (_log_sending_link == nullptr) {
        return;
    }
    if (hal.util->get_soft_armed()) {
        // 可能正在飞行,不发送日志
        return;
    }
    
    // 根据传输活动状态处理
    switch (transfer_activity) {
    case TransferActivity::IDLE:  // 空闲状态
        break;
    case TransferActivity::LISTING:  // 列表传输状态
        handle_log_send_listing();
        break;
    case TransferActivity::SENDING:  // 数据发送状态
        handle_log_sending();
        break;
    }
}

/**
 * 处理日志数据发送
 */
void AP_Logger::handle_log_sending()
{
    WITH_SEMAPHORE(_log_send_sem);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // 在SITL中假设USB速度用于日志下载
    const uint8_t num_sends = 40;
#else
    // 根据连接类型调整发送数量
    uint8_t num_sends = 1;
    if (_log_sending_link->is_high_bandwidth() && hal.gpio->usb_connected()) {
        // USB连接时可以发送更多数据
        num_sends = 250;
    } else if (_log_sending_link->have_flow_control()) {
    #if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
        num_sends = 80;
    #else
        num_sends = 10;
    #endif
    }
#endif

    // 循环发送数据
    for (uint8_t i=0; i<num_sends; i++) {
        if (transfer_activity != TransferActivity::SENDING) {
            // 可能已完成数据发送
            break;
        }
        if (!handle_log_send_data()) {
            break;
        }
    }
}

/**
   如果有待发送的日志消息,触发发送日志列表
 */
void AP_Logger::handle_log_send_listing()
{
    WITH_SEMAPHORE(_log_send_sem);

    // 检查是否有足够的发送空间
    if (!HAVE_PAYLOAD_SPACE(_log_sending_link->get_chan(), LOG_ENTRY)) {
        return;
    }
    // 给心跳包一个机会
    if (AP_HAL::millis() - _log_sending_link->get_last_heartbeat_time() > 3000) {
        return;
    }

    // 获取日志信息
    uint32_t size, time_utc;
    if (_log_next_list_entry == 0) {
        size = 0;
        time_utc = 0;
    } else {
        get_log_info(_log_next_list_entry, size, time_utc);
    }
    
    // 发送日志条目信息
    mavlink_msg_log_entry_send(_log_sending_link->get_chan(),
                               _log_next_list_entry,
                               _log_num_logs,
                               _log_last_list_entry,
                               time_utc,
                               size);
                               
    // 检查是否完成列表发送
    if (_log_next_list_entry == _log_last_list_entry) {
        transfer_activity = TransferActivity::IDLE;
        _log_sending_link = nullptr;
    } else {
        _log_next_list_entry++;
    }
}

/**
   如果有待发送的日志数据,触发发送
   @return 是否成功发送数据
 */
bool AP_Logger::handle_log_send_data()
{
    WITH_SEMAPHORE(_log_send_sem);

    // 检查是否有足够的发送空间
    if (!HAVE_PAYLOAD_SPACE(_log_sending_link->get_chan(), LOG_DATA)) {
        return false;
    }
    // 给心跳包一个机会
    if (AP_HAL::millis() - _log_sending_link->get_last_heartbeat_time() > 3000) {
        return false;
    }

    int16_t nbytes = 0;
    uint32_t len = _log_data_remaining;
    mavlink_log_data_t packet;

    // 限制单次发送长度
    if (len > MAVLINK_MSG_LOG_DATA_FIELD_DATA_LEN) {
        len = MAVLINK_MSG_LOG_DATA_FIELD_DATA_LEN;
    }

    // 获取日志数据
    nbytes = get_log_data(_log_num_data, _log_data_page, _log_data_offset, len, packet.data);

    if (nbytes < 0) {
        // 错误时报告为EOF
        nbytes = 0;
    }
    // 填充剩余空间
    if (nbytes < MAVLINK_MSG_LOG_DATA_FIELD_DATA_LEN) {
        memset(&packet.data[nbytes], 0, MAVLINK_MSG_LOG_DATA_FIELD_DATA_LEN-nbytes);
    }

    // 设置数据包信息
    packet.ofs = _log_data_offset;
    packet.id = _log_num_data;
    packet.count = nbytes;
    
    // 发送数据包
    _mav_finalize_message_chan_send(_log_sending_link->get_chan(),
                                    MAVLINK_MSG_ID_LOG_DATA,
                                    (const char *)&packet,
                                    MAVLINK_MSG_ID_LOG_DATA_MIN_LEN,
                                    MAVLINK_MSG_ID_LOG_DATA_LEN,
                                    MAVLINK_MSG_ID_LOG_DATA_CRC);

    // 更新偏移量和剩余数据量
    _log_data_offset += nbytes;
    _log_data_remaining -= nbytes;
    
    // 检查是否完成数据发送
    if (nbytes < MAVLINK_MSG_LOG_DATA_FIELD_DATA_LEN || _log_data_remaining == 0) {
        end_log_transfer();
    }
    return true;
}

#endif  // HAL_LOGGING_ENABLED && HAL_GCS_ENABLED
