#pragma once

#include "AP_Logger_config.h"

#if HAL_LOGGING_ENABLED

#include <AP_Common/Bitmask.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Vehicle/ModeReason.h>
#include "LogStructure.h"

class LoggerMessageWriter_DFLogStart;

// 用于处理日志消息速率限制的类
class AP_Logger_RateLimiter
{
public:
    // 构造函数,接收日志记录器前端、速率限制和解除武装速率限制参数
    AP_Logger_RateLimiter(const class AP_Logger &_front, const AP_Float &_limit_hz, const AP_Float &_disarm_limit_hz);

    // 检查消息是否通过速率限制测试
    bool should_log(uint8_t msgid, bool writev_streaming);
    // 检查流式消息是否通过速率限制测试
    bool should_log_streaming(uint8_t msgid, float rate_hz);

private:
    const AP_Logger &front;          // 日志记录器前端引用
    const AP_Float &rate_limit_hz;   // 速率限制参数
    const AP_Float &disarm_rate_limit_hz; // 解除武装时的速率限制参数

    uint16_t last_send_ms[256];      // 每种消息类型最后发送时间(ms)
    uint16_t last_sched_count[256];  // 每种消息类型最后调度计数

    // 非流式消息类型的位掩码,用于缓存以避免频繁调用structure_for_msg_type
    Bitmask<256> not_streaming;

    // 每种消息类型的最后决定结果,用于多实例处理
    Bitmask<256> last_return;
};

// 日志记录器后端基类
class AP_Logger_Backend
{
public:
    // 定义用于写入车辆启动消息的函数类型
    FUNCTOR_TYPEDEF(vehicle_startup_message_Writer, void);

    // 构造函数
    AP_Logger_Backend(AP_Logger &front,
                      class LoggerMessageWriter_DFLogStart *writer);

    // 获取车辆消息写入器
    vehicle_startup_message_Writer vehicle_message_writer() const;

    // 检查存储卡是否插入
    virtual bool CardInserted(void) const = 0;

    // 擦除所有日志
    virtual void EraseAll() = 0;

    /* 在当前偏移位置写入数据块 */
    bool WriteBlock(const void *pBuffer, uint16_t size) {
        return WritePrioritisedBlock(pBuffer, size, false);
    }

    // 写入关键数据块
    bool WriteCriticalBlock(const void *pBuffer, uint16_t size) {
        return WritePrioritisedBlock(pBuffer, size, true);
    }

    // 写入优先级数据块
    bool WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical, bool writev_streaming=false);

    // 高级接口,通过日志列表中的位置索引
    virtual uint16_t find_last_log() = 0;                                                     // 查找最后一个日志
    virtual void get_log_boundaries(uint16_t list_entry, uint32_t & start_page, uint32_t & end_page) = 0;  // 获取日志边界
    virtual void get_log_info(uint16_t list_entry, uint32_t &size, uint32_t &time_utc) = 0;  // 获取日志信息
    virtual int16_t get_log_data(uint16_t list_entry, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) = 0;  // 获取日志数据
    virtual void end_log_transfer() = 0;                                                      // 结束日志传输
    virtual uint16_t get_num_logs() = 0;                                                      // 获取日志数量
    virtual uint16_t find_oldest_log();                                                       // 查找最旧的日志

    // 检查日志记录是否已启动
    virtual bool logging_started(void) const = 0;

    // 初始化
    virtual void Init() = 0;

    // 获取可用缓冲区空间
    virtual uint32_t bufferspace_available() = 0;

    // 为武装准备
    virtual void PrepForArming();

    // 开始新日志
    virtual void start_new_log() { }

    /* 停止日志记录 - 关闭输出文件等
     * 注意这不会阻止日志立即重新开始
     * 例如 AP_Logger_MAVLink 可能从客户端收到另一个开始包
     */
    virtual void stop_logging(void) = 0;
    
    // 异步停止日志记录,状态可通过logging_started()确定
    virtual void stop_logging_async(void) { stop_logging(); }

    // 填充日志格式信息
    void Fill_Format(const struct LogStructure *structure, struct log_Format &pkt);
    void Fill_Format_Units(const struct LogStructure *s, struct log_Format_Units &pkt);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    // 目前只有AP_Logger_File支持此功能:
    virtual void flush(void) { }
#endif

    // 用于Logger_MAVlink
    virtual void remote_log_block_status_msg(const class GCS_MAVLINK &link,
                                             const mavlink_message_t &msg) { }

    // 执行周期性任务
    virtual void periodic_tasks();

    // 获取日志结构相关信息
    uint8_t num_types() const;                                           // 获取类型数量
    const struct LogStructure *structure(uint8_t structure) const;       // 获取日志结构
    uint8_t num_units() const;                                          // 获取单位数量
    const struct UnitStructure *unit(uint8_t unit) const;               // 获取单位结构
    uint8_t num_multipliers() const;                                    // 获取乘数数量
    const struct MultiplierStructure *multiplier(uint8_t multiplier) const;  // 获取乘数结构

    // 写入各种数据
    bool Write_EntireMission();                                         // 写入整个任务
    bool Write_RallyPoint(uint8_t total,                               // 写入集结点
                          uint8_t sequence,
                          const class RallyLocation &rally_point);
    bool Write_Rally();                                                 // 写入所有集结点
#if HAL_LOGGER_FENCE_ENABLED
    bool Write_FencePoint(uint8_t total, uint8_t sequence, const class AC_PolyFenceItem &fence_point);  // 写入围栏点
    bool Write_Fence();                                                 // 写入所有围栏
#endif
    bool Write_Format(const struct LogStructure *structure);            // 写入格式
    bool have_emitted_format_for_type(LogMessages a_type) const {      // 检查是否已发出某类型的格式
        return _formats_written.get(uint8_t(a_type));
    }
    bool Write_Message(const char *message);                            // 写入消息
    bool Write_MessageF(const char *fmt, ...);                         // 写入格式化消息
    bool Write_Mission_Cmd(const AP_Mission &mission,                  // 写入任务命令
                               const AP_Mission::Mission_Command &cmd);
    bool Write_Mode(uint8_t mode, const ModeReason reason);            // 写入模式
    bool Write_Parameter(const char *name, float value, float default_val);  // 写入参数
    bool Write_Parameter(const AP_Param *ap,                           // 写入参数(重载)
                             const AP_Param::ParamToken &token,
                             enum ap_var_type type,
                             float default_val);
    bool Write_VER();                                                  // 写入版本信息

    // 获取丢弃的消息数量
    uint32_t num_dropped(void) const {
        return _dropped;
    }

    /*
     * 写入支持
     */
    // 写入FMT消息(如果尚未写入)
    // 返回FMT消息是否曾经被写入
    bool Write_Emit_FMT(uint8_t msg_type);

    // 如果尚未写入则输出FMT消息
    void Safe_Write_Emit_FMT(uint8_t msg_type);

    // 写入指定类型的日志消息,参数在arg_list中
    bool Write(uint8_t msg_type, va_list arg_list, bool is_critical=false, bool is_streaming=false);

    // 用于通过mavlink报告系统状态
    virtual bool logging_enabled() const;                               // 检查日志是否启用
    virtual bool logging_failed() const = 0;                           // 检查日志是否失败

    // 在启动EKF前可能需要确保数据可记录
    bool allow_start_ekf() const;

    // 车辆解除武装时调用
    virtual void vehicle_was_disarmed();

    // 写入单位和乘数信息
    bool Write_Unit(const struct UnitStructure *s);
    bool Write_Multiplier(const struct MultiplierStructure *s);
    bool Write_Format_Units(const struct LogStructure *structure);

    // IO定时器
    virtual void io_timer(void) {}

protected:
    AP_Logger &_front;                                                 // 日志记录器前端引用

    // 周期性任务
    virtual void periodic_10Hz(const uint32_t now);                    // 10Hz周期任务
    virtual void periodic_1Hz();                                       // 1Hz周期任务
    virtual void periodic_fullrate();                                  // 全速率周期任务

    // 检查是否应该记录
    bool ShouldLog(bool is_critical);
    virtual bool WritesOK() const = 0;                                // 检查是否可以写入
    virtual bool StartNewLogOK() const;                               // 检查是否可以开始新日志

    // 由PrepForArming调用以实际开始日志记录
    virtual void PrepForArming_start_logging(void) {
        start_new_log();
    }

    // 写入更多启动消息
    virtual void WriteMoreStartupMessages();
    // 推送日志块
    virtual void push_log_blocks();

    LoggerMessageWriter_DFLogStart *_startup_messagewriter;            // 启动消息写入器
    bool _writing_startup_messages;                                    // 是否正在写入启动消息

    uint16_t _cached_oldest_log;                                      // 缓存的最旧日志编号

    uint32_t _dropped;                                                // 丢弃的消息数量
    bool _rotate_pending;                                             // 下次停止日志时是否需要轮换

    // 开始新日志时必须调用:
    virtual void start_new_log_reset_variables();
    // 在存储和标准化编号之间转换日志编号
    uint16_t log_num_from_list_entry(const uint16_t list_entry);

    // 计算关键消息保留空间
    uint32_t critical_message_reserved_space(uint32_t bufsize) const {
        uint32_t ret = 1024;
        if (ret > bufsize) {
            ret = bufsize;
        }
        return ret;
    };

    // 计算非消息写入器消息保留空间
    uint32_t non_messagewriter_message_reserved_space(uint32_t bufsize) const {
        uint32_t ret = 1024;
        if (ret >= bufsize) {
            ret = 0;
        }
        return ret;
    };

    // 写入优先级数据块的实际实现
    virtual bool _WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical) = 0;

    bool _initialised;                                                // 是否已初始化

    // 数据闪存统计相关函数
    void df_stats_gather(uint16_t bytes_written, uint32_t space_remaining);
    void df_stats_log();
    void df_stats_clear();

    AP_Logger_RateLimiter *rate_limiter;                             // 速率限制器

private:
    // 统计信息结构
    struct df_stats {
        uint16_t blocks;                                             // 块数
        uint32_t bytes;                                             // 字节数
        uint32_t buf_space_min;                                     // 最小缓冲区空间
        uint32_t buf_space_max;                                     // 最大缓冲区空间
        uint32_t buf_space_sigma;                                   // 缓冲区空间总和
    };
    struct df_stats stats;                                          // 统计信息实例

    uint32_t _last_periodic_1Hz;                                    // 上次1Hz周期任务时间
    uint32_t _last_periodic_10Hz;                                   // 上次10Hz周期任务时间
    bool have_logged_armed;                                         // 是否已记录武装状态

    // 写入统计信息
    void Write_AP_Logger_Stats_File(const struct df_stats &_stats);
    void validate_WritePrioritisedBlock(const void *pBuffer, uint16_t size);

    // 从数据块获取消息类型
    bool message_type_from_block(const void *pBuffer, uint16_t size, LogMessages &type) const;
    bool ensure_format_emitted(const void *pBuffer, uint16_t size);
    bool emit_format_for_type(LogMessages a_type);
    Bitmask<256> _formats_written;                                  // 已写入格式的位掩码

};

#endif  // HAL_LOGGING_ENABLED
