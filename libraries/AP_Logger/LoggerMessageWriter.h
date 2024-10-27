#pragma once

#include "AP_Logger_Backend.h"
#include <AP_Rally/AP_Rally.h>

// 日志消息写入器基类
// Base class for logger message writers
class LoggerMessageWriter {
public:

    // 重置写入器状态
    // Reset writer state
    virtual void reset() = 0;
    // 处理写入过程
    // Process writing
    virtual void process() = 0;
    // 检查是否完成写入
    // Check if writing is finished
    bool finished() const { return _finished; }

    // 设置日志后端
    // Set logger backend
    virtual void set_logger_backend(class AP_Logger_Backend *backend) {
        _logger_backend = backend;
    }

    // 检查是否没有足够时间写入消息
    // Check if there is not enough time for writing messages
    bool out_of_time_for_writing_messages() const;

protected:
    // 写入完成标志
    // Writing finished flag
    bool _finished = false;
    // 日志后端指针
    // Logger backend pointer
    AP_Logger_Backend *_logger_backend = nullptr;
};


// 系统信息写入器类
// System information writer class
class LoggerMessageWriter_WriteSysInfo : public LoggerMessageWriter {
public:

    void reset() override;
    void process() override;

private:
    // 写入阶段枚举
    // Writing stage enumeration
    enum class Stage : uint8_t {
        FIRMWARE_STRING = 0,  // 固件字符串
        GIT_VERSIONS,        // Git版本信息
        VER,                 // 版本信息("VER"消息)
        SYSTEM_ID,           // 系统ID
        PARAM_SPACE_USED,    // 参数空间使用情况
        RC_PROTOCOL,         // 遥控协议
        RC_OUTPUT,           // 遥控输出
    };
    Stage stage;
};

// 任务写入器类
// Mission writer class
class LoggerMessageWriter_WriteEntireMission : public LoggerMessageWriter {
public:

    void reset() override;
    void process() override;

private:
    // 写入阶段枚举
    // Writing stage enumeration
    enum class Stage {
        WRITE_NEW_MISSION_MESSAGE = 0,  // 写入新任务消息
        WRITE_MISSION_ITEMS,            // 写入任务项
        DONE                            // 完成
    };

    // 待发送的任务编号
    // Mission number to send
    uint16_t _mission_number_to_send;
    Stage stage;
};

// 集结点写入器类
// Rally points writer class
class LoggerMessageWriter_WriteAllRallyPoints : public LoggerMessageWriter {
public:

    void reset() override;
    void process() override;

private:
    // 写入阶段枚举
    // Writing stage enumeration
    enum class Stage {
        WRITE_NEW_RALLY_MESSAGE = 0,    // 写入新集结点消息
        WRITE_ALL_RALLY_POINTS,         // 写入所有集结点
        DONE                            // 完成
    };

    // 待发送的集结点编号
    // Rally point number to send
    uint16_t _rally_number_to_send;
    Stage stage = Stage::WRITE_NEW_RALLY_MESSAGE;
};

#if HAL_LOGGER_FENCE_ENABLED
// 多边形围栏写入器类
// Polygon fence writer class
class LoggerMessageWriter_Write_Polyfence : public LoggerMessageWriter {
public:

    void reset() override;
    void process() override;

private:
    // 写入阶段枚举
    // Writing stage enumeration
    enum class Stage {
        WRITE_NEW_FENCE_MESSAGE = 0,    // 写入新围栏消息
        WRITE_FENCE_ITEMS,              // 写入围栏项
        DONE                            // 完成
    };

    // 待发送的围栏编号
    // Fence number to send
    uint16_t _fence_number_to_send;
    Stage stage;
};
#endif // HAL_LOGGER_FENCE_ENABLED

// 数据闪存日志启动写入器类
// DataFlash log start writer class
class LoggerMessageWriter_DFLogStart : public LoggerMessageWriter {
public:
    LoggerMessageWriter_DFLogStart() :
        _writesysinfo()
#if AP_MISSION_ENABLED
        , _writeentiremission()
#endif
#if HAL_RALLY_ENABLED
        , _writeallrallypoints()
#endif
#if HAL_LOGGER_FENCE_ENABLED
        , _writeallpolyfence()
#endif
        {
        }

    // 设置日志后端,同时设置所有子写入器的后端
    // Set logger backend and all sub-writers' backend
    void set_logger_backend(class AP_Logger_Backend *backend) override final {
        LoggerMessageWriter::set_logger_backend(backend);
        _writesysinfo.set_logger_backend(backend);
#if AP_MISSION_ENABLED
        _writeentiremission.set_logger_backend(backend);
#endif
#if HAL_RALLY_ENABLED
        _writeallrallypoints.set_logger_backend(backend);
#endif
#if HAL_LOGGER_FENCE_ENABLED
        _writeallpolyfence.set_logger_backend(backend);
#endif
    }

    // 检查数据闪存消息写入是否超时
    // Check if DataFlash message writing is out of time
    bool out_of_time_for_writing_messages_df() const;

    void reset() override;
    void process() override;
    // 检查格式是否写入完成
    // Check if format writing is done
    bool fmt_done() const { return _fmt_done; }
    // 检查参数是否写入完成
    // Check if parameter writing is done
    bool params_done() const { return _params_done; }

    // 重置部分写入器以重新写入日志,仅在DONE状态下有效
    // Reset some writers to push logs again, only works in DONE state
#if AP_MISSION_ENABLED
    bool writeentiremission();
#endif
#if HAL_RALLY_ENABLED
    bool writeallrallypoints();
#endif
#if HAL_LOGGER_FENCE_ENABLED
    bool writeallfence();
#endif

private:

    // 检查处理时间限制
    // Check process time limit
    static bool check_process_limit(uint32_t start_us);

    // 写入阶段枚举
    // Writing stage enumeration
    enum class Stage {
        FORMATS = 0,             // 格式
        UNITS,                   // 单位
        MULTIPLIERS,             // 乘数
        FORMAT_UNITS,            // 格式单位
        PARMS,                   // 参数
        VEHICLE_MESSAGES,        // 车辆消息
        RUNNING_SUBWRITERS,      // 运行子写入器(必须最后运行,因为可以重做部分)
        DONE,                    // 完成
    };

    // 格式写入完成标志
    // Format writing done flag
    bool _fmt_done;
    // 参数写入完成标志
    // Parameter writing done flag
    bool _params_done;

    Stage stage;

    // 下一个要发送的格式
    // Next format to send
    uint16_t next_format_to_send;

    // 下一个要发送的单位
    // Next unit to send
    uint8_t _next_unit_to_send;
    // 下一个要发送的格式单位
    // Next format unit to send
    uint8_t _next_format_unit_to_send;
    // 下一个要发送的乘数
    // Next multiplier to send
    uint8_t _next_multiplier_to_send;

    // 参数令牌
    // Parameter token
    AP_Param::ParamToken token;
    // 参数指针
    // Parameter pointer
    AP_Param *ap;
    // 参数默认值
    // Parameter default value
    float param_default;
    // 参数类型
    // Parameter type
    enum ap_var_type type;


    // 系统信息写入器
    // System information writer
    LoggerMessageWriter_WriteSysInfo _writesysinfo;
#if AP_MISSION_ENABLED
    // 任务写入器
    // Mission writer
    LoggerMessageWriter_WriteEntireMission _writeentiremission;
#endif
#if HAL_RALLY_ENABLED
    // 集结点写入器
    // Rally points writer
    LoggerMessageWriter_WriteAllRallyPoints _writeallrallypoints;
#endif
#if HAL_LOGGER_FENCE_ENABLED
    // 围栏写入器
    // Fence writer
    LoggerMessageWriter_Write_Polyfence _writeallpolyfence;
#endif
};
