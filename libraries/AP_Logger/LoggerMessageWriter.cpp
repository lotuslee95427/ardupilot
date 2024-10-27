#include "AP_Logger_config.h"

#if HAL_LOGGING_ENABLED

#include "AP_Common/AP_FWVersion.h"
#include "LoggerMessageWriter.h"
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include "AP_Logger.h"

#if HAL_LOGGER_FENCE_ENABLED
    #include <AC_Fence/AC_Fence.h>
#endif

#if HAL_LOGGER_RALLY_ENABLED
#include <AP_Rally/AP_Rally.h>
#endif

#define FORCE_VERSION_H_INCLUDE
#include "ap_version.h"
#undef FORCE_VERSION_H_INCLUDE

// 消息写入器主要在periodic_fullrate()结束时调用,有300us的预算
// 在写入完成之前调用Write()将失败,所以我们希望尽可能多地使用预算
// 以便尽早开始正式记录日志
// the message writers are mainly called at the end of periodic_fullrate() with a budget of 300us
// until they are finished calls to Write() will fail so we want to take up as much of
// the budget as possible in order that logging can begin in earnest as early as possible
#define MIN_LOOP_TIME_REMAINING_FOR_MESSAGE_WRITE_US 50

extern const AP_HAL::HAL& hal;

/* LogStartup - 这些是简单的状态机,允许我们
 * 逐步向日志文件写入消息
 */
/* LogStartup - these are simple state machines which allow us to
 * trickle out messages to the log files
 */

// 重置状态机
void LoggerMessageWriter::reset()
{
    _finished = false;
}

// 检查是否没有足够时间写入消息
bool LoggerMessageWriter::out_of_time_for_writing_messages() const
{
#if AP_SCHEDULER_ENABLED
    return AP::scheduler().time_available_usec() < MIN_LOOP_TIME_REMAINING_FOR_MESSAGE_WRITE_US;
#else
    return false;
#endif
}

// 重置DFLogStart写入器的状态
void LoggerMessageWriter_DFLogStart::reset()
{
    LoggerMessageWriter::reset();

    _fmt_done = false;
    _params_done = false;
    _writesysinfo.reset();
#if AP_MISSION_ENABLED
    _writeentiremission.reset();
#endif
#if HAL_LOGGER_RALLY_ENABLED
    _writeallrallypoints.reset();
#endif
#if HAL_LOGGER_FENCE_ENABLED
    _writeallpolyfence.reset();
#endif

    stage = Stage::FORMATS;
    next_format_to_send = 0;
    _next_unit_to_send = 0;
    _next_multiplier_to_send = 0;
    _next_format_unit_to_send = 0;
    param_default = AP::logger().quiet_nanf();
    ap = AP_Param::first(&token, &type, &param_default);
}

// 检查DFLogStart写入器是否没有足够时间写入消息
bool LoggerMessageWriter_DFLogStart::out_of_time_for_writing_messages_df() const
{
    if (stage == Stage::FORMATS) {
        // 尽可能快地写出FMT消息
        // write out the FMT messages as fast as we can
#if AP_SCHEDULER_ENABLED
        return AP::scheduler().time_available_usec() == 0;
#else
        return false;
#endif
    }
    return LoggerMessageWriter::out_of_time_for_writing_messages();
}

/*
  检查是否在process()阶段花费了太长时间
  如果超时则返回true表示应该停止处理
 */
/*
  check if we've taken too long in a process() stage
  return true if we should stop processing now as we are out of time
 */
bool LoggerMessageWriter_DFLogStart::check_process_limit(uint32_t start_us)
{
    const uint32_t limit_us = 1000U;
    if (AP_HAL::micros() - start_us > limit_us) {
        return true;
    }
    return false;
}

// 处理日志写入
void LoggerMessageWriter_DFLogStart::process()
{
    if (out_of_time_for_writing_messages_df()) {
        return;
    }
    // 允许任何阶段运行最多1ms,以防止在解锁时出现长循环
    // allow any stage to run for max 1ms, to prevent a long loop on arming
    const uint32_t start_us = AP_HAL::micros();

    switch(stage) {
    case Stage::FORMATS:
        // 写入日志格式以使日志具有自描述性
        // write log formats so the log is self-describing
        while (next_format_to_send < _logger_backend->num_types()) {
            const auto &s { _logger_backend->structure(next_format_to_send) };
            if (_logger_backend->have_emitted_format_for_type((LogMessages)s->msg_type)) {
                next_format_to_send++;
                continue;
            }
            if (!_logger_backend->Write_Format(s)) {
                return; // 再次调用我!
            }
            next_format_to_send++;
            if (check_process_limit(start_us)) {
                return; // 再次调用我!
            }
        }
        _fmt_done = true;
        stage = Stage::PARMS;
        FALLTHROUGH;

    case Stage::PARMS: {
        while (ap) {
            if (!_logger_backend->Write_Parameter(ap, token, type, param_default)) {
                return;
            }
            param_default = AP::logger().quiet_nanf();
            ap = AP_Param::next_scalar(&token, &type, &param_default);
            if (check_process_limit(start_us)) {
                return; // 再次调用我!
            }
        }

        _params_done = true;
        stage = Stage::UNITS;
        }
        FALLTHROUGH;

    case Stage::UNITS:
        while (_next_unit_to_send < _logger_backend->num_units()) {
            if (!_logger_backend->Write_Unit(_logger_backend->unit(_next_unit_to_send))) {
                return; // 再次调用我!
            }
            _next_unit_to_send++;
            if (check_process_limit(start_us)) {
                return; // 再次调用我!
            }
        }
        stage = Stage::MULTIPLIERS;
        FALLTHROUGH;

    case Stage::MULTIPLIERS:
        while (_next_multiplier_to_send < _logger_backend->num_multipliers()) {
            if (!_logger_backend->Write_Multiplier(_logger_backend->multiplier(_next_multiplier_to_send))) {
                return; // 再次调用我!
            }
            _next_multiplier_to_send++;
            if (check_process_limit(start_us)) {
                return; // 再次调用我!
            }
        }
        stage = Stage::UNITS;
        FALLTHROUGH;

    case Stage::FORMAT_UNITS:
        while (_next_format_unit_to_send < _logger_backend->num_types()) {
            if (!_logger_backend->Write_Format_Units(_logger_backend->structure(_next_format_unit_to_send))) {
                return; // 再次调用我!
            }
            _next_format_unit_to_send++;
            if (check_process_limit(start_us)) {
                return; // 再次调用我!
            }
        }
        stage = Stage::RUNNING_SUBWRITERS;
        FALLTHROUGH;

    case Stage::RUNNING_SUBWRITERS: {
        LoggerMessageWriter *subwriters[] {
            &_writesysinfo,
#if AP_MISSION_ENABLED
            &_writeentiremission,
#endif
#if HAL_LOGGER_RALLY_ENABLED
            &_writeallrallypoints,
#endif
#if HAL_LOGGER_FENCE_ENABLED
            &_writeallpolyfence,
#endif
        };
        for (auto *sw : subwriters) {
            if (!sw->finished()) {
                sw->process();
                if (!sw->finished()) {
                    return;
                }
            }
        }
        stage = Stage::VEHICLE_MESSAGES;
        FALLTHROUGH;
    }
    case Stage::VEHICLE_MESSAGES:
        // 我们保证为车辆启动消息预留200字节的空间。
        // 这允许它们是简单的函数而不是基于LoggerMessageWriter的状态机
        // we guarantee 200 bytes of space for the vehicle startup
        // messages.  This allows them to be simple functions rather
        // than e.g. LoggerMessageWriter-based state machines
        if (_logger_backend->vehicle_message_writer()) {
            if (_logger_backend->bufferspace_available() < 200) {
                return;
            }
            (_logger_backend->vehicle_message_writer())();
        }
        stage = Stage::DONE;
        FALLTHROUGH;

    case Stage::DONE:
        break;
    }

    _finished = true;
}

#if AP_MISSION_ENABLED
// 写入整个任务
bool LoggerMessageWriter_DFLogStart::writeentiremission()
{
    if (stage != Stage::DONE) {
        return false;
    }
    stage = Stage::RUNNING_SUBWRITERS;
    _finished = false;
    _writeentiremission.reset();
    return true;
}
#endif

#if HAL_LOGGER_RALLY_ENABLED
// 写入所有集结点
bool LoggerMessageWriter_DFLogStart::writeallrallypoints()
{
    if (stage != Stage::DONE) {
        return false;
    }
    stage = Stage::RUNNING_SUBWRITERS;
    _finished = false;
    _writeallrallypoints.reset();
    return true;
}
#endif

#if HAL_LOGGER_FENCE_ENABLED
// 写入所有围栏点
bool LoggerMessageWriter_DFLogStart::writeallfence()
{
    if (stage != Stage::DONE) {
        return false;
    }
    stage = Stage::RUNNING_SUBWRITERS;
    _finished = false;
    _writeallpolyfence.reset();
    return true;
}
#endif

// 重置系统信息写入器
void LoggerMessageWriter_WriteSysInfo::reset()
{
    LoggerMessageWriter::reset();
    stage = Stage::FIRMWARE_STRING;
}

// 处理系统信息写入
void LoggerMessageWriter_WriteSysInfo::process() {
    const AP_FWVersion &fwver = AP::fwversion();

    switch(stage) {

    case Stage::FIRMWARE_STRING:
#ifdef AP_CUSTOM_FIRMWARE_STRING
        // 如果不同,也记录原始固件字符串
        // also log original firmware string if different
        if (! _logger_backend->Write_MessageF("%s [%s]",
                                              fwver.fw_string,
                                              fwver.fw_string_original)) {
            return; // 再次调用我
        }
#else
        if (! _logger_backend->Write_Message(fwver.fw_string)) {
            return; // 再次调用我
        }
#endif
        stage = Stage::GIT_VERSIONS;
        FALLTHROUGH;

    case Stage::GIT_VERSIONS:
        if (fwver.middleware_name && fwver.os_name) {
            if (! _logger_backend->Write_MessageF("%s: %s %s: %s",
                                                        fwver.middleware_name,
                                                        fwver.middleware_hash_str,
                                                        fwver.os_name,
                                                        fwver.os_hash_str)) {
                return; // 再次调用我
            }
        } else if (fwver.os_name) {
            if (! _logger_backend->Write_MessageF("%s: %s",
                                                        fwver.os_name,
                                                        fwver.os_hash_str)) {
                return; // 再次调用我
            }
        }
        stage = Stage::VER;
        FALLTHROUGH;

    case Stage::VER: {
        if (!_logger_backend->Write_VER()) {
            return;
        }
        stage = Stage::SYSTEM_ID;
        FALLTHROUGH;
    }
    case Stage::SYSTEM_ID:
        char sysid[50];
        if (hal.util->get_system_id(sysid)) {
            if (! _logger_backend->Write_Message(sysid)) {
                return; // 再次调用我
            }
        }
        stage = Stage::PARAM_SPACE_USED;
        FALLTHROUGH;

    case Stage::PARAM_SPACE_USED:
        if (! _logger_backend->Write_MessageF("Param space used: %u/%u", AP_Param::storage_used(), AP_Param::storage_size())) {
            return; // 再次调用我
        }
        stage = Stage::RC_PROTOCOL;
        FALLTHROUGH;

    case Stage::RC_PROTOCOL: {
        const char *prot = hal.rcin->protocol();
        if (prot == nullptr) {
            prot = "None";
        }
        if (! _logger_backend->Write_MessageF("RC Protocol: %s", prot)) {
            return; // 再次调用我
        }
        stage = Stage::RC_OUTPUT;
        FALLTHROUGH;
    }
    case Stage::RC_OUTPUT: {
        char banner_msg[50];
        if (hal.rcout->get_output_mode_banner(banner_msg, sizeof(banner_msg))) {
            if (!_logger_backend->Write_Message(banner_msg)) {
                return; // 再次调用我
            }
        }
        break;
    }
    }

    _finished = true;  // 全部完成!
}

#if HAL_LOGGER_RALLY_ENABLED
// 处理集结点写入
void LoggerMessageWriter_WriteAllRallyPoints::process()
{
    const AP_Rally *_rally = AP::rally();
    if (_rally == nullptr) {
        _finished = true;
        return;
    }

    switch(stage) {

    case Stage::WRITE_NEW_RALLY_MESSAGE:
        if (! _logger_backend->Write_Message("New rally")) {
            return; // 再次调用我
        }
        stage = Stage::WRITE_ALL_RALLY_POINTS;
        FALLTHROUGH;

    case Stage::WRITE_ALL_RALLY_POINTS:
        while (_rally_number_to_send < _rally->get_rally_total()) {
            if (out_of_time_for_writing_messages()) {
                return;
            }
            RallyLocation rallypoint;
            if (_rally->get_rally_point_with_index(_rally_number_to_send, rallypoint)) {
                if (!_logger_backend->Write_RallyPoint(
                        _rally->get_rally_total(),
                        _rally_number_to_send,
                        rallypoint)) {
                    return; // 再次调用我
                }
            }
            _rally_number_to_send++;
        }
        stage = Stage::DONE;
        FALLTHROUGH;

    case Stage::DONE:
        break;
    }

    _finished = true;
}

// 重置集结点写入器
void LoggerMessageWriter_WriteAllRallyPoints::reset()
{
    LoggerMessageWriter::reset();
    stage = Stage::WRITE_NEW_RALLY_MESSAGE;
    _rally_number_to_send = 0;
}
#endif  // HAL_LOGGER_RALLY_ENABLED

// 处理任务写入
void LoggerMessageWriter_WriteEntireMission::process() {
    const AP_Mission *_mission = AP::mission();
    if (_mission == nullptr) {
        _finished = true;
        return;
    }

    switch(stage) {

    case Stage::WRITE_NEW_MISSION_MESSAGE:
        if (! _logger_backend->Write_Message("New mission")) {
            return; // 再次调用我
        }
        stage = Stage::WRITE_MISSION_ITEMS;
        FALLTHROUGH;

    case Stage::WRITE_MISSION_ITEMS: {
        AP_Mission::Mission_Command cmd;
        while (_mission_number_to_send < _mission->num_commands()) {
            if (out_of_time_for_writing_messages()) {
                return;
            }
            // 写入任务失败时我们将从存储重新读取;
            // 这可以改进。
            // upon failure to write the mission we will re-read from
            // storage; this could be improved.
            if (_mission->read_cmd_from_storage(_mission_number_to_send,cmd)) {
                if (!_logger_backend->Write_Mission_Cmd(*_mission, cmd)) {
                    return; // 再次调用我
                }
            }
            _mission_number_to_send++;
        }
        stage = Stage::DONE;
        FALLTHROUGH;
    }

    case Stage::DONE:
        break;
    }

    _finished = true;
}

// 重置任务写入器
void LoggerMessageWriter_WriteEntireMission::reset()
{
    LoggerMessageWriter::reset();
    stage = Stage::WRITE_NEW_MISSION_MESSAGE;
    _mission_number_to_send = 0;
}

#if HAL_LOGGER_FENCE_ENABLED
#if APM_BUILD_TYPE(APM_BUILD_Replay)
// 用于Replay构建的虚拟方法
// dummy methods to allow build with Replay
void LoggerMessageWriter_Write_Polyfence::process() { };
void LoggerMessageWriter_Write_Polyfence::reset() { };
#else
// 处理围栏写入
void LoggerMessageWriter_Write_Polyfence::process() {

    AC_Fence *fence = AP::fence();
    if (fence == nullptr) {
        _finished = true;
        return;
    }

    switch(stage) {

    case Stage::WRITE_NEW_FENCE_MESSAGE:
        if (!_logger_backend->Write_Message("New fence")) {
            return; // 再次调用我
        }
        stage = Stage::WRITE_FENCE_ITEMS;
        FALLTHROUGH;

    case Stage::WRITE_FENCE_ITEMS: {
        while (_fence_number_to_send < fence->polyfence().num_stored_items()) {
            if (out_of_time_for_writing_messages()) {
                return;
            }

            // 写入围栏失败时我们将从存储重新读取;
            // 这可以改进。
            // upon failure to write the fence we will re-read from
            // storage; this could be improved.
            AC_PolyFenceItem fenceitem {};
            if (fence->polyfence().get_item(_fence_number_to_send, fenceitem)) {
                if (!_logger_backend->Write_FencePoint(fence->polyfence().num_stored_items(), _fence_number_to_send, fenceitem)) {
                    return; // 再次调用我
                }
            }
            _fence_number_to_send++;
        }
        stage = Stage::DONE;
        FALLTHROUGH;
    }

    case Stage::DONE:
        break;
    }

    _finished = true;
}

// 重置围栏写入器
void LoggerMessageWriter_Write_Polyfence::reset()
{
    LoggerMessageWriter::reset();
    stage = Stage::WRITE_NEW_FENCE_MESSAGE;
    _fence_number_to_send = 0;
}
#endif // !APM_BUILD_TYPE(APM_BUILD_Replay)
#endif // AP_FENCE_ENABLED

#endif  // HAL_LOGGING_ENABLED
