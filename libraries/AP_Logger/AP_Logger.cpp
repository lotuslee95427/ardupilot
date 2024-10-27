#include "AP_Logger.h"

#if HAL_LOGGING_ENABLED

#include "AP_Logger_Backend.h"

#include "AP_Logger_File.h"
#include "AP_Logger_Flash_JEDEC.h"
#include "AP_Logger_W25NXX.h"
#include "AP_Logger_MAVLink.h"

#include <AP_InternalError/AP_InternalError.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Rally/AP_Rally.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#if HAL_LOGGER_FENCE_ENABLED
    #include <AC_Fence/AC_Fence.h>
#endif

// 全局单例指针
AP_Logger *AP_Logger::_singleton;

extern const AP_HAL::HAL& hal;

// 定义日志文件缓冲区大小,根据内存等级设置不同大小
#ifndef HAL_LOGGING_FILE_BUFSIZE
#if HAL_MEM_CLASS >= HAL_MEM_CLASS_1000
#define HAL_LOGGING_FILE_BUFSIZE  200
#elif HAL_MEM_CLASS >= HAL_MEM_CLASS_500
#define HAL_LOGGING_FILE_BUFSIZE  80
#elif HAL_MEM_CLASS >= HAL_MEM_CLASS_300
#define HAL_LOGGING_FILE_BUFSIZE  50
#else
#define HAL_LOGGING_FILE_BUFSIZE  16
#endif
#endif

// 默认使用 JEDEC Flash 作为数据存储驱动
#ifndef HAL_LOGGING_DATAFLASH_DRIVER
#define HAL_LOGGING_DATAFLASH_DRIVER AP_Logger_Flash_JEDEC
#endif

// 日志任务栈大小
#ifndef HAL_LOGGING_STACK_SIZE
#define HAL_LOGGING_STACK_SIZE 1580
#endif

// MAVLink日志缓冲区大小
#ifndef HAL_LOGGING_MAV_BUFSIZE
#define HAL_LOGGING_MAV_BUFSIZE  8
#endif 

// 日志文件写入超时时间(秒)
#ifndef HAL_LOGGING_FILE_TIMEOUT
#define HAL_LOGGING_FILE_TIMEOUT 5
#endif 

// 默认解除武装后继续记录15秒日志
#ifndef HAL_LOGGER_ARM_PERSIST
#define HAL_LOGGER_ARM_PERSIST 15
#endif

// 根据不同平台设置默认的日志后端类型
#ifndef HAL_LOGGING_BACKENDS_DEFAULT
# if HAL_LOGGING_FILESYSTEM_ENABLED && (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#  define HAL_LOGGING_BACKENDS_DEFAULT Backend_Type::FILESYSTEM
# elif HAL_LOGGING_DATAFLASH_ENABLED
#  define HAL_LOGGING_BACKENDS_DEFAULT Backend_Type::BLOCK
# elif HAL_LOGGING_FILESYSTEM_ENABLED
#  define HAL_LOGGING_BACKENDS_DEFAULT Backend_Type::FILESYSTEM
# elif HAL_LOGGING_MAVLINK_ENABLED
#  define HAL_LOGGING_BACKENDS_DEFAULT Backend_Type::MAVLINK
# else
#  define HAL_LOGGING_BACKENDS_DEFAULT 0
# endif
#endif

// 回放模式下使用不同的动态消息ID起始值
#if APM_BUILD_TYPE(APM_BUILD_Replay)
#define LOGGING_FIRST_DYNAMIC_MSGID REPLAY_LOG_NEW_MSG_MAX
#else
#define LOGGING_FIRST_DYNAMIC_MSGID 254
#endif

// 最大和最小日志文件数量
static constexpr uint16_t MAX_LOG_FILES = 500;
static constexpr uint16_t MIN_LOG_FILES = 2;

// 参数定义
const AP_Param::GroupInfo AP_Logger::var_info[] = {
    // @Param: _BACKEND_TYPE
    // @DisplayName: AP_Logger Backend Storage type
    // @Description: 日志后端存储类型的位掩码。基于块的日志记录在SITL和带有dataflash芯片的板子上可用。可以选择多个后端。
    // @Bitmask: 0:File,1:MAVLink,2:Block
    // @User: Standard
    AP_GROUPINFO("_BACKEND_TYPE",  0, AP_Logger, _params.backend_types,       uint8_t(HAL_LOGGING_BACKENDS_DEFAULT)),

    // @Param: _FILE_BUFSIZE
    // @DisplayName: 文件和块后端最大缓冲区大小(KB)
    // @Description: 文件和块后端使用缓冲区在写入块设备前存储数据。增加此值可能减少SD卡日志中的"间隙"。缓冲区大小可能会根据可用内存减小。PixHawk至少需要4KB。最大值为64KB。
    // @User: Standard
    AP_GROUPINFO("_FILE_BUFSIZE",  1, AP_Logger, _params.file_bufsize,       HAL_LOGGING_FILE_BUFSIZE),

    // @Param: _DISARMED
    // @DisplayName: 解除武装时启用日志记录
    // @Description: 如果设为1,则在所有时间(包括解除武装时)启用日志记录。解除武装前的日志记录会产生很大的日志文件,但在追踪启动问题时很有帮助,如果通过LOG_REPLAY参数选择了EKF回放数据记录则是必需的。如果设为2,则在解除武装时启用日志记录,但在检测到USB连接时禁用。这可用于防止在通过USB连接下载日志或更改参数时生成不需要的数据日志。如果设为3,则在解除武装时记录日志,但如果飞行器从未武装,则在下次启动时丢弃使用文件系统后端的日志。
    // @Values: 0:禁用,1:启用,2:USB连接时禁用,3:如果从未武装则在重启时丢弃日志
    // @User: Standard
    AP_GROUPINFO("_DISARMED",  2, AP_Logger, _params.log_disarmed,       0),

    // @Param: _REPLAY
    // @DisplayName: 启用回放所需信息的日志记录
    // @Description: 如果设为1,则EKF2和EKF3状态估计器将记录诊断卡尔曼滤波器问题所需的详细信息。必须将LOG_DISARMED设为1或2,否则日志将不包含回放测试EKF所需的预飞行数据。建议同时增加LOG_FILE_BUFSIZE以提供更多缓冲空间用于日志记录,并使用高质量的microSD卡以确保不丢失传感器数据。
    // @Values: 0:禁用,1:启用
    // @User: Standard
    AP_GROUPINFO("_REPLAY",  3, AP_Logger, _params.log_replay,       0),

    // @Param: _FILE_DSRMROT
    // @DisplayName: 解除武装时停止记录当前日志文件
    // @Description: 设置后,当飞行器解除武装时关闭当前日志文件。如果设置了LOG_DISARMED,则会打开一个新的日志文件。适用于文件和块日志后端。
    // @Values: 0:禁用,1:启用
    // @User: Standard
    AP_GROUPINFO("_FILE_DSRMROT",  4, AP_Logger, _params.file_disarm_rot,       0),

#if HAL_LOGGING_MAVLINK_ENABLED
    // @Param: _MAV_BUFSIZE
    // @DisplayName: MAVLink后端最大缓冲区大小
    // @Description: 分配给基于MAVLink的日志记录的最大内存
    // @User: Advanced
    // @Units: kB
    AP_GROUPINFO("_MAV_BUFSIZE",  5, AP_Logger, _params.mav_bufsize,       HAL_LOGGING_MAV_BUFSIZE),
#endif

    // @Param: _FILE_TIMEOUT
    // @DisplayName: 放弃文件写入前的超时时间
    // @Description: 控制在写入日志文件失败导致文件关闭和停止日志记录之前的等待时间。
    // @User: Standard
    // @Units: s
    AP_GROUPINFO("_FILE_TIMEOUT",  6, AP_Logger, _params.file_timeout,     HAL_LOGGING_FILE_TIMEOUT),

    // @Param: _FILE_MB_FREE
    // @DisplayName: 删除旧日志以维持的SD卡空闲空间
    // @Description: 设置此值使空闲空间大于典型飞行日志的最大大小
    // @Units: MB
    // @Range: 10 1000
    // @User: Standard
    AP_GROUPINFO("_FILE_MB_FREE",  7, AP_Logger, _params.min_MB_free, 500),

    // @Param: _FILE_RATEMAX
    // @DisplayName: 文件后端最大日志记录速率
    // @Description: 设置流式日志消息写入文件后端的最大速率。值为0表示禁用速率限制。
    // @Units: Hz
    // @Range: 0 1000
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("_FILE_RATEMAX",  8, AP_Logger, _params.file_ratemax, 0),

#if HAL_LOGGING_MAVLINK_ENABLED
    // @Param: _MAV_RATEMAX
    // @DisplayName: MAVLink后端最大日志记录速率
    // @Description: 设置流式日志消息写入MAVLink后端的最大速率。值为0表示禁用速率限制。
    // @Units: Hz
    // @Range: 0 1000
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("_MAV_RATEMAX",  9, AP_Logger, _params.mav_ratemax, 0),
#endif

#if HAL_LOGGING_BLOCK_ENABLED
    // @Param: _BLK_RATEMAX
    // @DisplayName: 块后端最大日志记录速率
    // @Description: 设置流式日志消息写入块后端的最大速率。值为0表示禁用速率限制。
    // @Units: Hz
    // @Range: 0 1000
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("_BLK_RATEMAX", 10, AP_Logger, _params.blk_ratemax, 0),
#endif

    // @Param: _DARM_RATEMAX
    // @DisplayName: 解除武装时的最大日志记录速率
    // @Description: 设置解除武装时流式日志消息写入任何后端的最大速率。值为0表示应用正常的后端速率限制。
    // @Units: Hz
    // @Range: 0 1000
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("_DARM_RATEMAX",  11, AP_Logger, _params.disarm_ratemax, 0),

    // @Param: _MAX_FILES
    // @DisplayName: 最大日志文件数
    // @Description: 设置在开始轮换日志编号之前将在dataflash或sd卡上写入的最大日志文件数。限制为500个日志。
    // @Range: 2 500
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("_MAX_FILES", 12, AP_Logger, _params.max_log_files, MAX_LOG_FILES),

    AP_GROUPEND
};

#define streq(x, y) (!strcmp(x, y))

// 构造函数
AP_Logger::AP_Logger()
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Logger must be singleton");
    }

    _singleton = this;
}

// 初始化日志记录器
void AP_Logger::init(const AP_Int32 &log_bitmask, const struct LogStructure *structures, uint8_t num_types)
{
    _log_bitmask = &log_bitmask;

    // 将8位LOG_FILE_BUFSIZE转换为16位
    _params.file_bufsize.convert_parameter_width(AP_PARAM_INT8);

    // 如果是看门狗复位,强制启用日志记录
    if (hal.util->was_watchdog_armed()) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Forcing logging for watchdog reset");
        _params.log_disarmed.set(LogDisarmed::LOG_WHILE_DISARMED);
    }
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    validate_structures(structures, num_types);
    dump_structures(structures, num_types);
#endif
    _num_types = num_types;
    _structures = structures;

    // 主日志类型需要在mavlink之前,以确保索引0正确
    static const struct {
        Backend_Type type;
        AP_Logger_Backend* (*probe_fn)(AP_Logger&, LoggerMessageWriter_DFLogStart*);
    } backend_configs[] {
#if HAL_LOGGING_FILESYSTEM_ENABLED
        { Backend_Type::FILESYSTEM, AP_Logger_File::probe },
#endif
#if HAL_LOGGING_DATAFLASH_ENABLED
        { Backend_Type::BLOCK, HAL_LOGGING_DATAFLASH_DRIVER::probe },
#endif
#if HAL_LOGGING_MAVLINK_ENABLED
        { Backend_Type::MAVLINK, AP_Logger_MAVLink::probe },
#endif
};

    // 初始化后端
    for (const auto &backend_config : backend_configs) {
        if ((_params.backend_types & uint8_t(backend_config.type)) == 0) {
            continue;
        }
        if (_next_backend == LOGGER_MAX_BACKENDS) {
            AP_BoardConfig::config_error("Too many backends");
            return;
        }
        LoggerMessageWriter_DFLogStart *message_writer =
            NEW_NOTHROW LoggerMessageWriter_DFLogStart();
        if (message_writer == nullptr)  {
            AP_BoardConfig::allocation_error("message writer");
        }
        backends[_next_backend] = backend_config.probe_fn(*this, message_writer);
        if (backends[_next_backend] == nullptr) {
            AP_BoardConfig::allocation_error("logger backend");
        }
        _next_backend++;
    }

    for (uint8_t i=0; i<_next_backend; i++) {
        backends[i]->Init();
    }

    // 启动IO线程
    start_io_thread();

    EnableWrites(true);
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <stdio.h>

#define DEBUG_LOG_STRUCTURES 0

extern const AP_HAL::HAL& hal;
#define Debug(fmt, args ...)  do {::fprintf(stderr, "%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)

// 返回字符串中逗号的数量
static uint8_t count_commas(const char *string)
{
    uint8_t ret = 0;
    for (uint8_t i=0; i<strlen(string); i++) {
        if (string[i] == ',') {
            ret++;
        }
    }
    return ret;
}

// 根据ID返回单位名称
const char* AP_Logger::unit_name(const uint8_t unit_id)
{
    for (uint8_t i=0; i<_num_units; i++) {
        if (_units[i].ID == unit_id) {
            return _units[i].unit;
        }
    }
    return nullptr;
}

// 根据ID返回乘数值
double AP_Logger::multiplier_name(const uint8_t multiplier_id)
{
    for (uint8_t i=0; i<_num_multipliers; i++) {
        if (_multipliers[i].ID == multiplier_id) {
            return _multipliers[i].multiplier;
        }
    }
    // 是否应该在这里中止?
    return 1.0f;
}

// 打印日志结构中的字段信息
void AP_Logger::dump_structure_field(const struct LogStructure *logstructure, const char *label, const uint8_t fieldnum)
{
    ::fprintf(stderr, "  %s (%s)*(%f)\n", label, unit_name(logstructure->units[fieldnum]), multiplier_name(logstructure->multipliers[fieldnum]));
}

// 打印日志结构
// 注意:结构必须格式良好
void AP_Logger::dump_structures(const struct LogStructure *logstructures, const uint8_t num_types)
{
#if DEBUG_LOG_STRUCTURES
    for (uint16_t i=0; i<num_types; i++) {
        const struct LogStructure *logstructure = &logstructures[i];
        ::fprintf(stderr, "%s\n", logstructure->name);
        char label[32] = { };
        uint8_t labeloffset = 0;
        int8_t fieldnum = 0;
        for (uint8_t j=0; j<strlen(logstructure->labels); j++) {
            char labelchar = logstructure->labels[j];
            if (labelchar == '\0') {
                break;
            }
            if (labelchar == ',') {
                dump_structure_field(logstructure, label, fieldnum);
                fieldnum++;
                labeloffset = 0;
                memset(label, '\0', 32);
            } else {
                label[labeloffset++] = labelchar;
            }
        }
        dump_structure_field(logstructure, label, fieldnum);
        ::fprintf(stderr, "\n"); // 只是在输出中添加一个回车
    }
#endif
}

// 检查标签字符串是否合法
bool AP_Logger::labels_string_is_good(const char *labels) const
{
    bool passed = true;
    if (strlen(labels) >= LS_LABELS_SIZE) {
        Debug("Labels string too long (%u > %u)", unsigned(strlen(labels)), unsigned(LS_LABELS_SIZE));
        passed = false;
    }
    // 通过将逗号改为空字符来切分标签字符串
    // 在label_offsets中保持对每个字符串的引用
    char *label_offsets[LS_LABELS_SIZE];
    uint8_t label_offsets_offset = 0;
    char labels_copy[LS_LABELS_SIZE+1] {};
    strncpy(labels_copy, labels, LS_LABELS_SIZE);
    if (labels_copy[0] == ',') {
        Debug("Leading comma in (%s)", labels);
        passed = false;
    }
    label_offsets[label_offsets_offset++] = labels_copy;
    const uint8_t len = strnlen(labels_copy, LS_LABELS_SIZE);
    for (uint8_t i=0; i<len; i++) {
        if (labels_copy[i] == ',') {
            if (labels_copy[i+1] == '\0') {
                Debug("Trailing comma in (%s)", labels);
                passed = false;
                continue;
            }
            labels_copy[i] = '\0';
            label_offsets[label_offsets_offset++] = &labels_copy[i+1];
        }
    }
    for (uint8_t i=0; i<label_offsets_offset-1; i++) {
        for (uint8_t j=i+1; j<label_offsets_offset; j++) {
            if (!strcmp(label_offsets[i], label_offsets[j])) {
                Debug("Duplicate label (%s) in (%s)", label_offsets[i], labels);
                passed = false;
            }
        }
    }
    return passed;
}

// 验证日志结构
bool AP_Logger::validate_structure(const struct LogStructure *logstructure, const int16_t offset)
{
    bool passed = true;

#if DEBUG_LOG_STRUCTURES
    Debug("offset=%d ID=%d NAME=%s", offset, logstructure->msg_type, logstructure->name);
#endif

    // 字段必须以空字符结尾
#define CHECK_ENTRY(fieldname,fieldname_s,fieldlen)                     \
    do {                                                                \
        if (strnlen(logstructure->fieldname, fieldlen) > fieldlen-1) {  \
            Debug("  Message %s." fieldname_s " not NULL-terminated or too long", logstructure->name); \
            passed = false;                                             \
        }                                                               \
    } while (false)
    CHECK_ENTRY(name, "name", LS_NAME_SIZE);
    CHECK_ENTRY(format, "format", LS_FORMAT_SIZE);
    CHECK_ENTRY(labels, "labels", LS_LABELS_SIZE);
    CHECK_ENTRY(units, "units", LS_UNITS_SIZE);
    CHECK_ENTRY(multipliers, "multipliers", LS_MULTIPLIERS_SIZE);
#undef CHECK_ENTRY

    // 确保每个消息ID只使用一次
    if (seen_ids[logstructure->msg_type]) {
        Debug("  ID %d used twice (LogStructure offset=%d)", logstructure->msg_type, offset);
        passed = false;
    }
    seen_ids[logstructure->msg_type] = true;

    // 确保有足够的标签覆盖所有列
    uint8_t fieldcount = strlen(logstructure->format);
    uint8_t labelcount = count_commas(logstructure->labels)+1;
    if (fieldcount != labelcount) {
        Debug("  %s fieldcount=%u does not match labelcount=%u",
              logstructure->name, fieldcount, labelcount);
        passed = false;
    }

    if (!labels_string_is_good(logstructure->labels)) {
        passed = false;
    }

    // 检查结构长度是否适合容纳字段
    const int16_t msg_len = Write_calc_msg_len(logstructure->format);
    if (msg_len != logstructure->msg_len) {
        Debug("  Calculated message length for (%s) based on format field (%s) does not match structure size (%d != %u)", logstructure->name, logstructure->format, msg_len, logstructure->msg_len);
        passed = false;
    }

    // 确保每个字段都有单位
    if (strlen(logstructure->units) != fieldcount) {
        Debug("  %s fieldcount=%u does not match unitcount=%u",
              logstructure->name, (unsigned)fieldcount, (unsigned)strlen(logstructure->units));
        passed = false;
    }

    // 确保每个字段都有乘数
    if (strlen(logstructure->multipliers) != fieldcount) {
        Debug("  %s fieldcount=%u does not match multipliercount=%u",
              logstructure->name, (unsigned)fieldcount, (unsigned)strlen(logstructure->multipliers));
        passed = false;
    }

    // 确保FMTU消息引用有效的单位
    for (uint8_t j=0; j<strlen(logstructure->units); j++) {
        char logunit = logstructure->units[j];
        uint8_t k;
        for (k=0; k<_num_units; k++) {
            if (logunit == _units[k].ID) {
                // 找到这个单位
                break;
            }
        }
        if (k == _num_units) {
            Debug("  invalid unit=%c", logunit);
            passed = false;
        }
    }

    // 确保FMTU消息引用有效的乘数
    for (uint8_t j=0; j<strlen(logstructure->multipliers); j++) {
        char logmultiplier = logstructure->multipliers[j];
        uint8_t k;
        for (k=0; k<_num_multipliers; k++) {
            if (logmultiplier == _multipliers[k].ID) {
                // 找到这个乘数
                break;
            }
        }
        if (k == _num_multipliers) {
            Debug("  invalid multiplier=%c", logmultiplier);
            passed = false;
        }
    }

    // 确保任何浮点数都有零乘数
    if (false && passed) {
        for (uint8_t j=0; j<strlen(logstructure->multipliers); j++) {
            const char fmt = logstructure->format[j];
            if (fmt != 'f' && fmt != 'd' && fmt != 'g') {
                continue;
            }
            const char logmultiplier = logstructure->multipliers[j];
            if (logmultiplier == '0' ||
                logmultiplier == '?' ||
                logmultiplier == '-') {
                continue;
            }
            Debug("  %s[%u] float with non-zero multiplier=%c",
                  logstructure->name,
                  j,
                  logmultiplier);
            passed = false;
        }
    }

    return passed;
}

// 验证所有日志结构
void AP_Logger::validate_structures(const struct LogStructure *logstructures, const uint8_t num_types)
{
    Debug("Validating structures");
    bool passed = true;

    for (uint16_t i=0; i<num_types; i++) {
        const struct LogStructure *logstructure = &logstructures[i];
        passed = validate_structure(logstructure, i) && passed;
    }

    // 确保单位是唯一的
    for (uint16_t i=0; i<ARRAY_SIZE(log_Units); i++) {
        const struct UnitStructure &a = log_Units[i];
        for (uint16_t j=i+1; j<ARRAY_SIZE(log_Units); j++) {
            const struct UnitStructure &b = log_Units[j];
            if (a.ID == b.ID) {
                Debug("duplicate unit id=%c (%s/%s)", a.ID, a.unit, b.unit);
                passed = false;
            }
            if (streq(a.unit, b.unit)) {
                Debug("duplicate unit=%s (%c/%c)", a.unit, a.ID, b.ID);
                passed = false;
            }
        }
    }

    // 确保乘数是唯一的
    for (uint16_t i=0; i<ARRAY_SIZE(log_Multipliers); i++) {
        const struct MultiplierStructure &a = log_Multipliers[i];
        for (uint16_t j=i+1; j<ARRAY_SIZE(log_Multipliers); j++) {
            const struct MultiplierStructure &b = log_Multipliers[j];
            if (a.ID == b.ID) {
                Debug("duplicate multiplier id=%c (%f/%f)",
                      a.ID, a.multiplier, b.multiplier);
                passed = false;
            }
            if (is_equal(a.multiplier, b.multiplier)) {
                if (a.ID == '?' && b.ID == '0') {
                    // 特殊情况
                    continue;
                }
                Debug("duplicate multiplier=%f (%c/%c)",
                      a.multiplier, a.ID, b.ID);
                passed = false;
            }
        }
    }

    if (!passed) {
        AP_BoardConfig::config_error("See console: Log structures invalid");
    }
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL

// 返回指定编号的日志结构
const struct LogStructure *AP_Logger::structure(uint16_t num) const
{
    return &_structures[num];
}

// 检查是否存在日志记录器
bool AP_Logger::logging_present() const
{
    return _next_backend != 0;
}

// 检查日志记录是否启用
bool AP_Logger::logging_enabled() const
{
    if (_next_backend == 0) {
        return false;
    }
    for (uint8_t i=0; i<_next_backend; i++) {
        if (backends[i]->logging_enabled()) {
            return true;
        }
    }
    return false;
}

// 检查日志记录是否失败
bool AP_Logger::logging_failed() const
{
    if (_next_backend < 1) {
        // 不应该调用这个函数!
        return true;
    }
    for (uint8_t i=0; i<_next_backend; i++) {
        if (backends[i]->logging_failed()) {
            return true;
        }
    }
    return false;
}

// 写入格式化消息
// Write a formatted message
void AP_Logger::Write_MessageF(const char *fmt, ...)
{
    char msg[65] {}; // sizeof(log_Message.msg) + null-termination

    va_list ap;
    va_start(ap, fmt);
    hal.util->vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);

    Write_Message(msg);
}

// 后端开始新的日志记录时调用
// Called when a backend starts a new log
void AP_Logger::backend_starting_new_log(const AP_Logger_Backend *backend)
{
    _log_start_count++;
}

// 检查是否应该记录日志
// Check if we should log based on mask and armed state
bool AP_Logger::should_log(const uint32_t mask) const
{
    bool armed = vehicle_is_armed();

    // 检查掩码是否匹配
    // Check if mask matches log bitmask
    if (!(mask & *_log_bitmask)) {
        return false;
    }
    // 检查未解锁时是否允许记录日志
    // Check if we can log while disarmed
    if (!armed && !log_while_disarmed()) {
        return false;
    }
    // 检查是否正在下载日志
    // Check if we are downloading logs
    if (in_log_download()) {
        return false;
    }
    // 检查是否有后端
    // Check if we have any backends
    if (_next_backend == 0) {
        return false;
    }
    return true;
}

/*
  检查是否正在下载日志,这会阻止日志记录
  return true if in log download which should prevent logging
 */
bool AP_Logger::in_log_download() const
{
    if (uint8_t(_params.backend_types) & uint8_t(Backend_Type::BLOCK)) {
        // 当有BLOCK后端时,列举完全阻止日志记录
        // when we have a BLOCK backend then listing completely prevents logging
        return transfer_activity != TransferActivity::IDLE;
    }
    // 对于其他后端,列举不会干扰日志记录
    // for other backends listing does not interfere with logging
    return transfer_activity == TransferActivity::SENDING;
}

// 返回指定编号的单位结构
// Return unit structure for given number
const struct UnitStructure *AP_Logger::unit(uint16_t num) const
{
    return &_units[num];
}

// 返回指定编号的乘数结构
// Return multiplier structure for given number
const struct MultiplierStructure *AP_Logger::multiplier(uint16_t num) const
{
    return &log_Multipliers[num];
}

// 对每个后端执行指定方法的宏
// Macro to execute method on each backend
#define FOR_EACH_BACKEND(methodcall)              \
    do {                                          \
        for (uint8_t i=0; i<_next_backend; i++) { \
            backends[i]->methodcall;              \
        }                                         \
    } while (0)

// 准备解锁
// Prepare for arming
void AP_Logger::PrepForArming()
{
    FOR_EACH_BACKEND(PrepForArming());
}

// 设置飞行器启动消息写入器
// Set vehicle startup message writer
void AP_Logger::setVehicle_Startup_Writer(vehicle_startup_message_Writer writer)
{
    _vehicle_messages = writer;
}

// 设置飞行器解锁状态
// Set vehicle armed state
void AP_Logger::set_vehicle_armed(const bool armed_state)
{
    if (armed_state == _armed) {
        // 状态未改变
        // no change in status
        return;
    }
    _armed = armed_state;

    if (_armed) {
         // 从未解锁变为解锁状态
         // went from disarmed to armed
#if HAL_LOGGER_FILE_CONTENTS_ENABLED
        // 记录@SYS文件集:
        // get a set of @SYS files logged:
        file_content_prepare_for_arming = true;
#endif
    } else {
        // 从解锁变为未解锁状态
        // went from armed to disarmed
        FOR_EACH_BACKEND(vehicle_was_disarmed());
    }
}

#if APM_BUILD_TYPE(APM_BUILD_Replay)
/*
  记住回放的格式。这允许WriteV()在回放中工作
  remember formats for replay. This allows WriteV() to work within
  replay
*/
void AP_Logger::save_format_Replay(const void *pBuffer)
{
    if (((uint8_t *)pBuffer)[2] == LOG_FORMAT_MSG) {
        struct log_Format *fmt = (struct log_Format *)pBuffer;
        struct log_write_fmt *f = NEW_NOTHROW log_write_fmt;
        f->msg_type = fmt->type;
        f->msg_len = fmt->length;
        f->name = strndup(fmt->name, sizeof(fmt->name));
        f->fmt = strndup(fmt->format, sizeof(fmt->format));
        f->labels = strndup(fmt->labels, sizeof(fmt->labels));
        f->next = log_write_fmts;
        log_write_fmts = f;
    }
}
#endif


// 开始函数直接传递给后端:
// start functions pass straight through to backend:
void AP_Logger::WriteBlock(const void *pBuffer, uint16_t size) {
#if APM_BUILD_TYPE(APM_BUILD_Replay)
    save_format_Replay(pBuffer);
#endif
    FOR_EACH_BACKEND(WriteBlock(pBuffer, size));
}

// 只需要第一个后端写入成功即可
// only the first backend write need succeed for us to be successful
bool AP_Logger::WriteBlock_first_succeed(const void *pBuffer, uint16_t size) 
{
    if (_next_backend == 0) {
        return false;
    }
    
    for (uint8_t i=1; i<_next_backend; i++) {
        backends[i]->WriteBlock(pBuffer, size);
    }

    return backends[0]->WriteBlock(pBuffer, size);
}

// 写入回放块。如果后端没有空间存储消息,则返回false
// write a replay block. This differs from other as it returns false if a backend doesn't
// have space for the msg
bool AP_Logger::WriteReplayBlock(uint8_t msg_id, const void *pBuffer, uint16_t size) {
    bool ret = true;
    if (log_replay()) {
        uint8_t buf[3+size];
        buf[0] = HEAD_BYTE1;
        buf[1] = HEAD_BYTE2;
        buf[2] = msg_id;
        memcpy(&buf[3], pBuffer, size);
        for (uint8_t i=0; i<_next_backend; i++) {
            if (!backends[i]->WritePrioritisedBlock(buf, sizeof(buf), true)) {
                ret = false;
            }
        }
    }
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // 情况可能会变糟。但是,如果我们在未解锁时不记录日志,
    // 那么EKF可以启动并尝试记录日志,即使后端可能说"不"。
    // things will almost certainly go sour.  However, if we are not
    // logging while disarmed then the EKF can be started and trying
    // to log things even 'though the backends might be saying "no".
    if (!ret && log_while_disarmed()) {
        AP_HAL::panic("Failed to log replay block");
    }
#endif
    return ret;
}

// 写入关键块
// Write critical block
void AP_Logger::WriteCriticalBlock(const void *pBuffer, uint16_t size) {
    FOR_EACH_BACKEND(WriteCriticalBlock(pBuffer, size));
}

// 写入优先级块
// Write prioritised block
void AP_Logger::WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical) {
    FOR_EACH_BACKEND(WritePrioritisedBlock(pBuffer, size, is_critical));
}

// 擦除所有日志
// change me to "DoTimeConsumingPreparations"?
void AP_Logger::EraseAll() {
    FOR_EACH_BACKEND(EraseAll());
}

// 检查SD卡是否插入
// change me to "LoggingAvailable"?
bool AP_Logger::CardInserted(void) {
    for (uint8_t i=0; i< _next_backend; i++) {
        if (backends[i]->CardInserted()) {
            return true;
        }
    }
    return false;
}

// 停止日志记录
// Stop logging
void AP_Logger::StopLogging()
{
    FOR_EACH_BACKEND(stop_logging());
}

// 查找最后一个日志编号
// Find last log number
uint16_t AP_Logger::find_last_log() const {
    if (_next_backend == 0) {
        return 0;
    }
    return backends[0]->find_last_log();
}

// 获取日志边界
// Get log boundaries
void AP_Logger::get_log_boundaries(uint16_t log_num, uint32_t & start_page, uint32_t & end_page) {
    if (_next_backend == 0) {
        return;
    }
    backends[0]->get_log_boundaries(log_num, start_page, end_page);
}

// 获取日志信息
// Get log info
void AP_Logger::get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc) {
    if (_next_backend == 0) {
        return;
    }
    backends[0]->get_log_info(log_num, size, time_utc);
}

// 获取日志数据
// Get log data
int16_t AP_Logger::get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) {
    if (_next_backend == 0) {
        return 0;
    }
    return backends[0]->get_log_data(log_num, page, offset, len, data);
}

// 获取日志数量
// Get number of logs
uint16_t AP_Logger::get_num_logs(void) {
    if (_next_backend == 0) {
        return 0;
    }
    return backends[0]->get_num_logs();
}

// 获取最大日志文件数量
// Get maximum number of log files
uint16_t AP_Logger::get_max_num_logs() {
    const auto max_logs = constrain_uint16(_params.max_log_files.get(), MIN_LOG_FILES, MAX_LOG_FILES);
    if (_params.max_log_files.get() != max_logs) {
        _params.max_log_files.set_and_save_ifchanged(static_cast<int16_t>(max_logs));
    }
    return static_cast<uint16_t>(_params.max_log_files.get());
}

/* 如果任何后端已启动,则表示已启动 */
/* we're started if any of the backends are started */
bool AP_Logger::logging_started(void) {
    for (uint8_t i=0; i< _next_backend; i++) {
        if (backends[i]->logging_started()) {
            return true;
        }
    }
    return false;
}

// 处理MAVLink消息
// Handle MAVLink message
void AP_Logger::handle_mavlink_msg(GCS_MAVLINK &link, const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS:
        FOR_EACH_BACKEND(remote_log_block_status_msg(link, msg));
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
        FALLTHROUGH;
    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
        FALLTHROUGH;
    case MAVLINK_MSG_ID_LOG_ERASE:
        FALLTHROUGH;
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
        handle_log_message(link, msg);
        break;
    }
}

// 执行周期性任务
// Perform periodic tasks
void AP_Logger::periodic_tasks() {
#ifndef HAL_BUILD_AP_PERIPH
    handle_log_send();
#endif
    FOR_EACH_BACKEND(periodic_tasks());
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    // 目前只有AP_Logger_File支持这个:
    // currently only AP_Logger_File support this:
void AP_Logger::flush(void) {
     FOR_EACH_BACKEND(flush());
}
#endif


// 写入整个任务
// Write entire mission
void AP_Logger::Write_EntireMission()
{
    FOR_EACH_BACKEND(Write_EntireMission());
}

// 写入消息
// Write message
void AP_Logger::Write_Message(const char *message)
{
    FOR_EACH_BACKEND(Write_Message(message));
}

// 写入模式
// Write mode
void AP_Logger::Write_Mode(uint8_t mode, const ModeReason reason)
{
    FOR_EACH_BACKEND(Write_Mode(mode, reason));
}

// 写入参数
// Write parameter
void AP_Logger::Write_Parameter(const char *name, float value)
{
    FOR_EACH_BACKEND(Write_Parameter(name, value, quiet_nanf()));
}

// 写入任务命令
// Write mission command
void AP_Logger::Write_Mission_Cmd(const AP_Mission &mission,
                                            const AP_Mission::Mission_Command &cmd)
{
    FOR_EACH_BACKEND(Write_Mission_Cmd(mission, cmd));
}

#if HAL_RALLY_ENABLED
// 写入集结点
// Write rally point
void AP_Logger::Write_RallyPoint(uint8_t total,
                                 uint8_t sequence,
                                 const RallyLocation &rally_point)
{
    FOR_EACH_BACKEND(Write_RallyPoint(total, sequence, rally_point));
}

// 写入集结点
// Write rally
void AP_Logger::Write_Rally()
{
    FOR_EACH_BACKEND(Write_Rally());
}
#endif

#if HAL_LOGGER_FENCE_ENABLED
// 写入围栏
// Write fence
void AP_Logger::Write_Fence()
{
    FOR_EACH_BACKEND(Write_Fence());
}
#endif

// 写入命名的浮点值
// Write named value float
void AP_Logger::Write_NamedValueFloat(const char *name, float value)
{
    WriteStreaming(
        "NVF",
        "TimeUS,Name,Value",
        "s#-",
        "F--",
        "QNf",
        AP_HAL::micros(),
        name,
        value
        );
}

// 为每个后端输出FMT消息(如果尚未输出)
// output a FMT message for each backend if not already done so
void AP_Logger::Safe_Write_Emit_FMT(log_write_fmt *f)
{
    for (uint8_t i=0; i<_next_backend; i++) {
        backends[i]->Safe_Write_Emit_FMT(f->msg_type);
    }
}

// 返回丢弃的消息数量
// Return number of dropped messages
uint32_t AP_Logger::num_dropped() const
{
    if (_next_backend == 0) {
        return 0;
    }
    return backends[0]->num_dropped();
}


// 结束函数直接传递给后端
// end functions pass straight through to backend

/* Write support */
void AP_Logger::Write(const char *name, const char *labels, const char *fmt, ...)
{
    va_list arg_list;

    va_start(arg_list, fmt);
    WriteV(name, labels, nullptr, nullptr, fmt, arg_list);
    va_end(arg_list);
}

void AP_Logger::Write(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, ...)
{
    va_list arg_list;

    va_start(arg_list, fmt);
    WriteV(name, labels, units, mults, fmt, arg_list);
    va_end(arg_list);
}

void AP_Logger::WriteStreaming(const char *name, const char *labels, const char *fmt, ...)
{
    va_list arg_list;

    va_start(arg_list, fmt);
    WriteV(name, labels, nullptr, nullptr, fmt, arg_list, false, true);
    va_end(arg_list);
}

void AP_Logger::WriteStreaming(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, ...)
{
    va_list arg_list;

    va_start(arg_list, fmt);
    WriteV(name, labels, units, mults, fmt, arg_list, false, true);
    va_end(arg_list);
}

void AP_Logger::WriteCritical(const char *name, const char *labels, const char *fmt, ...)
{
    va_list arg_list;

    va_start(arg_list, fmt);
    WriteV(name, labels, nullptr, nullptr, fmt, arg_list, true);
    va_end(arg_list);
}

void AP_Logger::WriteCritical(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, ...)
{
    va_list arg_list;

    va_start(arg_list, fmt);
    WriteV(name, labels, units, mults, fmt, arg_list, true);
    va_end(arg_list);
}

// 写入V格式日志
// Write V format log
void AP_Logger::WriteV(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, va_list arg_list,
                       bool is_critical, bool is_streaming)
{
    // WriteV在回放中不安全,因为我们可能重用ID
    // WriteV is not safe in replay as we can re-use IDs
    const bool direct_comp = APM_BUILD_TYPE(APM_BUILD_Replay);
    struct log_write_fmt *f = msg_fmt_for_name(name, labels, units, mults, fmt, direct_comp);
    if (f == nullptr) {
        // 无法将名称映射到消息类型;可能是消息类型用完了,可能是槽用完了,...
        // unable to map name to a messagetype; could be out of
        // msgtypes, could be out of slots, ...
#if !APM_BUILD_TYPE(APM_BUILD_Replay)
        INTERNAL_ERROR(AP_InternalError::error_t::logger_mapfailure);
#endif
        return;
    }

    for (uint8_t i=0; i<_next_backend; i++) {
        va_list arg_copy;
        va_copy(arg_copy, arg_list);
        backends[i]->Write(f->msg_type, arg_copy, is_critical, is_streaming);
        va_end(arg_copy);
    }
}

/*
  当我们进行回放日志记录时,我们希望延迟EKF的启动,直到标头输出完毕,
  这样在回放时所有参数值都可用
  when we are doing replay logging we want to delay start of the EKF
  until after the headers are out so that on replay all parameter
  values are available
 */
bool AP_Logger::allow_start_ekf() const
{
    if (!log_replay() || !log_while_disarmed()) {
        return true;
    }

    for (uint8_t i=0; i<_next_backend; i++) {
        if (!backends[i]->allow_start_ekf()) {
            return false;
        }
    }

    return true;
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
// 断言名称的格式相同
// Assert format is same for name
bool AP_Logger::assert_same_fmt_for_name(const AP_Logger::log_write_fmt *f,
                                               const char *name,
                                               const char *labels,
                                               const char *units,
                                               const char *mults,
                                               const char *fmt) const
{
    bool passed = true;
    if (!streq(f->name, name)) {
        // 为什么要调用我们?!
        // why exactly were we called?!
        Debug("format names differ (%s) != (%s)", f->name, name);
        passed = false;
    }
    if (!streq(f->labels, labels)) {
        Debug("format labels differ (%s) vs (%s)", f->labels, labels);
        passed = false;
    }
    if (!streq(f->fmt, fmt)) {
        Debug("format fmt differ (%s) vs (%s)",
              (f->fmt ? f->fmt : "nullptr"),
              (fmt ? fmt : "nullptr"));
        passed = false;
    }
#if !APM_BUILD_TYPE(APM_BUILD_Replay)
    if ((f->units != nullptr && units == nullptr) ||
        (f->units == nullptr && units != nullptr) ||
        (units !=nullptr && !streq(f->units, units))) {
        Debug("format units differ (%s) vs (%s)",
              (f->units ? f->units : "nullptr"),
              (units ? units : "nullptr"));
        passed = false;
    }
    if ((f->mults != nullptr && mults == nullptr) ||
        (f->mults == nullptr && mults != nullptr) ||
        (mults != nullptr && !streq(f->mults, mults))) {
        Debug("format mults differ (%s) vs (%s)",
              (f->mults ? f->mults : "nullptr"),
              (mults ? mults : "nullptr"));
        passed = false;
    }
    if (!passed) {
        AP_BoardConfig::config_error("See console: Format definition must be consistent for every call of Write");
    }
#endif
    return passed;
}
#endif

// 获取名称的消息格式
// Get message format for name
AP_Logger::log_write_fmt *AP_Logger::msg_fmt_for_name(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, const bool direct_comp, const bool copy_strings)
{
    WITH_SEMAPHORE(log_write_fmts_sem);
    struct log_write_fmt *f;
    for (f = log_write_fmts; f; f=f->next) {
        if (!direct_comp) {
            if (f->name == name) { // 指针比较 // ptr comparison
                // 已经有这个名称的ID:
                // already have an ID for this name:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                if (!assert_same_fmt_for_name(f, name, labels, units, mults, fmt)) {
                    return nullptr;
                }
#endif
                return f;
            }
        } else {
            // 从脚本使用直接比较,其中不维护指针
            // direct comparison used from scripting where pointer is not maintained
            if (strcmp(f->name,name) == 0) {
                // 已经有这个名称的ID:
                // already have an ID for this name:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                if (!assert_same_fmt_for_name(f, name, labels, units, mults, fmt)) {
                    return nullptr;
                }
#endif
                return f;
            }
        }
    }

    f = (struct log_write_fmt *)calloc(1, sizeof(*f));
    if (f == nullptr) {
        // 内存不足
        // out of memory
        return nullptr;
    }
    // 没有为此名称分配消息类型。尝试分配一个:
    // no message type allocated for this name.  Try to allocate one:
    int16_t msg_type = find_free_msg_type();
    if (msg_type == -1) {
        free(f);
        return nullptr;
    }
    f->msg_type = msg_type;

    if (copy_strings) {
        // 不能使用可能移动的内存指针,必须分配和复制
        // cannot use pointers to memory that might move, must allocate and copy
        struct log_write_fmt_strings *ls_copy = (struct log_write_fmt_strings*)malloc(sizeof(log_write_fmt_strings));
        if (ls_copy == nullptr) {
            free(f);
            return nullptr;
        }

        strncpy_noterm(ls_copy->name, name, sizeof(ls_copy->name));
        strncpy_noterm(ls_copy->format, fmt, sizeof(ls_copy->format));
        strncpy_noterm(ls_copy->labels, labels, sizeof(ls_copy->labels));

        f->name = ls_copy->name;
        f->fmt = ls_copy->format;
        f->labels = ls_copy->labels;

        if (units != nullptr) {
            strncpy_noterm(ls_copy->units, units, sizeof(ls_copy->units));
            f->units = ls_copy->units;
        }

        if (mults != nullptr) {
            strncpy_noterm(ls_copy->multipliers, mults, sizeof(ls_copy->multipliers));
            f->mults = ls_copy->multipliers;
        }

    } else {
        f->name = name;
        f->fmt = fmt;
        f->labels = labels;
        f->units = units;
        f->mults = mults;
    }

    int16_t tmp = Write_calc_msg_len(fmt);
    if (tmp == -1) {
        free(f);
        return nullptr;
    }

    f->msg_len = tmp;

    // 将direct_comp格式添加到列表开头,否则添加到末尾,这可以最小化将来调用时遍历列表的字符串比较次数
    // add direct_comp formats to start of list, otherwise add to the end, this minimises the number of string comparisons when walking the list in future calls
    if (direct_comp || (log_write_fmts == nullptr)) {
        f->next = log_write_fmts;
        log_write_fmts = f;
    } else {
        struct log_write_fmt *list_end = log_write_fmts;
        while (list_end->next) {
            list_end=list_end->next;
        }
        list_end->next = f;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // 创建一个临时的日志格式字符串结构体
    struct log_write_fmt_strings ls_strings = {};
    // 创建一个临时的日志结构体
    struct LogStructure ls = {
        f->msg_type,
        f->msg_len,
        ls_strings.name,
        ls_strings.format,
        ls_strings.labels,
        ls_strings.units,
        ls_strings.multipliers
    };
    // 复制名称字符串,使用MIN防止缓冲区溢出
    memcpy((char*)ls_strings.name, f->name, MIN(sizeof(ls_strings.name), strlen(f->name)));
    // 复制格式字符串
    memcpy((char*)ls_strings.format, f->fmt, MIN(sizeof(ls_strings.format), strlen(f->fmt)));
    // 复制标签字符串
    memcpy((char*)ls_strings.labels, f->labels, MIN(sizeof(ls_strings.labels), strlen(f->labels)));
    // 如果有单位字符串,则复制单位字符串,否则用'?'填充
    if (f->units != nullptr) {
        memcpy((char*)ls_strings.units, f->units, MIN(sizeof(ls_strings.units), strlen(f->units)));
    } else {
        memset((char*)ls_strings.units, '?', MIN(sizeof(ls_strings.format), strlen(f->fmt)));
    }
    // 如果有乘数字符串,则复制乘数字符串,否则用'?'填充
    if (f->mults != nullptr) {
        memcpy((char*)ls_strings.multipliers, f->mults, MIN(sizeof(ls_strings.multipliers), strlen(f->mults)));
    } else {
        memset((char*)ls_strings.multipliers, '?', MIN(sizeof(ls_strings.format), strlen(f->fmt)));
    }
    // 验证日志结构的有效性,如果无效则显示错误信息
    if (!validate_structure(&ls, (int16_t)-1)) {
        AP_BoardConfig::config_error("See console: Log structure invalid");
    }
#endif

    return f;
}

/*
  检查是否需要保存崩溃转储。如果没有可用的崩溃转储或已成功保存到SD卡,则返回true。
  此函数会持续调用直到成功,以处理microSD卡延迟挂载的情况
 */
bool AP_Logger::check_crash_dump_save(void)
{
    // 尝试打开系统崩溃转储文件
    int fd = AP::FS().open("@SYS/crash_dump.bin", O_RDONLY);
    if (fd == -1) {
        // 没有崩溃转储文件。@SYS文件系统对空文件的open返回-1
        return true;
    }
    // 尝试在SD卡上创建新文件
    int fd2 = AP::FS().open("APM/crash_dump.bin", O_WRONLY|O_CREAT|O_TRUNC);
    if (fd2 == -1) {
        // SD卡尚未就绪,稍后重试
        AP::FS().close(fd);
        return false;
    }
    // 分块读取并写入新文件
    uint8_t buf[128];
    int32_t n;
    while ((n = AP::FS().read(fd, buf, sizeof(buf))) > 0) {
        AP::FS().write(fd2, buf, n);
    }
    AP::FS().close(fd2);
    AP::FS().close(fd);
    // 发送保存成功的通知
    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Saved crash_dump.bin");
    return true;
}

// IO处理线程 - 通常IO操作涉及对SPI设备的长时间阻塞DMA写入
// 线程在此期间会休眠,防止其他任务运行,因此需要在独立线程中运行IO操作
void AP_Logger::io_thread(void)
{
    uint32_t last_run_us = AP_HAL::micros();
    uint32_t last_stack_us = last_run_us;
    uint32_t last_crash_check_us = last_run_us;
    bool done_crash_dump_save = false;

    while (true) {
        uint32_t now = AP_HAL::micros();

        // 始终保持一定的延迟
        uint32_t delay = 250U; 
        if (now - last_run_us < 1000) {
            delay = MAX(1000 - (now - last_run_us), delay);
        }
        hal.scheduler->delay_microseconds(delay);

        last_run_us = AP_HAL::micros();

        // 调用每个后端的io_timer
        FOR_EACH_BACKEND(io_timer());

        // 每100ms记录一次栈信息
        if (now - last_stack_us > 100000U) {
            last_stack_us = now;
            hal.util->log_stack_info();
        }

        // 每5秒检查一次是否需要保存崩溃转储文件
        if (!done_crash_dump_save &&
            now - last_crash_check_us > 5000000U) {
            last_crash_check_us = now;
            done_crash_dump_save = check_crash_dump_save();
        }
#if HAL_LOGGER_FILE_CONTENTS_ENABLED
        file_content_update();
#endif
    }
}

// 启动更新线程
void AP_Logger::start_io_thread(void)
{
    WITH_SEMAPHORE(_log_send_sem);

    // 如果线程已启动则直接返回
    if (_io_thread_started) {
        return;
    }

    // 创建IO线程
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Logger::io_thread, void), "log_io", HAL_LOGGING_STACK_SIZE, AP_HAL::Scheduler::PRIORITY_IO, 1)) {
        AP_HAL::panic("Failed to start Logger IO thread");
    }

    _io_thread_started = true;
    return;
}

/* Write支持结束 */

#undef FOR_EACH_BACKEND

// 写入事件数据包
void AP_Logger::Write_Event(LogEvent id)
{
    const struct log_Event pkt{
        LOG_PACKET_HEADER_INIT(LOG_EVENT_MSG),
        time_us  : AP_HAL::micros64(),
        id       : (uint8_t)id
    };
    WriteCriticalBlock(&pkt, sizeof(pkt));
}

// 写入错误数据包
void AP_Logger::Write_Error(LogErrorSubsystem sub_system,
                            LogErrorCode error_code)
{
  const struct log_Error pkt{
      LOG_PACKET_HEADER_INIT(LOG_ERROR_MSG),
      time_us       : AP_HAL::micros64(),
      sub_system    : uint8_t(sub_system),
      error_code    : uint8_t(error_code),
  };
  WriteCriticalBlock(&pkt, sizeof(pkt));
}

/*
  检查是否处于日志持续状态,即在解除武装或武装失败后继续记录日志
 */
bool AP_Logger::in_log_persistance(void) const
{
    uint32_t now = AP_HAL::millis();
    uint32_t persist_ms = HAL_LOGGER_ARM_PERSIST*1000U;
    if (_force_long_log_persist) {
        // 日志持续时间延长10倍
        persist_ms *= 10U;
    }

    // 解除武装后继续记录HAL_LOGGER_ARM_PERSIST秒
    const uint32_t arm_change_ms = hal.util->get_last_armed_change();
    if (!hal.util->get_soft_armed() && arm_change_ms != 0 && now - arm_change_ms < persist_ms) {
        return true;
    }

    // 武装失败后继续记录HAL_LOGGER_ARM_PERSIST秒
    if (_last_arming_failure_ms && now - _last_arming_failure_ms < persist_ms) {
        return true;
    }

    return false;
}


/*
  检查在解除武装状态下是否应该记录日志
 */
bool AP_Logger::log_while_disarmed(void) const
{
    if (_force_log_disarmed) {
        return true;
    }
    if (_params.log_disarmed == LogDisarmed::LOG_WHILE_DISARMED ||
        _params.log_disarmed == LogDisarmed::LOG_WHILE_DISARMED_DISCARD ||
        (_params.log_disarmed == LogDisarmed::LOG_WHILE_DISARMED_NOT_USB && !hal.gpio->usb_connected())) {
        return true;
    }

    return in_log_persistance();
}

#if HAL_LOGGER_FILE_CONTENTS_ENABLED
// 准备武装时的系统文件日志记录
void AP_Logger::prepare_at_arming_sys_file_logging()
{
    // 释放现有内容
    at_arm_file_content.reset();

    /*
      在武装时记录有用于诊断的文件。我们在武装时记录是因为在LOG_DISARMED模式下,
      我们不希望在启动时记录统计信息,否则无法获得关键系统值的真实情况。
      注意其中一些文件可能不存在,这种情况下会被忽略
     */
    static const char *log_content_filenames[] = {
        "@SYS/uarts.txt",
#ifdef HAL_DEBUG_BUILD
        // 记录dma.txt会影响性能
        "@SYS/dma.txt",
#endif
        "@SYS/memory.txt",
        "@SYS/threads.txt",
        "@SYS/timers.txt",
        "@ROMFS/hwdef.dat",
        "@SYS/storage.bin",
        "@SYS/crash_dump.bin",
        "@ROMFS/defaults.parm",
    };
    for (const auto *name : log_content_filenames) {
        log_file_content(at_arm_file_content, name);
    }
}

// 重置文件内容
void AP_Logger::FileContent::reset()
{
    WITH_SEMAPHORE(sem);
    file_list *next = nullptr;
    for (auto *c = head; c != nullptr; c = next) {
        next = c->next;
        delete [] c->filename;
        delete c;
    }
    head = nullptr;
    tail = nullptr;
    if (fd != -1) {
        AP::FS().close(fd);
        fd = -1;
    }
    counter = 0;
    fast = false;
    offset = 0;
}

// 从FileContent中移除并删除指定的文件列表项
void AP_Logger::FileContent::remove_and_free(file_list *victim)
{
    WITH_SEMAPHORE(sem);

    file_list *prev = nullptr;
    for (auto *c = head; c != nullptr; prev = c, c = c->next) {
        if (c != victim) {
            continue;
        }

        // 找到要移除的项,移除并返回
        if (prev == nullptr) {
            head = victim->next;
        } else {
            prev->next = victim->next;
        }
        delete [] victim->filename;
        delete victim;
        return;
    }
}


/*
  在FILE日志消息中记录文件内容
 */
void AP_Logger::log_file_content(const char *filename)
{
    log_file_content(normal_file_content, filename);
}

// 记录文件内容到指定的FileContent对象
void AP_Logger::log_file_content(FileContent &file_content, const char *filename)
{
    WITH_SEMAPHORE(file_content.sem);
    auto *file = NEW_NOTHROW file_list;
    if (file == nullptr) {
        return;
    }
    // 复制文件名以允许原始文件名超出作用域
    const size_t len = strlen(filename)+1;
    char * tmp_filename = NEW_NOTHROW char[len];
    if (tmp_filename == nullptr) {
        delete file;
        return;
    }
    strncpy(tmp_filename, filename, len);
    file->filename = tmp_filename;
    // 如果完整文件名太长则移除目录部分
    const char * name = strrchr(file->filename, '/');
    if ((len-1 > sizeof(file->log_filename)) && (name != nullptr)) {
        strncpy_noterm(file->log_filename, name+1, sizeof(file->log_filename));
    } else {
        strncpy_noterm(file->log_filename, file->filename, sizeof(file->log_filename));
    }
    if (file_content.head == nullptr) {
        file_content.tail = file_content.head = file;
        file_content.fd = -1;
    } else {
        file_content.tail->next = file;
        file_content.tail = file;
    }
}

/*
  定期更新文件内容
 */
void AP_Logger::file_content_update(void)
{
    if (file_content_prepare_for_arming) {
        file_content_prepare_for_arming = false;
        prepare_at_arming_sys_file_logging();
    }

    file_content_update(at_arm_file_content);
    file_content_update(normal_file_content);
}

// 更新指定FileContent对象的文件内容
void AP_Logger::file_content_update(FileContent &file_content)
{
    auto *file = file_content.head;
    if (file == nullptr) {
        return;
    }

    /* 此函数平均以约100Hz的频率调用(在400Hz的直升机上测试)。
       我们不想让文件数据占用太多日志空间,所以我们将64字节文件写入的频率
       降低10倍。对于crash_dump.bin文件,我们以10倍速度转储,这样可以在
       合理的时间内完成(450k的完整转储大约需要1分钟)
    */
    file_content.counter++;
    const uint8_t frequency = file_content.fast?1:10;
    if (file_content.counter % frequency != 0) {
        return;
    }

    if (file_content.fd == -1) {
        // 打开新文件
        file_content.fd  = AP::FS().open(file->filename, O_RDONLY);
        file_content.fast = strncmp(file->filename, "@SYS/crash_dump", 15) == 0;
        if (file_content.fd == -1) {
            file_content.remove_and_free(file);
            return;
        }
        file_content.offset = 0;
        if (file_content.fast) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Logging %s", file->filename);
        }
    }

    struct log_File pkt {
        LOG_PACKET_HEADER_INIT(LOG_FILE_MSG),
    };
    memcpy(pkt.filename, file->log_filename, sizeof(pkt.filename));
    const auto length = AP::FS().read(file_content.fd, pkt.data, sizeof(pkt.data));
    if (length <= 0) {
        AP::FS().close(file_content.fd);
        file_content.fd = -1;
        file_content.remove_and_free(file);
        return;
    }
    pkt.offset = file_content.offset;
    pkt.length = length;
    if (WriteBlock_first_succeed(&pkt, sizeof(pkt))) {
        file_content.offset += length;
    } else {
        // 回退以便下次重试
        AP::FS().lseek(file_content.fd, file_content.offset, SEEK_SET);
    }
}
#endif // HAL_LOGGER_FILE_CONTENTS_ENABLED

namespace AP {

// 返回AP_Logger单例对象
AP_Logger &logger()
{
    return *AP_Logger::get_singleton();
}

};

#endif // HAL_LOGGING_ENABLED
