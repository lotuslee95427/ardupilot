/* 
   AP_Logger日志记录 - 基于文件的变体

   使用posix文件IO在指定目录下创建名为logs/NN.bin的日志文件

   PixHawk上的SD卡性能:
    - 删除速率约50个文件/秒
    - stat操作约150次/秒 
    - 读取511个条目的目录需要约62,000微秒
 */

#include "AP_Logger_config.h"

#if HAL_LOGGING_FILESYSTEM_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Filesystem/AP_Filesystem.h>

#include "AP_Logger.h"
#include "AP_Logger_File.h"

#include <AP_Common/AP_Common.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_AHRS/AP_AHRS.h>

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>


extern const AP_HAL::HAL& hal;

// 日志页大小定义为1024字节
#define LOGGER_PAGE_SIZE 1024UL

// MB和B的转换系数
#define MB_to_B 1000000
#define B_to_MB 0.000001

// 尝试打开日志的时间间隔(毫秒)
#define LOGGER_FILE_REOPEN_MS 5000

/*
  构造函数
 */
AP_Logger_File::AP_Logger_File(AP_Logger &front,
                               LoggerMessageWriter_DFLogStart *writer) :
    AP_Logger_Backend(front, writer),
    _log_directory(HAL_BOARD_LOG_DIRECTORY)
{
    // 清除统计数据
    df_stats_clear();
}


/*
  确保日志目录存在
*/
void AP_Logger_File::ensure_log_directory_exists()
{
    int ret;
    struct stat st;

    EXPECT_DELAY_MS(3000);
    // 检查目录是否存在
    ret = AP::FS().stat(_log_directory, &st);
    if (ret == -1) {
        // 目录不存在,创建目录
        ret = AP::FS().mkdir(_log_directory);
    }
    if (ret == -1 && errno != EEXIST) {
        // 创建目录失败,打印错误信息
        printf("Failed to create log directory %s : %s\n", _log_directory, strerror(errno));
    }
}

/*
  初始化函数
*/
void AP_Logger_File::Init()
{
    // 确定并限制文件后端缓冲区大小
    uint32_t bufsize = _front._params.file_bufsize;
    bufsize *= 1024;

    const uint32_t desired_bufsize = bufsize;

    // 如果无法分配完整大小,尝试减小直到可以分配
    while (!_writebuf.set_size(bufsize) && bufsize >= _writebuf_chunk) {
        bufsize *= 0.9;
    }
    if (bufsize >= _writebuf_chunk && bufsize != desired_bufsize) {
        DEV_PRINTF("AP_Logger: reduced buffer %u/%u\n", (unsigned)bufsize, (unsigned)desired_bufsize);
    }

    // 检查是否成功分配缓冲区
    if (!_writebuf.get_size()) {
        DEV_PRINTF("Out of memory for logging\n");
        return;
    }

    DEV_PRINTF("AP_Logger_File: buffer size=%u\n", (unsigned)bufsize);

    _initialised = true;

    // 检查是否有自定义日志目录
    const char* custom_dir = hal.util->get_custom_log_directory();
    if (custom_dir != nullptr){
        _log_directory = custom_dir;
    }

    // 查找最后一个日志编号
    uint16_t last_log_num = find_last_log();
    if (last_log_is_marked_discard) {
        // 删除标记为丢弃的最后一个日志(LOG_DISARMED=3)
        char *filename = _log_file_name(last_log_num);
        if (filename != nullptr) {
            AP::FS().unlink(filename);
            free(filename);
        }
    }

    // 准备最小空间
    Prep_MinSpace();
}

/*
  检查文件是否存在
*/
bool AP_Logger_File::file_exists(const char *filename) const
{
    struct stat st;
    EXPECT_DELAY_MS(3000);
    if (AP::FS().stat(filename, &st) == -1) {
        // 希望errno==ENOENT。如果发生其他错误,最好假设文件存在
        return false;
    }
    return true;
}

/*
  检查指定编号的日志是否存在
*/
bool AP_Logger_File::log_exists(const uint16_t lognum) const
{
    char *filename = _log_file_name(lognum);
    if (filename == nullptr) {
        return false; // ?!
    }
    bool ret = file_exists(filename);
    free(filename);
    return ret;
}

/*
  1Hz周期任务
*/
void AP_Logger_File::periodic_1Hz()
{
    AP_Logger_Backend::periodic_1Hz();

    // 擦除后重新开始日志记录
    if (_initialised &&
        _write_fd == -1 && _read_fd == -1 &&
        erase.log_num == 0 &&
        erase.was_logging) {
        erase.was_logging = false;
        start_new_log_pending = true;
    }
    
    // 在后端线程中打开日志
    if (_initialised &&
        !start_new_log_pending &&
        _write_fd == -1 && _read_fd == -1 &&
        logging_enabled() &&
        !recent_open_error()) {
        start_new_log_pending = true;
    }

    // 检查IO线程是否存活
    if (!io_thread_alive()) {
        if (io_thread_warning_decimation_counter == 0 && _initialised) {
            // 只有在初始化后才打印此错误。当_initialised设置为true时,我们注册IO定时器回调
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "AP_Logger: stuck thread (%s)", last_io_operation);
        }
        if (io_thread_warning_decimation_counter++ > 30) {
            io_thread_warning_decimation_counter = 0;
        }
    }

    // 设置速率限制器
    if (rate_limiter == nullptr &&
        (_front._params.file_ratemax > 0 ||
         _front._params.disarm_ratemax > 0 ||
         _front._log_pause)) {
        rate_limiter = NEW_NOTHROW AP_Logger_RateLimiter(_front, _front._params.file_ratemax, _front._params.disarm_ratemax);
    }
}

/*
  全速率周期任务
*/
void AP_Logger_File::periodic_fullrate()
{
    AP_Logger_Backend::push_log_blocks();
}

/*
  获取可用缓冲区空间
*/
uint32_t AP_Logger_File::bufferspace_available()
{
    const uint32_t space = _writebuf.space();
    const uint32_t crit = critical_message_reserved_space(_writebuf.get_size());

    return (space > crit) ? space - crit : 0;
}

/*
  检查是否最近发生打开错误
*/
bool AP_Logger_File::recent_open_error(void) const
{
    if (_open_error_ms == 0) {
        return false;
    }
    return AP_HAL::millis() - _open_error_ms < LOGGER_FILE_REOPEN_MS;
}

/*
  检查存储卡是否已插入
*/
bool AP_Logger_File::CardInserted(void) const
{
    return _initialised && !recent_open_error();
}

/*
  获取日志目录中的可用磁盘空间(字节)
  失败返回-1
*/
int64_t AP_Logger_File::disk_space_avail()
{
    return AP::FS().disk_free(_log_directory);
}

/*
  获取日志目录中的总磁盘空间(使用+可用)(字节)
  失败返回-1
*/
int64_t AP_Logger_File::disk_space()
{
    return AP::FS().disk_space(_log_directory);
}

/*
  将dirent转换为日志编号
*/
bool AP_Logger_File::dirent_to_log_num(const dirent *de, uint16_t &log_num) const
{
    uint8_t length = strlen(de->d_name);
    if (length < 5) {
        return false;
    }
    if (strncmp(&de->d_name[length-4], ".BIN", 4) != 0) {
        // 不以.BIN结尾
        return false;
    }

    uint16_t thisnum = strtoul(de->d_name, nullptr, 10);
    if (thisnum > _front.get_max_num_logs()) {
        return false;
    }
    log_num = thisnum;
    return true;
}


/*
  查找最早的日志
  如果没有找到日志返回0
*/
uint16_t AP_Logger_File::find_oldest_log()
{
    if (_cached_oldest_log != 0) {
        return _cached_oldest_log;
    }

    uint16_t last_log_num = find_last_log();
    if (last_log_num == 0) {
        return 0;
    }

    uint16_t current_oldest_log = 0; // 0是无效的

    // 我们可以计数到find_last_log(),但如果人们开始依赖min_avail_space_percent功能,
    // 我们可能会做大量的asprintf()和stat()操作
    EXPECT_DELAY_MS(3000);
    auto *d = AP::FS().opendir(_log_directory);
    if (d == nullptr) {
        // SD卡可能已死?在linux上可能有人rm -rf了目录
        return 0;
    }

    // 我们只删除看起来像xxx.BIN的文件
    EXPECT_DELAY_MS(3000);
    for (struct dirent *de=AP::FS().readdir(d); de; de=AP::FS().readdir(d)) {
        EXPECT_DELAY_MS(3000);
        uint16_t thisnum;
        if (!dirent_to_log_num(de, thisnum)) {
            // 不是日志文件名
            continue;
        }
        if (current_oldest_log == 0) {
            current_oldest_log = thisnum;
        } else {
            if (current_oldest_log <= last_log_num) {
                if (thisnum > last_log_num) {
                    current_oldest_log = thisnum;
                } else if (thisnum < current_oldest_log) {
                    current_oldest_log = thisnum;
                }
            } else { // current_oldest_log > last_log_num
                if (thisnum > last_log_num) {
                    if (thisnum < current_oldest_log) {
                        current_oldest_log = thisnum;
                    }
                }
            }
        }
    }
    AP::FS().closedir(d);
    _cached_oldest_log = current_oldest_log;

    return current_oldest_log;
}

/*
  准备最小空间
*/
void AP_Logger_File::Prep_MinSpace()
{
    if (hal.util->was_watchdog_reset()) {
        // 看门狗复位时不清理空间,需要太长时间
        return;
    }

    if (!CardInserted()) {
        return;
    }

    const uint16_t first_log_to_remove = find_oldest_log();
    if (first_log_to_remove == 0) {
        // 没有文件需要删除
        return;
    }

    const int64_t target_free = (int64_t)_front._params.min_MB_free * MB_to_B;

    uint16_t log_to_remove = first_log_to_remove;

    uint16_t count = 0;
    do {
        int64_t avail = disk_space_avail();
        if (avail == -1) {
            break;
        }
        if (avail >= target_free) {
            break;
        }
        if (count++ > _front.get_max_num_logs() + 10) {
            // 这里删除太多了。可能是内部错误。
            INTERNAL_ERROR(AP_InternalError::error_t::logger_too_many_deletions);
            break;
        }
        char *filename_to_remove = _log_file_name(log_to_remove);
        if (filename_to_remove == nullptr) {
            INTERNAL_ERROR(AP_InternalError::error_t::logger_bad_getfilename);
            break;
        }
        if (file_exists(filename_to_remove)) {
            DEV_PRINTF("Removing (%s) for minimum-space requirements (%.0fMB < %.0fMB)\n",
                                filename_to_remove, (double)avail*B_to_MB, (double)target_free*B_to_MB);
            EXPECT_DELAY_MS(2000);
            if (AP::FS().unlink(filename_to_remove) == -1) {
                _cached_oldest_log = 0;
                DEV_PRINTF("Failed to remove %s: %s\n", filename_to_remove, strerror(errno));
                free(filename_to_remove);
                if (errno == ENOENT) {
                    // 损坏 - 应该总是有连续的文件序列...但可能还有其他文件,所以继续。
                } else {
                    break;
                }
            } else {
                free(filename_to_remove);
            }
        }
        log_to_remove++;
        if (log_to_remove > _front.get_max_num_logs()) {
            log_to_remove = 1;
        }
    } while (log_to_remove != first_log_to_remove);
}

/*
  根据日志编号构造日志文件名。
  日志文件名中的编号将用零填充。
  注意:调用者必须释放内存。
 */
char *AP_Logger_File::_log_file_name(const uint16_t log_num) const
{
    char *buf = nullptr;
    if (asprintf(&buf, "%s/%08u.BIN", _log_directory, (unsigned)log_num) == -1) {
        return nullptr;
    }
    return buf;
}

/*
  返回lastlog.txt标记文件的路径名
  注意:调用者必须释放内存。
 */
char *AP_Logger_File::_lastlog_file_name(void) const
{
    char *buf = nullptr;
    if (asprintf(&buf, "%s/LASTLOG.TXT", _log_directory) == -1) {
        return nullptr;
    }
    return buf;
}


/*
  删除所有日志文件
*/
void AP_Logger_File::EraseAll()
{
    if (hal.util->get_soft_armed()) {
        // 在飞行等状态下不想执行任何文件系统操作
        return;
    }
    if (!_initialised) {
        return;
    }

    erase.was_logging = (_write_fd != -1);
    stop_logging();

    erase.log_num = 1;
}

/*
  检查写入是否正常
*/
bool AP_Logger_File::WritesOK() const
{
    if (_write_fd == -1) {
        return false;
    }
    if (recent_open_error()) {
        return false;
    }
    return true;
}


/*
  检查是否可以开始新日志
*/
bool AP_Logger_File::StartNewLogOK() const
{
    if (recent_open_error()) {
        return false;
    }
#if !APM_BUILD_TYPE(APM_BUILD_Replay) && !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    if (hal.scheduler->in_main_thread()) {
        return false;
    }
#endif
    return AP_Logger_Backend::StartNewLogOK();
}

/* 
  在当前偏移量写入数据块
*/
bool AP_Logger_File::_WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical)
{
    WITH_SEMAPHORE(semaphore);

#if APM_BUILD_TYPE(APM_BUILD_Replay)
    if (AP::FS().write(_write_fd, pBuffer, size) != size) {
        AP_HAL::panic("Short write");
    }
    return true;
#endif


    uint32_t space = _writebuf.space();

    if (_writing_startup_messages &&
        _startup_messagewriter->fmt_done()) {
        // 状态机已调用我们,并且已完成写入格式消息。
        // 它以后可以随时再次给我们发送消息,所以让我们为其他内容留出空间:
        const uint32_t now = AP_HAL::millis();
        const bool must_dribble = (now - last_messagewrite_message_sent) > 100;
        if (!must_dribble &&
            space < non_messagewriter_message_reserved_space(_writebuf.get_size())) {
            // 这条消息不会丢失,它会再次发送...
            return false;
        }
        last_messagewrite_message_sent = now;
    } else {
        // 我们为关键消息保留一些空间:
        if (!is_critical && space < critical_message_reserved_space(_writebuf.get_size())) {
            _dropped++;
            return false;
        }
    }

    // 如果没有整个消息的空间 - 丢弃它:
    if (space < size) {
        _dropped++;
        return false;
    }

    _writebuf.write((uint8_t*)pBuffer, size);
    df_stats_gather(size, _writebuf.space());
    return true;
}

/*
  查找最高的日志编号
 */
uint16_t AP_Logger_File::find_last_log()
{
    unsigned ret = 0;
    char *fname = _lastlog_file_name();
    if (fname == nullptr) {
        return ret;
    }
    EXPECT_DELAY_MS(3000);
    FileData *fd = AP::FS().load_file(fname);
    free(fname);
    last_log_is_marked_discard = false;
    if (fd != nullptr) {
        char *endptr = nullptr;
        ret = strtol((const char *)fd->data, &endptr, 10);
        if (endptr != nullptr) {
            last_log_is_marked_discard = *endptr == 'D';
        }
        delete fd;
    }
    return ret;
}

/*
  获取日志大小
*/
uint32_t AP_Logger_File::_get_log_size(const uint16_t log_num)
{
    char *fname = _log_file_name(log_num);
    if (fname == nullptr) {
        return 0;
    }
    if (_write_fd != -1 && write_fd_semaphore.take_nonblocking()) {
        if (_write_filename != nullptr && strcmp(_write_filename, fname) == 0) {
            // 这是我们当前正在写入的文件
            free(fname);
            write_fd_semaphore.give();
            return _write_offset;
        }
        write_fd_semaphore.give();
    }
    struct stat st;
    EXPECT_DELAY_MS(3000);
    if (AP::FS().stat(fname, &st) != 0) {
        free(fname);
        return 0;
    }
    free(fname);
    return st.st_size;
}

/*
  获取日志时间
*/
uint32_t AP_Logger_File::_get_log_time(const uint16_t log_num)
{
    char *fname = _log_file_name(log_num);
    if (fname == nullptr) {
        return 0;
    }
    if (_write_fd != -1 && write_fd_semaphore.take_nonblocking()) {
        if (_write_filename != nullptr && strcmp(_write_filename, fname) == 0) {
            // 这是我们当前正在写入的文件
            free(fname);
            write_fd_semaphore.give();
#if AP_RTC_ENABLED
            uint64_t utc_usec;
            if (!AP::rtc().get_utc_usec(utc_usec)) {
                return 0;
            }
            return utc_usec / 1000000U;
#else
            return 0;
#endif
        }
        write_fd_semaphore.give();
    }
    struct stat st;
    EXPECT_DELAY_MS(3000);
    if (AP::FS().stat(fname, &st) != 0) {
        free(fname);
        return 0;
    }
    free(fname);
    return st.st_mtime;
}

/*
  获取日志边界
*/
void AP_Logger_File::get_log_boundaries(const uint16_t list_entry, uint32_t & start_page, uint32_t & end_page)
{
    const uint16_t log_num = log_num_from_list_entry(list_entry);
    if (log_num == 0) {
        // 失败 - 可能没有日志
        start_page = 0;
        end_page = 0;
        return;
    }

    start_page = 0;
    end_page = _get_log_size(log_num) / LOGGER_PAGE_SIZE;
}

/*
  从日志文件中检索数据
 */
int16_t AP_Logger_File::get_log_data(const uint16_t list_entry, const uint16_t page, const uint32_t offset, const uint16_t len, uint8_t *data)
{
    if (!_initialised || recent_open_error()) {
        return -1;
    }

    const uint16_t log_num = log_num_from_list_entry(list_entry);
    if (log_num == 0) {
        // 失败 - 可能没有日志
        return -1;
    }

    if (_read_fd != -1 && log_num != _read_fd_log_num) {
        AP::FS().close(_read_fd);
        _read_fd = -1;
    }
    if (_read_fd == -1) {
        char *fname = _log_file_name(log_num);
        if (fname == nullptr) {
            return -1;
        }
        stop_logging();
        EXPECT_DELAY_MS(3000);
        _read_fd = AP::FS().open(fname, O_RDONLY);
        if (_read_fd == -1) {
            _open_error_ms = AP_HAL::millis();
            int saved_errno = errno;
            ::printf("Log read open fail for %s - %s\n",
                     fname, strerror(saved_errno));
            DEV_PRINTF("Log read open fail for %s - %s\n",
                                fname, strerror(saved_errno));
            free(fname);
            return -1;            
        }
        free(fname);
        _read_offset = 0;
        _read_fd_log_num = log_num;
    }
    uint32_t ofs = page * (uint32_t)LOGGER_PAGE_SIZE + offset;

    if (ofs != _read_offset) {
        if (AP::FS().lseek(_read_fd, ofs, SEEK_SET) == (off_t)-1) {
            AP::FS().close(_read_fd);
            _read_fd = -1;
            return -1;
        }
        _read_offset = ofs;
    }
    int16_t ret = (int16_t)AP::FS().read(_read_fd, data, len);
    if (ret > 0) {
        _read_offset += ret;
    }
    return ret;
}

/*
  结束日志传输
*/
void AP_Logger_File::end_log_transfer()
{
    if (_read_fd != -1) {
        AP::FS().close(_read_fd);
        _read_fd = -1;
    }
}

/*
  获取日志大小和时间
*/
void AP_Logger_File::get_log_info(const uint16_t list_entry, uint32_t &size, uint32_t &time_utc)
{
    uint16_t log_num = log_num_from_list_entry(list_entry);
    if (log_num == 0) {
        // 失败 - 可能没有日志
        size = 0;
        time_utc = 0;
        return;
    }

    size = _get_log_size(log_num);
    time_utc = _get_log_time(log_num);
}


/*
  获取日志数量 - 注意日志编号必须连续
*/
uint16_t AP_Logger_File::get_num_logs()
{
    auto *d = AP::FS().opendir(_log_directory);
    if (d == nullptr) {
        return 0;
    }
    uint16_t high = find_last_log();
    uint16_t ret = high;
    uint16_t smallest_above_last = 0;

    EXPECT_DELAY_MS(2000);
    for (struct dirent *de=AP::FS().readdir(d); de; de=AP::FS().readdir(d)) {
        EXPECT_DELAY_MS(100);
        uint16_t thisnum;
        if (!dirent_to_log_num(de, thisnum)) {
            // 不是日志文件名
            continue;
        }

        if (thisnum > high && (smallest_above_last == 0 || thisnum < smallest_above_last)) {
            smallest_above_last = thisnum;
        }
    }
    AP::FS().closedir(d);
    if (smallest_above_last != 0) {
        // 我们已经回绕,加入高编号的日志
        ret += (_front.get_max_num_logs() - smallest_above_last) + 1;
    }

    return ret;
}

/*
  停止日志记录
*/
void AP_Logger_File::stop_logging(void)
{
    // 尽最大努力避免打扰IO线程
    const bool have_sem = write_fd_semaphore.take(hal.util->get_soft_armed()?1:20);
    if (_write_fd != -1) {
        int fd = _write_fd;
        _write_fd = -1;
        AP::FS().close(fd);
    }
    if (have_sem) {
        write_fd_semaphore.give();
    }
}

/*
  在日志记录器线程中执行start_new_log
*/
void AP_Logger_File::PrepForArming_start_logging()
{
    if (logging_started()) {
        return;
    }

    uint32_t start_ms = AP_HAL::millis();
    const uint32_t open_limit_ms = 1000;

    /*
      日志打开发生在io_timer线程中。我们允许最多1秒完成打开
     */
    start_new_log_pending = true;
    EXPECT_DELAY_MS(1000);
    while (AP_HAL::millis() - start_ms < open_limit_ms) {
        if (logging_started()) {
            break;
        }
#if !APM_BUILD_TYPE(APM_BUILD_Replay) && AP_AHRS_ENABLED
        // 保持EKF运行
        AP::ahrs().update();
#endif
        hal.scheduler->delay(1);
    }
}

/*
  开始写入新的日志文件
*/
void AP_Logger_File::start_new_log(void)
{
    if (recent_open_error()) {
        // 我们之前打开文件失败 - 不要再尝试
        // 以防止我们在飞行时尝试打开文件
        return;
    }

    if (erase.log_num != 0) {
        // 擦除时不要开始新日志,但记录我们想在擦除完成后开始日志记录
        erase.was_logging = true;
        return;
    }

    const bool open_error_ms_was_zero = (_open_error_ms == 0);

    // 在这里设置_open_error以避免无限递归。简单地
    // 写入优先级块可能会尝试打开日志 - 这意味着
    // 如果start_new_log路径中的任何内容执行GCS_SEND_TEXT()
    // (例如),如果我们不采取预防措施,你将最终递归。
    // 如果我们真的设法打开日志,我们将重置_open_error...
    // 设置打开错误时间戳为当前时间
    _open_error_ms = AP_HAL::millis();

    // 停止当前日志记录
    stop_logging();

    // 重置新日志相关变量
    start_new_log_reset_variables();

    // 如果读取文件描述符存在,关闭它
    if (_read_fd != -1) {
        AP::FS().close(_read_fd);
        _read_fd = -1;
    }

    // 检查磁盘剩余空间是否足够
    if (disk_space_avail() < _free_space_min_avail && disk_space() > 0) {
        DEV_PRINTF("Out of space for logging\n");
        return;
    }

    // 查找最后一个日志编号
    uint16_t log_num = find_last_log();
    // 如果可能的话重用空日志
    if (_get_log_size(log_num) > 0 || log_num == 0) {
        log_num++;
    }
    // 如果日志编号超过最大值,重置为1
    if (log_num > _front.get_max_num_logs()) {
        log_num = 1;
    }
    // 获取写入文件信号量
    if (!write_fd_semaphore.take(1)) {
        return;
    }
    // 释放旧的文件名内存
    if (_write_filename) {
        free(_write_filename);
        _write_filename = nullptr;        
    }
    // 生成新的日志文件名
    _write_filename = _log_file_name(log_num);
    if (_write_filename == nullptr) {
        write_fd_semaphore.give();
        return;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    // 记住打开文件时是否有UTC时间
#if AP_RTC_ENABLED
    uint64_t utc_usec;
    _need_rtc_update = !AP::rtc().get_utc_usec(utc_usec);
#endif
#endif

    // 确保日志目录存在
    ensure_log_directory_exists();

    // 打开新的日志文件
    EXPECT_DELAY_MS(3000);
    _write_fd = AP::FS().open(_write_filename, O_WRONLY|O_CREAT|O_TRUNC);
    _cached_oldest_log = 0;

    // 如果打开失败,处理错误
    if (_write_fd == -1) {
        write_fd_semaphore.give();
        int saved_errno = errno;
        if (open_error_ms_was_zero) {
            ::printf("Log open fail for %s - %s\n",
                     _write_filename, strerror(saved_errno));
            DEV_PRINTF("Log open fail for %s - %s\n",
                                _write_filename, strerror(saved_errno));
        }
        return;
    }
    // 初始化写入相关变量
    _last_write_ms = AP_HAL::millis();
    _open_error_ms = 0;
    _write_offset = 0;
    _writebuf.clear();
    write_fd_semaphore.give();

    // 更新lastlog.txt文件,记录新的日志编号
    last_log_is_marked_discard = _front._params.log_disarmed == AP_Logger::LogDisarmed::LOG_WHILE_DISARMED_DISCARD;
    if (!write_lastlog_file(log_num)) {
        _open_error_ms = AP_HAL::millis();
    }

