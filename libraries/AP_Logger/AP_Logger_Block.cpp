/*
  基于块的日志记录,用于带有闪存日志功能的开发板
 */

#include "AP_Logger_config.h"

#if HAL_LOGGING_BLOCK_ENABLED

#include "AP_Logger_Block.h"
#include "AP_Logger.h"
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include <AP_RTC/AP_RTC.h>
#include <GCS_MAVLink/GCS.h>

const extern AP_HAL::HAL& hal;

// 最后一页保存日志格式的前4个字节。只有在底层格式改变时才需要修改此值
#define DF_LOGGING_FORMAT    0x1901201B

// 构造函数,初始化前端对象和写入缓冲区
AP_Logger_Block::AP_Logger_Block(AP_Logger &front, LoggerMessageWriter_DFLogStart *writer) :
    AP_Logger_Backend(front, writer),
    writebuf(0)
{
    df_stats_clear();
}

// Init()在驱动初始化后调用,驱动程序需要确保设备已准备好接受命令
void AP_Logger_Block::Init(void)
{
    // 缓冲区用于读写操作,所以访问必须在信号量内
    buffer = (uint8_t *)hal.util->malloc_type(df_PageSize, AP_HAL::Util::MEM_DMA_SAFE);
    if (buffer == nullptr) {
        AP_HAL::panic("日志记录DMA内存不足");
    }

    //flash_test();

    if (CardInserted()) {
        // 在最后一个扇区保留版本信息的空间
        df_NumPages -= df_PagePerBlock;

        // 确定和限制文件后端缓冲区大小
        uint32_t bufsize = _front._params.file_bufsize;
        if (bufsize > 64) {
            bufsize = 64;
        }
        bufsize *= 1024;

        // 如果无法分配完整大小,尝试减小直到可以分配
        while (!writebuf.set_size(bufsize) && bufsize >= df_PageSize * df_PagePerBlock) {
            DEV_PRINTF("AP_Logger_Block: 无法设置缓冲区大小为=%u\n", (unsigned)bufsize);
            bufsize >>= 1;
        }

        if (!writebuf.get_size()) {
            DEV_PRINTF("日志记录内存不足\n");
            return;
        }

        DEV_PRINTF("AP_Logger_Block: 缓冲区大小=%u\n", (unsigned)bufsize);
        _initialised = true;
    }

    WITH_SEMAPHORE(sem);

    if (NeedErase()) {
        EraseAll();
    } else {
        validate_log_structure();
    }
}

// 返回可用的缓冲区空间
uint32_t AP_Logger_Block::bufferspace_available()
{
    // 由于AP_Logger_Block设备是环形缓冲区,所以总是有空间
    return df_NumPages * df_PageSize;
}

// *** 日志记录器公共函数 ***

// 开始写入指定页地址
void AP_Logger_Block::StartWrite(uint32_t PageAdr)
{
    df_PageAdr = PageAdr;
}

// 完成写入操作
void AP_Logger_Block::FinishWrite(void)
{
    // 将缓冲区写入闪存
    BufferToPage(df_PageAdr);
    df_PageAdr++;

    // 如果到达内存末尾,从头开始
    if (df_PageAdr > df_NumPages) {
        df_PageAdr = 1;
    }

    // 开始新扇区时擦除它
    if ((df_PageAdr-1) % df_PagePerBlock == 0) {
        // 如果我们已经覆盖了现有日志,强制重新计算最旧的日志
        if (_cached_oldest_log > 0) {
            uint16_t log_num = StartRead(df_PageAdr);
            if (log_num != 0xFFFF && log_num >= _cached_oldest_log) {
                _cached_oldest_log = 0;
            }
        }
        // 我们是否要擦除包含自己头部的扇区?
        if (df_Write_FilePage > df_NumPages - df_PagePerBlock) {
            chip_full = true;
            return;
        }
        SectorErase(get_block(df_PageAdr));
    }
}

// 检查是否可以写入
bool AP_Logger_Block::WritesOK() const
{
    if (!CardInserted() || erase_started) {
        return false;
    }
    return true;
}

// 写入优先级数据块
bool AP_Logger_Block::_WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical)
{
    // is_critical被忽略 - 我们是环形缓冲区,永远不会用完空间
    // 如果我们做更复杂的带宽限制,可以基于is_critical保留带宽
    if (!WritesOK()) {
        return false;
    }

    WITH_SEMAPHORE(write_sem);

    const uint32_t space = writebuf.space();

    if (_writing_startup_messages &&
        _startup_messagewriter->fmt_done()) {
        // 状态机已调用我们,并且已完成写入格式消息
        // 它以后可以随时返回更多消息,所以让我们为其他内容留出空间:
        const uint32_t now = AP_HAL::millis();
        const bool must_dribble = (now - last_messagewrite_message_sent) > 100;
        if (!must_dribble &&
            space < non_messagewriter_message_reserved_space(writebuf.get_size())) {
            // 此消息不会丢弃,它将再次发送...
            return false;
        }
        last_messagewrite_message_sent = now;
    } else {
        // 我们为关键消息保留一些空间:
        if (!is_critical && space < critical_message_reserved_space(writebuf.get_size())) {
            _dropped++;
            return false;
        }
    }

    // 如果没有整个消息的空间 - 丢弃它:
    if (space < size) {
        _dropped++;
        return false;
    }

    writebuf.write((uint8_t*)pBuffer, size);
    df_stats_gather(size, writebuf.space());

    return true;
}

// 从页地址读取并返回该位置的文件号
uint16_t AP_Logger_Block::StartRead(uint32_t PageAdr)
{
    // 将闪存页复制到缓冲区
    if (erase_started) {
        df_Read_PageAdr = PageAdr;
        memset(buffer, 0xff, df_PageSize);
    } else {
        PageToBuffer(PageAdr);
    }
    return ReadHeaders();
}

// 在当前读取点读取头部并返回文件号
uint16_t AP_Logger_Block::ReadHeaders()
{
    // 我们正在开始一个新页面 - 读取文件号和文件页
    struct PageHeader ph;
    BlockRead(0, &ph, sizeof(ph));
    df_FileNumber = ph.FileNumber;
    df_FilePage = ph.FilePage;
#if BLOCK_LOG_VALIDATE
    if (ph.crc != DF_LOGGING_FORMAT + df_FilePage && df_FileNumber != 0xFFFF) {
        printf("ReadHeaders: 在%d处读取无效块\n", df_Read_PageAdr);
    }
#endif
    df_Read_BufferIdx = sizeof(ph);
    // 如果我们在文件开头,读取文件头
    if (df_FilePage == 1) {
        struct FileHeader fh;
        BlockRead(sizeof(ph), &fh, sizeof(fh));
        df_FileTime = fh.utc_secs;
        df_Read_BufferIdx += sizeof(fh);
    }

    return df_FileNumber;
}

// 读取数据块
bool AP_Logger_Block::ReadBlock(void *pBuffer, uint16_t size)
{
    if (erase_started) {
        return false;
    }
    while (size > 0) {
        uint16_t n = df_PageSize - df_Read_BufferIdx;
        if (n > size) {
            n = size;
        }

        if (!BlockRead(df_Read_BufferIdx, pBuffer, n)) {
            return false;
        }
        size -= n;
        pBuffer = (void *)(n + (uintptr_t)pBuffer);

        df_Read_BufferIdx += n;

        if (df_Read_BufferIdx == df_PageSize) {
            uint32_t new_page_addr = df_Read_PageAdr + 1;
            if (new_page_addr > df_NumPages) {
                new_page_addr = 1;
            }
            if (erase_started) {
                memset(buffer, 0xff, df_PageSize);
                df_Read_PageAdr = new_page_addr;
            } else {
                PageToBuffer(new_page_addr);
            }

            // 我们正在开始一个新页面 - 读取文件号和文件页
            ReadHeaders();
        }
    }
    return true;
}

// 为给定的文件号初始化日志数据
void AP_Logger_Block::StartLogFile(uint16_t FileNumber)
{
    df_FileNumber = FileNumber;
    df_Write_FileNumber = FileNumber;
    df_FilePage = 1;
    df_Write_FilePage = 1;
}

// 获取文件号
uint16_t AP_Logger_Block::GetFileNumber() const
{
    return df_FileNumber;
}

// 擦除所有数据
void AP_Logger_Block::EraseAll()
{
    if (hal.util->get_soft_armed()) {
        // 在飞行等状态下不想执行任何文件系统操作
        return;
    }

    // 在停止日志记录前推送消息
    if (!erase_started) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "开始芯片擦除");
    }

    WITH_SEMAPHORE(sem);

    if (erase_started) {
        // 已经在擦除
        return;
    }
    erase_started = true;

    // 记住我们正在做什么
    new_log_pending = log_write_started;

    // 丢弃所有内容
    log_write_started = false;
    writebuf.clear();

    // 重置格式版本和包装状态,以便捕获任何不完整的擦除
    Sector4kErase(get_sector(df_NumPages));

    StartErase();
}

// 1Hz周期任务
void AP_Logger_Block::periodic_1Hz()
{
    AP_Logger_Backend::periodic_1Hz();

    if (rate_limiter == nullptr &&
        (_front._params.blk_ratemax > 0 ||
         _front._params.disarm_ratemax > 0 ||
         _front._log_pause)) {
        // 如果日志速率最大值>0Hz或请求暂停流式条目的日志,则设置速率限制
        rate_limiter = NEW_NOTHROW AP_Logger_RateLimiter(_front, _front._params.blk_ratemax, _front._params.disarm_ratemax);
    }
    
    if (!io_thread_alive()) {
        if (warning_decimation_counter == 0 && _initialised) {
            // 除非我们已初始化,否则不打印此错误。当_initialised设置为true时,
            // 我们注册IO定时器回调
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "AP_Logger: IO线程死亡");
        }
        if (warning_decimation_counter++ > 57) {
            warning_decimation_counter = 0;
        }
        _initialised = false;
    } else if (chip_full) {
        if (warning_decimation_counter == 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "芯片已满,日志记录已停止");
        }
        if (warning_decimation_counter++ > 57) {
            warning_decimation_counter = 0;
        }
    }
}

// EraseAll是异步的,但我们不能在子线程中启动新日志
// 所以此任务从io定时器获取提示,将锁定保持在最小值
void AP_Logger_Block::periodic_10Hz(const uint32_t now)
{
    if (erase_started || InErase()) {
        return;
    }

    // 不要在io线程中打印状态消息,在这里打印
    switch (status_msg) {
    case StatusMessage::ERASE_COMPLETE:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "芯片擦除完成");
        status_msg = StatusMessage::NONE;
        break;
    case StatusMessage::RECOVERY_COMPLETE:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "日志恢复完成");
        status_msg = StatusMessage::NONE;
        break;
    case StatusMessage::NONE:
        break;
    }

    // EraseAll应该只在主线程中设置这个
    if (new_log_pending) {
        start_new_log();
    }
}

/*
 * 如果日志格式已更改,我们需要擦除
 */
bool AP_Logger_Block::NeedErase(void)
{
    uint32_t version = 0;
    PageToBuffer(df_NumPages+1); // 最后一页
    BlockRead(0, &version, sizeof(version));
    if (version == DF_LOGGING_FORMAT) {
        // 只有在我们不打算销毁所有内容时才将读取点保持在合理的位置
        StartRead(1);
        return false;
    }
    return true;
}

/*
 * 遍历所有日志文件,查找损坏的文件并修复
 */
void AP_Logger_Block::validate_log_structure()
{
    WITH_SEMAPHORE(sem);
    bool wrapped = is_wrapped();
    uint32_t page = 1;
    uint32_t page_start = 1;

    uint16_t file = StartRead(page);
    uint16_t first_file = file;
    uint16_t next_file = file;
    uint16_t last_file = 0;

    while (file != 0xFFFF && page <= df_NumPages && (file == next_file || (wrapped && file < next_file))) {
        uint32_t end_page = find_last_page_of_log(file);
        if (end_page == 0 || end_page < page) { // 这可能发生并可能导致我们看到的损坏
            break;
        }
        page = end_page + 1;
        file = StartRead(page);
        next_file++;
        // 跳过已擦除块的其余部分
        if (wrapped && file == 0xFFFF) {
            file = StartRead((get_block(page) + 1) * df_PagePerBlock + 1);
        }
        if (wrapped && file < next_file) {
            page_start = page;
            next_file = file;
            first_file = file;
        } else if (last_file < next_file) {
            last_file = file;
        }
        if (file == next_file) {
            DEV_PRINTF("在%X-%X处找到完整的日志%d\n", int(file), unsigned(page), unsigned(find_last_page_of_log(file)));
        }
    }

    if (file != 0xFFFF && file != next_file && page <= df_NumPages && page > 0) {
        DEV_PRINTF("在0x%04X处找到损坏的日志%d,正在擦除", int(file), unsigned(page));
        df_EraseFrom = page;
    } else if (next_file != 0xFFFF && page > 0 && next_file > 1) { // 芯片为空
        DEV_PRINTF("在0x%04X-0x%04X处找到%d个完整的日志", int(next_file - first_file), unsigned(page_start), unsigned(page - 1));
    }
}

/**
 * 从日志获取原始数据 - page是日志的起始页,offset是从该页开始的日志内的偏移量
 */
int16_t AP_Logger_Block::get_log_data_raw(uint16_t log_num, uint32_t page, uint32_t offset, uint16_t len, uint8_t *data)
{
    WITH_SEMAPHORE(sem);
    const uint16_t data_page_size = df_PageSize - sizeof(struct PageHeader);
    const uint16_t first_page_size = data_page_size - sizeof(struct FileHeader);

    // offset是文件中的真实偏移量,所以我们必须计算考虑页头的偏移量
    if (offset >= first_page_size) {
        offset -= first_page_size;
        page = page + offset / data_page_size + 1;
        offset %= data_page_size;

        if (page > df_NumPages) {
            page = page % df_NumPages;
        }
    }

    // 检查我们是否被要求超出日志末尾的偏移量
    if (StartRead(page) != log_num) {
        return -1;
    }

    df_Read_BufferIdx += offset;

    if (!ReadBlock(data, len)) {
        return -1;
    }

    return (int16_t)len;
}

/**
  * 从日志获取数据,考虑添加FMT头
 */
int16_t AP_Logger_Block::get_log_data(uint16_t list_entry, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data)
{
    const uint16_t log_num = log_num_from_list_entry(list_entry);

    if (log_num == 0) {
        // 失败 - 可能没有日志
        return -1;
    }

    //printf("get_log_data(%d, %d, %d, %d)\n", log_num, page, offset, len);
    WITH_SEMAPHORE(sem);

    uint16_t ret = 0;
    if (len > 0) {
        const int16_t bytes = get_log_data_raw(log_num, page, offset, len, data);
        if (bytes == -1) {
            return -1;
        }
        ret += bytes;
    }

    return ret;
}


// 此函数确定AP_Logger中完整日志文件的数量
// 部分日志被拒绝,因为没有头部它们相对无用
uint16_t AP_Logger_Block::get_num_logs(void)
{
    WITH_SEMAPHORE(sem);
    uint32_t lastpage;
    uint32_t last;

    if (!CardInserted() || find_last_page() == 1) {
        return 0;
    }

    uint32_t first = StartRead(1);
    
    if (first == 0xFFFF) {
        return 0;
    }

    lastpage = find_last_page();
    last = StartRead(lastpage);

    if (is_wrapped()) {
        // 如果我们包装了,那么块的其余部分将填充0xFFFF,因为我们总是在写入之前擦除块
        // 为了找到第一页,我们必须在下一个块边界之后读取
        first = StartRead((get_block(lastpage) + 1) * df_PagePerBlock + 1);
        // 除非我们恰好落在正在覆盖的文件的第一页上,否则跳到下一个文件
        if (df_FilePage > 1) {
            first++;
        }
    }

    if (last == first) {
        return 1;
    }

    return (last - first + 1);
}

// 立即停止日志记录
void AP_Logger_Block::stop_logging(void)
{
    WITH_SEMAPHORE(sem);

    log_write_started = false;

    // 清除任何先前的日志写入
    writebuf.clear();
}

// 停止日志记录并刷新任何剩余数据
void AP_Logger_Block::stop_logging_async(void)
{
    stop_log_pending = true;
}

// 此函数在AP_Logger中启动新的日志文件
// 这里不应该写入实际数据到存储
// 这应该全部由IO线程处理
void AP_Logger_Block::start_new_log(void)
{
    if (erase_started) {
        // 当前正在擦除
        return;
    }

    WITH_SEMAPHORE(sem);

    if (logging_started()) {
        stop_logging();
    }

    // 不需要再调度这个
    new_log_pending = false;

    uint32_t last_page = find_last_page();

    StartRead(last_page);

    log_write_started = true;
    uint16_t new_log_num = 1;

    if (find_last_log() == 0 || GetFileNumber() == 0xFFFF) {
        StartLogFile(new_log_num);
        StartWrite(1);
    // 检查长度为1页的日志并抑制
    } else if (df_FilePage <= 1) {
        new_log_num = GetFileNumber();
        // 最后一个日志太短,重用其编号
        // 并覆盖它
        StartLogFile(new_log_num);
        StartWrite(last_page);
    } else {
        new_log_num = GetFileNumber()+1;
        if (last_page == 0xFFFF) {
            last_page=0;
        }
        StartLogFile(new_log_num);
        StartWrite(last_page + 1);
    }

    // 在前4个字节保存UTC时间,以便以后可以检索
    uint64_t utc_usec;
    FileHeader hdr {};
    if (AP::rtc().get_utc_usec(utc_usec)) {
        hdr.utc_secs = utc_usec / 1000000U;
    }
    writebuf.write((uint8_t*)&hdr, sizeof(FileHeader));

    start_new_log_reset_variables();

    return;
}

// 此函数查找日志文件的第一页和最后一页
// 如果AP_Logger已满并部分覆盖,第一页可能大于最后一页。
void AP_Logger_Block::get_log_boundaries(uint16_t list_entry, uint32_t & start_page, uint32_t & end_page)
{
    const uint16_t log_num = log_num_from_list_entry(list_entry);
    if (log_num == 0) {
        // 失败 - 可能没有日志
        start_page = 0;
        end_page = 0;
        return;
    }

    WITH_SEMAPHORE(sem);
    uint16_t num = get_num_logs();
    uint32_t look;

    end_page = find_last_page_of_log(log_num);

    if (num == 1 || log_num == 1) {
        if (!is_wrapped()) {
            start_page = 1;
        } else {
            StartRead(end_page);
            start_page = (end_page + df_NumPages - df_FilePage) % df_NumPages + 1;
        }
    } else {
        // 寻找第一个日志,它前面可能有空隙
        if (list_entry == 1) {
            StartRead(end_page);
            if (end_page > df_FilePage) { // 日志未包装
                start_page = end_page - df_FilePage + 1;
            } else { // 日志已包装
                start_page = (end_page + df_NumPages - df_FilePage) % df_NumPages + 1;
            }
        } else {
            look = log_num-1;
            do {
                start_page = find_last_page_of_log(look) + 1;
                look--;
            } while (start_page <= 0 && look >=1);
        }
    }

    if (start_page == df_NumPages + 1 || start_page == 0) {
        start_page = 1;
    }

    if (end_page == 0) {
        end_page = start_page;
    }

}

// 如果日志已环绕到芯片开头则返回true
bool AP_Logger_Block::is_wrapped(void)
{
    return StartRead(df_NumPages) != 0xFFFF;
}


// 此函数查找最后一个日志号
uint16_t AP_Logger_Block::find_last_log(void)
{
    WITH_SEMAPHORE(sem);
    uint32_t last_page = find_last_page();
    return StartRead(last_page);
}

// 此函数查找最后一个文件的最后一页
uint32_t AP_Logger_Block::find_last_page(void)
{
    uint32_t look;
    uint32_t bottom = 1;
    uint32_t top = df_NumPages;
    uint64_t look_hash;
    uint64_t bottom_hash;
    uint64_t top_hash;

    WITH_SEMAPHORE(sem);

    StartRead(bottom);
    bottom_hash = ((int64_t)GetFileNumber()<<32) | df_FilePage;

    while (top-bottom > 1) {
        look = (top+bottom)/2;
        StartRead(look);
        look_hash = (int64_t)GetFileNumber()<<32 | df_FilePage;
        // 擦除的扇区所以可以忽略上面的所有内容
        if (look_hash >= 0xFFFF00000000) {
            look_hash = 0;
        }

        if (look_hash < bottom_hash) {
            // 向下移动
            top = look;
        } else {
            // 向上移动
            bottom = look;
            bottom_hash = look_hash;
        }
    }

    StartRead(top);
    top_hash = ((int64_t)GetFileNumber()<<32) | df_FilePage;
    if (top_hash >= 0xFFFF00000000) {
        top_hash = 0;
    }
    if (top_hash > bottom_hash) {
        return top;
    }

    return bottom;
}

// 此函数查找特定日志文件的最后一页
uint32_t AP_Logger_Block::find_last_page_of_log(uint16_t log_number)
{
    uint32_t look;
    uint32_t bottom;
    uint32_t top;
    uint64_t look_hash;
    uint64_t check_hash;

    WITH_SEMAPHORE(sem);

    if (is_wrapped()) {
        bottom = StartRead(1);
        if (bottom > log_number) {
            bottom = find_last_page();
            top = df_NumPages;
        } else {
            bottom = 1;
            top = find_last_page();
        }
    } else {
        bottom = 1;
        top = find_last_page();
    }

    check_hash = (int64_t)log_number<<32 | 0xFFFFFFFF;

    while (top-bottom > 1) {
        look = (top+bottom)/2;
        StartRead(look);
        look_hash = (int64_t)GetFileNumber()<<32 | df_FilePage;
        if (look_hash >= 0xFFFF00000000) {
            look_hash = 0;
        }

        if (look_hash > check_hash) {
            // 向下移动
            top = look;
        } else {
            // 向上移动
            bottom = look;
        }
    }

    // 检查顶部页面是否包含目标日志号
    if (StartRead(top) == log_number) {
        return top;
    }

    // 检查底部页面是否包含目标日志号
    if (StartRead(bottom) == log_number) {
        return bottom;
    }

    // 如果在顶部和底部都找不到目标日志号,发送错误消息
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "在top=%X或bot=%X处找不到日志%d的最后一页", int(log_number), unsigned(top), unsigned(bottom));
    return 0;
}

// 获取指定日志条目的大小和UTC时间信息
void AP_Logger_Block::get_log_info(uint16_t list_entry, uint32_t &size, uint32_t &time_utc)
{
    uint32_t start, end;

    WITH_SEMAPHORE(sem);

    // 获取日志的起始和结束页面
    get_log_boundaries(list_entry, start, end);
    
    // 计算日志大小
    if (end >= start) {
        // 如果结束页大于等于起始页,直接计算大小
        size = (end + 1 - start) * (uint32_t)(df_PageSize - sizeof(PageHeader));
    } else {
        // 如果结束页小于起始页(说明日志跨越了存储器边界),需要特殊计算
        size = (df_NumPages + end + 1 - start) * (uint32_t)(df_PageSize - sizeof(PageHeader));
    }

    // 从总大小中减去文件头大小
    size -= sizeof(FileHeader);

    // 开始读取日志起始页
    StartRead(start);

    // 如果是当前正在写入的日志且时间戳为0,则获取当前UTC时间
    if (df_FileTime == 0 && df_FileNumber == df_Write_FileNumber) {
        uint64_t utc_usec;
        if (AP::rtc().get_utc_usec(utc_usec)) {
            df_FileTime = utc_usec / 1000000U;
        }
    }
    time_utc = df_FileTime;
}

// 从缓冲区读取指定大小的数据
bool AP_Logger_Block::BlockRead(uint16_t IntPageAdr, void *pBuffer, uint16_t size)
{
    memcpy(pBuffer, &buffer[IntPageAdr], size);
    return true;
}

// 检查日志记录是否失败
bool AP_Logger_Block::logging_failed() const
{
    // 检查初始化状态
    if (!_initialised) {
        return true;
    }
    // 检查IO线程是否存活
    if (!io_thread_alive()) {
        return true;
    }
    // 检查存储芯片是否已满
    if (chip_full) {
        return true;
    }

    return false;
}

// 检测IO线程是否存活
// 这是日志系统的关键功能,需要非常确定其状态
bool AP_Logger_Block::io_thread_alive() const
{
    // 如果IO线程超过3秒没有心跳,则认为已死亡
    // 除非系统尚未完成初始化
    return (AP_HAL::millis() - io_timer_heartbeat) < 3000U || !hal.scheduler->is_system_initialized();
}

/*
  IO定时器函数,运行在IO线程上
  IO定时器每1ms运行一次(1KHz)。标准闪存芯片写入速度约130KB/s,
  所以每次写入不超过130字节是合理的 - 或者说一页(256字节)。
  W25Q128FV数据手册给出的典型页面编程时间(tpp)为0.7ms,
  这意味着最大写入速率为365KB/s,即每个周期略多于一页。
 */
void AP_Logger_Block::io_timer(void)
{
    uint32_t tnow = AP_HAL::millis();
    io_timer_heartbeat = tnow;

    // 在前2秒不写入任何数据,给闪存芯片一个准备时间
    if (!_initialised || tnow < 2000) {
        return;
    }

    // 处理擦除操作
    if (erase_started) {
        WITH_SEMAPHORE(sem);

        if (InErase()) {
            return;
        }
        // 在最后一页写入日志格式
        StartWrite(df_NumPages+1);
        uint32_t version = DF_LOGGING_FORMAT;
        memset(buffer, 0, df_PageSize);
        memcpy(buffer, &version, sizeof(version));
        FinishWrite();
        erase_started = false;
        chip_full = false;
        status_msg = StatusMessage::ERASE_COMPLETE;
        return;
    }

    // 处理从指定位置开始的擦除操作
    if (df_EraseFrom > 0) {
        WITH_SEMAPHORE(sem);

        // 计算扇区相关参数
        const uint32_t sectors = df_NumPages / df_PagePerSector;
        const uint32_t block_size = df_PagePerBlock * df_PageSize;
        const uint32_t sectors_in_block = block_size / (df_PagePerSector * df_PageSize);
        uint32_t next_sector = get_sector(df_EraseFrom);
        const uint32_t aligned_sector = sectors - (((df_NumPages - df_EraseFrom + 1) / df_PagePerSector) / sectors_in_block) * sectors_in_block;
        
        // 擦除未对齐的扇区
        while (next_sector < aligned_sector) {
            Sector4kErase(next_sector);
            io_timer_heartbeat = AP_HAL::millis();
            next_sector++;
        }
        
        // 擦除对齐的块
        while (next_sector < sectors) {
            SectorErase(next_sector / sectors_in_block);
            io_timer_heartbeat = AP_HAL::millis();
            next_sector += sectors_in_block;
        }
        status_msg = StatusMessage::RECOVERY_COMPLETE;
        df_EraseFrom = 0;
    }

    // 检查是否可以继续写入
    if (!CardInserted() || new_log_pending || chip_full) {
        return;
    }

    // 处理停止日志请求
    if (stop_log_pending) {
        WITH_SEMAPHORE(sem);

        log_write_started = false;

        // 完成之前日志的写入,每次写入一页以避免长时间持有锁
        if (writebuf.available()) {
            write_log_page();
        } else {
            writebuf.clear();
            stop_log_pending = false;
        }

    // 当缓冲区中有足够数据时写入一页
    } else if (writebuf.available() >= df_PageSize - sizeof(struct PageHeader)) {
        WITH_SEMAPHORE(sem);

        write_log_page();
    }
}

// 写入一页日志数据
void AP_Logger_Block::write_log_page()
{
    // 准备页头信息
    struct PageHeader ph;
    ph.FileNumber = df_Write_FileNumber;
    ph.FilePage = df_Write_FilePage;
#if BLOCK_LOG_VALIDATE
    ph.crc = DF_LOGGING_FORMAT + df_Write_FilePage;
#endif
    // 复制页头到缓冲区
    memcpy(buffer, &ph, sizeof(ph));
    
    // 计算页面数据区大小并读取数据
    const uint32_t pagesize = df_PageSize - sizeof(ph);
    uint32_t nbytes = writebuf.read(&buffer[sizeof(ph)], pagesize);
    
    // 如果数据不足一页,用0填充剩余空间
    if (nbytes <  pagesize) {
        memset(&buffer[sizeof(ph) + nbytes], 0, pagesize - nbytes);
    }
    
    // 完成写入并递增页号
    FinishWrite();
    df_Write_FilePage++;
}

// 闪存测试函数
void AP_Logger_Block::flash_test()
{
    const uint32_t pages_to_check = 128;
    
    // 写入测试
    for (uint32_t i=1; i<=pages_to_check; i++) {
        // 每个块开始时执行块擦除
        if ((i-1) % df_PagePerBlock == 0) {
            printf("Block erase %u\n", get_block(i));
            SectorErase(get_block(i));
        }
        // 用页号填充缓冲区
        memset(buffer, uint8_t(i), df_PageSize);
        // 打印进度信息
        if (i<5) {
            printf("Flash fill 0x%x\n", uint8_t(i));
        } else if (i==5) {
            printf("Flash fill pages 5-%u\n", pages_to_check);
        }
        // 将缓冲区写入页面
        BufferToPage(i);
    }
    
    // 读取验证
    for (uint32_t i=1; i<=pages_to_check; i++) {
        // 打印进度信息
        if (i<5) {
            printf("Flash check 0x%x\n", uint8_t(i));
        } else if (i==5) {
            printf("Flash check pages 5-%u\n", pages_to_check);
        }
        // 读取页面到缓冲区
        PageToBuffer(i);
        
        // 验证数据
        uint32_t bad_bytes = 0;
        uint32_t first_bad_byte = 0;
        for (uint32_t j=0; j<df_PageSize; j++) {
            if (buffer[j] != uint8_t(i)) {
                bad_bytes++;
                if (bad_bytes == 1) {
                    first_bad_byte = j;
                }
            }
        }
        // 如果发现错误,打印错误信息
        if (bad_bytes > 0) {
            printf("Test failed: page %u, %u of %u bad bytes, first=0x%x\n",
                i, bad_bytes, df_PageSize, buffer[first_bad_byte]);
        }
    }
}

#endif // HAL_LOGGING_BLOCK_ENABLED
