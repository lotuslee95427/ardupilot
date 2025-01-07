// 包含必要的头文件
#include "AP_HAL.h"
#include "Util.h"
#include "utility/print_vprintf.h"

// 根据不同平台包含不同的时间相关头文件
#if defined(__APPLE__) && defined(__MACH__)
#include <sys/time.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include "ch.h"
#include "hal.h"
#else
#include <time.h>
#endif

/* 
 * 辅助类BufferPrinter实现AP_HAL::Print接口,用于格式化字符串输出
 * 将格式化后的字符串写入指定的缓冲区
 */
class BufferPrinter : public AP_HAL::BetterStream {
public:
    // 构造函数,传入目标字符串缓冲区和大小
    BufferPrinter(char* str, size_t size)  :
        _offs(0), _str(str), _size(size)  {}

    // 写入单个字符
    size_t write(uint8_t c) override {
        if (_offs < _size) {
            _str[_offs] = c;
        }
        _offs++;
        return 1;
    }

    // 写入字符串缓冲区
    size_t write(const uint8_t *buffer, size_t size) override {
        size_t n = 0;
        while (size--) {
            n += write(*buffer++);
        }
        return n;
    }

    size_t _offs;              // 当前写入位置偏移量
    char* const  _str;         // 目标字符串缓冲区
    const size_t _size;        // 缓冲区大小

    // 以下是BetterStream接口要求的函数,但在这里未使用
    uint32_t available() override { return 0; }
    bool read(uint8_t &b) override { return false; }
    uint32_t txspace() override { return 0; }
    bool discard_input() override { return false; }
};

/*
 * 格式化字符串输出函数,类似标准C库的snprintf
 * @param str: 目标字符串缓冲区
 * @param size: 缓冲区大小
 * @param format: 格式化字符串
 * @return: 实际写入的字符数
 */
int AP_HAL::Util::snprintf(char* str, size_t size, const char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    int res = vsnprintf(str, size, format, ap);
    va_end(ap);
    return res;
}

/*
 * 格式化字符串输出函数的va_list版本
 * @param str: 目标字符串缓冲区
 * @param size: 缓冲区大小
 * @param format: 格式化字符串
 * @param ap: 可变参数列表
 * @return: 实际写入的字符数
 */
int AP_HAL::Util::vsnprintf(char* str, size_t size, const char *format, va_list ap)
{
    // 注意:size==0需要特殊处理,因为vasprintf()等函数依赖返回值表示
    // 如果有足够空间时会打印的字节数
    BufferPrinter buf(str, size?size-1:0);
    print_vprintf(&buf, format, ap);
    // 添加字符串结束符
    size_t ret = buf._offs;
    if (ret < size) {
        // 如果字符串适合缓冲区大小,在末尾添加\0
        str[ret] = '\0';
    } else if (size > 0) {
        // 如果字符串超出缓冲区大小,在size-1处添加\0
        str[size-1] = 0;
    }
    return int(ret);
}

/*
 * 设置软件锁定状态
 * @param b: true表示锁定,false表示解锁
 */
void AP_HAL::Util::set_soft_armed(const bool b)
{
    if (b != soft_armed) {
        soft_armed = b;
        last_armed_change_ms = AP_HAL::millis();
        if (!was_watchdog_reset()) {
            persistent_data.armed = b;
        }
    }
}
