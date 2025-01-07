// 包含BetterStream头文件
#include "BetterStream.h"

// 包含print_vprintf头文件
#include "print_vprintf.h"

// 格式化打印函数,使用可变参数
void AP_HAL::BetterStream::printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
}

// 格式化打印函数的具体实现
void AP_HAL::BetterStream::vprintf(const char *fmt, va_list ap)
{
    print_vprintf(this, fmt, ap);
}

// 写入字符串函数
size_t AP_HAL::BetterStream::write(const char *str)
{
    return write((const uint8_t *)str, strlen(str));
}

// 读取单个字节函数
int16_t AP_HAL::BetterStream::read()
{
    uint8_t b;
    if (!read(b)) {
        return -1;
    }
    return b;
}

// 从流中读取指定数量字节到缓冲区
ssize_t AP_HAL::BetterStream::read(uint8_t *buffer, uint16_t count)
{
    size_t offset = 0;
    while (count--) {
        const int16_t x = read();
        if (x == -1) {
            return offset;
        }
        buffer[offset++] = (uint8_t)x;
    }
    return offset;
}
