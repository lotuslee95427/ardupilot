/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//
//      Copyright (c) 2010 Michael Smith. All rights reserved.
//
//
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Namespace.h>

#include <stdarg.h>
#include <unistd.h>

// BetterStream类提供了一个改进的流式接口
class AP_HAL::BetterStream {
public:
    // 格式化打印函数
    virtual void printf(const char *, ...) FMT_PRINTF(2, 3);
    // 使用va_list的格式化打印函数
    virtual void vprintf(const char *, va_list);

    // 打印字符串
    void print(const char *str) { write(str); }
    // 打印字符串并添加回车换行
    void println(const char *str) { printf("%s\r\n", str); }

    // 写入单个字节
    virtual size_t write(uint8_t) = 0;
    // 写入字节缓冲区
    virtual size_t write(const uint8_t *buffer, size_t size) = 0;
    // 写入字符串
    virtual size_t write(const char *str);

    // 返回可读取的字节数
    virtual uint32_t available() = 0;

    // 读取单个字节
    virtual int16_t read(void);
    // 读取单个字节到引用参数
    virtual bool read(uint8_t &b) WARN_IF_UNUSED = 0;

    // 丢弃所有可读取的数据
    // 没有基类实现以强制子类高效实现
    // 循环处理2^32-1字节会很糟糕
    // 如果丢弃失败(如端口锁定)则返回false
    virtual bool discard_input() = 0; 

    // 从缓冲区读取指定数量的字节
    // 错误时返回-1(如端口锁定)
    // 否则返回读取的字节数
    virtual ssize_t read(uint8_t *buffer, uint16_t count);

    /* 注意:txspace在FastSerial库的BetterStream中是一个传统成员
     * 就关注点而言,它属于available()函数 */
    // 返回可用于发送的缓冲区空间
    virtual uint32_t txspace() = 0;
};
