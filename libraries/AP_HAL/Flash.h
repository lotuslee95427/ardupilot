/*
  Flash读写接口
  定义了Flash存储器的基本操作接口
 */
#pragma once

#include <stdint.h>
#include "AP_HAL_Namespace.h"

class AP_HAL::Flash {
public:
    // 获取指定页的起始地址
    virtual uint32_t getpageaddr(uint32_t page) = 0;

    // 获取指定页的大小(字节)
    virtual uint32_t getpagesize(uint32_t page) = 0;

    // 获取Flash总页数
    virtual uint32_t getnumpages(void) = 0;

    // 擦除指定页
    // @param page: 要擦除的页号
    // @return: 擦除成功返回true,失败返回false
    virtual bool erasepage(uint32_t page) = 0;

    // 写入数据到Flash
    // @param addr: 写入的目标地址
    // @param buf: 要写入的数据缓冲区
    // @param count: 要写入的字节数
    // @return: 写入成功返回true,失败返回false
    virtual bool write(uint32_t addr, const void *buf, uint32_t count) = 0;

    // 设置Flash解锁状态
    // @param set: true保持解锁状态,false允许自动锁定
    virtual void keep_unlocked(bool set) = 0;

    // 检查指定页是否已被擦除
    // @param page: 要检查的页号
    // @return: 页已擦除返回true,未擦除返回false
    virtual bool ispageerased(uint32_t page) = 0;
};
