#pragma once

#include <stdint.h>
#include "AP_HAL_Namespace.h"

// Storage类 - 用于处理持久化存储操作的抽象基类
class AP_HAL::Storage {
public:
    // 初始化存储系统
    virtual void init() = 0;

    // 擦除存储内容
    // 返回值: true表示擦除成功,false表示失败
    virtual bool erase();

    // 从存储器读取数据块
    // @param dst: 目标缓冲区指针
    // @param src: 源地址偏移量
    // @param n: 要读取的字节数
    virtual void read_block(void *dst, uint16_t src, size_t n) = 0;

    // 向存储器写入数据块
    // @param dst: 目标地址偏移量
    // @param src: 源数据指针
    // @param n: 要写入的字节数
    virtual void write_block(uint16_t dst, const void* src, size_t n) = 0;

    // 定时器滴答回调函数,用于周期性任务
    virtual void _timer_tick(void) {};

    // 检查存储系统是否健康
    // 返回值: true表示健康,false表示异常
    virtual bool healthy(void) { return true; }

    // 获取存储器的直接访问指针和大小
    // @param ptr: 输出参数,存储器指针
    // @param size: 输出参数,存储器大小
    // 返回值: true表示成功获取,false表示不支持直接访问
    virtual bool get_storage_ptr(void *&ptr, size_t &size) { return false; }
};
