#pragma once

// 包含必要的头文件
#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_ChibiOS_Namespace.h"
#include "hwdef/common/flash.h"

// ChibiOS Flash类,继承自AP_HAL::Flash,用于管理STM32 Flash存储器
class ChibiOS::Flash : public AP_HAL::Flash {
public:
    // 获取指定页的起始地址
    uint32_t getpageaddr(uint32_t page) override { return stm32_flash_getpageaddr(page); }
    
    // 获取指定页的大小
    uint32_t getpagesize(uint32_t page) override { return stm32_flash_getpagesize(page); }
    
    // 获取Flash总页数
    uint32_t getnumpages(void) override { return stm32_flash_getnumpages(); }
    
    // 擦除指定页
    // 使用信号量保护Flash操作
    bool erasepage(uint32_t page) override {
        WITH_SEMAPHORE(sem);
        return stm32_flash_erasepage(page); }
    
    // 写入数据到指定地址
    // @param addr: 写入的目标地址
    // @param buf: 要写入的数据缓冲区
    // @param count: 要写入的字节数
    bool write(uint32_t addr, const void *buf, uint32_t count) override {
        WITH_SEMAPHORE(sem);
        return stm32_flash_write(addr, buf, count); }
    
    // 设置Flash解锁状态
    // @param set: true保持解锁,false允许自动锁定
    void keep_unlocked(bool set) override { stm32_flash_keep_unlocked(set); }
    
    // 检查指定页是否已被擦除
    bool ispageerased(uint32_t page) override { return stm32_flash_ispageerased(page); }

private:
    // 用于保护Flash操作的信号量
    HAL_Semaphore sem;
};
