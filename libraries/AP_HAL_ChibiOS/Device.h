/*
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// 防止头文件重复包含
#pragma once

// 包含必要的头文件
#include <inttypes.h>
#include <AP_HAL/HAL.h>
#if !defined(HAL_BOOTLOADER_BUILD)
#include "Semaphores.h"
#else
#include <AP_HAL_Empty/Semaphores.h>
#endif
#include "AP_HAL_ChibiOS.h"

// 如果启用了I2C、SPI或WSPI中的任意一个
#if HAL_USE_I2C == TRUE || HAL_USE_SPI == TRUE || HAL_USE_WSPI == TRUE

#include "Scheduler.h"
#include "shared_dma.h"
#include "hwdef/common/bouncebuffer.h"

namespace ChibiOS {

// 设备总线类,用于管理I2C/SPI总线访问
class DeviceBus {
public:
    // 构造函数,设置线程优先级
    DeviceBus(uint8_t _thread_priority = APM_I2C_PRIORITY);
    
    // 构造函数重载,可指定是否使用AXI SRAM
    DeviceBus(uint8_t _thread_priority, bool axi_sram);

    // 指向下一个设备总线的指针,用于链表
    struct DeviceBus *next;
#if defined(HAL_BOOTLOADER_BUILD)
    // bootloader模式下使用空信号量
    Empty::Semaphore semaphore;
#else
    // 正常模式下使用实际信号量
    Semaphore semaphore;
#endif
    // DMA句柄
    Shared_DMA *dma_handle;

    // 注册周期性回调函数
    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb, AP_HAL::Device *hal_device);
    // 调整定时器周期
    bool adjust_timer(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec);
    // 总线线程函数
    static void bus_thread(void *arg);

    // 设置DMA弹性缓冲区
    bool bouncebuffer_setup(const uint8_t *&buf_tx, uint16_t tx_len,
                            uint8_t *&buf_rx, uint16_t rx_len) WARN_IF_UNUSED;
    // 完成DMA弹性缓冲区传输                            
    void bouncebuffer_finish(const uint8_t *buf_tx, uint8_t *buf_rx, uint16_t rx_len);

private:
    // 回调函数信息结构体
    struct callback_info {
        struct callback_info *next;  // 指向下一个回调的指针
        AP_HAL::Device::PeriodicCb cb;  // 回调函数
        uint32_t period_usec;  // 回调周期(微秒)
        uint64_t next_usec;    // 下次调用时间
    } *callbacks;
    uint8_t thread_priority;   // 线程优先级
    thread_t* thread_ctx;      // 线程上下文
    bool thread_started;       // 线程是否已启动
    AP_HAL::Device *hal_device;  // HAL设备指针

    // DMA安全传输的弹性缓冲区
    struct bouncebuffer_t *bounce_buffer_tx;  // 发送弹性缓冲区
    struct bouncebuffer_t *bounce_buffer_rx;  // 接收弹性缓冲区
};

}

#endif // I2C or SPI
