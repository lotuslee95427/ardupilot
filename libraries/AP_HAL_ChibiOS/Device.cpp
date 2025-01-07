/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// 包含必要的头文件
#include <hal.h>
#include "Device.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <stdio.h>

// 如果启用了I2C、SPI或WSPI中的任意一个
#if HAL_USE_I2C == TRUE || HAL_USE_SPI == TRUE || HAL_USE_WSPI == TRUE

#include "Scheduler.h"
#include "Semaphores.h"
#include "Util.h"
#include "hwdef/common/stm32_util.h"

// 如果未定义设备线程栈大小,则默认为1024字节
#ifndef HAL_DEVICE_THREAD_STACK
#define HAL_DEVICE_THREAD_STACK 1024
#endif

using namespace ChibiOS;

// 声明外部HAL实例引用
extern const AP_HAL::HAL& hal;

// DeviceBus构造函数,初始化总线优先级和弹性缓冲区
DeviceBus::DeviceBus(uint8_t _thread_priority) :
        thread_priority(_thread_priority)
{
    bouncebuffer_init(&bounce_buffer_tx, 10, false);
    bouncebuffer_init(&bounce_buffer_rx, 10, false);
}

// DeviceBus构造函数重载,可指定是否使用AXI SRAM
DeviceBus::DeviceBus(uint8_t _thread_priority, bool axi_sram) :
        thread_priority(_thread_priority)
{
    bouncebuffer_init(&bounce_buffer_tx, 10, axi_sram);
    bouncebuffer_init(&bounce_buffer_rx, 10, axi_sram);
}

/*
  per-bus callback thread
*/
// 总线回调线程函数,处理周期性回调
void DeviceBus::bus_thread(void *arg)
{
    struct DeviceBus *binfo = (struct DeviceBus *)arg;

    while (true) {
        uint64_t now = AP_HAL::micros64();
        DeviceBus::callback_info *callback;

        // 遍历所有回调,找到需要执行的回调
        for (callback = binfo->callbacks; callback; callback = callback->next) {
            if (now >= callback->next_usec) {
                // 更新下次执行时间
                while (now >= callback->next_usec) {
                    callback->next_usec += callback->period_usec;
                }
                // 在信号量保护下执行回调
                WITH_SEMAPHORE(binfo->semaphore);
                callback->cb();
            }
        }

        // 计算下一次需要执行的时间
        uint64_t next_needed = 0;
        now = AP_HAL::micros64();

        for (callback = binfo->callbacks; callback; callback = callback->next) {
            if (next_needed == 0 ||
                callback->next_usec < next_needed) {
                next_needed = callback->next_usec;
                if (next_needed < now) {
                    next_needed = now;
                }
            }
        }

        // 延时最多50ms,以处理新添加的回调
        uint32_t delay = 50000;
        if (next_needed >= now && next_needed - now < delay) {
            delay = next_needed - now;
        }
        // 最小延时100us,避免一个线程完全占用CPU
        if (delay < 100) {
            delay = 100;
        }
        hal.scheduler->delay_microseconds(delay);
    }
    return;
}

#if CH_CFG_USE_HEAP == TRUE
// 注册周期性回调函数
AP_HAL::Device::PeriodicHandle DeviceBus::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb, AP_HAL::Device *_hal_device)
{
    // 如果线程未启动,则创建并启动线程
    if (!thread_started) {
        thread_started = true;

        hal_device = _hal_device;
        // 设置线程名称
        const uint8_t name_len = 7;
        char *name = (char *)malloc(name_len);
        if (name == nullptr){
            return nullptr;
        }
        // 根据总线类型设置线程名称
        switch (hal_device->bus_type()) {
        case AP_HAL::Device::BUS_TYPE_I2C:
            snprintf(name, name_len, "I2C%u",
                     hal_device->bus_num());
            break;

        case AP_HAL::Device::BUS_TYPE_SPI:
            snprintf(name, name_len, "SPI%u",
                     hal_device->bus_num());
            break;
        default:
            break;
        }

        // 创建线程
        thread_ctx = thread_create_alloc(THD_WORKING_AREA_SIZE(HAL_DEVICE_THREAD_STACK),
                                         name,
                                         thread_priority,           /* Initial priority.    */
                                         DeviceBus::bus_thread,    /* Thread function.     */
                                         this);                     /* Thread parameter.    */
        if (thread_ctx == nullptr) {
            AP_HAL::panic("Failed to create bus thread %s", name);
        }
    }
    // 创建新的回调信息结构
    DeviceBus::callback_info *callback = NEW_NOTHROW DeviceBus::callback_info;
    if (callback == nullptr) {
        return nullptr;
    }
    callback->cb = cb;
    callback->period_usec = period_usec;
    callback->next_usec = AP_HAL::micros64() + period_usec;

    // 将回调添加到链表头部
    callback->next = callbacks;
    callbacks = callback;

    return callback;
}
#endif // CH_CFG_USE_HEAP

/*
 * Adjust the timer for the next call: it needs to be called from the bus
 * thread, otherwise it will race with it
 */
// 调整定时器下次调用时间,必须在总线线程中调用
bool DeviceBus::adjust_timer(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    if (chThdGetSelfX() != thread_ctx) {
        return false;
    }

    DeviceBus::callback_info *callback = static_cast<DeviceBus::callback_info *>(h);

    callback->period_usec = period_usec;
    callback->next_usec = AP_HAL::micros64() + period_usec;

    return true;
}

/*
  setup to use DMA-safe bouncebuffers for device transfers
 */
// 设置使用DMA安全的弹性缓冲区进行设备传输
bool DeviceBus::bouncebuffer_setup(const uint8_t *&buf_tx, uint16_t tx_len,
                                   uint8_t *&buf_rx, uint16_t rx_len)
{
    if (buf_rx) {
        if (!bouncebuffer_setup_read(bounce_buffer_rx, &buf_rx, rx_len)) {
            return false;
        }
    }
    if (buf_tx) {
        if (!bouncebuffer_setup_write(bounce_buffer_tx, &buf_tx, tx_len)) {
            if (buf_rx) {
                bouncebuffer_abort(bounce_buffer_rx);
            }
            return false;
        }
    }
    return true;
}

/*
  complete a transfer using DMA bounce buffer
 */
// 完成使用DMA弹性缓冲区的传输
void DeviceBus::bouncebuffer_finish(const uint8_t *buf_tx, uint8_t *buf_rx, uint16_t rx_len)
{
    if (buf_rx) {
        bouncebuffer_finish_read(bounce_buffer_rx, buf_rx, rx_len);
    }
    if (buf_tx) {
        bouncebuffer_finish_write(bounce_buffer_tx, buf_tx);
    }
}

#endif // HAL_USE_I2C || HAL_USE_SPI
