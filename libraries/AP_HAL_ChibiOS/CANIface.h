/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2014 Pavel Kirienko
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

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
 *
 * Code by Siddharth Bharat Purohit
 */

// 防止头文件重复包含
#pragma once

// 包含ChibiOS HAL头文件
#include "AP_HAL_ChibiOS.h"

// 根据不同的处理器型号选择不同的CAN驱动实现
# if defined(STM32H7XX) || defined(STM32G4)
#include "CANFDIface.h"
# else

// 如果定义了CAN接口数量
#if HAL_NUM_CAN_IFACES

// 包含bxCAN驱动头文件
#include "bxcan.hpp"

// 如果未定义CAN接收队列大小,则默认设置为128
#ifndef HAL_CAN_RX_QUEUE_SIZE
#define HAL_CAN_RX_QUEUE_SIZE 128
#endif

// 静态断言确保接收队列大小不超过254
static_assert(HAL_CAN_RX_QUEUE_SIZE <= 254, "Invalid CAN Rx queue size");

/**
 * Single CAN iface.
 * The application shall not use this directly.
 */
// ChibiOS CAN接口类,继承自AP_HAL::CANIface
class ChibiOS::CANIface : public AP_HAL::CANIface
{
    // CAN帧标识符相关常量定义
    static constexpr unsigned long IDE = (0x40000000U); // Identifier Extension - 标识符扩展位
    static constexpr unsigned long STID_MASK = (0x1FFC0000U); // Standard Identifier Mask - 标准标识符掩码
    static constexpr unsigned long EXID_MASK = (0x1FFFFFFFU); // Extended Identifier Mask - 扩展标识符掩码
    static constexpr unsigned long RTR       = (0x20000000U); // Remote Transmission Request - 远程传输请求位
    static constexpr unsigned long DLC_MASK  = (0x000F0000U); // Data Length Code - 数据长度代码掩码

    // 临界区保护类,用于保护共享资源访问
    struct CriticalSectionLocker {
        CriticalSectionLocker()
        {
            chSysLock();
        }
        ~CriticalSectionLocker()
        {
            chSysUnlock();
        }
    };

    // CAN总线时序参数结构体
    struct Timings {
        uint16_t prescaler;  // 预分频器值
        uint8_t sjw;         // 同步跳转宽度
        uint8_t bs1;         // 位段1
        uint8_t bs2;         // 位段2

        Timings()
            : prescaler(0)
            , sjw(0)
            , bs1(0)
            , bs2(0)
        { }
    };

    // 定义发送邮箱数量和过滤器数量
    enum { NumTxMailboxes = 3 };  // 发送邮箱数量为3
    enum { NumFilters = 14 };     // 过滤器数量为14
    static const uint32_t TSR_ABRQx[NumTxMailboxes];  // 发送中止请求位数组

    ChibiOS::bxcan::CanType* can_;  // CAN外设寄存器指针

    // ISR接收处理器状态变量,放在类中以避免扩展所有线程的栈大小
    AP_HAL::CANFrame isr_rx_frame;  // ISR接收帧缓存
    CanRxItem isr_rx_item;          // ISR接收项缓存

    // 接收和发送相关缓冲区
    CanRxItem rx_buffer[HAL_CAN_RX_QUEUE_SIZE];  // 接收缓冲区数组
    ByteBuffer rx_bytebuffer_;                    // 接收字节缓冲区
    ObjectBuffer<CanRxItem> rx_queue_;            // 接收队列
    CanTxItem pending_tx_[NumTxMailboxes];       // 待发送项数组

    // 状态标志位
    bool irq_init_:1;      // 中断初始化标志
    bool initialised_:1;   // 接口初始化标志  
    bool had_activity_:1;  // 活动标志
    AP_HAL::BinarySemaphore *sem_handle;  // 二值信号量句柄

    const uint8_t self_index_;  // 接口索引

    // 内部功能函数声明
    bool computeTimings(uint32_t target_bitrate, Timings& out_timings);  // 计算CAN总线时序参数
    void setupMessageRam(void);                                          // 设置消息RAM
    bool readRxFIFO(uint8_t fifo_index);                                // 读取接收FIFO
    void discardTimedOutTxMailboxes(uint64_t current_time);             // 丢弃超时的发送邮箱
    bool canAcceptNewTxFrame(const AP_HAL::CANFrame& frame) const;      // 检查是否可以接受新的发送帧
    bool isRxBufferEmpty() const;                                       // 检查接收缓冲区是否为空
    bool recover_from_busoff();                                         // 从总线关闭状态恢复
    void pollErrorFlags();                                              // 轮询错误标志
    void checkAvailable(bool& read, bool& write,                        // 检查读写可用性
                        const AP_HAL::CANFrame* pending_tx) const;
    bool waitMsrINakBitStateChange(bool target_state);                  // 等待MSR INAK位状态改变
    void handleTxMailboxInterrupt(uint8_t mailbox_index,               // 处理发送邮箱中断
                                 bool txok, 
                                 const uint64_t timestamp_us);
    void initOnce(bool enable_irq);                                     // 初始化一次

#if !defined(HAL_BOOTLOADER_BUILD)
    /*
      additional statistics
     */
    // 总线统计信息结构体
    struct bus_stats : public AP_HAL::CANIface::bus_stats_t {
        uint32_t num_events;  // 事件数量
        uint32_t esr;        // 错误状态寄存器值
    } stats;
#endif

public:
    /******************************************
     *   Common CAN methods                   *
     * ****************************************/
    CANIface(uint8_t index);   // 构造函数
    CANIface();                // 默认构造函数
    static uint8_t next_interface;  // 下一个接口索引

    // 初始化CAN外设
    bool init(const uint32_t bitrate, const OperatingMode mode) override;

    // 发送帧到发送FIFO,返回负值表示错误,0表示缓冲区满,1表示成功推入FIFO
    int16_t send(const AP_HAL::CANFrame& frame, uint64_t tx_deadline,
                 CanIOFlags flags) override;

    // 从接收缓冲区接收帧,返回负值表示错误,0表示无可用数据,1表示成功取出一帧
    int16_t receive(AP_HAL::CANFrame& out_frame, uint64_t& out_timestamp_us,
                    CanIOFlags& out_flags) override;

#if !defined(HAL_BOOTLOADER_BUILD)
    // 配置过滤器以忽略不需要处理的帧
    bool configureFilters(const CanFilterConfig* filter_configs,
                          uint16_t num_configs) override;
#endif

    // 在BxCAN中总线关闭错误会自动清除,所以始终返回false
    bool is_busoff() const override
    {
        return false;
    }

    // 清除接收缓冲区
    void clear_rx() override;

    // 获取过滤器配置数量
    uint16_t getNumFilters() const override
    {
        return NumFilters;
    }

    // 获取发现的错误总数
#if !defined(HAL_BUILD_AP_PERIPH) && !defined(HAL_BOOTLOADER_BUILD)
    uint32_t getErrorCount() const override;
#endif

    // 返回init是否成功调用
    bool is_initialized() const override
    {
        return initialised_;
    }

    /******************************************
     * Select Method                          *
     * ****************************************/
    // 等待选定事件可用,超时返回false,否则返回true
    bool select(bool &read, bool &write,
                const AP_HAL::CANFrame* const pending_tx,
                uint64_t blocking_deadline) override;
    
    // 设置事件等待句柄
    bool set_event_handle(AP_HAL::BinarySemaphore *handle) override;

#if !defined(HAL_BUILD_AP_PERIPH) && !defined(HAL_BOOTLOADER_BUILD)
    // 获取统计信息文本并返回其大小,结果可通过@SYS/can0_stats.txt或@SYS/can1_stats.txt访问
    void get_stats(ExpandingString &str) override;
#endif

#if !defined(HAL_BOOTLOADER_BUILD)
    /*
      return statistics structure
     */
    // 返回统计信息结构体
    const bus_stats_t *get_statistics(void) const override {
        return &stats;
    }
#endif

    /************************************
     * Methods used inside interrupt    *
     ************************************/
    void handleTxInterrupt(uint64_t timestamp_us);                      // 处理发送中断
    void handleRxInterrupt(uint8_t fifo_index, uint64_t timestamp_us); // 处理接收中断
    
    // 处理发生的错误,执行必要操作如丢弃帧和计数错误
    void pollErrorFlagsFromISR(void);

    // CAN外设寄存器结构数组
    static constexpr bxcan::CanType* const Can[HAL_NUM_CAN_IFACES] = { HAL_CAN_BASE_LIST };

protected:
    // 添加项到接收队列
    bool add_to_rx_queue(const CanRxItem &rx_item) override {
        return rx_queue_.push(rx_item);
    }

    // 获取接口编号
    int8_t get_iface_num(void) const override {
        return self_index_;
    }
};
#endif //HAL_NUM_CAN_IFACES
#endif //# if defined(STM32H7XX) || defined(STM32G4)
