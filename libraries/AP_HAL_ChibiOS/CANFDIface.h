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

// 如果定义了CAN接口数量
#if HAL_NUM_CAN_IFACES

// 如果未定义CAN接收队列大小,则默认为128
#ifndef HAL_CAN_RX_QUEUE_SIZE
#define HAL_CAN_RX_QUEUE_SIZE 128
#endif

// 静态断言确保接收队列大小不超过254
static_assert(HAL_CAN_RX_QUEUE_SIZE <= 254, "Invalid CAN Rx queue size");

/**
 * Single CAN iface.
 * 单个CAN接口的实现类
 */
class ChibiOS::CANIface : public AP_HAL::CANIface
{
    // CAN帧标识符相关常量定义
    static constexpr unsigned long IDE = (0x40000000U); // Identifier Extension - 标识符扩展位
    static constexpr unsigned long STID_MASK = (0x1FFC0000U); // Standard Identifier Mask - 标准标识符掩码
    static constexpr unsigned long EXID_MASK = (0x1FFFFFFFU); // Extended Identifier Mask - 扩展标识符掩码
    static constexpr unsigned long RTR       = (0x20000000U); // Remote Transmission Request - 远程传输请求位
    static constexpr unsigned long DLC_MASK  = (0x000F0000U); // Data Length Code - 数据长度代码掩码
    static constexpr unsigned long FDF       = (0x00200000U); // CAN FD Frame - CAN FD帧标志
    static constexpr unsigned long BRS       = (0x00100000U); // Bit Rate Switching - 比特率切换标志
    

    /**
     * CANx register sets
     * CAN外设寄存器集类型定义
     */
    typedef FDCAN_GlobalTypeDef CanType;

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

    // 消息RAM结构定义,存储各种过滤器和FIFO的起始地址
    struct MessageRAM {
        uint32_t StandardFilterSA;  // 标准过滤器起始地址
        uint32_t ExtendedFilterSA;  // 扩展过滤器起始地址
        uint32_t RxFIFO0SA;        // 接收FIFO0起始地址
        uint32_t RxFIFO1SA;        // 接收FIFO1起始地址
        uint32_t TxFIFOQSA;        // 发送FIFO队列起始地址
        uint32_t EndAddress;       // 结束地址
    } MessageRam_;

    // CAN总线时序参数结构
    struct Timings {
        uint16_t sample_point_permill;  // 采样点位置(千分比)
        uint16_t prescaler;             // 预分频器值
        uint8_t sjw;                    // 同步跳转宽度
        uint8_t bs1;                    // 位段1
        uint8_t bs2;                    // 位段2

        Timings()
            : sample_point_permill(0)
            , prescaler(0)
            , sjw(0)
            , bs1(0)
            , bs2(0)
        { }
    };

    // 发送邮箱数量定义
    enum { NumTxMailboxes = 32 };

    // FDCAN消息RAM偏移量
    static uint32_t FDCANMessageRAMOffset_;

    CanType* can_;                      // CAN外设寄存器指针
    
    CanRxItem rx_buffer[HAL_CAN_RX_QUEUE_SIZE];  // 接收缓冲区
    ByteBuffer rx_bytebuffer_;          // 字节缓冲区
    ObjectBuffer<CanRxItem> rx_queue_;  // 接收队列
    CanTxItem pending_tx_[NumTxMailboxes];  // 待发送项数组
    uint8_t peak_tx_mailbox_index_;     // 发送邮箱峰值索引
    bool irq_init_;                     // 中断初始化标志
    bool initialised_;                  // 接口初始化标志
    bool had_activity_;                 // 活动标志
    AP_HAL::BinarySemaphore *sem_handle;  // 二值信号量句柄

    const uint8_t self_index_;          // 接口索引

    // 计算CAN总线时序参数
    bool computeTimings(uint32_t target_bitrate, Timings& out_timings) const;
    // 计算CAN FD总线时序参数
    bool computeFDTimings(uint32_t target_bitrate, Timings& out_timings) const;

    // 设置消息RAM
    void setupMessageRam(void);

    // 从FIFO读取接收数据
    bool readRxFIFO(uint8_t fifo_index);

    // 丢弃超时的发送邮箱
    void discardTimedOutTxMailboxes(uint64_t current_time);

    // 检查是否可以接受新的发送帧
    bool canAcceptNewTxFrame() const;

    // 检查接收缓冲区是否为空
    bool isRxBufferEmpty() const;

    // 从总线关闭状态恢复
    bool recover_from_busoff();

    // 轮询错误标志
    void pollErrorFlags();

    // 检查读写可用性
    void checkAvailable(bool& read, bool& write,
                        const AP_HAL::CANFrame* pending_tx) const;

    // 清除错误状态
    void clearErrors();

    static uint32_t FDCAN2MessageRAMOffset_;  // FDCAN2消息RAM偏移量
    static bool clock_init_;                  // 时钟初始化标志

    bool _detected_bus_off;                   // 总线关闭检测标志
    Timings timings, fdtimings;              // CAN和CAN FD时序参数
    uint32_t _bitrate, _fdbitrate;           // CAN和CAN FD比特率

    /*
      additional statistics
      附加统计信息结构
     */
    struct bus_stats : public AP_HAL::CANIface::bus_stats_t {
        uint32_t num_events;        // 事件数量
        uint32_t ecr;              // 错误计数寄存器值
        uint32_t fdf_tx_requests;  // CAN FD发送请求数
        uint32_t fdf_tx_success;   // CAN FD发送成功数
        uint32_t fdf_rx_received;  // CAN FD接收成功数
    } stats;

public:
    /******************************************
     *   Common CAN methods                   *
     * CAN接口公共方法                        *
     * ****************************************/
    CANIface(uint8_t index);
    CANIface();
    static uint8_t next_interface;  // 下一个接口索引

    // Initialise CAN Peripheral
    // 初始化CAN外设(不支持CAN FD)
    bool init(const uint32_t bitrate, const OperatingMode mode) override {
        return init(bitrate, 0, mode);
    }

    // 初始化CAN外设(支持CAN FD)
    bool init(const uint32_t bitrate, const uint32_t fdbitrate, const OperatingMode mode) override;

    // Put frame into Tx FIFO returns negative on error, 0 on buffer full, 
    // 1 on successfully pushing a frame into FIFO
    // 发送CAN帧,返回负值表示错误,0表示缓冲区满,1表示成功
    int16_t send(const AP_HAL::CANFrame& frame, uint64_t tx_deadline,
                 CanIOFlags flags) override;

    // Receive frame from Rx Buffer, returns negative on error, 0 on nothing available, 
    // 1 on successfully poping a frame
    // 接收CAN帧,返回负值表示错误,0表示无数据,1表示成功
    int16_t receive(AP_HAL::CANFrame& out_frame, uint64_t& out_timestamp_us,
                    CanIOFlags& out_flags) override;

    // Set Filters to ignore frames not to be handled by us
    // 配置过滤器,忽略不需要处理的帧
    bool configureFilters(const CanFilterConfig* filter_configs,
                          uint16_t num_configs) override;

    // returns true if busoff state was detected and not handled yet
    // 检查是否检测到总线关闭状态且未处理
    bool is_busoff() const override
    {
        return _detected_bus_off;
    }

    // Clear the Rx buffer
    // 清空接收缓冲区
    void clear_rx() override;

    // Get number of Filter configurations
    // 获取过滤器配置数量
    uint16_t getNumFilters() const override;

    // Get total number of Errors discovered
    // 获取发现的错误总数
    uint32_t getErrorCount() const override;

    // returns true if init was successfully called
    // 检查是否成功初始化
    bool is_initialized() const override
    {
        return initialised_;
    }

    /******************************************
     * Select Method                          *
     * 选择方法                               *
     * ****************************************/
    // wait until selected event is available, false when timed out waiting else true
    // 等待选定事件可用,超时返回false,否则返回true
    bool select(bool &read, bool &write,
                const AP_HAL::CANFrame* const pending_tx,
                uint64_t blocking_deadline) override;

    // setup event handle for waiting on events
    // 设置事件等待句柄
    bool set_event_handle(AP_HAL::BinarySemaphore *handle) override;

#if !defined(HAL_BOOTLOADER_BUILD)
    // fetch stats text and return the size of the same,
    // results available via @SYS/can0_stats.txt or @SYS/can1_stats.txt 
    // 获取统计信息文本
    void get_stats(ExpandingString &str) override;

    /*
      return statistics structure
      返回统计信息结构
     */
    const bus_stats_t *get_statistics(void) const override {
        return &stats;
    }

#endif
    /************************************
     * Methods used inside interrupt    *
     * 中断内部使用的方法               *
     ************************************/
    void handleTxCompleteInterrupt(uint64_t timestamp_us);  // 处理发送完成中断
    void handleRxInterrupt(uint8_t fifo_index);            // 处理接收中断
    void handleBusOffInterrupt();                          // 处理总线关闭中断

    // handle if any error occured, and do the needful such as,
    // droping the frame, and counting errors
    // 处理错误,如丢弃帧和计数错误
    void pollErrorFlagsFromISR(void);

    // CAN Peripheral register structure, pointing at base
    // register. Indexed by locical interface number
    // CAN外设寄存器结构数组,按逻辑接口号索引
    static constexpr CanType* const Can[HAL_NUM_CAN_IFACES] = { HAL_CAN_BASE_LIST };

protected:
    // 添加项到接收队列
    bool add_to_rx_queue(const CanRxItem &rx_item) override {
        return rx_queue_.push(rx_item);
    }

    // 获取接口号
    int8_t get_iface_num(void) const override {
        return self_index_;
    }
};


#endif //HAL_NUM_CAN_IFACES
