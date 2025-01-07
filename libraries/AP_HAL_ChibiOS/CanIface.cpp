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

// 包含ChibiOS HAL头文件
#include <hal.h>
#include "AP_HAL_ChibiOS.h"

// 如果定义了CAN接口数量
#if HAL_NUM_CAN_IFACES
// 包含必要的头文件
#include <cassert>
#include <cstring>
#include <AP_Math/AP_Math.h>
# include <hal.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/ExpandingString.h>

// 如果不是STM32H7或STM32G4系列
# if !defined(STM32H7XX) && !defined(STM32G4)
#include "CANIface.h"

/* STM32F3's only CAN inteface does not have a number. */
// 针对STM32F3系列的特殊定义,因为它只有一个CAN接口且没有编号
#if defined(STM32F3XX)
#define RCC_APB1ENR_CAN1EN     RCC_APB1ENR_CANEN
#define RCC_APB1RSTR_CAN1RST   RCC_APB1RSTR_CANRST
#define CAN1_TX_IRQn           CAN_TX_IRQn
#define CAN1_RX0_IRQn          CAN_RX0_IRQn
#define CAN1_RX1_IRQn          CAN_RX1_IRQn
#define CAN1_TX_IRQ_Handler      STM32_CAN1_TX_HANDLER
#define CAN1_RX0_IRQ_Handler     STM32_CAN1_RX0_HANDLER
#define CAN1_RX1_IRQ_Handler     STM32_CAN1_RX1_HANDLER
#else
// 其他STM32系列的CAN中断处理函数定义
#define CAN1_TX_IRQ_Handler      STM32_CAN1_TX_HANDLER
#define CAN1_RX0_IRQ_Handler     STM32_CAN1_RX0_HANDLER
#define CAN1_RX1_IRQ_Handler     STM32_CAN1_RX1_HANDLER
#define CAN2_TX_IRQ_Handler      STM32_CAN2_TX_HANDLER
#define CAN2_RX0_IRQ_Handler     STM32_CAN2_RX0_HANDLER
#define CAN2_RX1_IRQ_Handler     STM32_CAN2_RX1_HANDLER
#endif // #if defined(STM32F3XX)

// 如果启用了CAN管理器,定义调试输出宏
#if HAL_CANMANAGER_ENABLED
#define Debug(fmt, args...) do { AP::can().log_text(AP_CANManager::LOG_DEBUG, "CANIface", fmt, ##args); } while (0)
#else
#define Debug(fmt, args...)
#endif

// 如果不是引导加载程序构建,定义性能统计宏
#if !defined(HAL_BOOTLOADER_BUILD)
#define PERF_STATS(x) (x++)
#else
#define PERF_STATS(x)
#endif

// 获取HAL实例的外部引用
extern const AP_HAL::HAL& hal;

using namespace ChibiOS;

// 定义CAN外设寄存器指针数组
constexpr bxcan::CanType* const CANIface::Can[];
// 定义CAN接口对象数组
static ChibiOS::CANIface* can_ifaces[HAL_NUM_CAN_IFACES];

// 下一个要初始化的接口索引
uint8_t CANIface::next_interface;

// 从逻辑接口到物理接口的映射,第一个物理接口是0,第一个逻辑接口是0
static constexpr uint8_t can_interfaces[HAL_NUM_CAN_IFACES] = { HAL_CAN_INTERFACE_LIST };

// 从物理接口到逻辑接口的映射,第一个物理接口是0,第一个逻辑接口是0
static constexpr int8_t can_iface_to_idx[3] = { HAL_CAN_INTERFACE_REV_LIST };

// 处理发送中断的内联函数
static inline void handleTxInterrupt(uint8_t phys_index)
{
    // 获取逻辑接口索引
    const int8_t iface_index = can_iface_to_idx[phys_index];
    // 检查索引是否有效
    if (iface_index < 0 || iface_index >= HAL_NUM_CAN_IFACES) {
        return;
    }
    // 获取精确时间戳
    uint64_t precise_time = AP_HAL::micros64();
    if (precise_time > 0) {
        precise_time--;
    }
    // 调用对应接口的发送中断处理函数
    if (can_ifaces[iface_index] != nullptr) {
        can_ifaces[iface_index]->handleTxInterrupt(precise_time);
    }
}

// 处理接收中断的内联函数
static inline void handleRxInterrupt(uint8_t phys_index, uint8_t fifo_index)
{
    // 获取逻辑接口索引
    const int8_t iface_index = can_iface_to_idx[phys_index];
    // 检查索引是否有效
    if (iface_index < 0 || iface_index >= HAL_NUM_CAN_IFACES) {
        return;
    }
    // 获取精确时间戳
    uint64_t precise_time = AP_HAL::micros64();
    if (precise_time > 0) {
        precise_time--;
    }
    // 调用对应接口的接收中断处理函数
    if (can_ifaces[iface_index] != nullptr) {
        can_ifaces[iface_index]->handleRxInterrupt(fifo_index, precise_time);
    }
}

/*
 * CANIface
 */
// 定义发送邮箱中止请求位的数组
const uint32_t CANIface::TSR_ABRQx[CANIface::NumTxMailboxes] = {
    bxcan::TSR_ABRQ0,
    bxcan::TSR_ABRQ1,
    bxcan::TSR_ABRQ2
};

// 带索引参数的构造函数
CANIface::CANIface(uint8_t index) :
    rx_bytebuffer_((uint8_t*)rx_buffer, sizeof(rx_buffer)),
    rx_queue_(&rx_bytebuffer_),
    self_index_(index)
{
    // 检查索引是否有效
    if (index >= HAL_NUM_CAN_IFACES) {
        AP_HAL::panic("Bad CANIface index.");
    } else {
        can_ = Can[index];
    }
}

// 适用于数组的默认构造函数
CANIface::CANIface() :
    CANIface(next_interface++)
{}

// 计算CAN总线时序参数
bool CANIface::computeTimings(uint32_t target_bitrate, Timings& out_timings)
{
    // 检查目标比特率是否有效
    if (target_bitrate < 1) {
        return false;
    }

    /*
     * Hardware configuration
     */
    // 获取APB1时钟频率
    const uint32_t pclk = STM32_PCLK1;

    // 定义最大BS1和BS2值
    static const int MaxBS1 = 16;
    static const int MaxBS2 = 8;

    /*
     * Ref. "Automatic Baudrate Detection in CANopen Networks", U. Koppe, MicroControl GmbH & Co. KG
     *      CAN in Automation, 2003
     *
     * According to the source, optimal quanta per bit are:
     *   Bitrate        Optimal Maximum
     *   1000 kbps      8       10
     *   500  kbps      16      17
     *   250  kbps      16      17
     *   125  kbps      16      17
     */
    // 根据比特率确定每位最大量子数
    const int max_quanta_per_bit = (target_bitrate >= 1000000) ? 10 : 17;

    // 定义最大采样点位置(千分比)
    static const int MaxSamplePointLocation = 900;

    /*
     * Computing (prescaler * BS):
     *   BITRATE = 1 / (PRESCALER * (1 / PCLK) * (1 + BS1 + BS2))       -- See the Reference Manual
     *   BITRATE = PCLK / (PRESCALER * (1 + BS1 + BS2))                 -- Simplified
     * let:
     *   BS = 1 + BS1 + BS2                                             -- Number of time quanta per bit
     *   PRESCALER_BS = PRESCALER * BS
     * ==>
     *   PRESCALER_BS = PCLK / BITRATE
     */
    // 计算预分频器和BS的乘积
    const uint32_t prescaler_bs = pclk / target_bitrate;

    /*
     * Searching for such prescaler value so that the number of quanta per bit is highest.
     */
    // 寻找使每位量子数最大的预分频器值
    uint8_t bs1_bs2_sum = uint8_t(max_quanta_per_bit - 1);

    // 调整BS1和BS2的和,直到找到合适的值
    while ((prescaler_bs % (1 + bs1_bs2_sum)) != 0) {
        if (bs1_bs2_sum <= 2) {
            return false;          // No solution
        }
        bs1_bs2_sum--;
    }

    // 计算预分频器值
    const uint32_t prescaler = prescaler_bs / (1 + bs1_bs2_sum);
    if ((prescaler < 1U) || (prescaler > 1024U)) {
        return false;              // No solution
    }

    /*
     * Now we have a constraint: (BS1 + BS2) == bs1_bs2_sum.
     * We need to find the values so that the sample point is as close as possible to the optimal value.
     *
     *   Solve[(1 + bs1)/(1 + bs1 + bs2) == 7/8, bs2]  (* Where 7/8 is 0.875, the recommended sample point location *)
     *   {{bs2 -> (1 + bs1)/7}}
     *
     * Hence:
     *   bs2 = (1 + bs1) / 7
     *   bs1 = (7 * bs1_bs2_sum - 1) / 8
     *
     * Sample point location can be computed as follows:
     *   Sample point location = (1 + bs1) / (1 + bs1 + bs2)
     *
     * Since the optimal solution is so close to the maximum, we prepare two solutions, and then pick the best one:
     *   - With rounding to nearest
     *   - With rounding to zero
     */
    // 定义BS参数对结构体
    struct BsPair {
        uint8_t bs1;
        uint8_t bs2;
        uint16_t sample_point_permill;

        BsPair() :
            bs1(0),
            bs2(0),
            sample_point_permill(0)
        { }

        BsPair(uint8_t bs1_bs2_sum, uint8_t arg_bs1) :
            bs1(arg_bs1),
            bs2(uint8_t(bs1_bs2_sum - bs1)),
            sample_point_permill(uint16_t(1000 * (1 + bs1) / (1 + bs1 + bs2)))
        {}

        // 检查BS参数是否有效
        bool isValid() const
        {
            return (bs1 >= 1) && (bs1 <= MaxBS1) && (bs2 >= 1) && (bs2 <= MaxBS2);
        }
    };

    // 第一次尝试,使用四舍五入
    BsPair solution(bs1_bs2_sum, uint8_t(((7 * bs1_bs2_sum - 1) + 4) / 8));

    // 如果采样点位置超过最大值,尝试向下取整
    if (solution.sample_point_permill > MaxSamplePointLocation) {
        solution = BsPair(bs1_bs2_sum, uint8_t((7 * bs1_bs2_sum - 1) / 8));
    }

    /*
     * Final validation
     * Helpful Python:
     * def sample_point_from_btr(x):
     *     assert 0b0011110010000000111111000000000 & x == 0
     *     ts2,ts1,brp = (x>>20)&7, (x>>16)&15, x&511
     *     return (1+ts1+1)/(1+ts1+1+ts2+1)
     *
     */
    // 最终验证计算结果
    if ((target_bitrate != (pclk / (prescaler * (1 + solution.bs1 + solution.bs2)))) || !solution.isValid()) {
        return false;
    }

    // 输出调试信息
    Debug("Timings: quanta/bit: %d, sample point location: %.1f%%",
          int(1 + solution.bs1 + solution.bs2), float(solution.sample_point_permill) / 10.F);

    // 设置输出参数
    out_timings.prescaler = uint16_t(prescaler - 1U);
    out_timings.sjw = 0;                                        // Which means one
    out_timings.bs1 = uint8_t(solution.bs1 - 1);
    out_timings.bs2 = uint8_t(solution.bs2 - 1);
    return true;
}

// 发送CAN帧
// Send CAN frame
int16_t CANIface::send(const AP_HAL::CANFrame& frame, uint64_t tx_deadline,
                       CanIOFlags flags)
{
    // 检查是否为错误帧或数据长度超过8字节
    // Check if error frame or data length > 8 bytes
    if (frame.isErrorFrame() || frame.dlc > 8) {
        return -1;
    }
    // 更新发送请求统计
    // Update transmit request statistics
    PERF_STATS(stats.tx_requests);

    /*
     * Normally we should perform the same check as in @ref canAcceptNewTxFrame(), because
     * it is possible that the highest-priority frame between select() and send() could have been
     * replaced with a lower priority one due to TX timeout. But we don't do this check because:
     *
     *  - It is a highly unlikely scenario.
     *
     *  - Frames do not timeout on a properly functioning bus. Since frames do not timeout, the new
     *    frame can only have higher priority, which doesn't break the logic.
     *
     *  - If high-priority frames are timing out in the TX queue, there's probably a lot of other
     *    issues to take care of before this one becomes relevant.
     *
     *  - It takes CPU time. Not just CPU time, but critical section time, which is expensive.
     */

    {
        // 进入临界区
        // Enter critical section
        CriticalSectionLocker lock;

        /*
         * Seeking for an empty slot
         */
        // 寻找空闲的发送邮箱
        // Look for an empty transmit mailbox
        uint8_t txmailbox = 0xFF;
        if ((can_->TSR & bxcan::TSR_TME0) == bxcan::TSR_TME0) {
            txmailbox = 0;
        } else if ((can_->TSR & bxcan::TSR_TME1) == bxcan::TSR_TME1) {
            txmailbox = 1;
        } else if ((can_->TSR & bxcan::TSR_TME2) == bxcan::TSR_TME2) {
            txmailbox = 2;
        } else {
            // 所有邮箱都已满,更新溢出统计
            // All mailboxes full, update overflow statistics
            PERF_STATS(stats.tx_overflow);
#if !defined(HAL_BOOTLOADER_BUILD)
            if (stats.tx_success == 0) {
                /*
                  if we have never successfully transmitted a frame
                  then we may be operating with just MAVCAN or UDP
                  MCAST. Consider the frame sent if the send
                  succeeds. This allows for UDP MCAST and MAVCAN to
                  operate fully when the CAN bus has no cable plugged
                  in
                 */
                return AP_HAL::CANIface::send(frame, tx_deadline, flags);
            }
#endif
            return 0;       // No transmission for you.
        }

        /*
         * Setting up the mailbox
         */
        // 配置发送邮箱
        // Configure transmit mailbox
        bxcan::TxMailboxType& mb = can_->TxMailbox[txmailbox];
        if (frame.isExtended()) {
            // 扩展帧ID设置
            // Set extended frame ID
            mb.TIR = ((frame.id & AP_HAL::CANFrame::MaskExtID) << 3) | bxcan::TIR_IDE;
        } else {
            // 标准帧ID设置
            // Set standard frame ID
            mb.TIR = ((frame.id & AP_HAL::CANFrame::MaskStdID) << 21);
        }

        // 设置远程帧标志
        // Set remote frame flag
        if (frame.isRemoteTransmissionRequest()) {
            mb.TIR |= bxcan::TIR_RTR;
        }

        // 设置数据长度
        // Set data length
        mb.TDTR = frame.dlc;

        // 设置数据
        // Set frame data
        mb.TDHR = frame.data_32[1];
        mb.TDLR = frame.data_32[0];

        // 启动发送
        // Start transmission
        mb.TIR |= bxcan::TIR_TXRQ;  // Go.

        /*
         * Registering the pending transmission so we can track its deadline and loopback it as needed
         */
        // 注册待发送项以跟踪其截止时间和回环需求
        // Register pending transmission to track deadline and loopback
        CanTxItem& txi = pending_tx_[txmailbox];
        txi.deadline       = tx_deadline;
        txi.frame          = frame;
        txi.loopback       = (flags & Loopback) != 0;
        txi.abort_on_error = (flags & AbortOnError) != 0;
        // 设置帧初始状态
        // Setup frame initial state
        txi.pushed         = false;
    }

    // 同时在MAVCAN上发送,发送失败不视为错误
    // Also send on MAVCAN, don't consider it an error if we can't send
    AP_HAL::CANIface::send(frame, tx_deadline, flags);

    return 1;
}

// 接收CAN帧
// Receive CAN frame
int16_t CANIface::receive(AP_HAL::CANFrame& out_frame, uint64_t& out_timestamp_us, CanIOFlags& out_flags)
{
    {
        // 进入临界区
        // Enter critical section
        CriticalSectionLocker lock;
        CanRxItem rx_item;
        // 从接收队列中获取数据
        // Get data from receive queue
        if (!rx_queue_.pop(rx_item)) {
            return 0;
        }
        out_frame    = rx_item.frame;
        out_timestamp_us = rx_item.timestamp_us;
        out_flags    = rx_item.flags;
    }

    return AP_HAL::CANIface::receive(out_frame, out_timestamp_us, out_flags);
}

#if !defined(HAL_BOOTLOADER_BUILD)
// 配置CAN过滤器
// Configure CAN filters
bool CANIface::configureFilters(const CanFilterConfig* filter_configs,
                                uint16_t num_configs)
{
#if !defined(HAL_BUILD_AP_PERIPH)
    // 仅在AP_Periph中进行过滤
    // only do filtering for AP_Periph
    can_->FMR &= ~bxcan::FMR_FINIT;
    return true;
#else
    if (mode_ != FilteredMode) {
        return false;
    }
    if (num_configs <= NumFilters && filter_configs != nullptr) {
        // 进入临界区
        // Enter critical section
        CriticalSectionLocker lock;

        // 进入过滤器初始化模式
        // Enter filter initialization mode
        can_->FMR |= bxcan::FMR_FINIT;

        // 从机(CAN2)获得一半过滤器
        // Slave (CAN2) gets half of the filters
        can_->FMR &= ~0x00003F00UL;
        can_->FMR |= static_cast<uint32_t>(NumFilters) << 8;

        // 配置过滤器基本参数
        // Configure basic filter parameters
        can_->FFA1R = 0x0AAAAAAA; // FIFO's are interleaved between filters
        can_->FM1R = 0; // Identifier Mask mode
        can_->FS1R = 0x7ffffff; // Single 32-bit for all

        // 计算过滤器起始索引
        // Calculate filter start index
        const uint8_t filter_start_index = (self_index_ == 0) ? 0 : NumFilters;

        if (num_configs == 0) {
            // 如果没有配置,则禁用所有过滤器
            // If no configs, disable all filters
            can_->FilterRegister[filter_start_index].FR1 = 0;
            can_->FilterRegister[filter_start_index].FR2 = 0;
            can_->FA1R = 1 << filter_start_index;
        } else {
            // 配置每个过滤器
            // Configure each filter
            for (uint8_t i = 0; i < NumFilters; i++) {
                if (i < num_configs) {
                    uint32_t id   = 0;
                    uint32_t mask = 0;

                    const CanFilterConfig* const cfg = filter_configs + i;

                    // 配置扩展帧或标准帧ID和掩码
                    // Configure extended or standard frame ID and mask
                    if ((cfg->id & AP_HAL::CANFrame::FlagEFF) || !(cfg->mask & AP_HAL::CANFrame::FlagEFF)) {
                        id   = (cfg->id   & AP_HAL::CANFrame::MaskExtID) << 3;
                        mask = (cfg->mask & AP_HAL::CANFrame::MaskExtID) << 3;
                        id |= bxcan::RIR_IDE;
                    } else {
                        id   = (cfg->id   & AP_HAL::CANFrame::MaskStdID) << 21;  // Regular std frames, nothing fancy.
                        mask = (cfg->mask & AP_HAL::CANFrame::MaskStdID) << 21;  // Boring.
                    }

                    // 配置远程帧标志
                    // Configure remote frame flag
                    if (cfg->id & AP_HAL::CANFrame::FlagRTR) {
                        id |= bxcan::RIR_RTR;
                    }

                    // 配置扩展帧和远程帧掩码
                    // Configure extended frame and remote frame masks
                    if (cfg->mask & AP_HAL::CANFrame::FlagEFF) {
                        mask |= bxcan::RIR_IDE;
                    }

                    if (cfg->mask & AP_HAL::CANFrame::FlagRTR) {
                        mask |= bxcan::RIR_RTR;
                    }

                    // 设置过滤器寄存器
                    // Set filter registers
                    can_->FilterRegister[filter_start_index + i].FR1 = id;
                    can_->FilterRegister[filter_start_index + i].FR2 = mask;

                    // 激活过滤器
                    // Activate filter
                    can_->FA1R |= (1 << (filter_start_index + i));
                } else {
                    // 禁用未使用的过滤器
                    // Disable unused filters
                    can_->FA1R &= ~(1 << (filter_start_index + i));
                }
            }
        }

        // 退出过滤器初始化模式
        // Exit filter initialization mode
        can_->FMR &= ~bxcan::FMR_FINIT;

        return true;
    }

    return false;
#endif // AP_Periph
}
#endif

// 等待MSR INAK位状态改变
// Wait for MSR INAK bit state change
bool CANIface::waitMsrINakBitStateChange(bool target_state)
{
    const unsigned Timeout = 1000;
    for (unsigned wait_ack = 0; wait_ack < Timeout; wait_ack++) {
        const bool state = (can_->MSR & bxcan::MSR_INAK) != 0;
        if (state == target_state) {
            return true;
        }
        chThdSleep(chTimeMS2I(1));
    }
    return false;
}

void CANIface::handleTxMailboxInterrupt(uint8_t mailbox_index, bool txok, const uint64_t timestamp_us)
{
    // 检查邮箱索引是否有效
    if (mailbox_index > NumTxMailboxes) {
        return;
    }

    // 更新活动状态标志
    had_activity_ = had_activity_ || txok;

    // 获取对应邮箱的发送项
    CanTxItem& txi = pending_tx_[mailbox_index];

    // 如果需要回环且发送成功且未推送,则添加到接收队列
    if (txi.loopback && txok && !txi.pushed) {
        CanRxItem rx_item;
        rx_item.frame = txi.frame;
        rx_item.timestamp_us = timestamp_us;
        rx_item.flags = AP_HAL::CANIface::Loopback;
        add_to_rx_queue(rx_item);
    }

    // 如果发送成功且未推送,更新状态
    if (txok && !txi.pushed) {
        txi.pushed = true;
        PERF_STATS(stats.tx_success);
#if !defined(HAL_BOOTLOADER_BUILD)
        stats.last_transmit_us = timestamp_us;
#endif
    }
}

void CANIface::handleTxInterrupt(const uint64_t utc_usec)
{
    // 检查并处理邮箱0的发送完成中断
    if (can_->TSR & bxcan::TSR_RQCP0) {
        const bool txok = can_->TSR & bxcan::TSR_TXOK0;
        can_->TSR = bxcan::TSR_RQCP0;
        handleTxMailboxInterrupt(0, txok, utc_usec);
    }
    // 检查并处理邮箱1的发送完成中断
    if (can_->TSR & bxcan::TSR_RQCP1) {
        const bool txok = can_->TSR & bxcan::TSR_TXOK1;
        can_->TSR = bxcan::TSR_RQCP1;
        handleTxMailboxInterrupt(1, txok, utc_usec);
    }
    // 检查并处理邮箱2的发送完成中断
    if (can_->TSR & bxcan::TSR_RQCP2) {
        const bool txok = can_->TSR & bxcan::TSR_TXOK2;
        can_->TSR = bxcan::TSR_RQCP2;
        handleTxMailboxInterrupt(2, txok, utc_usec);
    }

    // 如果有信号量句柄,发送信号
    if (sem_handle != nullptr) {
        PERF_STATS(stats.num_events);
        sem_handle->signal_ISR();
    }

    // 检查错误标志
    pollErrorFlagsFromISR();
}

void CANIface::handleRxInterrupt(uint8_t fifo_index, uint64_t timestamp_us)
{
    // 获取对应FIFO的寄存器指针
    volatile uint32_t* const rfr_reg = (fifo_index == 0) ? &can_->RF0R : &can_->RF1R;
    if ((*rfr_reg & bxcan::RFR_FMP_MASK) == 0) {
        return;
    }

    /*
     * Register overflow as a hardware error
     */
    // 检查FIFO溢出错误
    if ((*rfr_reg & bxcan::RFR_FOVR) != 0) {
        PERF_STATS(stats.rx_errors);
    }

    /*
     * Read the frame contents
     */
    // 读取帧内容
    AP_HAL::CANFrame &frame = isr_rx_frame;
    const bxcan::RxMailboxType& rf = can_->RxMailbox[fifo_index];

    // 解析标准帧或扩展帧ID
    if ((rf.RIR & bxcan::RIR_IDE) == 0) {
        frame.id = AP_HAL::CANFrame::MaskStdID & (rf.RIR >> 21);
    } else {
        frame.id = AP_HAL::CANFrame::MaskExtID & (rf.RIR >> 3);
        frame.id |= AP_HAL::CANFrame::FlagEFF;
    }

    // 检查是否为远程帧
    if ((rf.RIR & bxcan::RIR_RTR) != 0) {
        frame.id |= AP_HAL::CANFrame::FlagRTR;
    }

    // 获取数据长度
    frame.dlc = rf.RDTR & 15;

    // 读取数据字节
    frame.data[0] = uint8_t(0xFF & (rf.RDLR >> 0));
    frame.data[1] = uint8_t(0xFF & (rf.RDLR >> 8));
    frame.data[2] = uint8_t(0xFF & (rf.RDLR >> 16));
    frame.data[3] = uint8_t(0xFF & (rf.RDLR >> 24));
    frame.data[4] = uint8_t(0xFF & (rf.RDHR >> 0));
    frame.data[5] = uint8_t(0xFF & (rf.RDHR >> 8));
    frame.data[6] = uint8_t(0xFF & (rf.RDHR >> 16));
    frame.data[7] = uint8_t(0xFF & (rf.RDHR >> 24));

    // 释放FIFO
    *rfr_reg = bxcan::RFR_RFOM | bxcan::RFR_FOVR | bxcan::RFR_FULL;  // Release FIFO entry we just read

    /*
     * Store with timeout into the FIFO buffer and signal update event
     */
    // 将接收到的帧存入队列
    CanRxItem &rx_item = isr_rx_item;
    rx_item.frame = frame;
    rx_item.timestamp_us = timestamp_us;
    rx_item.flags = 0;
    if (add_to_rx_queue(rx_item)) {
        PERF_STATS(stats.rx_received);
    } else {
        PERF_STATS(stats.rx_overflow);
    }

    // 更新活动状态
    had_activity_ = true;

    // 如果有信号量句柄,发送信号
    if (sem_handle != nullptr) {
        PERF_STATS(stats.num_events);
        sem_handle->signal_ISR();
    }

    // 检查错误标志
    pollErrorFlagsFromISR();
}

void CANIface::pollErrorFlagsFromISR()
{
    // 获取最后错误代码
    const uint8_t lec = uint8_t((can_->ESR & bxcan::ESR_LEC_MASK) >> bxcan::ESR_LEC_SHIFT);
    if (lec != 0) {
#if !defined(HAL_BUILD_AP_PERIPH) && !defined(HAL_BOOTLOADER_BUILD)
        stats.esr = can_->ESR; // Record error status
#endif
        can_->ESR = 0;

        // 处理中止请求
        for (int i = 0; i < NumTxMailboxes; i++) {
            CanTxItem& txi = pending_tx_[i];
            if (txi.aborted && txi.abort_on_error) {
                can_->TSR = TSR_ABRQx[i];
                txi.aborted = true;
                PERF_STATS(stats.tx_abort);
            }
        }
    }
}

void CANIface::discardTimedOutTxMailboxes(uint64_t current_time)
{
    // 检查并丢弃超时的发送邮箱
    CriticalSectionLocker lock;
    for (int i = 0; i < NumTxMailboxes; i++) {
        CanTxItem& txi = pending_tx_[i];
        if (txi.aborted || !txi.setup) {
            continue;
        }
        if (txi.deadline < current_time) {
            can_->TSR = TSR_ABRQx[i];  // Goodnight sweet transmission
            pending_tx_[i].aborted = true;
            PERF_STATS(stats.tx_timedout);
        }
    }
}

void CANIface::clear_rx()
{
    // 清空接收队列
    CriticalSectionLocker lock;
    rx_queue_.clear();
}

void CANIface::pollErrorFlags()
{
    // 检查错误标志
    CriticalSectionLocker cs_locker;
    pollErrorFlagsFromISR();
}

bool CANIface::canAcceptNewTxFrame(const AP_HAL::CANFrame& frame) const
{
    /*
     * We can accept more frames only if the following conditions are satisfied:
     *  - There is at least one TX mailbox free (obvious enough);
     *  - The priority of the new frame is higher than priority of all TX mailboxes.
     */
    {
        // 检查是否有空闲的发送邮箱
        static const uint32_t TME = bxcan::TSR_TME0 | bxcan::TSR_TME1 | bxcan::TSR_TME2;
        const uint32_t tme = can_->TSR & TME;

        if (tme == TME) {   // All TX mailboxes are free (as in freedom).
            return true;
        }

        if (tme == 0) {     // All TX mailboxes are busy transmitting.
            return false;
        }
    }

    /*
     * The second condition requires a critical section.
     */
    // 检查新帧优先级
    CriticalSectionLocker lock;

    for (int mbx = 0; mbx < NumTxMailboxes; mbx++) {
        if (!(pending_tx_[mbx].pushed || pending_tx_[mbx].aborted) && !frame.priorityHigherThan(pending_tx_[mbx].frame)) {
            return false;       // There's a mailbox whose priority is higher or equal the priority of the new frame.
        }
    }

    return true;                // This new frame will be added to a free TX mailbox in the next @ref send().
}

bool CANIface::isRxBufferEmpty() const
{
    // 检查接收缓冲区是否为空
    CriticalSectionLocker lock;
    return rx_queue_.available() == 0;
}

#if !defined(HAL_BUILD_AP_PERIPH) && !defined(HAL_BOOTLOADER_BUILD)
uint32_t CANIface::getErrorCount() const
{
    // 获取错误计数
    CriticalSectionLocker lock;
    return stats.num_busoff_err +
           stats.rx_errors +
           stats.rx_overflow +
           stats.tx_rejected +
           stats.tx_abort +
           stats.tx_timedout;
}

#endif // #if !defined(HAL_BUILD_AP_PERIPH) && !defined(HAL_BOOTLOADER_BUILD)

bool CANIface::set_event_handle(AP_HAL::BinarySemaphore *handle)
{
    // 设置事件句柄
    sem_handle = handle;
    return true;
}


void CANIface::checkAvailable(bool& read, bool& write, const AP_HAL::CANFrame* pending_tx) const
{
    // 检查读写可用性
    write = false;
    read = !isRxBufferEmpty();

    if (pending_tx != nullptr) {
        write = canAcceptNewTxFrame(*pending_tx);
    }
}

bool CANIface::select(bool &read, bool &write,
                      const AP_HAL::CANFrame* pending_tx,
                      uint64_t blocking_deadline)
{
    // 选择操作
    const bool in_read = read;
    const bool in_write= write;
    uint64_t time = AP_HAL::micros64();

    if (!read && !write) {
        //invalid request
        return false;
    }

    // 丢弃超时的发送邮箱并检查错误标志
    discardTimedOutTxMailboxes(time);              // Check TX timeouts - this may release some TX slots
    pollErrorFlags();

    // 检查是否已有请求的事件
    checkAvailable(read, write, pending_tx);          // Check if we already have some of the requested events
    if ((read && in_read) || (write && in_write)) {
        return true;
    }

#if !defined(HAL_BUILD_AP_PERIPH) && !defined(HAL_BOOTLOADER_BUILD)
    // we don't support blocking select in AP_Periph and bootloader
    // 在AP_Periph和bootloader中不支持阻塞式select
    while (time < blocking_deadline) {
        // 如果没有设置信号量句柄,退出循环
        if (sem_handle == nullptr) {
            break;
        }
        // 等待信号量,直到超时或任何接口更新
        IGNORE_RETURN(sem_handle->wait(blocking_deadline - time)); // Block until timeout expires or any iface updates
        // 检查是否有可用的读写事件
        checkAvailable(read, write, pending_tx);  // Check what we got
        // 如果有请求的读写事件,返回true
        if ((read && in_read) || (write && in_write)) {
            return true;
        }
        // 更新当前时间
        time = AP_HAL::micros64();
    }
#endif // #if !defined(HAL_BUILD_AP_PERIPH) && !defined(HAL_BOOTLOADER_BUILD)
    return true;
}

void CANIface::initOnce(bool enable_irq)
{
    /*
     * CAN1, CAN2
     */
    {
        // 进入临界区
        CriticalSectionLocker lock;
        // 根据CAN接口编号进行初始化
        switch (can_interfaces[self_index_]) {
        case 0:
#if defined(RCC_APB1ENR1_CAN1EN)
            // 使能CAN1时钟
            RCC->APB1ENR1 |=  RCC_APB1ENR1_CAN1EN;
            // 复位CAN1
            RCC->APB1RSTR1 |=  RCC_APB1RSTR1_CAN1RST;
            RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_CAN1RST;
#else
            // 使能CAN1时钟
            RCC->APB1ENR  |=  RCC_APB1ENR_CAN1EN;
            // 复位CAN1
            RCC->APB1RSTR |=  RCC_APB1RSTR_CAN1RST;
            RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN1RST;
#endif
            break;
#if defined(RCC_APB1ENR1_CAN2EN)
        case 1:
            // 使能CAN2时钟
            RCC->APB1ENR1  |=  RCC_APB1ENR1_CAN2EN;
            // 复位CAN2
            RCC->APB1RSTR1 |=  RCC_APB1RSTR1_CAN2RST;
            RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_CAN2RST;
            break;
#elif defined(RCC_APB1ENR_CAN2EN)
        case 1:
            // 使能CAN2时钟
            RCC->APB1ENR  |=  RCC_APB1ENR_CAN2EN;
            // 复位CAN2
            RCC->APB1RSTR |=  RCC_APB1RSTR_CAN2RST;
            RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN2RST;
            break;
#endif
#ifdef RCC_APB1ENR_CAN3EN
        case 2:
            // 使能CAN3时钟
            RCC->APB1ENR  |=  RCC_APB1ENR_CAN3EN;
            // 复位CAN3
            RCC->APB1RSTR |=  RCC_APB1RSTR_CAN3RST;
            RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN3RST;
            break;
#endif
        }
    }

    /*
     * IRQ
     */
    // 如果中断未初始化且需要使能中断
    if (!irq_init_ && enable_irq) {
        CriticalSectionLocker lock;
        // 根据CAN接口编号配置中断
        switch (can_interfaces[self_index_]) {
        case 0:
#ifdef HAL_CAN_IFACE1_ENABLE
            // 使能CAN1的发送和接收中断
            nvicEnableVector(CAN1_TX_IRQn,  CORTEX_MAX_KERNEL_PRIORITY);
            nvicEnableVector(CAN1_RX0_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
            nvicEnableVector(CAN1_RX1_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
#endif
            break;
        case 1:
#ifdef HAL_CAN_IFACE2_ENABLE
            // 使能CAN2的发送和接收中断
            nvicEnableVector(CAN2_TX_IRQn,  CORTEX_MAX_KERNEL_PRIORITY);
            nvicEnableVector(CAN2_RX0_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
            nvicEnableVector(CAN2_RX1_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
#endif
            break;
        case 2:
#ifdef HAL_CAN_IFACE3_ENABLE
            // 使能CAN3的发送和接收中断
            nvicEnableVector(CAN3_TX_IRQn,  CORTEX_MAX_KERNEL_PRIORITY);
            nvicEnableVector(CAN3_RX0_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
            nvicEnableVector(CAN3_RX1_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
#endif
            break;
        }
        irq_init_ = true;
    }
}

bool CANIface::init(const uint32_t bitrate, const CANIface::OperatingMode mode)
{
    // 打印调试信息
    Debug("Bitrate %lu mode %d", static_cast<unsigned long>(bitrate), static_cast<int>(mode));
    // 检查接口索引是否有效
    if (self_index_ > HAL_NUM_CAN_IFACES) {
        Debug("CAN drv init failed");
        return false;
    }
    // 如果接口未初始化,则进行初始化
    if (can_ifaces[self_index_] == nullptr) {
        can_ifaces[self_index_] = this;
#if !defined(HAL_BOOTLOADER_BUILD)
        AP_HAL::get_HAL_mutable().can[self_index_] = this;
#endif
    }

    // 保存波特率和工作模式
    bitrate_ = bitrate;
    mode_ = mode;

    // 确保CAN0接口已初始化
    if (can_ifaces[0] == nullptr) {
        can_ifaces[0] = NEW_NOTHROW CANIface(0);
        Debug("Failed to allocate CAN iface 0");
        if (can_ifaces[0] == nullptr) {
            return false;
        }
    }
    // 如果是CAN1接口且CAN0未初始化,则先初始化CAN0
    if (self_index_ == 1 && !can_ifaces[0]->is_initialized()) {
        Debug("Iface 0 is not initialized yet but we need it for Iface 1, trying to init it");
        Debug("Enabling CAN iface 0");
        can_ifaces[0]->initOnce(false);
        Debug("Initing iface 0...");
        if (!can_ifaces[0]->init(bitrate, mode)) {
            Debug("Iface 0 init failed");
            return false;
        }

        Debug("Enabling CAN iface");
    }
    // 初始化当前接口
    initOnce(true);
    /*
     * We need to silence the controller in the first order, otherwise it may interfere with the following operations.
     */
    {
        CriticalSectionLocker lock;

        // 退出睡眠模式并请求初始化
        can_->MCR &= ~bxcan::MCR_SLEEP; // Exit sleep mode
        can_->MCR |= bxcan::MCR_INRQ;   // Request init

        // 在初始化过程中禁用中断
        can_->IER = 0;                  // Disable interrupts while initialization is in progress
    }

    // 等待进入初始化模式
    if (!waitMsrINakBitStateChange(true)) {
        Debug("MSR INAK not set");
        can_->MCR = bxcan::MCR_RESET;
        return false;
    }

    /*
     * Object state - interrupts are disabled, so it's safe to modify it now
     */
    // 清空接收队列
    rx_queue_.clear();

    // 初始化发送邮箱
    for (uint32_t i=0; i < NumTxMailboxes; i++) {
        pending_tx_[i] = CanTxItem();
    }
    had_activity_ = false;

    /*
     * CAN timings for this bitrate
     */
    // 计算CAN总线时序参数
    Timings timings;
    if (!computeTimings(bitrate, timings)) {
        can_->MCR = bxcan::MCR_RESET;
        return false;
    }
    Debug("Timings: presc=%u sjw=%u bs1=%u bs2=%u",
          unsigned(timings.prescaler), unsigned(timings.sjw), unsigned(timings.bs1), unsigned(timings.bs2));

    /*
     * Hardware initialization (the hardware has already confirmed initialization mode, see above)
     */
    // 配置CAN控制器
    can_->MCR = bxcan::MCR_ABOM | bxcan::MCR_AWUM | bxcan::MCR_INRQ;  // RM page 648

    // 设置位时序参数
    can_->BTR = ((timings.sjw & 3U)  << 24) |
                ((timings.bs1 & 15U) << 16) |
                ((timings.bs2 & 7U)  << 20) |
                (timings.prescaler & 1023U) |
                ((mode == SilentMode) ? bxcan::BTR_SILM : 0);

    // 使能相关中断
    can_->IER = bxcan::IER_TMEIE |   // TX mailbox empty
                bxcan::IER_FMPIE0 |  // RX FIFO 0 is not empty
                bxcan::IER_FMPIE1;   // RX FIFO 1 is not empty

    // 退出初始化模式
    can_->MCR &= ~bxcan::MCR_INRQ;   // Leave init mode

    // 等待退出初始化模式
    if (!waitMsrINakBitStateChange(false)) {
        Debug("MSR INAK not cleared");
        can_->MCR = bxcan::MCR_RESET;
        return false;
    }

    /*
     * Default filter configuration
     */
    // 配置CAN过滤器(仅CAN1需要配置)
    if (self_index_ == 0) {
        // 进入过滤器初始化模式
        can_->FMR |= bxcan::FMR_FINIT;

        // 配置过滤器数量
        can_->FMR &= 0xFFFFC0F1;
        can_->FMR |= static_cast<uint32_t>(NumFilters) << 8;  // Slave (CAN2) gets half of the filters

        // 配置过滤器模式
        can_->FFA1R = 0;                           // All assigned to FIFO0 by default
        can_->FM1R = 0;                            // Indentifier Mask mode

#if HAL_NUM_CAN_IFACES > 1
        // 多接口配置
        can_->FS1R = 0x7ffffff;                    // Single 32-bit for all
        can_->FilterRegister[0].FR1 = 0;          // CAN1 accepts everything
        can_->FilterRegister[0].FR2 = 0;
        can_->FilterRegister[NumFilters].FR1 = 0; // CAN2 accepts everything
        can_->FilterRegister[NumFilters].FR2 = 0;
        can_->FA1R = 1 | (1 << NumFilters);        // One filter per each iface
#else
        // 单接口配置
        can_->FS1R = 0x1fff;
        can_->FilterRegister[0].FR1 = 0;
        can_->FilterRegister[0].FR2 = 0;
        can_->FA1R = 1;
#endif
        // 退出过滤器初始化模式
        // Exit filter initialization mode
        can_->FMR &= ~bxcan::FMR_FINIT;
    }
    // 标记CAN接口已初始化
    // Mark CAN interface as initialized
    initialised_ = true;

    return true;
}

#if !defined(HAL_BUILD_AP_PERIPH) && !defined(HAL_BOOTLOADER_BUILD)
// 获取CAN接口统计信息并格式化输出到字符串
// Get CAN interface statistics and format output to string
void CANIface::get_stats(ExpandingString &str)
{
    // 进入临界区保护
    // Enter critical section for protection
    CriticalSectionLocker lock;
    // 格式化输出各项统计数据
    // Format and output various statistics
    str.printf("tx_requests:    %lu\n"   // 发送请求次数
               "tx_rejected:    %lu\n"    // 发送被拒绝次数
               "tx_success:     %lu\n"    // 发送成功次数
               "tx_timedout:    %lu\n"    // 发送超时次数
               "tx_abort:       %lu\n"    // 发送中止次数
               "rx_received:    %lu\n"    // 接收成功次数
               "rx_overflow:    %lu\n"    // 接收溢出次数
               "rx_errors:      %lu\n"    // 接收错误次数
               "num_busoff_err: %lu\n"    // 总线关闭错误次数
               "num_events:     %lu\n"    // 事件总数
               "ESR:            %lx\n",   // 错误状态寄存器值
               stats.tx_requests,
               stats.tx_rejected,
               stats.tx_success,
               stats.tx_timedout,
               stats.tx_abort,
               stats.rx_received,
               stats.rx_overflow,
               stats.rx_errors,
               stats.num_busoff_err,
               stats.num_events,
               stats.esr);
}
#endif

/*
 * Interrupt handlers
 */
extern "C"
{
#ifdef HAL_CAN_IFACE1_ENABLE
    // CAN1中断处理函数
    // CAN1 interrupt handlers
    CH_IRQ_HANDLER(CAN1_TX_IRQ_Handler);
    CH_IRQ_HANDLER(CAN1_TX_IRQ_Handler)
    {
        // 中断处理开始
        // Start of interrupt handling
        CH_IRQ_PROLOGUE();
        // 处理CAN1发送中断
        // Handle CAN1 transmit interrupt
        handleTxInterrupt(0);
        // 中断处理结束
        // End of interrupt handling
        CH_IRQ_EPILOGUE();
    }

    CH_IRQ_HANDLER(CAN1_RX0_IRQ_Handler);
    CH_IRQ_HANDLER(CAN1_RX0_IRQ_Handler)
    {
        // 中断处理开始
        // Start of interrupt handling
        CH_IRQ_PROLOGUE();
        // 处理CAN1接收FIFO0中断
        // Handle CAN1 receive FIFO0 interrupt
        handleRxInterrupt(0, 0);
        // 中断处理结束
        // End of interrupt handling
        CH_IRQ_EPILOGUE();
    }

    CH_IRQ_HANDLER(CAN1_RX1_IRQ_Handler);
    CH_IRQ_HANDLER(CAN1_RX1_IRQ_Handler)
    {
        // 中断处理开始
        // Start of interrupt handling
        CH_IRQ_PROLOGUE();
        // 处理CAN1接收FIFO1中断
        // Handle CAN1 receive FIFO1 interrupt
        handleRxInterrupt(0, 1);
        // 中断处理结束
        // End of interrupt handling
        CH_IRQ_EPILOGUE();
    }
#endif

#ifdef HAL_CAN_IFACE2_ENABLE
    // CAN2中断处理函数
    // CAN2 interrupt handlers
    CH_IRQ_HANDLER(CAN2_TX_IRQ_Handler);
    CH_IRQ_HANDLER(CAN2_TX_IRQ_Handler)
    {
        // 中断处理开始
        // Start of interrupt handling
        CH_IRQ_PROLOGUE();
        // 处理CAN2发送中断
        // Handle CAN2 transmit interrupt
        handleTxInterrupt(1);
        // 中断处理结束
        // End of interrupt handling
        CH_IRQ_EPILOGUE();
    }

    CH_IRQ_HANDLER(CAN2_RX0_IRQ_Handler);
    CH_IRQ_HANDLER(CAN2_RX0_IRQ_Handler)
    {
        // 中断处理开始
        // Start of interrupt handling
        CH_IRQ_PROLOGUE();
        // 处理CAN2接收FIFO0中断
        // Handle CAN2 receive FIFO0 interrupt
        handleRxInterrupt(1, 0);
        // 中断处理结束
        // End of interrupt handling
        CH_IRQ_EPILOGUE();
    }

    CH_IRQ_HANDLER(CAN2_RX1_IRQ_Handler);
    CH_IRQ_HANDLER(CAN2_RX1_IRQ_Handler)
    {
        // 中断处理开始
        // Start of interrupt handling
        CH_IRQ_PROLOGUE();
        // 处理CAN2接收FIFO1中断
        // Handle CAN2 receive FIFO1 interrupt
        handleRxInterrupt(1, 1);
        // 中断处理结束
        // End of interrupt handling
        CH_IRQ_EPILOGUE();
    }
#endif

#ifdef HAL_CAN_IFACE3_ENABLE
    // CAN3中断处理函数
    // CAN3 interrupt handlers
    CH_IRQ_HANDLER(CAN3_TX_IRQ_Handler);
    CH_IRQ_HANDLER(CAN3_TX_IRQ_Handler)
    {
        // 中断处理开始
        // Start of interrupt handling
        CH_IRQ_PROLOGUE();
        // 处理CAN3发送中断
        // Handle CAN3 transmit interrupt
        handleTxInterrupt(2);
        // 中断处理结束
        // End of interrupt handling
        CH_IRQ_EPILOGUE();
    }

    CH_IRQ_HANDLER(CAN3_RX0_IRQ_Handler);
    CH_IRQ_HANDLER(CAN3_RX0_IRQ_Handler)
    {
        // 中断处理开始
        // Start of interrupt handling
        CH_IRQ_PROLOGUE();
        // 处理CAN3接收FIFO0中断
        // Handle CAN3 receive FIFO0 interrupt
        handleRxInterrupt(2, 0);
        // 中断处理结束
        // End of interrupt handling
        CH_IRQ_EPILOGUE();
    }

    CH_IRQ_HANDLER(CAN3_RX1_IRQ_Handler);
    CH_IRQ_HANDLER(CAN3_RX1_IRQ_Handler)
    {
        // 中断处理开始
        // Start of interrupt handling
        CH_IRQ_PROLOGUE();
        // 处理CAN3接收FIFO1中断
        // Handle CAN3 receive FIFO1 interrupt
        handleRxInterrupt(2, 1);
        // 中断处理结束
        // End of interrupt handling
        CH_IRQ_EPILOGUE();
    }
#endif
    
} // extern "C"

#endif //!defined(STM32H7XX)

#endif //HAL_NUM_CAN_IFACES
