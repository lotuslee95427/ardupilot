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

// 包含必要的头文件
#include <hal.h>
#include "AP_HAL_ChibiOS.h"

#if HAL_NUM_CAN_IFACES
#include <cassert>
#include <cstring>
#include <AP_Math/AP_Math.h>
# include <hal.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/ExpandingString.h>

// 仅在STM32H7或STM32G4平台上编译
# if defined(STM32H7XX) || defined(STM32G4)
#include "CANFDIface.h"

// 定义中断处理函数名称
#define FDCAN1_IT0_IRQHandler      STM32_FDCAN1_IT0_HANDLER
#define FDCAN1_IT1_IRQHandler      STM32_FDCAN1_IT1_HANDLER
#define FDCAN2_IT0_IRQHandler      STM32_FDCAN2_IT0_HANDLER
#define FDCAN2_IT1_IRQHandler      STM32_FDCAN2_IT1_HANDLER

// FIFO元素间隔为18个字
#define FDCAN_FRAME_BUFFER_SIZE 18


// 消息RAM分配(以字为单位)

#if defined(STM32H7)
// STM32H7平台的配置
#define MAX_FILTER_LIST_SIZE 78U            //78个标准过滤器列表元素或40个扩展过滤器列表
#define FDCAN_NUM_RXFIFO0_SIZE 108U         //6帧
#define FDCAN_TX_FIFO_BUFFER_SIZE 126U      //7帧
#define MESSAGE_RAM_END_ADDR 0x4000B5FC

#elif defined(STM32G4)
// STM32G4平台的配置
#define MAX_FILTER_LIST_SIZE 80U            //80个标准过滤器列表元素或40个扩展过滤器列表
#define FDCAN_NUM_RXFIFO0_SIZE 104U         //26帧
#define FDCAN_TX_FIFO_BUFFER_SIZE 128U      //32帧
#define FDCAN_MESSAGERAM_STRIDE 0x350       // 消息RAM区域间的间隔
#define FDCAN_EXFILTER_OFFSET 0x70
#define FDCAN_RXFIFO0_OFFSET 0xB0
#define FDCAN_RXFIFO1_OFFSET 0x188
#define FDCAN_TXFIFO_OFFSET 0x278

#define MESSAGE_RAM_END_ADDR 0x4000B5FC

#else
#error "Unsupported MCU for FDCAN"
#endif

extern const AP_HAL::HAL& hal;

#define STR(x) #x
#define XSTR(x) STR(x)
#if !defined(HAL_LLD_USE_CLOCK_MANAGEMENT)
// 确保FDCAN时钟频率为80MHz
static_assert(STM32_FDCANCLK == 80U*1000U*1000U, "FDCAN clock must be 80MHz, got " XSTR(STM32_FDCANCLK));
#endif

using namespace ChibiOS;

// 调试输出宏定义
#if HAL_CANMANAGER_ENABLED
#define Debug(fmt, args...) do { AP::can().log_text(AP_CANManager::LOG_DEBUG, "CANFDIface", fmt, ##args); } while (0)
#else
#define Debug(fmt, args...)
#endif

// CAN接口相关的静态变量定义
constexpr CANIface::CanType* const CANIface::Can[];
static ChibiOS::CANIface* can_ifaces[HAL_NUM_CAN_IFACES];

uint8_t CANIface::next_interface;

// 逻辑接口到物理接口的映射,第一个物理接口是0,第一个逻辑接口是0
static constexpr uint8_t can_interfaces[HAL_NUM_CAN_IFACES] = { HAL_CAN_INTERFACE_LIST };

// 物理接口到逻辑接口的映射,第一个物理接口是0,第一个逻辑接口是0
static constexpr int8_t can_iface_to_idx[3] = { HAL_CAN_INTERFACE_REV_LIST };

// 设置寄存器的超时时间为250ms
#define REG_SET_TIMEOUT 250 

// 检查驱动是否已初始化
static inline bool driver_initialised(uint8_t iface_index)
{
    if (can_ifaces[iface_index] == nullptr) {
        return false;
    }
    return true;
}

// CAN中断处理函数
static inline void handleCANInterrupt(uint8_t phys_index, uint8_t line_index)
{
    const int8_t iface_index = can_iface_to_idx[phys_index];
    if (iface_index < 0 || iface_index >= HAL_NUM_CAN_IFACES) {
        return;
    }
    if (!driver_initialised(iface_index)) {
        //如果驱动未初始化,重置所有中断并返回
        CANIface::Can[iface_index]->IR = FDCAN_IR_RF0N;
        CANIface::Can[iface_index]->IR = FDCAN_IR_RF1N;
        CANIface::Can[iface_index]->IR = FDCAN_IR_TEFN;
        return;
    }
    if (line_index == 0) {
        // 处理接收中断
        if ((CANIface::Can[iface_index]->IR & FDCAN_IR_RF0N) ||
            (CANIface::Can[iface_index]->IR & FDCAN_IR_RF0F)) {
            CANIface::Can[iface_index]->IR = FDCAN_IR_RF0N | FDCAN_IR_RF0F;
            can_ifaces[iface_index]->handleRxInterrupt(0);
        }
        if ((CANIface::Can[iface_index]->IR & FDCAN_IR_RF1N) ||
            (CANIface::Can[iface_index]->IR & FDCAN_IR_RF1F)) {
            CANIface::Can[iface_index]->IR = FDCAN_IR_RF1N | FDCAN_IR_RF1F;
            can_ifaces[iface_index]->handleRxInterrupt(1);
        }
    } else {
        // 处理发送完成中断
        if (CANIface::Can[iface_index]->IR & FDCAN_IR_TC) {
            CANIface::Can[iface_index]->IR = FDCAN_IR_TC;
            uint64_t timestamp_us = AP_HAL::micros64();
            if (timestamp_us > 0) {
                timestamp_us--;
            }
            can_ifaces[iface_index]->handleTxCompleteInterrupt(timestamp_us);
        }

        // 处理总线关闭中断
        if ((CANIface::Can[iface_index]->IR & FDCAN_IR_BO)) {
            CANIface::Can[iface_index]->IR = FDCAN_IR_BO;
            can_ifaces[iface_index]->handleBusOffInterrupt();
        }
    }
    can_ifaces[iface_index]->pollErrorFlagsFromISR();
}

uint32_t CANIface::FDCANMessageRAMOffset_ = 0;

// CANIface构造函数
CANIface::CANIface(uint8_t index) :
    rx_bytebuffer_((uint8_t*)rx_buffer, sizeof(rx_buffer)),
    rx_queue_(&rx_bytebuffer_),
    self_index_(index)
{
    if (index >= HAL_NUM_CAN_IFACES) {
         AP_HAL::panic("Bad CANIface index.");
    } else {
        can_ = Can[index];
    }
}

// 适用于数组的构造函数
CANIface::CANIface() :
    CANIface(next_interface++)
{}

// 处理总线关闭中断
void CANIface::handleBusOffInterrupt()
{
    _detected_bus_off = true;
}

// 计算CAN总线时序参数
bool CANIface::computeTimings(const uint32_t target_bitrate, Timings& out_timings) const
{
    // 检查目标比特率是否有效
    if (target_bitrate < 1) {
        return false;
    }

    /*
     * Hardware configuration
     */
    // 获取FDCAN时钟频率
    const uint32_t pclk = STM32_FDCANCLK;

    // 定义BS1和BS2的最大值
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
    // 根据目标比特率选择每位最大时间量子数
    const int max_quanta_per_bit = (target_bitrate >= 1000000) ? 10 : 17;

    // 定义最大采样点位置(千分之)
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
    // 初始化BS1和BS2的和
    uint8_t bs1_bs2_sum = uint8_t(max_quanta_per_bit - 1);

    // 寻找合适的BS1和BS2的和,使prescaler_bs能被整除
    while ((prescaler_bs % (1 + bs1_bs2_sum)) != 0) {
        if (bs1_bs2_sum <= 2) {
            return false;          // No solution
        }
        bs1_bs2_sum--;
    }

    // 计算预分频器值
    const uint32_t prescaler = prescaler_bs / (1 + bs1_bs2_sum);
    if ((prescaler < 1U) || (prescaler > 1024U)) {
        Debug("Timings: No Solution found\n");
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

        // 默认构造函数
        BsPair() :
            bs1(0),
            bs2(0),
            sample_point_permill(0)
        { }

        // 带参数构造函数,计算采样点位置
        BsPair(uint8_t bs1_bs2_sum, uint8_t arg_bs1) :
            bs1(arg_bs1),
            bs2(uint8_t(bs1_bs2_sum - bs1)),
            sample_point_permill(uint16_t(1000 * (1 + bs1) / (1 + bs1 + bs2)))
        {}

        // 验证BS1和BS2是否在有效范围内
        bool isValid() const
        {
            return (bs1 >= 1) && (bs1 <= MaxBS1) && (bs2 >= 1) && (bs2 <= MaxBS2);
        }
    };

    // 第一次尝试:四舍五入到最近值
    BsPair solution(bs1_bs2_sum, uint8_t(((7 * bs1_bs2_sum - 1) + 4) / 8));

    // 如果采样点位置超过最大值,尝试向下取整
    if (solution.sample_point_permill > MaxSamplePointLocation) {
        solution = BsPair(bs1_bs2_sum, uint8_t((7 * bs1_bs2_sum - 1) / 8));
    }

    // 打印调试信息
    Debug("Timings: quanta/bit: %d, sample point location: %.1f%%\n",
          int(1 + solution.bs1 + solution.bs2), float(solution.sample_point_permill) / 10.F);

    /*
     * Final validation
     * Helpful Python:
     * def sample_point_from_btr(x):
     *     assert 0b0011110010000000111111000000000 & x == 0
     *     ts2,ts1,brp = (x>>20)&7, (x>>16)&15, x&511
     *     return (1+ts1+1)/(1+ts1+1+ts2+1)
     *
     */
    // 验证计算结果是否有效
    if ((target_bitrate != (pclk / (prescaler * (1 + solution.bs1 + solution.bs2)))) || !solution.isValid()) {
        Debug("Timings: Invalid Solution %lu %lu %d %d %lu \n", pclk, prescaler, int(solution.bs1), int(solution.bs2), (pclk / (prescaler * (1 + solution.bs1 + solution.bs2))));
        return false;
    }

    // 设置输出参数
    // 设置采样点位置(千分比)
    out_timings.sample_point_permill = solution.sample_point_permill;
    // 设置预分频器值
    out_timings.prescaler = uint16_t(prescaler);
    // 设置同步跳转宽度为1
    out_timings.sjw = 1;
    // 设置BS1值
    out_timings.bs1 = uint8_t(solution.bs1);
    // 设置BS2值
    out_timings.bs2 = uint8_t(solution.bs2);
    return true;
}

/*
  table driven timings for CANFD
  These timings are from https://www.kvaser.com/support/calculators/can-fd-bit-timing-calculator
 */
// 计算CANFD时序参数
bool CANIface::computeFDTimings(const uint32_t target_bitrate, Timings& out_timings) const
{
    // 定义CANFD时序参数表
    static const struct {
        uint8_t bitrate_mbaud;  // 比特率(Mbps)
        uint8_t prescaler;      // 预分频器值
        uint8_t bs1;           // BS1值
        uint8_t bs2;           // BS2值
        uint8_t sjw;           // 同步跳转宽度
        uint8_t sample_point_pct; // 采样点位置(百分比)
    } CANFD_timings[] {
        { 1, 4, 14, 5, 5, 75},
        { 2, 2, 14, 5, 5, 75},
        { 4, 1, 14, 5, 5, 75},
        { 5, 1, 11, 4, 4, 75},
        { 8, 1,  6, 3, 3, 70},
    };
    // 查找匹配的时序参数
    for (const auto &t : CANFD_timings) {
        if (t.bitrate_mbaud*1000U*1000U == target_bitrate) {
            // 设置输出参数
            out_timings.prescaler = t.prescaler;
            out_timings.bs1 = t.bs1;
            out_timings.bs2 = t.bs2;
            out_timings.sjw = t.sjw;
            out_timings.sample_point_permill = t.sample_point_pct*10;
            return true;
        }
    }
    return false;
}

// 发送CAN帧
int16_t CANIface::send(const AP_HAL::CANFrame& frame, uint64_t tx_deadline,
                       CanIOFlags flags)
{
    // 检查接口是否已初始化
    if (!initialised_) {
        return -1;
    }

    // 更新发送统计信息
    stats.tx_requests++;
    // 验证帧格式是否有效
    if (frame.isErrorFrame() || (frame.dlc > 8 && !frame.isCanFDFrame()) ||
        frame.dlc > 15) {
        stats.tx_rejected++;
        return -1;
    }

    {
        // 进入临界区
        CriticalSectionLocker lock;

        /*
         * Seeking for an empty slot
         */
        uint8_t index;

        // 检查发送FIFO是否已满
        if ((can_->TXFQS & FDCAN_TXFQS_TFQF) != 0) {
            stats.tx_overflow++;
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
            return 0;    //we don't have free space
        }
        // 获取发送FIFO空闲索引
        index = ((can_->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos);

        // 计算发送元素地址
        uint32_t* buffer = (uint32_t *)(MessageRam_.TxFIFOQSA + (index * FDCAN_FRAME_BUFFER_SIZE * 4));

        // 设置帧ID
        if (frame.isExtended()) {
            buffer[0] = (IDE | frame.id);
        } else {
            buffer[0] = (frame.id << 18);
        }
        if (frame.isRemoteTransmissionRequest()) {
            buffer[0] |= RTR;
        }
        // 写入数据长度代码和消息标记
        buffer[1] =  frame.dlc << 16 | index << 24;

        // 处理CANFD帧
        if (frame.isCanFDFrame()) {
            buffer[1] |= FDF | BRS; // 启用CANFD传输和比特率切换
            stats.fdf_tx_requests++;
            pending_tx_[index].canfd_frame = true;
        } else {
            pending_tx_[index].canfd_frame = false;
        }

        // 将帧数据写入消息RAM
        const uint8_t data_length = AP_HAL::CANFrame::dlcToDataLength(frame.dlc);
        uint32_t *data_ptr = &buffer[2];
        for (uint8_t i = 0; i < (data_length+3)/4; i++) {
            data_ptr[i] = frame.data_32[i];
        }

        // 设置发送请求
        can_->TXBAR = (1 << index);

        // 注册待发送信息
        pending_tx_[index].deadline       = tx_deadline;
        pending_tx_[index].frame          = frame;
        pending_tx_[index].loopback       = (flags & AP_HAL::CANIface::Loopback) != 0;
        pending_tx_[index].abort_on_error = (flags & AP_HAL::CANIface::AbortOnError) != 0;
        pending_tx_[index].index          = index;
        // 设置帧初始状态
        pending_tx_[index].aborted        = false;
        pending_tx_[index].setup          = true;
        pending_tx_[index].pushed         = false;
    }

    // 同时在MAVCAN上发送,但如果MAVCAN发送失败不视为错误
    AP_HAL::CANIface::send(frame, tx_deadline, flags);

    return 1;
}

// 接收CAN帧
int16_t CANIface::receive(AP_HAL::CANFrame& out_frame, uint64_t& out_timestamp_us, CanIOFlags& out_flags)
{
    {
        // 进入临界区
        CriticalSectionLocker lock;
        CanRxItem rx_item;
        // 从接收队列中获取数据
        if (!rx_queue_.pop(rx_item) || !initialised_) {
            return 0;
        }
        // 设置输出参数
        out_frame    = rx_item.frame;
        out_timestamp_us = rx_item.timestamp_us;
        out_flags    = rx_item.flags;
    }

    return AP_HAL::CANIface::receive(out_frame, out_timestamp_us, out_flags);
}

// 配置CAN过滤器
bool CANIface::configureFilters(const CanFilterConfig* filter_configs,
                                uint16_t num_configs)
{
    // only enable filters in AP_Periph. It makes no sense on the flight controller
#if !defined(HAL_BUILD_AP_PERIPH) || defined(STM32G4)
    // 不使用过滤器
    // 退出初始化模式
    can_->CCCR &= ~FDCAN_CCCR_INIT; // 退出初始化模式
    uint32_t while_start_ms = AP_HAL::millis();
    // 等待退出初始化模式完成
    while ((can_->CCCR & FDCAN_CCCR_INIT) == 1) {
        // 如果超时则返回失败
        if ((AP_HAL::millis() - while_start_ms) > REG_SET_TIMEOUT) {
            return false;
        }
    }
    // 标记初始化完成
    initialised_ = true;
    return true;
#else
    // 统计标准ID和扩展ID过滤器数量
    uint32_t num_extid = 0, num_stdid = 0;
    // 可用过滤器列表大小
    uint32_t total_available_list_size = MAX_FILTER_LIST_SIZE;
    uint32_t* filter_ptr;
    // 检查是否已初始化或模式不正确
    if (initialised_ || mode_ != FilteredMode) {
        // 已初始化则无法配置过滤器,返回失败
        return false;
    }
    // 遍历过滤器配置,统计标准ID和扩展ID数量
    for (uint8_t i = 0; i < num_configs; i++) {
        const CanFilterConfig* const cfg = filter_configs + i;
        if ((cfg->id & AP_HAL::CANFrame::FlagEFF) || !(cfg->mask & AP_HAL::CANFrame::FlagEFF)) {
            num_extid++;
        } else {
            num_stdid++;
        }
    }
    CriticalSectionLocker lock;
    // 配置标准ID过滤器
    if (num_stdid == 0) { // 如果没有标准ID帧需要接收
#if defined(STM32G4)
        can_->RXGFC |= 0x2; // 拒绝所有标准ID帧
#else
        can_->GFC |= 0x2; // 拒绝所有标准ID帧
#endif
    } else if ((num_stdid < total_available_list_size) && (num_stdid <= 128)) {
        // 配置标准ID过滤器列表
        can_->SIDFC = (FDCANMessageRAMOffset_ << 2) | (num_stdid << 16);
        MessageRam_.StandardFilterSA = SRAMCAN_BASE + (FDCANMessageRAMOffset_ * 4U);
        FDCANMessageRAMOffset_ += num_stdid;
        total_available_list_size -= num_stdid;
        can_->GFC |= (0x3U << 4); // 拒绝不匹配的标准帧
    } else {    // 过滤器列表太大,返回失败
        can_->CCCR &= ~FDCAN_CCCR_INIT; // 退出初始化模式
        uint32_t while_start_ms = AP_HAL::millis();
        while ((can_->CCCR & FDCAN_CCCR_INIT) == 1) {
            if ((AP_HAL::millis() - while_start_ms) > REG_SET_TIMEOUT) {
                return false;
            }
        }
        return false;
    }

    // 如果有标准ID过滤器需要配置
    if (num_stdid) {
        num_stdid = 0;  // 重置列表计数
        filter_ptr = (uint32_t*)MessageRam_.StandardFilterSA;
        // 遍历过滤器列表并设置标准ID过滤器
        for (uint8_t i = 0; i < num_configs; i++) {
            uint32_t id = 0;
            uint32_t mask = 0;
            const CanFilterConfig* const cfg = filter_configs + i;
            if (!((cfg->id & AP_HAL::CANFrame::FlagEFF) || !(cfg->mask & AP_HAL::CANFrame::FlagEFF))) {
                id   = (cfg->id   & AP_HAL::CANFrame::MaskStdID);  // 提取标准ID
                mask = (cfg->mask & 0x7F);
                // 配置过滤器参数
                filter_ptr[num_stdid] = 0x2U << 30  | // 经典CAN过滤器
                                        0x1U << 27 |  // 匹配时存储到Rx FIFO0
                                        id << 16 |
                                        mask;
                num_stdid++;
            }
        }
    }

    // 配置扩展ID过滤器
    if (num_extid == 0) { // 如果没有扩展ID帧需要接收
        can_->GFC |= 0x1; // 拒绝所有扩展ID帧
    } else if ((num_extid < (total_available_list_size/2)) && (num_extid <= 64)) {
        // 配置扩展ID过滤器列表
        can_->XIDFC = (FDCANMessageRAMOffset_ << 2) | (num_extid << 16);
        MessageRam_.ExtendedFilterSA = SRAMCAN_BASE + (FDCANMessageRAMOffset_ * 4U);
        FDCANMessageRAMOffset_ += num_extid*2;
        can_->GFC |= (0x3U << 2); // 拒绝不匹配的扩展帧
    } else {    // 过滤器列表太大,返回失败
        can_->CCCR &= ~FDCAN_CCCR_INIT; // 退出初始化模式
        uint32_t while_start_ms = AP_HAL::millis();
        while ((can_->CCCR & FDCAN_CCCR_INIT) == 1) {
            if ((AP_HAL::millis() - while_start_ms) > REG_SET_TIMEOUT) {
                return false;
            }
        }
        return false;
    }

    // 如果有扩展ID过滤器需要配置
    if (num_extid) {
        num_extid = 0;
        filter_ptr = (uint32_t*)MessageRam_.ExtendedFilterSA;
        // 遍历过滤器列表并设置扩展ID过滤器
        for (uint8_t i = 0; i < num_configs; i++) {
            uint32_t id = 0;
            uint32_t mask = 0;
            const CanFilterConfig* const cfg = filter_configs + i;
            if ((cfg->id & AP_HAL::CANFrame::FlagEFF) || !(cfg->mask & AP_HAL::CANFrame::FlagEFF)) {
                id   = (cfg->id   & AP_HAL::CANFrame::MaskExtID);
                mask = (cfg->mask & AP_HAL::CANFrame::MaskExtID);
                // 配置过滤器参数
                filter_ptr[num_extid*2]       = 0x1U << 29 | id; // 匹配时存储到Rx FIFO0
                filter_ptr[num_extid*2 + 1]   = 0x2U << 30 | mask; // 经典CAN过滤器
                num_extid++;
            }
        }
    }

    // 计算消息RAM结束地址
    MessageRam_.EndAddress = SRAMCAN_BASE + (FDCANMessageRAMOffset_ * 4U);
    // 检查是否超出消息RAM限制
    if (MessageRam_.EndAddress > MESSAGE_RAM_END_ADDR) {
        // 消息RAM溢出,触发紧急停止
        AP_HAL::panic("CANFDIface: Message RAM Overflow!");
    }

    // 退出配置模式
    can_->CCCR &= ~FDCAN_CCCR_INIT; // 退出初始化模式
    uint32_t while_start_ms = AP_HAL::millis();
    // 等待退出初始化模式完成
    while ((can_->CCCR & FDCAN_CCCR_INIT) == 1) {
        if ((AP_HAL::millis() - while_start_ms) > REG_SET_TIMEOUT) {
            return false;
        }
    }
    // 标记初始化完成
    initialised_ = true;
    return true;
#endif // AP_Periph, STM32G4
}

    /*
     * Object state - interrupts are disabled, so it's safe to modify it now
     */
    // 清空接收队列
    rx_queue_.clear();
    // 清空所有发送邮箱的待发送项
    for (uint32_t i=0; i < NumTxMailboxes; i++) {
        pending_tx_[i] = CanTxItem();
    }
    // 重置发送邮箱峰值索引
    peak_tx_mailbox_index_ = 0;
    // 重置活动标志
    had_activity_ = false;

    /*
     * CAN timings for this bitrate
     */
    // 计算给定比特率的时序参数
    if (!computeTimings(bitrate, timings)) {
        // 如果计算失败,退出初始化模式
        can_->CCCR &= ~FDCAN_CCCR_INIT;
        uint32_t while_start_ms = AP_HAL::millis();
        // 等待退出初始化模式完成
        while ((can_->CCCR & FDCAN_CCCR_INIT) == 1) {
            if ((AP_HAL::millis() - while_start_ms) > REG_SET_TIMEOUT) {
                return false;
            }
        }
        return false;
    }
    // 保存当前比特率
    _bitrate = bitrate;
    Debug("Timings: presc=%u sjw=%u bs1=%u bs2=%u\n",
          unsigned(timings.prescaler), unsigned(timings.sjw), unsigned(timings.bs1), unsigned(timings.bs2));

    // 配置标称比特率时序寄存器
    can_->NBTP = (((timings.sjw-1) << FDCAN_NBTP_NSJW_Pos)   |
                  ((timings.bs1-1) << FDCAN_NBTP_NTSEG1_Pos) |
                  ((timings.bs2-1) << FDCAN_NBTP_NTSEG2_Pos)  |
                  ((timings.prescaler-1) << FDCAN_NBTP_NBRP_Pos));

    // 如果启用了CANFD,配置数据比特率
    if (fdbitrate) {
        // 计算CANFD数据比特率时序参数
        if (!computeFDTimings(fdbitrate, fdtimings)) {
            // 如果计算失败,退出初始化模式
            can_->CCCR &= ~FDCAN_CCCR_INIT;
            uint32_t while_start_ms = AP_HAL::millis();
            while ((can_->CCCR & FDCAN_CCCR_INIT) == 1) {
                if ((AP_HAL::millis() - while_start_ms) > REG_SET_TIMEOUT) {
                    return false;
                }
            }
            return false;
        }
        // 保存CANFD数据比特率
        _fdbitrate = fdbitrate;
        Debug("CANFD Timings: presc=%u bs1=%u bs2=%u\n",
              unsigned(fdtimings.prescaler), unsigned(fdtimings.bs1), unsigned(fdtimings.bs2));
        // 配置数据比特率时序寄存器
        can_->DBTP = (((fdtimings.bs1-1) << FDCAN_DBTP_DTSEG1_Pos) |
                      ((fdtimings.bs2-1) << FDCAN_DBTP_DTSEG2_Pos)  |
                      ((fdtimings.prescaler-1) << FDCAN_DBTP_DBRP_Pos) |
                      ((fdtimings.sjw-1) << FDCAN_DBTP_DSJW_Pos)) |
            FDCAN_DBTP_TDC;
        // 配置发送延迟补偿,适用于MCP2557FD收发器120ns延迟
        can_->TDCR = 10<<FDCAN_TDCR_TDCO_Pos;
    }

    //RX Config
#if defined(STM32H7)
    // 设置接收帧大小为8字节
    can_->RXESC = 0; //Set for 8Byte Frames
#endif

    // 初始化消息RAM区域
    setupMessageRam();
    // 重置总线关闭标志
    _detected_bus_off = false;
    // 清除所有中断标志
    can_->IR = 0x3FFFFFFF;
    // 使能相关中断
    can_->IE =  FDCAN_IE_TCE |  // 发送完成中断
                FDCAN_IE_BOE |  // 总线关闭错误中断
                FDCAN_IE_RF0NE |  // FIFO 0新消息中断
                FDCAN_IE_RF0FE |  // FIFO 0满中断
                FDCAN_IE_RF1NE |  // FIFO 1新消息中断
                FDCAN_IE_RF1FE;   // FIFO 1满中断
#if defined(STM32G4)
    // STM32G4特定中断线配置
    can_->ILS = FDCAN_ILS_PERR | FDCAN_ILS_SMSG;
#else
    // 设置发送完成和总线关闭中断到中断线1
    can_->ILS = FDCAN_ILS_TCL | FDCAN_ILS_BOE;  
#endif
    // 使能发送中断
#if defined(STM32G4)
    can_->TXBTIE = 0x7;
#else
    can_->TXBTIE = 0xFFFFFFFF;
#endif
    // 使能中断线0和1
    can_->ILE = 0x3;

#if HAL_CANFD_SUPPORTED
    // 使能CANFD和比特率切换功能
    can_->CCCR |= FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE; 
#endif

    // 如果是过滤模式,则在configureFilter中完成初始化
    // 否则在这里完成
    if (mode != FilteredMode) {
        // 退出初始化模式
        can_->CCCR &= ~FDCAN_CCCR_INIT; 
        uint32_t while_start_ms = AP_HAL::millis();
        // 等待退出初始化模式完成
        while ((can_->CCCR & FDCAN_CCCR_INIT) == 1) {
            if ((AP_HAL::millis() - while_start_ms) > REG_SET_TIMEOUT) {
                return false;
            }
        }

        // 标记初始化完成
        initialised_ = true;
    }
    return true;
}

bool CANIface::readRxFIFO(uint8_t fifo_index)
{
    uint32_t *frame_ptr;
    uint32_t index;
    uint64_t timestamp_us = AP_HAL::micros64();
    if (fifo_index == 0) {
#if !defined(STM32G4)
        //Check if RAM allocated to RX FIFO
        // 检查是否已为RX FIFO分配RAM
        if ((can_->RXF0C & FDCAN_RXF0C_F0S) == 0) {
            return false;
        }
#endif
        //Register Message Lost as a hardware error
        // 如果有消息丢失,记录为硬件错误
        if ((can_->RXF0S & FDCAN_RXF0S_RF0L) != 0) {
            stats.rx_errors++;
        }

        // 检查FIFO 0中是否还有消息
        if ((can_->RXF0S & FDCAN_RXF0S_F0FL) == 0) {
            return false; //No More messages in FIFO
        } else {
            // 获取FIFO 0中下一个要读取的消息索引
            index = ((can_->RXF0S & FDCAN_RXF0S_F0GI) >> 8);
            // 计算消息在RAM中的地址
            frame_ptr = (uint32_t *)(MessageRam_.RxFIFO0SA + (index * FDCAN_FRAME_BUFFER_SIZE * 4));
        }
    } else if (fifo_index == 1) {
#if !defined(STM32G4)
        //Check if RAM allocated to RX FIFO
        // 检查是否已为RX FIFO 1分配RAM
        if ((can_->RXF1C & FDCAN_RXF1C_F1S) == 0) {
            return false;
        }
#endif
        //Register Message Lost as a hardware error
        // 如果有消息丢失,记录为硬件错误
        if ((can_->RXF1S & FDCAN_RXF1S_RF1L) != 0) {
            stats.rx_errors++;
        }

        // 检查FIFO 1中是否还有消息
        if ((can_->RXF1S & FDCAN_RXF1S_F1FL) == 0) {
            return false;
        } else {
            // 获取FIFO 1中下一个要读取的消息索引
            index = ((can_->RXF1S & FDCAN_RXF1S_F1GI) >> 8);
            // 计算消息在RAM中的地址
            frame_ptr = (uint32_t *)(MessageRam_.RxFIFO1SA + (index * FDCAN_FRAME_BUFFER_SIZE * 4));
        }
    } else {
        return false;
    }

    // Read the frame contents
    // 创建CAN帧对象用于存储读取的数据
    AP_HAL::CANFrame frame {};
    uint32_t id = frame_ptr[0];
    if ((id & IDE) == 0) {
        //Standard ID
        // 标准ID格式,取出11位ID
        frame.id = ((id & STID_MASK) >> 18) & AP_HAL::CANFrame::MaskStdID;
    } else {
        //Extended ID
        // 扩展ID格式,取出29位ID
        frame.id = (id & EXID_MASK) & AP_HAL::CANFrame::MaskExtID;
        frame.id |= AP_HAL::CANFrame::FlagEFF;
    }

    // 检查是否为远程请求帧
    if ((id & RTR) != 0) {
        frame.id |= AP_HAL::CANFrame::FlagRTR;
    }

    // 检查是否为CANFD帧
    if (frame_ptr[1] & FDF) {
        frame.setCanFD(true);
        stats.fdf_rx_received++;
    } else {
        frame.setCanFD(false);
    }

    // 获取数据长度代码
    frame.dlc = (frame_ptr[1] & DLC_MASK) >> 16;
    uint8_t *data = (uint8_t*)&frame_ptr[2];

    // 复制数据到帧缓冲区
    for (uint8_t i = 0; i < AP_HAL::CANFrame::dlcToDataLength(frame.dlc); i++) {
        frame.data[i] = data[i];
    }

    //Acknowledge the FIFO entry we just read
    // 确认已读取的FIFO条目
    if (fifo_index == 0) {
        can_->RXF0A = index;
    } else if (fifo_index == 1) {
        can_->RXF1A = index;
    }

    /*
     * Store with timeout into the FIFO buffer
     */
    // 创建接收项并存入接收队列
    CanRxItem rx_item;
    rx_item.frame = frame;
    rx_item.timestamp_us = timestamp_us;
    rx_item.flags = 0;
    if (add_to_rx_queue(rx_item)) {
        stats.rx_received++;
    } else {
        stats.rx_overflow++;
    }
    return true;
}

void CANIface::handleRxInterrupt(uint8_t fifo_index)
{
    // 持续读取FIFO直到为空
    while (readRxFIFO(fifo_index)) {
        had_activity_ = true;
    }
    stats.num_events++;
    // 如果有信号量句柄,发出中断信号
    if (sem_handle != nullptr) {
        sem_handle->signal_ISR();
    }
}

/**
 * This method is used to count errors and abort transmission on error if necessary.
 * This functionality used to be implemented in the SCE interrupt handler, but that approach was
 * generating too much processing overhead, especially on disconnected interfaces.
 *
 * Should be called from RX ISR, TX ISR, and select(); interrupts must be enabled.
 */
void CANIface::pollErrorFlagsFromISR()
{
    // 获取错误计数器值
    const uint8_t cel = can_->ECR >> 16;

    // 如果有错误发生
    if (cel != 0) {
        // 记录错误寄存器状态
        stats.ecr = can_->ECR;
        // 遍历所有发送邮箱
        for (int i = 0; i < NumTxMailboxes; i++) {
            // 如果该邮箱不需要在错误时中止,或已经被中止,则跳过
            if (!pending_tx_[i].abort_on_error || pending_tx_[i].aborted) {
                continue;
            }
            // 检查该邮箱是否正在发送
            if (((1 << pending_tx_[i].index) & can_->TXBRP)) {
                // 中止该邮箱的发送
                can_->TXBCR = 1 << pending_tx_[i].index;  // Goodnight sweet transmission
                // 标记为已中止
                pending_tx_[i].aborted = true;
                // 增加发送中止计数
                stats.tx_abort++;
            }
        }
    }
}

// 在非中断上下文中轮询错误标志
void CANIface::pollErrorFlags()
{
    // 进入临界区
    CriticalSectionLocker cs_locker;
    // 调用中断版本的错误标志轮询函数
    pollErrorFlagsFromISR();
}

// 检查是否可以接受新的发送帧
bool CANIface::canAcceptNewTxFrame() const
{
#if !defined(STM32G4)
    // 检查发送FIFO是否已分配
    if ((can_->TXBC & FDCAN_TXBC_TFQS) == 0) {
        return false;
    }
#endif
    // 检查发送FIFO是否已满
    if ((can_->TXFQS & FDCAN_TXFQS_TFQF) != 0) {
        return false;    //we don't have free space
    }

    return true;
}

/**
 * Total number of hardware failures and other kinds of errors (e.g. queue overruns).
 * May increase continuously if the interface is not connected to the bus.
 */
uint32_t CANIface::getErrorCount() const
{
    // 进入临界区
    CriticalSectionLocker lock;
    // 返回所有类型错误的总和
    return stats.num_busoff_err +
           stats.rx_errors +
           stats.rx_overflow +
           stats.tx_rejected +
           stats.tx_abort +
           stats.tx_timedout;
}

// 设置事件处理句柄
bool CANIface::set_event_handle(AP_HAL::BinarySemaphore *handle)
{
    sem_handle = handle;
    return true;
}

// 检查接收缓冲区是否为空
bool CANIface::isRxBufferEmpty() const
{
    // 进入临界区
    CriticalSectionLocker lock;
    return rx_queue_.available() == 0;
}

// 清除错误状态
void CANIface::clearErrors()
{
    // 如果检测到总线关闭
    if (_detected_bus_off) {
        // 尝试从总线关闭状态恢复
        // 退出初始化模式,等待11个连续的隐性位(总线空闲)后
        // 才能重新参与总线活动
        can_->CCCR &= ~FDCAN_CCCR_INIT; // Leave init mode
        // 增加总线关闭错误计数
        stats.num_busoff_err++;
        // 清除总线关闭标志
        _detected_bus_off = false;
    }
}

// 丢弃超时的发送邮箱
void CANIface::discardTimedOutTxMailboxes(uint64_t current_time)
{
    // 进入临界区
    CriticalSectionLocker lock;
    // 遍历所有发送邮箱
    for (int i = 0; i < NumTxMailboxes; i++) {
        // 如果邮箱已中止或未设置,则跳过
        if (pending_tx_[i].aborted || !pending_tx_[i].setup) {
            continue;
        }
        // 如果邮箱正在发送且已超时
        if (((1 << pending_tx_[i].index) & can_->TXBRP) && pending_tx_[i].deadline < current_time) {
            // 中止发送
            can_->TXBCR = 1 << pending_tx_[i].index;  // Goodnight sweet transmission
            // 标记为已中止
            pending_tx_[i].aborted = true;
            // 增加发送超时计数
            stats.tx_timedout++;
        }
    }
}

// 检查CAN接口的读写可用性
// Check read/write availability of CAN interface
void CANIface::checkAvailable(bool& read, bool& write, const AP_HAL::CANFrame* pending_tx) const
{
    // 默认写不可用
    // Default write to false
    write = false;
    // 检查接收缓冲区是否为空来判断读是否可用
    // Check if read is available by checking if rx buffer is empty
    read = !isRxBufferEmpty();
    // 如果有待发送帧,检查是否可以接受新的发送帧
    // If there's a pending tx frame, check if we can accept new tx frame
    if (pending_tx != nullptr) {
        write = canAcceptNewTxFrame();
    }
}

// 等待CAN接口读写就绪
// Wait for CAN interface read/write ready
bool CANIface::select(bool &read, bool &write,
                      const AP_HAL::CANFrame* pending_tx,
                      uint64_t blocking_deadline)
{
    // 保存输入的读写标志
    // Save input read/write flags
    const bool in_read = read;
    const bool in_write= write;
    // 获取当前时间
    // Get current time
    uint64_t time = AP_HAL::micros64();

    // 检查请求是否有效
    // Check if request is valid
    if (!read && !write) {
        //invalid request
        return false;
    }

    // 丢弃超时的发送邮箱
    // Discard timed out tx mailboxes
    discardTimedOutTxMailboxes(time);              // Check TX timeouts - this may release some TX slots
    // 检查错误标志
    // Check error flags
    pollErrorFlags();
    // 清除错误状态
    // Clear error states
    clearErrors();

    // 检查是否已经有请求的事件
    // Check if we already have requested events
    checkAvailable(read, write, pending_tx);          // Check if we already have some of the requested events
    if ((read && in_read) || (write && in_write)) {
        return true;
    }
    // 在超时之前循环等待
    // Loop until timeout
    while (time < blocking_deadline) {
        // 如果没有信号量句柄则退出
        // Exit if no semaphore handle
        if (sem_handle == nullptr) {
            break;
        }
        // 等待信号量或超时
        // Wait for semaphore or timeout
        IGNORE_RETURN(sem_handle->wait(blocking_deadline - time)); // Block until timeout expires or any iface updates
        // 再次检查可用性
        // Check availability again
        checkAvailable(read, write, pending_tx);  // Check what we got
        if ((read && in_read) || (write && in_write)) {
            return true;
        }
        time = AP_HAL::micros64();
    }
    return false;
}

#if !defined(HAL_BOOTLOADER_BUILD)
// 获取CAN接口统计信息
// Get CAN interface statistics
void CANIface::get_stats(ExpandingString &str)
{
    // 进入临界区
    // Enter critical section
    CriticalSectionLocker lock;
    // 打印时钟配置和接口统计信息
    // Print clock config and interface statistics
    str.printf("------- Clock Config -------\n"
               "CAN_CLK_FREQ:   %luMHz\n"
               "Std Timings: bitrate=%lu presc=%u\n"
               "sjw=%u bs1=%u bs2=%u sample_point=%f%%\n"
               "FD Timings:  bitrate=%lu presc=%u\n"
               "sjw=%u bs1=%u bs2=%u sample_point=%f%%\n"
               "------- CAN Interface Stats -------\n"
               "tx_requests:    %lu\n"
               "tx_rejected:    %lu\n"
               "tx_overflow:    %lu\n"
               "tx_success:     %lu\n"
               "tx_timedout:    %lu\n"
               "tx_abort:       %lu\n"
               "rx_received:    %lu\n"
               "rx_overflow:    %lu\n"
               "rx_errors:      %lu\n"
               "num_busoff_err: %lu\n"
               "num_events:     %lu\n"
               "ECR:            %lx\n"
               "fdf_rx:         %lu\n"
               "fdf_tx_req:     %lu\n"
               "fdf_tx:         %lu\n",
               STM32_FDCANCLK/1000000UL,
               _bitrate, unsigned(timings.prescaler),
               unsigned(timings.sjw), unsigned(timings.bs1),
               unsigned(timings.bs2), timings.sample_point_permill/10.0f,
               _fdbitrate, unsigned(fdtimings.prescaler),
               unsigned(fdtimings.sjw), unsigned(fdtimings.bs1),
               unsigned(fdtimings.bs2), fdtimings.sample_point_permill/10.0f,
               stats.tx_requests,
               stats.tx_rejected,
               stats.tx_overflow,
               stats.tx_success,
               stats.tx_timedout,
               stats.tx_abort,
               stats.rx_received,
               stats.rx_overflow,
               stats.rx_errors,
               stats.num_busoff_err,
               stats.num_events,
               stats.ecr,
               stats.fdf_rx_received,
               stats.fdf_tx_requests,
               stats.fdf_tx_success);
}
#endif

/*
 * Interrupt handlers
 */
extern "C"
{
#ifdef HAL_CAN_IFACE1_ENABLE
    // FDCAN1中断处理函数
    // FDCAN1 interrupt handlers
    CH_IRQ_HANDLER(FDCAN1_IT0_IRQHandler);
    CH_IRQ_HANDLER(FDCAN1_IT0_IRQHandler)
    {
        CH_IRQ_PROLOGUE();
        handleCANInterrupt(0, 0);
        CH_IRQ_EPILOGUE();
    }

    CH_IRQ_HANDLER(FDCAN1_IT1_IRQHandler);
    CH_IRQ_HANDLER(FDCAN1_IT1_IRQHandler)
    {
        CH_IRQ_PROLOGUE();
        handleCANInterrupt(0, 1);
        CH_IRQ_EPILOGUE();
    }
#endif

#ifdef HAL_CAN_IFACE2_ENABLE
    // FDCAN2中断处理函数
    // FDCAN2 interrupt handlers
    CH_IRQ_HANDLER(FDCAN2_IT0_IRQHandler);
    CH_IRQ_HANDLER(FDCAN2_IT0_IRQHandler)
    {
        CH_IRQ_PROLOGUE();
        handleCANInterrupt(1, 0);
        CH_IRQ_EPILOGUE();
    }

    CH_IRQ_HANDLER(FDCAN2_IT1_IRQHandler);
    CH_IRQ_HANDLER(FDCAN2_IT1_IRQHandler)
    {
        CH_IRQ_PROLOGUE();
        handleCANInterrupt(1, 1);
        CH_IRQ_EPILOGUE();
    }
#endif

#ifdef HAL_CAN_IFACE3_ENABLE
    // FDCAN3中断处理函数
    // FDCAN3 interrupt handlers
    CH_IRQ_HANDLER(FDCAN3_IT0_IRQHandler);
    CH_IRQ_HANDLER(FDCAN3_IT0_IRQHandler)
    {
        CH_IRQ_PROLOGUE();
        handleCANInterrupt(2, 0);
        CH_IRQ_EPILOGUE();
    }

    CH_IRQ_HANDLER(FDCAN3_IT1_IRQHandler);
    CH_IRQ_HANDLER(FDCAN3_IT1_IRQHandler)
    {
        CH_IRQ_PROLOGUE();
        handleCANInterrupt(2, 1);
        CH_IRQ_EPILOGUE();
    }
#endif
    
} // extern "C"

#endif //defined(STM32H7XX) || defined(STM32G4)

#endif //HAL_NUM_CAN_IFACES
