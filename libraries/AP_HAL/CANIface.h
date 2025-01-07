/*
 * Copyright (C) 2020 Siddharth B Purohit
 *
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

// 确保头文件只被包含一次
#pragma once

// 包含标准整数类型定义
#include <stdint.h>
// 包含HAL命名空间定义
#include "AP_HAL_Namespace.h"
// 包含HAL支持的主板定义
#include "AP_HAL_Boards.h"

class ExpandingString;

/**
 * 原始CAN帧,用于CAN驱动程序的输入/输出
 */
struct AP_HAL::CANFrame {
    static const uint32_t MaskStdID = 0x000007FFU;    // 标准帧ID掩码(11位)
    static const uint32_t MaskExtID = 0x1FFFFFFFU;    // 扩展帧ID掩码(29位)
    static const uint32_t FlagEFF = 1U << 31;         // 扩展帧格式标志位
    static const uint32_t FlagRTR = 1U << 30;         // 远程传输请求标志位
    static const uint32_t FlagERR = 1U << 29;         // 错误帧标志位

#if HAL_CANFD_SUPPORTED
    // 支持CANFD时的最大数据长度定义
    static const uint8_t NonFDCANMaxDataLen = 8;      // 普通CAN最大数据长度
    static const uint8_t MaxDataLen = 64;             // CANFD最大数据长度
#else
    // 不支持CANFD时的最大数据长度定义
    static const uint8_t NonFDCANMaxDataLen = 8;
    static const uint8_t MaxDataLen = 8;
#endif
    uint32_t id;                // CAN ID及标志位
    union {
        uint8_t data[MaxDataLen];       // 字节数组形式的数据
        uint32_t data_32[MaxDataLen/4]; // 32位整数数组形式的数据
    };
    uint8_t dlc;               // 数据长度代码
    bool canfd;                // 是否为CANFD帧

    // 构造函数,初始化所有成员
    CANFrame() :
        id(0),
        dlc(0),
        canfd(false)
    {
        memset(data,0, MaxDataLen);
    }

    // 带参数的构造函数
    CANFrame(uint32_t can_id, const uint8_t* can_data, uint8_t data_len, bool canfd_frame = false);

    // 不等运算符重载
    bool operator!=(const CANFrame& rhs) const
    {
        return !operator==(rhs);
    }
    // 相等运算符重载
    bool operator==(const CANFrame& rhs) const
    {
        return (id == rhs.id) && (dlc == rhs.dlc) && (memcmp(data, rhs.data, dlc) == 0);
    }

    // 返回ID的有符号版本,用于脚本处理(uint32_t开销较大)
    int32_t id_signed(void) const {
        return isExtended()? int32_t(id & MaskExtID) : int32_t(id & MaskStdID);
    }

    // 判断是否为扩展帧
    bool isExtended() const
    {
        return id & FlagEFF;
    }
    // 判断是否为远程帧
    bool isRemoteTransmissionRequest() const
    {
        return id & FlagRTR;
    }
    // 判断是否为错误帧
    bool isErrorFrame() const
    {
        return id & FlagERR;
    }
    // 设置是否为CANFD帧
    void setCanFD(bool canfd_frame)
    {
        canfd = canfd_frame;
    }

    // 判断是否为CANFD帧
    bool isCanFDFrame() const
    {
        return canfd;
    }

    // DLC与实际数据长度的转换函数
    static uint8_t dlcToDataLength(uint8_t dlc);
    static uint8_t dataLengthToDlc(uint8_t data_length);

    /**
     * CAN帧仲裁规则,特别是标准帧vs扩展帧:
     * Marco Di Natale - "Understanding and using the Controller Area Network"
     * http://www6.in.tum.de/pub/Main/TeachingWs2013MSE/CANbus.pdf
     */
    // 判断优先级是否高于另一帧
    bool priorityHigherThan(const CANFrame& rhs) const;
    // 判断优先级是否低于另一帧
    bool priorityLowerThan(const CANFrame& rhs) const
    {
        return rhs.priorityHigherThan(*this);
    }
};

// CAN接口类
class AP_HAL::CANIface
{
public:
    // CAN接口工作模式枚举
    enum OperatingMode {
        PassThroughMode,    // 透传模式
        NormalMode,         // 正常模式
        SilentMode,         // 静默模式
        FilteredMode        // 过滤模式
    };

    // 获取当前工作模式
    OperatingMode get_operating_mode() { return mode_; }

    // CAN IO标志位类型定义
    typedef uint16_t CanIOFlags;
    static const CanIOFlags Loopback = 1;        // 回环模式
    static const CanIOFlags AbortOnError = 2;    // 出错时中止
    static const CanIOFlags IsMAVCAN = 4;        // MAVCAN协议

    // 单个接收帧及相关信息结构体
    struct CanRxItem {
        uint64_t timestamp_us = 0;   // 时间戳(微秒)
        CanIOFlags flags = 0;        // IO标志位
        CANFrame frame;              // CAN帧
    };

    // 单个发送帧及相关信息结构体
    struct CanTxItem {
        uint64_t deadline = 0;       // 截止时间
        CANFrame frame;              // CAN帧
        uint32_t index = 0;          // 索引
        bool loopback:1;             // 是否回环
        bool abort_on_error:1;       // 出错时是否中止
        bool aborted:1;              // 是否已中止
        bool pushed:1;               // 是否已推送
        bool setup:1;                // 是否已设置
        bool canfd_frame:1;          // 是否为CANFD帧

        // 小于运算符重载,用于优先级比较
        bool operator<(const CanTxItem& rhs) const
        {
            if (frame.priorityLowerThan(rhs.frame)) {
                return true;
            }
            if (frame.priorityHigherThan(rhs.frame)) {
                return false;
            }
            return index > rhs.index;
        }
    };

    // CAN过滤器配置结构体
    struct CanFilterConfig {
        uint32_t id = 0;            // 过滤器ID
        uint32_t mask = 0;          // 过滤器掩码

        // 相等运算符重载
        bool operator==(const CanFilterConfig& rhs) const
        {
            return rhs.id == id && rhs.mask == mask;
        }
    };

    // 初始化接口(支持CANFD)
    virtual bool init(const uint32_t bitrate, const uint32_t fdbitrate, const OperatingMode mode) {
        return init(bitrate, mode);
    }

    // 初始化接口(基本CAN)
    virtual bool init(const uint32_t bitrate, const OperatingMode mode) = 0;

    // 选择方法通知用户Rx和Tx缓冲区状态
    // 如果Rx缓冲区有帧可用,则read_select为true
    // 如果Tx缓冲区有空间可用,则write_select为true
    // 根据read_select和write_select的值等待Rx或Tx事件,直到超时
    // 如果等待期间发生Rx/Tx事件则返回true,超时则返回false
    virtual bool select(bool &read_select, bool &write_select,
                        const CANFrame* const pending_tx, uint64_t timeout)
    {
        return false;
    }

    // 设置事件句柄
    virtual bool set_event_handle(AP_HAL::BinarySemaphore *sem_handle)
    {
        return true;
    }

    // 将帧放入发送队列
    // 返回负值表示出错,0表示无空间,1表示成功
    // 必须在子类中实现
    virtual int16_t send(const CANFrame& frame, uint64_t tx_deadline, CanIOFlags flags);

    // 非阻塞接收帧,从缓冲区弹出接收到的帧
    // 返回负值表示出错,0表示无帧可用,1表示成功
    // 必须在子类中实现
    virtual int16_t receive(CANFrame& out_frame, uint64_t& out_ts_monotonic, CanIOFlags& out_flags);

    // 配置过滤器以拒绝不需要处理的帧
    virtual bool configureFilters(const CanFilterConfig* filter_configs, uint16_t num_configs)
    {
        return 0;
    }

    // 返回可用的硬件过滤器数量
    virtual uint16_t getNumFilters() const
    {
        return 0;
    }

    // 返回累计错误计数
    virtual uint32_t getErrorCount() const
    {
        return 0;
    }

    // 总线统计信息结构体
    typedef struct {
        uint32_t tx_requests;      // 发送请求数
        uint32_t tx_rejected;      // 发送被拒绝数
        uint32_t tx_overflow;      // 发送溢出数
        uint32_t tx_success;       // 发送成功数
        uint32_t tx_timedout;      // 发送超时数
        uint32_t tx_abort;         // 发送中止数
        uint32_t rx_received;      // 接收帧数
        uint32_t rx_overflow;      // 接收溢出数
        uint32_t rx_errors;        // 接收错误数
        uint32_t num_busoff_err;   // 总线关闭错误数
        uint64_t last_transmit_us; // 最后发送时间(微秒)
    } bus_stats_t;

#if !defined(HAL_BOOTLOADER_BUILD)
    // 获取接口状态信息
    virtual void get_stats(ExpandingString &str) {}

    /*
      返回总线统计信息用于日志记录
      如果没有统计信息可用则返回nullptr
     */
    virtual const bus_stats_t *get_statistics(void) const { return nullptr; };
#endif

    // 返回总线关闭状态是否被检测到且未清除
    virtual bool is_busoff() const
    {
        return false;
    }

    // 仅用于测试CANBus的方法
    // 不用于正常操作或使用场景
    virtual void flush_tx() {}     // 刷新发送缓冲区
    virtual void clear_rx() {}     // 清空接收缓冲区

    // 返回init是否被调用且成功
    virtual bool is_initialized() const = 0;

    // 定义帧回调函数类型
    FUNCTOR_TYPEDEF(FrameCb, void, uint8_t, const AP_HAL::CANFrame &);

    // 注册帧回调函数
    virtual bool register_frame_callback(FrameCb cb, uint8_t &cb_id);
    // 注销帧回调函数
    virtual void unregister_frame_callback(uint8_t cb_id);

protected:
    // 获取接口编号
    virtual int8_t get_iface_num() const = 0;
    // 添加到接收队列
    virtual bool add_to_rx_queue(const CanRxItem &rx_item) = 0;

    // 回调函数相关结构
    struct {
#ifndef HAL_BOOTLOADER_BUILD
        HAL_Semaphore sem;        // 信号量
#endif
        // 每个接口最多允许2个回调
        FrameCb cb[2];            // 回调函数数组
    } callbacks;

    uint32_t bitrate_;           // 波特率
    OperatingMode mode_;         // 工作模式
};
