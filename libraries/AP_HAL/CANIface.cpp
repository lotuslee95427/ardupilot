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

#include "CANIface.h"
#include "system.h"

// 判断当前帧优先级是否高于另一帧
bool AP_HAL::CANFrame::priorityHigherThan(const CANFrame& rhs) const
{
    // 清除标识位,只保留ID部分
    const uint32_t clean_id     = id     & MaskExtID;
    const uint32_t rhs_clean_id = rhs.id & MaskExtID;

    /*
     * STD vs EXT - if 11 most significant bits are the same, EXT loses.
     * 标准帧与扩展帧比较 - 如果最高11位相同,扩展帧优先级较低
     */
    const bool ext     = id     & FlagEFF;  // 是否为扩展帧
    const bool rhs_ext = rhs.id & FlagEFF;
    if (ext != rhs_ext) {
        // 获取11位仲裁段
        const uint32_t arb11     = ext     ? (clean_id >> 18)     : clean_id;
        const uint32_t rhs_arb11 = rhs_ext ? (rhs_clean_id >> 18) : rhs_clean_id;
        if (arb11 != rhs_arb11) {
            return arb11 < rhs_arb11;  // ID较小的优先级高
        } else {
            return rhs_ext;  // 标准帧优先级高于扩展帧
        }
    }

    /*
     * RTR vs Data frame - if frame identifiers and frame types are the same, RTR loses.
     * 远程帧与数据帧比较 - 如果帧ID和帧类型相同,远程帧优先级较低
     */
    const bool rtr     = id     & FlagRTR;  // 是否为远程帧
    const bool rhs_rtr = rhs.id & FlagRTR;
    if (clean_id == rhs_clean_id && rtr != rhs_rtr) {
        return rhs_rtr;  // 数据帧优先级高于远程帧
    }

    /*
     * Plain ID arbitration - greater value loses.
     * 普通ID仲裁 - ID值较大的优先级较低
     */
    return clean_id < rhs_clean_id;
}

/*
  parent class receive handling for MAVCAN
  MAVCAN接收处理的父类实现
 */
int16_t AP_HAL::CANIface::receive(CANFrame& out_frame, uint64_t& out_ts_monotonic, CanIOFlags& out_flags)
{
    if ((out_flags & IsMAVCAN) != 0) {
        return 1;
    }
#ifndef HAL_BOOTLOADER_BUILD
    WITH_SEMAPHORE(callbacks.sem);
#endif
    // 遍历所有回调函数并执行
    for (auto &cb : callbacks.cb) {
        if (cb != nullptr) {
            cb(get_iface_num(), out_frame);
        }
    }
    return 1;
}

/*
  parent class send handling for MAVCAN
  MAVCAN发送处理的父类实现
 */
int16_t AP_HAL::CANIface::send(const CANFrame& frame, uint64_t tx_deadline, CanIOFlags flags)
{
#ifndef HAL_BOOTLOADER_BUILD
    WITH_SEMAPHORE(callbacks.sem);
#endif
    bool added_to_rx_queue = false;
    // 遍历所有回调函数
    for (auto &cb : callbacks.cb) {
        if (cb == nullptr) {
            continue;
        }
        if ((flags & IsMAVCAN) == 0) {
            // 非MAVCAN帧直接调用回调
            cb(get_iface_num(), frame);
        } else if (!added_to_rx_queue) {
            // MAVCAN帧添加到接收队列
            CanRxItem rx_item;
            rx_item.frame = frame;
            rx_item.timestamp_us = AP_HAL::micros64();
            rx_item.flags = AP_HAL::CANIface::IsMAVCAN;
            add_to_rx_queue(rx_item);
            added_to_rx_queue = true;
        }
    }
    return 1;
}

/*
  register a callback for for sending CAN_FRAME messages.
  On success the returned callback_id can be used to unregister the callback
  注册CAN帧发送回调函数
  成功时返回的callback_id可用于注销回调
 */
bool AP_HAL::CANIface::register_frame_callback(FrameCb cb, uint8_t &callback_id)
{
#ifndef HAL_BOOTLOADER_BUILD
    WITH_SEMAPHORE(callbacks.sem);
#endif
    // 查找空闲的回调槽位
    for (uint8_t i=0; i<ARRAY_SIZE(callbacks.cb); i++) {
        if (callbacks.cb[i] == nullptr) {
            callbacks.cb[i] = cb;
            callback_id = i+1;
            return true;
        }
    }
    return false;
}

/*
  unregister a callback for for sending CAN_FRAME messages
  注销CAN帧发送回调函数
 */
void AP_HAL::CANIface::unregister_frame_callback(uint8_t callback_id)
{
#ifndef HAL_BOOTLOADER_BUILD
    WITH_SEMAPHORE(callbacks.sem);
#endif
    const uint8_t idx = callback_id - 1;
    if (idx < ARRAY_SIZE(callbacks.cb)) {
        callbacks.cb[idx] = nullptr;
    }
}

// CAN帧构造函数
AP_HAL::CANFrame::CANFrame(uint32_t can_id, const uint8_t* can_data, uint8_t data_len, bool canfd_frame) :
        id(can_id),
        canfd(canfd_frame)
{
    // 获取最大数据长度(CAN FD或普通CAN)
    const uint8_t data_len_max = canfd_frame ? MaxDataLen : NonFDCANMaxDataLen;
    // 参数检查
    if ((can_data == nullptr) || (data_len == 0) || (data_len > data_len_max)) {
        dlc = 0;
        memset(data, 0, MaxDataLen);
        return;
    }
    // 复制数据
    memcpy(this->data, can_data, data_len);
    memset(&this->data[data_len], 0, MaxDataLen-data_len);
    
    // 设置DLC
    if (data_len <= NonFDCANMaxDataLen) {
        dlc = data_len;
    } else {
        /*
        Data Length Code      9  10  11  12  13  14  15
        Number of data bytes 12  16  20  24  32  48  64
        */
        // CAN FD的DLC编码
        if (data_len <= 12) {
            dlc = 9;
        } else if (data_len <= 16) {
            dlc = 10;
        } else if (data_len <= 20) {
            dlc = 11;
        } else if  (data_len <= 24) {
            dlc = 12;
        } else if (data_len <= 32) {
            dlc = 13;
        } else if (data_len <= 48) {
            dlc = 14;
        } else if (data_len <= 64) {
            dlc = 15;
        }
    }
}

// 将数据长度转换为DLC编码
uint8_t AP_HAL::CANFrame::dataLengthToDlc(uint8_t data_length)
{
    if (data_length <= 8) {
        return data_length;
    } else if (data_length <= 12) {
        return 9;
    } else if (data_length <= 16) {
        return 10;
    } else if (data_length <= 20) {
        return 11;
    } else if (data_length <= 24) {
        return 12;
    } else if (data_length <= 32) {
        return 13;
    } else if (data_length <= 48) {
        return 14;
    }
    return 15;
}

// 将DLC编码转换为实际数据长度
uint8_t AP_HAL::CANFrame::dlcToDataLength(uint8_t dlc)
{
    /*
    Data Length Code      9  10  11  12  13  14  15
    Number of data bytes 12  16  20  24  32  48  64
    */
    if (dlc <= 8) {
        return dlc;
    } else if (dlc == 9) {
        return 12;
    } else if (dlc == 10) {
        return 16;
    } else if (dlc == 11) {
        return 20;
    } else if (dlc == 12) {
        return 24;
    } else if (dlc == 13) {
        return 32;
    } else if (dlc == 14) {
        return 48;
    }
    return 64;
}
