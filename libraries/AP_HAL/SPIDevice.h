/*
 * Copyright (C) 2015-2016  Intel Corporation. All rights reserved.
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
#pragma once

#include <inttypes.h>

#include "AP_HAL_Namespace.h"
#include "Device.h"
#include "utility/OwnPtr.h"

namespace AP_HAL {

// SPI设备类,继承自Device基类
class SPIDevice : public Device {
public:
    // 构造函数,指定设备类型为SPI
    SPIDevice() : Device(BUS_TYPE_SPI) { }

    // 虚析构函数
    virtual ~SPIDevice() { }
    
    /* Device接口实现 */

    /* 设置SPI通信速度,继承自Device类 */
    virtual bool set_speed(Device::Speed speed) override = 0;

    /* 
     * SPI数据传输函数,继承自Device类
     * @param send: 发送数据缓冲区
     * @param send_len: 发送数据长度
     * @param recv: 接收数据缓冲区
     * @param recv_len: 接收数据长度
     * @return: 传输是否成功
     */
    virtual bool transfer(const uint8_t *send, uint32_t send_len,
                          uint8_t *recv, uint32_t recv_len) override = 0;

    /*
     * 全双工SPI传输函数
     * 与transfer()不同,发送和接收同时进行,因此send和recv缓冲区必须等长
     * @param send: 发送数据缓冲区
     * @param recv: 接收数据缓冲区
     * @param len: 数据长度
     * @return: 传输是否成功
     */
    virtual bool transfer_fullduplex(const uint8_t *send, uint8_t *recv,
                                     uint32_t len) = 0;

    /* 
     * 在不使用片选(CS)的情况下发送指定数量的时钟脉冲
     * 用于初始化通过SPI接口的microSD卡
     * @param len: 时钟脉冲数量
     * @return: 操作是否成功
     */
    virtual bool clock_pulse(uint32_t len) { return false; }
    
    /* 获取设备信号量,继承自Device类 */
    virtual Semaphore *get_semaphore() override = 0;

    /* 注册周期性回调函数,继承自Device类 */
    virtual Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, Device::PeriodicCb) override = 0;

    /* 调整周期性回调函数的周期,继承自Device类 */
    virtual bool adjust_periodic_callback(
        PeriodicHandle h, uint32_t period_usec) override { return false; }

    // 设置总线时钟减速因子(可选接口)
    virtual void set_slowdown(uint8_t slowdown) {}
};

// SPI设备管理器类
class SPIDeviceManager {
public:
    // 根据名称获取SPI设备实例
    virtual OwnPtr<SPIDevice> get_device(const char *name)
    {
        return nullptr;
    }

    /* 返回当前注册的SPI设备数量 */
    virtual uint8_t get_count() { return 0; }

    /* 获取指定索引的SPI设备名称 */
    virtual const char *get_device_name(uint8_t idx) { return nullptr; }

    /* 设置寄存器读写回调函数 */
    virtual void set_register_rw_callback(const char* name, AP_HAL::Device::RegisterRWCb cb) {}
};

}
