/*
 * Copyright (C) 2015-2016  Intel Corporation. All rights reserved.
 *
 * 本文件是自由软件：您可以根据自由软件基金会发布的GNU通用公共许可证的条款
 * 重新分发和/或修改它，可以选择使用许可证的第3版或更高版本。
 *
 * 本文件的发布是希望它能够有用，但不提供任何保证；甚至不保证它的
 * 适销性或适合特定用途。详细信息请参见GNU通用公共许可证。
 *
 * 您应该已经收到了GNU通用公共许可证的副本。如果没有，
 * 请访问 <http://www.gnu.org/licenses/>。
 */
#pragma once

#include <inttypes.h>

#include "AP_HAL_Namespace.h"
#include "Device.h"
#include "utility/OwnPtr.h"

namespace AP_HAL {

// I2CDevice类 - 定义了I2C设备的基本接口
class I2CDevice : public Device {
public:
    // 构造函数，设置设备类型为I2C
    I2CDevice() : Device(BUS_TYPE_I2C) { }

    // 虚析构函数
    virtual ~I2CDevice() { }

    /* 设备接口实现 */

    // 设置设备通信速度
    virtual bool set_speed(Device::Speed speed) override = 0;

    // 执行数据传输：发送和接收数据
    virtual bool transfer(const uint8_t *send, uint32_t send_len,
                          uint8_t *recv, uint32_t recv_len) override = 0;

    /*
     * 多次读取设备寄存器
     * @param first_reg: 起始寄存器地址
     * @param recv: 接收数据缓冲区
     * @param recv_len: 每次接收的数据长度
     * @param times: 读取次数
     */
    virtual bool read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                         uint32_t recv_len, uint8_t times) = 0;

    // 获取信号量，用于多线程同步
    virtual Semaphore *get_semaphore() override = 0;

    // 注册周期性回调函数
    virtual Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, Device::PeriodicCb) override = 0;

    // 调整周期性回调函数的时间间隔
    virtual bool adjust_periodic_callback(
        Device::PeriodicHandle h, uint32_t period_usec) override = 0;

    /*
     * 强制I2C传输在发送和接收部分之间分离，并在它们之间插入停止条件。
     * 这允许在这些设备上继续方便地使用read_*和transfer()方法。
     *
     * 某些平台可能始终进行分离传输，在这种情况下不需要此方法。
     */
    virtual void set_split_transfers(bool set) {};
};

// I2C设备管理器类 - 负责管理和创建I2C设备实例
class I2CDeviceManager {
public:
    /* 
     * 获取设备句柄
     * @param bus: 总线号
     * @param address: 设备地址
     * @param bus_clock: 总线时钟频率(默认400kHz)
     * @param use_smbus: 是否使用SMBus协议
     * @param timeout_ms: 超时时间(毫秒)
     */
    virtual OwnPtr<AP_HAL::I2CDevice> get_device(uint8_t bus, uint8_t address,
                                                 uint32_t bus_clock=400000,
                                                 bool use_smbus = false,
                                                 uint32_t timeout_ms=4) = 0;
    /*
     * 获取所有已配置I2C总线的掩码
     * 返回值默认为0x0F，表示支持0-3号总线
     */
    virtual uint32_t get_bus_mask(void) const { return 0x0F; }

    /*
     * 获取所有已配置外部I2C总线的掩码
     * 返回值默认为0x0F，表示支持0-3号外部总线
     */
    virtual uint32_t get_bus_mask_external(void) const { return 0x0F; }

    /*
     * 获取所有已配置内部I2C总线的掩码
     * 返回值默认为0x01，表示支持0号内部总线
     */
    virtual uint32_t get_bus_mask_internal(void) const { return 0x01; }
};

/*
 * 用于遍历I2C总线号的便捷宏
 * FOREACH_I2C_MASK: 遍历指定掩码的总线号
 * FOREACH_I2C_EXTERNAL: 遍历所有外部I2C总线
 * FOREACH_I2C_INTERNAL: 遍历所有内部I2C总线
 * FOREACH_I2C: 遍历所有I2C总线
 */
#define FOREACH_I2C_MASK(i,mask) for (uint32_t _bmask=mask, i=0; i<32; i++) if ((1U<<i)&_bmask)
#define FOREACH_I2C_EXTERNAL(i) FOREACH_I2C_MASK(i,hal.i2c_mgr->get_bus_mask_external())
#define FOREACH_I2C_INTERNAL(i) FOREACH_I2C_MASK(i,hal.i2c_mgr->get_bus_mask_internal())
#define FOREACH_I2C(i) FOREACH_I2C_MASK(i,hal.i2c_mgr->get_bus_mask())

}
