/*
 * 本文件是自由软件:你可以在自由软件基金会发布的GNU通用公共许可证的条款下
 * 重新分发和/或修改它,可以选择使用版本3或更高版本的许可证。
 *
 * 本文件的发布是希望它能有用,但不提供任何保证;甚至没有对适销性或特定用途
 * 适用性的暗示保证。详细信息请参见GNU通用公共许可证。
 *
 * 你应该随程序收到一份GNU通用公共许可证的副本。如果没有,
 * 请参见<http://www.gnu.org/licenses/>。
 */

#include "Device.h"

#include <stdio.h>
#include <AP_Common/AP_Common.h>

/*
  使用寄存器检查功能可以在运行时检查一组关键寄存器值是否发生变化。
  这对于可能遇到掉电或其他问题的关键传感器(如IMU)很有用。

  要使用寄存器检查功能:
  1. 调用setup_checked_registers()一次来分配检查寄存器值的空间
  2. 在需要保护的write_register()调用中设置checked标志
  3. 定期(比如50Hz)调用check_next_register()。如果返回false,
     说明传感器的寄存器值已被破坏。此时应将传感器标记为不健康。
     错误的值会被自动纠正。
 */

/*
  设置nregs个检查寄存器
 */
bool AP_HAL::Device::setup_checked_registers(uint8_t nregs, uint8_t frequency)
{
    if (_checked.regs != nullptr) {
        delete[] _checked.regs;
        _checked.n_allocated = 0;
        _checked.n_set = 0;
        _checked.next = 0;
    }
    _checked.regs = NEW_NOTHROW struct checkreg[nregs];
    if (_checked.regs == nullptr) {
        return false;
    }
    _checked.n_allocated = nregs;
    _checked.frequency = frequency;
    _checked.counter = 0;
    return true;
}

// 设置设备类型
void AP_HAL::Device::set_device_type(uint8_t devtype) {
    _bus_id.devid_s.devtype = devtype;
}

// 读取指定bank的寄存器值
bool AP_HAL::Device::read_bank_registers(uint8_t bank, uint8_t first_reg, uint8_t *recv, uint32_t recv_len)
{
    first_reg |= _read_flag;
    return transfer_bank(bank, &first_reg, 1, recv, recv_len);
}

// 写入指定bank的寄存器值
bool AP_HAL::Device::write_bank_register(uint8_t bank, uint8_t reg, uint8_t val, bool checked)
{
    uint8_t buf[2] = { reg, val };
    if (checked) {
        set_checked_register(bank, reg, val);
    }
    return transfer_bank(bank, buf, sizeof(buf), nullptr, 0);
}

/*
  设置一个检查寄存器的值
 */
void AP_HAL::Device::set_checked_register(uint8_t reg, uint8_t val)
{
    set_checked_register(0, reg, val);
}

// 设置指定bank的检查寄存器值
void AP_HAL::Device::set_checked_register(uint8_t bank, uint8_t reg, uint8_t val)
{
    if (_checked.regs == nullptr) {
        return;
    }
    struct checkreg *regs = _checked.regs;
    for (uint8_t i=0; i<_checked.n_set; i++) {
        if (regs[i].regnum == reg && regs[i].bank == bank) {
            regs[i].value = val;
            return;
        }
    }
    if (_checked.n_set == _checked.n_allocated) {
        printf("设备0x%x的寄存器0x%02x没有足够的检查寄存器空间\n",
               (unsigned)get_bus_id(), (unsigned)reg);
        return;
    }
    regs[_checked.n_set].bank = bank;
    regs[_checked.n_set].regnum = reg;
    regs[_checked.n_set].value = val;
    _checked.n_set++;
}

/*
  检查下一个寄存器的值
 */
bool AP_HAL::Device::check_next_register(void)
{
    if (_checked.n_set == 0) {
        return true;
    }
    if (++_checked.counter < _checked.frequency) {
        return true;
    }
    _checked.counter = 0;

    struct checkreg &reg = _checked.regs[_checked.next];
    uint8_t v, v2;

    if (_bank_select) {
        if (!_bank_select(reg.bank)) {
            // 无法设置bank
#if 0
            printf("设备0x%x设置bank 0x%02x失败\n",
                   (unsigned)get_bus_id(),
                   (unsigned)reg.bank);
#endif
            _checked.last_reg_fail = reg;
            return false;
        }
    }

    if ((!read_registers(reg.regnum, &v, 1) || v != reg.value) &&
        (!read_registers(reg.regnum, &v2, 1) || v2 != reg.value)) {
        // 寄存器值意外改变。尝试恢复并在下次重新检查
#if 0
        printf("设备0x%x修复寄存器0x%02x 0x%02x -> 0x%02x\n",
               (unsigned)get_bus_id(),
               (unsigned)reg.regnum, (unsigned)v, (unsigned)reg.value);
#endif
        write_register(reg.regnum, reg.value);
        _checked.last_reg_fail = reg;
        _checked.last_reg_fail.value = v;
        return false;
    }
    _checked.next = (_checked.next+1) % _checked.n_set;
    return true;
}

/*
  检查一个寄存器值,返回失败信息
 */
bool AP_HAL::Device::check_next_register(struct checkreg &fail)
{
    if (check_next_register()) {
        return true;
    }
    fail = _checked.last_reg_fail;
    return false;
}

// 写入寄存器值
bool AP_HAL::Device::write_register(uint8_t reg, uint8_t val, bool checked)
{
    uint8_t buf[2] = { reg, val };
    if (checked) {
        set_checked_register(reg, val);
    }
    bool result = transfer(buf, sizeof(buf), nullptr, 0);
    if (_register_rw_callback && result) {
        _register_rw_callback(reg, &val, 1, true);
    }
    return result;
}

// 读取寄存器值
bool AP_HAL::Device::read_registers(uint8_t first_reg, uint8_t *recv, uint32_t recv_len)
{
    uint8_t read_reg = first_reg;
    first_reg |= _read_flag;
    bool result = transfer(&first_reg, 1, recv, recv_len);
    if (_register_rw_callback != nullptr && result) {
        _register_rw_callback(read_reg, recv, recv_len, false);
    }
    return result;
}

// 在指定bank上传输数据
bool AP_HAL::Device::transfer_bank(uint8_t bank, const uint8_t *send, uint32_t send_len,
                        uint8_t *recv, uint32_t recv_len)
{
    if (_bank_select) {
        if (!_bank_select(bank)) {
            return false;
        }
    }
    return transfer(send, send_len, recv, recv_len);
}

/**
 * 一些连接在I2C或SPI总线上的设备需要在寄存器地址上设置一个位来执行读操作。
 * 这个函数设置read_registers()使用的标志。标志的默认值为零。
 */
void AP_HAL::Device::set_read_flag(uint8_t flag)
{
    _read_flag = flag;
}

/**
 * 根据总线类型、总线号、总线地址和设备类型生成总线ID。
 * 这用于不使用标准HAL设备类型的设备,如UAVCAN设备。
 */
uint32_t AP_HAL::Device::make_bus_id(enum BusType bus_type, uint8_t bus, uint8_t address, uint8_t devtype) {
    union DeviceId d {};
    d.devid_s.bus_type = bus_type;
    d.devid_s.bus = bus;
    d.devid_s.address = address;
    d.devid_s.devtype = devtype;
    return d.devid;
}

/**
 * 为相同的总线连接返回一个新的总线ID,但使用新的设备类型。
 * 这用于辅助总线连接。
 */
uint32_t AP_HAL::Device::change_bus_id(uint32_t old_id, uint8_t devtype) {
    union DeviceId d;
    d.devid = old_id;
    d.devid_s.devtype = devtype;
    return d.devid;
}

/**
 * 返回带有新设备类型的总线ID
 */
uint32_t AP_HAL::Device::get_bus_id_devtype(uint8_t devtype) const {
    return change_bus_id(get_bus_id(), devtype);
}

/**
 * 获取总线类型
 */
enum AP_HAL::Device::BusType AP_HAL::Device::devid_get_bus_type(uint32_t dev_id) {
    union DeviceId d;
    d.devid = dev_id;
    return d.devid_s.bus_type;
}

// 获取总线号
uint8_t AP_HAL::Device::devid_get_bus(uint32_t dev_id) {
    union DeviceId d;
    d.devid = dev_id;
    return d.devid_s.bus;
}

// 获取总线地址
uint8_t AP_HAL::Device::devid_get_address(uint32_t dev_id) {
    union DeviceId d;
    d.devid = dev_id;
    return d.devid_s.address;
}

// 获取设备类型
uint8_t AP_HAL::Device::devid_get_devtype(uint32_t dev_id) {
    union DeviceId d;
    d.devid = dev_id;
    return d.devid_s.devtype;
}
