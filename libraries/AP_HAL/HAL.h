// 防止头文件重复包含
#pragma once

class AP_Param;

// 包含所需的头文件
#include "AP_HAL_Namespace.h"

#include "AnalogIn.h"      // 模拟输入
#include "GPIO.h"          // 通用输入输出
#include "RCInput.h"       // 遥控器输入
#include "RCOutput.h"      // 遥控器输出
#include "SPIDevice.h"     // SPI设备
#include "WSPIDevice.h"    // WSPI设备
#include "Storage.h"       // 存储
#include "UARTDriver.h"    // 串口驱动
#include "system.h"        // 系统
#include "OpticalFlow.h"   // 光流
#include "DSP.h"          // 数字信号处理
#include "CANIface.h"      // CAN接口


// HAL(Hardware Abstraction Layer)硬件抽象层类定义
class AP_HAL::HAL {
public:
    // 构造函数,初始化各种硬件接口
    HAL(AP_HAL::UARTDriver* _serial0, // 控制台
        AP_HAL::UARTDriver* _serial1, // 遥测1
        AP_HAL::UARTDriver* _serial2, // 遥测2
        AP_HAL::UARTDriver* _serial3, // GPS1
        AP_HAL::UARTDriver* _serial4, // GPS2
        AP_HAL::UARTDriver* _serial5, // 额外串口1
        AP_HAL::UARTDriver* _serial6, // 额外串口2
        AP_HAL::UARTDriver* _serial7, // 额外串口3
        AP_HAL::UARTDriver* _serial8, // 额外串口4
        AP_HAL::UARTDriver* _serial9, // 额外串口5
        AP_HAL::I2CDeviceManager* _i2c_mgr,    // I2C设备管理器
        AP_HAL::SPIDeviceManager* _spi,        // SPI设备管理器
        AP_HAL::WSPIDeviceManager* _wspi,      // WSPI设备管理器
        AP_HAL::AnalogIn*   _analogin,         // 模拟输入
        AP_HAL::Storage*    _storage,          // 存储
        AP_HAL::UARTDriver* _console,          // 控制台
        AP_HAL::GPIO*       _gpio,             // GPIO
        AP_HAL::RCInput*    _rcin,             // 遥控输入
        AP_HAL::RCOutput*   _rcout,            // 遥控输出
        AP_HAL::Scheduler*  _scheduler,        // 调度器
        AP_HAL::Util*       _util,             // 工具类
        AP_HAL::OpticalFlow*_opticalflow,      // 光流
        AP_HAL::Flash*      _flash,            // Flash存储
#if AP_SIM_ENABLED && CONFIG_HAL_BOARD != HAL_BOARD_SITL
        class AP_HAL::SIMState*   _simstate,   // 仿真状态
#endif
#if HAL_WITH_DSP
        AP_HAL::DSP*        _dsp,              // DSP处理器
#endif
#if HAL_NUM_CAN_IFACES > 0
        AP_HAL::CANIface* _can_ifaces[HAL_NUM_CAN_IFACES])  // CAN接口数组
#else
        AP_HAL::CANIface** _can_ifaces)       // CAN接口指针
#endif
        :
        // 成员初始化列表
        i2c_mgr(_i2c_mgr),
        spi(_spi),
        wspi(_wspi),
        analogin(_analogin),
        storage(_storage),
        console(_console),
        gpio(_gpio),
        rcin(_rcin),
        rcout(_rcout),
        scheduler(_scheduler),
        util(_util),
        opticalflow(_opticalflow),
#if AP_SIM_ENABLED && CONFIG_HAL_BOARD != HAL_BOARD_SITL
        simstate(_simstate),
#endif
        flash(_flash),
#if HAL_WITH_DSP
        dsp(_dsp),
#endif
        serial_array{
            _serial0,
            _serial1,
            _serial2,
            _serial3,
            _serial4,
            _serial5,
            _serial6,
            _serial7,
            _serial8,
            _serial9}
    {
#if HAL_NUM_CAN_IFACES > 0
        // 初始化CAN接口
        if (_can_ifaces == nullptr) {
            for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++)
                can[i] = nullptr;
        } else {
            for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++)
                can[i] = _can_ifaces[i];
        }
#endif

        AP_HAL::init();
    }

    // 回调函数结构体定义
    struct Callbacks {
        virtual void setup() = 0;  // 初始化函数
        virtual void loop() = 0;   // 循环函数
    };

    // 函数回调结构体定义
    struct FunCallbacks : public Callbacks {
        FunCallbacks(void (*setup_fun)(void), void (*loop_fun)(void));

        void setup() override { _setup(); }
        void loop() override { _loop(); }

    private:
        void (*_setup)(void);  // 初始化函数指针
        void (*_loop)(void);   // 循环函数指针
    };

    // 运行函数,由具体硬件实现
    virtual void run(int argc, char * const argv[], Callbacks* callbacks) const = 0;

public:
    // 硬件接口指针
    AP_HAL::I2CDeviceManager* i2c_mgr;    // I2C管理器
    AP_HAL::SPIDeviceManager* spi;        // SPI管理器
    AP_HAL::WSPIDeviceManager* wspi;      // WSPI管理器
    AP_HAL::AnalogIn*   analogin;         // 模拟输入
    AP_HAL::Storage*    storage;          // 存储
    AP_HAL::UARTDriver* console;          // 控制台
    AP_HAL::GPIO*       gpio;             // GPIO
    AP_HAL::RCInput*    rcin;             // 遥控输入
    AP_HAL::RCOutput*   rcout;            // 遥控输出
    AP_HAL::Scheduler*  scheduler;        // 调度器
    AP_HAL::Util        *util;            // 工具类
    AP_HAL::OpticalFlow *opticalflow;     // 光流
    AP_HAL::Flash       *flash;           // Flash存储
    AP_HAL::DSP         *dsp;             // DSP处理器
#if HAL_NUM_CAN_IFACES > 0
    AP_HAL::CANIface* can[HAL_NUM_CAN_IFACES];  // CAN接口数组
#else
    AP_HAL::CANIface** can;                     // CAN接口指针
#endif

    // 通过序号访问串口
    UARTDriver* serial(uint8_t sernum) const;

    // 串口数量常量
    static constexpr uint8_t num_serial = 10;

private:
    // 串口驱动数组
    AP_HAL::UARTDriver* serial_array[num_serial];

public:
#if AP_SIM_ENABLED && CONFIG_HAL_BOARD != HAL_BOARD_SITL
    AP_HAL::SIMState *simstate;           // 仿真状态
#endif

// 调试打印宏定义
#ifndef HAL_CONSOLE_DISABLED
# define DEV_PRINTF(fmt, args ...)  do { hal.console->printf(fmt, ## args); } while(0)
#else
# define DEV_PRINTF(fmt, args ...)
#endif

};

// 通过序号获取串口驱动的内联函数实现
inline AP_HAL::UARTDriver* AP_HAL::HAL::serial(uint8_t sernum) const
{
    if (sernum >= ARRAY_SIZE(serial_array)) {
        return nullptr;
    }
    return serial_array[sernum];
}
