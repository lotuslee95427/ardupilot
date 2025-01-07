#pragma once

#include "string.h"
#include "utility/functor.h"

namespace AP_HAL {

    /* 顶层纯虚类HAL */
    class HAL;

    /* 驱动程序的顶层类名: */
    class UARTDriver;      // 串口驱动
    class I2CDevice;       // I2C设备
    class I2CDeviceManager;// I2C设备管理器
    class Device;          // 设备基类

    class SPIDevice;       // SPI设备
    class SPIDeviceDriver; // SPI设备驱动
    class SPIDeviceManager;// SPI设备管理器

    class AnalogSource;    // 模拟输入源
    class AnalogIn;        // 模拟输入
    class Storage;         // 存储
    class DigitalSource;   // 数字输入源
    class PWMSource;       // PWM输入源
    class GPIO;            // 通用输入输出
    class RCInput;         // 遥控输入
    class RCOutput;        // 遥控输出
    class Scheduler;       // 调度器
    class Semaphore;       // 信号量
    class BinarySemaphore; // 二值信号量
    class OpticalFlow;     // 光流
    class DSP;            // 数字信号处理

    class WSPIDevice;      // WSPI设备
    class WSPIDeviceDriver;// WSPI设备驱动
    class WSPIDeviceManager;// WSPI设备管理器

    class CANIface;        // CAN接口
    class CANFrame;        // CAN帧

    class Util;            // 工具类
    class Flash;           // Flash存储

    /* 工具类 */
    class Print;           // 打印类
    class Stream;          // 流类
    class BetterStream;    // 增强流类

    /* 函数指针类型定义(过程,成员过程)
       对于成员函数,我们使用FastDelegate委托类
       它允许我们将成员函数封装为一个类型
     */
    typedef void(*Proc)(void);  // 普通函数指针类型
    FUNCTOR_TYPEDEF(MemberProc, void);  // 成员函数指针类型

    /**
     * 所有平台上所有现有SPI设备的全局名称
     */
    enum SPIDeviceType {
        // 使用AP_HAL::SPIDevice抽象的设备
        SPIDevice_Type              = -1,
    };

    class SIMState;       // 仿真状态类

    // 必须由具体的HAL实现并返回相同的引用
    const HAL& get_HAL();      // 获取HAL只读引用
    HAL& get_HAL_mutable();    // 获取HAL可修改引用
}
