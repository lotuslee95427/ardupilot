#pragma once

namespace ChibiOS {
    class AnalogIn;        // 模拟输入类,用于读取模拟传感器数据
    class AnalogSource;    // 模拟信号源类,表示单个模拟输入通道
    class DigitalSource;   // 数字信号源类,用于读取数字IO
#if HAL_WITH_IO_MCU
    class IOMCU_DigitalSource;  // IO协处理器数字信号源类
#endif
    class DSP;            // 数字信号处理类
    class GPIO;           // 通用输入输出接口类
    class I2CBus;         // I2C总线类
    class I2CDevice;      // I2C设备类
    class I2CDeviceManager;  // I2C设备管理器类
    class OpticalFlow;    // 光流传感器类
    class RCInput;        // 遥控输入类,用于接收遥控信号
    class RCOutput;       // 遥控输出类,用于控制舵机等执行器
    class Scheduler;      // 调度器类,用于任务调度
    class Semaphore;      // 信号量类,用于线程同步
    class BinarySemaphore;  // 二值信号量类
    class SPIBus;         // SPI总线类
    class SPIDesc;        // SPI设备描述符类
    class SPIDevice;      // SPI设备类
    class SPIDeviceDriver;  // SPI设备驱动类
    class SPIDeviceManager;  // SPI设备管理器类
    class WSPIBus;        // WSPI(无线SPI)总线类
    class WSPIDesc;       // WSPI设备描述符类
    class WSPIDevice;     // WSPI设备类
    class WSPIDeviceManager;  // WSPI设备管理器类
    class Storage;        // 存储类,用于数据持久化
    class UARTDriver;     // 串口驱动类
    class Util;           // 实用工具类
    class Shared_DMA;     // 共享DMA类
    class SoftSigReader;  // 软件信号读取器类
    class SoftSigReaderInt;  // 软件信号读取器中断类
    class CANIface;       // CAN总线接口类
    class Flash;          // Flash存储器操作类
}
