#pragma once

/* Umbrella header for all private headers of the AP_HAL_ChibiOS module.
 * Only import this header from inside AP_HAL_ChibiOS
 */

// 包含模拟输入相关功能,用于读取模拟传感器数据
#include "AnalogIn.h"
// 包含通用输入输出接口功能,用于控制GPIO引脚
#include "GPIO.h"
// 包含任务调度器功能,用于管理系统任务的执行
#include "Scheduler.h"
// 包含实用工具功能,提供一些通用的辅助函数
#include "Util.h"
// 包含串口驱动功能,用于串口通信
#include "UARTDriver.h"
// 包含SPI设备功能,用于SPI总线通信
#include "SPIDevice.h"
// 包含存储功能,用于数据持久化存储
#include "Storage.h"
// 包含遥控输入功能,用于接收遥控器信号
#include "RCInput.h"
// 包含遥控输出功能,用于控制舵机等执行器
#include "RCOutput.h"
// 包含I2C设备功能,用于I2C总线通信
#include "I2CDevice.h"
// 包含Flash存储器操作功能
#include "Flash.h"
// 包含数字信号处理功能
#include "DSP.h"
// 包含无线SPI设备功能,用于WSPI通信
#include "WSPIDevice.h"
