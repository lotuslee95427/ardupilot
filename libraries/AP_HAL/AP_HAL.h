#pragma once  // 确保头文件只被包含一次

#include <stdint.h>  // 包含标准整数类型定义

// 包含HAL基础头文件
#include "AP_HAL_Namespace.h"  // HAL命名空间定义
#include "AP_HAL_Boards.h"     // HAL支持的主板定义
#include "AP_HAL_Macros.h"     // HAL宏定义
#include "AP_HAL_Main.h"       // HAL主程序入口点定义

/* HAL模块类(全部为纯虚类) */
#include "UARTDriver.h"    // 串口驱动
#include "AnalogIn.h"      // 模拟输入
#include "Storage.h"       // 存储接口
#include "GPIO.h"         // 通用输入输出接口
#include "RCInput.h"      // 遥控输入接口
#include "RCOutput.h"     // 遥控输出接口
#include "Scheduler.h"    // 任务调度器
#include "Semaphores.h"   // 信号量
#include "Util.h"         // 工具类
#include "OpticalFlow.h"  // 光流传感器接口
#include "Flash.h"        // Flash存储接口
#include "DSP.h"         // 数字信号处理接口

#include "CANIface.h"     // CAN总线接口

#include "utility/BetterStream.h"  // 增强的数据流类

/* HAL类定义 */
#include "HAL.h"          // HAL抽象层主类

#include "system.h"       // 系统相关定义
