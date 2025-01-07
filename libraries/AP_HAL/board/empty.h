#pragma once  // 确保头文件只被包含一次

// 定义空板的名称
#define HAL_BOARD_NAME "EMPTY"

// 定义CPU等级为150MHz
#define HAL_CPU_CLASS HAL_CPU_CLASS_150

// 定义内存等级为192KB
#define HAL_MEM_CLASS HAL_MEM_CLASS_192

// 如果存储大小未定义,则设置默认值
#ifndef HAL_STORAGE_SIZE
#define HAL_STORAGE_SIZE            16384  // 设置存储大小为16KB
#endif

// 定义可用存储大小等于总存储大小
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE

// 设置默认惯性导航系统(INS)为空
#define HAL_INS_DEFAULT HAL_INS_NONE

// 设置默认气压计为空
#define HAL_BARO_DEFAULT HAL_BARO_NONE

// 设置HAL板子子类型为空
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_NONE

// 板载电压监测相关配置
#define HAL_HAVE_BOARD_VOLTAGE 1    // 启用板载电压监测
#define HAL_HAVE_SERVO_VOLTAGE 0    // 禁用舵机电压监测
#define HAL_HAVE_SAFETY_SWITCH 1    // 启用安全开关

// 定义信号量类型
#define HAL_Semaphore Empty::Semaphore          // 定义普通信号量
#define HAL_BinarySemaphore Empty::BinarySemaphore  // 定义二值信号量
