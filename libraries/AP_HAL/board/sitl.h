#pragma once

// 定义板子名称为SITL(软件在环仿真)
#define HAL_BOARD_NAME "SITL"
// 定义CPU等级为1000MHz
#define HAL_CPU_CLASS HAL_CPU_CLASS_1000
// 定义内存等级为1000
#define HAL_MEM_CLASS HAL_MEM_CLASS_1000
// 启用操作系统套接字
#define HAL_OS_SOCKETS 1

// 设置闪存存储类型为3
#define AP_FLASHSTORAGE_TYPE 3

#if AP_FLASHSTORAGE_TYPE == 1
// 模拟F1/F3闪存
#define HAL_STORAGE_SIZE 15360
#define HAL_FLASH_SECTOR_SIZE 16*1024
#define HAL_FLASH_MIN_WRITE_SIZE 1
#define HAL_FLASH_ALLOW_UPDATE 0

#elif AP_FLASHSTORAGE_TYPE == 2
// 模拟F4/F7闪存
#define HAL_STORAGE_SIZE 15360
#define HAL_FLASH_SECTOR_SIZE 16*1024
#define HAL_FLASH_MIN_WRITE_SIZE 1
#define HAL_FLASH_ALLOW_UPDATE 1

#elif AP_FLASHSTORAGE_TYPE == 3
// 模拟H7闪存
#define HAL_STORAGE_SIZE 16384
#define HAL_FLASH_SECTOR_SIZE 128*1024
#define HAL_FLASH_MIN_WRITE_SIZE 32
#define HAL_FLASH_ALLOW_UPDATE 0
#endif

// 如果未定义板载闪存大小,则设为4096
#ifndef BOARD_FLASH_SIZE
#define BOARD_FLASH_SIZE 4096
#endif

// 如果未定义存储大小,则设为32768
#ifndef HAL_STORAGE_SIZE
#define HAL_STORAGE_SIZE            32768
#endif

// 定义可用存储大小等于总存储大小
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE
// 定义日志目录
#define HAL_BOARD_LOG_DIRECTORY "logs"
// 定义地形数据目录
#define HAL_BOARD_TERRAIN_DIRECTORY "terrain"
// 定义参数默认路径为空
#define HAL_PARAM_DEFAULTS_PATH nullptr
// 设置默认IMU为空
#define HAL_INS_DEFAULT HAL_INS_NONE
// 设置默认气压计为空
#define HAL_BARO_DEFAULT HAL_BARO_NONE

// 默认禁用模拟LED,因为会产生大量SIM_GPIO_MASK消息

// #define AP_NOTIFY_GPIO_LED_RGB_ENABLED 1
// 设置RGB LED引脚(在SIM_PIN_MASK中设置)
#define AP_NOTIFY_GPIO_LED_RGB_RED_PIN    8  
#define AP_NOTIFY_GPIO_LED_RGB_GREEN_PIN  9
#define AP_NOTIFY_GPIO_LED_RGB_BLUE_PIN  10

// #define AP_NOTIFY_GPIO_LED_1_ENABLED 1
// 设置LED A引脚(在SIM_PIN_MASK中设置)
#define AP_NOTIFY_GPIO_LED_A_PIN          8  

// 启用板载电压监测
#define HAL_HAVE_BOARD_VOLTAGE 1
// 启用舵机电压监测
#define HAL_HAVE_SERVO_VOLTAGE 1
// 启用安全开关
#define HAL_HAVE_SAFETY_SWITCH 1

// 仅在编译C++代码时包含
#ifdef __cplusplus
// 允许使用静态信号量
#include <AP_HAL_SITL/Semaphores.h>
#define HAL_Semaphore HALSITL::Semaphore
#define HAL_BinarySemaphore HALSITL::BinarySemaphore
#endif

// 如果未定义CAN接口数量,则设为0
#ifndef HAL_NUM_CAN_IFACES
#define HAL_NUM_CAN_IFACES 0
#endif

// 如果未定义存储目录,则设为当前目录
#ifndef HAL_BOARD_STORAGE_DIRECTORY
#define HAL_BOARD_STORAGE_DIRECTORY "."
#endif

// 如果未定义硬件双精度支持,则启用
#ifndef HAL_HAVE_HARDWARE_DOUBLE
#define HAL_HAVE_HARDWARE_DOUBLE 1
#endif

// EKF双精度支持依赖于硬件双精度支持
#ifndef HAL_WITH_EKF_DOUBLE
#define HAL_WITH_EKF_DOUBLE HAL_HAVE_HARDWARE_DOUBLE
#endif

// 如果未定义默认CAN驱动,则设为0
#ifndef HAL_CAN_DRIVER_DEFAULT
#define HAL_CAN_DRIVER_DEFAULT 0
#endif

// 如果未定义串口监视器,则启用
#ifndef AP_UART_MONITOR_ENABLED
#define AP_UART_MONITOR_ENABLED 1
#endif

// 如果未定义滤波器,则启用
#ifndef AP_FILTER_ENABLED
#define AP_FILTER_ENABLED 1
#endif

// 启用Solo云台功能
#define HAL_SOLO_GIMBAL_ENABLED 1
