#pragma once

// 根据不同的ESP32板子类型包含相应的头文件
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_DIY
#include "esp32diy.h" // Charles
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_BUZZ
#include "esp32buzz.h" //Buzz
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_ICARUS
#include "esp32icarus.h" //Alex
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_EMPTY
#include "esp32empty.h" //wiktor-m
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_TOMTE76
#include "esp32tomte76.h" //tomte76 on discord
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_NICK
#include "esp32nick.h" //Nick K. on discord
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_S3DEVKIT
#include "esp32s3devkit.h" //Nick K. on discord
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_S3EMPTY
#include "esp32s3empty.h"
#else
#error "Invalid CONFIG_HAL_BOARD_SUBTYPE for esp32"
#endif

// 定义板子基本参数
#define HAL_BOARD_NAME "ESP32"  // 定义板子名称
#define HAL_CPU_CLASS HAL_CPU_CLASS_150  // 定义CPU等级为150MHz
#define HAL_WITH_DRONECAN 0  // 禁用DRONECAN功能
#define HAL_WITH_UAVCAN 0    // 禁用UAVCAN功能
#define HAL_MAX_CAN_PROTOCOL_DRIVERS 0  // 设置CAN协议驱动数量为0
#define HAL_HAVE_SAFETY_SWITCH 0  // 禁用安全开关
#define HAL_HAVE_BOARD_VOLTAGE 0  // 禁用板载电压监测
#define HAL_HAVE_SERVO_VOLTAGE 0  // 禁用舵机电压监测

#define HAL_WITH_IO_MCU 0  // 禁用IO协处理器

#define O_CLOEXEC 0  // 设置文件描述符标志
#define HAL_STORAGE_SIZE (16384)  // 设置存储大小为16KB

#ifdef __cplusplus
// allow for static semaphores
// 允许使用静态信号量
#include <AP_HAL_ESP32/Semaphores.h>
#define HAL_Semaphore ESP32::Semaphore
#define HAL_BinarySemaphore ESP32::BinarySemaphore
#endif

#define HAL_NUM_CAN_IFACES 0  // 设置CAN接口数量为0
#define HAL_MEM_CLASS HAL_MEM_CLASS_192  // 定义内存等级

// disable uncommon stuff that we'd otherwise get 
// 禁用一些不常用的功能
#define HAL_EXTERNAL_AHRS_ENABLED 0  // 禁用外部AHRS
#define HAL_GENERATOR_ENABLED 0      // 禁用发电机功能

// 定义字节序
#define __LITTLE_ENDIAN  1234
#define __BYTE_ORDER     __LITTLE_ENDIAN

// whenver u get ... error: "xxxxxxx" is not defined, evaluates to 0 [-Werror=undef]  just define it below as 0
// 当遇到未定义错误时，将相关宏定义为0
#define CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY 0
#define XCHAL_ERRATUM_453 0
#define CONFIG_FREERTOS_CHECK_STACKOVERFLOW_NONE 0
#define CONFIG_FREERTOS_CHECK_STACKOVERFLOW_PTRVAL 0
#define CONFIG_FREERTOS_ENABLE_STATIC_TASK_CLEAN_UP 0
#define CONFIG_FREERTOS_USE_TICKLESS_IDLE 0
#define CONFIG_SYSVIEW_ENABLE 0
#define CONFIG_SPI_FLASH_DANGEROUS_WRITE_ALLOWED 0
#define CONFIG_SPI_FLASH_ENABLE_COUNTERS 0
#define USE_LIBC_REALLOC 0
#define CONFIG_LWIP_DHCP_RESTORE_LAST_IP 0
#define CONFIG_LWIP_STATS 0
#define CONFIG_LWIP_PPP_SUPPORT 0
#define CONFIG_LWIP_STATS 0
#define CONFIG_NEWLIB_NANO_FORMAT 0
#define CONFIG_LWIP_IP4_REASSEMBLY 0
#define CONFIG_LWIP_IP6_REASSEMBLY 0
#define CONFIG_LWIP_STATS 0
#define LWIP_COMPAT_SOCKET_INET 0
#define LWIP_COMPAT_SOCKET_ADDR 0

// 默认禁用所有指南针
#ifndef AP_COMPASS_BACKEND_DEFAULT_ENABLED
#define AP_COMPASS_BACKEND_DEFAULT_ENABLED 0
#endif

// 将舵机通道数量限制为16
#define NUM_SERVO_CHANNELS 16

// 默认禁用陀螺仪温度校准
#define HAL_INS_TEMPERATURE_CAL_ENABLE 0

// 禁用一系列高级飞行器调度功能
#define AP_ADVANCEDFAILSAFE_ENABLED 0  // 禁用高级失效保护
#define AP_ICENGINE_ENABLED 0          // 禁用内燃机功能
#define AP_OPTICALFLOW_ENABLED 0       // 禁用光流功能
#define AP_RPM_ENABLED 0               // 禁用RPM传感器
#define AP_AIRSPEED_AUTOCAL_ENABLE 0   // 禁用空速自动校准
#define HAL_MOUNT_ENABLED 0            // 禁用云台功能
#define AP_CAMERA_ENABLED 0            // 禁用相机功能
#define HAL_SOARING_ENABLED 0          // 禁用滑翔功能
#define AP_TERRAIN_AVAILABLE 0         // 禁用地形功能
#define HAL_ADSB_ENABLED 0             // 禁用ADS-B功能
#define HAL_BUTTON_ENABLED 0           // 禁用按钮功能
#define AP_GRIPPER_ENABLED 0           // 禁用抓取器功能
#define AP_LANDINGGEAR_ENABLED 0       // 禁用起落架功能

// 在copter中禁用避障-围栏-跟随功能，这些功能相互依赖
#define AP_AVOIDANCE_ENABLED 0         // 禁用避障功能
#define AP_FENCE_ENABLED 0             // 禁用地理围栏
#define MODE_FOLLOW_ENABLED 0          // 禁用跟随模式
#define AP_OAPATHPLANNER_ENABLED 0     // 禁用障碍物避让路径规划

// 禁用其他大型功能
#define HAL_QUADPLANE_ENABLED 0        // 禁用四旋翼飞机功能
#define HAL_GYROFFT_ENABLED 0          // 禁用陀螺仪FFT功能

// 由于ESP32性能较慢，增加调度器超时余量
#define AP_SCHEDULER_OVERTIME_MARGIN_US 50000UL
