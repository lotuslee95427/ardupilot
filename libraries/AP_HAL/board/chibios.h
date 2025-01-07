#pragma once

#include <hwdef.h>

// 定义开发板名称
#define HAL_BOARD_NAME "ChibiOS"

// 检查是否使用了过时的LED定义
#ifdef HAL_HAVE_PIXRACER_LED
#error "use AP_NOTIFY_GPIO_LED_RGB_ENABLED in place of HAL_HAVE_PIXRACER_LED (and rename your pins!)"
#endif

// 根据总内存大小定义内存类别
#if HAL_MEMORY_TOTAL_KB >= 1000
#define HAL_MEM_CLASS HAL_MEM_CLASS_1000  // 内存大于等于1000KB
#elif HAL_MEMORY_TOTAL_KB >= 500
#define HAL_MEM_CLASS HAL_MEM_CLASS_500   // 内存大于等于500KB
#elif HAL_MEMORY_TOTAL_KB >= 300
#define HAL_MEM_CLASS HAL_MEM_CLASS_300   // 内存大于等于300KB
#elif HAL_MEMORY_TOTAL_KB >= 192
#define HAL_MEM_CLASS HAL_MEM_CLASS_192   // 内存大于等于192KB
#elif HAL_MEMORY_TOTAL_KB >= 64
#define HAL_MEM_CLASS HAL_MEM_CLASS_64    // 内存大于等于64KB
#else
#define HAL_MEM_CLASS HAL_MEM_CLASS_20    // 内存小于64KB
#endif

// 定义CAN接口数量，默认为0
#ifndef HAL_NUM_CAN_IFACES
#define HAL_NUM_CAN_IFACES 0
#endif

// 定义是否有板载电压监测功能，默认为0
#ifndef HAL_HAVE_BOARD_VOLTAGE
#define HAL_HAVE_BOARD_VOLTAGE 0
#endif

// 定义是否有舵机电压监测功能，默认为0
#ifndef HAL_HAVE_SERVO_VOLTAGE
#define HAL_HAVE_SERVO_VOLTAGE 0
#endif

// 如果定义了安全开关引脚，则启用安全开关功能
#ifdef HAL_GPIO_PIN_SAFETY_IN
#define HAL_HAVE_SAFETY_SWITCH 1
#endif

// 定义是否有安全开关，默认为0
#ifndef HAL_HAVE_SAFETY_SWITCH
#define HAL_HAVE_SAFETY_SWITCH 0
#endif

// 定义可用存储空间大小
#define HAL_STORAGE_SIZE_AVAILABLE HAL_STORAGE_SIZE

// 定义是否使用IO协处理器，默认为0
#ifndef HAL_WITH_IO_MCU
#define HAL_WITH_IO_MCU 0
#endif

// 定义是否使用RAMTRON存储器，默认为0
#ifndef HAL_WITH_RAMTRON
#define HAL_WITH_RAMTRON 0
#endif

// 定义是否使用双精度EKF，默认使用硬件双精度能力
#ifndef HAL_WITH_EKF_DOUBLE
#define HAL_WITH_EKF_DOUBLE HAL_HAVE_HARDWARE_DOUBLE
#endif

// C++代码部分，定义信号量类型
#ifdef __cplusplus
// allow for static semaphores
#include <AP_HAL_ChibiOS/Semaphores.h>
#define HAL_Semaphore ChibiOS::Semaphore
#define HAL_BinarySemaphore ChibiOS::BinarySemaphore
#endif

/* string names for well known SPI devices */
// 定义各种SPI设备的名称
#define HAL_BARO_MS5611_NAME "ms5611"
#ifndef HAL_BARO_MS5611_SPI_INT_NAME
#define HAL_BARO_MS5611_SPI_INT_NAME "ms5611_int"
#endif
#define HAL_BARO_MS5611_SPI_EXT_NAME "ms5611_ext"
#define HAL_BARO_LPS22H_NAME "lps22h"
#define HAL_BARO_BMP280_NAME "bmp280"

// 定义IMU传感器的名称
#define HAL_INS_MPU60x0_NAME "mpu6000"
#define HAL_INS_MPU60x0_EXT_NAME "mpu6000_ext"

#define HAL_INS_LSM9DS0_G_NAME "lsm9ds0_g"
#define HAL_INS_LSM9DS0_A_NAME "lsm9ds0_am"

#define HAL_INS_LSM9DS0_EXT_G_NAME "lsm9ds0_ext_g"
#define HAL_INS_LSM9DS0_EXT_A_NAME "lsm9ds0_ext_am"

#define HAL_INS_MPU9250_NAME "mpu9250"
#define HAL_INS_MPU9250_EXT_NAME "mpu9250_ext"

#define HAL_INS_MPU6500_NAME "mpu6500"

#define HAL_INS_ICM20608_NAME "icm20608"
#define HAL_INS_ICM20608_AM_NAME "icm20608-am"
#define HAL_INS_ICM20608_EXT_NAME "icm20608_ext"

// 定义罗盘传感器的名称
#define HAL_COMPASS_HMC5843_NAME "hmc5843"
#define HAL_COMPASS_LIS3MDL_NAME "lis3mdl"

// 允许在hwdef.dat中覆盖短名称
#ifndef CHIBIOS_SHORT_BOARD_NAME
#define CHIBIOS_SHORT_BOARD_NAME CHIBIOS_BOARD_NAME
#endif

// 如果未定义板子子类型，使用通用类型
#ifndef CONFIG_HAL_BOARD_SUBTYPE
// allow for generic boards
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_CHIBIOS_GENERIC
#endif

// 支持BLHeli穿透的RC串口功能
#ifndef HAL_SUPPORT_RCOUT_SERIAL
#define HAL_SUPPORT_RCOUT_SERIAL 1
#endif

// 默认第一个I2C总线为内部总线
#ifndef HAL_I2C_INTERNAL_MASK
#define HAL_I2C_INTERNAL_MASK 1
#endif

// 将所有文件存储在/APM目录下
#ifndef HAL_BOARD_STORAGE_DIRECTORY
#define HAL_BOARD_STORAGE_DIRECTORY "/APM"
#endif

// 定义QUADSPI和OCTOSPI接口的使用状态
#if defined(STM32_WSPI_USE_QUADSPI1) && STM32_WSPI_USE_QUADSPI1
#define HAL_USE_QUADSPI1 TRUE
#else
#define HAL_USE_QUADSPI1 FALSE
#endif
#if defined(STM32_WSPI_USE_QUADSPI2) && STM32_WSPI_USE_QUADSPI2
#define HAL_USE_QUADSPI2 TRUE
#else
#define HAL_USE_QUADSPI2 FALSE
#endif
#if defined(STM32_WSPI_USE_OCTOSPI1) && STM32_WSPI_USE_OCTOSPI1
#define HAL_USE_OCTOSPI1 TRUE
#else
#define HAL_USE_OCTOSPI1 FALSE
#endif
#if defined(STM32_WSPI_USE_OCTOSPI2) && STM32_WSPI_USE_OCTOSPI2
#define HAL_USE_OCTOSPI2 TRUE
#else
#define HAL_USE_OCTOSPI2 FALSE
#endif
#define HAL_USE_QUADSPI (HAL_USE_QUADSPI1 || HAL_USE_QUADSPI2)
#define HAL_USE_OCTOSPI (HAL_USE_OCTOSPI1 || HAL_USE_OCTOSPI2)
