/**
 * C preprocessor enumeration of the boards supported by the AP_HAL.
 * This list exists so HAL_BOARD == HAL_BOARD_xxx preprocessor blocks
 * can be used to exclude HAL boards from the build when appropriate.
 * It's not an elegant solution but we can improve it in future.
 * 
 * AP_HAL支持的主板的C预处理器枚举。
 * 这个列表的存在使得可以使用HAL_BOARD == HAL_BOARD_xxx预处理器块
 * 在适当的时候从构建中排除HAL主板。
 * 这不是一个优雅的解决方案，但我们将来可以改进它。
 */
#pragma once

// 定义不同的主板类型
#define HAL_BOARD_SITL     3    // SITL模拟器
#define HAL_BOARD_SMACCM   4    // 未使用
#define HAL_BOARD_PX4      5    // 未使用
#define HAL_BOARD_LINUX    7    // Linux主板
#define HAL_BOARD_VRBRAIN  8    // VRBrain主板
#define HAL_BOARD_CHIBIOS  10   // ChibiOS主板
#define HAL_BOARD_F4LIGHT  11   // 保留
#define HAL_BOARD_ESP32	   12   // ESP32主板
#define HAL_BOARD_QURT     13   // QURT主板
#define HAL_BOARD_EMPTY    99   // 空主板

/* 默认主板子类型为-1 */
#define HAL_BOARD_SUBTYPE_NONE -1

/* HAL Linux子类型，从1000开始 */
#define HAL_BOARD_SUBTYPE_LINUX_NONE       1000  // Linux无特定子类型
#define HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD  1001  // ErleBoard
#define HAL_BOARD_SUBTYPE_LINUX_PXF        1002  // PXF
#define HAL_BOARD_SUBTYPE_LINUX_NAVIO      1003  // Navio
#define HAL_BOARD_SUBTYPE_LINUX_ZYNQ       1004  // Zynq
#define HAL_BOARD_SUBTYPE_LINUX_BBBMINI    1005  // BeagleBone Black Mini
#define HAL_BOARD_SUBTYPE_LINUX_BEBOP      1006  // Bebop
#define HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2 1009  // ErleBrain2
#define HAL_BOARD_SUBTYPE_LINUX_BH         1010  // BH
#define HAL_BOARD_SUBTYPE_LINUX_PXFMINI    1012  // PXFmini
#define HAL_BOARD_SUBTYPE_LINUX_NAVIO2     1013  // Navio2
#define HAL_BOARD_SUBTYPE_LINUX_DISCO      1014  // Disco
#define HAL_BOARD_SUBTYPE_LINUX_AERO       1015  // Aero
#define HAL_BOARD_SUBTYPE_LINUX_DARK       1016  // Dark
#define HAL_BOARD_SUBTYPE_LINUX_BLUE       1018  // Blue
#define HAL_BOARD_SUBTYPE_LINUX_OCPOC_ZYNQ 1019  // OCPOC Zynq
#define HAL_BOARD_SUBTYPE_LINUX_EDGE       1020  // Edge
#define HAL_BOARD_SUBTYPE_LINUX_RST_ZYNQ   1021  // RST Zynq
#define HAL_BOARD_SUBTYPE_LINUX_POCKET     1022  // Pocket
#define HAL_BOARD_SUBTYPE_LINUX_NAVIGATOR  1023  // Navigator
#define HAL_BOARD_SUBTYPE_LINUX_VNAV       1024  // VNAV
#define HAL_BOARD_SUBTYPE_LINUX_OBAL_V1    1025  // OBAL V1
#define HAL_BOARD_SUBTYPE_LINUX_CANZERO    1026  // CanZero

/* HAL CHIBIOS子类型，从5000开始

   注意!! 除非真的需要，否则不要添加更多子类型。大多数
   主板不需要定义子类型。只有当我们需要使用
   #ifdef代码来改变行为时才需要
*/
#define HAL_BOARD_SUBTYPE_CHIBIOS_SKYVIPER_F412	5000  // SkyViper F412
#define HAL_BOARD_SUBTYPE_CHIBIOS_FMUV3         5001  // FMUv3
// #define HAL_BOARD_SUBTYPE_CHIBIOS_FMUV4         5002  // FMUv4
#define HAL_BOARD_SUBTYPE_CHIBIOS_GENERIC       5009  // 通用ChibiOS
#define HAL_BOARD_SUBTYPE_CHIBIOS_FMUV5         5013  // FMUv5
// #define HAL_BOARD_SUBTYPE_CHIBIOS_VRBRAIN_V51   5016  // VRBrain v5.1
// #define HAL_BOARD_SUBTYPE_CHIBIOS_VRBRAIN_V52   5017  // VRBrain v5.2
// #define HAL_BOARD_SUBTYPE_CHIBIOS_VRUBRAIN_V51  5018  // VRUBrain v5.1
// #define HAL_BOARD_SUBTYPE_CHIBIOS_VRCORE_V10    5019  // VRCore v1.0
// #define HAL_BOARD_SUBTYPE_CHIBIOS_VRBRAIN_V54   5020  // VRBrain v5.4

/* ESP32子类型 */
#define HAL_BOARD_SUBTYPE_ESP32_DIY             6001  // DIY ESP32
#define HAL_BOARD_SUBTYPE_ESP32_ICARUS          6002  // Icarus
#define HAL_BOARD_SUBTYPE_ESP32_BUZZ            6003  // Buzz
#define HAL_BOARD_SUBTYPE_ESP32_EMPTY           6004  // Empty
#define HAL_BOARD_SUBTYPE_ESP32_TOMTE76         6005  // Tomte76
#define HAL_BOARD_SUBTYPE_ESP32_NICK            6006  // Nick
#define HAL_BOARD_SUBTYPE_ESP32_S3DEVKIT        6007  // S3 DevKit
#define HAL_BOARD_SUBTYPE_ESP32_S3EMPTY         6008  // S3 Empty

/* 惯性传感器驱动类型 */
#define HAL_INS_NONE         0    // 无惯性传感器
#define HAL_INS_MPU60XX_SPI  2    // MPU60XX SPI接口
#define HAL_INS_MPU60XX_I2C  3    // MPU60XX I2C接口
#define HAL_INS_HIL_UNUSED   4    // 未使用
#define HAL_INS_VRBRAIN      8    // VRBrain
#define HAL_INS_MPU9250_SPI  9    // MPU9250 SPI接口
#define HAL_INS_MPU9250_I2C 13    // MPU9250 I2C接口
#define HAL_INS_MPU6500     19    // MPU6500
#define HAL_INS_INV2_I2C    24    // INV2 I2C接口
#define HAL_INS_INV2_SPI    25    // INV2 SPI接口

/* 气压计驱动类型 */
#define HAL_BARO_NONE        0    // 无气压计
#define HAL_BARO_HIL_UNUSED  6    // 未使用
#define HAL_BARO_20789_I2C_I2C  14  // 20789 I2C-I2C
#define HAL_BARO_20789_I2C_SPI  15  // 20789 I2C-SPI
#define HAL_BARO_LPS25H_IMU_I2C 17  // LPS25H IMU I2C

/* 加热类型 */
#define HAL_LINUX_HEAT_PWM 1      // Linux PWM加热

/* CPU类别，用于选择是否应该使用CPU密集型算法
 * 注意这些只是近似值，不是确切的CPU速度。 */

/* 150Mhz: STM32F4或类似。假设:
 *  - 硬件浮点
 *  - 数十KB可用内存
*/
#define HAL_CPU_CLASS_150  3      // 150MHz级CPU

/* GigaHz级: SITL, BeagleBone等。假设有MB级可用内存。 */
#define HAL_CPU_CLASS_1000 4      // 1GHz级CPU

/*
  内存类别，单位为KB。主板必须至少有给定数量的内存
*/
#define HAL_MEM_CLASS_20   1      // 20KB内存
#define HAL_MEM_CLASS_64   2      // 64KB内存
#define HAL_MEM_CLASS_192  3      // 192KB内存
#define HAL_MEM_CLASS_300  4      // 300KB内存
#define HAL_MEM_CLASS_500  5      // 500KB内存
#define HAL_MEM_CLASS_1000 6      // 1000KB内存

/* 操作系统特性
 *
 * HAL实现可以将以下额外的特性定义为1（如果可用）:
 *
 * - HAL_OS_POSIX_IO : 具有类POSIX的文件系统IO
 * - HAL_OS_SOCKETS  : 具有类POSIX的套接字 */

/* 主板定义 */

// 根据配置的主板类型包含相应的头文件
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    #include <AP_HAL/board/sitl.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    #include <AP_HAL/board/linux.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_EMPTY
    #include <AP_HAL/board/empty.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    #include <AP_HAL/board/vrbrain.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
	#include <AP_HAL/board/chibios.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    #include <AP_HAL/board/esp32.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_QURT
    #include <AP_HAL/board/qurt.h>
#else
#error "Unknown CONFIG_HAL_BOARD type"
#endif

// 检查是否设置了主板子类型
#ifndef CONFIG_HAL_BOARD_SUBTYPE
#error "No CONFIG_HAL_BOARD_SUBTYPE set"
#endif

// 设置默认值
#ifndef HAL_OS_SOCKETS
#define HAL_OS_SOCKETS 0
#endif

#ifndef HAL_PARAM_DEFAULTS_PATH
#define HAL_PARAM_DEFAULTS_PATH nullptr
#endif

#ifndef HAL_HAVE_IMU_HEATER
#define HAL_HAVE_IMU_HEATER 0
#endif

#ifndef HAL_NUM_CAN_IFACES
#define HAL_NUM_CAN_IFACES 0
#endif

#ifndef HAL_WITH_IO_MCU
#define HAL_WITH_IO_MCU 0
#endif

#ifndef HAL_WITH_IO_MCU_BIDIR_DSHOT
#define HAL_WITH_IO_MCU_BIDIR_DSHOT 0
#endif

#ifndef HAL_WITH_IO_MCU_DSHOT
#define HAL_WITH_IO_MCU_DSHOT HAL_WITH_IO_MCU_BIDIR_DSHOT
#endif

#ifndef HAL_REQUIRES_BDSHOT_SUPPORT
#define HAL_REQUIRES_BDSHOT_SUPPORT (defined(HAL_WITH_BIDIR_DSHOT) || HAL_WITH_IO_MCU_BIDIR_DSHOT)
#endif

// 仅当可能接收到此类遥测时才启用Extended DShot Telemetry v2支持
// 可以来自伺服输出或IOMCU

// 如果不需要，设置为0 - 如果IOMCU启用了双向DShot，也需要重新编译它
// 否则与IOMCU的通信会中断！
#ifndef AP_EXTENDED_DSHOT_TELEM_V2_ENABLED
#define AP_EXTENDED_DSHOT_TELEM_V2_ENABLED HAL_REQUIRES_BDSHOT_SUPPORT
#endif

#ifndef BOARD_FLASH_SIZE
#define BOARD_FLASH_SIZE 2048
#endif

#ifndef HAL_GYROFFT_ENABLED
#define HAL_GYROFFT_ENABLED (BOARD_FLASH_SIZE > 1024)
#endif

// 仅在需要时启用AP_GyroFFT库:
#ifndef HAL_WITH_DSP
#define HAL_WITH_DSP HAL_GYROFFT_ENABLED
#endif

#ifndef HAL_OS_FATFS_IO
#define HAL_OS_FATFS_IO 0
#endif

#ifndef HAL_BARO_DEFAULT
#define HAL_BARO_DEFAULT HAL_BARO_NONE
#endif

#ifndef HAL_INS_DEFAULT
#define HAL_INS_DEFAULT HAL_INS_NONE
#endif

#ifndef HAL_GPS1_TYPE_DEFAULT
#define HAL_GPS1_TYPE_DEFAULT 1
#endif

#ifndef HAL_CAN_DRIVER_DEFAULT
#define HAL_CAN_DRIVER_DEFAULT 0
#endif

#ifndef HAL_MAX_CAN_PROTOCOL_DRIVERS
    #define HAL_MAX_CAN_PROTOCOL_DRIVERS HAL_NUM_CAN_IFACES
#endif

#ifndef HAL_CANMANAGER_ENABLED
#define HAL_CANMANAGER_ENABLED (HAL_MAX_CAN_PROTOCOL_DRIVERS > 0)
#endif

#ifndef HAL_ENABLE_DRONECAN_DRIVERS
#define HAL_ENABLE_DRONECAN_DRIVERS HAL_CANMANAGER_ENABLED
#endif

#ifndef AP_TEST_DRONECAN_DRIVERS
#define AP_TEST_DRONECAN_DRIVERS 0
#endif

#ifdef HAVE_LIBDL
#define AP_MODULE_SUPPORTED 1
#else
#define AP_MODULE_SUPPORTED 0
#endif

#ifndef HAL_SUPPORT_RCOUT_SERIAL
#define HAL_SUPPORT_RCOUT_SERIAL 0
#endif

#ifndef HAL_FORWARD_OTG2_SERIAL
#define HAL_FORWARD_OTG2_SERIAL 0
#endif

#ifndef HAL_HAVE_DUAL_USB_CDC
#define HAL_HAVE_DUAL_USB_CDC 0
#endif

#ifndef AP_CAN_SLCAN_ENABLED
#if HAL_NUM_CAN_IFACES && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#define AP_CAN_SLCAN_ENABLED 1
#else
#define AP_CAN_SLCAN_ENABLED 0
#endif
#endif

#ifndef USE_LIBC_REALLOC
#define USE_LIBC_REALLOC 1
#endif

#ifndef AP_HAL_SHARED_DMA_ENABLED
#define AP_HAL_SHARED_DMA_ENABLED 1
#endif

#ifndef HAL_ENABLE_THREAD_STATISTICS
#define HAL_ENABLE_THREAD_STATISTICS 0
#endif

#ifndef AP_STATS_ENABLED
#define AP_STATS_ENABLED 1
#endif

#ifndef HAL_WITH_MCU_MONITORING
#define HAL_WITH_MCU_MONITORING 0
#endif

#ifndef AP_CRASHDUMP_ENABLED
#define AP_CRASHDUMP_ENABLED 0
#endif

#ifndef AP_SIGNED_FIRMWARE
#define AP_SIGNED_FIRMWARE 0
#endif

#ifndef HAL_DSHOT_ALARM_ENABLED
#define HAL_DSHOT_ALARM_ENABLED 0
#endif

#ifndef HAL_DSHOT_ENABLED
#define HAL_DSHOT_ENABLED 1
#endif

#ifndef HAL_SERIALLED_ENABLED
#define HAL_SERIALLED_ENABLED HAL_DSHOT_ENABLED
#endif

#ifndef HAL_SERIAL_ESC_COMM_ENABLED
#define HAL_SERIAL_ESC_COMM_ENABLED 1
#endif

#ifndef AP_BOOTLOADER_FLASHING_ENABLED
#define AP_BOOTLOADER_FLASHING_ENABLED 0
#endif

#ifndef HAL_HNF_MAX_FILTERS
// 在F7上，1个陷波和24个陷波之间的CPU负载差异约为2%
// 1Khz后端和2Khz后端之间的CPU负载差异约为10%
// 因此在1Khz时，F7和H7几乎可以支持所有陷波组合
#if defined(STM32H7) || CONFIG_HAL_BOARD == HAL_BOARD_SITL
// 足够用于八轴使用三个IMU和一个谐波的每个电机双陷波
// 加上一个带一个双陷波谐波的静态陷波
#define HAL_HNF_MAX_FILTERS 54
#elif defined(STM32F7)
// 足够用于八轴使用三个IMU和一个谐波的每个电机陷波
// 加上一个带一个谐波的静态陷波
#define HAL_HNF_MAX_FILTERS 27
#else
// 足够用于八轴四轴使用两个IMU和一个谐波的每个电机陷波
// 加上一个带一个谐波的静态陷波
// 或者在一个IMU上每个电机三重陷波加一个谐波
#define HAL_HNF_MAX_FILTERS 24
#endif
#endif // HAL_HNF_MAX_FILTERS

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL // 允许SITL有所有CANFD选项
#define HAL_CANFD_SUPPORTED 8
#elif !defined(HAL_CANFD_SUPPORTED)
#define HAL_CANFD_SUPPORTED 0
#endif

#ifndef HAL_USE_QUADSPI
#define HAL_USE_QUADSPI 0
#endif

#ifndef HAL_USE_OCTOSPI
#define HAL_USE_OCTOSPI 0
#endif

#ifndef __RAMFUNC__
#define __RAMFUNC__
#endif

#ifndef __FASTRAMFUNC__
#define __FASTRAMFUNC__
#endif

#ifndef __EXTFLASHFUNC__
#define __EXTFLASHFUNC__
#endif

#ifndef HAL_ENABLE_DFU_BOOT
#define HAL_ENABLE_DFU_BOOT 0
#endif

#ifndef HAL_ENABLE_SENDING_STATS
#define HAL_ENABLE_SENDING_STATS BOARD_FLASH_SIZE >= 256
#endif

#ifndef HAL_GPIO_LED_ON
#define HAL_GPIO_LED_ON 0
#elif HAL_GPIO_LED_ON == 0
#error "Do not specify HAL_GPIO_LED_ON if you are setting it to the default, 0"
#endif

#ifdef HAL_GPIO_LED_OFF
#error "HAL_GPIO_LED_OFF must not be defined, it is implicitly !HAL_GPIO_LED_ON"
#endif

#ifndef HAL_WITH_POSTYPE_DOUBLE
#define HAL_WITH_POSTYPE_DOUBLE BOARD_FLASH_SIZE > 1024
#endif

#define HAL_GPIO_LED_OFF (!HAL_GPIO_LED_ON)
