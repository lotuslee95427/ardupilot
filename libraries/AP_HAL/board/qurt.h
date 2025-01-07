#pragma once

// 包含QURT HAL主头文件
#include <AP_HAL_QURT/AP_HAL_QURT_Main.h>

// 定义板子基本信息
#define HAL_BOARD_NAME "QURT"  // 定义板子名称
#define HAL_CPU_CLASS HAL_CPU_CLASS_1000  // 定义CPU等级为1000MHz
#define HAL_MEM_CLASS HAL_MEM_CLASS_1000  // 定义内存等级为1000
#define HAL_STORAGE_SIZE            32768  // 定义存储大小为32KB
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE  // 定义可用存储大小

// 仅在编译C++代码时包含
#ifdef __cplusplus
#include <AP_HAL_QURT/Semaphores.h>
#define HAL_Semaphore QURT::Semaphore  // 定义信号量类型
#define HAL_BinarySemaphore QURT::BinarySemaphore  // 定义二值信号量类型
#endif

// 硬件双精度浮点支持配置
#ifndef HAL_HAVE_HARDWARE_DOUBLE
#define HAL_HAVE_HARDWARE_DOUBLE 0  // 默认禁用硬件双精度浮点
#endif

// EKF双精度支持配置
#ifndef HAL_WITH_EKF_DOUBLE
#define HAL_WITH_EKF_DOUBLE HAL_HAVE_HARDWARE_DOUBLE  // EKF双精度支持依赖于硬件双精度支持
#endif

// 陀螺仪FFT功能配置
#ifndef HAL_GYROFFT_ENABLED
#define HAL_GYROFFT_ENABLED 0  // 默认禁用陀螺仪FFT
#endif

/*
  为初始移植禁用一些功能
 */
#define HAL_HAVE_BOARD_VOLTAGE 0  // 禁用板载电压监测
#define HAL_HAVE_SERVO_VOLTAGE 0  // 禁用舵机电压监测
#define HAL_HAVE_SAFETY_SWITCH 0  // 禁用安全开关
#define HAL_WITH_MCU_MONITORING 0  // 禁用MCU监控
#define HAL_USE_QUADSPI 0  // 禁用QUADSPI
#define HAL_WITH_DSP 0  // 禁用DSP

// CAN总线相关配置
#define HAL_CANFD_SUPPORTED 0  // 不支持CANFD
#define HAL_NUM_CAN_IFACES 0  // CAN接口数量为0

// 系统功能配置
#define AP_CRASHDUMP_ENABLED 0  // 禁用崩溃转储
#define HAL_ENABLE_DFU_BOOT 0  // 禁用DFU引导

// 日志系统配置
#define HAL_LOGGING_MAVLINK_ENABLED 0  // 禁用MAVLink日志
#define HAL_LOGGING_FILESYSTEM_ENABLED 1  // 启用文件系统日志

// 文件系统功能配置
#define AP_FILESYSTEM_POSIX_HAVE_UTIME 0  // 禁用utime支持
#define AP_FILESYSTEM_POSIX_HAVE_FSYNC 0  // 禁用fsync支持
#define AP_FILESYSTEM_POSIX_HAVE_STATFS 0  // 禁用statfs支持
#define AP_FILESYSTEM_HAVE_DIRENT_DTYPE 0  // 禁用dirent dtype支持

// 文件系统路径配置
#define AP_FILESYSTEM_POSIX_MAP_FILENAME_ALLOC 1
#define AP_FILESYSTEM_POSIX_MAP_FILENAME_BASEDIR "/data"  // 基础目录
#define HAL_BOARD_STORAGE_DIRECTORY "APM"  // 存储目录
#define HAL_BOARD_LOG_DIRECTORY "APM/logs"  // 日志目录
#define HAL_BOARD_TERRAIN_DIRECTORY "APM/terrain"  // 地形数据目录

#define SCRIPTING_DIRECTORY "APM/scripts"  // 脚本目录

/*
  该车辆的默认参数文件
 */
#ifndef HAL_PARAM_DEFAULTS_PATH
// 这是AP_Param要求的绝对路径
#define HAL_PARAM_DEFAULTS_PATH AP_FILESYSTEM_POSIX_MAP_FILENAME_BASEDIR "/APM/defaults.parm"
#endif

#define USE_LIBC_REALLOC 1  // 使用libc的realloc

#define HAL_WITH_ESC_TELEM 1  // 启用电调遥测

/*
  电池监测设置,通过ESC获取
 */
#define HAL_BATT_VOLT_PIN 1  // 电压检测引脚
#define HAL_BATT_CURR_PIN 2  // 电流检测引脚
#define HAL_BATT_MONITOR_DEFAULT 4  // 默认电池监测类型
#define HAL_BATT_VOLT_SCALE 1  // 电压比例
#define HAL_BATT_CURR_SCALE 1  // 电流比例

#define HAL_PROBE_EXTERNAL_I2C_COMPASSES  // 启用外部I2C罗盘探测

/*
  罗盘列表
 */
#define PROBE_MAG_I2C(driver, bus, addr, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe(GET_I2C_DEVICE(bus, addr),##args))
#define HAL_MAG_PROBE_LIST PROBE_MAG_I2C(QMC5883L, 0, 0x0d, true, ROTATION_NONE)

/*
  气压计列表
 */
#define PROBE_BARO_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_Baro_ ## driver::probe(*this,std::move(GET_I2C_DEVICE(bus, addr)),##args))
#define HAL_BARO_PROBE_LIST PROBE_BARO_I2C(ICP101XX, 2, 0x63)

/*
  IMU列表
 */
#define PROBE_IMU_SPI(driver, devname, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname),##args))
#define HAL_INS_PROBE_LIST PROBE_IMU_SPI(Invensensev3, "INV3", ROTATION_NONE)

/*
  引入缺失的标准库函数
 */
#include <AP_HAL_QURT/replace.h>

#define DEFAULT_SERIAL4_PROTOCOL 23 // RC输入协议
