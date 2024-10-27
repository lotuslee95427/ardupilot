#pragma once

// 包含必要的头文件
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Filesystem/AP_Filesystem_config.h>
#include <GCS_MAVLink/GCS_config.h>

// 定义是否启用日志记录功能,默认启用
#ifndef HAL_LOGGING_ENABLED
#define HAL_LOGGING_ENABLED 1
#endif

// 定义是否启用默认日志记录后端,默认与HAL_LOGGING_ENABLED相同
#ifndef HAL_LOGGING_BACKEND_DEFAULT_ENABLED
#define HAL_LOGGING_BACKEND_DEFAULT_ENABLED HAL_LOGGING_ENABLED
#endif

// 设置DataFlash日志记录的默认值
// 仅在SITL(软件仿真)模式下启用DataFlash日志记录
#ifndef HAL_LOGGING_DATAFLASH_ENABLED
#define HAL_LOGGING_DATAFLASH_ENABLED HAL_LOGGING_BACKEND_DEFAULT_ENABLED && (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

// 定义是否启用MAVLink日志记录
// 只有在启用GCS(地面站)功能时才启用MAVLink日志记录
#ifndef HAL_LOGGING_MAVLINK_ENABLED
    #define HAL_LOGGING_MAVLINK_ENABLED HAL_LOGGING_BACKEND_DEFAULT_ENABLED && HAL_GCS_ENABLED
#endif

// 定义是否启用文件系统日志记录
// 只有在文件系统支持写入时才启用
#ifndef HAL_LOGGING_FILESYSTEM_ENABLED
#define HAL_LOGGING_FILESYSTEM_ENABLED HAL_LOGGING_BACKEND_DEFAULT_ENABLED && AP_FILESYSTEM_FILE_WRITING_ENABLED
#endif

// 定义是否启用块设备日志记录
// 只有在启用DataFlash日志记录时才启用块设备日志记录
#if HAL_LOGGING_DATAFLASH_ENABLED
    #define HAL_LOGGING_BLOCK_ENABLED HAL_LOGGING_ENABLED
#else
    #define HAL_LOGGING_BLOCK_ENABLED 0
#endif

// 定义是否启用W25NXX Flash存储日志记录
#ifndef HAL_LOGGING_FLASH_W25NXX_ENABLED
#define HAL_LOGGING_FLASH_W25NXX_ENABLED HAL_LOGGING_BLOCK_ENABLED
#endif

// 定义是否启用JEDEC Flash存储日志记录
#ifndef HAL_LOGGING_FLASH_JEDEC_ENABLED
#define HAL_LOGGING_FLASH_JEDEC_ENABLED HAL_LOGGING_BLOCK_ENABLED
#endif

// 文件系统日志记录相关配置
#if HAL_LOGGING_FILESYSTEM_ENABLED

// 检查是否定义了日志目录路径
#if !defined (HAL_BOARD_LOG_DIRECTORY)
#error Need HAL_BOARD_LOG_DIRECTORY for filesystem backend support
#endif

#endif

// 定义是否启用文件内容日志记录
#ifndef HAL_LOGGER_FILE_CONTENTS_ENABLED
#define HAL_LOGGER_FILE_CONTENTS_ENABLED HAL_LOGGING_FILESYSTEM_ENABLED
#endif

// 定义回放时新消息ID的范围
// 在回放过程中添加新消息时,需要避免与现有消息ID冲突
#define REPLAY_LOG_NEW_MSG_MAX 230
#define REPLAY_LOG_NEW_MSG_MIN 220

// 包含围栏配置头文件并定义是否启用围栏日志记录
#include <AC_Fence/AC_Fence_config.h>
#define HAL_LOGGER_FENCE_ENABLED HAL_LOGGING_ENABLED && AP_FENCE_ENABLED

// 包含返航点配置头文件并定义是否启用返航点日志记录
#include <AP_Rally/AP_Rally_config.h>
#define HAL_LOGGER_RALLY_ENABLED HAL_LOGGING_ENABLED && HAL_RALLY_ENABLED
