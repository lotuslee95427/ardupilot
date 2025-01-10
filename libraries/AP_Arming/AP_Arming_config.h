// 防止头文件重复包含
#pragma once

// 包含HAL板级配置头文件
#include <AP_HAL/AP_HAL_Boards.h>

// 如果未定义AP_ARMING_ENABLED,则默认启用解锁功能
#ifndef AP_ARMING_ENABLED
#define AP_ARMING_ENABLED 1
#endif

// 如果未定义AP_ARMING_AUX_AUTH_ENABLED,则根据脚本功能是否启用来决定是否启用辅助认证
#ifndef AP_ARMING_AUX_AUTH_ENABLED
#define AP_ARMING_AUX_AUTH_ENABLED AP_SCRIPTING_ENABLED
#endif

// 如果未定义AP_ARMING_CRASHDUMP_ACK_ENABLED,则根据崩溃转储功能是否启用来决定是否启用崩溃确认
#ifndef AP_ARMING_CRASHDUMP_ACK_ENABLED
#define AP_ARMING_CRASHDUMP_ACK_ENABLED AP_CRASHDUMP_ENABLED
#endif
