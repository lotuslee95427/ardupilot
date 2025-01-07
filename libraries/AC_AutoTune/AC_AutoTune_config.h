// 防止头文件重复包含
#pragma once

// 包含HAL板卡定义头文件
#include <AP_HAL/AP_HAL_Boards.h>

// 如果未定义自动调参功能开关,则默认开启
#ifndef AC_AUTOTUNE_ENABLED
#define AC_AUTOTUNE_ENABLED 1
#endif
