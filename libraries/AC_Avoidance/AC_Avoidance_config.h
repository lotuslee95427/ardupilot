#pragma once

// 包含HAL板卡定义头文件
#include <AP_HAL/AP_HAL_Boards.h>
// 包含围栏配置头文件
#include <AC_Fence/AC_Fence_config.h>

// 如果未定义避障功能开关,则设置为与围栏功能相同的开关状态
#ifndef AP_AVOIDANCE_ENABLED
#define AP_AVOIDANCE_ENABLED AP_FENCE_ENABLED
#endif

// 如果未定义障碍物避障路径规划器开关,则设置为与围栏功能相同的开关状态
#ifndef AP_OAPATHPLANNER_ENABLED
#define AP_OAPATHPLANNER_ENABLED AP_FENCE_ENABLED
#endif

// 如果未定义障碍物避障路径规划器后端默认开关,则设置为与路径规划器相同的开关状态
#ifndef AP_OAPATHPLANNER_BACKEND_DEFAULT_ENABLED
#define AP_OAPATHPLANNER_BACKEND_DEFAULT_ENABLED AP_OAPATHPLANNER_ENABLED
#endif

// 如果未定义BendyRuler避障算法开关,则设置为与路径规划器后端默认相同的开关状态
#ifndef AP_OAPATHPLANNER_BENDYRULER_ENABLED
#define AP_OAPATHPLANNER_BENDYRULER_ENABLED AP_OAPATHPLANNER_BACKEND_DEFAULT_ENABLED
#endif

// 如果未定义Dijkstra避障算法开关,则设置为与路径规划器后端默认相同的开关状态
#ifndef AP_OAPATHPLANNER_DIJKSTRA_ENABLED
#define AP_OAPATHPLANNER_DIJKSTRA_ENABLED AP_OAPATHPLANNER_BACKEND_DEFAULT_ENABLED
#endif



// 如果未定义障碍物避障数据库开关,则设置为与路径规划器相同的开关状态
#ifndef AP_OADATABASE_ENABLED
#define AP_OADATABASE_ENABLED AP_OAPATHPLANNER_ENABLED
#endif
