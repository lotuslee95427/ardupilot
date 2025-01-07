#pragma once

#include "AC_Avoidance_config.h"

#if AP_OAPATHPLANNER_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Param/AP_Param.h>
#include <AP_HAL/Semaphores.h>

#include "AP_OABendyRuler.h"
#include "AP_OADijkstra.h"
#include "AP_OADatabase.h"

/*
 * This class provides path planning around fence, stay-out zones and moving obstacles
 * 该类提供了围栏、禁区和移动障碍物周围的路径规划
 */
class AP_OAPathPlanner {

public:
    AP_OAPathPlanner();

    /* Do not allow copies */
    // 不允许拷贝
    CLASS_NO_COPY(AP_OAPathPlanner);

    // get singleton instance
    // 获取单例实例
    static AP_OAPathPlanner *get_singleton() {
        return _singleton;
    }

    // perform any required initialisation
    // 执行所需的初始化
    void init();

    /// returns true if all pre-takeoff checks have completed successfully
    /// 如果所有起飞前检查都成功完成则返回true
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const;

    // object avoidance processing return status enum
    // 障碍物避障处理返回状态枚举
    enum OA_RetState : uint8_t {
        OA_NOT_REQUIRED = 0,            // object avoidance is not required 不需要避障
        OA_PROCESSING,                  // still calculating alternative path 仍在计算替代路径
        OA_ERROR,                       // error during calculation 计算过程中出错
        OA_SUCCESS                      // success 成功
    };

    // path planner responsible for a particular result
    // 负责特定结果的路径规划器
    enum OAPathPlannerUsed : uint8_t {
        None = 0,                   // 无
        BendyRulerHorizontal,      // 水平弯曲标尺
        BendyRulerVertical,        // 垂直弯曲标尺
        Dijkstras                  // Dijkstra算法
    };

    // provides an alternative target location if path planning around obstacles is required
    // returns true and updates result_origin, result_destination and result_next_destination with an intermediate path
    // result_dest_to_next_dest_clear is set to true if the path from result_destination to result_next_destination is clear  (only supported by Dijkstras)
    // path_planner_used updated with which path planner produced the result
    // 如果需要绕过障碍物进行路径规划,则提供替代目标位置
    // 返回true并使用中间路径更新result_origin、result_destination和result_next_destination
    // 如果从result_destination到result_next_destination的路径畅通,则将result_dest_to_next_dest_clear设置为true(仅Dijkstra支持)
    // path_planner_used更新为产生结果的路径规划器
    OA_RetState mission_avoidance(const Location &current_loc,
                           const Location &origin,
                           const Location &destination,
                           const Location &next_destination,
                           Location &result_origin,
                           Location &result_destination,
                           Location &result_next_destination,
                           bool &result_dest_to_next_dest_clear,
                           OAPathPlannerUsed &path_planner_used) WARN_IF_UNUSED;

    // enumerations for _TYPE parameter
    // _TYPE参数的枚举
    enum OAPathPlanTypes {
        OA_PATHPLAN_DISABLED = 0,               // 禁用
        OA_PATHPLAN_BENDYRULER = 1,            // 弯曲标尺
        OA_PATHPLAN_DIJKSTRA = 2,              // Dijkstra算法
        OA_PATHPLAN_DJIKSTRA_BENDYRULER = 3,   // Dijkstra和弯曲标尺结合
    };

    // enumeration for _OPTION parameter
    // _OPTION参数的枚举
    enum OARecoveryOptions {
        OA_OPTION_DISABLED = 0,                     // 禁用
        OA_OPTION_WP_RESET = (1 << 0),             // 航点重置
        OA_OPTION_LOG_DIJKSTRA_POINTS = (1 << 1),  // 记录Dijkstra点
        OA_OPTION_FAST_WAYPOINTS = (1 << 2),       // 快速航点
    };

    uint16_t get_options() const { return _options;}

    static const struct AP_Param::GroupInfo var_info[];

private:

    // avoidance thread that continually updates the avoidance_result structure based on avoidance_request
    // 基于避障请求持续更新避障结果结构的避障线程
    void avoidance_thread();
    bool start_thread();

    // helper function to map OABendyType to OAPathPlannerUsed
    // 将OABendyType映射到OAPathPlannerUsed的辅助函数
    OAPathPlannerUsed map_bendytype_to_pathplannerused(AP_OABendyRuler::OABendyType bendy_type);

    // an avoidance request from the navigation code
    // 来自导航代码的避障请求
    struct avoidance_info {
        Location current_loc;           // 当前位置
        Location origin;                // 起点
        Location destination;           // 目标点
        Location next_destination;      // 下一个目标点
        Vector2f ground_speed_vec;      // 地速向量
        uint32_t request_time_ms;       // 请求时间(毫秒)
    } avoidance_request, avoidance_request2;

    // an avoidance result from the avoidance thread
    // 来自避障线程的避障结果
    struct {
        Location destination;       // destination vehicle is trying to get to (also used to verify the result matches a recent request)
                                  // 车辆试图到达的目标点(也用于验证结果是否匹配最近的请求)
        Location next_destination;  // next destination vehicle is trying to get to (also used to verify the result matches a recent request)
                                  // 车辆试图到达的下一个目标点(也用于验证结果是否匹配最近的请求)
        Location origin_new;        // intermediate origin.  The start of line segment that vehicle should follow
                                  // 中间起点。车辆应该遵循的线段的起点
        Location destination_new;   // intermediate destination vehicle should move towards
                                  // 车辆应该移动到的中间目标点
        Location next_destination_new; // intermediate next destination vehicle should move towards
                                     // 车辆应该移动到的中间下一个目标点
        bool dest_to_next_dest_clear; // true if the path from destination_new to next_destination_new is clear and does not require path planning  (only supported by Dijkstras)
                                     // 如果从destination_new到next_destination_new的路径畅通且不需要路径规划,则为true(仅Dijkstra支持)
        uint32_t result_time_ms;    // system time the result was calculated (used to verify the result is recent)
                                   // 计算结果的系统时间(用于验证结果是最近的)
        OAPathPlannerUsed path_planner_used;    // path planner that produced the result
                                               // 产生结果的路径规划器
        OA_RetState ret_state;      // OA_SUCCESS if the vehicle should move along the path from origin_new to destination_new
                                   // 如果车辆应该沿着从origin_new到destination_new的路径移动,则为OA_SUCCESS
    } avoidance_result;

    // parameters
    // 参数
    AP_Int8 _type;                  // avoidance algorithm to be used 要使用的避障算法
    AP_Float _margin_max;           // object avoidance will ignore objects more than this many meters from vehicle 避障将忽略距离车辆超过这么多米的物体
    AP_Int16 _options;              // Bitmask for options while recovering from Object Avoidance 从避障恢复时的选项位掩码
    
    // internal variables used by front end
    // 前端使用的内部变量
    HAL_Semaphore _rsem;            // semaphore for multi-thread use of avoidance_request and avoidance_result 用于多线程使用avoidance_request和avoidance_result的信号量
    bool _thread_created;           // true once background thread has been created 后台线程创建后为true
    AP_OABendyRuler *_oabendyruler; // Bendy Ruler algorithm 弯曲标尺算法
    AP_OADijkstra *_oadijkstra;     // Dijkstra's algorithm Dijkstra算法
    AP_OADatabase _oadatabase;      // Database of dynamic objects to avoid 要避开的动态物体数据库
    uint32_t avoidance_latest_ms;   // last time Dijkstra's or BendyRuler algorithms ran (in the avoidance thread) Dijkstra或BendyRuler算法最后运行时间(在避障线程中)
    uint32_t _last_update_ms;       // system time that mission_avoidance was called in main thread 在主线程中调用mission_avoidance的系统时间
    uint32_t _activated_ms;         // system time that object avoidance was most recently activated (used to avoid timeout error on first run) 最近激活避障的系统时间(用于避免首次运行时的超时错误)

    bool proximity_only = true;     // 仅使用近距离避障
    static AP_OAPathPlanner *_singleton;  // 单例指针
};

namespace AP {
    AP_OAPathPlanner *ap_oapathplanner();
};

#endif  // AP_OAPATHPLANNER_ENABLED
