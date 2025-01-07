/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// 包含避障配置头文件
#include "AC_Avoidance_config.h"

#if AP_OAPATHPLANNER_ENABLED

// 包含所需头文件
#include "AP_OAPathPlanner.h"
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AC_Fence/AC_Fence.h>
#include <AP_Logger/AP_Logger.h>
#include "AP_OABendyRuler.h"
#include "AP_OADijkstra.h"

extern const AP_HAL::HAL &hal;

// 参数默认值
static constexpr float OA_MARGIN_MAX_DEFAULT = 5;  // 最大边距默认值
static constexpr int16_t OA_OPTIONS_DEFAULT = 1;   // 选项默认值

static constexpr int16_t OA_UPDATE_MS = 1000;      // 路径规划以1Hz频率更新
static constexpr int16_t OA_TIMEOUT_MS = 3000;     // 超过3秒的结果将被忽略

// 参数定义
const AP_Param::GroupInfo AP_OAPathPlanner::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Object Avoidance Path Planning algorithm to use
    // @Description: Enabled/disable path planning around obstacles
    // @Values: 0:Disabled,1:BendyRuler,2:Dijkstra,3:Dijkstra with BendyRuler
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE", 1,  AP_OAPathPlanner, _type, OA_PATHPLAN_DISABLED, AP_PARAM_FLAG_ENABLE),

    // Note: Do not use Index "2" for any new parameter
    //       It was being used by _LOOKAHEAD which was later moved to AP_OABendyRuler 

    // @Param: MARGIN_MAX
    // @DisplayName: Object Avoidance wide margin distance
    // @Description: Object Avoidance will ignore objects more than this many meters from vehicle
    // @Units: m
    // @Range: 0.1 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MARGIN_MAX", 3, AP_OAPathPlanner, _margin_max, OA_MARGIN_MAX_DEFAULT),

    // @Group: DB_
    // @Path: AP_OADatabase.cpp
    AP_SUBGROUPINFO(_oadatabase, "DB_", 4, AP_OAPathPlanner, AP_OADatabase),

    // @Param: OPTIONS
    // @DisplayName: Options while recovering from Object Avoidance
    // @Description: Bitmask which will govern vehicles behaviour while recovering from Obstacle Avoidance (i.e Avoidance is turned off after the path ahead is clear).   
    // @Bitmask{Rover}: 0: Reset the origin of the waypoint to the present location, 1: log Dijkstra points
    // @Bitmask{Copter}: 1:log Dijkstra points, 2:Allow fast waypoints (Dijkastras only)
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 5, AP_OAPathPlanner, _options, OA_OPTIONS_DEFAULT),

    // @Group: BR_
    // @Path: AP_OABendyRuler.cpp
    AP_SUBGROUPPTR(_oabendyruler, "BR_", 6, AP_OAPathPlanner, AP_OABendyRuler),

    AP_GROUPEND
};

/// 构造函数
AP_OAPathPlanner::AP_OAPathPlanner()
{
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

// 执行必要的初始化
void AP_OAPathPlanner::init()
{
    // 根据选择的避障类型运行后台任务寻找最佳替代目标点
    switch (_type) {
    case OA_PATHPLAN_DISABLED:
        // 禁用时不执行任何操作
        return;
    case OA_PATHPLAN_BENDYRULER:
        if (_oabendyruler == nullptr) {
            _oabendyruler = NEW_NOTHROW AP_OABendyRuler();
            AP_Param::load_object_from_eeprom(_oabendyruler, AP_OABendyRuler::var_info);
        }
        break;
    case OA_PATHPLAN_DIJKSTRA:
#if AP_FENCE_ENABLED
        if (_oadijkstra == nullptr) {
            _oadijkstra = NEW_NOTHROW AP_OADijkstra(_options);
        }
#endif
        break;
    case OA_PATHPLAN_DJIKSTRA_BENDYRULER:
#if AP_FENCE_ENABLED
        if (_oadijkstra == nullptr) {
            _oadijkstra = NEW_NOTHROW AP_OADijkstra(_options);
        }
#endif
        if (_oabendyruler == nullptr) {
            _oabendyruler = NEW_NOTHROW AP_OABendyRuler();
            AP_Param::load_object_from_eeprom(_oabendyruler, AP_OABendyRuler::var_info);
        }
        break;
    }

    _oadatabase.init();
    start_thread();
}

// 起飞前检查算法是否已成功初始化
bool AP_OAPathPlanner::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    // 检查初始化是否成功
    switch (_type) {
    case OA_PATHPLAN_DISABLED:
        // 禁用时不执行任何操作
        break;
    case OA_PATHPLAN_BENDYRULER:
        if (_oabendyruler == nullptr) {
            hal.util->snprintf(failure_msg, failure_msg_len, "BendyRuler OA requires reboot");
            return false;
        }
        break;
    case OA_PATHPLAN_DIJKSTRA:
        if (_oadijkstra == nullptr) {
            hal.util->snprintf(failure_msg, failure_msg_len, "Dijkstra OA requires reboot");
            return false;
        }
        break;
    case OA_PATHPLAN_DJIKSTRA_BENDYRULER:
        if(_oadijkstra == nullptr || _oabendyruler == nullptr) {
            hal.util->snprintf(failure_msg, failure_msg_len, "OA requires reboot");
            return false;
        }
        break;
    }
    return true;
}

// 启动避障线程
bool AP_OAPathPlanner::start_thread()
{
    WITH_SEMAPHORE(_rsem);

    if (_thread_created) {
        return true;
    }
    if (_type == OA_PATHPLAN_DISABLED) {
        return false;
    }

    // 创建低优先级的避障线程
    // 它应该利用空闲CPU周期根据avoidance_request中的请求填充avoidance_result结构
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_OAPathPlanner::avoidance_thread, void),
                                      "avoidance",
                                      8192, AP_HAL::Scheduler::PRIORITY_IO, -1)) {
        return false;
    }
    _thread_created = true;
    return true;
}

// 辅助函数,将OABendyType映射到OAPathPlannerUsed
AP_OAPathPlanner::OAPathPlannerUsed AP_OAPathPlanner::map_bendytype_to_pathplannerused(AP_OABendyRuler::OABendyType bendy_type)
{
    switch (bendy_type) {
    case AP_OABendyRuler::OABendyType::OA_BENDY_HORIZONTAL:
        return OAPathPlannerUsed::BendyRulerHorizontal;

    case AP_OABendyRuler::OABendyType::OA_BENDY_VERTICAL:
        return OAPathPlannerUsed::BendyRulerVertical;

    default:
    case AP_OABendyRuler::OABendyType::OA_BENDY_DISABLED:
        return OAPathPlannerUsed::None;
    }
}

// 如果需要绕障,提供替代目标位置
// 如果需要中间路径,返回true并更新result_origin、result_destination、result_next_destination
// 如果从result_destination到result_next_destination的路径通畅,则result_dest_to_next_dest_clear设为true(仅Dijkstra支持)
// path_planner_used更新为使用的路径规划器
AP_OAPathPlanner::OA_RetState AP_OAPathPlanner::mission_avoidance(const Location &current_loc,
                                         const Location &origin,
                                         const Location &destination,
                                         const Location &next_destination,
                                         Location &result_origin,
                                         Location &result_destination,
                                         Location &result_next_destination,
                                         bool &result_dest_to_next_dest_clear,
                                         OAPathPlannerUsed &path_planner_used)
{
    // 如果禁用或线程因初始化失败未运行,立即退出
    if (_type == OA_PATHPLAN_DISABLED || !_thread_created) {
        return OA_NOT_REQUIRED;
    }

    // 检查是否刚激活以避免初始超时错误
    const uint32_t now = AP_HAL::millis();
    if (now - _last_update_ms > 200) {
        _activated_ms = now;
    }
    _last_update_ms = now;

    WITH_SEMAPHORE(_rsem);

    // 为线程提供新的请求
    avoidance_request.current_loc = current_loc;
    avoidance_request.origin = origin;
    avoidance_request.destination = destination;
    avoidance_request.next_destination = next_destination;
    avoidance_request.ground_speed_vec = AP::ahrs().groundspeed_vector();
    avoidance_request.request_time_ms = now;

    // 检查结果的destination和next_destination是否与我们的请求匹配
    // 例如检查此结果是否使用我们当前的输入而不是来自旧请求
    const bool destination_matches = destination.same_latlon_as(avoidance_result.destination);
    const bool next_destination_matches = next_destination.same_latlon_as(avoidance_result.next_destination);

    // 检查结果是否超时
    const bool timed_out = (now - avoidance_result.result_time_ms > OA_TIMEOUT_MS) && (now - _activated_ms > OA_TIMEOUT_MS);

    // 返回后台线程最新检查的结果
    if (destination_matches && next_destination_matches && !timed_out) {
        // 我们有来自线程的结果
        result_origin = avoidance_result.origin_new;
        result_destination = avoidance_result.destination_new;
        result_next_destination = avoidance_result.next_destination_new;
        result_dest_to_next_dest_clear = avoidance_result.dest_to_next_dest_clear;
        path_planner_used = avoidance_result.path_planner_used;
        return avoidance_result.ret_state;
    }

    // 如果超时则路径规划器响应时间过长
    if (timed_out) {
        return OA_ERROR;
    }

    // 后台线程正在处理新的目标点
    return OA_PROCESSING;
}

// 避障线程,根据avoidance_request持续更新avoidance_result结构
void AP_OAPathPlanner::avoidance_thread()
{
    // 需要设置ekf原点
    bool origin_set = false;
    while (!origin_set) {
        hal.scheduler->delay(500);
        Location ekf_origin {};
        {
            WITH_SEMAPHORE(AP::ahrs().get_semaphore());
            origin_set = AP::ahrs().get_origin(ekf_origin);    
        }
    }

    while (true) {

        // 如果数据库队列需要处理,更快地处理它
        if (_oadatabase.process_queue()) {
            hal.scheduler->delay(1);
        } else {
            hal.scheduler->delay(20);
        }

        const uint32_t now = AP_HAL::millis();
        if (now - avoidance_latest_ms < OA_UPDATE_MS) {
            continue;
        }
        avoidance_latest_ms = now;

        _oadatabase.update();

        // 路径规划器返回的值
        Location origin_new;
        Location destination_new;
        Location next_destination_new;
        bool dest_to_next_dest_clear = false;
        {
            WITH_SEMAPHORE(_rsem);
            if (now - avoidance_request.request_time_ms > OA_TIMEOUT_MS) {
                // 这是一个很旧的请求,不处理它
                continue;
            }

            // 复制请求以避免与主线程冲突
            avoidance_request2 = avoidance_request;

            // 存储传入的origin、destination和next_destination,如果不需要避障则返回它们
            origin_new = avoidance_request.origin;
            destination_new = avoidance_request.destination;
            next_destination_new = avoidance_request.next_destination;
        }

        // 运行后台任务寻找最佳替代目标点
        OA_RetState res = OA_NOT_REQUIRED;
        OAPathPlannerUsed path_planner_used = OAPathPlannerUsed::None;
        switch (_type) {
        case OA_PATHPLAN_DISABLED:
            continue;
        case OA_PATHPLAN_BENDYRULER: {
            if (_oabendyruler == nullptr) {
                continue;
            }
            _oabendyruler->set_config(_margin_max);

            AP_OABendyRuler::OABendyType bendy_type;
            if (_oabendyruler->update(avoidance_request2.current_loc, avoidance_request2.destination, avoidance_request2.ground_speed_vec, origin_new, destination_new, bendy_type, false)) {
                res = OA_SUCCESS;
            }
            path_planner_used = map_bendytype_to_pathplannerused(bendy_type);
            break;
        }

        case OA_PATHPLAN_DIJKSTRA: {
#if AP_FENCE_ENABLED
            if (_oadijkstra == nullptr) {
                continue;
            }
            _oadijkstra->set_fence_margin(_margin_max);
            const AP_OADijkstra::AP_OADijkstra_State dijkstra_state = _oadijkstra->update(avoidance_request2.current_loc,
                                                                                          avoidance_request2.destination,
                                                                                          avoidance_request2.next_destination,
                                                                                          origin_new,
                                                                                          destination_new,
                                                                                          next_destination_new,
                                                                                          dest_to_next_dest_clear);
            switch (dijkstra_state) {
            case AP_OADijkstra::DIJKSTRA_STATE_NOT_REQUIRED:
                res = OA_NOT_REQUIRED;
                break;
            case AP_OADijkstra::DIJKSTRA_STATE_ERROR:
                res = OA_ERROR;
                break;
            case AP_OADijkstra::DIJKSTRA_STATE_SUCCESS:
                res = OA_SUCCESS;
                break;
            }
            path_planner_used = OAPathPlannerUsed::Dijkstras;
#endif
            break;
        }

        case OA_PATHPLAN_DJIKSTRA_BENDYRULER: {
            if ((_oabendyruler == nullptr) || _oadijkstra == nullptr) {
                continue;
            } 
            _oabendyruler->set_config(_margin_max);
            AP_OABendyRuler::OABendyType bendy_type;
            if (_oabendyruler->update(avoidance_request2.current_loc, avoidance_request2.destination, avoidance_request2.ground_speed_vec, origin_new, destination_new, bendy_type, proximity_only)) {
                // 通过车辆的近距离传感器检测到障碍物。切换到BendyRuler避障直到障碍物离开
                proximity_only = false;
                res = OA_SUCCESS;
                path_planner_used = map_bendytype_to_pathplannerused(bendy_type);
                break;
            } else {
                // 清除所有障碍物,触发Dijkstra基于当前偏离位置计算路径
#if AP_FENCE_ENABLED
                if (proximity_only == false) {
                    _oadijkstra->recalculate_path();
                }
#endif
                // 现在BendyRuler只使用近距离避障
                proximity_only = true;
            }
#if AP_FENCE_ENABLED
            _oadijkstra->set_fence_margin(_margin_max);
            const AP_OADijkstra::AP_OADijkstra_State dijkstra_state = _oadijkstra->update(avoidance_request2.current_loc,
                                                                                          avoidance_request2.destination,
                                                                                          avoidance_request2.next_destination,
                                                                                          origin_new,
                                                                                          destination_new,
                                                                                          next_destination_new,
                                                                                          dest_to_next_dest_clear);
            switch (dijkstra_state) {
            case AP_OADijkstra::DIJKSTRA_STATE_NOT_REQUIRED:
                res = OA_NOT_REQUIRED;
                break;
            case AP_OADijkstra::DIJKSTRA_STATE_ERROR:
                res = OA_ERROR;
                break;
            case AP_OADijkstra::DIJKSTRA_STATE_SUCCESS:
                res = OA_SUCCESS;
                break;
            }
            path_planner_used = OAPathPlannerUsed::Dijkstras;
#endif
            break;
        }

        } // switch

        {
            // 将避障结果提供给主线程
            WITH_SEMAPHORE(_rsem);

            // 将使用的destination和next_destination放入结果(由调用者用于验证结果是否匹配其请求)
            avoidance_result.destination = avoidance_request2.destination;
            avoidance_result.next_destination = avoidance_request2.next_destination;
            avoidance_result.dest_to_next_dest_clear = dest_to_next_dest_clear;

            // 用中间路径填充结果结构
            avoidance_result.origin_new = (res == OA_SUCCESS) ? origin_new : avoidance_result.origin_new;
            avoidance_result.destination_new = (res == OA_SUCCESS) ? destination_new : avoidance_result.destination;
            avoidance_result.next_destination_new = (res == OA_SUCCESS) ? next_destination_new : avoidance_result.next_destination;

            // 创建新的avoidance_result.dest_to_next_dest_clear字段。用dijkstra的结果填充或保持未知
            avoidance_result.result_time_ms = AP_HAL::millis();
            avoidance_result.path_planner_used = path_planner_used;
            avoidance_result.ret_state = res;
        }
    }
}

// 单例实例
AP_OAPathPlanner *AP_OAPathPlanner::_singleton;

namespace AP {

AP_OAPathPlanner *ap_oapathplanner()
{
    return AP_OAPathPlanner::get_singleton();
}

}

#endif  // AP_OAPATHPLANNER_ENABLED
