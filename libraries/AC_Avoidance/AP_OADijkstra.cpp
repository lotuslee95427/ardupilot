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

// 包含配置头文件
#include "AC_Avoidance_config.h"

#if AP_OAPATHPLANNER_DIJKSTRA_ENABLED

// 包含所需头文件
#include "AP_OADijkstra.h"
#include "AP_OAPathPlanner.h"

#include <AC_Fence/AC_Fence.h>

#if AP_FENCE_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

// 定义Dijkstra算法相关常量
#define OA_DIJKSTRA_EXPANDING_ARRAY_ELEMENTS_PER_CHUNK  32      // 扩展数组每次增长的元素数量
#define OA_DIJKSTRA_POLYGON_SHORTPATH_NOTSET_IDX        255     // 表示节点没有临时最短路径的索引值
#define OA_DIJKSTRA_ERROR_REPORTING_INTERVAL_MS         5000    // 错误消息发送到GCS的时间间隔(毫秒)

/// 构造函数
AP_OADijkstra::AP_OADijkstra(AP_Int16 &options) :
        _inclusion_polygon_pts(OA_DIJKSTRA_EXPANDING_ARRAY_ELEMENTS_PER_CHUNK),
        _exclusion_polygon_pts(OA_DIJKSTRA_EXPANDING_ARRAY_ELEMENTS_PER_CHUNK),
        _exclusion_circle_pts(OA_DIJKSTRA_EXPANDING_ARRAY_ELEMENTS_PER_CHUNK),
        _short_path_data(OA_DIJKSTRA_EXPANDING_ARRAY_ELEMENTS_PER_CHUNK),
        _path(OA_DIJKSTRA_EXPANDING_ARRAY_ELEMENTS_PER_CHUNK),
        _options(options)
{
}

// 计算避开围栏的目标点
// 如果需要避障,返回DIJKSTRA_STATE_SUCCESS并更新origin_new、destination_new和next_destination_new
// 如果有下一个目标点,next_destination_new将不为零
// dest_to_next_dest_clear将设置为true,如果从(输入的)destination到(输入的)next_destination的路径是通畅的
AP_OADijkstra::AP_OADijkstra_State AP_OADijkstra::update(const Location &current_loc,
                                                         const Location &destination,
                                                         const Location &next_destination,
                                                         Location& origin_new,
                                                         Location& destination_new,
                                                         Location& next_destination_new,
                                                         bool& dest_to_next_dest_clear)
{
    WITH_SEMAPHORE(AP::fence()->polyfence().get_loaded_fence_semaphore());

    // 如果没有围栏则不需要避障
    if (!some_fences_enabled()) {
        dest_to_next_dest_clear = _dest_to_next_dest_clear = true;
        Write_OADijkstra(DIJKSTRA_STATE_NOT_REQUIRED, 0, 0, 0, destination, destination);
        return DIJKSTRA_STATE_NOT_REQUIRED;
    }

    // 如果目标点与当前位置相同则不需要避障
    if (current_loc.same_latlon_as(destination)) {
        // 不检查到下一个目标点的路径,保守地设置为false
        dest_to_next_dest_clear = _dest_to_next_dest_clear = false;
        Write_OADijkstra(DIJKSTRA_STATE_NOT_REQUIRED, 0, 0, 0, destination, destination);
        return DIJKSTRA_STATE_NOT_REQUIRED;
    }

    // 检查包含多边形是否更新
    if (check_inclusion_polygon_updated()) {
        _inclusion_polygon_with_margin_ok = false;
        _polyfence_visgraph_ok = false;
        _shortest_path_ok = false;
    }

    // 检查排除多边形是否更新
    if (check_exclusion_polygon_updated()) {
        _exclusion_polygon_with_margin_ok = false;
        _polyfence_visgraph_ok = false;
        _shortest_path_ok = false;
    }

    // 检查排除圆形是否更新
    if (check_exclusion_circle_updated()) {
        _exclusion_circle_with_margin_ok = false;
        _polyfence_visgraph_ok = false;
        _shortest_path_ok = false;
    }

    // 创建内部多边形围栏
    if (!_inclusion_polygon_with_margin_ok) {
        _inclusion_polygon_with_margin_ok = create_inclusion_polygon_with_margin(_polyfence_margin * 100.0f, _error_id);
        if (!_inclusion_polygon_with_margin_ok) {
            dest_to_next_dest_clear = _dest_to_next_dest_clear = false;
            report_error(_error_id);
            Write_OADijkstra(DIJKSTRA_STATE_ERROR, (uint8_t)_error_id, 0, 0, destination, destination);
            return DIJKSTRA_STATE_ERROR;
        }
    }

    // 创建排除多边形外部围栏
    if (!_exclusion_polygon_with_margin_ok) {
        _exclusion_polygon_with_margin_ok = create_exclusion_polygon_with_margin(_polyfence_margin * 100.0f, _error_id);
        if (!_exclusion_polygon_with_margin_ok) {
            dest_to_next_dest_clear = _dest_to_next_dest_clear = false;
            report_error(_error_id);
            Write_OADijkstra(DIJKSTRA_STATE_ERROR, (uint8_t)_error_id, 0, 0, destination, destination);
            return DIJKSTRA_STATE_ERROR;
        }
    }

    // 创建排除圆形点
    if (!_exclusion_circle_with_margin_ok) {
        _exclusion_circle_with_margin_ok = create_exclusion_circle_with_margin(_polyfence_margin * 100.0f, _error_id);
        if (!_exclusion_circle_with_margin_ok) {
            dest_to_next_dest_clear = _dest_to_next_dest_clear = false;
            report_error(_error_id);
            Write_OADijkstra(DIJKSTRA_STATE_ERROR, (uint8_t)_error_id, 0, 0, destination, destination);
            return DIJKSTRA_STATE_ERROR;
        }
    }

    // 为所有围栏(带边距)点创建可见性图
    if (!_polyfence_visgraph_ok) {
        _polyfence_visgraph_ok = create_fence_visgraph(_error_id);
        if (!_polyfence_visgraph_ok) {
            _shortest_path_ok = false;
            dest_to_next_dest_clear = _dest_to_next_dest_clear = false;
            report_error(_error_id);
            Write_OADijkstra(DIJKSTRA_STATE_ERROR, (uint8_t)_error_id, 0, 0, destination, destination);
            return DIJKSTRA_STATE_ERROR;
        }
        // 重置日志计数以重新开始记录更新的图
        _log_num_points = 0;
        _log_visgraph_version++;
    }

    // 每次循环记录一个可见性图点
    if (_polyfence_visgraph_ok && (_log_num_points < total_numpoints()) && (_options & AP_OAPathPlanner::OARecoveryOptions::OA_OPTION_LOG_DIJKSTRA_POINTS) ) {
        Vector2f vis_point;
        if (get_point(_log_num_points, vis_point)) {
            Location log_location(Vector3f{vis_point.x, vis_point.y, 0.0}, Location::AltFrame::ABOVE_ORIGIN);
            Write_Visgraph_point(_log_visgraph_version, _log_num_points, log_location.lat, log_location.lng);
            _log_num_points++;
        }
    }

    // 如果目标点或下一个目标点发生变化则重建路径
    if (!destination.same_latlon_as(_destination_prev) || !next_destination.same_latlon_as(_next_destination_prev)) {
        _destination_prev = destination;
        _next_destination_prev = next_destination;
        _shortest_path_ok = false;
    }

    // 计算从当前位置到目标点的最短路径
    if (!_shortest_path_ok) {
        _shortest_path_ok = calc_shortest_path(current_loc, destination, _error_id);
        if (!_shortest_path_ok) {
            dest_to_next_dest_clear = _dest_to_next_dest_clear = false;
            report_error(_error_id);
            Write_OADijkstra(DIJKSTRA_STATE_ERROR, (uint8_t)_error_id, 0, 0, destination, destination);
            return DIJKSTRA_STATE_ERROR;
        }
        // 从路径的第2个点开始(第1个是原始起点)
        _path_idx_returned = 1;

        // 检查从目标点到下一个目标点的路径是否与围栏相交
        _dest_to_next_dest_clear = false;
        if (!next_destination.is_zero()) {
            Vector2f seg_start, seg_end;
            if (destination.get_vector_xy_from_origin_NE(seg_start) && next_destination.get_vector_xy_from_origin_NE(seg_end)) {
                _dest_to_next_dest_clear = !intersects_fence(seg_start, seg_end);
            }
        }
    }

    // 路径已创建,返回最新点
    Vector2f dest_pos;
    const uint8_t path_length = get_shortest_path_numpoints() > 0 ? (get_shortest_path_numpoints() - 1) : 0;
    if ((_path_idx_returned < path_length) && get_shortest_path_point(_path_idx_returned, dest_pos)) {

        // 对于第一个点,返回origin作为current_loc
        Vector2f origin_pos;
        if ((_path_idx_returned > 0) && get_shortest_path_point(_path_idx_returned-1, origin_pos)) {
            // 将相对于ekf原点的偏移转换为Location
            Location temp_loc(Vector3f{origin_pos.x, origin_pos.y, 0.0}, Location::AltFrame::ABOVE_ORIGIN);
            origin_new = temp_loc;
        } else {
            // 对于第一个点使用当前位置作为origin
            origin_new = current_loc;
        }

        // 将相对于ekf原点的偏移转换为Location
        Location temp_loc(Vector3f{dest_pos.x, dest_pos.y, 0.0}, Location::AltFrame::ABOVE_ORIGIN);
        destination_new = destination;
        destination_new.lat = temp_loc.lat;
        destination_new.lng = temp_loc.lng;

        // 提供下一个目标点以实现平滑转弯
        next_destination_new.zero();
        Vector2f next_dest_pos;
        if ((_path_idx_returned + 1 < path_length) && get_shortest_path_point(_path_idx_returned + 1, next_dest_pos)) {
            // 将相对于ekf原点的偏移转换为Location
            Location next_loc(Vector3f{next_dest_pos.x, next_dest_pos.y, 0.0}, Location::AltFrame::ABOVE_ORIGIN);
            next_destination_new = destination;
            next_destination_new.lat = next_loc.lat;
            next_destination_new.lng = next_loc.lng;
        } else {
            // 返回目标点作为下一个目标点
            next_destination_new = destination;
        }

        // 到下一个目标点的路径通畅状态仍然有效(与最短路径一起计算)
        dest_to_next_dest_clear = _dest_to_next_dest_clear;

        // 检查是否应该前进到下一个点
        const bool near_oa_wp = current_loc.get_distance(destination_new) <= 2.0f;
        const bool past_oa_wp = current_loc.past_interval_finish_line(origin_new, destination_new);
        if (near_oa_wp || past_oa_wp) {
            _path_idx_returned++;
        }
        // 记录成功
        Write_OADijkstra(DIJKSTRA_STATE_SUCCESS, 0, _path_idx_returned, get_shortest_path_numpoints(), destination, destination_new);
        return DIJKSTRA_STATE_SUCCESS;
    }

    // 已到达目标点,不再需要避障
    // 到下一个目标点的路径通畅状态仍然有效
    dest_to_next_dest_clear = _dest_to_next_dest_clear;
    Write_OADijkstra(DIJKSTRA_STATE_NOT_REQUIRED, 0, 0, 0, destination, destination);
    return DIJKSTRA_STATE_NOT_REQUIRED;
}

// 返回是否至少启用了一个包含或排除区域
bool AP_OADijkstra::some_fences_enabled() const
{
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return false;
    }
    if ((fence->polyfence().get_inclusion_polygon_count() == 0) &&
        (fence->polyfence().get_exclusion_polygon_count() == 0) &&
        (fence->polyfence().get_exclusion_circle_count() == 0)) {
        return false;
    }
    return ((fence->get_enabled_fences() & AC_FENCE_TYPE_POLYGON) > 0);
}

// 返回给定错误ID的错误消息
const char* AP_OADijkstra::get_error_msg(AP_OADijkstra_Error error_id) const
{
    switch (error_id) {
    case AP_OADijkstra_Error::DIJKSTRA_ERROR_NONE:
        return "no error";
        break;
    case AP_OADijkstra_Error::DIJKSTRA_ERROR_OUT_OF_MEMORY:
        return "out of memory";
        break;
    case AP_OADijkstra_Error::DIJKSTRA_ERROR_OVERLAPPING_POLYGON_POINTS:
        return "overlapping polygon points";
        break;
    case AP_OADijkstra_Error::DIJKSTRA_ERROR_FAILED_TO_BUILD_INNER_POLYGON:
        return "failed to build inner polygon";
        break;
    case AP_OADijkstra_Error::DIJKSTRA_ERROR_OVERLAPPING_POLYGON_LINES:
        return "overlapping polygon lines";
        break;
    case AP_OADijkstra_Error::DIJKSTRA_ERROR_FENCE_DISABLED:
        return "fence disabled";
        break;
    case AP_OADijkstra_Error::DIJKSTRA_ERROR_TOO_MANY_FENCE_POINTS:
        return "too many fence points";
        break;
    case AP_OADijkstra_Error::DIJKSTRA_ERROR_NO_POSITION_ESTIMATE:
        return "no position estimate";
        break;
    case AP_OADijkstra_Error::DIJKSTRA_ERROR_COULD_NOT_FIND_PATH:
        return "could not find path";
        break;
    }

    // 不应该到达这里,以防万一
    return "unknown error";
}

// 向GCS报告错误
void AP_OADijkstra::report_error(AP_OADijkstra_Error error_id)
{
    // 每5秒向GCS报告错误
    uint32_t now_ms = AP_HAL::millis();
    if ((error_id != AP_OADijkstra_Error::DIJKSTRA_ERROR_NONE) &&
        ((error_id != _error_last_id) || ((now_ms - _error_last_report_ms) > OA_DIJKSTRA_ERROR_REPORTING_INTERVAL_MS))) {
        const char* error_msg = get_error_msg(error_id);
        (void)error_msg;  // 如果!HAL_GCS_ENABLED
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Dijkstra: %s", error_msg);
        _error_last_id = error_id;
        _error_last_report_ms = now_ms;
    }
}

// 检查多边形围栏是否在创建内部围栏后更新。如果更改则返回true
bool AP_OADijkstra::check_inclusion_polygon_updated() const
{
    // 如果多边形围栏未启用则立即退出
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return false;
    }
    return (_inclusion_polygon_update_ms != fence->polyfence().get_inclusion_polygon_update_ms());
}

// 在现有包含多边形内创建多边形
// 成功时返回true。失败时返回false并更新err_id
bool AP_OADijkstra::create_inclusion_polygon_with_margin(float margin_cm, AP_OADijkstra_Error &err_id)
{
    const AC_Fence *fence = AC_Fence::get_singleton();

    if (fence == nullptr) {
        err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_FENCE_DISABLED;
        return false;
    }

    // 如果之前的围栏点未更改,则跳过不必要的重试以构建包含多边形
    if (_inclusion_polygon_update_ms == fence->polyfence().get_inclusion_polygon_update_ms()) {
        return false;
    }

    _inclusion_polygon_update_ms = fence->polyfence().get_inclusion_polygon_update_ms();

    // 清除所有点
    _inclusion_polygon_numpoints = 0;

    // 如果没有多边形则立即返回
    const uint8_t num_inclusion_polygons = fence->polyfence().get_inclusion_polygon_count();

    // 遍历多边形并创建内部点
    for (uint8_t i = 0; i < num_inclusion_polygons; i++) {
        uint16_t num_points;
        const Vector2f* boundary = fence->polyfence().get_inclusion_polygon(i, num_points);

        // 对于包含多边形中的每个点
        // 注意:boundary是"未闭合的",意味着最后一个点*不*与第一个点相同
        uint16_t new_points = 0;
        for (uint16_t j = 0; j < num_points; j++) {

            // 找到当前点之前和之后的点(相对于当前点)
            const uint16_t before_idx = (j == 0) ? num_points-1 : j-1;
            const uint16_t after_idx = (j == num_points-1) ? 0 : j+1;
            Vector2f before_pt = boundary[before_idx] - boundary[j];
            Vector2f after_pt = boundary[after_idx] - boundary[j];

            // 如果点重叠则失败
            if (before_pt.is_zero() || after_pt.is_zero() || (before_pt == after_pt)) {
                err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_OVERLAPPING_POLYGON_POINTS;
                return false;
            }

            // 将点缩放为单位向量
            before_pt.normalize();
            after_pt.normalize();

            // 计算中间点并缩放到边距
            Vector2f intermediate_pt = after_pt + before_pt;
            intermediate_pt.normalize();
            intermediate_pt *= margin_cm;

            // 找到在内部多边形外的最终点
            Vector2f temp_point = boundary[j] + intermediate_pt;
            if (Polygon_outside(temp_point, boundary, num_points)) {
                intermediate_pt *= -1.0;
                temp_point = boundary[j] + intermediate_pt;
                if (Polygon_outside(temp_point, boundary, num_points)) {
                    // 在任一侧都找不到在排除多边形外的点,所以失败
                    // 如果排除多边形有重叠的线可能会发生这种情况
                    err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_OVERLAPPING_POLYGON_LINES;
                    return false;
                }
            }

            // 不要在角落添加点
            if (fabsf(intermediate_pt.angle() - before_pt.angle()) < M_PI_2) {
                continue;
            }

            // 如果需要则扩展数组
            if (!_inclusion_polygon_pts.expand_to_hold(_inclusion_polygon_numpoints + new_points + 1)) {
                err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_OUT_OF_MEMORY;
                return false;
            }
            // 添加点
            _inclusion_polygon_pts[_inclusion_polygon_numpoints + new_points] = temp_point;
            new_points++;
        }

        // 更新点总数
        _inclusion_polygon_numpoints += new_points;
    }
    return true;
}

// 检查排除多边形是否在运行create_exclusion_polygon_with_margin后更新
// 如果更改则返回true
bool AP_OADijkstra::check_exclusion_polygon_updated() const
{
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return false;
    }
    return (_exclusion_polygon_update_ms != fence->polyfence().get_exclusion_polygon_update_ms());
}

// 在现有排除多边形周围创建多边形
// 成功时返回true。失败时返回false并更新err_id
bool AP_OADijkstra::create_exclusion_polygon_with_margin(float margin_cm, AP_OADijkstra_Error &err_id)
{
    const AC_Fence *fence = AC_Fence::get_singleton();

    if (fence == nullptr) {
        err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_FENCE_DISABLED;
        return false;
    }

    // 如果之前的围栏点未更改,则跳过不必要的重试以构建排除多边形
    if (_exclusion_polygon_update_ms == fence->polyfence().get_exclusion_polygon_update_ms()) {
        return false;
    }

    _exclusion_polygon_update_ms = fence->polyfence().get_exclusion_polygon_update_ms();


    // 清除所有点
    _exclusion_polygon_numpoints = 0;

    // 如果没有排除多边形则立即返回
    const uint8_t num_exclusion_polygons = fence->polyfence().get_exclusion_polygon_count();

    // 遍历排除多边形并创建外部点
    for (uint8_t i = 0; i < num_exclusion_polygons; i++) {
        uint16_t num_points;
        const Vector2f* boundary = fence->polyfence().get_exclusion_polygon(i, num_points);
   
        // 对于排除多边形中的每个点
        // 注意:boundary是"未闭合的",意味着最后一个点*不*与第一个点相同
        uint16_t new_points = 0;
        for (uint16_t j = 0; j < num_points; j++) {

            // 找到当前点之前和之后的点(相对于当前点)
            const uint16_t before_idx = (j == 0) ? num_points-1 : j-1;
            const uint16_t after_idx = (j == num_points-1) ? 0 : j+1;
            Vector2f before_pt = boundary[before_idx] - boundary[j];
            Vector2f after_pt = boundary[after_idx] - boundary[j];

            // 如果点重叠则失败
            if (before_pt.is_zero() || after_pt.is_zero() || (before_pt == after_pt)) {
                err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_OVERLAPPING_POLYGON_POINTS;
                return false;
            }

            // 将点缩放为单位向量
            before_pt.normalize();
            after_pt.normalize();

            // 计算中间点并缩放到边距
            Vector2f intermediate_pt = after_pt + before_pt;
            intermediate_pt.normalize();
            intermediate_pt *= margin_cm;

            // 找到在原始多边形外的最终点
            Vector2f temp_point = boundary[j] + intermediate_pt;
            if (!Polygon_outside(temp_point, boundary, num_points)) {
                intermediate_pt *= -1;
                temp_point = boundary[j] + intermediate_pt;
                if (!Polygon_outside(temp_point, boundary, num_points)) {
                    // 在任一侧都找不到在排除多边形外的点,所以失败
                    // 如果排除多边形有重叠的线可能会发生这种情况
                    err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_OVERLAPPING_POLYGON_LINES;
                    return false;
                }
            }

            // 不要在角落添加点
            if (fabsf(intermediate_pt.angle() - before_pt.angle()) < M_PI_2) {
                continue;
            }

            // 如果需要则扩展数组
            if (!_exclusion_polygon_pts.expand_to_hold(_exclusion_polygon_numpoints + new_points + 1)) {
                err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_OUT_OF_MEMORY;
                return false;
            }
            // 添加点
            _exclusion_polygon_pts[_exclusion_polygon_numpoints + new_points] = temp_point;
            new_points++;
        }

        // 更新点总数
        _exclusion_polygon_numpoints += new_points;
    }
    return true;
}

// 检查排除圆形是否在运行create_exclusion_circle_with_margin后更新
// 如果更改则返回true
bool AP_OADijkstra::check_exclusion_circle_updated() const
{
    // 如果围栏未启用则立即退出
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return false;
    }
    return (_exclusion_circle_update_ms != fence->polyfence().get_exclusion_circle_update_ms());
}

// 在现有排除圆形周围创建多边形
// 成功时返回true。失败时返回false并更新err_id
bool AP_OADijkstra::create_exclusion_circle_with_margin(float margin_cm, AP_OADijkstra_Error &err_id)
{
    // 如果围栏未启用则立即退出
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_FENCE_DISABLED;
        return false;
    }

    // 清除所有点
    _exclusion_circle_numpoints = 0;

    // 圆形周围多边形点的单位长度偏移
    const Vector2f unit_offsets[] = {
            {cosf(radians(30)), cosf(radians(30-90))},  // 东北
            {cosf(radians(90)), cosf(radians(90-90))},  // 东
            {cosf(radians(150)), cosf(radians(150-90))},// 东南
            {cosf(radians(210)), cosf(radians(210-90))},// 西南
            {cosf(radians(270)), cosf(radians(270-90))},// 西
            {cosf(radians(330)), cosf(radians(330-90))},// 西北
    };
    const uint8_t num_points_per_circle = ARRAY_SIZE(unit_offsets);

    // 如果需要则扩展多边形点数组
    const uint8_t num_exclusion_circles = fence->polyfence().get_exclusion_circle_count();
    if (!_exclusion_circle_pts.expand_to_hold(num_exclusion_circles * num_points_per_circle)) {
        err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_OUT_OF_MEMORY;
        return false;
    }

    // 遍历排除圆形并创建外部多边形点
    // iterate through exclusion circles and create outer polygon points
    for (uint8_t i = 0; i < num_exclusion_circles; i++) {
        Vector2f circle_pos_cm;
        float radius;
        if (fence->polyfence().get_exclusion_circle(i, circle_pos_cm, radius)) {
            // 确保点之间的线段不会与圆相交的缩放因子
            // scaler to ensure lines between points do not intersect circle
            const float scaler = (1.0f / cosf(radians(180.0f / (float)num_points_per_circle))) * ((radius * 100.0f) + margin_cm);

            // 将点添加到数组中
            // add points to array
            for (uint8_t j = 0; j < num_points_per_circle; j++) {
                _exclusion_circle_pts[_exclusion_circle_numpoints] = circle_pos_cm + (unit_offsets[j] * scaler);
                _exclusion_circle_numpoints++;
            }
        }
    }

    // 记录围栏更新时间,以避免重复处理这些排除圆形
    // record fence update time so we don't process these exclusion circles again
    _exclusion_circle_update_ms = fence->polyfence().get_exclusion_circle_update_ms();

    return true;
}

// 返回所有围栏类型的点总数
// returns total number of points across all fence types
uint16_t AP_OADijkstra::total_numpoints() const
{
    return _inclusion_polygon_numpoints + _exclusion_polygon_numpoints + _exclusion_circle_numpoints;
}

// 从所有围栏类型的点列表中获取单个点
// get a single point across the total list of points from all fence types
bool AP_OADijkstra::get_point(uint16_t index, Vector2f &point) const
{
    // 检查索引是否有效
    // sanity check index
    if (index >= total_numpoints()) {
        return false;
    }

    // 返回包含多边形点
    // return an inclusion polygon point
    if (index < _inclusion_polygon_numpoints) {
        point = _inclusion_polygon_pts[index];
        return true;
    }
    index -= _inclusion_polygon_numpoints;

    // 返回排除多边形点
    // return an exclusion polygon point
    if (index < _exclusion_polygon_numpoints) {
        point = _exclusion_polygon_pts[index];
        return true;
    }
    index -= _exclusion_polygon_numpoints;

    // 返回排除圆形点
    // return an exclusion circle point
    if (index < _exclusion_circle_numpoints) {
        point = _exclusion_circle_pts[index];
        return true;
    }

    // 不应该执行到这里,以防万一返回false
    // we should never get here but just in case
    return false;
}

// 如果线段与多边形或圆形围栏相交则返回true
// returns true if line segment intersects polygon or circular fence
bool AP_OADijkstra::intersects_fence(const Vector2f &seg_start, const Vector2f &seg_end) const
{
    // 如果围栏未启用则立即返回
    // return immediately if fence is not enabled
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return false;
    }

    // 判断线段是否与任何包含多边形相交
    // determine if segment crosses any of the inclusion polygons
    uint16_t num_points = 0;
    for (uint8_t i = 0; i < fence->polyfence().get_inclusion_polygon_count(); i++) {
        const Vector2f* boundary = fence->polyfence().get_inclusion_polygon(i, num_points);
        if (boundary != nullptr) {
            Vector2f intersection;
            if (Polygon_intersects(boundary, num_points, seg_start, seg_end, intersection)) {
                return true;
            }
        }
    }

    // 判断线段是否与任何排除多边形相交
    // determine if segment crosses any of the exclusion polygons
    for (uint8_t i = 0; i < fence->polyfence().get_exclusion_polygon_count(); i++) {
        const Vector2f* boundary = fence->polyfence().get_exclusion_polygon(i, num_points);
        if (boundary != nullptr) {
            Vector2f intersection;
            if (Polygon_intersects(boundary, num_points, seg_start, seg_end, intersection)) {
                return true;
            }
        }
    }

    // 判断线段是否与任何包含圆形相交
    // determine if segment crosses any of the inclusion circles
    for (uint8_t i = 0; i < fence->polyfence().get_inclusion_circle_count(); i++) {
        Vector2f center_pos_cm;
        float radius;
        if (fence->polyfence().get_inclusion_circle(i, center_pos_cm, radius)) {
            // 如果起点或终点到圆心的距离大于半径则相交
            // intersects circle if either start or end is further from the center than the radius
            const float radius_cm_sq = sq(radius * 100.0f) ;
            if ((seg_start - center_pos_cm).length_squared() > radius_cm_sq) {
                return true;
            }
            if ((seg_end - center_pos_cm).length_squared() > radius_cm_sq) {
                return true;
            }
        }
    }

    // 判断线段是否与任何排除圆形相交
    // determine if segment crosses any of the exclusion circles
    for (uint8_t i = 0; i < fence->polyfence().get_exclusion_circle_count(); i++) {
        Vector2f center_pos_cm;
        float radius;
        if (fence->polyfence().get_exclusion_circle(i, center_pos_cm, radius)) {
            // 计算圆心到线段的距离
            // calculate distance between circle's center and segment
            const float dist_cm = Vector2f::closest_distance_between_line_and_point(seg_start, seg_end, center_pos_cm);

            // 如果距离小于半径则相交
            // intersects if distance is less than radius
            if (dist_cm <= (radius * 100.0f)) {
                return true;
            }
        }
    }

    // 如果执行到这里说明没有相交
    // if we got this far then no intersection
    return false;
}

// 为所有围栏点(带边距)创建可见性图
// create visibility graph for all fence (with margin) points
// 成功时返回true。失败时返回false并更新err_id
// returns true on success.  returns false on failure and err_id is updated
// 需要先运行这些函数:create_inclusion_polygon_with_margin, create_exclusion_polygon_with_margin, create_exclusion_circle_with_margin
// requires these functions to have been run create_inclusion_polygon_with_margin, create_exclusion_polygon_with_margin, create_exclusion_circle_with_margin
bool AP_OADijkstra::create_fence_visgraph(AP_OADijkstra_Error &err_id)
{
    // 如果围栏未启用则立即退出
    // exit immediately if fence is not enabled
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_FENCE_DISABLED;
        return false;
    }

    // 如果围栏点数超过算法能处理的范围则失败
    // fail if more fence points than algorithm can handle
    if (total_numpoints() >= OA_DIJKSTRA_POLYGON_SHORTPATH_NOTSET_IDX) {
        err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_TOO_MANY_FENCE_POINTS;
        return false;
    }

    // 清除围栏点可见性图
    // clear fence points visibility graph
    _fence_visgraph.clear();

    // 计算每个点到所有其他点的距离
    // calculate distance from each point to all other points
    for (uint8_t i = 0; i < total_numpoints() - 1; i++) {
        Vector2f start_seg;
        if (get_point(i, start_seg)) {
            for (uint8_t j = i + 1; j < total_numpoints(); j++) {
                Vector2f end_seg;
                if (get_point(j, end_seg)) {
                    // 如果线段不与任何包含或排除区域相交则添加到可见性图
                    // if line segment does not intersect with any inclusion or exclusion zones add to visgraph
                    if (!intersects_fence(start_seg, end_seg)) {
                        if (!_fence_visgraph.add_item({AP_OAVisGraph::OATYPE_INTERMEDIATE_POINT, i},
                                                      {AP_OAVisGraph::OATYPE_INTERMEDIATE_POINT, j},
                                                      (start_seg - end_seg).length())) {
                            // 添加点失败只可能是由于内存不足
                            // failure to add a point can only be caused by out-of-memory
                            err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_OUT_OF_MEMORY;
                            return false;
                        }
                    }
                }
            }
        }
    }

    return true;
}

// 为给定位置更新可见性图,该位置是相对于ekf原点的偏移(单位:厘米)
// updates visibility graph for a given position which is an offset (in cm) from the ekf origin
// 要添加额外位置(即目标点)时,设置add_extra_position = true并在extra_position参数中提供位置
// to add an additional position (i.e. the destination) set add_extra_position = true and provide the position in the extra_position argument
// 需要先运行create_inclusion_polygon_with_margin
// requires create_inclusion_polygon_with_margin to have been run
// 成功时返回true
// returns true on success
bool AP_OADijkstra::update_visgraph(AP_OAVisGraph& visgraph, const AP_OAVisGraph::OAItemID& oaid, const Vector2f &position, bool add_extra_position, Vector2f extra_position)
{
    // 清除可见性图
    // clear visibility graph
    visgraph.clear();

    // 计算位置到所有包含/排除围栏点的距离
    // calculate distance from position to all inclusion/exclusion fence points
    for (uint8_t i = 0; i < total_numpoints(); i++) {
        Vector2f seg_end;
        if (get_point(i, seg_end)) {
            if (!intersects_fence(position, seg_end)) {
                // 线段不与围栏相交,添加到可见性图
                // line segment does not intersect with fences so add to visgraph
                if (!visgraph.add_item(oaid, {AP_OAVisGraph::OATYPE_INTERMEDIATE_POINT, i}, (position - seg_end).length())) {
                    return false;
                }
            }
        }
    }

    // 如果额外点不与多边形围栏或排除多边形相交,则添加到可见性图
    // add extra point to visibility graph if it doesn't intersect with polygon fence or exclusion polygons
    if (add_extra_position) {
        if (!intersects_fence(position, extra_position)) {
            if (!visgraph.add_item(oaid, {AP_OAVisGraph::OATYPE_DESTINATION, 0}, (position - extra_position).length())) {
                return false;
            }
        }
    }

    return true;
}

// 更新从当前节点可见的所有节点的总距离
// update total distance for all nodes visible from current node
// curr_node_idx是_short_path_data数组的索引
// curr_node_idx is an index into the _short_path_data array
void AP_OADijkstra::update_visible_node_distances(node_index curr_node_idx)
{
    // 检查参数有效性
    // sanity check
    if (curr_node_idx >= _short_path_data_numpoints) {
        return;
    }

    // 为方便起见获取当前节点
    // get current node for convenience
    const ShortPathNode &curr_node = _short_path_data[curr_node_idx];

    // 遍历每个可见性图
    // for each visibility graph
    const AP_OAVisGraph* visgraphs[] = {&_fence_visgraph, &_destination_visgraph};
    for (uint8_t v=0; v<ARRAY_SIZE(visgraphs); v++) {

        // 如果为空则跳过
        // skip if empty
        const AP_OAVisGraph &curr_visgraph = *visgraphs[v];
        if (curr_visgraph.num_items() == 0) {
            continue;
        }

        // 在可见性图中搜索从当前节点可见的项目
        // search visibility graph for items visible from current_node
        for (uint16_t i = 0; i < curr_visgraph.num_items(); i++) {
            const AP_OAVisGraph::VisGraphItem &item = curr_visgraph[i];
            // 如果当前节点的id与图中任一id匹配(即向量的任一端)
            // match if current node's id matches either of the id's in the graph (i.e. either end of the vector)
            if ((curr_node.id == item.id1) || (curr_node.id == item.id2)) {
                AP_OAVisGraph::OAItemID matching_id = (curr_node.id == item.id1) ? item.id2 : item.id1;
                // 在节点数组中查找项目的id
                // find item's id in node array
                node_index item_node_idx;
                if (find_node_from_id(matching_id, item_node_idx)) {
                    // 如果当前节点的距离+到项目的距离小于项目的当前距离,则更新项目的距离
                    // if current node's distance + distance to item is less than item's current distance, update item's distance
                    const float dist_to_item_via_current_node = _short_path_data[curr_node_idx].distance_cm + item.distance_cm;
                    if (dist_to_item_via_current_node < _short_path_data[item_node_idx].distance_cm) {
                        // 更新项目的距离并将"distance_from_idx"设置为当前节点的索引
                        // update item's distance and set "distance_from_idx" to current node's index
                        _short_path_data[item_node_idx].distance_cm = dist_to_item_via_current_node;
                        _short_path_data[item_node_idx].distance_from_idx = curr_node_idx;
                    }
                }
            }
        }
    }
}

// 从节点的id(即id类型和id编号)中查找其在_short_path_data数组中的索引
// find a node's index into _short_path_data array from it's id (i.e. id type and id number)
// 成功时返回true并更新node_idx
// returns true if successful and node_idx is updated
bool AP_OADijkstra::find_node_from_id(const AP_OAVisGraph::OAItemID &id, node_index &node_idx) const
{
    switch (id.id_type) {
    case AP_OAVisGraph::OATYPE_SOURCE:
        // 源节点始终是第一个节点
        // source node is always the first node
        if (_short_path_data_numpoints > 0) {
            node_idx = 0;
            return true;
        }
        break;
    case AP_OAVisGraph::OATYPE_DESTINATION:
        // 目标点始终是第二个节点
        // destination is always the 2nd node
        if (_short_path_data_numpoints > 1) {
            node_idx = 1;
            return true;
        }
        break;
    case AP_OAVisGraph::OATYPE_INTERMEDIATE_POINT:
        // 中间节点从第三个节点开始
        // intermediate nodes start from 3rd node
        if (_short_path_data_numpoints > id.id_num + 2) {
            node_idx = id.id_num + 2;
            return true;
        }
        break;
    }

    // 找不到节点
    // could not find node
    return false;
}

// 查找具有最低暂定距离的节点的索引(忽略已访问的节点)
// find index of node with lowest tentative distance (ignore visited nodes)
// 成功时返回true并更新node_idx参数
// returns true if successful and node_idx argument is updated
bool AP_OADijkstra::find_closest_node_idx(node_index &node_idx) const
{
    node_index lowest_idx = 0;
    float lowest_dist = FLT_MAX;

    // 扫描所有节点寻找最近的
    // scan through all nodes looking for closest
    for (node_index i=0; i<_short_path_data_numpoints; i++) {
        const ShortPathNode &node = _short_path_data[i];
        if (node.visited || is_equal(_short_path_data[i].distance_cm, FLT_MAX)) {
            // 如果节点已访问或尚无法到达,我们不能使用它
            // if node is already visited OR cannot be reached yet, we can't use it
            continue;
        }
        // 计算此节点的位置
        // figure out the pos of this node
        Vector2f node_pos;
        float dist_with_heuristics = FLT_MAX;
        if (convert_node_to_point(node.id, node_pos)) {
            // 启发式是从节点到目标的简单欧几里得距离
            // 这应该是可接受的,因此保证最优路径
            // heuristics is is simple Euclidean distance from the node to the destination
            // This should be admissible, therefore optimal path is guaranteed
            const float heuristics = (node_pos-_path_destination).length();
            dist_with_heuristics = node.distance_cm + heuristics;
        } else {
            // 不应该发生
            // shouldn't happen
            return false;
        }
        if (dist_with_heuristics < lowest_dist) {
            // 目前,这是最近的节点
            // for NOW, this is the closest node
            lowest_idx = i;
            lowest_dist = dist_with_heuristics;
        }
    }

    if (lowest_dist < FLT_MAX) {
        // 找到最近的节点
        // found the closest node
        node_idx = lowest_idx;
        return true;
    }
    return false;
}

// 计算从起点到终点的最短路径
// calculate shortest path from origin to destination
// 成功时返回true。失败时返回false并更新err_id
// returns true on success.  returns false on failure and err_id is updated
// 需要先运行这些函数:create_inclusion_polygon_with_margin, create_exclusion_polygon_with_margin, create_exclusion_circle_with_margin, create_polygon_fence_visgraph
// requires these functions to have been run: create_inclusion_polygon_with_margin, create_exclusion_polygon_with_margin, create_exclusion_circle_with_margin, create_polygon_fence_visgraph
// 结果路径存储在_shortest_path数组中,作为相对于EKF原点的向量偏移
// resulting path is stored in _shortest_path array as vector offsets from EKF origin
bool AP_OADijkstra::calc_shortest_path(const Location &origin, const Location &destination, AP_OADijkstra_Error &err_id)
{
    // 将起点和终点转换为相对于EKF原点的偏移
    // convert origin and destination to offsets from EKF origin
    if (!origin.get_vector_xy_from_origin_NE(_path_source) || !destination.get_vector_xy_from_origin_NE(_path_destination)) {
        err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_NO_POSITION_ESTIMATE;
        return false;
    }

    // 创建起点和终点到围栏点的可见性图
    // create visgraphs of origin and destination to fence points
    if (!update_visgraph(_source_visgraph, {AP_OAVisGraph::OATYPE_SOURCE, 0}, _path_source, true, _path_destination)) {
        err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_OUT_OF_MEMORY;
        return false;
    }
    if (!update_visgraph(_destination_visgraph, {AP_OAVisGraph::OATYPE_DESTINATION, 0}, _path_destination)) {
        err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_OUT_OF_MEMORY;
        return false;
    }

    // 如有必要扩展_short_path_data
    // expand _short_path_data if necessary
    if (!_short_path_data.expand_to_hold(2 + total_numpoints())) {
        err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_OUT_OF_MEMORY;
        return false;
    }

    // 将起点和终点(node_type, id, visited, distance_from_idx, distance_cm)添加到short_path_data数组
    // add origin and destination (node_type, id, visited, distance_from_idx, distance_cm) to short_path_data array
    _short_path_data[0] = {{AP_OAVisGraph::OATYPE_SOURCE, 0}, false, 0, 0};
    _short_path_data[1] = {{AP_OAVisGraph::OATYPE_DESTINATION, 0}, false, OA_DIJKSTRA_POLYGON_SHORTPATH_NOTSET_IDX, FLT_MAX};
    _short_path_data_numpoints = 2;

    // 将所有包含和排除围栏点添加到short_path_data数组(node_type, id, visited, distance_from_idx, distance_cm)
    // add all inclusion and exclusion fence points to short_path_data array (node_type, id, visited, distance_from_idx, distance_cm)
    for (uint8_t i=0; i<total_numpoints(); i++) {
        _short_path_data[_short_path_data_numpoints++] = {{AP_OAVisGraph::OATYPE_INTERMEDIATE_POINT, i}, false, OA_DIJKSTRA_POLYGON_SHORTPATH_NOTSET_IDX, FLT_MAX};
    }

    // 从源点开始算法
    // start algorithm from source point
    node_index current_node_idx = 0;

    // 更新从源点可见的节点
    // update nodes visible from source point
    for (uint16_t i = 0; i < _source_visgraph.num_items(); i++) {
        node_index node_idx;
        if (find_node_from_id(_source_visgraph[i].id2, node_idx)) {
            _short_path_data[node_idx].distance_cm = _source_visgraph[i].distance_cm;
            _short_path_data[node_idx].distance_from_idx = current_node_idx;
        } else {
            err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_COULD_NOT_FIND_PATH;
            return false;
        }
    }
    // 将源节点标记为已访问
    // mark source node as visited
    _short_path_data[current_node_idx].visited = true;

    // 将current_node_idx移动到距离最小的节点
    // move current_node_idx to node with lowest distance
    while (find_closest_node_idx(current_node_idx)) {
        node_index dest_node;
        // 查看这个下一个"最近"节点是否实际上是目标点
        // See if this next "closest" node is actually the destination
        if (find_node_from_id({AP_OAVisGraph::OATYPE_DESTINATION,0}, dest_node) && current_node_idx == dest_node) {
            // 我们已经发现目标点..不用管图的其余部分
            // We have discovered destination.. Don't bother with the rest of the graph
            break;
        }
        // 更新当前节点所有邻居的距离
        // update distances to all neighbours of current node
        update_visible_node_distances(current_node_idx);

        // 将当前节点标记为已访问
        // mark current node as visited
        _short_path_data[current_node_idx].visited = true;
    }

    // 从目标点开始提取路径
    // extract path starting from destination
    bool success = false;
    node_index nidx;
    if (!find_node_from_id({AP_OAVisGraph::OATYPE_DESTINATION,0}, nidx)) {
        err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_COULD_NOT_FIND_PATH;
        return false;
    }
    _path_numpoints = 0;
    while (true) {
        if (!_path.expand_to_hold(_path_numpoints + 1)) {
            err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_OUT_OF_MEMORY;
            return false;
        }
        // 如果最新节点的distance_from_index无效则失败
        // fail if newest node has invalid distance_from_index
        if ((_short_path_data[nidx].distance_from_idx == OA_DIJKSTRA_POLYGON_SHORTPATH_NOTSET_IDX) ||
            (_short_path_data[nidx].distance_cm >= FLT_MAX)) {
            break;
        } else {
            // 将节点的id添加到路径数组
            // add node's id to path array
            _path[_path_numpoints] = _short_path_data[nidx].id;
            _path_numpoints++;

            // 如果节点是源点则完成
            // we are done if node is the source
            if (_short_path_data[nidx].id.id_type == AP_OAVisGraph::OATYPE_SOURCE) {
                success = true;
                break;
            } else {
                // 沿着节点的"distance_from_idx"到路径上的前一个节点
                // follow node's "distance_from_idx" to previous node on path
                nidx = _short_path_data[nidx].distance_from_idx;
            }
        }
    }
    // 如果未找到路径则报告错误
    // report error in case path not found
    if (!success) {
        err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_COULD_NOT_FIND_PATH;
    }

    return success;
}

// 从最终路径返回点作为相对于ekf原点的偏移(单位:厘米)
// return point from final path as an offset (in cm) from the ekf origin
bool AP_OADijkstra::get_shortest_path_point(uint8_t point_num, Vector2f& pos) const
{
    if ((_path_numpoints == 0) || (point_num >= _path_numpoints)) {
        return false;
    }

    // 从路径获取id
    // get id from path
    AP_OAVisGraph::OAItemID id = _path[_path_numpoints - point_num - 1];

    return convert_node_to_point(id, pos);
}

// 查找节点的位置作为相对于ekf原点的偏移(单位:厘米)
// find the position of a node as an offset (in cm) from the ekf origin
bool AP_OADijkstra::convert_node_to_point(const AP_OAVisGraph::OAItemID& id, Vector2f& pos) const
{
    // 将id转换为相对于EKF原点的位置偏移
    // convert id to a position offset from EKF origin
    switch (id.id_type) {
    case AP_OAVisGraph::OATYPE_SOURCE:
        pos = _path_source;
        return true;
    case AP_OAVisGraph::OATYPE_DESTINATION:
        pos = _path_destination;
        return true;
    case AP_OAVisGraph::OATYPE_INTERMEDIATE_POINT:
        return get_point(id.id_num, pos);
    }

    // 我们不应该执行到这里,以防万一
    // we should never reach here but just in case
    return false;
}
#endif // AP_FENCE_ENABLED


#endif  // AP_OAPATHPLANNER_DIJKSTRA_ENABLED
