#pragma once

#include "AC_Avoidance_config.h"

#if AP_OAPATHPLANNER_DIJKSTRA_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include "AP_OAVisGraph.h"
#include <AP_Logger/AP_Logger_config.h>

/*
 * Dijkstra's algorithm for path planning around polygon fence
 * 用于多边形围栏周围路径规划的Dijkstra算法
 */

class AP_OADijkstra {
public:

    AP_OADijkstra(AP_Int16 &options);

    CLASS_NO_COPY(AP_OADijkstra);  /* Do not allow copies */ // 不允许复制

    // set fence margin (in meters) used when creating "safe positions" within the polygon fence
    // 设置创建多边形围栏内"安全位置"时使用的围栏边距(单位:米)
    void set_fence_margin(float margin) { _polyfence_margin = MAX(margin, 0.0f); }

    // trigger Dijkstra's to recalculate shortest path based on current location
    // 触发Dijkstra重新计算基于当前位置的最短路径 
    void recalculate_path() { _shortest_path_ok = false; }

    // update return status enum
    // 更新返回状态枚举
    enum AP_OADijkstra_State : uint8_t {
        DIJKSTRA_STATE_NOT_REQUIRED = 0,  // 不需要避障
        DIJKSTRA_STATE_ERROR,             // 错误状态
        DIJKSTRA_STATE_SUCCESS            // 成功状态
    };

    // calculate a destination to avoid the polygon fence
    // returns DIJKSTRA_STATE_SUCCESS and populates origin_new, destination_new and next_destination_new if avoidance is required
    // next_destination_new will be non-zero if there is a next destination
    // dest_to_next_dest_clear will be set to true if the path from (the input) destination to (input) next_destination is clear
    // 计算避开多边形围栏的目标点
    // 如果需要避障,返回DIJKSTRA_STATE_SUCCESS并更新origin_new、destination_new和next_destination_new
    // 如果有下一个目标点,next_destination_new将不为零
    // 如果从(输入的)destination到(输入的)next_destination的路径是通畅的,dest_to_next_dest_clear将设置为true
    AP_OADijkstra_State update(const Location &current_loc,
                               const Location &destination,
                               const Location &next_destination,
                               Location& origin_new,
                               Location& destination_new,
                               Location& next_destination_new,
                               bool& dest_to_next_dest_clear);

private:

    // returns true if at least one inclusion or exclusion zone is enabled
    // 如果至少启用了一个包含区或排除区则返回true
    bool some_fences_enabled() const;

    // error enumeration
    // 错误枚举
    enum class AP_OADijkstra_Error : uint8_t {
        DIJKSTRA_ERROR_NONE = 0,                          // 无错误
        DIJKSTRA_ERROR_OUT_OF_MEMORY,                     // 内存不足
        DIJKSTRA_ERROR_OVERLAPPING_POLYGON_POINTS,        // 多边形点重叠
        DIJKSTRA_ERROR_FAILED_TO_BUILD_INNER_POLYGON,     // 构建内部多边形失败
        DIJKSTRA_ERROR_OVERLAPPING_POLYGON_LINES,         // 多边形线段重叠
        DIJKSTRA_ERROR_FENCE_DISABLED,                    // 围栏已禁用
        DIJKSTRA_ERROR_TOO_MANY_FENCE_POINTS,            // 围栏点太多
        DIJKSTRA_ERROR_NO_POSITION_ESTIMATE,             // 无位置估计
        DIJKSTRA_ERROR_COULD_NOT_FIND_PATH               // 找不到路径
    } _error_id;

    // return error message for a given error id
    // 返回给定错误ID的错误消息
    const char* get_error_msg(AP_OADijkstra_Error error_id) const;

    // report error to ground station
    // 向地面站报告错误
    void report_error(AP_OADijkstra_Error error_id);

    //
    // inclusion polygon methods
    // 包含多边形方法
    //

    // check if inclusion polygons have been updated since create_inclusion_polygon_with_margin was run
    // returns true if changed
    // 检查自运行create_inclusion_polygon_with_margin以来包含多边形是否已更新
    // 如果更改则返回true
    bool check_inclusion_polygon_updated() const;

    // create polygons inside the existing inclusion polygons
    // returns true on success.  returns false on failure and err_id is updated
    // 在现有包含多边形内创建多边形
    // 成功时返回true。失败时返回false并更新err_id
    bool create_inclusion_polygon_with_margin(float margin_cm, AP_OADijkstra_Error &err_id);

    //
    // exclusion polygon methods
    // 排除多边形方法
    //

    // check if exclusion polygons have been updated since create_exclusion_polygon_with_margin was run
    // returns true if changed
    // 检查自运行create_exclusion_polygon_with_margin以来排除多边形是否已更新
    // 如果更改则返回true
    bool check_exclusion_polygon_updated() const;

    // create polygons around existing exclusion polygons
    // returns true on success.  returns false on failure and err_id is updated
    // 在现有排除多边形周围创建多边形
    // 成功时返回true。失败时返回false并更新err_id
    bool create_exclusion_polygon_with_margin(float margin_cm, AP_OADijkstra_Error &err_id);

    //
    // exclusion circle methods
    // 排除圆形方法
    //

    // check if exclusion circles have been updated since create_exclusion_circle_with_margin was run
    // returns true if changed
    // 检查自运行create_exclusion_circle_with_margin以来排除圆形是否已更新
    // 如果更改则返回true
    bool check_exclusion_circle_updated() const;

    // create polygons around existing exclusion circles
    // returns true on success.  returns false on failure and err_id is updated
    // 在现有排除圆形周围创建多边形
    // 成功时返回true。失败时返回false并更新err_id
    bool create_exclusion_circle_with_margin(float margin_cm, AP_OADijkstra_Error &err_id);

    //
    // other methods
    // 其他方法
    //

    // returns total number of points across all fence types
    // 返回所有围栏类型的点总数
    uint16_t total_numpoints() const;

    // get a single point across the total list of points from all fence types
    // also returns the type of point
    // 从所有围栏类型的点列表中获取单个点
    // 同时返回点的类型
    bool get_point(uint16_t index, Vector2f& point) const;

    // returns true if line segment intersects polygon or circular fence
    // 如果线段与多边形或圆形围栏相交则返回true
    bool intersects_fence(const Vector2f &seg_start, const Vector2f &seg_end) const;

    // create visibility graph for all fence (with margin) points
    // returns true on success.  returns false on failure and err_id is updated
    // 为所有围栏(带边距)点创建可见性图
    // 成功时返回true。失败时返回false并更新err_id
    bool create_fence_visgraph(AP_OADijkstra_Error &err_id);

    // calculate shortest path from origin to destination
    // returns true on success.  returns false on failure and err_id is updated
    // requires create_polygon_fence_with_margin and create_polygon_fence_visgraph to have been run
    // resulting path is stored in _shortest_path array as vector offsets from EKF origin
    // 计算从起点到目标点的最短路径
    // 成功时返回true。失败时返回false并更新err_id
    // 需要先运行create_polygon_fence_with_margin和create_polygon_fence_visgraph
    // 结果路径作为相对EKF原点的向量偏移存储在_shortest_path数组中
    bool calc_shortest_path(const Location &origin, const Location &destination, AP_OADijkstra_Error &err_id);

    // shortest path state variables
    // 最短路径状态变量
    bool _inclusion_polygon_with_margin_ok;      // 包含多边形(带边距)是否正常
    bool _exclusion_polygon_with_margin_ok;      // 排除多边形(带边距)是否正常
    bool _exclusion_circle_with_margin_ok;       // 排除圆形(带边距)是否正常
    bool _polyfence_visgraph_ok;                // 多边形围栏可见性图是否正常
    bool _shortest_path_ok;                      // 最短路径是否正常

    Location _destination_prev;     // destination of previous iterations (used to determine if path should be re-calculated)
                                   // 上一次迭代的目标点(用于确定是否需要重新计算路径)
    Location _next_destination_prev;// next_destination of previous iterations (used to determine if path should be re-calculated)
                                   // 上一次迭代的下一个目标点(用于确定是否需要重新计算路径)
    uint8_t _path_idx_returned;     // index into _path array which gives location vehicle should be currently moving towards
                                   // _path数组的索引,指示载具当前应该移动到的位置
    bool _dest_to_next_dest_clear;  // true if path from dest to next_dest is clear (i.e. does not intersects a fence)
                                   // 如果从dest到next_dest的路径是通畅的(即不与围栏相交)则为true

    // inclusion polygon (with margin) related variables
    // 包含多边形(带边距)相关变量
    float _polyfence_margin = 10;           // margin around polygon defaults to 10m but is overriden with set_fence_margin
                                           // 多边形周围的边距默认为10米,但可以通过set_fence_margin覆盖
    AP_ExpandingArray<Vector2f> _inclusion_polygon_pts; // array of nodes corresponding to inclusion polygon points plus a margin
                                                       // 对应包含多边形点加边距的节点数组
    uint8_t _inclusion_polygon_numpoints;   // number of points held in above array
                                           // 上述数组中包含的点数
    uint32_t _inclusion_polygon_update_ms;  // system time of boundary update from AC_Fence (used to detect changes to polygon fence)
                                           // 从AC_Fence更新边界的系统时间(用于检测多边形围栏的变化)

    // exclusion polygon related variables
    // 排除多边形相关变量
    AP_ExpandingArray<Vector2f> _exclusion_polygon_pts; // array of nodes corresponding to exclusion polygon points plus a margin
                                                       // 对应排除多边形点加边距的节点数组
    uint8_t _exclusion_polygon_numpoints;   // number of points held in above array
                                           // 上述数组中包含的点数
    uint32_t _exclusion_polygon_update_ms;  // system time exclusion polygon was updated (used to detect changes)
                                           // 更新排除多边形的系统时间(用于检测变化)

    // exclusion circle related variables
    // 排除圆形相关变量
    AP_ExpandingArray<Vector2f> _exclusion_circle_pts; // array of nodes surrounding exclusion circles plus a margin
                                                      // 围绕排除圆形加边距的节点数组
    uint8_t _exclusion_circle_numpoints;    // number of points held in above array
                                           // 上述数组中包含的点数
    uint32_t _exclusion_circle_update_ms;   // system time exclusion circles were updated (used to detect changes)
                                           // 更新排除圆形的系统时间(用于检测变化)

    // visibility graphs
    // 可见性图
    AP_OAVisGraph _fence_visgraph;          // holds distances between all inclusion/exclusion fence points (with margin)
                                           // 保存所有包含/排除围栏点之间的距离(带边距)
    AP_OAVisGraph _source_visgraph;         // holds distances from source point to all other nodes
                                           // 保存从源点到所有其他节点的距离
    AP_OAVisGraph _destination_visgraph;    // holds distances from the destination to all other nodes
                                           // 保存从目标点到所有其他节点的距离

    // updates visibility graph for a given position which is an offset (in cm) from the ekf origin
    // to add an additional position (i.e. the destination) set add_extra_position = true and provide the position in the extra_position argument
    // requires create_polygon_fence_with_margin to have been run
    // returns true on success
    // 更新给定位置的可见性图,该位置是相对EKF原点的偏移(单位:厘米)
    // 要添加额外位置(即目标点),设置add_extra_position = true并在extra_position参数中提供位置
    // 需要先运行create_polygon_fence_with_margin
    // 成功时返回true
    bool update_visgraph(AP_OAVisGraph& visgraph, const AP_OAVisGraph::OAItemID& oaid, const Vector2f &position, bool add_extra_position = false, Vector2f extra_position = Vector2f(0,0));

    typedef uint8_t node_index;         // indices into short path data
                                       // 最短路径数据的索引
    struct ShortPathNode {
        AP_OAVisGraph::OAItemID id;     // unique id for node (combination of type and id number)
                                       // 节点的唯一ID(类型和ID号的组合)
        bool visited;                   // true if all this node's neighbour's distances have been updated
                                       // 如果该节点所有邻居的距离都已更新则为true
        node_index distance_from_idx;   // index into _short_path_data from where distance was updated (or 255 if not set)
                                       // 距离更新来源的_short_path_data索引(如果未设置则为255)
        float distance_cm;              // distance from source (number is tentative until this node is the current node and/or visited = true)
                                       // 到源点的距离(在该节点成为当前节点且/或visited = true之前,该数字是暂定的)
    };
    AP_ExpandingArray<ShortPathNode> _short_path_data;
    node_index _short_path_data_numpoints;  // number of elements in _short_path_data array
                                           // _short_path_data数组中的元素数量

    // update total distance for all nodes visible from current node
    // curr_node_idx is an index into the _short_path_data array
    // 更新从当前节点可见的所有节点的总距离
    // curr_node_idx是_short_path_data数组的索引
    void update_visible_node_distances(node_index curr_node_idx);

    // find a node's index into _short_path_data array from it's id (i.e. id type and id number)
    // returns true if successful and node_idx is updated
    // 从节点的ID(即ID类型和ID号)查找其在_short_path_data数组中的索引
    // 如果成功且node_idx已更新则返回true
    bool find_node_from_id(const AP_OAVisGraph::OAItemID &id, node_index &node_idx) const;

    // find index of node with lowest tentative distance (ignore visited nodes)
    // returns true if successful and node_idx argument is updated
    // 查找具有最小暂定距离的节点的索引(忽略已访问的节点)
    // 如果成功且node_idx参数已更新则返回true
    bool find_closest_node_idx(node_index &node_idx) const;

    // final path variables and functions
    // 最终路径变量和函数
    AP_ExpandingArray<AP_OAVisGraph::OAItemID> _path;   // ids of points on return path in reverse order (i.e. destination is first element)
                                                        // 返回路径上点的ID,按相反顺序排列(即目标点是第一个元素)
    uint8_t _path_numpoints;                            // number of points on return path
                                                        // 返回路径上的点数
    Vector2f _path_source;                              // source point used in shortest path calculations (offset in cm from EKF origin)
                                                        // 最短路径计算中使用的源点(相对EKF原点的偏移,单位:厘米)
    Vector2f _path_destination;                         // destination position used in shortest path calculations (offset in cm from EKF origin)
                                                        // 最短路径计算中使用的目标点位置(相对EKF原点的偏移,单位:厘米)

    // return number of points on path
    // 返回路径上的点数
    uint8_t get_shortest_path_numpoints() const { return _path_numpoints; }

    // return point from final path as an offset (in cm) from the ekf origin
    // 返回最终路径中的点,作为相对EKF原点的偏移(单位:厘米)
    bool get_shortest_path_point(uint8_t point_num, Vector2f& pos) const;

    // find the position of a node as an offset (in cm) from the ekf origin
    // returns true if successful and pos is updated
    // 查找节点的位置,作为相对EKF原点的偏移(单位:厘米)
    // 如果成功且pos已更新则返回true
    bool convert_node_to_point(const AP_OAVisGraph::OAItemID& id, Vector2f& pos) const;

    AP_OADijkstra_Error _error_last_id;                 // last error id sent to GCS
                                                        // 发送到GCS的最后一个错误ID
    uint32_t _error_last_report_ms;                     // last time an error message was sent to GCS
                                                        // 最后一次向GCS发送错误消息的时间

#if HAL_LOGGING_ENABLED
    // Logging functions
    // 日志记录函数
    void Write_OADijkstra(const uint8_t state, const uint8_t error_id, const uint8_t curr_point, const uint8_t tot_points, const Location &final_dest, const Location &oa_dest) const;
    void Write_Visgraph_point(const uint8_t version, const uint8_t point_num, const int32_t Lat, const int32_t Lon) const;
#else
    void Write_OADijkstra(const uint8_t state, const uint8_t error_id, const uint8_t curr_point, const uint8_t tot_points, const Location &final_dest, const Location &oa_dest) const {}
    void Write_Visgraph_point(const uint8_t version, const uint8_t point_num, const int32_t Lat, const int32_t Lon) const {}
#endif
    uint8_t _log_num_points;                            // 日志点数
    uint8_t _log_visgraph_version;                      // 可见性图版本

    // reference to AP_OAPathPlanner options param
    // AP_OAPathPlanner选项参数的引用
    AP_Int16 &_options;
};

#endif  // AP_OAPATHPLANNER_DIJKSTRA_ENABLED
