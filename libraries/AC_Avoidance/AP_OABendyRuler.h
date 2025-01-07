#pragma once

#include "AC_Avoidance_config.h"

#if AP_OAPATHPLANNER_BENDYRULER_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger_config.h>

/*
 * BendyRuler避障算法,用于避开多边形围栏、圆形围栏以及近距离传感器检测到的动态障碍物
 */
class AP_OABendyRuler {
public:
    AP_OABendyRuler();

    CLASS_NO_COPY(AP_OABendyRuler);  /* 不允许复制 */

    // 设置存储在前端参数中的配置信息
    void set_config(float margin_max) { _margin_max = MAX(margin_max, 0.0f); }

    // BendyRuler避障类型枚举
    enum class OABendyType {
        OA_BENDY_DISABLED   = 0,    // 禁用
        OA_BENDY_HORIZONTAL = 1,    // 水平避障
        OA_BENDY_VERTICAL   = 2,    // 垂直避障
    };

    // 运行后台任务寻找最佳路径
    // 如果找到最佳路径则返回true并更新origin_new和destination_new,如果不需要避障则返回false
    // bendy_type设置为使用的BendyRuler类型
    bool update(const Location& current_loc, const Location& destination, const Vector2f &ground_speed_vec, Location &origin_new, Location &destination_new, OABendyType &bendy_type, bool proximity_only);

    static const struct AP_Param::GroupInfo var_info[];

private:

    // 返回正在使用的BendyRuler类型
    OABendyType get_type() const;

    // 在XY平面搜索路径
    bool search_xy_path(const Location& current_loc, const Location& destination, float ground_course_deg, Location &destination_new, float lookahead_step_1_dist, float lookahead_step_2_dist, float bearing_to_dest, float distance_to_dest, bool proximity_only);

    // 在垂直方向搜索路径
    bool search_vertical_path(const Location &current_loc, const Location &destination, Location &destination_new, float lookahead_step1_dist, float lookahead_step2_dist, float bearing_to_dest, float distance_to_dest, bool proximity_only);

    // 计算路径与任何障碍物之间的最小距离
    float calc_avoidance_margin(const Location &start, const Location &end, bool proximity_only) const;

    // 判断BendyRuler是否应该接受新的航向或尝试抵抗它。如果航向未改变则返回true
    bool resist_bearing_change(const Location &destination, const Location &current_loc, bool active, float bearing_test, float lookahead_step1_dist, float margin, Location &prev_dest, float &prev_bearing, float &final_bearing, float &final_margin, bool proximity_only) const;    

    // 计算路径与圆形围栏(以home点为中心)之间的最小距离
    // 成功时返回true并更新margin
    bool calc_margin_from_circular_fence(const Location &start, const Location &end, float &margin) const;

    // 计算路径与高度围栏之间的最小距离
    // 成功时返回true并更新margin
    bool calc_margin_from_alt_fence(const Location &start, const Location &end, float &margin) const;

    // 计算路径与所有包含和排除多边形之间的最小距离
    // 成功时返回true并更新margin
    bool calc_margin_from_inclusion_and_exclusion_polygons(const Location &start, const Location &end, float &margin) const;

    // 计算路径与所有包含和排除圆形之间的最小距离
    // 成功时返回true并更新margin
    bool calc_margin_from_inclusion_and_exclusion_circles(const Location &start, const Location &end, float &margin) const;

    // 计算路径与近距离传感器障碍物之间的最小距离
    // 成功时返回true并更新margin
    bool calc_margin_from_object_database(const Location &start, const Location &end, float &margin) const;

    // 日志记录函数
#if HAL_LOGGING_ENABLED
    void Write_OABendyRuler(const uint8_t type, const bool active, const float target_yaw, const float target_pitch, const bool resist_chg, const float margin, const Location &final_dest, const Location &oa_dest) const;
#else
    void Write_OABendyRuler(const uint8_t type, const bool active, const float target_yaw, const float target_pitch, const bool resist_chg, const float margin, const Location &final_dest, const Location &oa_dest) const {}
#endif

    // 避障通用参数
    float _margin_max;              // 避障将忽略距离载具超过这么多米的物体
    
    // BendyRuler参数
    AP_Float _lookahead;            // 避障将在载具前方这么多米处寻找障碍物
    AP_Float _bendy_ratio;          // 如果边距比率变化小于此参数,避障将避免大的方向改变
    AP_Int16 _bendy_angle;          // 避障将尝试避免超过这个角度的方向改变
    AP_Int8  _bendy_type;           // 要运行的BendyRuler类型
    
    // 后台线程使用的内部变量
    float _current_lookahead;       // 当前在载具前方这么多米处寻找障碍物
    float _bearing_prev;            // 存储的航向角(度)
    Location _destination_prev;     // 上一个目标点,用于检查目标点是否发生变化
};

#endif  // AP_OAPATHPLANNER_BENDYRULER_ENABLED
