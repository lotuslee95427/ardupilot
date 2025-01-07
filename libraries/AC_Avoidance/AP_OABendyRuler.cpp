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

#if AP_OAPATHPLANNER_BENDYRULER_ENABLED

// 包含BendyRuler避障器头文件
#include "AP_OABendyRuler.h"
#include <AC_Avoidance/AP_OADatabase.h>
#include <AC_Fence/AC_Fence.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

// 参数默认值
const float OA_BENDYRULER_LOOKAHEAD_DEFAULT = 15.0f;  // 默认前视距离
const float OA_BENDYRULER_RATIO_DEFAULT = 1.5f;       // 默认比率
const int16_t OA_BENDYRULER_ANGLE_DEFAULT = 75;       // 默认角度
const int16_t OA_BENDYRULER_TYPE_DEFAULT = 1;         // 默认类型

// 水平方向每5度检查一次
const int16_t OA_BENDYRULER_BEARING_INC_XY = 5;            
// 垂直方向每90度检查一次
const int16_t OA_BENDYRULER_BEARING_INC_VERTICAL = 90;
// 第二步前视距离是第一步的比例
const float OA_BENDYRULER_LOOKAHEAD_STEP2_RATIO = 1.0f; 
// 第二步最小前视距离
const float OA_BENDYRULER_LOOKAHEAD_STEP2_MIN = 2.0f;   
// 前视距离至少超过目标点这么多米
const float OA_BENDYRULER_LOOKAHEAD_PAST_DEST = 2.0f;   
// 当地速低于此值时使用航向角
const float OA_BENDYRULER_LOW_SPEED_SQUARED = (0.2f * 0.2f);    

// 仅在直升机和多旋翼上启用垂直避障
#define VERTICAL_ENABLED APM_BUILD_COPTER_OR_HELI

// 参数定义
const AP_Param::GroupInfo AP_OABendyRuler::var_info[] = {

    // @Param: LOOKAHEAD
    // @DisplayName: 物体避障最大前视距离
    // @Description: 物体避障将在车辆前方这么多米处进行检查
    // @Units: m
    // @Range: 1 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("LOOKAHEAD", 1, AP_OABendyRuler, _lookahead, OA_BENDYRULER_LOOKAHEAD_DEFAULT),

    // @Param: CONT_RATIO
    // @DisplayName: BendyRuler改变航向的避障边距比率
    // @Description: BendyRuler只有在当前障碍物边距与之前边距的比率至少达到此值时才会改变航向
    // @Range: 1.1 2
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("CONT_RATIO", 2, AP_OABendyRuler, _bendy_ratio, OA_BENDYRULER_RATIO_DEFAULT),

    // @Param: CONT_ANGLE
    // @DisplayName: BendyRuler航向改变阻力阈值角度
    // @Description: BendyRuler将抵抗超过此角度的航向改变
    // @Range: 20 180
    // @Increment: 5
    // @User: Standard
    AP_GROUPINFO("CONT_ANGLE", 3, AP_OABendyRuler, _bendy_angle, OA_BENDYRULER_ANGLE_DEFAULT),

    // @Param{Copter}: TYPE
    // @DisplayName: BendyRuler类型
    // @Description: BendyRuler将沿此参数定义的方向搜索清晰路径
    // @Values: 1:水平搜索, 2:垂直搜索
    // @User: Standard
    AP_GROUPINFO_FRAME("TYPE", 4, AP_OABendyRuler, _bendy_type, OA_BENDYRULER_TYPE_DEFAULT, AP_PARAM_FRAME_COPTER | AP_PARAM_FRAME_HELI | AP_PARAM_FRAME_TRICOPTER),

    AP_GROUPEND
};

// 构造函数
AP_OABendyRuler::AP_OABendyRuler() 
{ 
    AP_Param::setup_object_defaults(this, var_info); 
    _bearing_prev = FLT_MAX;
}

// 运行后台任务寻找最佳路径
// 如果找到最佳路径则返回true并更新origin_new和destination_new。如果不需要避障则返回false
// bendy_type设置为使用的BendyRuler类型
bool AP_OABendyRuler::update(const Location& current_loc, const Location& destination, const Vector2f &ground_speed_vec, Location &origin_new, Location &destination_new, OABendyType &bendy_type, bool proximity_only)
{
    // bendy ruler总是将起点设为当前位置
    origin_new = current_loc;

    // 初始化返回的bendy_type
    bendy_type = OABendyType::OA_BENDY_DISABLED;

    // 计算到最终目标点的方位角和距离
    const float bearing_to_dest = current_loc.get_bearing_to(destination) * 0.01f;
    const float distance_to_dest = current_loc.get_distance(destination);

    // 确保用户设置了有意义的前视距离值
    _lookahead.set(MAX(_lookahead,1.0f));

    // 根据避障结果动态调整前视距离
    _current_lookahead = constrain_float(_current_lookahead, _lookahead * 0.5f, _lookahead);

    // 计算第一步的前视距离和时间。距离可以略长于到目标点的距离以留出躲避空间
    const float lookahead_step1_dist = MIN(_current_lookahead, distance_to_dest + OA_BENDYRULER_LOOKAHEAD_PAST_DEST);

    // 计算第二步的前视距离
    const float lookahead_step2_dist = _current_lookahead * OA_BENDYRULER_LOOKAHEAD_STEP2_RATIO;

    // 获取地速航向
    float ground_course_deg;
    if (ground_speed_vec.length_squared() < OA_BENDYRULER_LOW_SPEED_SQUARED) {
        // 地速为零时使用车辆航向
        ground_course_deg = AP::ahrs().yaw_sensor * 0.01f;
    } else {
        ground_course_deg = degrees(ground_speed_vec.angle());
    }

    bool ret;
    switch (get_type()) {
        case OABendyType::OA_BENDY_VERTICAL:
        #if VERTICAL_ENABLED 
            ret = search_vertical_path(current_loc, destination, destination_new, lookahead_step1_dist, lookahead_step2_dist, bearing_to_dest, distance_to_dest, proximity_only);
            bendy_type = OABendyType::OA_BENDY_VERTICAL;
            break;
        #endif

        case OABendyType::OA_BENDY_HORIZONTAL:
        default:
            ret = search_xy_path(current_loc, destination, ground_course_deg, destination_new, lookahead_step1_dist, lookahead_step2_dist, bearing_to_dest, distance_to_dest, proximity_only);
            bendy_type = OABendyType::OA_BENDY_HORIZONTAL;
    }

    return ret;
}

// 在水平方向搜索路径
bool AP_OABendyRuler::search_xy_path(const Location& current_loc, const Location& destination, float ground_course_deg, Location &destination_new, float lookahead_step1_dist, float lookahead_step2_dist, float bearing_to_dest, float distance_to_dest, bool proximity_only) 
{
    // 检查OA_BEARING_INC定义是否允许检查所有方向
    static_assert(360 % OA_BENDYRULER_BEARING_INC_XY == 0, "check 360 is a multiple of OA_BEARING_INC");

    // 以OA_BENDYRULER_BEARING_INC度为增量在车辆周围交替左右搜索。
    // 对每个方向检查车辆是否能避开所有障碍物
    float best_bearing = bearing_to_dest;
    float best_bearing_margin = -FLT_MAX;
    bool have_best_bearing = false;
    float best_margin = -FLT_MAX;
    float best_margin_bearing = best_bearing;

    for (uint8_t i = 0; i <= (170 / OA_BENDYRULER_BEARING_INC_XY); i++) {
        for (uint8_t bdir = 0; bdir <= 1; bdir++) {
            // 跳过直接朝向目标点的重复检查
            if ((i==0) && (bdir > 0)) {
                continue;
            }
            // 正在探测的方位角
            const float bearing_delta = i * OA_BENDYRULER_BEARING_INC_XY * (bdir == 0 ? -1.0f : 1.0f);
            const float bearing_test = wrap_180(bearing_to_dest + bearing_delta);

            // ToDo: 使用空速添加有效地速计算
            // ToDo: 添加车辆转向所需航向时位置变化的预测

            // 测试位置从当前位置按测试方位角投影
            Location test_loc = current_loc;
            test_loc.offset_bearing(bearing_test, lookahead_step1_dist);

            // 计算此场景下与障碍物的边距
            float margin = calc_avoidance_margin(current_loc, test_loc, proximity_only);
            if (margin > best_margin) {
                best_margin_bearing = bearing_test;
                best_margin = margin;
            }
            if (margin > _margin_max) {
                // 这个方位角可以避开前视距离内的障碍物
                // 现在检查朝向目标点的三个方向是否有清晰路径
                if (!have_best_bearing) {
                    best_bearing = bearing_test;
                    best_bearing_margin = margin;
                    have_best_bearing = true;
                } else if (fabsf(wrap_180(ground_course_deg - bearing_test)) <
                           fabsf(wrap_180(ground_course_deg - best_bearing))) {
                    // 用更接近当前地速航向的方位角替换
                    best_bearing = bearing_test;
                    best_bearing_margin = margin;
                }

                // 在三个方向进行第二阶段测试寻找障碍物
                const float test_bearings[] { 0.0f, 45.0f, -45.0f };
                const float bearing_to_dest2 = test_loc.get_bearing_to(destination) * 0.01f;
                float distance2 = constrain_float(lookahead_step2_dist, OA_BENDYRULER_LOOKAHEAD_STEP2_MIN, test_loc.get_distance(destination));
                for (uint8_t j = 0; j < ARRAY_SIZE(test_bearings); j++) {
                    float bearing_test2 = wrap_180(bearing_to_dest2 + test_bearings[j]);
                    Location test_loc2 = test_loc;
                    test_loc2.offset_bearing(bearing_test2, distance2);

                    // 计算此场景下与围栏和障碍物的最小边距
                    float margin2 = calc_avoidance_margin(test_loc, test_loc2, proximity_only);
                    if (margin2 > _margin_max) {
                        // 如果选择的方向直接朝向目标点则可以关闭避障
                        // i == 0 && j == 0 表示没有偏离朝向目标点的方位角
                        const bool active = (i != 0 || j != 0);
                        float final_bearing = bearing_test;
                        float final_margin = margin;
                        // 检查是否需要忽略test_bearing并继续使用之前的方位角
                        const bool ignore_bearing_change = resist_bearing_change(destination, current_loc, active, bearing_test, lookahead_step1_dist, margin, _destination_prev,_bearing_prev, final_bearing, final_margin, proximity_only);

                        // 一切正常,现在按完整距离在选择的方向上投影
                        destination_new = current_loc;
                        destination_new.offset_bearing(final_bearing, MIN(distance_to_dest, lookahead_step1_dist));
                        _current_lookahead = MIN(_lookahead, _current_lookahead * 1.1f);
                        Write_OABendyRuler((uint8_t)OABendyType::OA_BENDY_HORIZONTAL, active, bearing_to_dest, 0.0f, ignore_bearing_change, final_margin, destination, destination_new);
                        return active;
                    }
                }
            }
        }
    }

    float chosen_bearing;
    float chosen_distance;
    if (have_best_bearing) {
        // 测试的方向都不适合2步检查。选择第一步最好的方向
        chosen_bearing = best_bearing;
        chosen_distance = MAX(lookahead_step1_dist + MIN(best_bearing_margin, 0), 0);
        _current_lookahead = MIN(_lookahead, _current_lookahead * 1.05f);
    } else {
        // 所有可能的路径边距都为负。选择边距最大的
        chosen_bearing = best_margin_bearing;
        chosen_distance = MAX(lookahead_step1_dist + MIN(best_margin, 0), 0);
        _current_lookahead = MAX(_lookahead * 0.5f, _current_lookahead * 0.9f);
    }

    // 基于最佳尝试计算新目标
    destination_new = current_loc;
    destination_new.offset_bearing(chosen_bearing, chosen_distance);

    // 记录结果
    Write_OABendyRuler((uint8_t)OABendyType::OA_BENDY_HORIZONTAL, true, chosen_bearing, 0.0f, false, best_margin, destination, destination_new);

    return true;
}

// 在垂直方向搜索路径
bool AP_OABendyRuler::search_vertical_path(const Location &current_loc, const Location &destination, Location &destination_new, float lookahead_step1_dist, float lookahead_step2_dist, float bearing_to_dest, float distance_to_dest, bool proximity_only)
{
    // 检查OA_BEARING_INC_VERTICAL定义是否允许检查所有方向
    static_assert(360 % OA_BENDYRULER_BEARING_INC_VERTICAL == 0, "check 360 is a multiple of OA_BEARING_INC_VERTICAL");
    float best_pitch = 0.0f;
    bool  have_best_pitch = false;
    float best_margin = -FLT_MAX;
    float best_margin_pitch = best_pitch;
    const uint8_t angular_limit = 180 / OA_BENDYRULER_BEARING_INC_VERTICAL;

    for (uint8_t i = 0; i <= angular_limit; i++) {
        for (uint8_t bdir = 0; bdir <= 1; bdir++) {
            // 跳过直接朝向目标点或180度相反方向的重复检查
            if (((i==0) && (bdir > 0)) || ((i == angular_limit) && (bdir > 0))) {
                continue;
            }

            // 正在探测的俯仰角
            const float pitch_delta = i * OA_BENDYRULER_BEARING_INC_VERTICAL * (bdir == 0 ? 1.0f : -1.0f);

            Location test_loc = current_loc;
            test_loc.offset_bearing_and_pitch(bearing_to_dest, pitch_delta, lookahead_step1_dist);

            // 计算此场景下与障碍物的边距
            float margin = calc_avoidance_margin(current_loc, test_loc, proximity_only);

            if (margin > best_margin) {
                best_margin_pitch = pitch_delta;
                best_margin = margin;
            }

            if (margin > _margin_max) {
                // 这条路径可以用要求的边距避开障碍物,现在检查前方的路径
                if (!have_best_pitch) {
                    best_pitch = pitch_delta;
                    have_best_pitch = true;
                }
                const float test_pitch_step2[] { 0.0f, 90.0f, -90.0f, 180.0f};
                float bearing_to_dest2;
                if (is_equal(fabsf(pitch_delta), 90.0f)) {
                    bearing_to_dest2 = bearing_to_dest; 
                } else { 
                    bearing_to_dest2 = test_loc.get_bearing_to(destination) * 0.01f;
                }
                float distance2 = constrain_float(lookahead_step2_dist, OA_BENDYRULER_LOOKAHEAD_STEP2_MIN, test_loc.get_distance(destination));

                for (uint8_t j = 0; j < ARRAY_SIZE(test_pitch_step2); j++) {
                    float bearing_test2 = wrap_180(test_pitch_step2[j]);
                    Location test_loc2 = test_loc;
                    test_loc2.offset_bearing_and_pitch(bearing_to_dest2, bearing_test2, distance2);

                    // 计算此场景下与围栏和障碍物的最小边距
                    float margin2 = calc_avoidance_margin(test_loc, test_loc2, proximity_only);
                    if (margin2 > _margin_max) {
                        // 如果选择的方向直接朝向目标点我们可能会关闭避障
                        // i == 0 && j == 0 表示没有偏离朝向目标点的方向
                        bool active = (i != 0 || j != 0);
                        if (!active) {
                            // 进行子测试检查近距离障碍物以确认是否真的应该关闭BendyRuler
                            const float sub_test_pitch_step2[] {-90.0f, 90.0f};
                            for (uint8_t k = 0; k < ARRAY_SIZE(sub_test_pitch_step2); k++) {
                                Location test_loc_sub_test = test_loc;
                                test_loc_sub_test.offset_bearing_and_pitch(bearing_to_dest2, sub_test_pitch_step2[k], _margin_max);
                                float margin_sub_test = calc_avoidance_margin(test_loc, test_loc_sub_test, true);
                                if (margin_sub_test < _margin_max) {
                                    // BendyRuler将保持激活
                                    active = true;
                                    break;
                                }
                            }
                        }
                        // 按完整距离在选择的方向上投影
                        destination_new = current_loc;
                        destination_new.offset_bearing_and_pitch(bearing_to_dest, pitch_delta, distance_to_dest);
                        _current_lookahead = MIN(_lookahead, _current_lookahead * 1.1f);

                        Write_OABendyRuler((uint8_t)OABendyType::OA_BENDY_VERTICAL, active, bearing_to_dest, pitch_delta, false, margin, destination, destination_new);
                        return active;
                    }
                }
            }
        }        
    }   

    float chosen_pitch;
    if (have_best_pitch) {
        // 测试的方向都不适合2步检查。选择第一步最好的方向
        chosen_pitch = best_pitch;
        _current_lookahead = MIN(_lookahead, _current_lookahead * 1.05f);
    } else {
        // 所有可能的路径边距都为负。选择边距最大的
        chosen_pitch = best_margin_pitch;
        _current_lookahead = MAX(_lookahead * 0.5f, _current_lookahead * 0.9f);
    }

    // 基于最佳尝试计算新目标
    destination_new = current_loc;
    destination_new.offset_bearing_and_pitch(bearing_to_dest, chosen_pitch, distance_to_dest);

    // 记录结果
    Write_OABendyRuler((uint8_t)OABendyType::OA_BENDY_VERTICAL, true, bearing_to_dest, chosen_pitch,false, best_margin, destination, destination_new);

    return true;
}

// 获取BendyRuler类型
AP_OABendyRuler::OABendyType AP_OABendyRuler::get_type() const
{
    switch (_bendy_type) {
        case (uint8_t)OABendyType::OA_BENDY_VERTICAL:
        #if VERTICAL_ENABLED 
            return OABendyType::OA_BENDY_VERTICAL;
        #endif

        case (uint8_t)OABendyType::OA_BENDY_HORIZONTAL:
        default:
            return OABendyType::OA_BENDY_HORIZONTAL;
    }
    // 永远不会到达这里
    return OABendyType::OA_BENDY_HORIZONTAL;
}

/*
当BendyRuler找到一个至少在lookahead_step1_dist和lookahead_step2_dist处无障碍的方位角时调用此函数
在许多情况下,这个新方位角可以在障碍物的左边或右边,BendyRuler可能难以在两者之间做出选择。
如果在新迭代中获得的边距稍好一些,它就倾向于来回移动车辆。
因此,此方法试图避免将车辆的方向改变超过_bendy_angle度,
除非新边距至少是之前计算的方位角边距的_bendy_ratio倍。
如果我们抵制了改变并将遵循上次计算的方位角,则返回true。
*/
bool AP_OABendyRuler::resist_bearing_change(const Location &destination, const Location &current_loc, bool active, float bearing_test, float lookahead_step1_dist, float margin, Location &prev_dest, float &prev_bearing, float &final_bearing, float &final_margin, bool proximity_only) const
{      
    bool resisted_change = false;
    // 看看目标点是否改变,如果改变则不抵制方位角改变
    bool dest_change = false;
    if (!destination.same_latlon_as(prev_dest)) {
        dest_change = true;
        prev_dest = destination;
    }
                        
    // 检查是否需要抵制车辆方向的改变。如果我们有一条通向目标点的清晰路径,则无论如何都要去那里
    if (active && !dest_change && is_positive(_bendy_ratio)) { 
        // 检查新计算的方位角与之前存储的BendyRuler方位角之间的变化
        if ((fabsf(wrap_180(prev_bearing-bearing_test)) > _bendy_angle) && (!is_equal(prev_bearing,FLT_MAX))) {
            // 检查上一个方位角方向的边距
            Location test_loc_previous_bearing = current_loc;
            test_loc_previous_bearing.offset_bearing(wrap_180(prev_bearing), lookahead_step1_dist);
            float previous_bearing_margin = calc_avoidance_margin(current_loc,test_loc_previous_bearing, proximity_only);

            if (margin < (_bendy_ratio * previous_bearing_margin)) {
                // 不要突然改变方向。如果边距差异不显著,则遵循上一个方向
                final_bearing = prev_bearing;
                final_margin  = previous_bearing_margin;
                resisted_change = true;
            } 
        } 
    } else {
        // 如果BendyRuler未激活或航点已改变,则重置存储的方位角以避免不必要的路径改变阻力
        prev_bearing = FLT_MAX;
    }
    if (!resisted_change) {
        // 我们没有抵制改变,因此存储BendyRuler当前计算的方位角以供将来迭代使用
        prev_bearing = bearing_test;
    }

    return resisted_change;
}

// 计算线段与任何障碍物之间的最小距离
float AP_OABendyRuler::calc_avoidance_margin(const Location &start, const Location &end, bool proximity_only) const
{
    float margin_min = FLT_MAX;

    float latest_margin;
    
    if (calc_margin_from_object_database(start, end, latest_margin)) {
        margin_min = MIN(margin_min, latest_margin);
    }
    
    if (proximity_only) {
        // 只需要近距离数据的边距
        return margin_min;
    }
    
    if (calc_margin_from_circular_fence(start, end, latest_margin)) {
        margin_min = MIN(margin_min, latest_margin);
    }
    
    #if VERTICAL_ENABLED 
    // 高度围栏只在垂直避障中需要
    if (get_type() == OABendyType::OA_BENDY_VERTICAL) {
        if (calc_margin_from_alt_fence(start, end, latest_margin)) {
            margin_min = MIN(margin_min, latest_margin);
        }
    }
    #endif

    if (calc_margin_from_inclusion_and_exclusion_polygons(start, end, latest_margin)) {
        margin_min = MIN(margin_min, latest_margin);
    }

    if (calc_margin_from_inclusion_and_exclusion_circles(start, end, latest_margin)) {
        margin_min = MIN(margin_min, latest_margin);
    }

    // 返回与任何障碍物的最小边距
    return margin_min;
}

// 计算路径与圆形围栏(以home点为中心)之间的最小距离
// 成功时返回true并更新margin
bool AP_OABendyRuler::calc_margin_from_circular_fence(const Location &start, const Location &end, float &margin) const
{
#if AP_FENCE_ENABLED
    // 如果多边形围栏未启用则立即退出
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return false;
    }
    if ((fence->get_enabled_fences() & AC_FENCE_TYPE_CIRCLE) == 0) {
        return false;
    }

    // 计算起点和终点到home点的距离
    const Location &ahrs_home = AP::ahrs().get_home();
    const float start_dist_sq = ahrs_home.get_distance_NE(start).length_squared();
    const float end_dist_sq = ahrs_home.get_distance_NE(end).length_squared();

    // 获取圆形围栏半径+边距
    const float fence_radius_plus_margin = fence->get_radius() - fence->get_margin();

    // 边距是围栏半径减去起点或终点距离中的较大值
    margin = fence_radius_plus_margin - sqrtf(MAX(start_dist_sq, end_dist_sq));
    return true;
#else
    return false;
#endif // AP_FENCE_ENABLED
}

// 计算路径与高度围栏之间的最小距离
// 成功时返回true并更新margin
// calculate minimum distance between a path and the altitude fence
// on success returns true and updates margin
bool AP_OABendyRuler::calc_margin_from_alt_fence(const Location &start, const Location &end, float &margin) const
{
#if AP_FENCE_ENABLED
    // 如果高度围栏未启用则立即退出
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return false;
    }
    if ((fence->get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) == 0) {
        return false;
    }

    // 获取起点和终点相对home点的高度(厘米)
    int32_t alt_above_home_cm_start, alt_above_home_cm_end;    
    if (!start.get_alt_cm(Location::AltFrame::ABOVE_HOME, alt_above_home_cm_start)) {
        return false;
    }
    if (!end.get_alt_cm(Location::AltFrame::ABOVE_HOME, alt_above_home_cm_end )) {
        return false;
    }

    // 安全最大高度 = 围栏高度 - 围栏边距
    const float max_fence_alt = fence->get_safe_alt_max();
    // 计算起点和终点到最大高度的边距
    const float margin_start =  max_fence_alt - alt_above_home_cm_start * 0.01f;
    const float margin_end =  max_fence_alt - alt_above_home_cm_end * 0.01f;

    // 边距是起点或终点到围栏的最小距离
    margin = MIN(margin_start,margin_end);

    return true;
#else
    return false;
#endif // AP_FENCE_ENABLED
}

// 计算路径与所有包含和排除多边形之间的最小距离
// 成功时返回true并更新margin
bool AP_OABendyRuler::calc_margin_from_inclusion_and_exclusion_polygons(const Location &start, const Location &end, float &margin) const
{
#if AP_FENCE_ENABLED
    // 获取围栏单例
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return false;
    }

    // 检查多边形围栏是否启用
    if ((fence->get_enabled_fences() & AC_FENCE_TYPE_POLYGON) == 0) {
        return false;
    }

    // 如果没有包含和排除多边形则立即返回
    const uint8_t num_inclusion_polygons = fence->polyfence().get_inclusion_polygon_count();
    const uint8_t num_exclusion_polygons = fence->polyfence().get_exclusion_polygon_count();
    if ((num_inclusion_polygons == 0) && (num_exclusion_polygons == 0)) {
        return false;
    }

    // 将起点和终点转换为相对EKF原点的偏移量
    Vector2f start_NE, end_NE;
    if (!start.get_vector_xy_from_origin_NE(start_NE) || !end.get_vector_xy_from_origin_NE(end_NE)) {
        return false;
    }

    // 获取围栏边距
    const float fence_margin = fence->get_margin();

    // 遍历包含多边形并计算最小边距
    bool margin_updated = false;
    for (uint8_t i = 0; i < num_inclusion_polygons; i++) {
        uint16_t num_points;
        const Vector2f* boundary = fence->polyfence().get_inclusion_polygon(i, num_points);
     
        // 如果在围栏外,边距是最近距离但带负号
        const float sign = Polygon_outside(start_NE, boundary, num_points) ? -1.0f : 1.0f;

        // 计算线段到多边形的最小距离(米)
        float margin_new = (sign * Polygon_closest_distance_line(boundary, num_points, start_NE, end_NE) * 0.01f) - fence_margin;
        if (!margin_updated || (margin_new < margin)) {
            margin_updated = true;
            margin = margin_new;
        }
    }

    // 遍历排除多边形并计算最小边距
    for (uint8_t i = 0; i < num_exclusion_polygons; i++) {
        uint16_t num_points;
        const Vector2f* boundary = fence->polyfence().get_exclusion_polygon(i, num_points);
   
        // 如果起点在多边形内,边距的符号反转
        const float sign = Polygon_outside(start_NE, boundary, num_points) ? 1.0f : -1.0f;

        // 计算线段到多边形的最小距离(米)
        float margin_new = (sign * Polygon_closest_distance_line(boundary, num_points, start_NE, end_NE) * 0.01f) - fence_margin;
        if (!margin_updated || (margin_new < margin)) {
            margin_updated = true;
            margin = margin_new;
        }
    }

    return margin_updated;
#else
    return false;
#endif // AP_FENCE_ENABLED
}

// 计算路径与所有包含和排除圆之间的最小距离
// 成功时返回true并更新margin
bool AP_OABendyRuler::calc_margin_from_inclusion_and_exclusion_circles(const Location &start, const Location &end, float &margin) const
{
#if AP_FENCE_ENABLED
    // 如果围栏未启用则立即退出
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return false;
    }

    // 检查多边形围栏是否启用
    if ((fence->get_enabled_fences() & AC_FENCE_TYPE_POLYGON) == 0) {
        return false;
    }

    // 如果没有包含和排除圆则立即返回
    const uint8_t num_inclusion_circles = fence->polyfence().get_inclusion_circle_count();
    const uint8_t num_exclusion_circles = fence->polyfence().get_exclusion_circle_count();
    if ((num_inclusion_circles == 0) && (num_exclusion_circles == 0)) {
        return false;
    }

    // 将起点和终点转换为相对EKF原点的偏移量
    Vector2f start_NE, end_NE;
    if (!start.get_vector_xy_from_origin_NE(start_NE) || !end.get_vector_xy_from_origin_NE(end_NE)) {
        return false;
    }

    // 获取围栏边距
    const float fence_margin = fence->get_margin();

    // 遍历包含圆并计算最小边距
    bool margin_updated = false;
    for (uint8_t i = 0; i < num_inclusion_circles; i++) {
        Vector2f center_pos_cm;
        float radius;
        if (fence->polyfence().get_inclusion_circle(i, center_pos_cm, radius)) {

            // 计算起点和终点到圆心的距离
            const float start_dist_sq = (start_NE - center_pos_cm).length_squared();
            const float end_dist_sq = (end_NE - center_pos_cm).length_squared();

            // 边距是围栏半径减去起点或终点距离中的较大值
            const float margin_new = (radius + fence_margin) - (sqrtf(MAX(start_dist_sq, end_dist_sq)) * 0.01f);

            // 更新最小边距
            if (!margin_updated || (margin_new < margin)) {
                margin_updated = true;
                margin = margin_new;
            }
        }
    }

    // 遍历排除圆并计算最小边距
    for (uint8_t i = 0; i < num_exclusion_circles; i++) {
        Vector2f center_pos_cm;
        float radius;
        if (fence->polyfence().get_exclusion_circle(i, center_pos_cm, radius)) {

            // 首先计算圆心到线段的距离
            const float dist_cm = Vector2f::closest_distance_between_line_and_point(start_NE, end_NE, center_pos_cm);

            // 边距是到圆心的距离减去半径
            const float margin_new = (dist_cm * 0.01f) - (radius + fence_margin);

            // 更新最小边距
            if (!margin_updated || (margin_new < margin)) {
                margin_updated = true;
                margin = margin_new;
            }
        }
    }

    return margin_updated;
#else
    return false;
#endif // AP_FENCE_ENABLED
}

// 计算路径与近距离传感器障碍物之间的最小距离
// 成功时返回true并更新margin
bool AP_OABendyRuler::calc_margin_from_object_database(const Location &start, const Location &end, float &margin) const
{
    // 如果数据库为空则立即退出
    AP_OADatabase *oaDb = AP::oadatabase();
    if (oaDb == nullptr || !oaDb->healthy()) {
        return false;
    }

    // 将起点和终点转换为相对EKF原点的偏移量(厘米)
    Vector3f start_NEU,end_NEU;
    if (!start.get_vector_from_origin_NEU(start_NEU) || !end.get_vector_from_origin_NEU(end_NEU)) {
        return false;
    }
    if (start_NEU == end_NEU) {
        return false;
    }

    // 检查每个障碍物到线段的距离
    float smallest_margin = FLT_MAX;
    for (uint16_t i=0; i<oaDb->database_count(); i++) {
        const AP_OADatabase::OA_DbItem& item = oaDb->get_item(i);
        const Vector3f point_cm = item.pos * 100.0f;
        // 边距是线段到障碍物的距离减去障碍物的半径
        const float m = Vector3f::closest_distance_between_line_and_point(start_NEU, end_NEU, point_cm) * 0.01f - item.radius;
        if (m < smallest_margin) {
            smallest_margin = m;
        }
    }

    // 返回最小边距
    if (smallest_margin < FLT_MAX) {
        margin = smallest_margin;
        return true;
    }

    return false;
}

#endif  // AP_OAPATHPLANNER_BENDYRULER_ENABLED
