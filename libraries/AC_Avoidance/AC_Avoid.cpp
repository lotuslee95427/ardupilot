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

#include "AC_Avoidance_config.h"

#if AP_AVOIDANCE_ENABLED

#include "AC_Avoid.h"
#include <AP_AHRS/AP_AHRS.h>     // AHRS library
#include <AC_Fence/AC_Fence.h>         // Failsafe fence library
#include <AP_Proximity/AP_Proximity.h>
#include <AP_Beacon/AP_Beacon.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <stdio.h>

#if !APM_BUILD_TYPE(APM_BUILD_ArduPlane)

// 根据不同的车辆类型设置默认避障行为
#if APM_BUILD_TYPE(APM_BUILD_Rover)
 # define AP_AVOID_BEHAVE_DEFAULT AC_Avoid::BehaviourType::BEHAVIOR_STOP
#else
 # define AP_AVOID_BEHAVE_DEFAULT AC_Avoid::BehaviourType::BEHAVIOR_SLIDE
#endif

// 对于直升机和多旋翼启用Z轴避障
#if APM_BUILD_COPTER_OR_HELI
    # define AP_AVOID_ENABLE_Z          1
#endif

// 参数定义
const AP_Param::GroupInfo AC_Avoid::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Avoidance control enable/disable
    // @Description: Enabled/disable avoidance input sources
    // @Bitmask: 0:UseFence,1:UseProximitySensor,2:UseBeaconFence
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1,  AC_Avoid, _enabled, AC_AVOID_DEFAULT, AP_PARAM_FLAG_ENABLE),

    // @Param{Copter}: ANGLE_MAX
    // @DisplayName: Avoidance max lean angle in non-GPS flight modes
    // @Description: Max lean angle used to avoid obstacles while in non-GPS modes
    // @Units: cdeg
    // @Increment: 10
    // @Range: 0 4500
    // @User: Standard
    AP_GROUPINFO_FRAME("ANGLE_MAX", 2,  AC_Avoid, _angle_max, 1000, AP_PARAM_FRAME_COPTER | AP_PARAM_FRAME_HELI | AP_PARAM_FRAME_TRICOPTER),

    // @Param{Copter}: DIST_MAX
    // @DisplayName: Avoidance distance maximum in non-GPS flight modes
    // @Description: Distance from object at which obstacle avoidance will begin in non-GPS modes
    // @Units: m
    // @Range: 1 30
    // @User: Standard
    AP_GROUPINFO_FRAME("DIST_MAX", 3,  AC_Avoid, _dist_max, AC_AVOID_NONGPS_DIST_MAX_DEFAULT, AP_PARAM_FRAME_COPTER | AP_PARAM_FRAME_HELI | AP_PARAM_FRAME_TRICOPTER),

    // @Param: MARGIN
    // @DisplayName: Avoidance distance margin in GPS modes
    // @Description: Vehicle will attempt to stay at least this distance (in meters) from objects while in GPS modes
    // @Units: m
    // @Range: 1 10
    // @User: Standard
    AP_GROUPINFO("MARGIN", 4, AC_Avoid, _margin, 2.0f),

    // @Param{Copter, Rover}: BEHAVE
    // @DisplayName: Avoidance behaviour
    // @Description: Avoidance behaviour (slide or stop)
    // @Values: 0:Slide,1:Stop
    // @User: Standard
    AP_GROUPINFO_FRAME("BEHAVE", 5, AC_Avoid, _behavior, AP_AVOID_BEHAVE_DEFAULT, AP_PARAM_FRAME_COPTER | AP_PARAM_FRAME_HELI | AP_PARAM_FRAME_TRICOPTER | AP_PARAM_FRAME_ROVER),

    // @Param: BACKUP_SPD
    // @DisplayName: Avoidance maximum horizontal backup speed
    // @Description: Maximum speed that will be used to back away from obstacles horizontally in position control modes (m/s). Set zero to disable horizontal backup.
    // @Units: m/s
    // @Range: 0 2
    // @User: Standard
    AP_GROUPINFO("BACKUP_SPD", 6, AC_Avoid, _backup_speed_xy_max, 0.75f),

    // @Param{Copter}: ALT_MIN
    // @DisplayName: Avoidance minimum altitude
    // @Description: Minimum altitude above which proximity based avoidance will start working. This requires a valid downward facing rangefinder reading to work. Set zero to disable
    // @Units: m
    // @Range: 0 6
    // @User: Standard
    AP_GROUPINFO_FRAME("ALT_MIN", 7, AC_Avoid, _alt_min, 0.0f, AP_PARAM_FRAME_COPTER | AP_PARAM_FRAME_HELI | AP_PARAM_FRAME_TRICOPTER),

    // @Param: ACCEL_MAX
    // @DisplayName: Avoidance maximum acceleration
    // @Description: Maximum acceleration with which obstacles will be avoided with. Set zero to disable acceleration limits
    // @Units: m/s/s
    // @Range: 0 9
    // @User: Standard
    AP_GROUPINFO("ACCEL_MAX", 8, AC_Avoid, _accel_max, 3.0f),

    // @Param: BACKUP_DZ
    // @DisplayName: Avoidance deadzone between stopping and backing away from obstacle
    // @Description: Distance beyond AVOID_MARGIN parameter, after which vehicle will backaway from obstacles. Increase this parameter if you see vehicle going back and forth in front of obstacle.
    // @Units: m
    // @Range: 0 2
    // @User: Standard
    AP_GROUPINFO("BACKUP_DZ", 9, AC_Avoid, _backup_deadzone, 0.10f),

    // @Param: BACKZ_SPD
    // @DisplayName: Avoidance maximum vertical backup speed
    // @Description: Maximum speed that will be used to back away from obstacles vertically in height control modes (m/s). Set zero to disable vertical backup.
    // @Units: m/s
    // @Range: 0 2
    // @User: Standard
    AP_GROUPINFO("BACKZ_SPD", 10, AC_Avoid, _backup_speed_z_max, 0.75),

    AP_GROUPEND
};

/// 构造函数
AC_Avoid::AC_Avoid()
{
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

/*
* 此方法限制速度并计算从各种支持的围栏退后的速度
* 同时使用adjust_velocity_z方法限制垂直速度
*/
void AC_Avoid::adjust_velocity_fence(float kP, float accel_cmss, Vector3f &desired_vel_cms, Vector3f &backup_vel, float kP_z, float accel_cmss_z, float dt)
{   
    // 对于大多数围栏只需要水平分量,因为围栏是2D的
    Vector2f desired_velocity_xy_cms{desired_vel_cms.x, desired_vel_cms.y};

#if AP_FENCE_ENABLED || AP_BEACON_ENABLED
    // 限制加速度
    const float accel_cmss_limited = MIN(accel_cmss, AC_AVOID_ACCEL_CMSS_MAX);
#endif

    // 每个象限中期望的最大备用速度分量
    Vector2f quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel;

#if AP_FENCE_ENABLED
    if ((_enabled & AC_AVOID_STOP_AT_FENCE) > 0) {
        // 存储需要从围栏退后的速度
        Vector2f backup_vel_fence;

        adjust_velocity_circle_fence(kP, accel_cmss_limited, desired_velocity_xy_cms, backup_vel_fence, dt);
        find_max_quadrant_velocity(backup_vel_fence, quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel);
        
        // 每次围栏检查后将backup_vel_fence设为零,以防之前的方法未设置速度
        backup_vel_fence.zero();
        adjust_velocity_inclusion_and_exclusion_polygons(kP, accel_cmss_limited, desired_velocity_xy_cms, backup_vel_fence, dt);
        find_max_quadrant_velocity(backup_vel_fence, quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel);
        
        backup_vel_fence.zero();
        adjust_velocity_inclusion_circles(kP, accel_cmss_limited, desired_velocity_xy_cms, backup_vel_fence, dt);
        find_max_quadrant_velocity(backup_vel_fence, quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel);
        
        backup_vel_fence.zero();
        adjust_velocity_exclusion_circles(kP, accel_cmss_limited, desired_velocity_xy_cms, backup_vel_fence, dt);
        find_max_quadrant_velocity(backup_vel_fence, quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel);
    }
#endif // AP_FENCE_ENABLED

#if AP_BEACON_ENABLED
    if ((_enabled & AC_AVOID_STOP_AT_BEACON_FENCE) > 0) {
        // 存储需要从信标围栏退后的速度
        Vector2f backup_vel_beacon;
        adjust_velocity_beacon_fence(kP, accel_cmss_limited, desired_velocity_xy_cms, backup_vel_beacon, dt);
        find_max_quadrant_velocity(backup_vel_beacon, quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel);
    }
#endif // AP_BEACON_ENABLED

    // 检查垂直围栏
    float desired_velocity_z_cms = desired_vel_cms.z;
    float desired_backup_vel_z = 0.0f;
    adjust_velocity_z(kP_z, accel_cmss_z, desired_velocity_z_cms, desired_backup_vel_z, dt);

    // 期望的备用速度是每个象限中最大速度分量的总和
    const Vector2f desired_backup_vel_xy = quad_1_back_vel + quad_2_back_vel + quad_3_back_vel + quad_4_back_vel;
    backup_vel = Vector3f{desired_backup_vel_xy.x, desired_backup_vel_xy.y, desired_backup_vel_z};
    desired_vel_cms = Vector3f{desired_velocity_xy_cms.x, desired_velocity_xy_cms.y, desired_velocity_z_cms};
}

/*
* 调整期望速度,使车辆能在围栏/物体前停止
* kP, accel_cmss 用于水平轴
* kP_z, accel_cmss_z 用于垂直轴
*/
void AC_Avoid::adjust_velocity(Vector3f &desired_vel_cms, bool &backing_up, float kP, float accel_cmss, float kP_z, float accel_cmss_z, float dt)
{
    // 如果禁用则立即退出
    if (_enabled == AC_AVOID_DISABLED) {
        return;
    }

    // 复制输入速度,因为desired_vel_cms可能会改变
    const Vector3f desired_vel_cms_original = desired_vel_cms;

    // 限制加速度
    const float accel_cmss_limited = MIN(accel_cmss, AC_AVOID_ACCEL_CMSS_MAX);

    // 每个象限中水平期望备用速度的最大分量
    Vector2f quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel;
    float back_vel_up = 0.0f;
    float back_vel_down = 0.0f;
    
    // 响应近距离传感器的避障
    if (proximity_avoidance_enabled() && _proximity_alt_enabled) {
        // 存储需要从物理障碍物退后的速度
        Vector3f backup_vel_proximity;
        adjust_velocity_proximity(kP, accel_cmss_limited, desired_vel_cms, backup_vel_proximity, kP_z,accel_cmss_z, dt);
        find_max_quadrant_velocity_3D(backup_vel_proximity, quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel, back_vel_up, back_vel_down);
    }
    
    // 响应各种围栏的避障
    Vector3f backup_vel_fence;
    adjust_velocity_fence(kP, accel_cmss, desired_vel_cms, backup_vel_fence, kP_z, accel_cmss_z, dt);
    find_max_quadrant_velocity_3D(backup_vel_fence , quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel, back_vel_up, back_vel_down);
    
    // 期望的备用速度是每个象限中最大速度分量的总和
    const Vector2f desired_backup_vel_xy = quad_1_back_vel + quad_2_back_vel + quad_3_back_vel + quad_4_back_vel;
    const float desired_backup_vel_z = back_vel_down + back_vel_up;
    Vector3f desired_backup_vel{desired_backup_vel_xy.x, desired_backup_vel_xy.y, desired_backup_vel_z};

    const float max_back_spd_xy_cms = _backup_speed_xy_max * 100.0;
    if (!desired_backup_vel.xy().is_zero() && is_positive(max_back_spd_xy_cms)) {
        backing_up = true;
        // 限制水平退后速度
        desired_backup_vel.xy().limit_length(max_back_spd_xy_cms);

        // 如果用户以更大的速度退后,则让用户控制
        // 这必须分别对x,y,z进行。例如,用户在"x"方向做得很好,但可能需要在"y"方向退后
        if (!is_zero(desired_backup_vel.x)) {
            if (is_positive(desired_backup_vel.x)) {
                desired_vel_cms.x = MAX(desired_vel_cms.x, desired_backup_vel.x);
            } else {
                desired_vel_cms.x = MIN(desired_vel_cms.x, desired_backup_vel.x);
            }
        }
        if (!is_zero(desired_backup_vel.y)) {
            if (is_positive(desired_backup_vel.y)) {
                desired_vel_cms.y = MAX(desired_vel_cms.y, desired_backup_vel.y);
            } else {
                desired_vel_cms.y = MIN(desired_vel_cms.y, desired_backup_vel.y);
            }
        }
    }

    const float max_back_spd_z_cms = _backup_speed_z_max * 100.0;
    if (!is_zero(desired_backup_vel.z) && is_positive(max_back_spd_z_cms)) {
        backing_up = true;

        // 限制垂直退后速度
        desired_backup_vel.z = constrain_float(desired_backup_vel.z, -max_back_spd_z_cms, max_back_spd_z_cms);

        if (!is_zero(desired_backup_vel.z)) {
            if (is_positive(desired_backup_vel.z)) {
                desired_vel_cms.z = MAX(desired_vel_cms.z, desired_backup_vel.z);
            } else {
                desired_vel_cms.z = MIN(desired_vel_cms.z, desired_backup_vel.z);
            }
        }
    }

    // 限制加速度
    limit_accel(desired_vel_cms_original, desired_vel_cms, dt);

    if (desired_vel_cms_original != desired_vel_cms) {
        _last_limit_time = AP_HAL::millis();
    }

#if HAL_LOGGING_ENABLED
    if (limits_active()) {
        // 以不超过10hz的频率记录(adjust_velocity方法可能以400hz的频率调用!)
        uint32_t now = AP_HAL::millis();
        if ((now - _last_log_ms) > 100) {
            _last_log_ms = now;
            Write_SimpleAvoidance(true, desired_vel_cms_original, desired_vel_cms, backing_up);
        }
    } else {
        // 避障不再活动
        // 记录一次以便在日志中注册
        if (_last_log_ms) {
            Write_SimpleAvoidance(false, desired_vel_cms_original, desired_vel_cms, backing_up);
            // 这确保在避障再次激活之前不会再次记录
            _last_log_ms = 0;
        }
    }
#endif
}

/*
* 限制加速度,使避障库输出的速度变化受控
* 这有助于减少车辆的抖动和突然移动
*/
void AC_Avoid::limit_accel(const Vector3f &original_vel, Vector3f &modified_vel, float dt)
{
    if (original_vel == modified_vel || is_zero(_accel_max) || !is_positive(dt)) {
        // 如果满足这些条件之一,我们就不能限制加速度
        return;
    }

    if (AP_HAL::millis() - _last_limit_time > AC_AVOID_ACCEL_TIMEOUT_MS) {
        // 重置这个速度,因为避障已经很长时间没有激活了
        _prev_avoid_vel = original_vel;
    }

    // 避障要求的加速度
    const Vector3f accel = (modified_vel - _prev_avoid_vel)/dt;

    // 最大加速度(厘米)
    const float max_accel_cm = _accel_max * 100.0f;

    if (accel.length() > max_accel_cm) {
        // 减小加速度
        const Vector3f accel_direction = accel.normalized();
        modified_vel = (accel_direction * max_accel_cm) * dt + _prev_avoid_vel;
    }

    _prev_avoid_vel = modified_vel;
    return;
}

// 此方法在大多数Rover模式中使用,而不在Copter中使用
// 调整期望的水平速度,使车辆在围栏或物体前停止
// accel(最大加速度/减速度)单位为m/s/s
// heading单位为弧度
// speed单位为m/s
// kP应为零以获得线性响应,非零以获得非线性响应
void AC_Avoid::adjust_speed(float kP, float accel, float heading, float &speed, float dt)
{
    // 将航向和速度转换为速度向量
    Vector3f vel{
        cosf(heading) * speed * 100.0f,
        sinf(heading) * speed * 100.0f,
        0.0f
    };

    bool backing_up  = false;
    adjust_velocity(vel, backing_up, kP, accel * 100.0f, 0, 0, dt);
    const Vector2f vel_xy{vel.x, vel.y};

    if (backing_up) {
        // 退后
        if (fabsf(wrap_180(degrees(vel_xy.angle())) - degrees(heading)) > 90.0f) {
            // 速度向量方向和实际航向之间的差异很大,因此我们需要反转方向
            speed = -vel_xy.length() * 0.01f;
        } else {
            speed = vel_xy.length() * 0.01f;
        }
        return;
    }

    // 无需退后,因此根据需要将速度调整为零
    if (is_negative(speed)) {
        speed = -vel_xy.length() * 0.01f;
    } else {
        speed = vel_xy.length() * 0.01f;
    }
}

// 调整垂直爬升率,使车辆不会突破垂直围栏
void AC_Avoid::adjust_velocity_z(float kP, float accel_cmss, float& climb_rate_cms, float dt) {
    float backup_speed = 0.0f;
    adjust_velocity_z(kP, accel_cmss, climb_rate_cms, backup_speed, dt);
    if (!is_zero(backup_speed)) {
        if (is_negative(backup_speed)) {
            climb_rate_cms = MIN(climb_rate_cms, backup_speed);
        } else {
            climb_rate_cms = MAX(climb_rate_cms, backup_speed);
        }
    }
}

// 调整垂直爬升率,使车辆不会突破垂直围栏
void AC_Avoid::adjust_velocity_z(float kP, float accel_cmss, float& climb_rate_cms, float& backup_speed, float dt)
{
#ifdef AP_AVOID_ENABLE_Z

    // 如果禁用则立即退出
    if (_enabled == AC_AVOID_DISABLED) {
        return;
    }
    
    // 如果水平则不调整climb_rate
    if (is_zero(climb_rate_cms)) {
        return;
    }

    const AP_AHRS &_ahrs = AP::ahrs();
    // 限制加速度
    const float accel_cmss_limited = MIN(accel_cmss, AC_AVOID_ACCEL_CMSS_MAX);

    bool limit_min_alt = false;
    bool limit_max_alt = false;
    float max_alt_diff = 0.0f; // 车辆到高度限制的距离(米)(正值表示车辆在限制以下)
    float min_alt_diff = 0.0f;
#if AP_FENCE_ENABLED
    // 计算到围栏的距离
    AC_Fence *fence = AP::fence();
    if ((_enabled & AC_AVOID_STOP_AT_FENCE) > 0 && fence) {
        // 计算车辆到安全高度的距离
        float veh_alt;
        _ahrs.get_relative_position_D_home(veh_alt);
        if ((fence->get_enabled_fences() & AC_FENCE_TYPE_ALT_MIN) > 0) {
            // fence.get_safe_alt_max()向上,veh_alt向下:
            min_alt_diff = -(fence->get_safe_alt_min() + veh_alt);
            limit_min_alt = true;
        }
        if ((fence->get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) > 0) {
            // fence.get_safe_alt_max()向上,veh_alt向下:
            max_alt_diff = fence->get_safe_alt_max() + veh_alt;
            limit_max_alt = true;
        }
    }
#endif

    // 计算到(例如)光流高度限制的距离
    // AHRS值始终以米为单位
    float alt_limit;
    float curr_alt;
    if (_ahrs.get_hgt_ctrl_limit(alt_limit) &&
        _ahrs.get_relative_position_D_origin(curr_alt)) {
        // alt_limit向上,curr_alt向下:
        const float ctrl_alt_diff = alt_limit + curr_alt;
        if (!limit_max_alt || ctrl_alt_diff < max_alt_diff) {
            max_alt_diff = ctrl_alt_diff;
            limit_max_alt = true;
        }
    }

#if HAL_PROXIMITY_ENABLED
    // 从近距离传感器获取距离
    float proximity_alt_diff;
    AP_Proximity *proximity = AP::proximity();
    if (proximity && proximity_avoidance_enabled() && proximity->get_upward_distance(proximity_alt_diff)) {
        proximity_alt_diff -= _margin;
        if (!limit_max_alt || proximity_alt_diff < max_alt_diff) {
            max_alt_diff = proximity_alt_diff;
            limit_max_alt = true;
        }
    }
#endif

    // 限制爬升率
    if (limit_max_alt || limit_min_alt) {
        const float max_back_spd_cms = _backup_speed_z_max * 100.0;
        // 如果我们已经超过安全高度,则不允许爬升
        if (max_alt_diff <= 0.0f && limit_max_alt) {
            climb_rate_cms = MIN(climb_rate_cms, 0.0f);
            // 同时计算将使我们回到安全高度的备用速度
            if (is_positive(max_back_spd_cms)) {
                backup_speed = -1*(get_max_speed(kP, accel_cmss_limited, -max_alt_diff*100.0f, dt));

                // 限制在最大备用速度内
                backup_speed = MAX(backup_speed, -max_back_spd_cms);
            }
            return;
        // 如果我们已经超过安全高度,则不允许下降
        } else if (min_alt_diff <= 0.0f && limit_min_alt) {
            climb_rate_cms =  MAX(climb_rate_cms, 0.0f);
            // 同时计算将使我们回到安全高度的备用速度
            if (is_positive(max_back_spd_cms)) {
                backup_speed = get_max_speed(kP, accel_cmss_limited, -min_alt_diff*100.0f, dt);

                // 限制在最大备用速度内
                backup_speed = MIN(backup_speed, max_back_spd_cms);
            }
            return;
        }

        // 限制爬升率
        if (limit_max_alt) {
            const float max_alt_max_speed = get_max_speed(kP, accel_cmss_limited, max_alt_diff*100.0f, dt);
            climb_rate_cms = MIN(max_alt_max_speed, climb_rate_cms);
        }

        if (limit_min_alt) {
            const float max_alt_min_speed = get_max_speed(kP, accel_cmss_limited, min_alt_diff*100.0f, dt);
            climb_rate_cms = MAX(-max_alt_min_speed, climb_rate_cms);
        }
    }
#endif
}

// 调整横滚-俯仰以将车辆推离物体
// 横滚和俯仰值以厘度为单位
void AC_Avoid::adjust_roll_pitch(float &roll, float &pitch, float veh_angle_max)
{
    // 如果禁用基于近距离的避障,则立即退出
    if (!proximity_avoidance_enabled()) {
        return;
    }

    // 如果最大角度为零则立即退出
    if (_angle_max <= 0.0f || veh_angle_max <= 0.0f) {
        return;
    }

    float roll_positive = 0.0f;    // 最大正横滚值
    float roll_negative = 0.0f;    // 最小负横滚值
    float pitch_positive = 0.0f;   // 最大正俯仰值
    float pitch_negative = 0.0f;   // 最小负俯仰值

    // 从近距离传感器获取最大正负横滚和俯仰百分比
    get_proximity_roll_pitch_pct(roll_positive, roll_negative, pitch_positive, pitch_negative);

    // 将横滚和俯仰的最大正负百分比相加,转换为厘度
    Vector2f rp_out((roll_positive + roll_negative) * 4500.0f, (pitch_positive + pitch_negative) * 4500.0f);

    // 应用避障角度限制
    // 物体避障倾斜角度永远不超过总角度限制的75%,以允许驾驶员覆盖
    const float angle_limit = constrain_float(_angle_max, 0.0f, veh_angle_max * AC_AVOID_ANGLE_MAX_PERCENT);
    float vec_len = rp_out.length();
    if (vec_len > angle_limit) {
        rp_out *= (angle_limit / vec_len);
    }
    // 将传入的横滚和俯仰角度相加
    rp_out.x += roll;
    rp_out.y += pitch;

    // 应用总角度限制
    vec_len = rp_out.length();
    if (vec_len > veh_angle_max) {
        rp_out *= (veh_angle_max / vec_len);
    }

    // 返回调整后的横滚和俯仰
    roll = rp_out.x;
    pitch = rp_out.y;
    // add passed in roll, pitch angles
    // 添加传入的横滚和俯仰角度
    rp_out.x += roll;
    rp_out.y += pitch;

    // apply total angular limits
    // 应用总角度限制
    vec_len = rp_out.length();
    if (vec_len > veh_angle_max) {
        rp_out *= (veh_angle_max / vec_len);
    }

    // return adjusted roll, pitch
    // 返回调整后的横滚和俯仰
    roll = rp_out.x;
    pitch = rp_out.y;
}

/*
 * Note: This method is used to limit velocity horizontally only 
 * Limits the component of desired_vel_cms in the direction of the unit vector
 * limit_direction to be at most the maximum speed permitted by the limit_distance_cm.
 *
 * Uses velocity adjustment idea from Randy's second email on this thread:
 * https://groups.google.com/forum/#!searchin/drones-discuss/obstacle/drones-discuss/QwUXz__WuqY/qo3G8iTLSJAJ
 */
void AC_Avoid::limit_velocity_2D(float kP, float accel_cmss, Vector2f &desired_vel_cms, const Vector2f& limit_direction, float limit_distance_cm, float dt)
{
    const float max_speed = get_max_speed(kP, accel_cmss, limit_distance_cm, dt);
    // project onto limit direction
    // 投影到限制方向
    const float speed = desired_vel_cms * limit_direction;
    if (speed > max_speed) {
        // subtract difference between desired speed and maximum acceptable speed
        // 减去期望速度和最大可接受速度之间的差值
        desired_vel_cms += limit_direction*(max_speed - speed);
    }
}

/*
 * Note: This method is used to limit velocity horizontally and vertically given a 3D desired velocity vector 
 * Limits the component of desired_vel_cms in the direction of the obstacle_vector based on the passed value of "margin"
 */
void AC_Avoid::limit_velocity_3D(float kP, float accel_cmss, Vector3f &desired_vel_cms, const Vector3f& obstacle_vector, float margin_cm, float kP_z, float accel_cmss_z, float dt)
{  
    if (desired_vel_cms.is_zero()) {
        // nothing to limit
        // 没有需要限制的内容
        return;
    }
    // create a margin_cm length vector in the direction of desired_vel_cms
    // this will create larger margin towards the direction vehicle is travelling in
    // 在desired_vel_cms方向上创建一个长度为margin_cm的向量
    // 这将在车辆行驶方向上创建更大的边距
    const Vector3f margin_vector = desired_vel_cms.normalized() * margin_cm;
    const Vector2f limit_direction_xy{obstacle_vector.x, obstacle_vector.y};
    
    if (!limit_direction_xy.is_zero()) {
        const float distance_from_fence_xy = MAX((limit_direction_xy.length() - Vector2f{margin_vector.x, margin_vector.y}.length()), 0.0f);
        Vector2f velocity_xy{desired_vel_cms.x, desired_vel_cms.y};
        limit_velocity_2D(kP, accel_cmss, velocity_xy, limit_direction_xy.normalized(), distance_from_fence_xy, dt);
        desired_vel_cms.x = velocity_xy.x;
        desired_vel_cms.y = velocity_xy.y;
    }
    
    if (is_zero(desired_vel_cms.z) || is_zero(obstacle_vector.z)) {
        // nothing to limit vertically if desired_vel_cms.z is zero
        // if obstacle_vector.z is zero then the obstacle is probably horizontally located, and we can move vertically
        // 如果desired_vel_cms.z为零，则没有需要垂直限制的内容
        // 如果obstacle_vector.z为零，则障碍物可能位于水平位置，我们可以垂直移动
        return;
    }

    if (is_positive(desired_vel_cms.z) != is_positive(obstacle_vector.z)) {
        // why limit velocity vertically when we are going the opposite direction
        // 当我们朝相反方向行驶时，为什么要垂直限制速度
        return;
    }
    
    // to check if Z velocity changes
    // 检查Z速度是否变化
    const float velocity_z_original = desired_vel_cms.z;
    const float z_speed = fabsf(desired_vel_cms.z);

    // obstacle_vector.z and margin_vector.z should be in same direction as checked above
    // obstacle_vector.z和margin_vector.z应该与上面检查的方向相同
    const float dist_z = MAX(fabsf(obstacle_vector.z) - fabsf(margin_vector.z), 0.0f); 
    if (is_zero(dist_z)) {
        // eliminate any vertical velocity 
        // 消除任何垂直速度
        desired_vel_cms.z = 0.0f;
    } else {
        const float max_z_speed = get_max_speed(kP_z, accel_cmss_z, dist_z, dt);
        desired_vel_cms.z = MIN(max_z_speed, z_speed);
    }

    // make sure the direction of the Z velocity did not change
    // we are only limiting speed here, not changing directions 
    // check if original z velocity is positive or negative
    // 确保Z速度的方向没有改变
    // 我们这里只是限制速度，而不是改变方向
    // 检查原始z速度是正还是负
    if (is_negative(velocity_z_original)) {
        desired_vel_cms.z = desired_vel_cms.z * -1.0f;
    }
}

/*
 * Compute the back away horizontal velocity required to avoid breaching margin
 * INPUT: This method requires the breach in margin distance (back_distance_cm), direction towards the breach (limit_direction)
 *        It then calculates the desired backup velocity and passes it on to "find_max_quadrant_velocity" method to distribute the velocity vectors into respective quadrants
 * OUTPUT: The method then outputs four velocities (quad1/2/3/4_back_vel_cms), which correspond to the maximum horizontal desired backup velocity in each quadrant
 */
void AC_Avoid::calc_backup_velocity_2D(float kP, float accel_cmss, Vector2f &quad1_back_vel_cms, Vector2f &quad2_back_vel_cms, Vector2f &quad3_back_vel_cms, Vector2f &quad4_back_vel_cms, float back_distance_cm, Vector2f limit_direction, float dt)
{      
    if (limit_direction.is_zero()) {
        // protect against divide by zero
        // 防止除以零
        return; 
    }
    // speed required to move away the exact distance that we have breached the margin with 
    // 移动所需的速度，正好是我们突破边距的距离
    const float back_speed = get_max_speed(kP, 0.4f * accel_cmss, fabsf(back_distance_cm), dt);
    
    // direction to the obstacle
    // 障碍物的方向
    limit_direction.normalize();

    // move in the opposite direction with the required speed
    // 以所需的速度向相反方向移动
    Vector2f back_direction_vel = limit_direction * (-back_speed);
    // divide the vector into quadrants, find maximum velocity component in each quadrant 
    // 将向量分成象限，找到每个象限中的最大速度分量
    find_max_quadrant_velocity(back_direction_vel, quad1_back_vel_cms, quad2_back_vel_cms, quad3_back_vel_cms, quad4_back_vel_cms);
}

/*
* Compute the back away velocity required to avoid breaching margin, including vertical component
* min_z_vel is <= 0, and stores the greatest velocity in the downwards direction
* max_z_vel is >= 0, and stores the greatest velocity in the upwards direction
* eventually max_z_vel + min_z_vel will give the final desired Z backaway velocity
*/
void AC_Avoid::calc_backup_velocity_3D(float kP, float accel_cmss, Vector2f &quad1_back_vel_cms, Vector2f &quad2_back_vel_cms, Vector2f &quad3_back_vel_cms, Vector2f &quad4_back_vel_cms, float back_distance_cms, Vector3f limit_direction, float kp_z, float accel_cmss_z, float back_distance_z, float& min_z_vel, float& max_z_vel, float dt)
{   
    // backup horizontally 
    // 水平后退
    if (is_positive(back_distance_cms)) {
        Vector2f limit_direction_2d{limit_direction.x, limit_direction.y};
        calc_backup_velocity_2D(kP, accel_cmss, quad1_back_vel_cms, quad2_back_vel_cms, quad3_back_vel_cms, quad4_back_vel_cms, back_distance_cms, limit_direction_2d, dt);
    }

    // backup vertically 
    // 垂直后退
    if (!is_zero(back_distance_z)) {
        float back_speed_z = get_max_speed(kp_z, 0.4f * accel_cmss_z, fabsf(back_distance_z), dt);
        // Down is positive
        // 向下为正
        if (is_positive(back_distance_z)) {
            back_speed_z *= -1.0f;
        } 

        // store the z backup speed into min or max z if possible
        // 如果可能，将z后退速度存储到min或max z中
        if (back_speed_z < min_z_vel) {
            min_z_vel = back_speed_z;  
        }
        if (back_speed_z > max_z_vel) {
            max_z_vel = back_speed_z;
        }
    }
}
/*
 * Calculate maximum velocity vector that can be formed in each quadrant 
 * This method takes the desired backup velocity, and four other velocities corresponding to each quadrant
 * The desired velocity is then fit into one of the 4 quadrant velocities as per the sign of its components
 * This ensures that if we have multiple backup velocities, we can get the maximum of all of those velocities in each quadrant
 * 计算每个象限中可以形成的最大速度向量
 * 该方法采用所需的后退速度和对应于每个象限的四个其他速度
 * 然后根据其分量的符号将所需速度拟合到四个象限速度之一
 * 这确保了如果我们有多个后退速度，我们可以在每个象限中获得所有这些速度的最大值
*/
void AC_Avoid::find_max_quadrant_velocity(Vector2f &desired_vel, Vector2f &quad1_vel, Vector2f &quad2_vel, Vector2f &quad3_vel, Vector2f &quad4_vel) 
{   
    if (desired_vel.is_zero()) {
        return;
    }
     // first quadrant: +ve x, +ve y direction
     // 第一象限：+x方向，+y方向
    if (is_positive(desired_vel.x) && is_positive(desired_vel.y)) {
        quad1_vel = Vector2f{MAX(quad1_vel.x, desired_vel.x), MAX(quad1_vel.y,desired_vel.y)};
    }
    // second quadrant: -ve x, +ve y direction
    // 第二象限：-x方向，+y方向
    if (is_negative(desired_vel.x) && is_positive(desired_vel.y)) {
        quad2_vel = Vector2f{MIN(quad2_vel.x, desired_vel.x), MAX(quad2_vel.y,desired_vel.y)};
    }
    // third quadrant: -ve x, -ve y direction
    // 第三象限：-x方向，-y方向
    if (is_negative(desired_vel.x) && is_negative(desired_vel.y)) {
        quad3_vel = Vector2f{MIN(quad3_vel.x, desired_vel.x), MIN(quad3_vel.y,desired_vel.y)};
    }
    // fourth quadrant: +ve x, -ve y direction
    // 第四象限：+x方向，-y方向
    if (is_positive(desired_vel.x) && is_negative(desired_vel.y)) {
        quad4_vel = Vector2f{MAX(quad4_vel.x, desired_vel.x), MIN(quad4_vel.y,desired_vel.y)};
    }
}

/*
Calculate maximum velocity vector that can be formed in each quadrant and separately store max & min of vertical components
计算每个象限中可以形成的最大速度向量，并分别存储垂直分量的最大值和最小值
*/
void AC_Avoid::find_max_quadrant_velocity_3D(Vector3f &desired_vel, Vector2f &quad1_vel, Vector2f &quad2_vel, Vector2f &quad3_vel, Vector2f &quad4_vel, float &max_z_vel, float &min_z_vel)
{   
    // split into horizontal and vertical components 
    // 分成水平和垂直分量
    Vector2f velocity_xy{desired_vel.x, desired_vel.y};
    find_max_quadrant_velocity(velocity_xy, quad1_vel, quad2_vel, quad3_vel, quad4_vel);
    
    // store maximum and minimum of z 
    // 存储z的最大值和最小值
    if (is_positive(desired_vel.z) && (desired_vel.z > max_z_vel)) {
        max_z_vel = desired_vel.z;
    }
    if (is_negative(desired_vel.z) && (desired_vel.z < min_z_vel)) {
        min_z_vel = desired_vel.z;
    }
}

/*
 * Computes the speed such that the stopping distance
 * of the vehicle will be exactly the input distance.
 * 计算速度，使得车辆的停止距离正好是输入距离
 */
float AC_Avoid::get_max_speed(float kP, float accel_cmss, float distance_cm, float dt) const
{
    if (is_zero(kP)) {
        return safe_sqrt(2.0f * distance_cm * accel_cmss);
    } else {
        return sqrt_controller(distance_cm, kP, accel_cmss, dt);
    }
}

#if AP_FENCE_ENABLED

/*
 * Adjusts the desired velocity for the circular fence.
 * 调整圆形围栏的期望速度
 */
void AC_Avoid::adjust_velocity_circle_fence(float kP, float accel_cmss, Vector2f &desired_vel_cms, Vector2f &backup_vel, float dt)
{
    AC_Fence *fence = AP::fence();
    if (fence == nullptr) {
        return;
    }

    AC_Fence &_fence = *fence;

    // exit if circular fence is not enabled
    // 如果未启用圆形围栏则退出
    if ((_fence.get_enabled_fences() & AC_FENCE_TYPE_CIRCLE) == 0) {
        return;
    }

    // exit if the circular fence has already been breached
    // 如果圆形围栏已经被突破则退出
    if ((_fence.get_breaches() & AC_FENCE_TYPE_CIRCLE) != 0) {
        return;
    }

    // get desired speed
    // 获取期望速度
    const float desired_speed = desired_vel_cms.length();
    if (is_zero(desired_speed)) {
        // no avoidance necessary when desired speed is zero
        // 当期望速度为零时不需要避障
        return;
    }

    const AP_AHRS &_ahrs = AP::ahrs();

    // get position as a 2D offset from ahrs home
    // 获取相对于ahrs home的2D偏移位置
    Vector2f position_xy;
    if (!_ahrs.get_relative_position_NE_home(position_xy)) {
        // we have no idea where we are....
        // 我们不知道我们在哪里....
        return;
    }
    position_xy *= 100.0f; // m -> cm

    // get the fence radius in cm
    // 获取围栏半径（厘米）
    const float fence_radius = _fence.get_radius() * 100.0f;
    // get the margin to the fence in cm
    // 获取围栏的边距（厘米）
    const float margin_cm = _fence.get_margin() * 100.0f;

    if (margin_cm > fence_radius) {
        return;
    }

    // get vehicle distance from home
    // 获取车辆距离home的距离
    const float dist_from_home = position_xy.length();
    if (dist_from_home > fence_radius) {
        // outside of circular fence, no velocity adjustments
        // 在圆形围栏外，不调整速度
        return;
    }
    const float distance_to_boundary = fence_radius - dist_from_home;

    // for backing away
    // 用于后退
    Vector2f quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel;
    
    // back away if vehicle has breached margin
    // 如果车辆突破了边距则后退
    if (is_negative(distance_to_boundary - margin_cm)) {     
        calc_backup_velocity_2D(kP, accel_cmss, quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel, margin_cm - distance_to_boundary, position_xy, dt);
    }
    // desired backup velocity is sum of maximum velocity component in each quadrant 
    // 期望的后退速度是每个象限中最大速度分量的总和
    backup_vel = quad_1_back_vel + quad_2_back_vel + quad_3_back_vel + quad_4_back_vel;

    // vehicle is inside the circular fence
    // 车辆在圆形围栏内
    switch (_behavior) {
    case BEHAVIOR_SLIDE: {
        // implement sliding behaviour
        // 实现滑动行为
        const Vector2f stopping_point = position_xy + desired_vel_cms*(get_stopping_distance(kP, accel_cmss, desired_speed)/desired_speed);
        const float stopping_point_dist_from_home = stopping_point.length();
        if (stopping_point_dist_from_home <= fence_radius - margin_cm) {
            // stopping before before fence so no need to adjust
            // 在围栏前停止，因此无需调整
            return;
        }
        // unsafe desired velocity - will not be able to stop before reaching margin from fence
        // 不安全的期望速度 - 在到达围栏边距之前无法停止
        // Project stopping point radially onto fence boundary
        // Adjusted velocity will point towards this projected point at a safe speed
        // 将停止点径向投影到围栏边界
        // 调整后的速度将以安全速度指向该投影点
        const Vector2f target_offset = stopping_point * ((fence_radius - margin_cm) / stopping_point_dist_from_home);
        const Vector2f target_direction = target_offset - position_xy;
        const float distance_to_target = target_direction.length();
        if (is_positive(distance_to_target)) {
            const float max_speed = get_max_speed(kP, accel_cmss, distance_to_target, dt);
            desired_vel_cms = target_direction * (MIN(desired_speed,max_speed) / distance_to_target);
        }
      break;
    } 
  
    case (BEHAVIOR_STOP): {
        // implement stopping behaviour
        // 实现停止行为
        // calculate stopping point plus a margin so we look forward far enough to intersect with circular fence
        // 计算停止点加上边距，以便我们向前看足够远以与圆形围栏相交
        const Vector2f stopping_point_plus_margin = position_xy + desired_vel_cms*((2.0f + margin_cm + get_stopping_distance(kP, accel_cmss, desired_speed))/desired_speed);
        const float stopping_point_plus_margin_dist_from_home = stopping_point_plus_margin.length();
        if (dist_from_home >= fence_radius - margin_cm) {
            // vehicle has already breached margin around fence
            // if stopping point is even further from home (i.e. in wrong direction) then adjust speed to zero
            // otherwise user is backing away from fence so do not apply limits
            // 车辆已经突破围栏边距
            // 如果停止点离home更远（即错误方向），则将速度调整为零
            // 否则用户正在后退，因此不应用限制
            if (stopping_point_plus_margin_dist_from_home >= dist_from_home) {
                desired_vel_cms.zero();
            }
        } else {
            // shorten vector without adjusting its direction
            // 缩短向量而不调整其方向
            Vector2f intersection;
            if (Vector2f::circle_segment_intersection(position_xy, stopping_point_plus_margin, Vector2f(0.0f,0.0f), fence_radius - margin_cm, intersection)) {
                const float distance_to_target = (intersection - position_xy).length();
                const float max_speed = get_max_speed(kP, accel_cmss, distance_to_target, dt);
                if (max_speed < desired_speed) {
                    desired_vel_cms *= MAX(max_speed, 0.0f) / desired_speed;
                }
            }
        }
        break;
    }
    }
}

/*
 * Adjusts the desired velocity for the exclusion polygons
 * 调整排除多边形的期望速度
 */
void AC_Avoid::adjust_velocity_inclusion_and_exclusion_polygons(float kP, float accel_cmss, Vector2f &desired_vel_cms, Vector2f &backup_vel, float dt)
{
    const AC_Fence *fence = AP::fence();
    if (fence == nullptr) {
        return;
    }

    // exit if polygon fences are not enabled
    // 如果未启用多边形围栏则退出
    if ((fence->get_enabled_fences() & AC_FENCE_TYPE_POLYGON) == 0) {
        return;
    }

    // for backing away
    // 用于后退
    Vector2f quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel;

    // iterate through inclusion polygons
    // 遍历包含多边形
    const uint8_t num_inclusion_polygons = fence->polyfence().get_inclusion_polygon_count();
    for (uint8_t i = 0; i < num_inclusion_polygons; i++) {
        uint16_t num_points;
        const Vector2f* boundary = fence->polyfence().get_inclusion_polygon(i, num_points);
        Vector2f backup_vel_inc;
        // adjust velocity
        // 调整速度
        adjust_velocity_polygon(kP, accel_cmss, desired_vel_cms, backup_vel_inc, boundary, num_points, fence->get_margin(), dt, true);
        find_max_quadrant_velocity(backup_vel_inc, quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel);
    }

    // iterate through exclusion polygons
    // 遍历排除多边形
    const uint8_t num_exclusion_polygons = fence->polyfence().get_exclusion_polygon_count();
    for (uint8_t i = 0; i < num_exclusion_polygons; i++) {
        uint16_t num_points;
        const Vector2f* boundary = fence->polyfence().get_exclusion_polygon(i, num_points);
        Vector2f backup_vel_exc;
        // adjust velocity
        // 调整速度
        adjust_velocity_polygon(kP, accel_cmss, desired_vel_cms, backup_vel_exc, boundary, num_points, fence->get_margin(), dt, false);
        find_max_quadrant_velocity(backup_vel_exc, quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel);
    }
    // desired backup velocity is sum of maximum velocity component in each quadrant 
    // 期望的后退速度是每个象限中最大速度分量的总和
    backup_vel = quad_1_back_vel + quad_2_back_vel + quad_3_back_vel + quad_4_back_vel;
}

/*
 * Adjusts the desired velocity for the inclusion circles
 * 调整包含圆的期望速度
 */
void AC_Avoid::adjust_velocity_inclusion_circles(float kP, float accel_cmss, Vector2f &desired_vel_cms, Vector2f &backup_vel, float dt)
{
    const AC_Fence *fence = AP::fence();
    if (fence == nullptr) {
        return;
    }

    // return immediately if no inclusion circles
    // 如果没有包含圆则立即返回
    const uint8_t num_circles = fence->polyfence().get_inclusion_circle_count();
    if (num_circles == 0) {
        return;
    }

    // exit if polygon fences are not enabled
    // 如果未启用多边形围栏则退出
    if ((fence->get_enabled_fences() & AC_FENCE_TYPE_POLYGON) == 0) {
        return;
    }

    // get vehicle position
    // 获取车辆位置
    Vector2f position_NE;
    if (!AP::ahrs().get_relative_position_NE_origin(position_NE)) {
        // do not limit velocity if we don't have a position estimate
        // 如果我们没有位置估计，则不限制速度
        return;
    }
    position_NE = position_NE * 100.0f;  // m to cm

    // get the margin to the fence in cm
    // 获取围栏的边距（厘米）
    const float margin_cm = fence->get_margin() * 100.0f;

    // get desired speed
    // 获取期望速度
    const float desired_speed = desired_vel_cms.length();

    // get stopping distance as an offset from the vehicle
    // 获取停止距离作为车辆的偏移量
    Vector2f stopping_offset;
    if (!is_zero(desired_speed)) {
        switch (_behavior) {
            case BEHAVIOR_SLIDE:
                stopping_offset = desired_vel_cms*(get_stopping_distance(kP, accel_cmss, desired_speed)/desired_speed);
                break;
            case BEHAVIOR_STOP:
                // calculate stopping point plus a margin so we look forward far enough to intersect with circular fence
                // 计算停止点加上边距，以便我们向前看足够远以与圆形围栏相交
                stopping_offset = desired_vel_cms*((2.0f + margin_cm + get_stopping_distance(kP, accel_cmss, desired_speed))/desired_speed);
                break;
        }
    }

    // for backing away
    // 用于后退
    Vector2f quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel;

    // iterate through inclusion circles
    // 遍历包含圆
    for (uint8_t i = 0; i < num_circles; i++) {
        Vector2f center_pos_cm;
        float radius;
        if (fence->polyfence().get_inclusion_circle(i, center_pos_cm, radius)) {
            // get position relative to circle's center
            // 获取相对于圆心的位置
            const Vector2f position_NE_rel = (position_NE - center_pos_cm);

            // if we are outside this circle do not limit velocity for this circle
            // 如果我们在这个圆外，则不限制这个圆的速度
            const float dist_sq_cm = position_NE_rel.length_squared();
            const float radius_cm = (radius * 100.0f);
            if (dist_sq_cm > sq(radius_cm)) {
                continue;
            }

            const float radius_with_margin = radius_cm - margin_cm;
            if (is_negative(radius_with_margin)) {
                return;
            }
            
            const float margin_breach = radius_with_margin - safe_sqrt(dist_sq_cm);
            // back away if vehicle has breached margin
            // 如果车辆突破了边距则后退
            if (is_negative(margin_breach)) {
                calc_backup_velocity_2D(kP, accel_cmss, quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel, margin_breach, position_NE_rel, dt);
            }
            if (is_zero(desired_speed)) {
                // no avoidance necessary when desired speed is zero
                // 当期望速度为零时不需要避障
                continue;
            }

            switch (_behavior) {
                case BEHAVIOR_SLIDE: {
                    // implement sliding behaviour
                    // 实现滑动行为
                    const Vector2f stopping_point = position_NE_rel + stopping_offset;
                    const float stopping_point_dist = stopping_point.length();
                    if (is_zero(stopping_point_dist) || (stopping_point_dist <= (radius_cm - margin_cm))) {
                        // stopping before before fence so no need to adjust for this circle
                        // 在围栏前停止，因此无需为此圆调整
                        continue;
                    }
                    // unsafe desired velocity - will not be able to stop before reaching margin from fence
                    // 不安全的期望速度 - 在到达围栏边距之前无法停止
                    // project stopping point radially onto fence boundary
                    // adjusted velocity will point towards this projected point at a safe speed
                    // 将停止点径向投影到围栏边界
                    // 调整后的速度将以安全速度指向该投影点
                    const Vector2f target_offset = stopping_point * ((radius_cm - margin_cm) / stopping_point_dist);
                    const Vector2f target_direction = target_offset - position_NE_rel;
                    const float distance_to_target = target_direction.length();
                    if (is_positive(distance_to_target)) {
                        const float max_speed = get_max_speed(kP, accel_cmss, distance_to_target, dt);
                        desired_vel_cms = target_direction * (MIN(desired_speed,max_speed) / distance_to_target);
                    }
                }
                break;
                case BEHAVIOR_STOP: {
                    // implement stopping behaviour
                    // 实现停止行为
                    const Vector2f stopping_point_plus_margin = position_NE_rel + stopping_offset;
                    const float dist_cm = safe_sqrt(dist_sq_cm);
                    if (dist_cm >= radius_cm - margin_cm) {
                        // vehicle has already breached margin around fence
                        // if stopping point is even further from center (i.e. in wrong direction) then adjust speed to zero
                        // otherwise user is backing away from fence so do not apply limits
                        // 车辆已经突破围栏边距
                        // 如果停止点离中心更远（即错误方向），则将速度调整为零
                        // 否则用户正在后退，因此不应用限制
                        if (stopping_point_plus_margin.length() >= dist_cm) {
                            desired_vel_cms.zero();
                            // desired backup velocity is sum of maximum velocity component in each quadrant 
                            // 期望的后退速度是每个象限中最大速度分量的总和
                            backup_vel = quad_1_back_vel + quad_2_back_vel + quad_3_back_vel + quad_4_back_vel;
                            return;
                        }
                    } else {
                        // shorten vector without adjusting its direction
                        // 缩短向量而不调整其方向
                        Vector2f intersection;
                        if (Vector2f::circle_segment_intersection(position_NE_rel, stopping_point_plus_margin, Vector2f(0.0f,0.0f), radius_cm - margin_cm, intersection)) {
                            const float distance_to_target = (intersection - position_NE_rel).length();
                            const float max_speed = get_max_speed(kP, accel_cmss, distance_to_target, dt);
                            if (max_speed < desired_speed) {
                                desired_vel_cms *= MAX(max_speed, 0.0f) / desired_speed;
                            }
                        }
                    }
                }
                break;
            }
        }
    }
    // desired backup velocity is sum of maximum velocity component in each quadrant 
    // 期望的后退速度是每个象限中最大速度分量的总和
    backup_vel = quad_1_back_vel + quad_2_back_vel + quad_3_back_vel + quad_4_back_vel;
}

/*
 * Adjusts the desired velocity for the exclusion circles
 * 调整排除圆的期望速度
 */
void AC_Avoid::adjust_velocity_exclusion_circles(float kP, float accel_cmss, Vector2f &desired_vel_cms, Vector2f &backup_vel, float dt)
{
    const AC_Fence *fence = AP::fence();
    if (fence == nullptr) {
        return;
    }

    // return immediately if no inclusion circles
    // 如果没有包含圆则立即返回
    const uint8_t num_circles = fence->polyfence().get_exclusion_circle_count();
    if (num_circles == 0) {
        return;
    }

    // exit if polygon fences are not enabled
    // 如果未启用多边形围栏则退出
    if ((fence->get_enabled_fences() & AC_FENCE_TYPE_POLYGON) == 0) {
        return;
    }

    // get vehicle position
    // 获取车辆位置
    Vector2f position_NE;
    if (!AP::ahrs().get_relative_position_NE_origin(position_NE)) {
        // do not limit velocity if we don't have a position estimate
        // 如果我们没有位置估计，则不限制速度
        return;
    }
    position_NE = position_NE * 100.0f;  // m to cm

    // get the margin to the fence in cm
    // 获取围栏的边距（厘米）
    const float margin_cm = fence->get_margin() * 100.0f;

    // for backing away
    // 用于后退
    Vector2f quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel;

    // get desired speed
    // 获取期望速度
    const float desired_speed = desired_vel_cms.length();
    
    // calculate stopping distance as an offset from the vehicle (only used for BEHAVIOR_STOP)
    // add a margin so we look forward far enough to intersect with circular fence
    // 计算停止距离作为车辆的偏移量（仅用于BEHAVIOR_STOP）
    // 添加边距，以便我们向前看足够远以与圆形围栏相交
    Vector2f stopping_offset;
    if (!is_zero(desired_speed)) {
        if ((AC_Avoid::BehaviourType)_behavior.get() == BEHAVIOR_STOP) {
            stopping_offset = desired_vel_cms*((2.0f + margin_cm + get_stopping_distance(kP, accel_cmss, desired_speed))/desired_speed);
        }
    }
    // iterate through exclusion circles
    // 遍历排除圆
    for (uint8_t i = 0; i < num_circles; i++) {
        Vector2f center_pos_cm;
        float radius;
        if (fence->polyfence().get_exclusion_circle(i, center_pos_cm, radius)) {
            // get position relative to circle's center
            // 获取相对于圆心的位置
            const Vector2f position_NE_rel = (position_NE - center_pos_cm);

            // if we are inside this circle do not limit velocity for this circle
            // 如果我们在这个圆内，则不限制这个圆的速度
            const float dist_sq_cm = position_NE_rel.length_squared();
            const float radius_cm = (radius * 100.0f);
            if (radius_cm < margin_cm) {
                return;
            }
            if (dist_sq_cm < sq(radius_cm)) {
                continue;
            }
        
            const Vector2f vector_to_center = center_pos_cm - position_NE;
            const float dist_to_boundary = vector_to_center.length() - radius_cm;
            // back away if vehicle has breached margin
            // 如果车辆突破了边距则后退
            if (is_negative(dist_to_boundary - margin_cm)) {
                calc_backup_velocity_2D(kP, accel_cmss, quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel, margin_cm - dist_to_boundary, vector_to_center, dt);
            }
            if (is_zero(desired_speed)) {
                // no avoidance necessary when desired speed is zero
                // 当期望速度为零时不需要避障
                continue;
            }

            switch (_behavior) {
                case BEHAVIOR_SLIDE: {
                    // vector from current position to circle's center
                    // 从当前位置到圆心的向量
                    Vector2f limit_direction = vector_to_center;
                    if (limit_direction.is_zero()) {
                        // vehicle is exactly on circle center so do not limit velocity
                        // 车辆正好在圆心上，因此不限制速度
                        continue;
                    }
                    // calculate distance to edge of circle
                    // 计算到圆边的距离
                    const float limit_distance_cm = limit_direction.length() - radius_cm;
                    if (!is_positive(limit_distance_cm)) {
                        // vehicle is within circle so do not limit velocity
                        // 车辆在圆内，因此不限制速度
                        continue;
                    }
                    // vehicle is outside the circle, adjust velocity to stay outside
                    // 车辆在圆外，调整速度以保持在外面
                    limit_direction.normalize();
                    limit_velocity_2D(kP, accel_cmss, desired_vel_cms, limit_direction, MAX(limit_distance_cm - margin_cm, 0.0f), dt);
                }
                break;
                case BEHAVIOR_STOP: {
                    // implement stopping behaviour
                    // 实现停止行为
                    const Vector2f stopping_point_plus_margin = position_NE_rel + stopping_offset;
                    const float dist_cm = safe_sqrt(dist_sq_cm);
                    if (dist_cm < radius_cm + margin_cm) {
                        // vehicle has already breached margin around fence
                        // if stopping point is closer to center (i.e. in wrong direction) then adjust speed to zero
                        // otherwise user is backing away from fence so do not apply limits
                        // 车辆已经突破围栏边距
                        // 如果停止点离中心更近（即错误方向），则将速度调整为零
                        // 否则用户正在后退，因此不应用限制
                        if (stopping_point_plus_margin.length() <= dist_cm) {
                            desired_vel_cms.zero();
                            backup_vel = quad_1_back_vel + quad_2_back_vel + quad_3_back_vel + quad_4_back_vel;
                            return;
                        }
                    } else {
                        // shorten vector without adjusting its direction
                        // 缩短向量而不调整其方向
                        Vector2f intersection;
                        if (Vector2f::circle_segment_intersection(position_NE_rel, stopping_point_plus_margin, Vector2f(0.0f,0.0f), radius_cm + margin_cm, intersection)) {
                            const float distance_to_target = (intersection - position_NE_rel).length();
                            const float max_speed = get_max_speed(kP, accel_cmss, distance_to_target, dt);
                            if (max_speed < desired_speed) {
                                desired_vel_cms *= MAX(max_speed, 0.0f) / desired_speed;
                            }
                        }
                    }
                }
                break;
            }
        }
    }
    // desired backup velocity is sum of maximum velocity component in each quadrant 
    // 期望的后退速度是每个象限中最大速度分量的总和
    backup_vel = quad_1_back_vel + quad_2_back_vel + quad_3_back_vel + quad_4_back_vel;
}
#endif // AP_FENCE_ENABLED

#if AP_BEACON_ENABLED
/*
 * Adjusts the desired velocity for the beacon fence.
 * 调整信标围栏的期望速度
 */
void AC_Avoid::adjust_velocity_beacon_fence(float kP, float accel_cmss, Vector2f &desired_vel_cms, Vector2f &backup_vel, float dt)
{
    // 获取信标对象指针
    AP_Beacon *_beacon = AP::beacon();

    // 如果信标不存在则退出
    // exit if the beacon is not present
    if (_beacon == nullptr) {
        return;
    }

    // 从信标获取边界点
    // get boundary from beacons
    uint16_t num_points = 0;
    const Vector2f* boundary = _beacon->get_boundary_points(num_points);
    if ((boundary == nullptr) || (num_points == 0)) {
        return;
    }

    // 使用信标调整速度
    // adjust velocity using beacon
    float margin = 0;
#if AP_FENCE_ENABLED
    // 如果启用了围栏,获取围栏边距
    if (AP::fence()) {
        margin = AP::fence()->get_margin();
    }
#endif
    // 根据多边形边界调整速度
    adjust_velocity_polygon(kP, accel_cmss, desired_vel_cms, backup_vel, boundary, num_points, margin, dt, true);
}
#endif  // AP_BEACON_ENABLED

/*
 * Adjusts the desired velocity based on output from the proximity sensor
 * 根据近距离传感器的输出调整期望速度
 */
void AC_Avoid::adjust_velocity_proximity(float kP, float accel_cmss, Vector3f &desired_vel_cms, Vector3f &backup_vel, float kP_z, float accel_cmss_z, float dt)
{
#if HAL_PROXIMITY_ENABLED
    // 如果近距离传感器不存在则立即退出
    // exit immediately if proximity sensor is not present
    AP_Proximity *proximity = AP::proximity();
    if (!proximity) {
        return;
    }

    AP_Proximity &_proximity = *proximity;
    // 获取障碍物总数
    // get total number of obstacles
    const uint8_t obstacle_num = _proximity.get_obstacle_count();
    if (obstacle_num == 0) {
        // 没有障碍物
        // no obstacles
        return;
    }
 
    const AP_AHRS &_ahrs = AP::ahrs();
    
    // 用于后退的变量
    // for backing away
    Vector2f quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel;
    float max_back_vel_z = 0.0f;
    float min_back_vel_z = 0.0f; 

    // 将速度向量从地球坐标系旋转到机体坐标系,因为障碍物是在机体坐标系中
    // rotate velocity vector from earth frame to body-frame since obstacles are in body-frame
    const Vector2f desired_vel_body_cms = _ahrs.earth_to_body2D(Vector2f{desired_vel_cms.x, desired_vel_cms.y});
    
    // safe_vel将被调整以避开近距离障碍物
    // safe_vel will be adjusted to stay away from Proximity Obstacles
    Vector3f safe_vel = Vector3f{desired_vel_body_cms.x, desired_vel_body_cms.y, desired_vel_cms.z};
    const Vector3f safe_vel_orig = safe_vel;

    // 计算边距(厘米)
    // calc margin in cm
    const float margin_cm = MAX(_margin * 100.0f, 0.0f);
    Vector3f stopping_point_plus_margin;
    if (!desired_vel_cms.is_zero()) {
        // 仅用于"停止模式"。在这里预先计算停止点可以确保我们不需要在迭代中重复计算。
        // only used for "stop mode". Pre-calculating the stopping point here makes sure we do not need to repeat the calculations under iterations.
        const float speed = safe_vel.length();
        stopping_point_plus_margin = safe_vel * ((2.0f + margin_cm + get_stopping_distance(kP, accel_cmss, speed))/speed);
    }

    for (uint8_t i = 0; i<obstacle_num; i++) {
        // 从近距离库获取障碍物
        // get obstacle from proximity library
        Vector3f vector_to_obstacle;
        if (!_proximity.get_obstacle(i, vector_to_obstacle)) {
            // 这个障碍物无效
            // this one is not valid
            continue;
        }

        const float dist_to_boundary = vector_to_obstacle.length();
        if (is_zero(dist_to_boundary)) {
            continue;
        }

        // 如果车辆已突破边距则后退
        // back away if vehicle has breached margin
        if (is_negative(dist_to_boundary - margin_cm)) {
            const float breach_dist = margin_cm - dist_to_boundary;
            // 添加死区,这样车辆就不会反复后退和前进
            // add a deadzone so that the vehicle doesn't backup and go forward again and again
            const float deadzone = MAX(0.0f, _backup_deadzone) * 100.0f;
            if (breach_dist > deadzone) {
                // 这个向量将帮助我们决定在水平和垂直方向需要后退多少
                // this vector will help us decide how much we have to back away horizontally and vertically
                const Vector3f margin_vector = vector_to_obstacle.normalized() * breach_dist;
                const float xy_back_dist = margin_vector.xy().length();
                const float z_back_dist = margin_vector.z;
                calc_backup_velocity_3D(kP, accel_cmss, quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel, xy_back_dist, vector_to_obstacle, kP_z, accel_cmss_z, z_back_dist, min_back_vel_z, max_back_vel_z, dt);
            }
        }

        if (desired_vel_cms.is_zero()) {
            // 如果没有需要限制的速度则无法限制速度
            // 后退(如果需要)已经完成
            // cannot limit velocity if there is nothing to limit
            // backing up (if needed) has already been done
            continue;
        }

        switch (_behavior) {
        case BEHAVIOR_SLIDE: {
            Vector3f limit_direction{vector_to_obstacle};
            // 到最近点的距离
            // distance to closest point
            const float limit_distance_cm = limit_direction.length();
            if (is_zero(limit_distance_cm)) {
                // 我们正好在边缘上,这种情况理想情况下不应该发生
                // 即不调整速度
                // We are exactly on the edge, this should ideally never be possible
                // i.e. do not adjust velocity.
                continue;
            }
            // 调整速度以不违反边距
            // Adjust velocity to not violate margin.
            limit_velocity_3D(kP, accel_cmss, safe_vel, limit_direction, margin_cm, kP_z, accel_cmss_z, dt);
        
            break;
        }

        case BEHAVIOR_STOP: {
            // 从当前位置到障碍物的向量
            // vector from current position to obstacle
            Vector3f limit_direction;
            // 找到与线段最近的点
            // 同时查看车辆是否会与投影停止点处的边界"大致"相交
            // find closest point with line segment
            // also see if the vehicle will "roughly" intersect the boundary with the projected stopping point
            const bool intersect = _proximity.closest_point_from_segment_to_obstacle(i, Vector3f{}, stopping_point_plus_margin, limit_direction);
            if (intersect) {
                // 车辆正在与边界形成的平面相交
                // the vehicle is intersecting the plane formed by the boundary
                // 从停止点到最近点的距离
                // distance to the closest point from the stopping point
                float limit_distance_cm = limit_direction.length();
                if (is_zero(limit_distance_cm)) {
                    // 我们正好在边缘上,这种情况理想情况下不应该发生
                    // 即不调整速度
                    // We are exactly on the edge, this should ideally never be possible
                    // i.e. do not adjust velocity.
                    return;
                }
                if (limit_distance_cm <= margin_cm) {
                    // 我们在边距内,所以停止车辆
                    // we are within the margin so stop vehicle
                    safe_vel.zero();
                } else {
                    // 车辆在给定边缘内,调整速度以不违反此边缘
                    // vehicle inside the given edge, adjust velocity to not violate this edge
                    limit_velocity_3D(kP, accel_cmss, safe_vel, limit_direction, margin_cm, kP_z, accel_cmss_z, dt);
                }

                break;
            }
        }
        }
    }

    // 期望的后退速度是每个象限中最大速度分量的总和
    // desired backup velocity is sum of maximum velocity component in each quadrant 
    const Vector2f desired_back_vel_cms_xy = quad_1_back_vel + quad_2_back_vel + quad_3_back_vel + quad_4_back_vel;
    const float desired_back_vel_cms_z = max_back_vel_z + min_back_vel_z;

    if (safe_vel == safe_vel_orig && desired_back_vel_cms_xy.is_zero() && is_zero(desired_back_vel_cms_z)) {
        // 近距离避障没有做任何事情,没有必要进行下面的计算。提前返回
        // proximity avoidance did nothing, no point in doing the calculations below. Return early
        backup_vel.zero();
        return;
    }

    // 设置修改后的期望速度向量和后退速度向量
    // 向量在机体坐标系中,将结果向量旋转回地球坐标系
    // set modified desired velocity vector and back away velocity vector
    // vectors were in body-frame, rotate resulting vector back to earth-frame
    // 将机体坐标系中的安全速度向量转换到地球坐标系
    const Vector2f safe_vel_2d = _ahrs.body_to_earth2D(Vector2f{safe_vel.x, safe_vel.y});
    // 设置期望速度向量,包含水平和垂直分量
    desired_vel_cms = Vector3f{safe_vel_2d.x, safe_vel_2d.y, safe_vel.z};
    // 将机体坐标系中的后退速度向量转换到地球坐标系
    const Vector2f backup_vel_xy = _ahrs.body_to_earth2D(desired_back_vel_cms_xy);
    // 设置后退速度向量,包含水平和垂直分量
    backup_vel = Vector3f{backup_vel_xy.x, backup_vel_xy.y, desired_back_vel_cms_z};
#endif // HAL_PROXIMITY_ENABLED
}

/*
 * Adjusts the desired velocity for the polygon fence.
 * 调整多边形围栏的期望速度
 */
void AC_Avoid::adjust_velocity_polygon(float kP, float accel_cmss, Vector2f &desired_vel_cms, Vector2f &backup_vel, const Vector2f* boundary, uint16_t num_points, float margin, float dt, bool stay_inside)
{
    // exit if there are no points
    // 如果没有点则退出
    if (boundary == nullptr || num_points == 0) {
        return;
    }

    const AP_AHRS &_ahrs = AP::ahrs();

    // do not adjust velocity if vehicle is outside the polygon fence
    // 如果车辆在多边形围栏外则不调整速度
    Vector2f position_xy;
    if (!_ahrs.get_relative_position_NE_origin(position_xy)) {
        // boundary is in earth frame but we have no idea
        // where we are
        // 边界在地球坐标系中,但我们不知道自己在哪里
        return;
    }
    position_xy = position_xy * 100.0f;  // m to cm 米转换为厘米

    // return if we have already breached polygon
    // 如果已经突破多边形则返回
    const bool inside_polygon = !Polygon_outside(position_xy, boundary, num_points);
    if (inside_polygon != stay_inside) {
        return;
    }

    // Safe_vel will be adjusted to remain within fence.
    // We need a separate vector in case adjustment fails,
    // e.g. if we are exactly on the boundary.
    // 安全速度将被调整以保持在围栏内
    // 我们需要一个单独的向量以防调整失败
    // 例如,如果我们正好在边界上
    Vector2f safe_vel(desired_vel_cms);
    Vector2f desired_back_vel_cms;

    // calc margin in cm
    // 计算边距(厘米)
    const float margin_cm = MAX(margin * 100.0f, 0.0f);

    // for stopping
    // 用于停止
    const float speed = safe_vel.length();
    Vector2f stopping_point_plus_margin; 
    if (!desired_vel_cms.is_zero()) {
        stopping_point_plus_margin = position_xy + safe_vel*((2.0f + margin_cm + get_stopping_distance(kP, accel_cmss, speed))/speed);
    }

    // for backing away
    // 用于后退
    Vector2f quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel;
   
    for (uint16_t i=0; i<num_points; i++) {
        uint16_t j = i+1;
        if (j >= num_points) {
            j = 0;
        }
        // end points of current edge
        // 当前边的端点
        Vector2f start = boundary[j];
        Vector2f end = boundary[i];
        Vector2f vector_to_boundary = Vector2f::closest_point(position_xy, start, end) - position_xy;
        // back away if vehicle has breached margin
        // 如果车辆突破边距则后退
        if (is_negative(vector_to_boundary.length() - margin_cm)) {
            calc_backup_velocity_2D(kP, accel_cmss, quad_1_back_vel, quad_2_back_vel, quad_3_back_vel, quad_4_back_vel, margin_cm-vector_to_boundary.length(), vector_to_boundary, dt);
        }
        
        // exit immediately if no desired velocity
        // 如果没有期望速度则立即退出
        if (desired_vel_cms.is_zero()) {
            continue;
        }

        switch (_behavior) {
        case (BEHAVIOR_SLIDE): {
            // vector from current position to closest point on current edge
            // 从当前位置到当前边最近点的向量
            Vector2f limit_direction = vector_to_boundary;
            // distance to closest point
            // 到最近点的距离
            const float limit_distance_cm = limit_direction.length();
            if (is_zero(limit_distance_cm)) {
                // We are exactly on the edge - treat this as a fence breach.
                // i.e. do not adjust velocity.
                // 我们正好在边缘上 - 将其视为围栏突破
                // 即不调整速度
                return;
            }
            // We are strictly inside the given edge.
            // Adjust velocity to not violate this edge.
            // 我们严格在给定边缘内
            // 调整速度以不违反此边缘
            limit_direction /= limit_distance_cm;
            limit_velocity_2D(kP, accel_cmss, safe_vel, limit_direction, MAX(limit_distance_cm - margin_cm, 0.0f), dt);
            break;
        } 
        
        case (BEHAVIOR_STOP): {
            // find intersection with line segment
            // 查找与线段的交点
            Vector2f intersection;
            if (Vector2f::segment_intersection(position_xy, stopping_point_plus_margin, start, end, intersection)) {
                // vector from current position to point on current edge
                // 从当前位置到当前边上点的向量
                Vector2f limit_direction = intersection - position_xy;
                const float limit_distance_cm = limit_direction.length();
                if (is_zero(limit_distance_cm)) {
                    // We are exactly on the edge - treat this as a fence breach.
                    // i.e. do not adjust velocity.
                    // 我们正好在边缘上 - 将其视为围栏突破
                    // 即不调整速度
                    return;
                }
                if (limit_distance_cm <= margin_cm) {
                    // we are within the margin so stop vehicle
                    // 我们在边距内所以停止车辆
                    safe_vel.zero();
                } else {
                    // vehicle inside the given edge, adjust velocity to not violate this edge
                    // 车辆在给定边缘内,调整速度以不违反此边缘
                    limit_direction /= limit_distance_cm;
                    limit_velocity_2D(kP, accel_cmss, safe_vel, limit_direction, MAX(limit_distance_cm - margin_cm, 0.0f), dt);
                }
            }
        break;
        }
        }
    }
    // desired backup velocity is sum of maximum velocity component in each quadrant 
    // 期望的后退速度是每个象限中最大速度分量的总和
    desired_back_vel_cms = quad_1_back_vel + quad_2_back_vel + quad_3_back_vel + quad_4_back_vel;

    // set modified desired velocity vector or back away velocity vector
    // 设置修改后的期望速度向量或后退速度向量
    desired_vel_cms = safe_vel;
    backup_vel = desired_back_vel_cms;
}

/*
 * Computes distance required to stop, given current speed.
 * 计算给定当前速度所需的停止距离
 *
 * Implementation copied from AC_PosControl.
 * 实现从AC_PosControl复制
 */
float AC_Avoid::get_stopping_distance(float kP, float accel_cmss, float speed_cms) const
{
    // avoid divide by zero by using current position if the velocity is below 10cm/s, kP is very low or acceleration is zero
    // 如果速度低于10cm/s、kP很低或加速度为零,则使用当前位置来避免除以零
    if (accel_cmss <= 0.0f || is_zero(speed_cms)) {
        return 0.0f;
    }

    // handle linear deceleration
    // 处理线性减速
    if (kP <= 0.0f) {
        return 0.5f * sq(speed_cms) / accel_cmss;
    }

    // calculate distance within which we can stop
    // accel_cmss/kP is the point at which velocity switches from linear to sqrt
    // 计算我们可以停止的距离
    // accel_cmss/kP是速度从线性切换到平方根的点
    if (speed_cms < accel_cmss/kP) {
        return speed_cms/kP;
    } else {
        // accel_cmss/(2.0f*kP*kP) is the distance at which we switch from linear to sqrt response
        // accel_cmss/(2.0f*kP*kP)是我们从线性响应切换到平方根响应的距离
        return accel_cmss/(2.0f*kP*kP) + (speed_cms*speed_cms)/(2.0f*accel_cmss);
    }
}

// convert distance (in meters) to a lean percentage (in 0~1 range) for use in manual flight modes
// 将距离(以米为单位)转换为倾斜百分比(0~1范围)以用于手动飞行模式
float AC_Avoid::distance_to_lean_pct(float dist_m)
{
    // ignore objects beyond DIST_MAX
    // 忽略超出DIST_MAX的物体
    if (dist_m < 0.0f || dist_m >= _dist_max || _dist_max <= 0.0f) {
        return 0.0f;
    }
    // inverted but linear response
    // 反转但线性响应
    return 1.0f - (dist_m / _dist_max);
}

// returns the maximum positive and negative roll and pitch percentages (in -1 ~ +1 range) based on the proximity sensor
// 根据近距离传感器返回最大正负横滚和俯仰百分比(-1 ~ +1范围)
void AC_Avoid::get_proximity_roll_pitch_pct(float &roll_positive, float &roll_negative, float &pitch_positive, float &pitch_negative)
{
#if HAL_PROXIMITY_ENABLED
    AP_Proximity *proximity = AP::proximity();
    if (proximity == nullptr) {
        return;
    }
    AP_Proximity &_proximity = *proximity;
    const uint8_t obj_count = _proximity.get_object_count();
    // if no objects return
    // 如果没有物体则返回
    if (obj_count == 0) {
        return;
    }

    // calculate maximum roll, pitch values from objects
    // 从物体计算最大横滚、俯仰值
    for (uint8_t i=0; i<obj_count; i++) {
        float ang_deg, dist_m;
        if (_proximity.get_object_angle_and_distance(i, ang_deg, dist_m)) {
            if (dist_m < _dist_max) {
                // convert distance to lean angle (in 0 to 1 range)
                // 将距离转换为倾斜角度(0到1范围)
                const float lean_pct = distance_to_lean_pct(dist_m);
                // convert angle to roll and pitch lean percentages
                // 将角度转换为横滚和俯仰倾斜百分比
                const float angle_rad = radians(ang_deg);
                const float roll_pct = -sinf(angle_rad) * lean_pct;
                const float pitch_pct = cosf(angle_rad) * lean_pct;
                // update roll, pitch maximums
                // 更新横滚、俯仰最大值
                if (roll_pct > 0.0f) {
                    roll_positive = MAX(roll_positive, roll_pct);
                } else if (roll_pct < 0.0f) {
                    roll_negative = MIN(roll_negative, roll_pct);
                }
                if (pitch_pct > 0.0f) {
                    pitch_positive = MAX(pitch_positive, pitch_pct);
                } else if (pitch_pct < 0.0f) {
                    pitch_negative = MIN(pitch_negative, pitch_pct);
                }
            }
        }
    }
#endif // HAL_PROXIMITY_ENABLED
}

// singleton instance
// 单例实例
AC_Avoid *AC_Avoid::_singleton;

namespace AP {

AC_Avoid *ac_avoid()
{
    return AC_Avoid::get_singleton();
}

}

#endif // !APM_BUILD_Arduplane

#endif  // AP_AVOIDANCE_ENABLED
