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

#include "Plane.h"

/*
  高度处理例程。这些例程处理气压高度控制和地形跟随控制。
 */

/*
  根据模式调整目标高度
 */
void Plane::adjust_altitude_target()
{
    control_mode->update_target_altitude();
}

/*
  检查家的高度是否改变
 */
void Plane::check_home_alt_change(void)
{
    int32_t home_alt_cm = ahrs.get_home().alt;
    if (home_alt_cm != auto_state.last_home_alt_cm && hal.util->get_soft_armed()) {
        // 处理家的高度变化
        const int32_t alt_change_cm = home_alt_cm - auto_state.last_home_alt_cm;
        if (next_WP_loc.terrain_alt) {
            /*
              对于地形高度航点，next_WP_loc比较特殊。它们的
              terrain_alt=1，但relative_alt=0，并且已经计算为相对于家的高度。
              我们需要根据家的高度变化进行调整。
             */
            next_WP_loc.alt += alt_change_cm;
        }
        // 重置TECS以强制重新估算场地高度
        TECS_controller.reset();
    }
    auto_state.last_home_alt_cm = home_alt_cm;
}

/*
  如果合适的话，设置到下一个航点的渐进滑翔坡度
 */
void Plane::setup_glide_slope(void)
{
    // 确定我们到下一个航点的距离，用于计算高度变化率
    auto_state.wp_distance = current_loc.get_distance(next_WP_loc);
    auto_state.wp_proportion = current_loc.line_path_proportion(prev_WP_loc, next_WP_loc);
    TECS_controller.set_path_proportion(auto_state.wp_proportion);
    update_flight_stage();

    /*
      确定我们是否会逐渐改变高度，或者尝试尽快达到新的高度。
     */
    switch (control_mode->mode_number()) {
    case Mode::Number::RTL:
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
        /* 如果在目标高度之上，则缓慢下降，但如果在目标高度之下，则更快上升。
           参见 https://github.com/ArduPilot/ardupilot/issues/39
        */
        if (above_location_current(next_WP_loc)) {
            set_offset_altitude_location(prev_WP_loc, next_WP_loc);
        } else {
            reset_offset_altitude();
        }
        break;

    case Mode::Number::AUTO:

        // 如果启用了选项，则不进行滑翔坡度而直接爬升
        if (!above_location_current(next_WP_loc) && plane.flight_option_enabled(FlightOptions::IMMEDIATE_CLIMB_IN_AUTO)) {
            reset_offset_altitude();
            break;
        }

        // 我们只在高于20米或下降时在AUTO模式下进行滑翔坡度处理。
        // 20米阈值是任意的，基本上是为了防止在低高度时试图缓慢
        // 增加高度的情况，可能会撞到障碍物。
        if (adjusted_relative_altitude_cm() > 2000 || above_location_current(next_WP_loc)) {
            set_offset_altitude_location(prev_WP_loc, next_WP_loc);
        } else {
            reset_offset_altitude();
        }
        break;
    default:
        reset_offset_altitude();
        break;
    }
}

/*
  返回RTL高度（以AMSL厘米为单位）
 */
int32_t Plane::get_RTL_altitude_cm() const
{
    if (g.RTL_altitude < 0) {
        return current_loc.alt;
    }
    return g.RTL_altitude*100 + home.alt;
}

/*
  返回相对高度（以米为单位）（相对于地形（如果可用）或家）
 */
float Plane::relative_ground_altitude(bool use_rangefinder_if_available, bool use_terrain_if_available)
{
#if AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED
   float height_AGL;
   // 如果可用，使用外部HAGL
   if (get_external_HAGL(height_AGL)) {
       return height_AGL;
   }
#endif // AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED

#if AP_RANGEFINDER_ENABLED
   if (use_rangefinder_if_available && rangefinder_state.in_range) {
        return rangefinder_state.height_estimate;
   }
#endif

#if HAL_QUADPLANE_ENABLED && AP_RANGEFINDER_ENABLED
   if (use_rangefinder_if_available && quadplane.in_vtol_land_final() &&
       rangefinder.status_orient(rangefinder_orientation()) == RangeFinder::Status::OutOfRangeLow) {
       // 四旋翼着陆时测距仪低于最小值的特殊情况。
       // 认为我们距地面高度为零
       return 0;
   }
#endif

#if AP_TERRAIN_AVAILABLE
    float altitude;
    if (use_terrain_if_available &&
        terrain.status() == AP_Terrain::TerrainStatusOK &&
        terrain.height_above_terrain(altitude, true)) {
        return altitude;
    }
#endif

#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_land_descent() &&
        !quadplane.landing_with_fixed_wing_spiral_approach()) {
        // 在进行VTOL着陆时，我们可以使用航点高度作为
        // 地面高度。如果使用LAND_FW_APPROACH，我们不能这样做，
        // 因为它使用wp高度作为进近高度
        return height_above_target();
    }
#endif

    return relative_altitude;
}

// 使用地形的辅助方法（如果飞行器当前正在进行地形跟随）
float Plane::relative_ground_altitude(bool use_rangefinder_if_available)
{
#if AP_TERRAIN_AVAILABLE
    return relative_ground_altitude(use_rangefinder_if_available, target_altitude.terrain_following);
#else
    return relative_ground_altitude(use_rangefinder_if_available, false);
#endif
}


/*
  将目标高度设置为当前高度。这在设置高度保持时使用，
  例如在CRUISE模式下释放升降舵时。
 */
void Plane::set_target_altitude_current(void)
{
    // 记录当前时间的海平面以上高度作为我们的目标高度
    target_altitude.amsl_cm = current_loc.alt;

    // 重置任何滑翔坡度偏移
    reset_offset_altitude();

#if AP_TERRAIN_AVAILABLE
    // 如果可能，也记录地形高度
    float terrain_altitude;
    if (terrain_enabled_in_current_mode() && terrain.height_above_terrain(terrain_altitude, true) && !terrain_disabled()) {
        target_altitude.terrain_following = true;
        target_altitude.terrain_alt_cm = terrain_altitude*100;
    } else {
        // 如果地形跟随被禁用，或者在设置高度时我们不知道我们的
        // 地形高度，那么不进行地形跟随
        target_altitude.terrain_following = false;        
    }
#endif
}

/*
  设置目标高度为当前高度，并进行ALT_OFFSET调整
 */
void Plane::set_target_altitude_current_adjusted(void)
{
    set_target_altitude_current();

    // 使用adjusted_altitude_cm()来考虑ALTITUDE_OFFSET
    target_altitude.amsl_cm = adjusted_altitude_cm();
}

/*
  根据位置结构设置目标高度
 */
void Plane::set_target_altitude_location(const Location &loc)
{
    target_altitude.amsl_cm = loc.alt;
    if (loc.relative_alt) {
        target_altitude.amsl_cm += home.alt;
    }
#if AP_TERRAIN_AVAILABLE
    if (target_altitude.terrain_following_pending) {
        /* 当我们开始这个目标时，我们没有得到地形数据来初始化，重试 */
        setup_terrain_target_alt(next_WP_loc);
    }
    /*
      如果这个位置设置了terrain_alt标志，并且我们知道我们当前位置的
      地形高度，那么将其视为地形高度
     */
    float height;
    if (loc.terrain_alt && terrain.height_above_terrain(height, true)) {
        target_altitude.terrain_following = true;
        target_altitude.terrain_alt_cm = loc.alt;
        if (!loc.relative_alt) {
            // 它已经加上了home高度，去掉它
            target_altitude.terrain_alt_cm -= home.alt;
        }
    } else {
        target_altitude.terrain_following = false;
    }
#endif
}

/*
  返回相对于home的目标高度（以厘米为单位）。用于高度控制库
 */
int32_t Plane::relative_target_altitude_cm(void)
{
#if AP_TERRAIN_AVAILABLE
    float relative_home_height;
    if (target_altitude.terrain_following && 
        terrain.height_relative_home_equivalent(target_altitude.terrain_alt_cm*0.01f,
                                                relative_home_height, true)) {
        // 为目标高度添加前瞻调整
        target_altitude.lookahead = lookahead_adjustment();
        relative_home_height += target_altitude.lookahead;

#if AP_RANGEFINDER_ENABLED
        // 校正测距仪数据
        relative_home_height += rangefinder_correction();
#endif

        // 我们正在跟随地形，并且有当前位置的地形数据。使用它。
        return relative_home_height*100;
    }
#endif
    int32_t relative_alt = target_altitude.amsl_cm - home.alt;
    relative_alt += mission_alt_offset()*100;
#if AP_RANGEFINDER_ENABLED
    relative_alt += rangefinder_correction() * 100;
#endif
    return relative_alt;
}

/*
  改变当前目标高度，以厘米为单位。用于处理CRUISE或FBWB中由于升降舵引起的变化
 */
void Plane::change_target_altitude(int32_t change_cm)
{
    target_altitude.amsl_cm += change_cm;
#if AP_TERRAIN_AVAILABLE
    if (target_altitude.terrain_following && !terrain_disabled()) {
        target_altitude.terrain_alt_cm += change_cm;
    }
#endif
}
/*
  按目标高度偏移（从前一个WP到下一个WP的高度差）的比例改变目标高度。
  比例应在0到1之间。

  当比例为零时，我们已到达目的地。当比例为1时，我们在起始航点。

  注意，target_altitude最初是基于目标航点设置的
 */
void Plane::set_target_altitude_proportion(const Location &loc, float proportion)
{
    set_target_altitude_location(loc);
    proportion = constrain_float(proportion, 0.0f, 1.0f);
    change_target_altitude(-target_altitude.offset_cm*proportion);
    // 如果我们在滑翔坡度上方并且应该爬升，则重建滑翔坡度
    if(g.glide_slope_threshold > 0) {
        if(target_altitude.offset_cm > 0 && calc_altitude_error_cm() < -100 * g.glide_slope_threshold) {
            set_target_altitude_location(loc);
            set_offset_altitude_location(current
