#include "GCS_Plane.h"
#include "Plane.h"

// 返回当前飞行器的系统ID
uint8_t GCS_Plane::sysid_this_mav() const
{
    return plane.g.sysid_this_mav;
}

// 更新飞行器传感器状态标志
void GCS_Plane::update_vehicle_sensor_status_flags(void)
{
    // 检查反推力
    if (plane.have_reverse_thrust()) {
        control_sensors_present |= MAV_SYS_STATUS_REVERSE_MOTOR;
    }
    // 如果有反推力且油门为负值，则启用并标记为健康
    if (plane.have_reverse_thrust() && is_negative(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle))) {
        control_sensors_enabled |= MAV_SYS_STATUS_REVERSE_MOTOR;
        control_sensors_health |= MAV_SYS_STATUS_REVERSE_MOTOR;
    }

    // 设置飞行模式特定的传感器状态
    control_sensors_present |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION |
        MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
        MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;

    bool rate_controlled = false;
    bool attitude_stabilized = false;
    
    // 根据不同的飞行模式设置相应的控制标志
    switch (plane.control_mode->mode_number()) {
    case Mode::Number::MANUAL:
        break;

    case Mode::Number::ACRO:
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QACRO:
#endif
        rate_controlled = true;
        break;

    case Mode::Number::STABILIZE:
    case Mode::Number::FLY_BY_WIRE_A:
    case Mode::Number::AUTOTUNE:
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QSTABILIZE:
    case Mode::Number::QHOVER:
    case Mode::Number::QLAND:
    case Mode::Number::QLOITER:
#if QAUTOTUNE_ENABLED
    case Mode::Number::QAUTOTUNE:
#endif
#endif  // HAL_QUADPLANE_ENABLED
    case Mode::Number::FLY_BY_WIRE_B:
    case Mode::Number::CRUISE:
        rate_controlled = true;
        attitude_stabilized = true;
        break;

    case Mode::Number::TRAINING:
        // 在训练模式下，如果不是手动控制滚转或俯仰，则启用速率控制和姿态稳定
        if (!plane.training_manual_roll || !plane.training_manual_pitch) {
            rate_controlled = true;
            attitude_stabilized = true;
        }
        break;

    case Mode::Number::AUTO:
    case Mode::Number::RTL:
    case Mode::Number::LOITER:
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
    case Mode::Number::CIRCLE:
    case Mode::Number::TAKEOFF:
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QRTL:
    case Mode::Number::LOITER_ALT_QLAND:
#endif
    case Mode::Number::THERMAL:
        rate_controlled = true;
        attitude_stabilized = true;
        // 启用并标记为健康：偏航位置、高度控制和XY位置控制
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_YAW_POSITION;
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_YAW_POSITION;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        break;

    case Mode::Number::INITIALISING:
        break;
    }

    // 如果启用了速率控制，设置相应的标志
    if (rate_controlled) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D角速率控制
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D角速率控制
    }
    // 如果启用了姿态稳定，设置相应的标志
    if (attitude_stabilized) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;
    }

#if AP_TERRAIN_AVAILABLE
    // 更新地形跟随状态
    switch (plane.terrain.status()) {
    case AP_Terrain::TerrainStatusDisabled:
        break;
    case AP_Terrain::TerrainStatusUnhealthy:
        control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        break;
    case AP_Terrain::TerrainStatusOK:
        control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_health  |= MAV_SYS_STATUS_TERRAIN;
        break;
    }
#endif

#if AP_RANGEFINDER_ENABLED
    // 更新测距仪状态
    const RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder && rangefinder->has_orientation(plane.rangefinder_orientation())) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        if (plane.g.rangefinder_landing) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
        if (rangefinder->has_data_orient(plane.rangefinder_orientation())) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;            
        }
    }
#endif
}
