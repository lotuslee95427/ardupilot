#include "SIMState.h"

#if AP_SIM_ENABLED && CONFIG_HAL_BOARD != HAL_BOARD_SITL

/*
 * 这是一个精简版的 AP_HAL_SITL 对象。我们应该让 PA_HAL_SITL 使用这个对象
 * - 通过将更多代码从那里移到这里来实现。
 */

// 包含各种仿真模型的头文件
#include <SITL/SIM_Multicopter.h>
#include <SITL/SIM_Helicopter.h>
#include <SITL/SIM_SingleCopter.h>
#include <SITL/SIM_Plane.h>
#include <SITL/SIM_Glider.h>
#include <SITL/SIM_QuadPlane.h>
#include <SITL/SIM_Rover.h>
#include <SITL/SIM_BalanceBot.h>
#include <SITL/SIM_Sailboat.h>
#include <SITL/SIM_MotorBoat.h>
#include <SITL/SIM_Tracker.h>
#include <SITL/SIM_Submarine.h>
#include <SITL/SIM_Blimp.h>
#include <SITL/SIM_NoVehicle.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#include <AP_Baro/AP_Baro.h>

extern const AP_HAL::HAL& hal;

using namespace AP_HAL;

#include <AP_Terrain/AP_Terrain.h>

// 根据不同的编译目标定义仿真框架类型
#ifndef AP_SIM_FRAME_CLASS
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#define AP_SIM_FRAME_CLASS MultiCopter
#elif APM_BUILD_TYPE(APM_BUILD_Heli)
#define AP_SIM_FRAME_CLASS Helicopter
#elif APM_BUILD_TYPE(APM_BUILD_AntennaTracker)
#define AP_SIM_FRAME_CLASS Tracker
#elif APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#define AP_SIM_FRAME_CLASS Plane
#elif APM_BUILD_TYPE(APM_BUILD_Rover)
#define AP_SIM_FRAME_CLASS SimRover
#elif APM_BUILD_TYPE(APM_BUILD_Blimp)
#define AP_SIM_FRAME_CLASS Blimp
#elif APM_BUILD_TYPE(APM_BUILD_ArduSub)
#define AP_SIM_FRAME_CLASS Submarine
#else
#define AP_SIM_FRAME_CLASS NoVehicle
#endif
#endif

// 根据不同的编译目标定义仿真框架字符串
#ifndef AP_SIM_FRAME_STRING
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#define AP_SIM_FRAME_STRING "+"
#elif APM_BUILD_TYPE(APM_BUILD_Heli)
#define AP_SIM_FRAME_STRING "heli"
#elif APM_BUILD_TYPE(APM_BUILD_AntennaTracker)
#define AP_SIM_FRAME_STRING "tracker"
#elif APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#define AP_SIM_FRAME_STRING "plane"
#elif APM_BUILD_TYPE(APM_BUILD_Rover)
#define AP_SIM_FRAME_STRING "rover"
#elif APM_BUILD_TYPE(APM_BUILD_Blimp)
#define AP_SIM_FRAME_STRING "blimp"
#elif APM_BUILD_TYPE(APM_BUILD_ArduSub)
#define AP_SIM_FRAME_STRING "sub"
#else
#define AP_SIM_FRAME_STRING ""
#endif
#endif


// 更新仿真状态
void SIMState::update()
{
    static bool init_done;
    if (!init_done) {
        init_done = true;
        sitl_model = SITL::AP_SIM_FRAME_CLASS::create(AP_SIM_FRAME_STRING);
    }

    _fdm_input_step();
}

/*
 * 设置 SITL 处理
 */
void SIMState::_sitl_setup(const char *home_str)
{
    _home_str = home_str;

    printf("Starting SITL input\n");
}


/*
 * 执行一个时间步长的飞行动力学模型(FDM)仿真
 */
void SIMState::_fdm_input_step(void)
{
    fdm_input_local();
}

/*
 * 从本地模型获取 FDM 输入
 */
void SIMState::fdm_input_local(void)
{
    struct sitl_input input;

    // 构建用于 FDM 的舵机结构
    _simulator_servos(input);

    // 更新模型的原点位置
    sitl_model->update_home();
    // 根据输入更新模型状态
    sitl_model->update_model(input);

    // 从模型获取 FDM 输出
    if (_sitl == nullptr) {
        _sitl = AP::sitl();
    }
    if (_sitl) {
        sitl_model->fill_fdm(_sitl->state);
    }

#if AP_SIM_SOLOGIMBAL_ENABLED
    // 更新云台模拟
    if (gimbal != nullptr) {
        gimbal->update();
    }
#endif
#if HAL_SIM_ADSB_ENABLED
    // 更新 ADS-B 模拟
    if (adsb != nullptr) {
        adsb->update();
    }
#endif

    // 更新各种传感器的模拟状态
    if (vicon != nullptr) {
        Quaternion attitude;
        sitl_model->get_attitude(attitude);
        vicon->update(sitl_model->get_location(),
                      sitl_model->get_position_relhome(),
                      sitl_model->get_velocity_ef(),
                      attitude);
    }

    // 更新各种测距仪的模拟
    if (benewake_tf02 != nullptr) {
        benewake_tf02->update(sitl_model->rangefinder_range());
    }
    // ... (其他测距仪的更新代码)

    // 更新 GPS 模拟
    for (uint8_t i=0; i<ARRAY_SIZE(gps); i++) {
        if (gps[i] != nullptr) {
            gps[i]->update();
        }
    }

    // 更新仿真时间
    if (_sitl) {
        hal.scheduler->stop_clock(_sitl->state.timestamp_us);
    } else {
        hal.scheduler->stop_clock(AP_HAL::micros64()+100);
    }

    // 设置相对地面高度
    set_height_agl();

    _synthetic_clock_mode = true;
    _update_count++;
}

/*
 * 创建用于发送到 FDM 的 sitl_input 结构
 */
void SIMState::_simulator_servos(struct sitl_input &input)
{
    // 按照选定的帧率输出
    uint32_t now = AP_HAL::micros();

    // 查找气压计对象(如果存在)
    const auto *_barometer = AP_Baro::get_singleton();

    float altitude = _barometer?_barometer->get_altitude():0;
    float wind_speed = 0;
    float wind_direction = 0;
    float wind_dir_z = 0;

    // 给气速传感器5秒钟在0风速下校准
    if (wind_start_delay_micros == 0) {
        wind_start_delay_micros = now;
    } else if (_sitl && (now - wind_start_delay_micros) > 5000000 ) {
        // EKF 不喜欢阶跃输入,所以使用这个低通滤波器来保持其稳定
        wind_speed =     _sitl->wind_speed_active     = (0.95f*_sitl->wind_speed_active)     + (0.05f*_sitl->wind_speed);
        wind_direction = _sitl->wind_direction_active = (0.95f*_sitl->wind_direction_active) + (0.05f*_sitl->wind_direction);
        wind_dir_z =     _sitl->wind_dir_z_active     = (0.95f*_sitl->wind_dir_z_active)     + (0.05f*_sitl->wind_dir_z);
        
        // 根据不同的风类型参数 SIM_WIND_T* 将风传递给模拟器
        switch (_sitl->wind_type) {
        case SITL::SIM::WIND_TYPE_SQRT:
            if (altitude < _sitl->wind_type_alt) {
                wind_speed *= sqrtf(MAX(altitude / _sitl->wind_type_alt, 0));
            }
            break;

        case SITL::SIM::WIND_TYPE_COEF:
            wind_speed += (altitude - _sitl->wind_type_alt) * _sitl->wind_type_coef;
            break;

        case SITL::SIM::WIND_TYPE_NO_LIMIT:
        default:
            break;
        }

        // 不允许负风速
        wind_speed = MAX(wind_speed, 0);
    }

    // 设置风的参数
    input.wind.speed = wind_speed;
    input.wind.direction = wind_direction;
    input.wind.turbulence = _sitl?_sitl->wind_turbulance:0;
    input.wind.dir_z = wind_dir_z;

    // 设置舵机输出
    for (uint8_t i=0; i<SITL_NUM_CHANNELS; i++) {
        if (pwm_output[i] == 0xFFFF) {
            input.servos[i] = 0;
        } else {
            input.servos[i] = pwm_output[i];
        }
    }

    // FETtec ESC 仿真支持
    if (_sitl != nullptr) {
        if (_sitl->fetteconewireesc_sim.enabled()) {
            _sitl->fetteconewireesc_sim.update_sitl_input_pwm(input);
            for (uint8_t i=0; i<ARRAY_SIZE(input.servos); i++) {
                if (input.servos[i] != 0 && input.servos[i] < 1000) {
                    AP_HAL::panic("Bad input servo value (%u)", input.servos[i]);
                }
            }
        }
    }

    // 电池电压和电流模拟
    float voltage = 0;
    _current = 0;
    
    if (_sitl != nullptr) {
        if (_sitl->state.battery_voltage <= 0) {
        } else {
            voltage = _sitl->state.battery_voltage;
            _current = _sitl->state.battery_current;
        }
    }

    // 假设使用 3DR 电源模块
    voltage_pin_value = ((voltage / 10.1f) / 5.0f) * 1024;
    current_pin_value = ((_current / 17.0f) / 5.0f) * 1024;
    // 将电池2模拟为电池1的25%增益
    voltage2_pin_value = ((voltage * 0.25f / 10.1f) / 5.0f) * 1024;
    current2_pin_value = ((_current * 0.25f / 17.0f) / 5.0f) * 1024;
}

/*
 * 设置相对地面的高度(米)
 */
void SIMState::set_height_agl(void)
{
    static float home_alt = -1;

    if (!_sitl) {
        return;
    }

    // 记住第一个非零高度作为起始高度
    if (is_equal(home_alt, -1.0f) && _sitl->state.altitude > 0) {
        home_alt = _sitl->state.altitude;
    }

#if AP_TERRAIN_AVAILABLE
    // 如果启用了地形功能,从 AP_Terrain 获取地面高度
    if (_sitl != nullptr &&
        _sitl->terrain_enable) {
        float terrain_height_amsl;
        Location location;
        location.lat = _sitl->state.latitude*1.0e7;
        location.lng = _sitl->state.longitude*1.0e7;

        AP_Terrain *_terrain = AP_Terrain::get_singleton();
        if (_terrain != nullptr &&
            _terrain->height_amsl(location, terrain_height_amsl)) {
            _sitl->state.height_agl = _sitl->state.altitude - terrain_height_amsl;
            return;
        }
    }
#endif

    // 如果无法获取地形高度,则使用平地模型
    if (_sitl != nullptr) {
        _sitl->state.height_agl = _sitl->state.altitude - home_alt;
    }
}

#endif  // AP_SIM_ENABLED && CONFIG_HAL_BOARD != HAL_BOARD_SITL
