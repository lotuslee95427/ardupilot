// 确保头文件只被包含一次
#pragma once

#include <SITL/SITL.h>

#if AP_SIM_ENABLED

#include <AP_HAL/AP_HAL.h>

// 包含所有SITL模拟器相关的头文件
#include <SITL/SITL_Input.h>
#include <SITL/SIM_SoloGimbal.h>  // Solo云台模拟器
#include <SITL/SIM_ADSB.h>        // ADS-B模拟器
#include <SITL/SIM_Vicon.h>       // Vicon运动捕捉系统模拟器
// 各种激光/雷达测距仪模拟器
#include <SITL/SIM_RF_Benewake_TF02.h>
#include <SITL/SIM_RF_Benewake_TF03.h>
#include <SITL/SIM_RF_Benewake_TFmini.h>
#include <SITL/SIM_RF_NoopLoop.h>
#include <SITL/SIM_RF_TeraRanger_Serial.h>
#include <SITL/SIM_RF_LightWareSerial.h>
#include <SITL/SIM_RF_LightWareSerialBinary.h>
#include <SITL/SIM_RF_Lanbao.h>
#include <SITL/SIM_RF_BLping.h>
#include <SITL/SIM_RF_LeddarOne.h>
#include <SITL/SIM_RF_RDS02UF.h>
#include <SITL/SIM_RF_USD1_v0.h>
#include <SITL/SIM_RF_USD1_v1.h>
#include <SITL/SIM_RF_MaxsonarSerialLV.h>
#include <SITL/SIM_RF_Wasp.h>
#include <SITL/SIM_RF_NMEA.h>
#include <SITL/SIM_RF_MAVLink.h>
#include <SITL/SIM_RF_GYUS42v2.h>
// 导航系统模拟器
#include <SITL/SIM_VectorNav.h>
#include <SITL/SIM_MicroStrain.h>
#include <SITL/SIM_InertialLabs.h>
#include <SITL/SIM_AIS.h>
#include <SITL/SIM_GPS.h>

// 发动机控制接口模拟器
#include <SITL/SIM_EFI_Hirth.h>

// 遥控系统模拟器
#include <SITL/SIM_Frsky_D.h>
#include <SITL/SIM_CRSF.h>
// 激光扫描仪模拟器
#include <SITL/SIM_PS_RPLidarA2.h>
#include <SITL/SIM_PS_TeraRangerTower.h>
#include <SITL/SIM_PS_LightWare_SF45B.h>

// 电机和ESC模拟器
#include <SITL/SIM_RichenPower.h>
#include <SITL/SIM_Loweheiser.h>
#include <SITL/SIM_FETtecOneWireESC.h>
#include <AP_HAL/utility/Socket_native.h>

#include <AP_HAL/AP_HAL_Namespace.h>

// SITL(Software In The Loop)状态类，用于管理所有模拟器组件
class AP_HAL::SIMState {
public:
    // 模拟的传感器数值
    uint16_t sonar_pin_value;     // 声呐传感器值 (pin 0)
    uint16_t airspeed_pin_value[2]; // 空速计值 (pin 1)
    uint16_t voltage_pin_value;   // 电压值 (pin 13)
    uint16_t current_pin_value;   // 电流值 (pin 12)
    uint16_t voltage2_pin_value;  // 第二电压值 (pin 15)
    uint16_t current2_pin_value;  // 第二电流值 (pin 14)

    // 更新模拟器状态
    void update();

#if HAL_SIM_GPS_ENABLED
    // 设置GPS模拟器
    void set_gps0(SITL::GPS *_gps) { gps[0] = _gps; }
#endif

    uint16_t pwm_output[32];  // PWM输出通道值

private:
    // 私有成员函数
    void _set_param_default(const char *parm);  // 设置默认参数
    void _sitl_setup(const char *home_str);     // SITL初始化设置
    void _setup_timer(void);                    // 定时器设置
    void _setup_adc(void);                      // ADC设置

    void set_height_agl(void);                  // 设置地面高度
    void _set_signal_handlers(void) const;      // 设置信号处理器

    void _update_airspeed(float airspeed);      // 更新空速
    void _simulator_servos(struct sitl_input &input); // 更新舵机模拟
    void _fdm_input_step(void);                 // 飞行动力学模型输入步进
    void fdm_input_local(void);                 // 本地飞行动力学模型输入

    void wait_clock(uint64_t wait_time_usec);   // 时钟等待

    uint16_t pwm_input[16];  // PWM输入通道值

    // 内部状态变量
    uint8_t _instance;           // 实例编号
    uint16_t _base_port;         // 基础端口号
    pid_t _parent_pid;           // 父进程ID
    uint32_t _update_count;      // 更新计数器

    SITL::SIM *_sitl;            // SITL实例指针
    uint16_t _rcin_port;         // 遥控输入端口
    uint16_t _fg_view_port;      // FlightGear视图端口
    uint16_t _irlock_port;       // IR锁定端口
    float _current;              // 当前电流值

    bool _synthetic_clock_mode;   // 合成时钟模式标志
    bool _use_rtscts;            // 使用硬件流控制标志
    bool _use_fg_view;           // 使用FlightGear视图标志
    
    const char *_fg_address;      // FlightGear地址

    // SITL内部模型
    SITL::Aircraft *sitl_model;   // 飞行器模型指针

#if AP_SIM_SOLOGIMBAL_ENABLED
    // Solo云台模拟
    bool enable_gimbal;
    SITL::SoloGimbal *gimbal;
#endif

#if HAL_SIM_ADSB_ENABLED
    // ADS-B模拟
    SITL::ADSB *adsb;
#endif

    // Vicon系统模拟
    SITL::Vicon *vicon;

    // 各种测距仪模拟器实例
    SITL::RF_Benewake_TF02 *benewake_tf02;
    SITL::RF_Benewake_TF03 *benewake_tf03;
    SITL::RF_Benewake_TFmini *benewake_tfmini;
    SITL::RF_Nooploop *nooploop;
    SITL::RF_TeraRanger_Serial *teraranger_serial;
    SITL::RF_LightWareSerial *lightwareserial;
    SITL::RF_LightWareSerialBinary *lightwareserial_binary;
    SITL::RF_Lanbao *lanbao;
    SITL::RF_BLping *blping;
    SITL::RF_LeddarOne *leddarone;
    SITL::RF_RDS02UF *rds02uf;
    SITL::RF_USD1_v0 *USD1_v0;
    SITL::RF_USD1_v1 *USD1_v1;
    SITL::RF_MaxsonarSerialLV *maxsonarseriallv;
    SITL::RF_Wasp *wasp;
    SITL::RF_NMEA *nmea;
    SITL::RF_MAVLink *rf_mavlink;
    SITL::RF_GYUS42v2 *gyus42v2;

    // Frsky设备模拟
    SITL::Frsky_D *frsky_d;

#if HAL_SIM_PS_RPLIDARA2_ENABLED
    // RPLidarA2激光雷达模拟
    SITL::PS_RPLidarA2 *rplidara2;
#endif

    // FETtec OneWire ESC模拟
    SITL::FETtecOneWireESC *fetteconewireesc;

#if HAL_SIM_PS_LIGHTWARE_SF45B_ENABLED
    // SF45B近程传感器模拟
    SITL::PS_LightWare_SF45B *sf45b;
#endif

#if HAL_SIM_PS_TERARANGERTOWER_ENABLED
    SITL::PS_TeraRangerTower *terarangertower;
#endif

#if AP_SIM_CRSF_ENABLED
    // CRSF设备模拟
    SITL::CRSF *crsf;
#endif

    // 导航系统模拟器实例
    SITL::VectorNav *vectornav;
    SITL::MicroStrain5 *microstrain5;
    SITL::MicroStrain7 *microstrain7;
    SITL::InertialLabs *inertiallabs;
    
#if HAL_SIM_JSON_MASTER_ENABLED
    // JSON SITL后端的附加实例
    SITL::JSON_Master ride_along;
#endif

#if HAL_SIM_AIS_ENABLED
    // AIS流模拟
    SITL::AIS *ais;
#endif

    // EFI设备模拟
    SITL::EFI_MegaSquirt *efi_ms;
    SITL::EFI_Hirth *efi_hirth;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // FlightGear视图输出套接字
    SocketAPM_native fg_socket{true};
#endif

    // 默认参数路径
    const char *defaults_path = HAL_PARAM_DEFAULTS_PATH;

    // 起始位置字符串
    const char *_home_str;

    // 风力开始延迟时间(微秒)
    uint32_t wind_start_delay_micros;

#if HAL_SIM_GPS_ENABLED
    // GPS设备模拟
    SITL::GPS *gps[2];  // 受参数集数量限制
#endif
};

#endif // AP_SIM_ENABLED
