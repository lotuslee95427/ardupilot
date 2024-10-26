#include "Copter.h"

#if HAL_LOGGING_ENABLED

// 用于从AP_Logger日志内存中读取和写入数据包的代码
// 用于与用户交互以转储或擦除日志的代码

// 定义一个打包后的结构体，用于控制调优日志数据
struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;           // 日志数据包头部宏
    uint64_t time_us;            // 时间戳，单位微秒
    float    throttle_in;        // 输入油门值
    float    angle_boost;        // 角度增益
    float    throttle_out;       // 输出油门值
    float    throttle_hover;     // 悬停油门值
    float    desired_alt;        // 期望高度
    float    inav_alt;           // 惯性导航系统高度
    int32_t  baro_alt;           // 气压计高度
    float    desired_rangefinder_alt; // 期望测距仪高度
    float    rangefinder_alt;    // 实际测距仪高度
    float    terr_alt;           // 地形高度
    int16_t  target_climb_rate;  // 目标爬升率
    int16_t  climb_rate;         // 当前爬升率
};

// 写入控制调优数据包
void Copter::Log_Write_Control_Tuning()
{
    // 获取地形高度
    float terr_alt = 0.0f;
#if AP_TERRAIN_AVAILABLE
    if (!terrain.height_above_terrain(terr_alt, true)) {
        terr_alt = logger.quiet_nan(); // 如果无法获取地形高度，则设置为NaN
    }
#endif
    float des_alt_m = 0.0f; // 期望高度，单位米
    int16_t target_climb_rate_cms = 0; // 目标爬升率，单位厘米每秒
    if (!flightmode->has_manual_throttle()) {
        des_alt_m = pos_control->get_pos_target_z_cm() * 0.01f; // 获取位置控制器的目标高度并转换为米
        target_climb_rate_cms = pos_control->get_vel_target_z_cms(); // 获取位置控制器的目标垂直速度
    }

    float desired_rangefinder_alt;
#if AP_RANGEFINDER_ENABLED
    if (!surface_tracking.get_target_dist_for_logging(desired_rangefinder_alt)) {
        desired_rangefinder_alt = AP::logger().quiet_nan(); // 如果无法获取测距仪高度，则设置为NaN
    }
#else
    // 获取地面跟踪的高度
    desired_rangefinder_alt = AP::logger().quiet_nan(); // 如果未启用测距仪，则设置为NaN
#endif

    // 初始化控制调优数据包结构体
    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG), // 初始化日志头部
        time_us             : AP_HAL::micros64(), // 获取当前时间戳
        throttle_in         : attitude_control->get_throttle_in(), // 获取输入油门
        angle_boost         : attitude_control->angle_boost(), // 获取角度增益
        throttle_out        : motors->get_throttle(), // 获取输出油门
        throttle_hover      : motors->get_throttle_hover(), // 获取悬停油门
        desired_alt         : des_alt_m, // 设置期望高度
        inav_alt            : inertial_nav.get_position_z_up_cm() * 0.01f, // 获取惯性导航系统的高度并转换为米
        baro_alt            : baro_alt, // 设置气压计高度
        desired_rangefinder_alt : desired_rangefinder_alt, // 设置期望测距仪高度
#if AP_RANGEFINDER_ENABLED
        rangefinder_alt     : surface_tracking.get_dist_for_logging(), // 获取实际测距仪高度
#else
        rangefinder_alt     : AP::logger().quiet_nanf(), // 如果未启用测距仪，则设置为NaN
#endif
        terr_alt            : terr_alt, // 设置地形高度
        target_climb_rate   : target_climb_rate_cms, // 设置目标爬升率
        climb_rate          : int16_t(inertial_nav.get_velocity_z_up_cms()) // 获取当前爬升率并转换为int16_t
    };
    logger.WriteBlock(&pkt, sizeof(pkt)); // 将数据包写入日志
}

// 写入姿态数据包
void Copter::Log_Write_Attitude()
{
    attitude_control->Write_ANG(); // 写入角度数据
    attitude_control->Write_Rate(*pos_control); // 写入速率数据，传入位置控制器
}

// 写入PID控制器数据包
void Copter::Log_Write_PIDS()
{
   if (should_log(MASK_LOG_PID)) { // 检查是否应该记录PID数据
        logger.Write_PID(LOG_PIDR_MSG, attitude_control->get_rate_roll_pid().get_pid_info()); // 写入滚转PID信息
        logger.Write_PID(LOG_PIDP_MSG, attitude_control->get_rate_pitch_pid().get_pid_info()); // 写入俯仰PID信息
        logger.Write_PID(LOG_PIDY_MSG, attitude_control->get_rate_yaw_pid().get_pid_info()); // 写入偏航PID信息
        logger.Write_PID(LOG_PIDA_MSG, pos_control->get_accel_z_pid().get_pid_info() ); // 写入加速度Z轴PID信息
        if (should_log(MASK_LOG_NTUN) && (flightmode->requires_GPS() || landing_with_GPS())) { // 检查是否应该记录NTUN数据，并且飞行模式需要GPS或正在使用GPS着陆
            logger.Write_PID(LOG_PIDN_MSG, pos_control->get_vel_xy_pid().get_pid_info_x()); // 写入XY速度PID信息的X轴部分
            logger.Write_PID(LOG_PIDE_MSG, pos_control->get_vel_xy_pid().get_pid_info_y()); // 写入XY速度PID信息的Y轴部分
        }
    }
}

// 写入EKF和位置数据包
void Copter::Log_Write_EKF_POS()
{
    AP::ahrs().Log_Write(); // 调用AHRS系统的日志写入函数
}

// 定义一个打包后的结构体，用于int16_t类型的数据日志
struct PACKED log_Data_Int16t {
    LOG_PACKET_HEADER; // 日志数据包头部宏
    uint64_t time_us;  // 时间戳，单位微秒
    uint8_t id;        // 数据标识符
    int16_t data_value; // int16_t类型的数据值
};

// 写入一个int16_t类型的数据包
UNUSED_FUNCTION
void Copter::Log_Write_Data(LogDataID id, int16_t value)
{
    if (should_log(MASK_LOG_ANY)) { // 检查是否应该记录任何类型的数据
        struct log_Data_Int16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT16_MSG), // 初始化日志头部
            time_us     : AP_HAL::micros64(), // 获取当前时间戳
            id          : (uint8_t)id, // 设置数据标识符
            data_value  : value // 设置数据值
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt)); // 将数据包写入日志的关键区块
    }
}

// 定义一个打包后的结构体，用于uint16_t类型的数据日志
struct PACKED log_Data_UInt16t {
    LOG_PACKET_HEADER; // 日志数据包头部宏
    uint64_t time_us;  // 时间戳，单位微秒
    uint8_t id;        // 数据标识符
    uint16_t data_value; // uint16_t类型的数据值
};

// 写入一个uint16_t类型的数据包
UNUSED_FUNCTION 
void Copter::Log_Write_Data(LogDataID id, uint16_t value)
{
    if (should_log(MASK_LOG_ANY)) { // 检查是否应该记录任何类型的数据
        struct log_Data_UInt16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT16_MSG), // 初始化日志头部
            time_us     : AP_HAL::micros64(), // 获取当前时间戳
            id          : (uint8_t)id, // 设置数据标识符
            data_value  : value // 设置数据值
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt)); // 将数据包写入日志的关键区块
    }
}

// 定义一个打包后的结构体，用于int32_t类型的数据日志
struct PACKED log_Data_Int32t {
    LOG_PACKET_HEADER; // 日志数据包头部宏
    uint64_t time_us;  // 时间戳，单位微秒
    uint8_t id;        // 数据标识符
    int32_t data_value; // int32_t类型的数据值
};

// 写入一个int32_t类型的数据包
void Copter::Log_Write_Data(LogDataID id, int32_t value)
{
    if (should_log(MASK_LOG_ANY)) { // 检查是否应该记录任何类型的数据
        struct log_Data_Int32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT32_MSG), // 初始化日志头部
            time_us  : AP_HAL::micros64(), // 获取当前时间戳
            id          : (uint8_t)id, // 设置数据标识符
            data_value  : value // 设置数据值
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt)); // 将数据包写入日志的关键区块
    }
}

// 定义一个打包后的结构体，用于uint32_t类型的数据日志
struct PACKED log_Data_UInt32t {
    LOG_PACKET_HEADER; // 日志数据包头部宏
    uint64_t time_us;  // 时间戳，单位微秒
    uint8_t id;        // 数据标识符
    uint32_t data_value; // uint32_t类型的数据值
};

// 写入一个uint32_t类型的数据包
void Copter::Log_Write_Data(LogDataID id, uint32_t value)
{
    if (should_log(MASK_LOG_ANY)) { // 检查是否应该记录任何类型的数据
        struct log_Data_UInt32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT32_MSG), // 初始化日志头部
            time_us     : AP_HAL::micros64(), // 获取当前时间戳
            id          : (uint8_t)id, // 设置数据标识符
            data_value  : value // 设置数据值
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt)); // 将数据包写入日志的关键区块
    }
}

// 定义一个打包后的结构体，用于float类型的数据日志
struct PACKED log_Data_Float {
    LOG_PACKET_HEADER; // 日志数据包头部宏
    uint64_t time_us;  // 时间戳，单位微秒
    uint8_t id;        // 数据标识符
    float data_value;  // float类型的数据值
};

// 写入一个float类型的数据包
UNUSED_FUNCTION
void Copter::Log_Write_Data(LogDataID id, float value)
{
    if (should_log(MASK_LOG_ANY)) { // 检查是否应该记录任何类型的数据
        struct log_Data_Float pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_FLOAT_MSG), // 初始化日志头部
            time_us     : AP_HAL::micros64(), // 获取当前时间戳
            id          : (uint8_t)id, // 设置数据标识符
            data_value  : value // 设置数据值
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt)); // 将数据包写入日志的关键区块
    }
}

// 定义一个打包后的结构体，用于参数调优日志数据
struct PACKED log_ParameterTuning {
    LOG_PACKET_HEADER; // 日志数据包头部宏
    uint64_t time_us;  // 时间戳，单位微秒
    uint8_t  parameter;     // 当前调优的参数，例如39表示CH6_CIRCLE_RATE
    float    tuning_value;  // 调优函数内部使用的标准化值
    float    tuning_min;    // 调优的最小值
    float    tuning_max;    // 调优的最大值
};

// 写入参数调优数据包
void Copter::Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, float tune_min, float tune_max)
{
    struct log_ParameterTuning pkt_tune = {
        LOG_PACKET_HEADER_INIT(LOG_PARAMTUNE_MSG), // 初始化日志头部
        time_us        : AP_HAL::micros64(), // 获取当前时间戳
        parameter      : param, // 设置调优的参数标识符
        tuning_value   : tuning_val, // 设置调优值
        tuning_min     : tune_min, // 设置调优的最小值
        tuning_max     : tune_max // 设置调优的最大值
    };

    logger.WriteBlock(&pkt_tune, sizeof(pkt_tune)); // 将调优数据包写入日志
}

// 写入视频稳定化日志数据
void Copter::Log_Video_Stabilisation()
{
    if (!should_log(MASK_LOG_VIDEO_STABILISATION)) { // 检查是否应该记录视频稳定化数据
        return; // 如果不需要，则返回
    }
    ahrs.write_video_stabilisation(); // 调用AHRS系统的写入视频稳定化函数
}

// 定义一个打包后的结构体，用于系统辨识数据日志（数据类型D）
struct PACKED log_SysIdD {
    LOG_PACKET_HEADER; // 日志数据包头部宏
    uint64_t time_us; // 时间戳，单位微秒
    float    waveform_time; // 波形时间
    float    waveform_sample; // 波形采样值
    float    waveform_freq; // 波形频率
    float    angle_x; // X轴角度
    float    angle_y; // Y轴角度
    float    angle_z; // Z轴角度
    float    accel_x; // X轴加速度
    float    accel_y; // Y轴加速度
    float    accel_z; // Z轴加速度
};

// 写入系统辨识数据（数据类型D）包
void Copter::Log_Write_SysID_Data(float waveform_time, float waveform_sample, float waveform_freq, float angle_x, float angle_y, float angle_z, float accel_x, float accel_y, float accel_z)
{
#if MODE_SYSTEMID_ENABLED
    struct log_SysIdD pkt_sidd = {
        LOG_PACKET_HEADER_INIT(LOG_SYSIDD_MSG), // 初始化日志头部
        time_us         : AP_HAL::micros64(), // 获取当前时间戳
        waveform_time   : waveform_time, // 设置波形时间
        waveform_sample : waveform_sample, // 设置波形采样值
        waveform_freq   : waveform_freq, // 设置波形频率
        angle_x         : angle_x, // 设置X轴角度
        angle_y         : angle_y, // 设置Y轴角度
        angle_z         : angle_z, // 设置Z轴角度
        accel_x         : accel_x, // 设置X轴加速度
        accel_y         : accel_y, // 设置Y轴加速度
        accel_z         : accel_z // 设置Z轴加速度
    };
    logger.WriteBlock(&pkt_sidd, sizeof(pkt_sidd)); // 将系统辨识数据包写入日志
#endif
}

// 定义一个打包后的结构体，用于系统辨识设置日志数据
struct PACKED log_SysIdS {
    LOG_PACKET_HEADER; // 日志数据包头部宏
    uint64_t time_us; // 时间戳，单位微秒
    uint8_t  systemID_axis; // 系统辨识轴
    float    waveform_magnitude; // 波形幅度
    float    frequency_start; // 起始频率
    float    frequency_stop; // 结束频率
    float    time_fade_in; // 渐入时间
    float    time_const_freq; // 恒定频率时间
    float    time_record; // 记录时间
    float    time_fade_out; // 渐出时间
};

// 写入系统辨识设置数据包
void Copter::Log_Write_SysID_Setup(uint8_t systemID_axis, float waveform_magnitude, float frequency_start, float frequency_stop, float time_fade_in, float time_const_freq, float time_record, float time_fade_out)
{
#if MODE_SYSTEMID_ENABLED
    struct log_SysIdS pkt_sids = {
        LOG_PACKET_HEADER_INIT(LOG_SYSIDS_MSG), // 初始化日志头部
        time_us             : AP_HAL::micros64(), // 获取当前时间戳
        systemID_axis       : systemID_axis, // 设置系统辨识轴
        waveform_magnitude  : waveform_magnitude, // 设置波形幅度
        frequency_start     : frequency_start, // 设置起始频率
        frequency_stop      : frequency_stop, // 设置结束频率
        time_fade_in        : time_fade_in, // 设置渐入时间
        time_const_freq     : time_const_freq, // 设置恒定频率时间
        time_record         : time_record, // 设置记录时间
        time_fade_out       : time_fade_out // 设置渐出时间
    };
    logger.WriteBlock(&pkt_sids, sizeof(pkt_sids)); // 将系统辨识设置数据包写入日志
#endif
}

// 定义一个打包后的结构体，用于引导模式位置目标日志数据
struct PACKED log_Guided_Position_Target {
    LOG_PACKET_HEADER; // 日志数据包头部宏
    uint64_t time_us; // 时间戳，单位微秒
    uint8_t type; // 目标类型
    float pos_target_x; // X轴目标位置
    float pos_target_y; // Y轴目标位置
    float pos_target_z; // Z轴目标位置
    uint8_t terrain; // 地形标志，0表示高度基于EKF原点，1表示基于地形
    float vel_target_x; // X轴目标速度
    float vel_target_y; // Y轴目标速度
    float vel_target_z; // Z轴目标速度
    float accel_target_x; // X轴目标加速度
    float accel_target_y; // Y轴目标加速度
    float accel_target_z; // Z轴目标加速度
};

// 定义一个打包后的结构体，用于引导模式姿态目标日志数据
struct PACKED log_Guided_Attitude_Target {
    LOG_PACKET_HEADER; // 日志数据包头部宏
    uint64_t time_us; // 时间戳，单位微秒
    uint8_t type; // 目标类型
    float roll; // 俯仰角，单位弧度
    float pitch; // 横滚角，单位弧度
    float yaw; // 偏航角，单位弧度
    float roll_rate; // 俯仰角速度，单位弧度/秒
    float pitch_rate; // 横滚角速度，单位弧度/秒
    float yaw_rate; // 偏航角速度，单位弧度/秒
    float thrust; // 推力，范围0到1
    float climb_rate; // 爬升率，单位米/秒
};
#endif

// 写入引导模式位置目标
// pos_target 是纬度、经度、高度或相对于 EKF 原点的偏移量(单位:厘米)
// terrain 应为 0 表示 pos_target.z 是相对于 EKF 原点的高度,为 1 表示相对于地形的高度
// vel_target 单位是厘米/秒
void Copter::Log_Write_Guided_Position_Target(ModeGuided::SubMode target_type, const Vector3f& pos_target, bool terrain_alt, const Vector3f& vel_target, const Vector3f& accel_target)
{
    // 构造引导模式位置目标日志数据包
    const log_Guided_Position_Target pkt {
        LOG_PACKET_HEADER_INIT(LOG_GUIDED_POSITION_TARGET_MSG), // 初始化日志包头
        time_us         : AP_HAL::micros64(),                   // 当前时间戳(微秒)
        type            : (uint8_t)target_type,                 // 目标类型
        pos_target_x    : pos_target.x,                         // X轴目标位置
        pos_target_y    : pos_target.y,                         // Y轴目标位置
        pos_target_z    : pos_target.z,                         // Z轴目标位置
        terrain         : terrain_alt,                          // 地形标志
        vel_target_x    : vel_target.x,                         // X轴目标速度
        vel_target_y    : vel_target.y,                         // Y轴目标速度
        vel_target_z    : vel_target.z,                         // Z轴目标速度
        accel_target_x  : accel_target.x,                       // X轴目标加速度
        accel_target_y  : accel_target.y,                       // Y轴目标加速度
        accel_target_z  : accel_target.z                        // Z轴目标加速度
    };
    logger.WriteBlock(&pkt, sizeof(pkt));                       // 将数据包写入日志
}

// 写入引导模式姿态目标
// roll、pitch和yaw单位为弧度
// ang_vel: 角速度[横滚率、俯仰率、偏航率],单位为弧度/秒
// thrust范围为0到1
// climb_rate单位为米/秒
void Copter::Log_Write_Guided_Attitude_Target(ModeGuided::SubMode target_type, float roll, float pitch, float yaw, const Vector3f &ang_vel, float thrust, float climb_rate)
{
    // 构造引导模式姿态目标日志数据包
    const log_Guided_Attitude_Target pkt {
        LOG_PACKET_HEADER_INIT(LOG_GUIDED_ATTITUDE_TARGET_MSG), // 初始化日志包头
        time_us         : AP_HAL::micros64(),                   // 当前时间戳(微秒)
        type            : (uint8_t)target_type,                 // 目标类型
        roll            : degrees(roll),                        // 横滚角(弧度转换为角度)
        pitch           : degrees(pitch),                       // 俯仰角(弧度转换为角度)
        yaw             : degrees(yaw),                         // 偏航角(弧度转换为角度)
        roll_rate       : degrees(ang_vel.x),                   // 横滚角速度(弧度/秒转换为度/秒)
        pitch_rate      : degrees(ang_vel.y),                   // 俯仰角速度(弧度/秒转换为度/秒)
        yaw_rate        : degrees(ang_vel.z),                   // 偏航角速度(弧度/秒转换为度/秒)
        thrust          : thrust,                               // 推力值
        climb_rate      : climb_rate                           // 爬升率(米/秒)
    };
    logger.WriteBlock(&pkt, sizeof(pkt));                       // 将数据包写入日志
}

// 类型和单位信息可以在 libraries/AP_Logger/Logstructure.h 中找到
// 搜索 "log_Units" 可以找到单位信息
// 搜索 "Format characters" 可以找到字段类型信息
const struct LogStructure Copter::log_structure[] = {
    LOG_COMMON_STRUCTURES,  // 通用日志结构

// @LoggerMessage: PTUN - 参数调优信息
// @Description: 参数调优信息
// @URL: https://ardupilot.org/copter/docs/tuning.html#in-flight-tuning
// @Field: TimeUS: 系统启动后的时间
// @Field: Param: 正在调优的参数
// @Field: TunVal: 调优函数内使用的归一化值
// @Field: TunMin: 调优最小限制
// @Field: TunMax: 调优最大限制

    { LOG_PARAMTUNE_MSG, sizeof(log_ParameterTuning),
      "PTUN", "QBfff",         "TimeUS,Param,TunVal,TunMin,TunMax", "s----", "F----" },

// @LoggerMessage: CTUN - 控制调优信息
// @Description: 控制调优信息
// @Field: TimeUS: 系统启动后的时间
// @Field: ThI: 油门输入
// @Field: ABst: 角度增益
// @Field: ThO: 油门输出
// @Field: ThH: 计算的悬停油门
// @Field: DAlt: 期望高度
// @Field: Alt: 实际高度
// @Field: BAlt: 气压高度
// @Field: DSAlt: 期望测距仪高度
// @Field: SAlt: 实际测距仪高度
// @Field: TAlt: 地形高度
// @Field: DCRt: 期望爬升率
// @Field: CRt: 实际爬升率

// @LoggerMessage: D16 - 16位有符号整数存储
// @Description: 通用16位有符号整数存储
// @Field: TimeUS: 系统启动后的时间
// @Field: Id: 数据类型标识符
// @Field: Value: 值

// @LoggerMessage: DU16 - 16位无符号整数存储
// @Description: 通用16位无符号整数存储
// @Field: TimeUS: 系统启动后的时间
// @Field: Id: 数据类型标识符
// @Field: Value: 值

// @LoggerMessage: D32 - 32位有符号整数存储
// @Description: 通用32位有符号整数存储
// @Field: TimeUS: 系统启动后的时间
// @Field: Id: 数据类型标识符
// @Field: Value: 值

// @LoggerMessage: DFLT - 浮点数存储
// @Description: 通用浮点数存储
// @Field: TimeUS: 系统启动后的时间
// @Field: Id: 数据类型标识符
// @Field: Value: 值

// @LoggerMessage: DU32 - 32位无符号整数存储
// @Description: 通用32位无符号整数存储
// @Field: TimeUS: 系统启动后的时间
// @Field: Id: 数据类型标识符
// @Field: Value: 值

    { LOG_CONTROL_TUNING_MSG, sizeof(log_Control_Tuning),
      "CTUN", "Qffffffefffhh", "TimeUS,ThI,ABst,ThO,ThH,DAlt,Alt,BAlt,DSAlt,SAlt,TAlt,DCRt,CRt", "s----mmmmmmnn", "F----00B000BB" , true },
    { LOG_DATA_INT16_MSG, sizeof(log_Data_Int16t),         
      "D16",   "QBh",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_DATA_UINT16_MSG, sizeof(log_Data_UInt16t),         
      "DU16",  "QBH",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_DATA_INT32_MSG, sizeof(log_Data_Int32t),         
      "D32",   "QBi",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_DATA_UINT32_MSG, sizeof(log_Data_UInt32t),         
      "DU32",  "QBI",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_DATA_FLOAT_MSG, sizeof(log_Data_Float),         
      "DFLT",  "QBf",         "TimeUS,Id,Value", "s--", "F--" },

// @LoggerMessage: SIDD - 系统识别数据
// @Description: 系统识别数据
// @Field: TimeUS: 系统启动后的时间
// @Field: Time: 波形的时间参考
// @Field: Targ: 当前波形样本
// @Field: F: 瞬时波形频率
// @Field: Gx: X轴角度增量
// @Field: Gy: Y轴角度增量
// @Field: Gz: Z轴角度增量
// @Field: Ax: X轴速度增量
// @Field: Ay: Y轴速度增量
// @Field: Az: Z轴速度增量

    { LOG_SYSIDD_MSG, sizeof(log_SysIdD),
      "SIDD", "Qfffffffff",  "TimeUS,Time,Targ,F,Gx,Gy,Gz,Ax,Ay,Az", "ss-zkkkooo", "F---------" , true },

// @LoggerMessage: SIDS - 系统识别设置
// @Description: 系统识别设置
// @Field: TimeUS: 系统启动后的时间
// @Field: Ax: 被激励的轴
// @Field: Mag: 啁啾波形的幅度
// @Field: FSt: 啁啾开始频率
// @Field: FSp: 啁啾结束频率
// @Field: TFin: 达到啁啾最大幅度的时间
// @Field: TC: 啁啾开始前的恒定频率时间
// @Field: TR: 完成啁啾波形所需时间
// @Field: TFout: 啁啾结束后达到零幅度的时间

    { LOG_SYSIDS_MSG, sizeof(log_SysIdS),
      "SIDS", "QBfffffff",  "TimeUS,Ax,Mag,FSt,FSp,TFin,TC,TR,TFout", "s--ssssss", "F--------" , true },

// @LoggerMessage: GUIP - 引导模式位置目标信息
// @Description: 引导模式位置目标信息
// @Field: TimeUS: 系统启动后的时间
// @Field: Type: 引导模式类型
// @Field: pX: X轴目标位置
// @Field: pY: Y轴目标位置
// @Field: pZ: Z轴目标位置
// @Field: Terrain: Z轴目标位置是相对地形的高度
// @Field: vX: X轴目标速度
// @Field: vY: Y轴目标速度
// @Field: vZ: Z轴目标速度
// @Field: aX: X轴目标加速度
// @Field: aY: Y轴目标加速度
// @Field: aZ: Z轴目标加速度

    { LOG_GUIDED_POSITION_TARGET_MSG, sizeof(log_Guided_Position_Target),
      "GUIP",  "QBfffbffffff",    "TimeUS,Type,pX,pY,pZ,Terrain,vX,vY,vZ,aX,aY,aZ", "s-mmm-nnnooo", "F-BBB-BBBBBB" , true },

// @LoggerMessage: GUIA - 引导模式姿态目标信息
// @Description: 引导模式姿态目标信息
// @Field: TimeUS: 系统启动后的时间
// @Field: Type: 引导模式类型
// @Field: Roll: 目标横滚角
// @Field: Pitch: 目标俯仰角
// @Field: Yaw: 目标偏航角
// @Field: RollRt: 横滚角速率
// @Field: PitchRt: 俯仰角速率
// @Field: YawRt: 偏航角速率
// @Field: Thrust: 推力
// @Field: ClimbRt: 爬升率

    { LOG_GUIDED_ATTITUDE_TARGET_MSG, sizeof(log_Guided_Attitude_Target),
      "GUIA",  "QBffffffff",    "TimeUS,Type,Roll,Pitch,Yaw,RollRt,PitchRt,YawRt,Thrust,ClimbRt", "s-dddkkk-n", "F-000000-0" , true },
};

// 获取日志结构数量
uint8_t Copter::get_num_log_structures() const
{
    return ARRAY_SIZE(log_structure);
}

// 写入飞行器启动消息
void Copter::Log_Write_Vehicle_Startup_Messages()
{
    // AP_Logger只保证200(?)字节
    char frame_and_type_string[30];
    copter.motors->get_frame_and_type_string(frame_and_type_string, ARRAY_SIZE(frame_and_type_string));
    logger.Write_MessageF("%s", frame_and_type_string);                    // 写入机架类型字符串
    logger.Write_Mode((uint8_t)flightmode->mode_number(), control_mode_reason); // 写入飞行模式
    ahrs.Log_Write_Home_And_Origin();                                      // 写入Home点和原点信息
    gps.Write_AP_Logger_Log_Startup_messages();                            // 写入GPS启动消息
}

#endif // HAL_LOGGING_ENABLED
