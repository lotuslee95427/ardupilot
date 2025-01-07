// 包含ArduPilot硬件抽象层头文件
#include <AP_HAL/AP_HAL.h>

// 陀螺仪数据帧结构体
struct GyroFrame {
    float x[1024];    // X轴陀螺仪数据数组
    float y[1024];    // Y轴陀螺仪数据数组
    float z[1024];    // Z轴陀螺仪数据数组
};

// 陀螺仪数据帧数组
extern const GyroFrame gyro_frames[];
// 数据帧总数
extern const uint32_t NUM_FRAMES;
// 采样率
extern const uint16_t SAMPLE_RATE;