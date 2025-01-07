/*
   本程序是自由软件：您可以根据自由软件基金会发布的GNU通用公共许可证的条款
   重新分发和/或修改它，可以选择使用版本3的许可证，或任何更新的版本。

   本程序的发布是希望它能够有用，但不负任何担保责任；甚至没有适销性或特定
   用途适用性的默示担保。详细信息请参见GNU通用公共许可证。

   您应该已经收到了GNU通用公共许可证的副本。如果没有，
   请查看 <http://www.gnu.org/licenses/>。
 */
#pragma once

// 光流传感器抽象类
class AP_HAL::OpticalFlow {
public:
    // 光流数据帧结构体
    class Data_Frame {
    public:
        float pixel_flow_x_integral;    // X方向像素流量积分值
        float pixel_flow_y_integral;    // Y方向像素流量积分值
        float gyro_x_integral;          // X轴陀螺仪积分值
        float gyro_y_integral;          // Y轴陀螺仪积分值
        uint32_t delta_time;            // 采样时间间隔(微秒)
        uint8_t quality;                // 图像质量指标(0-255)
    };

    // 初始化光流传感器
    virtual void init() = 0;
    
    // 读取一帧光流数据
    // @param frame: 存储读取数据的帧结构
    // @return: 读取成功返回true，失败返回false
    virtual bool read(Data_Frame& frame) = 0;
    
    // 更新陀螺仪数据
    // @param gyro_x: X轴角速度(弧度/秒)
    // @param gyro_y: Y轴角速度(弧度/秒)
    // @param dt: 采样时间间隔(秒)
    virtual void push_gyro(float gyro_x, float gyro_y, float dt) = 0;
    
    // 更新陀螺仪偏置
    // @param gyro_bias_x: X轴陀螺仪偏置
    // @param gyro_bias_y: Y轴陀螺仪偏置
    virtual void push_gyro_bias(float gyro_bias_x, float gyro_bias_y) = 0;
};
