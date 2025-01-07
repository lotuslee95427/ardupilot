#pragma once

/*
 增益和相位确定算法
*/

#include <AP_Math/AP_Math.h>

// 频率响应分析类,用于自动调谐过程中确定系统的增益和相位特性
class AC_AutoTune_FreqResp {
public:
    // 构造函数
    AC_AutoTune_FreqResp()
{
}

    // 输入类型枚举
    enum InputType {
        DWELL = 0,                 // 驻留测试
        SWEEP = 1,                 // 扫频测试
    };

    // 响应类型枚举
    enum ResponseType {
        RATE = 0,                  // 角速度响应
        ANGLE = 1,                 // 角度响应
    };

    // 初始化频率响应对象
    // 在运行驻留或频率扫描测试之前必须调用此函数
    void init(InputType input_type, ResponseType response_type, uint8_t cycles);

    // 基于角度响应确定增益和相位
    // 用于驻留测试或扫频测试
    void update(float command, float tgt_resp, float meas_resp, float tgt_freq);

    // 允许外部查询周期是否完成以及频率响应数据是否可用
    bool is_cycle_complete() { return cycle_complete;}

    // 重置周期完成标志
    void reset_cycle_complete() { cycle_complete = false; }

    // 频率响应数据访问器
    float get_freq() { return curr_test_freq; }      // 获取当前测试频率
    float get_gain() { return curr_test_gain; }      // 获取当前增益
    float get_phase() { return curr_test_phase; }    // 获取当前相位
    float get_accel_max() { return max_accel; }      // 获取最大加速度

private:
    // 新目标值搜索的开始时间。防止噪声过早启动新目标值的搜索
    uint32_t new_tgt_time_ms;

    // 搜索新目标峰值的标志
    bool new_target = false;

    // 目标最大值
    float max_target;

    // 当前周期中目标最大值的时间
    uint32_t max_tgt_time;

    // 目标最大值计数器
    uint16_t max_target_cnt;

    // 当前周期运行时保存先前确定的目标最大值
    float temp_max_target;

    // 当前周期运行时保存先前确定的目标最大值时间
    uint32_t temp_max_tgt_time;

    // 目标最小值
    float min_target;

    // 目标最小值计数器
    uint16_t min_target_cnt;

    // 当前周期运行时保存先前确定的目标最小值
    float temp_min_target;

    // 上一周期的目标最大值
    float prev_target;

    // 上一周期的目标最大响应
    float prev_tgt_resp;

    // 保存用于增益计算的目标幅值
    float temp_tgt_ampl;

    // 新测量值搜索的开始时间。防止噪声过早启动新测量值的搜索
    uint32_t new_meas_time_ms;

    // 搜索新测量峰值的标志
    bool new_meas = false;

    // 测量最大值
    float max_meas;

    // 当前周期中测量最大值的时间
    uint32_t max_meas_time;

    // 测量最大值计数器
    uint16_t max_meas_cnt;

    // 当前周期运行时保存先前确定的测量最大值
    float temp_max_meas;

    // 当前周期运行时保存先前确定的测量最大值时间
    uint32_t temp_max_meas_time;

    // 测量最小值
    float min_meas;

    // 测量最小值计数器
    uint16_t min_meas_cnt;

    // 当前周期运行时保存先前确定的测量最小值
    float temp_min_meas;

    // 上一周期的测量最大值
    float prev_meas;

    // 上一周期的测量最大响应
    float prev_meas_resp;

    // 保存用于增益计算的测量幅值
    float temp_meas_ampl;

    // 从角度数据计算的目标角速度
    float target_rate;

    // 从角度数据计算的测量角速度
    float measured_rate;

    // 保存输入开始时间以跟踪输入运行的时间长度
    uint32_t input_start_time_ms;

    // 指示一个振荡周期是否完成的标志
    bool cycle_complete = false;

    // 驻留激励需要完成的驻留周期数
    uint8_t dwell_cycles;

    // 当前测试频率、增益和相位
    float curr_test_freq;
    float curr_test_gain;
    float curr_test_phase;

    // 激励过程中测得的最大角速度,用于最大加速度计算
    float max_meas_rate;

    // 与最大角速度相关的最大命令值,用于最大加速度计算
    float max_command;

    // 测试期间确定的最大加速度(单位:cdss)
    float max_accel;

    // 频率响应对象的输入类型
    InputType excitation;

    // 频率响应对象的响应类型
    ResponseType response;

    // 扫频峰值查找数据结构,用于跟踪峰值数据
    struct sweep_peak_finding_data {
        uint16_t count_m1;         // 计数器
        float amplitude_m1;        // 幅值
        float max_time_m1;         // 最大值时间
    };

    // 扫频测量峰值数据
    sweep_peak_finding_data sweep_meas;

    // 扫频目标峰值数据
    sweep_peak_finding_data sweep_tgt;

    // 在环形缓冲区中存储增益数据的结构
    struct peak_info {
        uint16_t curr_count;       // 当前计数
        float amplitude;           // 幅值
        uint32_t time_ms;         // 时间戳
    };

    // 测量峰值数据的缓冲区对象
    ObjectBuffer<peak_info> meas_peak_info_buffer{12};

    // 目标峰值数据的缓冲区对象
    ObjectBuffer<peak_info> tgt_peak_info_buffer{12};

    // 将数据推入测量峰值数据缓冲区
    void push_to_meas_buffer(uint16_t count, float amplitude, uint32_t time_ms);

    // 从测量峰值数据缓冲区拉取数据
    void pull_from_meas_buffer(uint16_t &count, float &amplitude, uint32_t &time_ms);

    // 将数据推入目标峰值数据缓冲区
    void push_to_tgt_buffer(uint16_t count, float amplitude, uint32_t time_ms);

    // 从目标峰值数据缓冲区拉取数据
    void pull_from_tgt_buffer(uint16_t &count, float &amplitude, uint32_t &time_ms);

};
