/*
该库在驻留测试或频率扫描测试期间接收时间历史数据(角速度或角度),并确定对输入的增益和相位响应。
对于驻留测试,一旦指定的周期数完成,就会确定最后5个周期的增益和相位的平均值,并设置cycle_complete标志。
对于频率扫描测试,每个周期都会确定相位和增益,并设置cycle_complete标志以指示何时提取相位和增益数据。
该标志会被重置以启用下一个周期的分析。在初始化驻留或频率扫描测试时必须使用init函数。
*/

#include <AP_HAL/AP_HAL.h>
#include "AC_AutoTune_FreqResp.h"

// 初始化频率响应对象。在运行驻留或频率扫描测试之前必须调用此函数
void AC_AutoTune_FreqResp::init(InputType input_type, ResponseType response_type, uint8_t cycles)
{
    excitation = input_type;        // 设置激励类型(驻留或扫频)
    response = response_type;       // 设置响应类型(角度或角速度)
    max_target_cnt = 0;            // 目标最大值计数器清零
    min_target_cnt = 0;            // 目标最小值计数器清零
    max_meas_cnt = 0;              // 测量最大值计数器清零
    min_meas_cnt = 0;              // 测量最小值计数器清零
    input_start_time_ms = 0;       // 输入开始时间清零
    new_tgt_time_ms = 0;           // 新目标时间清零
    new_meas_time_ms = 0;          // 新测量时间清零
    new_target = false;            // 新目标标志清零
    new_meas = false;              // 新测量标志清零
    curr_test_freq = 0.0f;         // 当前测试频率清零
    curr_test_gain = 0.0f;         // 当前测试增益清零
    curr_test_phase = 0.0f;        // 当前测试相位清零
    max_accel = 0.0f;              // 最大加速度清零
    max_meas_rate = 0.0f;          // 最大测量速率清零
    max_command = 0.0f;            // 最大命令值清零
    dwell_cycles = cycles;         // 设置驻留周期数
    meas_peak_info_buffer.clear(); // 清空测量峰值信息缓冲区
    tgt_peak_info_buffer.clear();  // 清空目标峰值信息缓冲区
    cycle_complete = false;        // 周期完成标志清零
}

// update_angle - 该函数在驻留和频率扫描测试期间接收时间历史数据用于角度P调谐,
// 并确定对输入的增益、相位和最大加速度响应。对于驻留测试,一旦指定的周期数完成,
// 就会确定最后5个周期的增益、相位和最大加速度的平均值,并设置cycles_complete标志。
// 对于频率扫描测试,每个周期都会确定相位和增益,并设置cycle_complete标志以指示何时提取相位和增益数据。
// 该标志会被重置以启用下一个周期的分析。
void AC_AutoTune_FreqResp::update(float command, float tgt_resp, float meas_resp, float tgt_freq)
{

    uint32_t now = AP_HAL::millis();  // 获取当前时间
    float dt = 0.0025;                // 设置时间步长
    uint32_t half_cycle_time_ms = 0;  // 半周期时间
    uint32_t cycle_time_ms = 0;       // 完整周期时间

    // 如果周期已完成则返回
    if (cycle_complete) {
        return;
    }

    // 根据目标频率计算半周期和完整周期时间
    if (!is_zero(tgt_freq)) {
        half_cycle_time_ms = (uint32_t)(300 * M_2PI / tgt_freq);
        cycle_time_ms = (uint32_t)(1000 * M_2PI / tgt_freq);
    }

    // 初始化输入开始时间和前一个响应值
    if (input_start_time_ms == 0) {
        input_start_time_ms = now;
        if (response == ANGLE) {
            prev_tgt_resp = tgt_resp;
            prev_meas_resp = meas_resp;
            prev_target = 0.0f;
            prev_meas = 0.0f;
        }
    }

    // 计算目标速率和测量速率
    if (response == ANGLE) {
        target_rate = (tgt_resp - prev_tgt_resp) / dt;
        measured_rate = (meas_resp - prev_meas_resp) / dt;
    } else {
        target_rate = tgt_resp;
        measured_rate = meas_resp;
    }

    // 驻留测试周期完成后,计算增益和相位并退出
    if (max_meas_cnt > dwell_cycles + 1 && max_target_cnt > dwell_cycles + 1 && excitation == DWELL) {
        float delta_time = 0.0f;      // 时间差
        float sum_gain = 0.0f;        // 增益和
        uint8_t cnt = 0;              // 计数器
        uint8_t gcnt = 0;             // 增益计数器
        uint16_t meas_cnt, tgt_cnt;   // 测量和目标计数
        float meas_ampl = 0.0f;       // 测量幅值
        float tgt_ampl = 0.0f;        // 目标幅值
        uint32_t meas_time = 0;       // 测量时间
        uint32_t tgt_time = 0;        // 目标时间

        // 处理指定数量的驻留周期
        for (uint8_t i = 0;  i < dwell_cycles; i++) {
            meas_cnt=0;
            tgt_cnt=0;
            pull_from_meas_buffer(meas_cnt, meas_ampl, meas_time);
            pull_from_tgt_buffer(tgt_cnt, tgt_ampl, tgt_time);
            push_to_meas_buffer(0, 0.0f, 0);
            push_to_tgt_buffer(0, 0.0f, 0);

            // 计算增益和相位
            if (meas_cnt == tgt_cnt && meas_cnt != 0) {
                if (tgt_ampl > 0.0f) {
                    sum_gain += meas_ampl / tgt_ampl;
                    gcnt++;
                }
                float d_time = (float)(meas_time - tgt_time);
                if (d_time < 2.0f * (float)cycle_time_ms) {
                    delta_time += d_time;
                    cnt++;
                }
            } else if (meas_cnt > tgt_cnt) {
                pull_from_tgt_buffer(tgt_cnt, tgt_ampl, tgt_time);
                push_to_tgt_buffer(0, 0.0f, 0);
            } else if (meas_cnt < tgt_cnt) {
                pull_from_meas_buffer(meas_cnt, meas_ampl, meas_time);
                push_to_meas_buffer(0, 0.0f, 0);
            }        
        }

        // 计算平均增益
        if (gcnt > 0) {
            curr_test_gain = sum_gain / gcnt;
        }
        // 计算平均时间差
        if (cnt > 0) {
            delta_time = delta_time / cnt;
        }
        // 计算相位
        curr_test_phase = tgt_freq * delta_time * 0.001f * 360.0f / M_2PI;
        if (curr_test_phase > 360.0f) {
            curr_test_phase = curr_test_phase - 360.0f;
        }

        // 对角度响应类型测试确定最大加速度
        float dwell_max_accel;
        if (response == ANGLE) {
            dwell_max_accel = tgt_freq * max_meas_rate * 5730.0f;
            if (!is_zero(max_command)) {
                // 根据输入大小归一化加速度
                dwell_max_accel = dwell_max_accel / (2.0f * max_command);
            }
            if (dwell_max_accel > max_accel) {
                max_accel = dwell_max_accel;
            }
        }

        curr_test_freq = tgt_freq;
        cycle_complete = true;
        return;
    }

    // 检测目标(输入)的正负半周期,以通知何时应该寻找最大值或最小值
    if (((response == ANGLE && is_positive(prev_target) && !is_positive(target_rate))
        || (response == RATE && !is_positive(prev_target) && is_positive(target_rate)))
        && !new_target && now > new_tgt_time_ms) {
        new_target = true;
        new_tgt_time_ms = now + half_cycle_time_ms;
        // 重置最大目标值
        max_target = 0.0f;
        max_target_cnt++;
        temp_min_target = min_target;
        if (min_target_cnt > 0) {
            sweep_tgt.max_time_m1 = temp_max_tgt_time;
            temp_max_tgt_time = max_tgt_time;
            sweep_tgt.count_m1 = min_target_cnt - 1;
            sweep_tgt.amplitude_m1 = temp_tgt_ampl;
            temp_tgt_ampl = temp_max_target - temp_min_target;
            if (excitation == DWELL) {
                push_to_tgt_buffer(min_target_cnt,temp_tgt_ampl,temp_max_tgt_time);
            }
        }

    } else if (((response == ANGLE && !is_positive(prev_target) && is_positive(target_rate))
               || (response == RATE && is_positive(prev_target) && !is_positive(target_rate)))
               && new_target && now > new_tgt_time_ms && max_target_cnt > 0) {
        new_target = false;
        new_tgt_time_ms = now + half_cycle_time_ms;
        min_target_cnt++;
        temp_max_target = max_target;
        min_target = 0.0f;
    }

    // 检测测量值(输出)的正负半周期,以通知何时应该寻找最大值或最小值
    if (((response == ANGLE && is_positive(prev_meas) && !is_positive(measured_rate))
         || (response == RATE && !is_positive(prev_meas) && is_positive(measured_rate)))
         && !new_meas && now > new_meas_time_ms && max_target_cnt > 0) {
        new_meas = true;
        new_meas_time_ms = now + half_cycle_time_ms;
        // 重置最大测量值
        max_meas = 0.0f;
        max_meas_cnt++;
        temp_min_meas = min_meas;
        if (min_meas_cnt > 0 && min_target_cnt > 0) {
            sweep_meas.max_time_m1 = temp_max_meas_time;
            temp_max_meas_time = max_meas_time;
            sweep_meas.count_m1 = min_meas_cnt - 1;
            sweep_meas.amplitude_m1 = temp_meas_ampl;
            temp_meas_ampl = temp_max_meas - temp_min_meas;
            if (excitation == DWELL) {
                push_to_meas_buffer(min_meas_cnt,temp_meas_ampl,temp_max_meas_time);
            }
            if (excitation == SWEEP) {
                float tgt_period = 0.001f * (temp_max_tgt_time - sweep_tgt.max_time_m1);
                if (!is_zero(tgt_period)) {
                    curr_test_freq = M_2PI / tgt_period;
                } else {
                    curr_test_freq = 0.0f;
                }
                if (!is_zero(sweep_tgt.amplitude_m1)) {
                    curr_test_gain = sweep_meas.amplitude_m1/sweep_tgt.amplitude_m1;
                } else {
                    curr_test_gain = 0.0f;
                }
                curr_test_phase = curr_test_freq * (float)(sweep_meas.max_time_m1 - sweep_tgt.max_time_m1) * 0.001f * 360.0f / M_2PI;
                cycle_complete = true;
            }
        } 
    } else if (((response == ANGLE && !is_positive(prev_meas) && is_positive(measured_rate))
                || (response == RATE && is_positive(prev_meas) && !is_positive(measured_rate)))
                && new_meas && now > new_meas_time_ms && max_meas_cnt > 0) {
        new_meas = false;
        new_meas_time_ms = now + half_cycle_time_ms;
        min_meas_cnt++;
        temp_max_meas = max_meas;
        min_meas = 0.0f;
    }

    // 更新目标最大最小值
    if (new_target) {
        if (tgt_resp > max_target) {
            max_target = tgt_resp;
            max_tgt_time = now;
        }
    } else {
        if (tgt_resp < min_target) {
            min_target = tgt_resp;
        }
    }

    // 更新测量最大最小值
    if (new_meas) {
        if (meas_resp > max_meas) {
            max_meas = meas_resp;
            max_meas_time = now;
        }
    } else {
        if (meas_resp < min_meas) {
            min_meas = meas_resp;
        }
    }

    // 对角度响应类型,在特定时间窗口内更新最大测量速率和命令值
    if (response == ANGLE) {
        if (now > (uint32_t)(input_start_time_ms + 7.0f * cycle_time_ms) && now < (uint32_t)(input_start_time_ms + 9.0f * cycle_time_ms)) {
            if (measured_rate > max_meas_rate) {
                max_meas_rate = measured_rate;
            }
            if (command > max_command) {
                max_command = command;
            }
        }
        prev_tgt_resp = tgt_resp;
        prev_meas_resp = meas_resp;
    }

    prev_target = target_rate;
    prev_meas = measured_rate;
}

// 将测量峰值信息推入缓冲区
void AC_AutoTune_FreqResp::push_to_meas_buffer(uint16_t count, float amplitude, uint32_t time_ms)
{
    peak_info sample;
    sample.curr_count = count;
    sample.amplitude = amplitude;
    sample.time_ms = time_ms;
    meas_peak_info_buffer.push(sample);
}

// 从缓冲区提取测量峰值信息
void AC_AutoTune_FreqResp::pull_from_meas_buffer(uint16_t &count, float &amplitude, uint32_t &time_ms)
{
    peak_info sample;
    if (!meas_peak_info_buffer.pop(sample)) {
        // 无样本时返回
        return;
    }
    count = sample.curr_count;
    amplitude = sample.amplitude;
    time_ms = sample.time_ms;
}

// 将目标峰值信息推入缓冲区
void AC_AutoTune_FreqResp::push_to_tgt_buffer(uint16_t count, float amplitude, uint32_t time_ms)
{
    peak_info sample;
    sample.curr_count = count;
    sample.amplitude = amplitude;
    sample.time_ms = time_ms;
    tgt_peak_info_buffer.push(sample);
}

// 从缓冲区提取目标峰值信息
void AC_AutoTune_FreqResp::pull_from_tgt_buffer(uint16_t &count, float &amplitude, uint32_t &time_ms)
{
    peak_info sample;
    if (!tgt_peak_info_buffer.pop(sample)) {
        // 无样本时返回
        return;
    }
    count = sample.curr_count;
    amplitude = sample.amplitude;
    time_ms = sample.time_ms;
}
