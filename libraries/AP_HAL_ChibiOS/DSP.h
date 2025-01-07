/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andy Piper and the betaflight team
 */
#pragma once

// 包含必要的头文件
#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_ChibiOS_Namespace.h"

// 如果启用了DSP功能
#if HAL_WITH_DSP

// 包含ARM数学库
#include <arm_math.h>

// 调试开关
#define DEBUG_FFT   0

// ChibiOS implementation of FFT analysis to run on STM32 processors
// ChibiOS DSP类,用于在STM32处理器上实现FFT分析
class ChibiOS::DSP : public AP_HAL::DSP {
public:
    // 初始化FFT实例
    virtual FFTWindowState* fft_init(uint16_t window_size, uint16_t sample_rate, uint8_t sliding_window_size) override;
    // 使用ObjectBuffer开始FFT分析
    virtual void fft_start(FFTWindowState* state, FloatBuffer& samples, uint16_t advance) override;
    // 执行FFT分析的剩余步骤
    virtual uint16_t fft_analyse(FFTWindowState* state, uint16_t start_bin, uint16_t end_bin, float noise_att_cutoff) override;

    // STM32-based FFT state
    // 基于STM32的FFT状态类
    class FFTWindowStateARM : public AP_HAL::DSP::FFTWindowState {
        friend class ChibiOS::DSP;
    public:
        // 构造和析构函数
        FFTWindowStateARM(uint16_t window_size, uint16_t sample_rate, uint8_t sliding_window_size);
        virtual ~FFTWindowStateARM();

    private:
        // underlying CMSIS data structure for FFT analysis
        // 用于FFT分析的CMSIS底层数据结构
        arm_rfft_fast_instance_f32 _fft_instance;
    };

protected:
    // 向量最大值计算函数
    void vector_max_float(const float* vin, uint16_t len, float* maxValue, uint16_t* maxIndex) const override {
        uint32_t mindex;
        arm_max_f32(vin, len, maxValue, &mindex);
        *maxIndex = mindex;
    }
    // 向量缩放函数
    void vector_scale_float(const float* vin, float scale, float* vout, uint16_t len) const override {
        arm_scale_f32(vin, scale, vout, len);
    }
    // 向量平均值计算函数
    float vector_mean_float(const float* vin, uint16_t len) const override {
        float mean_value;
        arm_mean_f32(vin, len, &mean_value);
        return mean_value;
    }
    // 向量加法函数
    void vector_add_float(const float* vin1, const float* vin2, float* vout, uint16_t len) const override {
        arm_add_f32(vin1, vin2, vout, len);
    }

private:
    // following are the six independent steps for calculating an FFT
    // 以下是计算FFT的六个独立步骤
    // 1. 汉宁窗处理
    void step_hanning(FFTWindowStateARM* fft, FloatBuffer& samples, uint16_t advance);
    // 2. ARM复数FFT计算
    void step_arm_cfft_f32(FFTWindowStateARM* fft);
    // 3. 位反转处理
    void step_bitreversal(FFTWindowStateARM* fft);
    // 4. 实数FFT阶段处理
    void step_stage_rfft_f32(FFTWindowStateARM* fft);
    // 5. 复数幅值计算
    void step_arm_cmplx_mag_f32(FFTWindowStateARM* fft, uint16_t start_bin, uint16_t end_bin, float noise_att_cutoff);
    // 6. 频率计算
    uint16_t step_calc_frequencies_f32(FFTWindowStateARM* fft, uint16_t start_bin, uint16_t end_bin);
    // Candan频率插值估计器
    float calculate_candans_estimator(const FFTWindowStateARM* fft, uint16_t k) const;

#if DEBUG_FFT
    // 用于调试的步骤计时器类
    class StepTimer {
    public:
        uint32_t _timer_total;    // 总计时
        uint32_t _timer_avg;      // 平均时间
        uint8_t _time_ticks;      // 计时周期

        void time(uint32_t start);
    };

    uint32_t  _output_count;                      // 输出计数
    StepTimer _hanning_timer;                     // 汉宁窗计时器
    StepTimer _arm_cfft_f32_timer;               // FFT计算计时器
    StepTimer _bitreversal_timer;                // 位反转计时器
    StepTimer _stage_rfft_f32_timer;             // 实数FFT计时器
    StepTimer _arm_cmplx_mag_f32_timer;          // 复数幅值计时器
    StepTimer _step_calc_frequencies;             // 频率计算计时器
#endif
};

#endif