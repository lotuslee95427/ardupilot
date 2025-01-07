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

// 包含必要的头文件
#include <hal.h>
#include "AP_HAL_ChibiOS.h"

#if HAL_WITH_DSP

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include "DSP.h"
#include <cmath>

using namespace ChibiOS;

// 调试宏定义,用于计时
#if DEBUG_FFT
#define TIMER_START(timer) \
void *istate = hal.scheduler->disable_interrupts_save(); \
uint32_t timer##now = AP_HAL::micros()
#define TIMER_END(timer) timer.time(timer##now); \
hal.scheduler->restore_interrupts(istate)
#else
#define TIMER_START(timer)
#define TIMER_END(timer)
#endif

// 定义计时周期
#define TICK_CYCLE 10

// 声明外部HAL实例引用
extern const AP_HAL::HAL& hal;

// 算法来源说明
// The algorithms originally came from betaflight but are now substantially modified based on theory and experiment.
// https://holometer.fnal.gov/GH_FFT.pdf "Spectrum and spectral density estimation by the Discrete Fourier transform (DFT),
// including a comprehensive list of window functions and some new flat-top windows." - Heinzel et. al is a great reference
// for understanding the underlying theory although we do not use spectral density here since time resolution is equally
// important as frequency resolution. Referred to as [Heinz] throughout the code.

// 初始化FFT状态机
// 参数:window_size - 窗口大小
//      sample_rate - 采样率
//      sliding_window_size - 滑动窗口大小
AP_HAL::DSP::FFTWindowState* DSP::fft_init(uint16_t window_size, uint16_t sample_rate, uint8_t sliding_window_size)
{
    // 创建新的FFT状态对象
    DSP::FFTWindowStateARM* fft = NEW_NOTHROW DSP::FFTWindowStateARM(window_size, sample_rate, sliding_window_size);
    // 检查内存分配是否成功
    if (fft == nullptr || fft->_hanning_window == nullptr || fft->_rfft_data == nullptr || fft->_freq_bins == nullptr || fft->_derivative_freq_bins == nullptr) {
        delete fft;
        return nullptr;
    }
    return fft;
}

// 开始FFT分析
// 参数:state - FFT状态对象
//      samples - 采样数据缓冲区
//      advance - 前进步长
void DSP::fft_start(FFTWindowState* state, FloatBuffer& samples, uint16_t advance)
{
    step_hanning((FFTWindowStateARM*)state, samples, advance);
}

// 执行FFT分析的剩余步骤
// 参数:state - FFT状态对象
//      start_bin - 起始频率bin
//      end_bin - 结束频率bin
//      noise_att_cutoff - 噪声衰减截止值
uint16_t DSP::fft_analyse(AP_HAL::DSP::FFTWindowState* state, uint16_t start_bin, uint16_t end_bin, float noise_att_cutoff)
{
    FFTWindowStateARM* fft = (FFTWindowStateARM*)state;
    // 执行FFT的各个步骤
    step_arm_cfft_f32(fft);
    step_bitreversal(fft);
    step_stage_rfft_f32(fft);
    step_arm_cmplx_mag_f32(fft, start_bin, end_bin, noise_att_cutoff);
    return step_calc_frequencies_f32(fft, start_bin, end_bin);
}

// FFT状态机构造函数
DSP::FFTWindowStateARM::FFTWindowStateARM(uint16_t window_size, uint16_t sample_rate, uint8_t sliding_window_size)
    : AP_HAL::DSP::FFTWindowState::FFTWindowState(window_size, sample_rate, sliding_window_size)
{
    // 检查内存分配
    if (_freq_bins == nullptr || _hanning_window == nullptr || _rfft_data == nullptr || _derivative_freq_bins == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Failed to allocate %u bytes for window %u for DSP",
            unsigned(sizeof(float) * (window_size * 3 + 2)), unsigned(window_size));
        return;
    }

    // 根据窗口大小初始化ARM FFT实例
    // 注意:不使用arm_rfft_fast_init_f32()以节省代码空间
    switch (window_size) {
    case 32:
        arm_rfft_32_fast_init_f32(&_fft_instance);
        break;
    case 64:
        arm_rfft_64_fast_init_f32(&_fft_instance);
        break;
    case 128:
        arm_rfft_128_fast_init_f32(&_fft_instance);
        break;
    case 256:
        arm_rfft_256_fast_init_f32(&_fft_instance);
        break;
#if defined(STM32H7)
// 仅在STM32H7上支持更大的FFT表
    case 512:
        arm_rfft_512_fast_init_f32(&_fft_instance);
        break;
    case 1024:
        arm_rfft_1024_fast_init_f32(&_fft_instance);
        break;
#endif
    }
}

// 析构函数
DSP::FFTWindowStateARM::~FFTWindowStateARM() {}

// 声明外部C函数
extern "C" {
    void stage_rfft_f32(arm_rfft_fast_instance_f32 *S, float32_t *p, float32_t *pOut);
    void arm_cfft_radix8by2_f32(arm_cfft_instance_f32 *S, float32_t *p1);
    void arm_cfft_radix8by4_f32(arm_cfft_instance_f32 *S, float32_t *p1);
    void arm_radix8_butterfly_f32(float32_t *pSrc, uint16_t fftLen, const float32_t *pCoef, uint16_t twidCoefModifier);
    void arm_bitreversal_32(uint32_t *pSrc, const uint16_t bitRevLen, const uint16_t *pBitRevTable);
}

// 步骤1:通过汉宁窗对输入样本进行滤波
void DSP::step_hanning(FFTWindowStateARM* fft, FloatBuffer& samples, uint16_t advance)
{
    TIMER_START(_hanning_timer);

    // 将样本数据复制到频率bin中并应用汉宁窗
    samples.peek(&fft->_freq_bins[0], fft->_window_size);
    samples.advance(advance);
    arm_mult_f32(&fft->_freq_bins[0], &fft->_hanning_window[0], &fft->_freq_bins[0], fft->_window_size);

    TIMER_END(_hanning_timer);
}

// 步骤2:执行复数FFT处理
void DSP::step_arm_cfft_f32(FFTWindowStateARM* fft)
{
    arm_cfft_instance_f32 *Sint = &(fft->_fft_instance.Sint);
    Sint->fftLen = fft->_fft_instance.fftLenRFFT / 2;

    TIMER_START(_arm_cfft_f32_timer);

    // 根据bin数量选择不同的FFT算法
    switch (fft->_bin_count) {
    case 16: // 窗口32
    case 128: // 窗口256
        arm_cfft_radix8by2_f32(Sint, fft->_freq_bins);
        break;
    case 32: // 窗口64
    case 256: // 窗口512
        arm_cfft_radix8by4_f32(Sint, fft->_freq_bins);
        break;
    case 64: // 窗口128
    case 512: // 窗口1024
        arm_radix8_butterfly_f32(fft->_freq_bins, fft->_bin_count, Sint->pTwiddle, 1);
        break;
    }

    TIMER_END(_arm_cfft_f32_timer);
}

// 步骤3:对输出进行位反转
void DSP::step_bitreversal(FFTWindowStateARM* fft)
{
    TIMER_START(_bitreversal_timer);
    
    arm_bitreversal_32((uint32_t *)fft->_freq_bins, fft->_fft_instance.Sint.bitRevLength, fft->_fft_instance.Sint.pBitRevTable);

    TIMER_END(_bitreversal_timer);
}

// 步骤4:将复数数据转换为实数数据
void DSP::step_stage_rfft_f32(FFTWindowStateARM* fft)
{
    TIMER_START(_stage_rfft_f32_timer);
    
    stage_rfft_f32(&fft->_fft_instance, fft->_freq_bins, fft->_rfft_data);

    TIMER_END(_stage_rfft_f32_timer);
}

// 步骤5:计算复数数据的幅值
void DSP::step_arm_cmplx_mag_f32(FFTWindowStateARM* fft, uint16_t start_bin, uint16_t end_bin, float noise_att_cutoff)
{
    TIMER_START(_arm_cmplx_mag_f32_timer);

    // 计算频率分量的幅值
    arm_cmplx_mag_squared_f32(&fft->_rfft_data[2], &fft->_freq_bins[1], fft->_bin_count - 1);
    fft->_freq_bins[0] = sq(fft->_rfft_data[0]);               // DC分量
    fft->_freq_bins[fft->_bin_count] = sq(fft->_rfft_data[1]); // 奈奎斯特频率
    fft->_rfft_data[fft->_window_size] = fft->_rfft_data[1]; // 插值器的奈奎斯特频率
    fft->_rfft_data[fft->_window_size + 1] = 0;

    step_cmplx_mag(fft, start_bin, end_bin, noise_att_cutoff);

    TIMER_END(_arm_cmplx_mag_f32_timer);
}

// 步骤6:找到能量最高的bin并插值计算所需频率
uint16_t DSP::step_calc_frequencies_f32(FFTWindowStateARM* fft, uint16_t start_bin, uint16_t end_bin)
{
    TIMER_START(_step_calc_frequencies);

    step_calc_frequencies(fft, start_bin, end_bin);

    TIMER_END(_step_calc_frequencies);

#if DEBUG_FFT
    _output_count++;
    // 大约每秒输出一次调试信息
    if (_output_count % 400 == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FFT(us): t1:%lu,t2:%lu,t3:%lu,t4:%lu,t5:%lu,t6:%lu",
                        _hanning_timer._timer_avg, _arm_cfft_f32_timer._timer_avg, _bitreversal_timer._timer_avg, _stage_rfft_f32_timer._timer_avg, _arm_cmplx_mag_f32_timer._timer_avg, _step_calc_frequencies._timer_avg);
    }
#endif

    return fft->_peak_data[CENTER]._bin;
}

// 定义常量
static const float PI_N = M_PI / 32.0f;
static const float CANDAN_FACTOR = tanf(PI_N) / PI_N;

// 使用Candan估计器计算中心频率
// 参考文献: http://users.metu.edu.tr/ccandan//pub_dir/FineDopplerEst_IEEE_SPL_June2011.pdf
float DSP::calculate_candans_estimator(const FFTWindowStateARM* fft, uint16_t k_max) const
{
    // 检查输入参数有效性
    if (k_max <= 1 || k_max == fft->_bin_count) {
        return 0.0f;
    }

    // 计算相邻bin的索引
    const uint16_t k_m1 = (k_max - 1) * 2;
    const uint16_t k_p1 = (k_max + 1) * 2;
    const uint16_t k = k_max * 2;

    // 计算实部和虚部的差值
    const float npr = fft->_rfft_data[k_m1] - fft->_rfft_data[k_p1];
    const float npc = fft->_rfft_data[k_m1 + 1] - fft->_rfft_data[k_p1 + 1];
    const float dpr = 2.0f * fft->_rfft_data[k] - fft->_rfft_data[k_m1] - fft->_rfft_data[k_p1];
    const float dpc = 2.0f * fft->_rfft_data[k + 1] - fft->_rfft_data[k_m1 + 1] - fft->_rfft_data[k_p1 + 1];

    // 计算实数部分
    const float realn = npr * dpr + npc * dpc;
    const float reald = dpr * dpr + dpc * dpc;

    // 检查除数是否为零
    if (is_zero(reald)) {
        return 0.0f;
    }

    // 计算频率偏移
    float d = CANDAN_FACTOR * (realn / reald);

    // 限制偏移范围在-0.5到0.5之间
    return constrain_float(d, -0.5f, 0.5f);
}

#if DEBUG_FFT
 // 调试用计时器
 void DSP::StepTimer::time(uint32_t start)
 {
    _timer_total += (AP_HAL::micros() - start);
    _time_ticks = (_time_ticks + 1) % TICK_CYCLE;
    if (_time_ticks == 0) {
        _timer_avg = _timer_total / TICK_CYCLE;
        _timer_total = 0;
    }
}
#endif

#endif
