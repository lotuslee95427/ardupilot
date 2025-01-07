/*
 * 本文件是自由软件:你可以在自由软件基金会发布的GNU通用公共许可证的条款下
 * 重新分发和/或修改它,可以选择使用版本3或更高版本的许可证。
 *
 * 本文件的发布是希望它能有用,但不提供任何保证;甚至没有对适销性或特定用途
 * 适用性的暗示保证。详细信息请参见GNU通用公共许可证。
 *
 * 你应该随程序收到一份GNU通用公共许可证的副本。如果没有,
 * 请参见<http://www.gnu.org/licenses/>。
 *
 * 代码作者: Andy Piper
 */

/*
  DSP设备接口
 */
#pragma once

#include <stdint.h>
#include "AP_HAL_Namespace.h"
#include <AP_HAL/utility/RingBuffer.h>

// 定义DSP内存区域为快速内存
#define DSP_MEM_REGION AP_HAL::Util::MEM_FAST
// 定义最大允许的信号丢失周期数
#define FFT_MAX_MISSED_UPDATES 5

class AP_HAL::DSP {
#if HAL_WITH_DSP
public:
    // 定义频率峰值类型枚举
    enum FrequencyPeak : uint8_t {
        CENTER = 0,           // 中心峰值
        LOWER_SHOULDER = 1,   // 低频肩部
        UPPER_SHOULDER = 2,   // 高频肩部
        MAX_TRACKED_PEAKS = 3,// 最大跟踪峰值数
        NONE = 4             // 无峰值
    };

    // 频率峰值数据结构
    struct FrequencyPeakData {
        float _freq_hz;         // FFT峰值频率估计值
        uint16_t _bin;          // 最大能量的FFT bin
        float _noise_width_hz;  // 峰值宽度
    };

    // 最大滑动窗口大小
    static const uint8_t MAX_SLIDING_WINDOW_SIZE = 8;

    // FFT窗口状态类
    class FFTWindowState {
    public:
        const float _bin_resolution;      // FFT bin的频率分辨率
        const uint16_t _bin_count;        // FFT bin数量
        const uint16_t _num_stored_freqs; // 存储的频率数量(bin_count + DC)
        const uint16_t _window_size;      // FFT窗口大小
        const uint8_t _sliding_window_size;// FFT滑动窗口大小
        float* _freq_bins;                // FFT频率bin数据
        float* _derivative_freq_bins;     // 导数实数据暂存空间
        float* _rfft_data;                // 中间实FFT数据
        float* _avg_freq_bins;            // 使用Welch方法的平均频率数据
        float* _sliding_window;           // bin_count帧的滑动窗口
        FrequencyPeakData _peak_data[MAX_TRACKED_PEAKS]; // 三个最高峰值
        float* _hanning_window;           // 输入样本的汉宁窗
        float _window_scale;              // 用于计算信号功率谱的窗口缩放因子
        bool _averaging;                  // 是否正在进行平均
        uint32_t _averaging_samples;      // 平均中的样本数
        uint8_t _current_slice;           // 当前滑动窗口片段

        // 从任意片段获取频率bin
        float get_freq_bin(uint16_t idx) { return _sliding_window == nullptr ? _freq_bins[idx] : _avg_freq_bins[idx]; }

        void free_data_structures();      // 释放数据结构
        virtual ~FFTWindowState();        // 析构函数
        FFTWindowState(uint16_t window_size, uint16_t sample_rate, uint8_t sliding_window_size); // 构造函数
    };

    // 初始化FFT实例
    virtual FFTWindowState* fft_init(uint16_t window_size, uint16_t sample_rate, uint8_t sliding_window_size = 0) = 0;
    // 使用ObjectBuffer开始FFT分析
    virtual void fft_start(FFTWindowState* state, FloatBuffer& samples, uint16_t advance) = 0;
    // 执行FFT分析的剩余步骤
    virtual uint16_t fft_analyse(FFTWindowState* state, uint16_t start_bin, uint16_t end_bin, float noise_att_cutoff) = 0;
    // 开始平均FFT数据
    bool fft_start_average(FFTWindowState* fft);
    // 结束平均过程
    uint16_t fft_stop_average(FFTWindowState* fft, uint16_t start_bin, uint16_t end_bin, float* peaks);

protected:
    // 步骤3:计算复数数据的幅度
    void step_cmplx_mag(FFTWindowState* fft, uint16_t start_bin, uint16_t end_bin, float noise_att_cutoff);
    // 基于输入参数计算峰值的噪声宽度
    float find_noise_width(float* freq_bins, uint16_t start_bin, uint16_t end_bin, uint16_t max_energy_bin, float cutoff,
        float bin_resolution, uint16_t& peak_top, uint16_t& peak_bottom) const;
    // 步骤4:找到能量最高的bin并插值计算所需频率
    uint16_t step_calc_frequencies(FFTWindowState* fft, uint16_t start_bin, uint16_t end_bin);
    // 计算最终平均输出
    void update_average_from_sliding_window(FFTWindowState* fft);
    // 计算单个频率
    uint16_t calc_frequency(FFTWindowState* fft, uint16_t start_bin, uint16_t peak_bin, uint16_t end_bin);
    // 在浮点数向量中找到最大值
    virtual void vector_max_float(const float* vin, uint16_t len, float* max_value, uint16_t* max_index) const = 0;
    // 计算浮点数向量的平均值
    virtual float vector_mean_float(const float* vin, uint16_t len) const = 0;
    // 将浮点数向量乘以缩放因子
    virtual void vector_scale_float(const float* vin, float scale, float* vout, uint16_t len) const = 0;
    // 将两个向量相加
    virtual void vector_add_float(const float* vin1, const float* vin2, float* vout, uint16_t len) const = 0;
    // 在噪声数据中查找峰值的算法
    uint16_t find_peaks(const float* input, uint16_t length, float* output, uint16_t* peaks, uint16_t peaklen, 
        float slopeThreshold, float ampThreshold, uint16_t smoothwidth, uint16_t peakgroup) const;
    uint16_t val2index(const float* vector, uint16_t n, float val) const;
    void derivative(const float* input, float* output, uint16_t n) const;
    void fastsmooth(float* input, uint16_t n, uint16_t smoothwidth) const;

    // Quinn频率插值器
    float calculate_quinns_second_estimator(const FFTWindowState* fft, const float* complex_fft, uint16_t k) const;
    float tau(const float x) const;
    // Jain估计器
    float calculate_jains_estimator(const FFTWindowState* fft, const float* real_fft, uint16_t k_max);
    // 初始化FFT数据平均
    bool fft_init_average(FFTWindowState* fft);

#endif // HAL_WITH_DSP
};
