/*
   本程序是自由软件:你可以在自由软件基金会发布的GNU通用公共许可证的条款下
   重新分发和/或修改它,可以选择使用版本3或更高版本的许可证。

   本程序的发布是希望它能有用,但不提供任何保证;甚至没有对适销性或特定用途
   适用性的暗示保证。详细信息请参见GNU通用公共许可证。

   你应该随程序收到一份GNU通用公共许可证的副本。如果没有,
   请参见<http://www.gnu.org/licenses/>。
 */
/*
  多旋翼自动调参支持库。基于ArduCopter原始自动调参代码,由Leonard Hall编写
  由Andrew Tridgell转换为库
 */
#pragma once

#include "AC_AutoTune_config.h"

#if AC_AUTOTUNE_ENABLED

#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AC_AttitudeControl/AC_PosControl.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include "AC_AutoTune_FreqResp.h"

// 自动调参轴向位掩码定义
#define AUTOTUNE_AXIS_BITMASK_ROLL            1    // 横滚轴
#define AUTOTUNE_AXIS_BITMASK_PITCH           2    // 俯仰轴  
#define AUTOTUNE_AXIS_BITMASK_YAW             4    // 偏航轴
#define AUTOTUNE_AXIS_BITMASK_YAW_D           8    // 偏航D参数

#define AUTOTUNE_SUCCESS_COUNT                4     // 需要冻结当前增益的成功迭代次数

// 地面站自动调参消息ID
#define AUTOTUNE_MESSAGE_STARTED 0        // 开始调参
#define AUTOTUNE_MESSAGE_STOPPED 1        // 停止调参
#define AUTOTUNE_MESSAGE_SUCCESS 2        // 调参成功
#define AUTOTUNE_MESSAGE_FAILED 3         // 调参失败
#define AUTOTUNE_MESSAGE_SAVED_GAINS 4    // 保存增益
#define AUTOTUNE_MESSAGE_TESTING 5        // 正在测试
#define AUTOTUNE_MESSAGE_TESTING_END 6    // 测试结束

#define AUTOTUNE_ANNOUNCE_INTERVAL_MS 2000 // 状态通告间隔(毫秒)

class AC_AutoTune
{
public:
    // 构造函数
    AC_AutoTune();

    // 主运行循环
    virtual void run();

    // 可能保存增益,在解除武装时调用
    void disarmed(const bool in_autotune_mode);

    // 停止调参,恢复增益
    void stop();

    // 自动调参辅助功能触发器
    void do_aux_function(const RC_Channel::AuxSwitchPos ch_flag);

protected:

    // 保存调参增益的虚函数,需要子类实现
    virtual void save_tuning_gains() = 0;

    // 重置自动调参,使增益不会再次保存,可以重新运行自动调参
    void reset() {
        mode = UNINITIALISED;
        axes_completed = 0;
        have_pilot_testing_command = false;
    }

    // 可调参的轴向
    enum class AxisType {
        ROLL = 0,                 // 横滚轴调参(角度或速率)
        PITCH = 1,                // 俯仰轴调参(角度或速率)
        YAW = 2,                  // 使用FLTE的偏航轴调参(角度或速率)
        YAW_D = 3,                // 使用D的偏航轴调参(角度或速率)
    };

    //
    // 必须由特定车辆子类提供的方法
    //
    virtual bool init(void) = 0;

    // 获取飞手期望的爬升速率
    virtual float get_pilot_desired_climb_rate_cms(void) const = 0;

    // 获取飞手期望的横滚、俯仰角度和偏航速率
    virtual void get_pilot_desired_rp_yrate_cd(float &roll_cd, float &pitch_cd, float &yaw_rate_cds) = 0;

    // 初始化位置控制器Z轴速度和加速度限制
    virtual void init_z_limits() = 0;

#if HAL_LOGGING_ENABLED
    // 在抖动期间以全速率记录PID
    virtual void log_pids() = 0;
#endif

    //
    // 加载和保存增益的方法
    //

    // 备份原始增益并准备开始调参
    virtual void backup_gains_and_initialise();

    // 切换到使用原始增益
    virtual void load_orig_gains() = 0;

    // 切换到上次成功自动调参找到的增益
    virtual void load_tuned_gains() = 0;

    // 加载测试间使用的增益。在测试模式的更新增益步骤中调用,在返回水平步骤之前设置增益
    virtual void load_intra_test_gains() = 0;

    // 加载下一次测试的增益。依赖于axis变量的设置
    virtual void load_test_gains() = 0;

    // 重置每个车辆的测试变量
    virtual void reset_vehicle_test_variables() = 0;

    // 重置每个车辆的更新增益变量
    virtual void reset_update_gain_variables() = 0;

    // 应该由每个车辆重写的测试初始化和运行方法
    virtual void test_init() = 0;
    virtual void test_run(AxisType test_axis, const float dir_sign) = 0;

    // 如果用户启用了横滚、俯仰或偏航轴的自动调参,返回true
    bool roll_enabled() const;
    bool pitch_enabled() const;
    bool yaw_enabled() const;
    bool yaw_d_enabled() const;

    // 更新速率P增益上调类型的增益
    virtual void updating_rate_p_up_all(AxisType test_axis)=0;

    // 更新速率D增益上调类型的增益
    virtual void updating_rate_d_up_all(AxisType test_axis)=0;

    // 更新速率D增益下调类型的增益
    virtual void updating_rate_d_down_all(AxisType test_axis)=0;

    // 更新角度P增益上调类型的增益
    virtual void updating_angle_p_up_all(AxisType test_axis)=0;

    // 更新角度P增益下调类型的增益
    virtual void updating_angle_p_down_all(AxisType test_axis)=0;

    // 为调参类型设置调参后的增益
    virtual void set_gains_post_tune(AxisType test_axis)=0;

    // 反转抖动测试方向
    virtual bool twitch_reverse_direction() = 0;

#if HAL_LOGGING_ENABLED
    virtual void Log_AutoTune() = 0;
    virtual void Log_AutoTuneDetails() = 0;
    virtual void Log_AutoTuneSweep() = 0;
#endif

    // 内部初始化函数,应从init()调用
    bool init_internals(bool use_poshold,
                        AC_AttitudeControl *attitude_control,
                        AC_PosControl *pos_control,
                        AP_AHRS_View *ahrs_view,
                        AP_InertialNav *inertial_nav);

    // 向用户发送调参状态的间歇性更新
    virtual void do_gcs_announcements() = 0;

    // 向用户发送测试后更新
    virtual void do_post_test_gcs_announcements() = 0;

    // 发送高级状态消息(如已启动、已停止)
    void update_gcs(uint8_t message_id) const;

    // 发送较低级别的步骤状态(如飞手覆盖激活)
    void send_step_string();

    // 将调参类型转换为字符串以报告
    const char *type_string() const;

    // 返回当前轴向字符串
    const char *axis_string() const;

    // 向GCS报告给定轴向的最终增益
    virtual void report_final_gains(AxisType test_axis) const = 0;

    // 为直升机自动调参添加的函数

    // 添加特定于直升机的额外更新增益函数
    // 子类用于更新速率FF上调调参类型增益的通用方法
    virtual void updating_rate_ff_up_all(AxisType test_axis)=0;

    // 子类用于更新最大增益调参类型的通用方法
    virtual void updating_max_gains_all(AxisType test_axis)=0;

    // 调参模式下执行的步骤
    enum StepType {
        WAITING_FOR_LEVEL = 0,    // 自动调参等待飞机返回水平状态后开始下一次抖动
        TESTING           = 1,    // 自动调参已开始测试并观察飞机运动结果
        UPDATE_GAINS      = 2,    // 自动调参已完成测试并基于结果更新增益
        ABORT             = 3     // 加载正常增益并返回WAITING_FOR_LEVEL
    };

    // 调参模式下测试步骤中执行的小步骤
    enum TuneType {
        RD_UP = 0,                // 速率D增益上调
        RD_DOWN = 1,              // 速率D增益下调
        RP_UP = 2,                // 速率P增益上调
        RFF_UP = 3,               // 速率FF增益上调
        SP_DOWN = 4,              // 角度P增益下调
        SP_UP = 5,                // 角度P增益上调
        MAX_GAINS = 6,            // 确定最大允许的稳定增益
        TUNE_CHECK = 7,           // 使用调参增益进行频率扫描
        TUNE_COMPLETE = 8         // 调参完成
    };
    TuneType tune_seq[6];         // 保存要执行的调参类型序列
    uint8_t tune_seq_curr;        // 当前调参序列步骤

    // 获取下一个调参类型
    void next_tune_type(TuneType &curr_tune_type, bool reset);

    // 为车辆设置可定制的调参序列
    virtual void set_tune_sequence() = 0;

    // get_axis_bitmask访问器
    virtual uint8_t get_axis_bitmask() const = 0;

    // get_testing_step_timeout_ms访问器
    virtual uint32_t get_testing_step_timeout_ms() const = 0;

    // 获取自动调参模式下的慢速位置保持姿态
    void get_poshold_attitude(float &roll_cd, float &pitch_cd, float &yaw_cd);

    // 要加载的增益类型
    enum GainType {
        GAIN_ORIGINAL   = 0,      // 原始增益
        GAIN_TEST       = 1,      // 测试增益
        GAIN_INTRA_TEST = 2,      // 测试间增益
        GAIN_TUNED      = 3,      // 调参增益
    } loaded_gains;
    void load_gains(enum GainType gain_type);

    // 自动调参模式(高级状态)
    enum TuneMode {
        UNINITIALISED = 0,        // 自动调参从未运行
        TUNING = 1,               // 自动调参正在测试增益
        SUCCESS = 2,              // 调参完成,用户正在飞行测试新增益
        FAILED = 3,               // 调参失败,用户使用原始增益飞行
    };
    TuneMode mode;                       // 见TuneMode了解允许的模式

    // 对象指针的副本,使代码更清晰
    AC_AttitudeControl *attitude_control;
    AC_PosControl *pos_control;
    AP_AHRS_View *ahrs_view;
    AP_InertialNav *inertial_nav;
    AP_Motors *motors;

    AxisType axis;                       // 当前调参的轴向。见AxisType枚举
    bool     positive_direction;         // false = 负方向调参(如横滚向左),true = 正方向调参(如横滚向右)
    StepType step;                       // 见StepType了解执行的步骤
    TuneType tune_type;                  // 见TuneType
    bool     twitch_first_iter;          // 抖动第一次迭代为true(用于表示我们必须步进姿态或速率目标)
    uint8_t  axes_completed;             // 已完成轴向的位掩码
    uint32_t step_start_time_ms;                    // 当前调参步骤的开始时间(用于超时检查)
    uint32_t step_time_limit_ms;                    // 当前自动调参过程的时间限制
    uint32_t level_start_time_ms;                   // 等待水平的开始时间
    int8_t   counter;                               // 调参增益计数器
    float    start_angle;                           // 开始角度
    float    start_rate;                            // 开始速率 - 父类和多旋翼
    float    test_accel_max;                        // 最大加速度变量
    float    desired_yaw_cd;                        // 调参期间的偏航航向 - 父类和传统直升机
    float    step_scaler;                           // 缩小最大目标步长的缩放器 - 父类和多旋翼

    LowPassFilterFloat  rotation_rate_filt;         // 滤波后的旋转速率(弧度/秒)

    // 当前正在调参的参数值的备份
    float    orig_roll_rp, orig_roll_ri, orig_roll_rd, orig_roll_rff, orig_roll_dff, orig_roll_fltt, orig_roll_smax, orig_roll_sp, orig_roll_accel, orig_roll_rate;
    float    orig_pitch_rp, orig_pitch_ri, orig_pitch_rd, orig_pitch_rff, orig_pitch_dff, orig_pitch_fltt, orig_pitch_smax, orig_pitch_sp, orig_pitch_accel, orig_pitch_rate;
    float    orig_yaw_rp, orig_yaw_ri, orig_yaw_rd, orig_yaw_rff, orig_yaw_dff, orig_yaw_fltt, orig_yaw_smax, orig_yaw_rLPF, orig_yaw_sp, orig_yaw_accel, orig_yaw_rate;
    bool     orig_bf_feedforward;

    // 当前正在调参的参数值
    float    tune_roll_rp, tune_roll_rd, tune_roll_sp, tune_roll_accel;
    float    tune_pitch_rp, tune_pitch_rd, tune_pitch_sp, tune_pitch_accel;
    float    tune_yaw_rp, tune_yaw_rLPF, tune_yaw_sp, tune_yaw_accel;
    float    tune_roll_rff, tune_pitch_rff, tune_yaw_rd, tune_yaw_rff;

    uint32_t announce_time;
    float lean_angle;
    float rotation_rate;
    float roll_cd, pitch_cd;

    // 直升机特定变量
    float    start_freq;                            // 驻留测试开始频率
    float    stop_freq;                             // 驻留测试结束频率

private:
    // 如果我们有良好的位置估计则返回true
    virtual bool position_ok();

    // 子类必须实现以指定最大/最小测试角度的方法:
    virtual float target_angle_max_rp_cd() const = 0;

    // 子类必须实现以指定最大/最小测试角度的方法:
    virtual float target_angle_max_y_cd() const = 0;

    // 子类必须实现以指定最大/最小测试角度的方法:
    virtual float target_angle_min_rp_cd() const = 0;

    // 子类必须实现以指定最大/最小测试角度的方法:
    virtual float target_angle_min_y_cd() const = 0;

    // 子类必须实现以指定最大/最小测试角度的方法:
    virtual float angle_lim_max_rp_cd() const = 0;

    // 子类必须实现以指定最大/最小测试角度的方法:
    virtual float angle_lim_neg_rpy_cd() const = 0;

    // 初始化位置控制器
    bool init_position_controller();

    // 主状态机用于使飞机保持水平、执行测试和更新增益
    // 直接用目标更新姿态控制器
    void control_attitude();

    // 如果飞机接近水平则返回true
    bool currently_level();

    bool     pilot_override;             // true = 飞手正在覆盖控制,因此我们暂时暂停调参
    bool     use_poshold;                // true = 启用位置保持
    bool     have_position;              // true = start_position有效
    Vector3f start_position;             // 保持位置时的目标,作为相对于EKF原点的偏移量(厘米,NEU坐标系)

    // 变量
    uint32_t override_time;                         // 飞手最后一次覆盖控制的时间

    // 最后一次飞手覆盖警告的时间(毫秒)
    uint32_t last_pilot_override_warning;

    // 如果我们曾经收到飞手测试调参增益的命令则为true。
    // 如果为true且当前正在使用调参增益,则解除武装时将保存
    bool have_pilot_testing_command;

};

#endif  // AC_AUTOTUNE_ENABLED
