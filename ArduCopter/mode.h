#pragma once

#include "Copter.h"
#include <AP_Math/chirp.h>
#include <AP_ExternalControl/AP_ExternalControl_config.h> // TODO 如果Copter.h已包含此头文件,为什么还需要再次包含?

#if ADVANCED_FAILSAFE
#include "afs_copter.h"
#endif

class Parameters;
class ParametersG2;

class GCS_Copter;

// 由Guided和Auto模式共享的自动起飞对象
// 位置控制器控制飞行器,但用户可以控制偏航
class _AutoTakeoff {
public:
    void run();
    void start(float complete_alt_cm, bool terrain_alt);
    bool get_completion_pos(Vector3p& pos_neu_cm);

    bool complete;          // 当起飞完成时为true

private:
    // 自动起飞不控制水平位置的高度阈值(相对于EKF原点)
    bool no_nav_active;
    float no_nav_alt_cm;

    // 自动起飞变量
    float complete_alt_cm;  // 完成高度,以cm为单位,相对于EKF原点或地形(取决于auto_takeoff_terrain_alt)
    bool terrain_alt;       // 如果高度是相对于地形,则为true
    Vector3p complete_pos;  // 目标起飞位置,以cm为单位,相对于EKF原点的偏移量
};

#if AC_PAYLOAD_PLACE_ENABLED
class PayloadPlace {
public:
    void run();
    void start_descent();
    bool verify();

    // 有效载荷放置状态枚举
    enum class State : uint8_t {
        FlyToLocation,    // 飞向目标位置
        Descent_Start,    // 开始下降
        Descent,          // 下降中
        Release,          // 释放
        Releasing,        // 释放中
        Delay,            // 延迟
        Ascent_Start,     // 开始上升
        Ascent,           // 上升中
        Done,             // 完成
    };

    // 这些由Mission代码设置:
    State state = State::Descent_Start; // 记录有效载荷放置状态
    float descent_max_cm;               // 最大下降高度(cm)

private:

    uint32_t descent_established_time_ms; // 下降建立时间(毫秒)
    uint32_t place_start_time_ms;         // 放置开始时间(毫秒)
    float descent_thrust_level;           // 下降推力水平
    float descent_start_altitude_cm;      // 下降开始高度(cm)
    float descent_speed_cms;              // 下降速度(cm/s)
};
#endif

class Mode {
    friend class PayloadPlace;

public:

    // 自动驾驶模式枚举
    enum class Number : uint8_t {
        STABILIZE =     0,  // 手动机体角度控制,手动油门
        ACRO =          1,  // 手动机体角速率控制,手动油门
        ALT_HOLD =      2,  // 手动机体角度控制,自动油门
        AUTO =          3,  // 全自动航点控制,使用任务命令
        GUIDED =        4,  // 全自动飞向坐标或按速度/方向飞行,使用GCS即时命令
        LOITER =        5,  // 自动水平加速度控制,自动油门
        RTL =           6,  // 自动返回起飞点
        CIRCLE =        7,  // 自动圆周飞行,自动油门
        LAND =          9,  // 自动降落,水平位置控制
        DRIFT =        11,  // 半自主位置、偏航和油门控制
        SPORT =        13,  // 手动地球坐标系角速率控制,手动油门
        FLIP =         14,  // 自动翻转
        AUTOTUNE =     15,  // 自动调谐飞行器的横滚和俯仰增益
        POSHOLD =      16,  // 自动位置保持,可手动覆盖,自动油门
        BRAKE =        17,  // 使用惯性/GPS系统全制动,无需飞行员输入
        THROW =        18,  // 使用惯性/GPS系统的抛飞模式,无需飞行员输入
        AVOID_ADSB =   19,  // 自动避开宏观障碍物 - 如全尺寸飞机
        GUIDED_NOGPS = 20,  // 引导模式,但只接受姿态和高度
        SMART_RTL =    21,  // 智能RTL,通过重新追踪步骤返回起飞点
        FLOWHOLD  =    22,  // 使用光流保持位置,不使用测距仪
        FOLLOW    =    23,  // 尝试跟随另一架飞行器或地面站
        ZIGZAG    =    24,  // 能够以预定义的A点和B点之间的之字形飞行
        SYSTEMID  =    25,  // 系统识别模式,在控制器中产生自动系统识别信号
        AUTOROTATE =   26,  // 自主自转
        AUTO_RTL =     27,  // 自动RTL,这不是一个真正的模式,如果进入执行DO_LAND_START着陆序列,AUTO将报告为此模式
        TURTLE =       28,  // 坠毁后翻转

        // 模式编号127保留给Skybrush分支中的"无人机表演模式"
        // 详见 https://github.com/skybrush-io/ardupilot
    };

    // 构造函数
    Mode(void);

    // 不允许复制
    CLASS_NO_COPY(Mode);

    friend class _AutoTakeoff;

    // 返回此模式的唯一编号
    virtual Number mode_number() const = 0;

    // 子类应重写这些方法
    virtual bool init(bool ignore_checks) {
        return true;
    }
    virtual void exit() {};
    virtual void run() = 0;
    virtual bool requires_GPS() const = 0;
    virtual bool has_manual_throttle() const = 0;
    virtual bool allows_arming(AP_Arming::Method method) const = 0;
    virtual bool is_autopilot() const { return false; }
    virtual bool has_user_takeoff(bool must_navigate) const { return false; }
    virtual bool in_guided_mode() const { return false; }
    virtual bool logs_attitude() const { return false; }
    virtual bool allows_save_trim() const { return false; }
    virtual bool allows_autotune() const { return false; }
    virtual bool allows_flip() const { return false; }
    virtual bool crash_check_enabled() const { return true; }

#if ADVANCED_FAILSAFE
    // 返回此模式的类型,用于高级故障安全
    virtual AP_AdvancedFailsafe_Copter::control_mode afs_mode() const { return AP_AdvancedFailsafe_Copter::control_mode::AFS_STABILIZED; }
#endif

    // 返回true,如果从GCS或脚本解锁时可以跳过油门高检查
    virtual bool allows_GCS_or_SCR_arming_with_throttle_high() const { return false; }

#if FRAME_CONFIG == HELI_FRAME
    virtual bool allows_inverted() const { return false; };
#endif

    // 返回此飞行模式的字符串
    virtual const char *name() const = 0;
    virtual const char *name4() const = 0;

    bool do_user_takeoff(float takeoff_alt_cm, bool must_navigate);
    virtual bool is_taking_off() const;
    static void takeoff_stop() { takeoff.stop(); }

    virtual bool is_landing() const { return false; }

    // 模式需要地形数据才能正常工作
    virtual bool requires_terrain_failsafe() const { return false; }

    // 用于向GCS报告的函数
    virtual bool get_wp(Location &loc) const { return false; };
    virtual int32_t wp_bearing() const { return 0; }
    virtual uint32_t wp_distance() const { return 0; }
    virtual float crosstrack_error() const { return 0.0f;}

    // 支持MAV_CMD_DO_CHANGE_SPEED命令的函数
    virtual bool set_speed_xy(float speed_xy_cms) {return false;}
    virtual bool set_speed_up(float speed_xy_cms) {return false;}
    virtual bool set_speed_down(float speed_xy_cms) {return false;}

    int32_t get_alt_above_ground_cm(void);

    // 飞行员输入处理
    void get_pilot_desired_lean_angles(float &roll_out_cd, float &pitch_out_cd, float angle_max_cd, float angle_limit_cd) const;
    Vector2f get_pilot_desired_velocity(float vel_max) const;
    float get_pilot_desired_yaw_rate() const;
    float get_pilot_desired_throttle() const;

    // 返回经过避障和高度限制调整后的目标爬升速率
    float get_avoidance_adjusted_climbrate(float target_rate);

    const Vector3f& get_vel_desired_cms() {
        // 注意,并非每个模式都使用位置控制,所以这可能返回无效数据:
        return pos_control->get_vel_desired_cms();
    }

    // 向电机输出,可被子类重写
    virtual void output_to_motors();

    // 返回true,如果应使用飞行员的偏航输入来调整飞行器的航向
    virtual bool use_pilot_yaw() const {return true; }

    // 暂停和恢复模式
    virtual bool pause() { return false; };
    virtual bool resume() { return false; };

    // 处理飞行器在地面等待起飞的情况
    void make_safe_ground_handling(bool force_throttle_unlimited = false);

    // 当前模式是否允许风向标
#if WEATHERVANE_ENABLED
    virtual bool allows_weathervaning() const { return false; }
#endif

protected:

    // 辅助函数
    bool is_disarmed_or_landed() const;
    void zero_throttle_and_relax_ac(bool spool_up = false);
    void zero_throttle_and_hold_attitude();

    // 返回停止点作为一个位置,高度相对于原点
    Location get_stopping_point() const;

    // 控制正常降落的函数。pause_descent为true时飞行器不应下降
    void land_run_horizontal_control();
    void land_run_vertical_control(bool pause_descent = false);
    void land_run_horiz_and_vert_control(bool pause_descent = false) {
        land_run_horizontal_control();
        land_run_vertical_control(pause_descent);
    }

#if AC_PAYLOAD_PLACE_ENABLED
    // 有效载荷放置飞行行为:
    static PayloadPlace payload_place;
#endif

    // 运行正常或精确降落(如果启用)
    // pause_descent为true时飞行器不应下降
    void land_run_normal_or_precland(bool pause_descent = false);

#if AC_PRECLAND_ENABLED
    // 移动到精确着陆状态机命令的位置以重试着陆
    // 传入的位置预期为NED坐标系,单位为米
    void precland_retry_position(const Vector3f &retry_pos);

    // 运行精确着陆状态机。任何想要进行精确着陆的模式都应调用此函数。
    // 这处理从精确着陆、精确着陆失败到重试和故障安全措施的所有内容
    void precland_run();
#endif

    // 返回预期的悬停油门设置:
    virtual float throttle_hover() const;

    // 基于Alt_Hold的飞行模式状态,用于Alt_Hold、Loiter和Sport模式
    enum class AltHoldModeState {
        MotorStopped,        // 电机停止
        Takeoff,             // 起飞
        Landed_Ground_Idle,  // 着陆地面怠速
        Landed_Pre_Takeoff,  // 着陆前起飞
        Flying               // 飞行中
    };
    AltHoldModeState get_alt_hold_state(float target_climb_rate_cms);

    // 便捷引用,以避免代码转换中的大量修改:
    Parameters &g;
    ParametersG2 &g2;
    AC_WPNav *&wp_nav;
    AC_Loiter *&loiter_nav;
    AC_PosControl *&pos_control;
    AP_InertialNav &inertial_nav;
    AP_AHRS &ahrs;
    AC_AttitudeControl *&attitude_control;
    MOTOR_CLASS *&motors;
    RC_Channel *&channel_roll;
    RC_Channel *&channel_pitch;
    RC_Channel *&channel_throttle;
    RC_Channel *&channel_yaw;
    float &G_Dt;

    // 注意,我们支持两种完全不同的自动起飞:

    // "用户起飞",可用于ALT_HOLD等模式
    // (参见has_user_takeoff方法)。"用户起飞"是一个简单的
    // 基于飞行员输入或参数达到高度的例程。

    // "自动起飞"由Guided和Auto模式使用,基本上是
    // 允许飞行员控制偏航的航点导航。

    // 用户起飞支持;起飞状态在所有模式实例间共享
    class _TakeOff {
    public:
        void start(float alt_cm);
        void stop();
        void do_pilot_takeoff(float& pilot_climb_rate);
        bool triggered(float target_climb_rate) const;

        bool running() const { return _running; }
    private:
        bool _running;
        float take_off_start_alt;
        float take_off_complete_alt;
    };

    static _TakeOff takeoff;

    virtual bool do_user_takeoff_start(float takeoff_alt_cm);

    static _AutoTakeoff auto_takeoff;

public:
    // 导航偏航控制类
    class AutoYaw {

    public:

        // 自动驾驶偏航模式枚举类型
        enum class Mode {
            HOLD =             0,  // 保持零偏航速率
            LOOK_AT_NEXT_WP =  1,  // 指向下一个航点（不接受飞行员输入）
            ROI =              2,  // 指向ROI中的位置（区域兴趣点，不接受飞行员输入）
            FIXED =            3,  // 指向特定角度（不接受飞行员输入）
            LOOK_AHEAD =       4,  // 指向飞机移动的方向
            RESETTOARMEDYAW =  5,  // 指向电机启动时的航向
            ANGLE_RATE =       6,  // 从起始角度按指定速率转向
            RATE =             7,  // 按指定速率转向（使用auto_yaw_rate）
            CIRCLE =           8,  // 使用AC_Circle提供的偏航（用于徘徊-转弯命令期间）
            PILOT_RATE =       9,  // 来自飞行员摇杆的目标速率
            WEATHERVANE =     10,  // 朝风向偏航
        };

        // 获取当前偏航模式
        Mode mode() const { return _mode; }

        // 设置偏航模式为默认模式，参数rtl指示是否为返航模式
        void set_mode_to_default(bool rtl);

        // 设置新的偏航模式
        void set_mode(Mode new_mode);

        // 获取默认偏航模式，参数rtl指示是否为返航模式
        Mode default_mode(bool rtl) const;

        // 设置偏航速率，单位为百分之一度/秒
        void set_rate(float new_rate_cds);

        // 设置“观察”位置，用于指向特定位置
        void set_roi(const Location &roi_location);

        // 设置固定偏航，参数为角度（度）、转向速率（度/秒）、方向和是否为相对角度
        void set_fixed_yaw(float angle_deg,
                           float turn_rate_dps,
                           int8_t direction,
                           bool relative_angle);

        // 设置偏航角度和速率
        void set_yaw_angle_rate(float yaw_angle_d, float yaw_rate_ds);

        // 检查是否达到固定偏航目标
        bool reached_fixed_yaw_target();

    #if WEATHERVANE_ENABLED
        // 更新风向偏航，参数为飞行员偏航速率（百分之一度/秒）
        void update_weathervane(const int16_t pilot_yaw_cds);
    #endif

        // 获取当前的航向命令
        AC_AttitudeControl::HeadingCommand get_heading();

    private:

        // 计算偏航角度（百分之一度）
        float yaw_cd();

        // 计算期望的偏航速率（百分之一度/秒）
        float rate_cds();

        // 返回移动方向的偏航角度（度）
        float look_ahead_yaw();

        // 获取区域兴趣点的偏航角度
        float roi_yaw() const;

        // 自动飞行模式的偏航模式
        Mode _mode = Mode::LOOK_AT_NEXT_WP;      // 当前偏航模式，默认指向下一个航点
        Mode _last_mode;                         // 上一次的偏航模式

        // 如果模式设置为ROI，则偏航将指向此位置
        Vector3f roi;

        // 固定偏航模式下使用的偏航偏移量（百分之一度）
        float _fixed_yaw_offset_cd;

        // 固定偏航模式下的转向速率（度/秒）
        float _fixed_yaw_slewrate_cds;

        // 上一次偏航更新的时间（毫秒）
        uint32_t _last_update_ms;

        // 在look_ahead模式下的航向
        float _look_ahead_yaw;

        // 当auto_yaw_mode设置为AUTO_YAW_RATE时的转向速率（百分之一度）
        float _yaw_angle_cd;
        float _yaw_rate_cds;
        float _pilot_yaw_rate_cds;
    };
    static AutoYaw auto_yaw;

    // 传递函数以减少转换时的代码修改量；
    // 这些函数候选移至Mode基类。
    /**
     * 获取飞行员期望的爬升速率
     * @param throttle_control 油门控制输入
     * @return 期望的爬升速率（厘米/秒）
     */
    float get_pilot_desired_climb_rate(float throttle_control);

    /**
     * 获取非起飞模式下的油门值
     * @return 非起飞油门值
     */
    float get_non_takeoff_throttle(void);

    /**
     * 更新简单模式状态
     */
    void update_simple_mode(void);

    /**
     * 设置飞行模式
     * @param mode 要设置的模式编号
     * @param reason 设置模式的原因
     * @return 成功返回true，失败返回false
     */
    bool set_mode(Mode::Number mode, ModeReason reason);

    /**
     * 设置着陆完成状态
     * @param b 是否完成着陆
     */
    void set_land_complete(bool b);

    /**
     * 获取GCS（地面控制站）对象
     * @return GCS_Copter引用
     */
    GCS_Copter &gcs();

    /**
     * 获取飞行员下行速率
     * @return 飞行员下行速率（单位不明）
     */
    uint16_t get_pilot_speed_dn(void);
    // end pass-through functions
};

#if MODE_ACRO_ENABLED
class ModeAcro : public Mode {

public:
    // 继承父类构造函数
    using Mode::Mode;

    /**
     * 获取模式编号，返回ACRO模式编号
     * @return ACRO模式编号
     */
    Number mode_number() const override { return Number::ACRO; }

    /**
     * 训练器模式枚举类型
     */
    enum class Trainer {
        OFF = 0,       // 关闭训练器
        LEVELING = 1,  // 水平训练模式
        LIMITED = 2,   // 限制性训练模式
    };

    /**
     * Acro模式选项枚举类型
     */
    enum class AcroOptions {
        AIR_MODE = 1 << 0,        // 空中模式选项
        RATE_LOOP_ONLY = 1 << 1,  // 仅速率环选项
    };

    /**
     * 运行模式的主要函数
     */
    virtual void run() override;

    /**
     * 是否需要GPS支持
     * @return 不需要GPS返回false
     */
    bool requires_GPS() const override { return false; }

    /**
     * 是否具有手动油门控制
     * @return 具有手动油门返回true
     */
    bool has_manual_throttle() const override { return true; }

    /**
     * 是否允许解锁，依据解锁方法
     * @param method 解锁方法
     * @return 始终允许解锁返回true
     */
    bool allows_arming(AP_Arming::Method method) const override { return true; };

    /**
     * 是否为自动驾驶模式
     * @return 不是自动驾驶模式返回false
     */
    bool is_autopilot() const override { return false; }

    /**
     * 初始化模式
     * @param ignore_checks 是否忽略检查
     * @return 初始化成功返回true，否则返回false
     */
    bool init(bool ignore_checks) override;

    /**
     * 退出模式
     */
    void exit() override;

    /**
     * 是否切换了空中模式辅助开关
     */
    void air_mode_aux_changed();

    /**
     * 是否允许保存并修剪
     * @return 允许返回true
     */
    bool allows_save_trim() const override { return true; }

    /**
     * 是否允许翻转
     * @return 允许返回true
     */
    bool allows_flip() const override { return true; }

    /**
     * 是否启用碰撞检测
     * @return 禁用碰撞检测返回false
     */
    bool crash_check_enabled() const override { return false; }

protected:

    /**
     * 获取模式名称
     * @return 模式名称字符串 "ACRO"
     */
    const char *name() const override { return "ACRO"; }

    /**
     * 获取模式短名称
     * @return 模式短名称字符串 "ACRO"
     */
    const char *name4() const override { return "ACRO"; }

    /**
     * 获取飞行员期望的角度速率
     * 将飞行员的标准化滚转、俯仰和偏航输入转换为期望的倾斜角度速率
     * 输入范围为-1到1，输出为百分之一度每秒
     * @param roll_in 输入的滚转值
     * @param pitch_in 输入的俯仰值
     * @param yaw_in 输入的偏航值
     * @param roll_out 输出的滚转速率
     * @param pitch_out 输出的俯仰速率
     * @param yaw_out 输出的偏航速率
     */
    void get_pilot_desired_angle_rates(float roll_in, float pitch_in, float yaw_in, float &roll_out, float &pitch_out, float &yaw_out);

    /**
     * 获取悬停油门值
     * @return 悬停油门值
     */
    float throttle_hover() const override;

private:
    bool disable_air_mode_reset;  // 是否禁用空中模式重置
};
#endif

#if FRAME_CONFIG == HELI_FRAME
class ModeAcro_Heli : public ModeAcro {

public:
    // 继承父类构造函数
    using ModeAcro::Mode;

    /**
     * 初始化Heli Acro模式
     * @param ignore_checks 是否忽略检查
     * @return 初始化成功返回true，否则返回false
     */
    bool init(bool ignore_checks) override;

    /**
     * 运行Heli Acro模式
     */
    void run() override;

    /**
     * 虚拟飞杆控制函数
     * @param roll_out 输出的滚转值
     * @param pitch_out 输出的俯仰值
     * @param yaw_out 输出的偏航值
     * @param pitch_leak 俯仰泄漏值
     * @param roll_leak 滚转泄漏值
     */
    void virtual_flybar(float &roll_out, float &pitch_out, float &yaw_out, float pitch_leak, float roll_leak);

protected:
private:
};
#endif


class ModeAltHold : public Mode {

public:
    // 继承父类构造函数
    using Mode::Mode;

    /**
     * 获取模式编号，返回ALT_HOLD模式编号
     * @return ALT_HOLD模式编号
     */
    Number mode_number() const override { return Number::ALT_HOLD; }

    /**
     * 初始化AltHold模式
     * @param ignore_checks 是否忽略检查
     * @return 初始化成功返回true，否则返回false
     */
    bool init(bool ignore_checks) override;

    /**
     * 运行AltHold模式
     */
    void run() override;

    /**
     * 是否需要GPS支持
     * @return 不需要GPS返回false
     */
    bool requires_GPS() const override { return false; }

    /**
     * 是否具有手动油门控制
     * @return 没有手动油门返回false
     */
    bool has_manual_throttle() const override { return false; }

    /**
     * 是否允许解锁，依据解锁方法
     * @param method 解锁方法
     * @return 始终允许解锁返回true
     */
    bool allows_arming(AP_Arming::Method method) const override { return true; };

    /**
     * 是否为自动驾驶模式
     * @return 不是自动驾驶模式返回false
     */
    bool is_autopilot() const override { return false; }

    /**
     * 是否具有用户起飞功能，依据是否必须导航
     * @param must_navigate 是否必须导航
     * @return 如果不必须导航则返回true
     */
    bool has_user_takeoff(bool must_navigate) const override {
        return !must_navigate;
    }

    /**
     * 是否允许自动调节
     * @return 允许返回true
     */
    bool allows_autotune() const override { return true; }

    /**
     * 是否允许翻转
     * @return 允许返回true
     */
    bool allows_flip() const override { return true; }

#if FRAME_CONFIG == HELI_FRAME
    /**
     * 是否允许反向飞行
     * @return 允许返回true
     */
    bool allows_inverted() const override { return true; };
#endif

protected:

    /**
     * 获取模式名称
     * @return 模式名称字符串 "ALT_HOLD"
     */
    const char *name() const override { return "ALT_HOLD"; }

    /**
     * 获取模式短名称
     * @return 模式短名称字符串 "ALTH"
     */
    const char *name4() const override { return "ALTH"; }

private:

};

class ModeAuto : public Mode {

public:
    friend class PayloadPlace;  // 以防wp_run意外需要访问

    // 继承父类构造函数
    using Mode::Mode;

    /**
     * 获取模式编号，根据是否为AUTO_RTL，返回AUTO或AUTO_RTL模式编号
     * @return AUTO或AUTO_RTL模式编号
     */
    Number mode_number() const override { return auto_RTL? Number::AUTO_RTL : Number::AUTO; }

    /**
     * 初始化Auto模式
     * @param ignore_checks 是否忽略检查
     * @return 初始化成功返回true，否则返回false
     */
    bool init(bool ignore_checks) override;

    /**
     * 退出Auto模式
     */
    void exit() override;

    /**
     * 运行Auto模式
     */
    void run() override;

    /**
     * 是否需要GPS支持
     * @return 如果需要GPS返回true，否则返回false
     */
    bool requires_GPS() const override;

    /**
     * 是否具有手动油门控制
     * @return 没有手动油门返回false
     */
    bool has_manual_throttle() const override { return false; }

    /**
     * 是否允许解锁，依据解锁方法
     * @param method 解锁方法
     * @return 根据具体逻辑决定
     */
    bool allows_arming(AP_Arming::Method method) const override;

    /**
     * 是否为自动驾驶模式
     * @return 是自动驾驶模式返回true
     */
    bool is_autopilot() const override { return true; }

    /**
     * 是否处于引导模式
     * @return 如果模式为NAVGUIDED或NAV_SCRIPT_TIME则返回true
     */
    bool in_guided_mode() const override { return _mode == SubMode::NAVGUIDED || _mode == SubMode::NAV_SCRIPT_TIME; }

#if FRAME_CONFIG == HELI_FRAME
    /**
     * 是否允许反向飞行
     * @return 允许返回true
     */
    bool allows_inverted() const override { return true; };
#endif

#if ADVANCED_FAILSAFE
    /**
     * 返回此模式类型，用于高级故障安全
     * @return AFS_AUTO模式
     */
    AP_AdvancedFailsafe_Copter::control_mode afs_mode() const override { return AP_AdvancedFailsafe_Copter::control_mode::AFS_AUTO; }
#endif

    /**
     * 当从GCS或脚本解锁时，是否可以跳过高油门解锁检查
     * @return 可以跳过返回true
     */
    bool allows_GCS_or_SCR_arming_with_throttle_high() const override { return true; }

    // 自动模式子模式枚举类型
    enum class SubMode : uint8_t {
        TAKEOFF,                 // 起飞
        WP,                      // 航点导航
        LAND,                    // 着陆
        RTL,                     // 返航
        CIRCLE_MOVE_TO_EDGE,     // 循环移动到边缘
        CIRCLE,                  // 循环
        NAVGUIDED,               // 引导导航
        LOITER,                  // 徘徊
        LOITER_TO_ALT,           // 徘徊到高度
#if AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED && AC_PAYLOAD_PLACE_ENABLED
        NAV_PAYLOAD_PLACE,      // 载荷放置导航
#endif
        NAV_SCRIPT_TIME,         // 脚本时间导航
        NAV_ATTITUDE_TIME,       // 姿态时间导航
    };

    /**
     * 设置子模式
     * @param new_submode 新的子模式
     */
    void set_submode(SubMode new_submode);

    /**
     * 暂停Auto模式
     * @return 暂停成功返回true，否则返回false
     */
    bool pause() override;

    /**
     * 恢复Auto模式
     * @return 恢复成功返回true，否则返回false
     */
    bool resume() override;

    /**
     * 检查Auto模式是否暂停
     * @return 如果暂停返回true，否则返回false
     */
    bool paused() const;

    /**
     * 启动徘徊模式
     * @return 启动成功返回true，否则返回false
     */
    bool loiter_start();

    /**
     * 启动返航模式
     */
    void rtl_start();

    /**
     * 启动作战命令的起飞
     * @param dest_loc 目标位置
     */
    void takeoff_start(const Location& dest_loc);

    /**
     * 启动作战命令的航点导航
     * @param dest_loc 目标位置
     * @return 启动成功返回true，否则返回false
     */
    bool wp_start(const Location& dest_loc);

    /**
     * 启动作战命令的着陆
     */
    void land_start();

    /**
     * 启动作战命令的循环移动到边缘
     * @param circle_center 循环中心位置
     * @param radius_m 循环半径（米）
     * @param ccw_turn 是否为逆时针转向
     */
    void circle_movetoedge_start(const Location &circle_center, float radius_m, bool ccw_turn);

    /**
     * 启动作战命令的循环
     */
    void circle_start();

    /**
     * 启动作战命令的引导导航
     */
    void nav_guided_start();

    /**
     * 检查当前是否为着陆状态
     * @return 如果着陆返回true，否则返回false
     */
    bool is_landing() const override;

    /**
     * 检查当前是否为起飞状态
     * @return 如果起飞返回true，否则返回false
     */
    bool is_taking_off() const override;

    /**
     * 是否使用飞行员的偏航
     * @return 根据具体逻辑决定
     */
    bool use_pilot_yaw() const override;

    /**
     * 设置水平速度
     * @param speed_xy_cms 水平速度（厘米/秒）
     * @return 设置成功返回true，否则返回false
     */
    bool set_speed_xy(float speed_xy_cms) override;

    /**
     * 设置上升速度
     * @param speed_up_cms 上升速度（厘米/秒）
     * @return 设置成功返回true，否则返回false
     */
    bool set_speed_up(float speed_up_cms) override;

    /**
     * 设置下降速度
     * @param speed_down_cms 下降速度（厘米/秒）
     * @return 设置成功返回true，否则返回false
     */
    bool set_speed_down(float speed_down_cms) override;

    /**
     * 是否需要地形故障安全
     * @return 需要返回true
     */
    bool requires_terrain_failsafe() const override { return true; }

    /**
     * 启动载荷放置
     */
    void payload_place_start();

    /**
     * GCS_MAVLink调用的导引指令
     * @param cmd 任务指令
     * @return 执行成功返回true，否则返回false
     */
    bool do_guided(const AP_Mission::Mission_Command& cmd);

    /**
     * 通过DO_LAND_START直接跳转到着陆序列，如果成功则伪装为Auto RTL模式
     * @param reason 设置模式的原因
     * @return 成功返回true，否则返回false
     */
    bool jump_to_landing_sequence_auto_RTL(ModeReason reason);

    /**
     * 在DO_RETURN_PATH_START航点后加入任务，如果成功则伪装为Auto RTL模式
     * @param reason 设置模式的原因
     * @return 成功返回true，否则返回false
     */
    bool return_path_start_auto_RTL(ModeReason reason);

    /**
     * 尝试加入返航路径，否则启动着陆
     * @param reason 设置模式的原因
     * @return 成功返回true，否则返回false
     */
    bool return_path_or_jump_to_landing_sequence_auto_RTL(ModeReason reason);

    /**
     * Lua访问器，用于导航脚本时间支持
     * @param id 命令ID
     * @param cmd 命令类型
     * @param arg1 参数1
     * @param arg2 参数2
     * @param arg3 参数3
     * @param arg4 参数4
     * @return 请求成功返回true，否则返回false
     */
    bool nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2, int16_t &arg3, int16_t &arg4);

    /**
     * 标记导航脚本时间命令完成
     * @param id 命令ID
     */
    void nav_script_time_done(uint16_t id);

    // 任务对象，绑定开始、验证和退出命令
    AP_Mission mission{
        FUNCTOR_BIND_MEMBER(&ModeAuto::start_command, bool, const AP_Mission::Mission_Command &),
        FUNCTOR_BIND_MEMBER(&ModeAuto::verify_command, bool, const AP_Mission::Mission_Command &),
        FUNCTOR_BIND_MEMBER(&ModeAuto::exit_mission, void)};

    // 任务变更检测器
    AP_Mission_ChangeDetector mis_change_detector;

    // 如果在Auto模式中允许风向偏航
#if WEATHERVANE_ENABLED
    /**
     * 检查是否允许风向偏航
     * @return 如果允许返回true，否则返回false
     */
    bool allows_weathervaning(void) const override;
#endif

protected:

    const char *name() const override { return auto_RTL? "AUTO RTL" : "AUTO"; }
    const char *name4() const override { return auto_RTL? "ARTL" : "AUTO"; }

    uint32_t wp_distance() const override;
    int32_t wp_bearing() const override;
    float crosstrack_error() const override { return wp_nav->crosstrack_error();}

    /**
     * 获取当前航点的位置
     * @param loc 用于存储航点位置的Location对象
     * @return 如果成功获取航点位置返回true，否则返回false
     */
    bool get_wp(Location &loc) const override;

private:

    /**
     * 自动模式的选项枚举
     */
    enum class Option : int32_t {
        AllowArming                        = (1 << 0U),  // 允许解锁
        AllowTakeOffWithoutRaisingThrottle = (1 << 1U),  // 允许不提高油门起飞
        IgnorePilotYaw                     = (1 << 2U),  // 忽略飞行员的偏航输入
        AllowWeatherVaning                 = (1 << 7U),  // 允许风向标功能
    };

    /**
     * 检查指定选项是否启用
     * @param option 要检查的选项
     * @return 如果选项启用返回true，否则返回false
     */
    bool option_is_enabled(Option option) const;

    /**
     * 进入自动返航伪模式
     * @param reason 进入模式的原因
     * @return 如果成功进入模式返回true，否则返回false
     */
    bool enter_auto_rtl(ModeReason reason);

    /**
     * 开始执行任务命令
     * @param cmd 要执行的任务命令
     * @return 如果成功开始执行返回true，否则返回false
     */
    bool start_command(const AP_Mission::Mission_Command& cmd);

    /**
     * 验证任务命令是否完成
     * @param cmd 要验证的任务命令
     * @return 如果命令完成返回true，否则返回false
     */
    bool verify_command(const AP_Mission::Mission_Command& cmd);

    /**
     * 退出任务
     */
    void exit_mission();

    /**
     * 检查任务是否发生变化
     * @return 如果任务发生变化返回true，否则返回false
     */
    bool check_for_mission_change();

    // 各种自动模式子模式的运行函数
    void takeoff_run();
    void wp_run();
    void land_run();
    void rtl_run();
    void circle_run();
    void nav_guided_run();
    void loiter_run();
    void loiter_to_alt_run();
    void nav_attitude_time_run();

    /**
     * 从任务命令获取位置信息
     * @param cmd 任务命令
     * @param default_loc 默认位置
     * @return 根据命令生成的Location对象
     */
    Location loc_from_cmd(const AP_Mission::Mission_Command& cmd, const Location& default_loc) const;

    SubMode _mode = SubMode::TAKEOFF;   // 控制运行哪个自动控制器

    /**
     * 将目标位置的高度调整为当前高度
     * @param target_loc 目标位置
     * @return 如果成功调整返回true，否则返回false
     */
    bool shift_alt_to_current_alt(Location& target_loc) const;

    // 各种任务命令的执行函数
    void do_takeoff(const AP_Mission::Mission_Command& cmd);
    void do_nav_wp(const AP_Mission::Mission_Command& cmd);
    bool set_next_wp(const AP_Mission::Mission_Command& current_cmd, const Location &default_loc);
    void do_land(const AP_Mission::Mission_Command& cmd);
    void do_loiter_unlimited(const AP_Mission::Mission_Command& cmd);
    void do_circle(const AP_Mission::Mission_Command& cmd);
    void do_loiter_time(const AP_Mission::Mission_Command& cmd);
    void do_loiter_to_alt(const AP_Mission::Mission_Command& cmd);
    void do_spline_wp(const AP_Mission::Mission_Command& cmd);
    void get_spline_from_cmd(const AP_Mission::Mission_Command& cmd, const Location& default_loc, Location& dest_loc, Location& next_dest_loc, bool& next_dest_loc_is_spline);
#if AC_NAV_GUIDED
    void do_nav_guided_enable(const AP_Mission::Mission_Command& cmd);
    void do_guided_limits(const AP_Mission::Mission_Command& cmd);
#endif
    void do_nav_delay(const AP_Mission::Mission_Command& cmd);
    void do_wait_delay(const AP_Mission::Mission_Command& cmd);
    void do_within_distance(const AP_Mission::Mission_Command& cmd);
    void do_yaw(const AP_Mission::Mission_Command& cmd);
    void do_change_speed(const AP_Mission::Mission_Command& cmd);
    void do_set_home(const AP_Mission::Mission_Command& cmd);
    void do_roi(const AP_Mission::Mission_Command& cmd);
    void do_mount_control(const AP_Mission::Mission_Command& cmd);
#if HAL_PARACHUTE_ENABLED
    void do_parachute(const AP_Mission::Mission_Command& cmd);
#endif
#if AP_WINCH_ENABLED
    void do_winch(const AP_Mission::Mission_Command& cmd);
#endif
    void do_payload_place(const AP_Mission::Mission_Command& cmd);
    void do_RTL(void);
#if AP_SCRIPTING_ENABLED
    void do_nav_script_time(const AP_Mission::Mission_Command& cmd);
#endif
    void do_nav_attitude_time(const AP_Mission::Mission_Command& cmd);

    // 各种任务命令的验证函数
    bool verify_takeoff();
    bool verify_land();
    bool verify_payload_place();
    bool verify_loiter_unlimited();
    bool verify_loiter_time(const AP_Mission::Mission_Command& cmd);
    bool verify_loiter_to_alt() const;
    bool verify_RTL();
    bool verify_wait_delay();
    bool verify_within_distance();
    bool verify_yaw();
    bool verify_nav_wp(const AP_Mission::Mission_Command& cmd);
    bool verify_circle(const AP_Mission::Mission_Command& cmd);
    bool verify_spline_wp(const AP_Mission::Mission_Command& cmd);
#if AC_NAV_GUIDED
    bool verify_nav_guided_enable(const AP_Mission::Mission_Command& cmd);
#endif
    bool verify_nav_delay(const AP_Mission::Mission_Command& cmd);
#if AP_SCRIPTING_ENABLED
    bool verify_nav_script_time();
#endif
    bool verify_nav_attitude_time(const AP_Mission::Mission_Command& cmd);

    // 盘旋控制相关变量
    uint16_t loiter_time_max;                // 任务脚本中应该在盘旋模式下停留的最长时间（单位：秒）
    uint32_t loiter_time;                    // 已经盘旋的时间 - 开始时间（单位：毫秒）

    // 盘旋到指定高度的状态结构体
    struct {
        bool reached_destination_xy : 1;     // 是否已到达目标水平位置
        bool loiter_start_done : 1;          // 是否已开始盘旋
        bool reached_alt : 1;                // 是否已到达目标高度
        float alt_error_cm;                  // 高度误差（单位：厘米）
        int32_t alt;                         // 目标高度
    } loiter_to_alt;

    // 延迟下一个导航命令
    uint32_t nav_delay_time_max_ms;  // 用于延迟导航命令（如着陆、起飞等）的最大时间
    uint32_t nav_delay_time_start_ms;  // 导航延迟的开始时间

    // 延迟任务脚本命令
    int32_t condition_value;  // 用于条件命令（如延迟、改变高度等）
    uint32_t condition_start;  // 条件命令的开始时间

    // 自动模式中的着陆状态
    enum class State {
        FlyToLocation = 0,  // 飞向目标位置
        Descending = 1      // 正在下降
    };
    State state = State::FlyToLocation;  // 当前状态

    bool waiting_to_start;  // 如果在等待飞机解锁或EKF原点初始化后开始任务，则为true

    // 如果我们进入AUTO模式执行DO_LAND_START着陆序列，并且应该报告为AUTO RTL模式，则为true
    bool auto_RTL;

#if AP_SCRIPTING_ENABLED
    // nav_script_time命令变量
    struct {
        bool done;          // 当Lua脚本指示已完成时为true
        uint16_t id;        // 唯一ID，用于避免命令和Lua脚本之间的竞争条件
        uint32_t start_ms;  // 接收nav_script_time命令的系统时间（用于超时）
        uint8_t command;    // 任务命令提供的命令编号
        uint8_t timeout_s;  // 任务命令提供的超时时间（单位：秒）
        float arg1;         // 任务命令提供的第1个参数
        float arg2;         // 任务命令提供的第2个参数
        int16_t arg3;       // 任务命令提供的第3个参数
        int16_t arg4;       // 任务命令提供的第4个参数
    } nav_scripting;
#endif

    // nav attitude time命令变量
    struct {
        int16_t roll_deg;   // 目标横滚角（单位：度）。由任务命令提供
        int8_t pitch_deg;   // 目标俯仰角（单位：度）。由任务命令提供
        int16_t yaw_deg;    // 目标偏航角（单位：度）。由任务命令提供
        float climb_rate;   // 爬升率（单位：米/秒）。由任务命令提供
        uint32_t start_ms;  // 接收nav attitude time命令的系统时间（用于超时）
    } nav_attitude_time;

    // 期望速度
    struct {
        float xy;     // 期望的水平速度（单位：米/秒）。如果未设置则为0
        float up;     // 期望的向上速度（单位：米/秒）。如果未设置则为0
        float down;   // 期望的向下速度（单位：米/秒）。如果未设置则为0
    } desired_speed_override;
};

#if AUTOTUNE_ENABLED
/*
  AC_AutoTune的包装类
 */

#if FRAME_CONFIG == HELI_FRAME
class AutoTune : public AC_AutoTune_Heli
#else
class AutoTune : public AC_AutoTune_Multi
#endif
{
public:
    /**
     * 初始化自动调参
     * @return 如果初始化成功返回true，否则返回false
     */
    bool init() override;

    /**
     * 运行自动调参
     */
    void run() override;

protected:
    /**
     * 检查位置是否合适
     * @return 如果位置合适返回true，否则返回false
     */
    bool position_ok() override;

    /**
     * 获取飞行员期望的爬升率
     * @return 飞行员期望的爬升率（单位：厘米/秒）
     */
    float get_pilot_desired_climb_rate_cms(void) const override;

    /**
     * 获取飞行员期望的横滚、俯仰和偏航速率
     * @param roll_cd 用于存储期望的横滚角（单位：厘度）
     * @param pitch_cd 用于存储期望的俯仰角（单位：厘度）
     * @param yaw_rate_cds 用于存储期望的偏航速率（单位：厘度/秒）
     */
    void get_pilot_desired_rp_yrate_cd(float &roll_cd, float &pitch_cd, float &yaw_rate_cds) override;

    /**
     * 初始化高度限制
     */
    void init_z_limits() override;

#if HAL_LOGGING_ENABLED
    /**
     * 记录PID参数
     */
    void log_pids() override;
#endif
};

class ModeAutoTune : public Mode {

    // ParametersG2在我们的autotune对象中设置一个指针：
    friend class ParametersG2;

public:
    // 继承构造函数
    using Mode::Mode;

    /**
     * 获取模式编号
     * @return 返回AUTOTUNE模式的编号
     */
    Number mode_number() const override { return Number::AUTOTUNE; }

    /**
     * 初始化自动调参模式
     * @param ignore_checks 是否忽略检查
     * @return 如果初始化成功返回true，否则返回false
     */
    bool init(bool ignore_checks) override;

    /**
     * 退出自动调参模式
     */
    void exit() override;

    /**
     * 运行自动调参模式
     */
    void run() override;

    /**
     * 检查是否需要GPS
     * @return 返回false，表示不需要GPS
     */
    bool requires_GPS() const override { return false; }

    /**
     * 检查是否有手动油门控制
     * @return 返回false，表示没有手动油门控制
     */
    bool has_manual_throttle() const override { return false; }

    /**
     * 检查是否允许解锁
     * @param method 解锁方法
     * @return 返回false，表示不允许解锁
     */
    bool allows_arming(AP_Arming::Method method) const override { return false; }

    /**
     * 检查是否为自动驾驶模式
     * @return 返回false，表示不是自动驾驶模式
     */
    bool is_autopilot() const override { return false; }

    AutoTune autotune;  // 自动调参对象

protected:

    /**
     * 获取模式名称
     * @return 返回"AUTOTUNE"
     */
    const char *name() const override { return "AUTOTUNE"; }

    /**
     * 获取模式的4字符缩写名称
     * @return 返回"ATUN"
     */
    const char *name4() const override { return "ATUN"; }
};
#endif

class ModeBrake : public Mode {

public:
    // 继承构造函数
    using Mode::Mode;
    
    // 返回模式编号
    Number mode_number() const override { return Number::BRAKE; }

    // 初始化函数
    bool init(bool ignore_checks) override;
    
    // 运行函数
    void run() override;

    // 是否需要GPS
    bool requires_GPS() const override { return true; }
    
    // 是否有手动油门控制
    bool has_manual_throttle() const override { return false; }
    
    // 是否允许解锁
    bool allows_arming(AP_Arming::Method method) const override { return false; };
    
    // 是否为自动驾驶模式
    bool is_autopilot() const override { return false; }

    // 设置超时时间后切换到悬停模式
    void timeout_to_loiter_ms(uint32_t timeout_ms);

protected:

    // 获取模式名称
    const char *name() const override { return "BRAKE"; }
    
    // 获取模式的4字符缩写名称
    const char *name4() const override { return "BRAK"; }

private:

    uint32_t _timeout_start;  // 超时计时开始时间
    uint32_t _timeout_ms;     // 超时时间(毫秒)

};


class ModeCircle : public Mode {

public:
    // 继承构造函数
    using Mode::Mode;
    
    // 返回模式编号
    Number mode_number() const override { return Number::CIRCLE; }

    // 初始化函数
    bool init(bool ignore_checks) override;
    
    // 运行函数
    void run() override;

    // 是否需要GPS
    bool requires_GPS() const override { return true; }
    
    // 是否有手动油门控制
    bool has_manual_throttle() const override { return false; }
    
    // 是否允许解锁
    bool allows_arming(AP_Arming::Method method) const override { return false; };
    
    // 是否为自动驾驶模式
    bool is_autopilot() const override { return true; }

protected:

    // 获取模式名称
    const char *name() const override { return "CIRCLE"; }
    
    // 获取模式的4字符缩写名称
    const char *name4() const override { return "CIRC"; }

    // 获取到下一个航点的距离
    uint32_t wp_distance() const override;
    
    // 获取到下一个航点的方位角
    int32_t wp_bearing() const override;

private:

    // 圆形模式
    bool speed_changing = false;     // 当滚转摇杆被按住以便于停止在0速率时为true

};


class ModeDrift : public Mode {

public:
    // 继承构造函数
    using Mode::Mode;
    
    // 返回模式编号
    Number mode_number() const override { return Number::DRIFT; }

    // 初始化函数
    bool init(bool ignore_checks) override;
    
    // 运行函数
    void run() override;

    // 是否需要GPS
    bool requires_GPS() const override { return true; }
    
    // 是否有手动油门控制
    bool has_manual_throttle() const override { return false; }
    
    // 是否允许解锁
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    
    // 是否为自动驾驶模式
    bool is_autopilot() const override { return false; }

protected:

    // 获取模式名称
    const char *name() const override { return "DRIFT"; }
    
    // 获取模式的4字符缩写名称
    const char *name4() const override { return "DRIF"; }

private:

    // 获取油门辅助值
    float get_throttle_assist(float velz, float pilot_throttle_scaled);

};


class ModeFlip : public Mode {

public:
    // 继承构造函数
    using Mode::Mode;
    
    // 返回模式编号
    Number mode_number() const override { return Number::FLIP; }

    // 初始化函数
    bool init(bool ignore_checks) override;
    
    // 运行函数
    void run() override;

    // 是否需要GPS
    bool requires_GPS() const override { return false; }
    
    // 是否有手动油门控制
    bool has_manual_throttle() const override { return false; }
    
    // 是否允许解锁
    bool allows_arming(AP_Arming::Method method) const override { return false; };
    
    // 是否为自动驾驶模式
    bool is_autopilot() const override { return false; }
    
    // 是否启用碰撞检查
    bool crash_check_enabled() const override { return false; }

protected:

    // 获取模式名称
    const char *name() const override { return "FLIP"; }
    
    // 获取模式的4字符缩写名称
    const char *name4() const override { return "FLIP"; }

private:

    // 翻转
    Vector3f orig_attitude;         // 翻转前的原始飞行器姿态

    // 翻转状态枚举
    enum class FlipState : uint8_t {
        Start,      // 开始
        Roll,       // 滚转
        Pitch_A,    // 俯仰A
        Pitch_B,    // 俯仰B
        Recover,    // 恢复
        Abandon     // 放弃
    };
    FlipState _state;               // 当前翻转状态
    Mode::Number   orig_control_mode;   // 开始翻转时的飞行模式
    uint32_t  start_time_ms;          // 翻转开始的时间
    int8_t    roll_dir;            // 滚转方向 (-1 = 向左滚转, 1 = 向右滚转)
    int8_t    pitch_dir;           // 俯仰方向 (-1 = 向前俯仰, 1 = 向后俯仰)
};


#if MODE_FLOWHOLD_ENABLED
/*
  支持FLOWHOLD模式的类，这是一个使用光流直接进行位置保持的模式，
  避免了对测距仪的需求
 */

class ModeFlowHold : public Mode {
public:
    // 需要一个构造函数来处理参数
    ModeFlowHold(void);
    
    // 返回模式编号
    Number mode_number() const override { return Number::FLOWHOLD; }

    // 初始化函数
    bool init(bool ignore_checks) override;
    
    // 运行函数
    void run(void) override;

    // 是否需要GPS
    bool requires_GPS() const override { return false; }
    
    // 是否有手动油门控制
    bool has_manual_throttle() const override { return false; }
    
    // 是否允许解锁
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    
    // 是否为自动驾驶模式
    bool is_autopilot() const override { return false; }
    
    // 是否有用户起飞
    bool has_user_takeoff(bool must_navigate) const override {
        return !must_navigate;
    }
    
    // 是否允许翻转
    bool allows_flip() const override { return true; }

    // 参数组信息
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // 获取模式名称
    const char *name() const override { return "FLOWHOLD"; }
    
    // 获取模式的4字符缩写名称
    const char *name4() const override { return "FHLD"; }

private:

    // FlowHold状态枚举
    enum FlowHoldModeState {
        FlowHold_MotorStopped,  // 电机停止
        FlowHold_Takeoff,       // 起飞
        FlowHold_Flying,        // 飞行中
        FlowHold_Landed         // 已着陆
    };

    // 根据光流数据计算姿态
    void flow_to_angle(Vector2f &bf_angle);

    LowPassFilterConstDtVector2f flow_filter;  // 光流滤波器

    // FlowHold模式初始化
    bool flowhold_init(bool ignore_checks);
    
    // FlowHold模式运行
    void flowhold_run();
    
    // 将光流转换为角度
    void flowhold_flow_to_angle(Vector2f &angle, bool stick_input);
    
    // 更新高度估计
    void update_height_estimate(void);

    // 最小假设高度
    const float height_min = 0.1f;

    // 最大缩放高度
    const float height_max = 3.0f;

    AP_Float flow_max;  // 最大光流
    AC_PI_2D flow_pi_xy{0.2f, 0.3f, 3000, 5, 0.0025f};  // XY平面的PI控制器
    AP_Float flow_filter_hz;  // 光流滤波器频率
    AP_Int8  flow_min_quality;  // 最小光流质量
    AP_Int8  brake_rate_dps;  // 刹车速率(度/秒)

    float quality_filtered;  // 滤波后的质量

    uint8_t log_counter;  // 日志计数器
    bool limited;  // 是否受限
    Vector2f xy_I;  // XY积分项

    // 自上次光流更新以来累积的INS北东方向速度增量
    Vector2f delta_velocity_ne;

    // 上次北东轴的光流速率(弧度/秒)
    Vector2f last_flow_rate_rps;

    // 上次光流数据的时间戳
    uint32_t last_flow_ms;

    float last_ins_height;  // 上次INS高度
    float height_offset;  // 高度偏移

    // 是否在飞行员输入后进行刹车
    bool braking;

    // 上次有显著摇杆输入的时间
    uint32_t last_stick_input_ms;
};
#endif // MODE_FLOWHOLD_ENABLED


class ModeGuided : public Mode {

public:
#if AP_EXTERNAL_CONTROL_ENABLED
    friend class AP_ExternalControl_Copter;
#endif

    // 继承构造函数
    using Mode::Mode;
    
    // 返回模式编号
    Number mode_number() const override { return Number::GUIDED; }

    // 初始化函数
    bool init(bool ignore_checks) override;
    
    // 运行函数
    void run() override;

    // 是否需要GPS
    bool requires_GPS() const override { return true; }
    
    // 是否有手动油门控制
    bool has_manual_throttle() const override { return false; }
    
    // 是否允许解锁
    bool allows_arming(AP_Arming::Method method) const override;
    
    // 是否为自动驾驶模式
    bool is_autopilot() const override { return true; }
    
    // 是否有用户起飞
    bool has_user_takeoff(bool must_navigate) const override { return true; }
    
    // 是否处于引导模式
    bool in_guided_mode() const override { return true; }

    // 是否需要地形故障保护
    bool requires_terrain_failsafe() const override { return true; }

#if ADVANCED_FAILSAFE
    // 返回此模式用于高级故障保护的类型
    AP_AdvancedFailsafe_Copter::control_mode afs_mode() const override { return AP_AdvancedFailsafe_Copter::control_mode::AFS_AUTO; }
#endif

    // 当从GCS或脚本解锁时，如果油门高，返回true表示可以跳过油门高解锁检查
    bool allows_GCS_or_SCR_arming_with_throttle_high() const override { return true; }

    // 设置引导模式的角度目标子模式：使用旋转四元数、角速度和爬升率或推力（取决于用户选项）
    // attitude_quat: 如果为零：必须提供ang_vel（角速度），即使全为零
    //                如果非零：使用姿态四元数和角速度进行姿态控制
    // ang_vel: 角速度（弧度/秒）
    // climb_rate_cms_or_thrust: 表示爬升率（厘米/秒）或缩放到[0, 1]的推力，无单位
    // use_thrust: 如果为true：climb_rate_cms_or_thrust表示推力
    //             如果为false：climb_rate_cms_or_thrust表示爬升率（厘米/秒）
    void set_angle(const Quaternion &attitude_quat, const Vector3f &ang_vel, float climb_rate_cms_or_thrust, bool use_thrust);

    // 设置目标位置
    bool set_destination(const Vector3f& destination, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false, bool terrain_alt = false);
    bool set_destination(const Location& dest_loc, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false);
    
    // 获取当前航点
    bool get_wp(Location &loc) const override;
    
    // 设置加速度
    void set_accel(const Vector3f& acceleration, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false, bool log_request = true);
    
    // 设置速度
    void set_velocity(const Vector3f& velocity, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false, bool log_request = true);
    
    // 设置速度和加速度
    void set_velaccel(const Vector3f& velocity, const Vector3f& acceleration, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false, bool log_request = true);
    
    // 设置目标位置和速度
    bool set_destination_posvel(const Vector3f& destination, const Vector3f& velocity, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false);
    
    // 设置目标位置、速度和加速度
    bool set_destination_posvelaccel(const Vector3f& destination, const Vector3f& velocity, const Vector3f& acceleration, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false);

    // 获取位置、速度和加速度目标
    const Vector3p& get_target_pos() const;
    const Vector3f& get_target_vel() const;
    const Vector3f& get_target_accel() const;

    // 返回GUIDED_OPTIONS参数是否建议将SET_ATTITUDE_TARGET的"thrust"字段解释为推力而不是爬升率
    bool set_attitude_target_provides_thrust() const;
    
    // 是否正在稳定XY位置
    bool stabilizing_pos_xy() const;
    
    // 是否正在稳定XY速度
    bool stabilizing_vel_xy() const;
    
    // 是否使用wpnav进行位置控制
    bool use_wpnav_for_position_control() const;

    // 清除限制
    void limit_clear();
    
    // 初始化限制时间和位置
    void limit_init_time_and_pos();
    
    // 设置限制
    void limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm);
    
    // 检查限制
    bool limit_check();

    // 是否正在起飞
    bool is_taking_off() const override;
    
    // 设置XY速度
    bool set_speed_xy(float speed_xy_cms) override;
    
    // 设置上升速度
    bool set_speed_up(float speed_up_cms) override;
    
    // 设置下降速度
    bool set_speed_down(float speed_down_cms) override;

    // 初始化位置控制器以实现起飞
    // takeoff_alt_cm被解释为相对于家的高度（以厘米为单位）或如果有测距仪可用，则为相对于地形的高度
    bool do_user_takeoff_start(float takeoff_alt_cm) override;

    // 子模式枚举
    enum class SubMode {
        TakeOff,        // 起飞
        WP,             // 航点
        Pos,            // 位置
        PosVelAccel,    // 位置速度加速度
        VelAccel,       // 速度加速度
        Accel,          // 加速度
        Angle,          // 角度
    };

    // 获取当前子模式
    SubMode submode() const { return guided_mode; }

    // 开始角度控制
    void angle_control_start();
    
    // 运行角度控制
    void angle_control_run();

    // 返回引导模式超时时间（毫秒）。仅用于速度、加速度、角度控制和角速度控制
    uint32_t get_timeout_ms() const;

    // 是否使用飞行员偏航
    bool use_pilot_yaw() const override;

    // 引导模式中的暂停和继续
    bool pause() override;
    bool resume() override;

    // 如果在引导模式下允许风向标功能，则返回true
#if WEATHERVANE_ENABLED
    bool allows_weathervaning(void) const override;
#endif

protected:

    // 获取模式名称
    const char *name() const override { return "GUIDED"; }
    
    // 获取模式的4字符缩写名称
    const char *name4() const override { return "GUID"; }

    // 获取到下一个航点的距离
    uint32_t wp_distance() const override;
    
    // 获取到下一个航点的方位角
    int32_t wp_bearing() const override;
    
    // 获取横向跟踪误差
    float crosstrack_error() const override;

private:

    // GUID_OPTIONS参数的枚举
    enum class Option : uint32_t {
        AllowArmingFromTX   = (1U << 0),  // 允许从发射机解锁
        // 这个位仍然可用，飞行员偏航被映射到第2位以与自动模式对称
        IgnorePilotYaw      = (1U << 2),  // 忽略飞行员偏航
        SetAttitudeTarget_ThrustAsThrust = (1U << 3),  // 将SET_ATTITUDE_TARGET的推力解释为推力
        DoNotStabilizePositionXY = (1U << 4),  // 不稳定XY位置
        DoNotStabilizeVelocityXY = (1U << 5),  // 不稳定XY速度
        WPNavUsedForPosControl = (1U << 6),  // 使用wpnav进行位置控制
        AllowWeatherVaning = (1U << 7)  // 允许风向标功能
    };

    // 返回Guided模式选项是否设置（参见GUID_OPTIONS）
    bool option_is_enabled(Option option) const;

    // 航点控制器
    void wp_control_start();
    void wp_control_run();

    void pva_control_start();
    void pos_control_start();
    void accel_control_start();
    void velaccel_control_start();
    void posvelaccel_control_start();
    void takeoff_run();
    void pos_control_run();
    void accel_control_run();
    void velaccel_control_run();
    void pause_control_run();
    void posvelaccel_control_run();
    void set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle);

    // 控制运行哪个控制器（位置或速度）：
    SubMode guided_mode = SubMode::TakeOff;
    bool send_notification;     // 用于向地面站发送一次性通知
    bool takeoff_complete;      // 一旦起飞完成为true（用于触发收回起落架）

    // 引导模式是否暂停
    bool _paused;
};


class ModeGuidedNoGPS : public ModeGuided {

public:
    // 继承构造函数
    using ModeGuided::Mode;
    
    // 返回模式编号
    Number mode_number() const override { return Number::GUIDED_NOGPS; }

    // 初始化函数
    bool init(bool ignore_checks) override;
    
    // 运行函数
    void run() override;

    // 是否需要GPS
    bool requires_GPS() const override { return false; }
    
    // 是否有手动油门控制
    bool has_manual_throttle() const override { return false; }
    
    // 是否为自动驾驶模式
    bool is_autopilot() const override { return true; }

protected:

    // 获取模式名称
    const char *name() const override { return "GUIDED_NOGPS"; }
    
    // 获取模式的4字符缩写名称
    const char *name4() const override { return "GNGP"; }

private:

};


class ModeLand : public Mode {

public:
    // 继承构造函数
    using Mode::Mode;
    
    // 返回模式编号
    Number mode_number() const override { return Number::LAND; }

    // 初始化函数
    bool init(bool ignore_checks) override;
    
    // 运行函数
    void run() override;

    // 是否需要GPS
    bool requires_GPS() const override { return false; }
    
    // 是否有手动油门控制
    bool has_manual_throttle() const override { return false; }
    
    // 是否允许解锁
    bool allows_arming(AP_Arming::Method method) const override { return false; };
    
    // 是否为自动驾驶模式
    bool is_autopilot() const override { return true; }

    // 是否正在着陆
    bool is_landing() const override { return true; };
 // Start of Selection

#if ADVANCED_FAILSAFE
    // 返回此模式的类型，用于高级失效安全处理
    AP_AdvancedFailsafe_Copter::control_mode afs_mode() const override { return AP_AdvancedFailsafe_Copter::control_mode::AFS_AUTO; }
#endif

    /**
     * 禁用使用GPS的功能
     */
    void do_not_use_GPS();

    /**
     * 如果LAND模式试图控制X/Y位置，则返回true
     *
     * @return 是否正在控制位置
     */
    bool controlling_position() const { return control_position; }

    /**
     * 设置着陆暂停状态
     *
     * @param new_value 新的暂停状态值
     */
    void set_land_pause(bool new_value) { land_pause = new_value; }

protected:

    /**
     * 获取模式的名称
     *
     * @return 模式名称字符串
     */
    const char *name() const override { return "LAND"; }

    /**
     * 获取模式的四字符名称
     *
     * @return 四字符模式名称字符串
     */
    const char *name4() const override { return "LAND"; }

private:

    /**
     * 使用GPS执行着陆逻辑
     */
    void gps_run();

    /**
     * 不使用GPS执行着陆逻辑
     */
    void nogps_run();

    bool control_position; // 如果使用外部参考来控制位置，则为true

    uint32_t land_start_time; // 着陆开始的时间戳
    bool land_pause; // 着陆是否处于暂停状态
};

class ModeLoiter : public Mode {

public:
    // 继承基类构造函数
    using Mode::Mode;

    /**
     * 获取模式的编号
     *
     * @return LOITER模式的编号
     */
    Number mode_number() const override { return Number::LOITER; }

    /**
     * 初始化LOITER模式
     *
     * @param ignore_checks 是否忽略初始化检查
     * @return 初始化是否成功
     */
    bool init(bool ignore_checks) override;

    /**
     * 运行LOITER模式的主要逻辑
     */
    void run() override;

    /**
     * 判断是否需要GPS
     *
     * @return 如果需要GPS则返回true
     */
    bool requires_GPS() const override { return true; }

    /**
     * 判断是否有手动油门控制
     *
     * @return 如果没有手动油门控制则返回false
     */
    bool has_manual_throttle() const override { return false; }

    /**
     * 判断是否允许装甲（arming）
     *
     * @param method 装甲方法
     * @return 如果允许装甲则返回true
     */
    bool allows_arming(AP_Arming::Method method) const override { return true; };

    /**
     * 判断是否为自动驾驶模式
     *
     * @return 如果是自动驾驶模式则返回false
     */
    bool is_autopilot() const override { return false; }

    /**
     * 判断是否允许用户起飞
     *
     * @param must_navigate 是否必须导航
     * @return 如果允许用户起飞则返回true
     */
    bool has_user_takeoff(bool must_navigate) const override { return true; }

    /**
     * 判断是否允许自动调谐
     *
     * @return 如果允许自动调谐则返回true
     */
    bool allows_autotune() const override { return true; }

#if FRAME_CONFIG == HELI_FRAME
    /**
     * 判断是否允许倒转
     *
     * @return 如果允许倒转则返回true
     */
    bool allows_inverted() const override { return true; };
#endif

#if AC_PRECLAND_ENABLED
    /**
     * 设置精密徘徊模式的启用状态
     *
     * @param value 新的启用状态
     */
    void set_precision_loiter_enabled(bool value) { _precision_loiter_enabled = value; }
#endif

protected:

    /**
     * 获取模式的名称
     *
     * @return LOITER模式的名称
     */
    const char *name() const override { return "LOITER"; }

    /**
     * 获取模式的四字符名称
     *
     * @return LOIT
     */
    const char *name4() const override { return "LOIT"; }

    /**
     * 获取当前航点与无人机之间的距离
     *
     * @return 航点距离
     */
    uint32_t wp_distance() const override;

    /**
     * 获取当前航点的方位角
     *
     * @return 航点方位角
     */
    int32_t wp_bearing() const override;

    /**
     * 获取横穿误差
     *
     * @return 横穿误差
     */
    float crosstrack_error() const override { return pos_control->crosstrack_error();}

#if AC_PRECLAND_ENABLED
    /**
     * 执行精密徘徊逻辑
     *
     * @return 是否成功执行
     */
    bool do_precision_loiter();

    /**
     * 执行XY方向的精密徘徊逻辑
     */
    void precision_loiter_xy();
#endif

private:

#if AC_PRECLAND_ENABLED
    bool _precision_loiter_enabled; // 是否启用精密徘徊
    bool _precision_loiter_active;  // 如果用户已开启精密徘徊则为true
#endif

};

class ModePosHold : public Mode {

public:
    // 继承基类构造函数
    using Mode::Mode;

    /**
     * 获取模式的编号
     *
     * @return POSHOLD模式的编号
     */
    Number mode_number() const override { return Number::POSHOLD; }

    /**
     * 初始化POSHOLD模式
     *
     * @param ignore_checks 是否忽略初始化检查
     * @return 初始化是否成功
     */
    bool init(bool ignore_checks) override;

    /**
     * 运行POSHOLD模式的主要逻辑
     */
    void run() override;

    /**
     * 判断是否需要GPS
     *
     * @return 如果需要GPS则返回true
     */
    bool requires_GPS() const override { return true; }

    /**
     * 判断是否有手动油门控制
     *
     * @return 如果没有手动油门控制则返回false
     */
    bool has_manual_throttle() const override { return false; }

    /**
     * 判断是否允许装甲（arming）
     *
     * @param method 装甲方法
     * @return 如果允许装甲则返回true
     */
    bool allows_arming(AP_Arming::Method method) const override { return true; };

    /**
     * 判断是否为自动驾驶模式
     *
     * @return 如果是自动驾驶模式则返回false
     */
    bool is_autopilot() const override { return false; }

    /**
     * 判断是否允许用户起飞
     *
     * @param must_navigate 是否必须导航
     * @return 如果允许用户起飞则返回true
     */
    bool has_user_takeoff(bool must_navigate) const override { return true; }

    /**
     * 判断是否允许自动调谐
     *
     * @return 如果允许自动调谐则返回true
     */
    bool allows_autotune() const override { return true; }

protected:

    /**
     * 获取模式的名称
     *
     * @return POSHOLD模式的名称
     */
    const char *name() const override { return "POSHOLD"; }

    /**
     * 获取模式的四字符名称
     *
     * @return PHLD
     */
    const char *name4() const override { return "PHLD"; }

private:

    /**
     * 更新飞行员倾斜角度（过滤后和原始值）
     *
     * @param lean_angle_filtered 过滤后的倾斜角度
     * @param lean_angle_raw 原始倾斜角度
     */
    void update_pilot_lean_angle(float &lean_angle_filtered, float &lean_angle_raw);

    /**
     * 混合控制输入
     *
     * @param mix_ratio 混合比率
     * @param first_control 第一个控制输入
     * @param second_control 第二个控制输入
     * @return 混合后的控制输出
     */
    float mix_controls(float mix_ratio, float first_control, float second_control);

    /**
     * 根据速度更新制动角度
     *
     * @param brake_angle 制动角度
     * @param velocity 速度
     */
    void update_brake_angle_from_velocity(float &brake_angle, float velocity);

    /**
     * 初始化风补偿估计
     */
    void init_wind_comp_estimate();

    /**
     * 更新风补偿估计
     */
    void update_wind_comp_estimate();

    /**
     * 获取风补偿的倾斜角度
     *
     * @param roll_angle 补偿后的滚转角度
     * @param pitch_angle 补偿后的俯仰角度
     */
    void get_wind_comp_lean_angles(float &roll_angle, float &pitch_angle);

    /**
     * 将控制器的滚转控制切换到飞行员控制
     */
    void roll_controller_to_pilot_override();

    /**
     * 将控制器的俯仰控制切换到飞行员控制
     */
    void pitch_controller_to_pilot_override();

    /**
     * 定义滚转和俯仰模式的枚举类型
     */
    enum class RPMode {
        PILOT_OVERRIDE=0,            // 飞行员控制此轴（即滚转或俯仰）
        BRAKE,                       // 此轴正在减速至零
        BRAKE_READY_TO_LOITER,       // 此轴已完成减速，准备进入徘徊模式（两个轴都必须达到此状态才能进入下一阶段）
        BRAKE_TO_LOITER,             // 车辆的两个轴（滚转和俯仰）正在从减速模式过渡到徘徊模式（减速和徘徊控制被混合）
        LOITER,                      // 车辆的两个轴都在保持位置
        CONTROLLER_TO_PILOT_OVERRIDE // 飞行员在此轴有输入，并且此轴正在过渡到飞行员控制（如果另一个轴没有飞行员输入，则将过渡到制动）
    };

    RPMode roll_mode; // 滚转轴的当前模式
    RPMode pitch_mode; // 俯仰轴的当前模式

    // 飞行员输入相关变量
    float pilot_roll;  // 飞行员请求的滚转角度（经过过滤以缓慢返回零）
    float pilot_pitch; // 飞行员请求的俯仰角度（经过过滤以缓慢返回零）

    // 制动相关变量
    struct {
        uint8_t time_updated_roll   : 1;    // 一旦重新估算了制动时间，则为true。当车辆开始减速后平稳下来时执行一次
        uint8_t time_updated_pitch  : 1;    // 一旦重新估算了制动时间，则为true。当车辆开始减速后平稳下来时执行一次

        float gain;                         // 制动期间将车辆速度转换为倾斜角度时使用的增益（根据速率计算）
        float roll;                         // 制动期间的目标滚转角度
        float pitch;                        // 制动期间的目标俯仰角度
        int16_t timeout_roll;               // 允许制动完成的周期数，此超时将在半制动时更新
        int16_t timeout_pitch;              // 允许制动完成的周期数，此超时将在半制动时更新
        float angle_max_roll;               // 制动期间达到的最大滚转倾斜角度。用于确定车辆何时开始平稳下来，以便重新估算制动时间
        float angle_max_pitch;              // 制动期间达到的最大俯仰倾斜角度。用于确定车辆何时开始平稳下来，以便重新估算制动时间
        int16_t to_loiter_timer;            // 在POSHOLD_TO_LOITER中混合制动和徘徊控制的周期数
    } brake;

    // 徘徊相关变量
    int16_t controller_to_pilot_timer_roll;     // 在POSHOLD_CONTROLLER_TO_PILOT中混合控制器和飞行员控制的滚转周期数
    int16_t controller_to_pilot_timer_pitch;    // 在POSHOLD_CONTROLLER_TO_PILOT中混合控制器和飞行员控制的俯仰周期数
    float controller_final_roll;                // 从控制器获取的最终滚转角度，用于退出制动或徘徊模式时与飞行员输入混合
    float controller_final_pitch;               // 从控制器获取的最终俯仰角度，用于退出制动或徘徊模式时与飞行员输入混合

    // 风补偿相关变量
    Vector2f wind_comp_ef;                      // 地球坐标系下的风补偿，来自位置控制器的过滤倾斜角度
    float wind_comp_roll;                       // 用于补偿风的滚转角度
    float wind_comp_pitch;                      // 用于补偿风的俯仰角度
    uint16_t wind_comp_start_timer;             // 启动风补偿的延迟计数器，在徘徊开始后短时间内延迟
    int8_t  wind_comp_timer;                    // 风补偿滚转/俯仰倾斜角度计算的减速计数器，降低到10Hz

    // 最终输出
    float roll;   // 发送到姿态控制器的最终滚转角度
    float pitch;  // 发送到姿态控制器的最终俯仰角度

};

class ModeRTL : public Mode {

public:
    // 继承基类构造函数
    using Mode::Mode;

    /**
     * 获取模式的编号
     *
     * @return RTL模式的编号
     */
    Number mode_number() const override { return Number::RTL; }

    /**
     * 初始化RTL模式
     *
     * @param ignore_checks 是否忽略初始化检查
     * @return 初始化是否成功
     */
    bool init(bool ignore_checks) override;

    /**
     * 运行RTL模式的主要逻辑
     */
    void run() override {
        return run(true);
    }

    /**
     * 运行RTL模式，并决定是否在着陆后解锁电机
     *
     * @param disarm_on_land 是否在着陆后解锁电机
     */
    void run(bool disarm_on_land);

    /**
     * 判断是否需要GPS
     *
     * @return 如果需要GPS则返回true
     */
    bool requires_GPS() const override { return true; }

    /**
     * 判断是否有手动油门控制
     *
     * @return 如果没有手动油门控制则返回false
     */
    bool has_manual_throttle() const override { return false; }

    /**
     * 判断是否允许装甲（arming）
     *
     * @param method 装甲方法
     * @return 如果不允许装甲则返回false
     */
    bool allows_arming(AP_Arming::Method method) const override { return false; };

    /**
     * 判断是否为自动驾驶模式
     *
     * @return 如果是自动驾驶模式则返回true
     */
    bool is_autopilot() const override { return true; }

    /**
     * 判断是否需要地形失效安全
     *
     * @return 如果需要地形失效安全则返回true
     */
    bool requires_terrain_failsafe() const override { return true; }

#if ADVANCED_FAILSAFE
    // 返回此模式的类型，用于高级失效安全处理
    AP_AdvancedFailsafe_Copter::control_mode afs_mode() const override { return AP_AdvancedFailsafe_Copter::control_mode::AFS_AUTO; }
#endif

    /**
     * 为地面控制站报告当前位置的航点
     *
     * @param loc 航点位置
     * @return 如果成功获取航点则返回true
     */
    bool get_wp(Location &loc) const override;

    /**
     * 判断是否使用飞行员的偏航控制
     *
     * @return 如果使用则返回true
     */
    bool use_pilot_yaw() const override;

    /**
     * 设置水平方向的速度
     *
     * @param speed_xy_cms 水平方向速度（厘米/秒）
     * @return 设置是否成功
     */
    bool set_speed_xy(float speed_xy_cms) override;

    /**
     * 设置上升速度
     *
     * @param speed_up_cms 上升速度（厘米/秒）
     * @return 设置是否成功
     */
    bool set_speed_up(float speed_up_cms) override;

    /**
     * 设置下降速度
     *
     * @param speed_down_cms 下降速度（厘米/秒）
     * @return 设置是否成功
     */
    bool set_speed_down(float speed_down_cms) override;

    /**
     * 定义RTL模式的子模式枚举
     */
    enum class SubMode : uint8_t {
        STARTING,          // 启动阶段
        INITIAL_CLIMB,     // 初始爬升阶段
        RETURN_HOME,       // 返回家园阶段
        LOITER_AT_HOME,    // 家园徘徊阶段
        FINAL_DESCENT,     // 最终下降阶段
        LAND               // 着陆阶段
    };

    /**
     * 获取当前的子模式
     *
     * @return 当前的RTL子模式
     */
    SubMode state() { return _state; }

    /**
     * 判断当前状态是否完成
     *
     * @return 如果当前状态完成则返回true
     */
    bool state_complete() const { return _state_complete; }

    /**
     * 判断是否正在着陆
     *
     * @return 如果正在着陆则返回true
     */
    virtual bool is_landing() const override;

    /**
     * 重新启动RTL模式而不依赖地形
     */
    void restart_without_terrain();

    /**
     * 获取RTL高度类型
     *
     * @return RTL高度类型
     */
    ModeRTL::RTLAltType get_alt_type() const;

    /**
     * 定义RTL高度类型的枚举
     */
    enum class RTLAltType : int8_t {
        RELATIVE = 0, // 相对高度
        TERRAIN = 1   // 基于地形的高度
    };

protected:

    /**
     * 获取模式的名称
     *
     * @return RTL模式的名称
     */
    const char *name() const override { return "RTL"; }

    /**
     * 获取模式的四字符名称
     *
     * @return RTL 
     */
    const char *name4() const override { return "RTL "; }

    /**
     * 获取当前航点与无人机之间的距离
     *
     * @return 航点距离
     */
    uint32_t wp_distance() const override;

    /**
     * 获取当前航点的方位角
     *
     * @return 航点方位角
     */
    int32_t wp_bearing() const override;

    /**
     * 获取横穿误差
     *
     * @return 横穿误差
     */
    float crosstrack_error() const override { return wp_nav->crosstrack_error();}

    /**
     * 开始下降阶段
     */
    void descent_start();

    /**
     * 运行下降逻辑
     */
    void descent_run();

    /**
     * 开始着陆阶段
     */
    void land_start();

    /**
     * 运行着陆逻辑，并决定是否在着陆后解锁电机
     *
     * @param disarm_on_land 是否解锁电机
     */
    void land_run(bool disarm_on_land);

    /**
     * 设置下降目标高度
     *
     * @param alt 目标高度
     */
    void set_descent_target_alt(uint32_t alt) { rtl_path.descent_target.alt = alt; }

private:

    /**
     * 开始爬升阶段
     */
    void climb_start();

    /**
     * 开始返回家园阶段
     */
    void return_start();

    /**
     * 运行爬升和返回逻辑
     */
    void climb_return_run();

    /**
     * 开始在家园徘徊
     */
    void loiterathome_start();

    /**
     * 运行在家园徘徊的逻辑
     */
    void loiterathome_run();

    /**
     * 构建返回路径
     */
    void build_path();

    /**
     * 计算返回目标位置
     */
    void compute_return_target();

    SubMode _state = SubMode::INITIAL_CLIMB;  // 记录RTL的当前状态（初始爬升、返回家园等）
    bool _state_complete = false; // 如果当前状态完成则设置为true

    /**
     * 定义RTL路径结构体，包含各个阶段的目标位置
     */
    struct {
        // NEU坐标系，Z轴元素为高于ekf原点的高度，除非use_terrain为true，则Z轴元素为高于地形的高度
        Location origin_point;   // 原点位置
        Location climb_target;   // 爬升目标位置
        Location return_target;  // 返回目标位置
        Location descent_target; // 下降目标位置
        bool land;               // 是否需要着陆
    } rtl_path;

    /**
     * 定义返回目标高度类型的枚举
     */
    enum class ReturnTargetAltType {
        RELATIVE = 0,       // 相对高度
        RANGEFINDER = 1,    // 基于测距仪的高度
        TERRAINDATABASE = 2 // 基于地形数据库的高度
    };

    uint32_t _loiter_start_time; // 徘徊开始的时间戳

    bool terrain_following_allowed; // 是否允许地形跟随

    /**
     * 定义RTL选项参数的枚举
     */
    enum class Options : int32_t {
        // 前两位位仍然可用，飞行员偏航被映射到第2位，以与自动模式对称
        IgnorePilotYaw    = (1U << 2),
    };

};

class ModeSmartRTL : public ModeRTL {

public:
    // 继承基类构造函数
    using ModeRTL::Mode;

    /**
     * 获取模式的编号
     *
     * @return SMART_RTL模式的编号
     */
    Number mode_number() const override { return Number::SMART_RTL; }

    /**
     * 初始化SMART_RTL模式
     *
     * @param ignore_checks 是否忽略初始化检查
     * @return 初始化是否成功
     */
    bool init(bool ignore_checks) override;

    /**
     * 运行SMART_RTL模式的主要逻辑
     */
    void run() override;

    /**
     * 判断是否需要GPS
     *
     * @return 如果需要GPS则返回true
     */
    bool requires_GPS() const override { return true; }

    /**
     * 判断是否有手动油门控制
     *
     * @return 如果没有手动油门控制则返回false
     */
    bool has_manual_throttle() const override { return false; }

    /**
     * 判断是否允许装甲（arming）
     *
     * @param method 装甲方法
     * @return 如果不允许装甲则返回false
     */
    bool allows_arming(AP_Arming::Method method) const override { return false; }

    /**
     * 判断是否为自动驾驶模式
     *
     * @return 如果是自动驾驶模式则返回true
     */
    bool is_autopilot() const override { return true; }

    /**
     * 保存当前位置
     */
    void save_position();

    /**
     * 退出SMART_RTL模式
     */
    void exit() override;

    /**
     * 判断是否正在着陆
     *
     * @return 如果正在着陆则返回true
     */
    bool is_landing() const override;

    /**
     * 判断是否使用飞行员的偏航控制
     *
     * @return 如果使用则返回true
     */
    bool use_pilot_yaw() const override;

    /**
     * 定义安全RTL模式的子模式枚举
     */
    enum class SubMode : uint8_t {
        WAIT_FOR_PATH_CLEANUP,   // 等待路径清理
        PATH_FOLLOW,             // 路径跟随
        PRELAND_POSITION,        // 着陆前位置
        DESCEND,                 // 下降
        LAND                     // 着陆
    };

protected:

    /**
     * 获取模式的名称
     *
     * @return SMART_RTL模式的名称
     */
    const char *name() const override { return "SMARTRTL"; }

    /**
     * 获取模式的四字符名称
     *
     * @return SRTL 
     */
    const char *name4() const override { return "SRTL"; }

    /**
     * 为地面控制站报告当前位置的航点
     *
     * @param loc 航点位置
     * @return 如果成功获取航点则返回true
     */
    bool get_wp(Location &loc) const override;

    /**
     * 获取当前航点与无人机之间的距离
     *
     * @return 航点距离
     */
    uint32_t wp_distance() const override;

    /**
     * 获取当前航点的方位角
     *
     * @return 航点方位角
     */
    int32_t wp_bearing() const override;

    /**
     * 获取横穿误差
     *
     * @return 横穿误差
     */
    float crosstrack_error() const override { return wp_nav->crosstrack_error();}

private:

    /**
     * 运行等待路径清理的逻辑
     */
    void wait_cleanup_run();

    /**
     * 运行路径跟随的逻辑
     */
    void path_follow_run();

    /**
     * 执行着陆前位置的逻辑
     */
    void pre_land_position_run();

    /**
     * 执行着陆逻辑
     */
    void land();

    SubMode smart_rtl_state = SubMode::PATH_FOLLOW; // 记录SMART_RTL的当前子模式

    /**
     * 记录在路径跟随过程中未能获取下一个返回点的时间
     * 如果等待时间过长，则可能选择着陆
     */
    uint32_t path_follow_last_pop_fail_ms;

    /**
     * 备份最后弹出的点，以便如果车辆在到达家园前退出SmartRTL模式，可以将其恢复到路径中。
     * 如果为零，则表示无效
     */
    Vector3f dest_NED_backup;

};

class ModeSport : public Mode {

public:
    // 继承基类构造函数
    using Mode::Mode;

    /**
     * 获取模式的编号
     *
     * @return SPORT模式的编号
     */
    Number mode_number() const override { return Number::SPORT; }

    /**
     * 初始化SPORT模式
     *
     * @param ignore_checks 是否忽略初始化检查
     * @return 初始化是否成功
     */
    bool init(bool ignore_checks) override;

    /**
     * 运行SPORT模式的主要逻辑
     */
    void run() override;

    /**
     * 判断是否需要GPS
     *
     * @return 如果不需要GPS则返回false
     */
    bool requires_GPS() const override { return false; }

    /**
     * 判断是否有手动油门控制
     *
     * @return 如果没有手动油门控制则返回false
     */
    bool has_manual_throttle() const override { return false; }

    /**
     * 判断是否允许装甲（arming）
     *
     * @param method 装甲方法
     * @return 如果允许装甲则返回true
     */
    bool allows_arming(AP_Arming::Method method) const override { return true; };

    /**
     * 判断是否为自动驾驶模式
     *
     * @return 如果不是自动驾驶模式则返回false
     */
    bool is_autopilot() const override { return false; }

    /**
     * 判断是否允许用户起飞，且不需要导航
     *
     * @param must_navigate 是否必须导航
     * @return 如果不需要导航则允许用户起飞
     */
    bool has_user_takeoff(bool must_navigate) const override {
        return !must_navigate;
    }

protected:

    /**
     * 获取模式的名称
     *
     * @return SPORT模式的名称
     */
    const char *name() const override { return "SPORT"; }

    /**
     * 获取模式的四字符名称
     *
     * @return SPRT 
     */
    const char *name4() const override { return "SPRT"; }

private:

};

class ModeStabilize : public Mode {

public:
    // 继承基类构造函数
    using Mode::Mode;

    /**
     * 获取模式的编号
     *
     * @return STABILIZE模式的编号
     */
    Number mode_number() const override { return Number::STABILIZE; }

    /**
     * 运行STABILIZE模式的主要逻辑
     */
    virtual void run() override;

    /**
     * 判断是否需要GPS
     *
     * @return 如果不需要GPS则返回false
     */
    bool requires_GPS() const override { return false; }

    /**
     * 判断是否有手动油门控制
     *
     * @return 如果有手动油门控制则返回true
     */
    bool has_manual_throttle() const override { return true; }

    /**
     * 判断是否允许装甲（arming）
     *
     * @param method 装甲方法
     * @return 如果允许装甲则返回true
     */
    bool allows_arming(AP_Arming::Method method) const override { return true; };

    /**
     * 判断是否为自动驾驶模式
     *
     * @return 如果不是自动驾驶模式则返回false
     */
    bool is_autopilot() const override { return false; }

    /**
     * 判断是否允许保存调整（save trim）
     *
     * @return 如果允许保存调整则返回true
     */
    bool allows_save_trim() const override { return true; }

    /**
     * 判断是否允许自动调谐
     *
     * @return 如果允许自动调谐则返回true
     */
    bool allows_autotune() const override { return true; }

    /**
     * 判断是否允许翻转
     *
     * @return 如果允许翻转则返回true
     */
    bool allows_flip() const override { return true; }

protected:

    /**
     * 获取模式的名称
     *
     * @return STABILIZE模式的名称
     */
    const char *name() const override { return "STABILIZE"; }

    /**
     * 获取模式的四字符名称
     *
     * @return STAB 
     */
    const char *name4() const override { return "STAB"; }

private:

};

#if FRAME_CONFIG == HELI_FRAME
class ModeStabilize_Heli : public ModeStabilize {

public:
    // 继承基类构造函数
    using ModeStabilize::Mode;

    /**
     * 初始化STABILIZE_HELI模式
     *
     * @param ignore_checks 是否忽略初始化检查
     * @return 初始化是否成功
     */
    bool init(bool ignore_checks) override;

    /**
     * 运行STABILIZE_HELI模式的主要逻辑
     */
    void run() override;

    /**
     * 判断是否允许倒转
     *
     * @return 如果允许倒转则返回true
     */
    bool allows_inverted() const override { return true; };

protected:

private:

};
#endif


class ModeSystemId : public Mode {

public:
    ModeSystemId(void);
    // 返回模式编号
    Number mode_number() const override { return Number::SYSTEMID; }

    // 初始化模式
    bool init(bool ignore_checks) override;
    // 运行模式主循环
    void run() override;
    // 退出模式
    void exit() override;

    // 是否需要GPS
    bool requires_GPS() const override { return false; }
    // 是否有手动油门控制
    bool has_manual_throttle() const override { return true; }
    // 是否允许解锁
    bool allows_arming(AP_Arming::Method method) const override { return false; };
    // 是否为自动驾驶模式
    bool is_autopilot() const override { return false; }
    // 是否记录姿态数据
    bool logs_attitude() const override { return true; }

    // 设置幅度
    void set_magnitude(float input) { waveform_magnitude.set(input); }

    // 参数组信息
    static const struct AP_Param::GroupInfo var_info[];

    // 啁啾输入对象
    Chirp chirp_input;

protected:

    // 返回模式名称
    const char *name() const override { return "SYSTEMID"; }
    // 返回模式4字符名称
    const char *name4() const override { return "SYSI"; }

private:

    // 记录数据
    void log_data() const;
    // 判断是否为位置控制轴类型
    bool is_poscontrol_axis_type() const;

    // 轴类型枚举
    enum class AxisType {
        NONE = 0,               // 无
        INPUT_ROLL = 1,         // 输入滚转轴激励
        INPUT_PITCH = 2,        // 输入俯仰轴激励
        INPUT_YAW = 3,          // 输入偏航轴激励
        RECOVER_ROLL = 4,       // 恢复滚转轴激励
        RECOVER_PITCH = 5,      // 恢复俯仰轴激励
        RECOVER_YAW = 6,        // 恢复偏航轴激励
        RATE_ROLL = 7,          // 滚转角速度激励
        RATE_PITCH = 8,         // 俯仰角速度激励
        RATE_YAW = 9,           // 偏航角速度激励
        MIX_ROLL = 10,          // 混合滚转轴激励
        MIX_PITCH = 11,         // 混合俯仰轴激励
        MIX_YAW = 12,           // 混合偏航轴激励
        MIX_THROTTLE = 13,      // 混合油门轴激励
        DISTURB_POS_LAT = 14,   // 横向位置扰动激励
        DISTURB_POS_LONG = 15,  // 纵向位置扰动激励
        DISTURB_VEL_LAT = 16,   // 横向速度扰动激励
        DISTURB_VEL_LONG = 17,  // 纵向速度扰动激励
        INPUT_VEL_LAT = 18,     // 横向速度输入激励
        INPUT_VEL_LONG = 19,    // 纵向速度输入激励
    };

    AP_Int8 axis;               // 控制激励轴,非零值显示其他参数
    AP_Float waveform_magnitude;// 啁啾波形幅度
    AP_Float frequency_start;   // 啁啾起始频率
    AP_Float frequency_stop;    // 啁啾结束频率
    AP_Float time_fade_in;      // 达到最大幅度的时间
    AP_Float time_record;       // 完成啁啾波形的时间
    AP_Float time_fade_out;     // 啁啾结束后达到零幅度的时间

    bool att_bf_feedforward;    // 姿态控制前馈设置
    float waveform_time;        // 波形时间参考
    float waveform_sample;      // 当前波形样本
    float waveform_freq_rads;   // 瞬时波形频率
    float time_const_freq;      // 啁啾开始前的恒定频率时间
    int8_t log_subsample;       // 日志子采样倍数
    Vector2f target_vel;        // 位置控制器模式的目标速度
    Vector2f target_pos;        // 目标位置
    Vector2f input_vel_last;    // 上一周期的输入速度
    // 系统识别状态
    enum class SystemIDModeState {
        SYSTEMID_STATE_STOPPED, // 停止状态
        SYSTEMID_STATE_TESTING  // 测试状态
    } systemid_state;
};

class ModeThrow : public Mode {

public:
    // 继承构造函数
    using Mode::Mode;
    // 返回模式编号
    Number mode_number() const override { return Number::THROW; }

    // 初始化模式
    bool init(bool ignore_checks) override;
    // 运行模式主循环
    void run() override;

    // 是否需要GPS
    bool requires_GPS() const override { return true; }
    // 是否有手动油门控制
    bool has_manual_throttle() const override { return false; }
    // 是否允许解锁
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    // 是否为自动驾驶模式
    bool is_autopilot() const override { return false; }

    // 抛飞类型枚举
    enum class ThrowType {
        Upward = 0, // 向上抛飞
        Drop = 1    // 下落
    };

    // 抛飞前电机状态枚举
    enum class PreThrowMotorState {
        STOPPED = 0, // 停止
        RUNNING = 1, // 运行
    };

protected:

    // 返回模式名称
    const char *name() const override { return "THROW"; }
    // 返回模式4字符名称
    const char *name4() const override { return "THRW"; }

private:

    // 检测是否抛飞
    bool throw_detected();
    // 检查抛飞位置是否合适
    bool throw_position_good() const;
    // 检查抛飞高度是否合适
    bool throw_height_good() const;
    // 检查抛飞姿态是否合适
    bool throw_attitude_good() const;

    // 抛飞阶段枚举
    enum ThrowModeStage {
        Throw_Disarmed,                 // 解锁状态
        Throw_Detecting,                // 检测抛飞
        Throw_Wait_Throttle_Unlimited,  // 等待油门解除限制
        Throw_Uprighting,               // 调整姿态
        Throw_HgtStabilise,             // 高度稳定
        Throw_PosHold                   // 位置保持
    };

    ThrowModeStage stage = Throw_Disarmed;  // 当前抛飞阶段
    ThrowModeStage prev_stage = Throw_Disarmed;  // 上一个抛飞阶段
    uint32_t last_log_ms;  // 上次记录日志的时间
    bool nextmode_attempted;  // 是否尝试切换到下一个模式
    uint32_t free_fall_start_ms;    // 检测到自由落体的系统时间
    float free_fall_start_velz;     // 检测到自由落体时的垂直速度
};

#if MODE_TURTLE_ENABLED
class ModeTurtle : public Mode {

public:
    // 继承构造函数
    using Mode::Mode;
    // 返回模式编号
    Number mode_number() const override { return Number::TURTLE; }

    // 初始化模式
    bool init(bool ignore_checks) override;
    // 运行模式主循环
    void run() override;
    // 退出模式
    void exit() override;

    // 是否需要GPS
    bool requires_GPS() const override { return false; }
    // 是否有手动油门控制
    bool has_manual_throttle() const override { return true; }
    // 是否允许解锁
    bool allows_arming(AP_Arming::Method method) const override;
    // 是否为自动驾驶模式
    bool is_autopilot() const override { return false; }
    // 改变电机方向
    void change_motor_direction(bool reverse);
    // 输出到电机
    void output_to_motors() override;

protected:
    // 返回模式名称
    const char *name() const override { return "TURTLE"; }
    // 返回模式4字符名称
    const char *name4() const override { return "TRTL"; }

private:
    // 解锁电机
    void arm_motors();
    // 锁定电机
    void disarm_motors();

    float motors_output;  // 电机输出
    Vector2f motors_input;  // 电机输入
    uint32_t last_throttle_warning_output_ms;  // 上次油门警告输出时间
};
#endif

// 以下模式依赖于引导模式,因此必须在最后声明(而不是按字母顺序)

class ModeAvoidADSB : public ModeGuided {

public:
    // 继承构造函数
    using ModeGuided::Mode;
    // 返回模式编号
    Number mode_number() const override { return Number::AVOID_ADSB; }

    // 初始化模式
    bool init(bool ignore_checks) override;
    // 运行模式主循环
    void run() override;

    // 是否需要GPS
    bool requires_GPS() const override { return true; }
    // 是否有手动油门控制
    bool has_manual_throttle() const override { return false; }
    // 是否允许解锁
    bool allows_arming(AP_Arming::Method method) const override { return false; }
    // 是否为自动驾驶模式
    bool is_autopilot() const override { return true; }

    // 设置速度
    bool set_velocity(const Vector3f& velocity_neu);

protected:

    // 返回模式名称
    const char *name() const override { return "AVOID_ADSB"; }
    // 返回模式4字符名称
    const char *name4() const override { return "AVOI"; }

private:

};

#if MODE_FOLLOW_ENABLED
class ModeFollow : public ModeGuided {

public:

    // 继承构造函数
    using ModeGuided::Mode;
    // 返回模式编号
    Number mode_number() const override { return Number::FOLLOW; }

    // 初始化模式
    bool init(bool ignore_checks) override;
    // 退出模式
    void exit() override;
    // 运行模式主循环
    void run() override;

    // 是否需要GPS
    bool requires_GPS() const override { return true; }
    // 是否有手动油门控制
    bool has_manual_throttle() const override { return false; }
    // 是否允许解锁
    bool allows_arming(AP_Arming::Method method) const override { return false; }
    // 是否为自动驾驶模式
    bool is_autopilot() const override { return true; }

protected:

    // 返回模式名称
    const char *name() const override { return "FOLLOW"; }
    // 返回模式4字符名称
    const char *name4() const override { return "FOLL"; }

    // 用于向地面站报告
    bool get_wp(Location &loc) const override;
    uint32_t wp_distance() const override;
    int32_t wp_bearing() const override;

    uint32_t last_log_ms;   // 上次记录期望速度的系统时间
};
#endif

class ModeZigZag : public Mode {        

public:
    ModeZigZag(void);

    // 继承构造函数
    using Mode::Mode;
    // 返回模式编号
    Number mode_number() const override { return Number::ZIGZAG; }

    // 目的地枚举
    enum class Destination : uint8_t {
        A,  // 目的地A
        B,  // 目的地B
    };

    // 方向枚举
    enum class Direction : uint8_t {
        FORWARD,        // 从偏航方向向前移动
        RIGHT,          // 从偏航方向向右移动
        BACKWARD,       // 从偏航方向向后移动
        LEFT,           // 从偏航方向向左移动
    } zigzag_direction;

    // 初始化模式
    bool init(bool ignore_checks) override;
    // 退出模式
    void exit() override;
    // 运行模式主循环
    void run() override;

    // 自动控制方法,飞行器飞行网格模式
    void run_auto();
    void suspend_auto();
    void init_auto();

    // 是否需要GPS
    bool requires_GPS() const override { return true; }
    // 是否有手动油门控制
    bool has_manual_throttle() const override { return false; }
    // 是否允许解锁
    bool allows_arming(AP_Arming::Method method) const override { return true; }
    // 是否为自动驾驶模式
    bool is_autopilot() const override { return true; }
    // 是否有用户起飞
    bool has_user_takeoff(bool must_navigate) const override { return true; }

    // 保存当前位置为A或B。如果A和B都已保存,则移动到指定的位置
    void save_or_move_to_destination(Destination ab_dest);

    // 返回手动控制给飞行员
    void return_to_manual_control(bool maintain_target);

    // 参数组信息
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // 返回模式名称
    const char *name() const override { return "ZIGZAG"; }
    // 返回模式4字符名称
    const char *name4() const override { return "ZIGZ"; }
    // 返回到航点的距离
    uint32_t wp_distance() const override;
    // 返回到航点的方位角
    int32_t wp_bearing() const override;
    // 返回横向跟踪误差
    float crosstrack_error() const override;

private:

    // 自动控制
    void auto_control();
    // 手动控制
    void manual_control();
    // 是否到达目的地
    bool reached_destination();
    // 计算下一个目的地
    bool calculate_next_dest(Destination ab_dest, bool use_wpnav_alt, Vector3f& next_dest, bool& terrain_alt) const;
    // 喷洒
    void spray(bool b);
    // 计算侧面目的地
    bool calculate_side_dest(Vector3f& next_dest, bool& terrain_alt) const;
    // 移动到侧面
    void move_to_side();

    Vector2f dest_A;    // A点位置,在EKF原点的NEU坐标系中,单位为厘米
    Vector2f dest_B;    // B点位置,在EKF原点的NEU坐标系中,单位为厘米
    Vector3f current_dest; // 当前目标位置(用于暂停后恢复)
    bool current_terr_alt; // 当前是否使用地形高度

    // 参数
    AP_Int8  _auto_enabled;    // 顶层启用/禁用控制
#if HAL_SPRAYER_ENABLED
    AP_Int8  _spray_enabled;   // 自动喷洒启用/禁用
#endif
    AP_Int8  _wp_delay;        // 之字形航点延迟
    AP_Float _side_dist;       // 侧向距离
    AP_Int8  _direction;       // 侧向方向
    AP_Int16 _line_num;        // 总行数

    // 之字形状态枚举
    enum ZigZagState {
        STORING_POINTS, // 存储A和B点,飞行员手动控制
        AUTO,           // A和B点定义后,飞行员切换开关从一侧到另一侧,飞行器自主飞行
        MANUAL_REGAIN   // 飞行员将开关切换到中间位置,恢复手动控制
    } stage;

    // 自动状态枚举
    enum AutoState {
        MANUAL,         // 不在之字形自动模式
        AB_MOVING,      // 从A移动到B或从B移动到A
        SIDEWAYS,       // 侧向移动
    } auto_stage;

    uint32_t reach_wp_time_ms = 0;  // 飞行器到达目的地的时间(如果尚未到达则为零)
    Destination ab_dest_stored;     // 存储当前目的地
    bool is_auto;                   // 启用之字形自动功能,自动化AB和侧向移动
    uint16_t line_count = 0;        // 当前行数
    int16_t line_num = 0;           // 目标行数
    bool is_suspended;              // 之字形自动模式是否暂停
};

#if MODE_AUTOROTATE_ENABLED
class ModeAutorotate : public Mode {

public:

    // 继承基类构造函数
    using Mode::Mode;
    // 返回模式编号
    Number mode_number() const override { return Number::AUTOROTATE; }

    // 初始化自动旋翼模式
    bool init(bool ignore_checks) override;
    // 运行自动旋翼模式的主要逻辑
    void run() override;

    // 判断是否为自动驾驶模式
    bool is_autopilot() const override { return true; }
    // 判断是否需要GPS
    bool requires_GPS() const override { return false; }
    // 判断是否有手动油门控制
    bool has_manual_throttle() const override { return false; }
    // 判断是否允许解锁
    bool allows_arming(AP_Arming::Method method) const override { return false; };

    // 参数组信息
    static const struct AP_Param::GroupInfo  var_info[];

protected:

    // 返回模式名称
    const char *name() const override { return "AUTOROTATE"; }
    // 返回模式4字符名称
    const char *name4() const override { return "AROT"; }

private:

    // --- 内部变量 ---
    float _initial_rpm;             // 飞行模式初始化时记录的主旋翼转速(RPM)
    float _target_head_speed;       // 目标主旋翼转速,归一化为主旋翼设定点
    float _desired_v_z;             // 期望垂直速度
    int32_t _pitch_target;          // 传递给姿态控制器的目标俯仰角
    uint32_t _entry_time_start_ms;  // 进入阶段转换到滑翔阶段的剩余时间
    float _hs_decay;                // 进入阶段的主旋翼加速度
    float _bail_time;               // 退出紧急中止阶段的计时器(秒)
    uint32_t _bail_time_start_ms;   // 紧急中止开始时间
    float _target_climb_rate_adjust;// 紧急中止阶段使用的目标垂直加速度
    float _target_pitch_adjust;     // 紧急中止阶段使用的目标俯仰速率

    // 自动旋翼阶段枚举
    enum class Autorotation_Phase {
        ENTRY,          // 进入阶段
        SS_GLIDE,       // 稳态滑翔阶段
        FLARE,          // 拉平阶段
        TOUCH_DOWN,     // 接地阶段
        BAIL_OUT        // 紧急中止阶段
    } phase_switch;
        
    // 导航决策枚举
    enum class Navigation_Decision {
        USER_CONTROL_STABILISED,    // 用户控制稳定
        STRAIGHT_AHEAD,             // 直线前进
        INTO_WIND,                  // 逆风
        NEAREST_RALLY               // 最近的集结点
    } nav_pos_switch;

    // --- 内部标志 ---
    struct controller_flags {
            bool entry_initial             : 1;    // 进入阶段初始化标志
            bool ss_glide_initial          : 1;    // 稳态滑翔阶段初始化标志
            bool flare_initial             : 1;    // 拉平阶段初始化标志
            bool touch_down_initial        : 1;    // 接地阶段初始化标志
            bool straight_ahead_initial    : 1;    // 直线前进初始化标志
            bool level_initial             : 1;    // 水平初始化标志
            bool break_initial             : 1;    // 制动初始化标志
            bool bail_out_initial          : 1;    // 紧急中止初始化标志
            bool bad_rpm                   : 1;    // 异常RPM标志
    } _flags;

    struct message_flags {
            bool bad_rpm                   : 1;    // 异常RPM消息标志
    } _msg_flags;

    //--- 内部函数 ---
    void warning_message(uint8_t message_n);    // 处理终端输出消息

};
#endif