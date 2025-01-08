#pragma once

#include <AP_Arming/AP_Arming.h>

// Copter特定的解锁检查类,继承自AP_Arming基类
class AP_Arming_Copter : public AP_Arming
{
public:
    friend class Copter;
    friend class ToyMode;

    // 构造函数,设置默认需要最小PWM值才能解锁
    AP_Arming_Copter() : AP_Arming()
    {
        // 默认将REQUIRE参数设为1(Copter没有实际的ARMING_REQUIRE参数)
        require.set_default((uint8_t)Required::YES_MIN_PWM);
    }

    /* 禁止拷贝构造 */
    CLASS_NO_COPY(AP_Arming_Copter);

    // 遥控器校准检查
    bool rc_calibration_checks(bool display_failure) override;

    // 锁定和解锁电机的方法
    bool disarm(AP_Arming::Method method, bool do_disarm_checks=true) override;
    bool arm(AP_Arming::Method method, bool do_arming_checks=true) override;

protected:

    // 预解锁检查相关函数
    bool pre_arm_checks(bool display_failure) override;
    bool pre_arm_ekf_attitude_check();
#if HAL_PROXIMITY_ENABLED
    // 障碍物检测系统检查
    bool proximity_checks(bool display_failure) const override;
#endif
    // 解锁检查
    bool arm_checks(AP_Arming::Method method) override;

    // 强制检查,即使ARMING_CHECK为0或强制解锁时也会执行
    bool mandatory_checks(bool display_failure) override;

    // 以下检查函数会调用AP_Arming基类中的检查:
    bool ins_checks(bool display_failure) override;        // 惯性导航系统检查
    bool gps_checks(bool display_failure) override;        // GPS检查
    bool barometer_checks(bool display_failure) override;  // 气压计检查
    bool board_voltage_checks(bool display_failure) override; // 电路板电压检查

    // 以下检查函数不会调用AP_Arming基类:
    bool parameter_checks(bool display_failure);           // 参数检查
    bool oa_checks(bool display_failure);                 // 避障系统检查
    bool mandatory_gps_checks(bool display_failure);      // 强制GPS检查
    bool gcs_failsafe_check(bool display_failure);        // 地面站失效保护检查
    bool winch_checks(bool display_failure) const;        // 绞车检查
    bool alt_checks(bool display_failure);                // 高度检查
    bool rc_throttle_failsafe_checks(bool display_failure) const; // 遥控器油门失效保护检查

    // 设置预解锁检查标志
    void set_pre_arm_check(bool b);

    // 检查是否需要完整的地形数据库
    bool terrain_database_required() const override;

private:

    // 实际执行预解锁检查,并存储检查结果
    bool run_pre_arm_checks(bool display_failure);

};
