#pragma once

#include <AP_Arming/AP_Arming.h>

#ifndef AP_PLANE_BLACKBOX_LOGGING
#define AP_PLANE_BLACKBOX_LOGGING 0
#endif

/*
  一个针对飞机的特定解锁类
 */
class AP_Arming_Plane : public AP_Arming
{
public:
    // 构造函数
    AP_Arming_Plane()
        : AP_Arming()
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* 不允许复制 */
    CLASS_NO_COPY(AP_Arming_Plane);

    // 预解锁检查
    bool pre_arm_checks(bool report) override;
    // 解锁检查
    bool arm_checks(AP_Arming::Method method) override;

    // 用于保存参数信息的var_info
    static const struct AP_Param::GroupInfo var_info[];

    // 上锁
    bool disarm(AP_Arming::Method method, bool do_disarm_checks=true) override;
    // 解锁
    bool arm(AP_Arming::Method method, bool do_arming_checks=true) override;

    // 更新软件解锁状态
    void update_soft_armed();
    // 获取延迟解锁状态
    bool get_delay_arming() const { return delay_arming; };

    // 强制性检查，不能被绕过。仅当ARMING_CHECK为零或强制解锁时才会调用此函数
    bool mandatory_checks(bool display_failure) override;

protected:
    // INS检查
    bool ins_checks(bool report) override;
    // 检查是否需要地形数据库
    bool terrain_database_required() const override;

    // 四旋翼检查
    bool quadplane_checks(bool display_failure);
    // 任务检查
    bool mission_checks(bool report) override;

    // 检查是否已接收到遥控信号（如果配置为使用遥控）
    bool rc_received_if_enabled_check(bool display_failure);

private:
    // 改变解锁状态
    void change_arm_state(void);

    // 一次性延迟，持续时间为AP_ARMING_DELAY_MS，由四旋翼用于在解锁后延迟启动：
    // 除非设置了OPTION_DELAY_ARMING或OPTION_TILT_DISARMED，否则将被忽略
    bool delay_arming;

#if AP_PLANE_BLACKBOX_LOGGING
    // 黑匣子记录速度
    AP_Float blackbox_speed;
    // 上次超过3D速度的时间
    uint32_t last_over_3dspeed_ms;
#endif
};
