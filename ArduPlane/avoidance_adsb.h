#pragma once

#include <AP_Avoidance/AP_Avoidance.h>

// 提供飞机特定的避障实现。虽然大部分实际避障逻辑存在于
// AP_Avoidance中，但此类允许飞机重写基础功能 - 例如，在着陆时不执行任何操作。
class AP_Avoidance_Plane : public AP_Avoidance {
public:

    using AP_Avoidance::AP_Avoidance;

    /* 不允许复制 */
    CLASS_NO_COPY(AP_Avoidance_Plane);

protected:
    // 重写避障处理器
    MAV_COLLISION_ACTION handle_avoidance(const AP_Avoidance::Obstacle *obstacle, MAV_COLLISION_ACTION requested_action) override;

    // 重写恢复处理器
    void handle_recovery(RecoveryAction recovery_action) override;

    // 检查飞行模式是否为avoid_adsb
    bool check_flightmode(bool allow_mode_change);

    // 垂直避障处理器
    bool handle_avoidance_vertical(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change, Location &new_loc);

    // 水平避障处理器
    bool handle_avoidance_horizontal(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change, Location &new_loc);

    // 避障开始前的控制模式
    enum Mode::Number prev_control_mode_number = Mode::Number::RTL;
};
