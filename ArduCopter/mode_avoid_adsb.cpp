#include "Copter.h"

/*
 * control_avoid.cpp - init and run calls for AP_Avoidance's AVOID flight mode
 * control_avoid.cpp - AP_Avoidance的AVOID飞行模式的初始化和运行调用
 *
 * This re-uses GUIDED mode functions but does not interfere with the GCS or companion computer's
 * use of guided mode because the velocity requests arrive from different sources (i.e MAVLink messages
 * for GCS and Companion Computers vs the AP_Avoidance_Copter class for adsb avoidance) and inputs from
 * each source are only accepted and processed in the appropriate flight mode.
 * 这里重用了GUIDED模式的功能,但不会干扰地面站或伴随计算机对引导模式的使用,
 * 因为速度请求来自不同的源(即地面站和伴随计算机的MAVLink消息 vs 用于ADSB避障的AP_Avoidance_Copter类),
 * 并且每个源的输入只在适当的飞行模式下被接受和处理。
 */

// initialise avoid_adsb controller
// 初始化avoid_adsb控制器
bool ModeAvoidADSB::init(const bool ignore_checks)
{
    // re-use guided mode
    // 重用引导模式
    return ModeGuided::init(ignore_checks);
}

bool ModeAvoidADSB::set_velocity(const Vector3f& velocity_neu)
{
    // check flight mode
    // 检查飞行模式
    if (copter.flightmode->mode_number() != Mode::Number::AVOID_ADSB) {
        return false;
    }

    // re-use guided mode's velocity controller
    // 重用引导模式的速度控制器
    ModeGuided::set_velocity(velocity_neu);
    return true;
}

// runs the AVOID_ADSB controller
// 运行AVOID_ADSB控制器
void ModeAvoidADSB::run()
{
    // re-use guided mode's velocity controller
    // Note: this is safe from interference from GCSs and companion computer's whose guided mode
    //       position and velocity requests will be ignored while the vehicle is not in guided mode
    // 重用引导模式的速度控制器
    // 注意:这可以避免来自地面站和伴随计算机的干扰,因为当飞行器不在引导模式时,
    //      它们的位置和速度请求将被忽略
    ModeGuided::run();
}
