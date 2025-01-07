#include "mode.h"
#include "Plane.h"

void ModeFBWA::update()
{
    // 使用摇杆设置导航滚转和俯仰角
    plane.nav_roll_cd  = plane.channel_roll->norm_input() * plane.roll_limit_cd;
    plane.update_load_factor();
    float pitch_input = plane.channel_pitch->norm_input();
    if (pitch_input > 0) {
        // 正俯仰输入
        plane.nav_pitch_cd = pitch_input * plane.aparm.pitch_limit_max*100;
    } else {
        // 负俯仰输入
        plane.nav_pitch_cd = -(pitch_input * plane.pitch_limit_min*100);
    }
    plane.adjust_nav_pitch_throttle();
    // 限制俯仰角在允许范围内
    plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, plane.pitch_limit_min*100, plane.aparm.pitch_limit_max.get()*100);
    if (plane.fly_inverted()) {
        // 倒飞时反转俯仰角
        plane.nav_pitch_cd = -plane.nav_pitch_cd;
    }
    if (plane.failsafe.rc_failsafe && plane.g.fs_action_short == FS_ACTION_SHORT_FBWA) {
        // FBWA失效保护滑翔
        plane.nav_roll_cd = 0;
        plane.nav_pitch_cd = 0;
        SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::MIN);
    }
    // 检查FBWA尾轮拖行起飞模式
    RC_Channel *chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FBWA_TAILDRAGGER);
    if (chan != nullptr) {
        // 检查用户是否启用FBWA尾轮拖行起飞模式
        bool tdrag_mode = chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH;
        if (tdrag_mode && !plane.auto_state.fbwa_tdrag_takeoff_mode) {
            if (plane.auto_state.highest_airspeed < plane.g.takeoff_tdrag_speed1) {
                // 启用FBWA尾轮拖行起飞模式
                plane.auto_state.fbwa_tdrag_takeoff_mode = true;
                plane.gcs().send_text(MAV_SEVERITY_WARNING, "FBWA tdrag mode");
            }
        }
    }
}

void ModeFBWA::run()
{
    // 运行基类函数，然后输出油门
    Mode::run();

    output_pilot_throttle();
}
