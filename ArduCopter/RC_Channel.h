// 确保头文件只被包含一次
#pragma once

// 包含所需的头文件
#include <RC_Channel/RC_Channel.h>
#include <AP_Motors/AP_Motors.h>
#include "mode.h"

// RC_Channel_Copter类继承自RC_Channel基类
class RC_Channel_Copter : public RC_Channel
{

public:

protected:
    // 初始化辅助功能
    void init_aux_function(AUX_FUNC ch_option, AuxSwitchPos) override;
    // 执行辅助功能
    bool do_aux_function(AUX_FUNC ch_option, AuxSwitchPos) override;

private:
    // 改变飞行模式的辅助功能
    void do_aux_function_change_mode(const Mode::Number mode,
                                     const AuxSwitchPos ch_flag);
    // 改变空中模式的辅助功能                                 
    void do_aux_function_change_air_mode(const AuxSwitchPos ch_flag);
    // 改变强制飞行状态的辅助功能
    void do_aux_function_change_force_flying(const AuxSwitchPos ch_flag);

    // 当模式开关位置改变时调用
    void mode_switch_changed(modeswitch_pos_t new_pos) override;

};

// RC_Channels_Copter类继承自RC_Channels基类
class RC_Channels_Copter : public RC_Channels
{
public:
    // 检查是否有有效的遥控输入
    bool has_valid_input() const override;
    // 检查是否处于遥控器故障保护状态
    bool in_rc_failsafe() const override;

    // 获取解锁通道
    RC_Channel *get_arming_channel(void) const override;

    // RC通道对象数组
    RC_Channel_Copter obj_channels[NUM_RC_CHANNELS];
    // 获取指定通道对象的指针
    RC_Channel_Copter *channel(const uint8_t chan) override {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

    // 检查是否需要进行油门解锁检查
    bool arming_check_throttle() const override;

protected:
    // 获取飞行模式通道号
    int8_t flight_mode_channel_number() const override;

};
