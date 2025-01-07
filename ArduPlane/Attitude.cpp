#include "Plane.h"

/*
  计算控制面的速度缩放因子。这个因子应用于PID控制器，
  以根据速度改变控制面的移动量。在高速时我们减小控制面移动，
  低速时增大移动。
 */
float Plane::calc_speed_scaler(void)
{
    float aspeed, speed_scaler;
    if (ahrs.airspeed_estimate(aspeed)) {
        // 如果有空速估计
        if (aspeed > auto_state.highest_airspeed && arming.is_armed_and_safety_off()) {
            auto_state.highest_airspeed = aspeed;
        }
        // 确保我们在全部配置的空速范围内都有缩放
        const float airspeed_min = MAX(aparm.airspeed_min, MIN_AIRSPEED_MIN);
        const float scale_min = MIN(0.5, g.scaling_speed / (2.0 * aparm.airspeed_max));
        const float scale_max = MAX(2.0, g.scaling_speed / (0.7 * airspeed_min));
        if (aspeed > 0.0001f) {
            speed_scaler = g.scaling_speed / aspeed;
        } else {
            speed_scaler = scale_max;
        }
        speed_scaler = constrain_float(speed_scaler, scale_min, scale_max);

#if HAL_QUADPLANE_ENABLED
        if (quadplane.in_vtol_mode() && arming.is_armed_and_safety_off()) {
            // 在VTOL模式下，在低速时限制控制面移动以防止不稳定
            float threshold = airspeed_min * 0.5;
            if (aspeed < threshold) {
                float new_scaler = linear_interpolate(0.001, g.scaling_speed / threshold, aspeed, 0, threshold);
                speed_scaler = MIN(speed_scaler, new_scaler);

                // 我们还衰减积分器，以防止低速时的积分在高速时持续存在
                rollController.decay_I();
                pitchController.decay_I();
                yawController.decay_I();
            }
        }
#endif
    } else if (arming.is_armed_and_safety_off()) {
        // 如果没有空速估计，使用油门输出来缩放假定的控制面移动
        float throttle_out = MAX(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle), 1);
        speed_scaler = sqrtf(THROTTLE_CRUISE / throttle_out);
        // 由于没有真实的速度信息，这种情况下的约束更严格
        speed_scaler = constrain_float(speed_scaler, 0.6f, 1.67f);
    } else {
        // 没有速度估计且未解锁，使用单位缩放
        speed_scaler = 1;
    }
    if (!plane.ahrs.using_airspeed_sensor()  && 
        (plane.flight_option_enabled(FlightOptions::SURPRESS_TKOFF_SCALING)) &&
        (plane.flight_stage == AP_FixedWing::FlightStage::TAKEOFF)) { 
        // 在自动起飞的爬升阶段，如果不使用空速传感器，
        // 由于不准确的空速估计可能导致问题，抑制缩放
        return MIN(speed_scaler, 1.0f) ;
    }
    return speed_scaler;
}

/*
  返回当前设置和模式是否应该允许摇杆混控
 */
bool Plane::stick_mixing_enabled(void)
{
    if (!rc().has_valid_input()) {
        // 没有有效的遥控输入时，永不进行摇杆混控
        return false;
    }
#if AP_FENCE_ENABLED
    const bool stickmixing = fence_stickmixing();
#else
    const bool stickmixing = true;
#endif
#if HAL_QUADPLANE_ENABLED
    if (control_mode == &mode_qrtl &&
        quadplane.poscontrol.get_state() >= QuadPlane::QPOS_POSITION1) {
        // 用户可能正在重新定位
        return false;
    }
    if (quadplane.in_vtol_land_poscontrol()) {
        // 用户可能正在重新定位
        return false;
    }
#endif
    if (control_mode->does_auto_throttle() && plane.control_mode->does_auto_navigation()) {
        // 我们处于自动模式。检查摇杆混控标志
        if (g.stick_mixing != StickMixing::NONE &&
            g.stick_mixing != StickMixing::VTOL_YAW &&
            stickmixing) {
            return true;
        } else {
            return false;
        }
    }

    if (failsafe.rc_failsafe && g.fs_action_short == FS_ACTION_SHORT_FBWA) {
        // 在FBWA滑翔模式下不进行摇杆混控
        return false;
    }

    // 非自动模式。始终进行摇杆混控
    return true;
}

/*
  这是主要的横滚稳定函数。它使用先前设置的nav_roll
  计算roll servo_out，试图在给定的横滚角度上稳定飞机
 */
void Plane::stabilize_roll()
{
    if (fly_inverted()) {
        // 我们想倒飞。我们需要处理roll_sensor的包裹与nav_roll的包裹干扰，
        // 这会真正混淆PID代码。处理这个问题最简单的方法是
        // 确保两者从零开始朝同一方向走
        nav_roll_cd += 18000;
        if (ahrs.roll_sensor < 0) nav_roll_cd -= 36000;
    }

    const float roll_out = stabilize_roll_get_roll_out();
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, roll_out);
}

float Plane::stabilize_roll_get_roll_out()
{
    const float speed_scaler = get_speed_scaler();
#if HAL_QUADPLANE_ENABLED
    if (!quadplane.use_fw_attitude_controllers()) {
        // 使用VTOL速率控制，以确保一致性
        const auto &pid_info = quadplane.attitude_control->get_rate_roll_pid().get_pid_info();

        // 将前馈缩放到角度P
        if (quadplane.option_is_set(QuadPlane::OPTION::SCALE_FF_ANGLE_P)) {
            const float mc_angR = quadplane.attitude_control->get_angle_roll_p().kP()
                * quadplane.attitude_control->get_last_angle_P_scale().x;
            if (is_positive(mc_angR)) {
                rollController.set_ff_scale(MIN(1.0, 1.0 / (mc_angR * rollController.tau())));
            }
        }

        const float roll_out = rollController.get_rate_out(degrees(pid_info.target), speed_scaler);
        /* 当固定翼控制从属于VTOL控制时，我们需要衰减积分器
           以防止两个控制器之间的对立积分相互平衡
        */
        rollController.decay_I();
        return roll_out;
    }
#endif

    bool disable_integrator = false;
    if (control_mode == &mode_stabilize && channel_roll->get_control_in() != 0) {
        disable_integrator = true;
    }
    return rollController.get_servo_out(nav_roll_cd - ahrs.roll_sensor, speed_scaler, disable_integrator,
                                        ground_mode && !(plane.flight_option_enabled(FlightOptions::DISABLE_GROUND_PID_SUPPRESSION)));
}

/*
  这是主要的俯仰稳定函数。它使用先前设置的nav_pitch
  计算servo_out值，试图在给定的姿态上稳定飞机。
 */
void Plane::stabilize_pitch()
{
    int8_t force_elevator = takeoff_tail_hold();
    if (force_elevator != 0) {
        // 我们在起飞时保持尾部向下。只需将百分比
        // 转换为-4500..4500厘度角
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, 45*force_elevator);
        return;
    }

    const float pitch_out = stabilize_pitch_get_pitch_out();
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pitch_out);
}

float Plane::stabilize_pitch_get_pitch_out()
{
    const float speed_scaler = get_speed_scaler();
#if HAL_QUADPLANE_ENABLED
    if (!quadplane.use_fw_attitude_controllers()) {
        // 使用VTOL速率控制，以确保一致性
        const auto &pid_info = quadplane.attitude_control->get_rate_pitch_pid().get_pid_info();

        // 将前馈缩放到角度P
        if (quadplane.option_is_set(QuadPlane::OPTION::SCALE_FF_ANGLE_P)) {
            const float mc_angP = quadplane.attitude_control->get_angle_pitch_p().kP()
                * quadplane.attitude_control->get_last_angle_P_scale().y;
            if (is_positive(mc_angP)) {
                pitchController.set_ff_scale(MIN(1.0, 1.0 / (mc_angP * pitchController.tau())));
            }
        }

        const int32_t pitch_out = pitchController.get_rate_out(degrees(pid_info.target), speed_scaler);
        /* 当固定翼控制从属于VTOL控制时，我们需要衰减积分器
           以防止两个控制器之间的对立积分相互平衡
        */
        pitchController.decay_I();
        return pitch_out;
    }
#endif
    // 如果设置了LANDING_FLARE RCx_OPTION开关，且在固定翼模式，手动油门，油门怠速，
    // 则如果设置了FORCE_FLARE_ATTITUDE飞行选项，将俯仰设置为LAND_PITCH_DEG
#if HAL_QUADPLANE_ENABLED
    const bool quadplane_in_frwd_transition = quadplane.in_frwd_transition();
#else
    const bool quadplane_in_frwd_transition = false;
#endif

    int32_t demanded_pitch = nav_pitch_cd + int32_t(g.pitch_trim * 100.0) + SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) * g.kff_throttle_to_pitch;
    bool disable_integrator = false;
    if (control_mode == &mode_stabilize && channel_pitch->get_control_in() != 0) {
        disable_integrator = true;
    }
    /* 在以下情况下强制使用着陆俯仰：
       - 拉平开关高
       - 油门杆在零推力位置
       - 在固定翼非自动油门模式
    */
    if (!quadplane_in_frwd_transition &&
        !control_mode->is_vtol_mode() &&
        !control_mode->does_auto_throttle() &&
        flare_mode == FlareMode::ENABLED_PITCH_TARGET &&
        throttle_at_zero()) {
        demanded_pitch = landing.get_pitch_cd();
    }

    return pitchController.get_servo_out(demanded_pitch - ahrs.pitch_sensor, speed_scaler, disable_integrator,
                                         ground_mode && !(plane.flight_option_enabled(FlightOptions::DISABLE_GROUND_PID_SUPPRESSION)));
}

/*
  这给予用户在稳定模式下对飞机的控制，仅在稳定模式下使用
  未来将移至mode_stabilize.cpp
 */
void ModeStabilize::stabilize_stick_mixing_direct()
{
    if (!plane.stick_mixing_enabled()) {
        return;
    }
#if HAL_QUADPLANE_ENABLED
    if (!plane.quadplane.allow_stick_mixing()) {
        return;
    }
#endif
    float aileron = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
    aileron = plane.channel_roll->stick_mixing(aileron);
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, aileron);

    float elevator = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
    elevator = plane.channel_pitch->stick_mixing(elevator);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, elevator);
}

/*
  这给予用户在稳定模式下使用FBW风格控制对飞机的控制
 */
void Plane::stabilize_stick_mixing_fbw()
{
    if (!stick_mixing_enabled() ||
        control_mode == &mode_acro ||
        control_mode == &mode_fbwa ||
        control_mode == &mode_autotune ||
        control_mode == &mode_fbwb ||
        control_mode == &mode_cruise ||
#if HAL_QUADPLANE_ENABLED
        control_mode == &mode_qstabilize ||
        control_mode == &mode_qhover ||
        control_mode == &mode_qloiter ||
        control_mode == &mode_qland ||
        control_mode == &mode_qacro ||
#if QAUTOTUNE_ENABLED
        control_mode == &mode_qautotune ||
#endif
        !quadplane.allow_stick_mixing() ||
#endif  // HAL_QUADPLANE_ENABLED
        control_mode == &mode_training) {
        return;
    }
    // 进行FBW风格的摇杆混控。然而，我们不是线性处理的。
    // 对于最大输入的一半以内的输入，我们使用线性
    // 加到nav_roll和nav_pitch上。超过那个点就变成
    // 非线性，最终达到最大值的2倍，以确保用户
    // 可以通过摇杆混控将飞机指向任何方向。
    float roll_input = channel_roll->norm_input_dz();
    if (roll_input > 0.5f) {
        roll_input = (3*roll_input - 1);
    } else if (roll_input < -0.5f) {
        roll_input = (3*roll_input + 1);
    }
    nav_roll_cd += roll_input * roll_limit_cd;
    nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);

    if ((control_mode == &mode_loiter) && (plane.flight_option_enabled(FlightOptions::ENABLE_LOITER_ALT_CONTROL))) {
        // loiter is using altitude control based on the pitch stick, don't use it again here
        return;
    }

    float pitch_input = channel_pitch->norm_input_dz();
    if (pitch_input > 0.5f) {
        pitch_input = (3*pitch_input - 1);
    } else if (pitch_input < -0.5f) {
        pitch_input = (3*pitch_input + 1);
    }
    if (fly_inverted()) {
        pitch_input = -pitch_input;
    }
    if (pitch_input > 0) {
        nav_pitch_cd += pitch_input * aparm.pitch_limit_max*100;
    } else {
        nav_pitch_cd += -(pitch_input * pitch_limit_min*100);
    }
    nav_pitch_cd = constrain_int32(nav_pitch_cd, pitch_limit_min*100, aparm.pitch_limit_max.get()*100);
}
/*
  稳定偏航轴。有3种操作模式：

    - 使用地面转向保持特定航向
    - 使用地面转向进行速率控制
    - 协调飞行的偏航控制    
 */
void Plane::stabilize_yaw()
{
    bool ground_steering = false;
    if (landing.is_flaring()) {
        // 在拉平时启用地面转向
        ground_steering = true;
    } else {
        // 否则，当没有输入控制且我们低于GROUND_STEER_ALT时使用地面转向
        ground_steering = (channel_roll->get_control_in() == 0 && 
                                            fabsf(relative_altitude) < g.ground_steer_alt);
        if (!landing.is_ground_steering_allowed()) {
            // 在着陆进近时不使用地面转向
            ground_steering = false;
        }
    }

    /*
      首先计算机头或尾轮的转向。
      在执行拉平（当机翼保持水平时）或在FBWA模式下保持航向
      （当我们低于GROUND_STEER_ALT时）时，我们对方向舵使用"航向保持"模式
     */
    float steering_output = 0.0;
    if (landing.is_flaring() ||
        (steer_state.hold_course_cd != -1 && ground_steering)) {
        steering_output = calc_nav_yaw_course();
    } else if (ground_steering) {
        steering_output = calc_nav_yaw_ground();
    }

    // 现在计算方向舵的输出
    const float rudder_output = calc_nav_yaw_coordinated();

    if (!ground_steering) {
        // 不进行地面转向时，在转向通道上输出方向舵
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rudder_output);
        SRV_Channels::set_output_scaled(SRV_Channel::k_steering, rudder_output);
    } else if (!SRV_Channels::function_assigned(SRV_Channel::k_steering)) {
        // 地面转向激活但没有配置转向输出，在方向舵通道上输出转向
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, steering_output);
        SRV_Channels::set_output_scaled(SRV_Channel::k_steering, steering_output);
    } else {
        // 同时使用转向和方向舵通道进行地面转向
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rudder_output);
        SRV_Channels::set_output_scaled(SRV_Channel::k_steering, steering_output);
    }
}

/*
  所有3个轴的主要稳定函数
 */
void Plane::stabilize()
{
    uint32_t now = AP_HAL::millis();
#if HAL_QUADPLANE_ENABLED
    if (quadplane.available()) {
        quadplane.transition->set_FW_roll_pitch(nav_pitch_cd, nav_roll_cd);
    }
#endif

    if (now - last_stabilize_ms > 2000) {
        // 如果我们2秒内没有运行速率控制器，则重置控制器
        control_mode->reset_controllers();
    }
    last_stabilize_ms = now;

    if (control_mode == &mode_training ||
            control_mode == &mode_manual) {
        plane.control_mode->run();
#if AP_SCRIPTING_ENABLED
    } else if (nav_scripting_active()) {
        // 脚本控制滚转和俯仰速率以及油门
        const float speed_scaler = get_speed_scaler();
        const float aileron = rollController.get_rate_out(nav_scripting.roll_rate_dps, speed_scaler);
        const float elevator = pitchController.get_rate_out(nav_scripting.pitch_rate_dps, speed_scaler);
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, aileron);
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, elevator);
        float rudder = 0;
        if (yawController.rate_control_enabled()) {
            rudder = nav_scripting.rudder_offset_pct * 45;
            if (nav_scripting.run_yaw_rate_controller) {
                rudder += yawController.get_rate_out(nav_scripting.yaw_rate_dps, speed_scaler, false);
            } else {
                yawController.reset_I();
            }
        }
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rudder);
        SRV_Channels::set_output_scaled(SRV_Channel::k_steering, rudder);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.nav_scripting.throttle_pct);
#endif
    } else {
        plane.control_mode->run();
    }

    /*
      检查是否应该将姿态控制器积分器归零
     */
    if (is_zero(get_throttle_input()) &&
        fabsf(relative_altitude) < 5.0f && 
        fabsf(barometer.get_climb_rate()) < 0.5f &&
        ahrs.groundspeed() < 3) {
        // 我们处于低高度，没有爬升率，零油门，
        // 且地速非常低。将姿态控制器积分器归零。
        // 这可以防止起飞前积分器累积。
        rollController.reset_I();
        pitchController.reset_I();
        yawController.reset_I();

        // 如果移动非常缓慢，也将转向积分器归零
        if (ahrs.groundspeed() < 1) {
            steerController.reset_I();            
        }
    }
}

/*
 * 设置油门输出。
 * 这被启用TECS的飞行模式调用，例如AUTO，GUIDED等。
*/
void Plane::calc_throttle()
{
    if (aparm.throttle_cruise <= 1) {
        // 用户要求零油门 - 这可能是由一个
        // 想要关闭发动机进行降落伞着陆的任务完成的
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        return;
    }

    // 读取TECS油门输出并将其设置到油门通道。
    float commanded_throttle = TECS_controller.get_throttle_demand();
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, commanded_throttle);
}

/*****************************************
* 计算所需的滚转/俯仰/偏航角度（在中频循环中）
*****************************************/

/*
  计算协调飞行的偏航控制
 */
int16_t Plane::calc_nav_yaw_coordinated()
{
    const float speed_scaler = get_speed_scaler();
    bool disable_integrator = false;
    int16_t rudder_in = rudder_input();

    int16_t commanded_rudder;
    bool using_rate_controller = false;

    // 在过去3秒内是否收到引导偏航的外部消息？
    if (control_mode->is_guided_mode() &&
            plane.guided_state.last_forced_rpy_ms.z > 0 &&
            millis() - plane.guided_state.last_forced_rpy_ms.z < 3000) {
        commanded_rudder = plane.guided_state.forced_rpy_cd.z;
    } else if (autotuning && g.acro_yaw_rate > 0 && yawController.rate_control_enabled()) {
        // 用户正在进行带有偏航速率控制的AUTOTUNE
        const float rudd_expo = rudder_in_expo(true);
        const float yaw_rate = (rudd_expo/SERVO_MAX) * g.acro_yaw_rate;
        // 加入协调转弯的偏航速率，使调整偏航速率控制器时更容易飞行
        const float coordination_yaw_rate = degrees(GRAVITY_MSS * tanf(radians(nav_roll_cd*0.01f))/MAX(aparm.airspeed_min,smoothed_airspeed));
        commanded_rudder = yawController.get_rate_out(yaw_rate+coordination_yaw_rate,  speed_scaler, false);
        using_rate_controller = true;
    } else {
        if (control_mode == &mode_stabilize && rudder_in != 0) {
            disable_integrator = true;
        }

        commanded_rudder = yawController.get_servo_out(speed_scaler, disable_integrator);

        // 加入来自滚转的方向舵混合
        commanded_rudder += SRV_Channels::get_output_scaled(SRV_Channel::k_aileron) * g.kff_rudder_mix;
        commanded_rudder += rudder_in;
    }

    if (!using_rate_controller) {
        /*
          当不运行偏航速率控制器时，我们需要重置速率
        */
        yawController.reset_rate_PID();
    }

    return constrain_int16(commanded_rudder, -4500, 4500);
}
/*
  计算地面转向的特定航向的偏航控制
 */
int16_t Plane::calc_nav_yaw_course(void)
{
    // 在地面上保持特定的导航航向。用于自动起飞和着陆
    int32_t bearing_error_cd = nav_controller->bearing_error_cd();
    int16_t steering = steerController.get_steering_out_angle_error(bearing_error_cd);
    if (stick_mixing_enabled()) {
        steering = channel_rudder->stick_mixing(steering);
    }
    return constrain_int16(steering, -4500, 4500);
}

/*
  计算地面转向的偏航控制
 */
int16_t Plane::calc_nav_yaw_ground(void)
{
    if (gps.ground_speed() < 1 && 
        is_zero(get_throttle_input()) &&
        flight_stage != AP_FixedWing::FlightStage::TAKEOFF &&
        flight_stage != AP_FixedWing::FlightStage::ABORT_LANDING) {
        // 静止时的手动方向舵控制
        steer_state.locked_course = false;
        steer_state.locked_course_err = 0;
        return rudder_input();
    }

    // 如果1秒内没有转向，则清除锁定航向
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - steer_state.last_steer_ms > 1000) {
        steer_state.locked_course = false;
    }
    steer_state.last_steer_ms = now_ms;

    float steer_rate = (rudder_input()/4500.0f) * g.ground_steer_dps;
    if (flight_stage == AP_FixedWing::FlightStage::TAKEOFF ||
        flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING) {
        steer_rate = 0;
    }
    if (!is_zero(steer_rate)) {
        // 飞行员正在给出方向舵输入
        steer_state.locked_course = false;        
    } else if (!steer_state.locked_course) {
        // 飞行员已释放方向舵摇杆或我们静止 - 锁定航向
        steer_state.locked_course = true;
        if (flight_stage != AP_FixedWing::FlightStage::TAKEOFF &&
            flight_stage != AP_FixedWing::FlightStage::ABORT_LANDING) {
            steer_state.locked_course_err = 0;
        }
    }

    int16_t steering;
    if (!steer_state.locked_course) {
        // 使用飞行员指定速率的速率控制器
        steering = steerController.get_steering_out_rate(steer_rate);
    } else {
        // 对累积误差使用误差控制器
        int32_t yaw_error_cd = -ToDeg(steer_state.locked_course_err)*100;
        steering = steerController.get_steering_out_angle_error(yaw_error_cd);
    }
    return constrain_int16(steering, -4500, 4500);
}

/*
  从速度高度控制器计算新的nav_pitch_cd
 */
void Plane::calc_nav_pitch()
{
    int32_t commanded_pitch = TECS_controller.get_pitch_demand();
    nav_pitch_cd = constrain_int32(commanded_pitch, pitch_limit_min*100, aparm.pitch_limit_max.get()*100);
}

/*
  从导航控制器计算新的nav_roll_cd
 */
void Plane::calc_nav_roll()
{
    int32_t commanded_roll = nav_controller->nav_roll_cd();
    nav_roll_cd = constrain_int32(commanded_roll, -roll_limit_cd, roll_limit_cd);
    update_load_factor();
}

/*
  调整nav_pitch_cd以适应STAB_PITCH_DOWN_CD。这用于使FBWA模式下保持良好空速更容易，
  因为飞机在低油门时会自动略微俯冲。这使得FBWA着陆不失速变得更容易。
 */
void Plane::adjust_nav_pitch_throttle(void)
{
    int8_t throttle = throttle_percentage();
    if (throttle >= 0 && throttle < aparm.throttle_cruise && flight_stage != AP_FixedWing::FlightStage::VTOL) {
        float p = (aparm.throttle_cruise - throttle) / (float)aparm.throttle_cruise;
        nav_pitch_cd -= g.stab_pitch_down * 100.0f * p;
    }
}

/*
  计算新的空气动力载荷系数并限制nav_roll_cd，
  以确保载荷系数不会使我们低于可持续空速
 */
void Plane::update_load_factor(void)
{
    float demanded_roll = fabsf(nav_roll_cd*0.01f);
    if (demanded_roll > 85) {
        // 限制为85度以防止数值错误
        demanded_roll = 85;
    }
    aerodynamic_load_factor = 1.0f / safe_sqrt(cosf(radians(demanded_roll)));

#if HAL_QUADPLANE_ENABLED
    if (quadplane.available() && quadplane.transition->set_FW_roll_limit(roll_limit_cd)) {
        nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);
        return;
    }
#endif

    if (!aparm.stall_prevention) {
        // 失速预防已禁用
        return;
    }
    if (fly_inverted()) {
        // 倒飞时没有滚转限制
        return;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.tailsitter.active()) {
        // 悬停时没有限制
        return;
    }
#endif

    float max_load_factor = smoothed_airspeed / MAX(aparm.airspeed_min, 1);
    if (max_load_factor <= 1) {
        // 我们的空速低于最小空速。将滚转限制在25度
        nav_roll_cd = constrain_int32(nav_roll_cd, -2500, 2500);
        roll_limit_cd = MIN(roll_limit_cd, 2500);
    } else if (max_load_factor < aerodynamic_load_factor) {
        // 要求的nav_roll会超过空气动力载荷限制。将我们的滚转限制在一个能保持载荷
        // 在机身可承受范围内的倾斜角度。然而，我们始终允许至少25度的滚转，
        // 以确保在空速估计不准确的情况下飞机仍可机动。在25度时，载荷系数为1.1（10%）
        int32_t roll_limit = degrees(acosf(sq(1.0f / max_load_factor)))*100;
        if (roll_limit < 2500) {
            roll_limit = 2500;
        }
        nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit, roll_limit);
        roll_limit_cd = MIN(roll_limit_cd, roll_limit);
    }    
}
