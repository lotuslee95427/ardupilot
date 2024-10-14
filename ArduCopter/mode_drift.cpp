#include "Copter.h"

#if MODE_DRIFT_ENABLED

/*
 * Init and run calls for drift flight mode
 * 漂移飞行模式的初始化和运行调用
 */

#ifndef DRIFT_SPEEDGAIN
 # define DRIFT_SPEEDGAIN 8.0f  // 漂移速度增益
#endif
#ifndef DRIFT_SPEEDLIMIT
 # define DRIFT_SPEEDLIMIT 560.0f  // 漂移速度限制
#endif

#ifndef DRIFT_THR_ASSIST_GAIN
 # define DRIFT_THR_ASSIST_GAIN 0.0018f    // gain controlling amount of throttle assistance
                                           // 控制油门辅助量的增益
#endif

#ifndef DRIFT_THR_ASSIST_MAX
 # define DRIFT_THR_ASSIST_MAX  0.3f    // maximum assistance throttle assist will provide
                                        // 油门辅助提供的最大辅助量
#endif

#ifndef DRIFT_THR_MIN
 # define DRIFT_THR_MIN         0.213f  // throttle assist will be active when pilot's throttle is above this value
                                        // 当飞行员的油门高于此值时，油门辅助将激活
#endif
#ifndef DRIFT_THR_MAX
 # define DRIFT_THR_MAX         0.787f  // throttle assist will be active when pilot's throttle is below this value
                                        // 当飞行员的油门低于此值时，油门辅助将激活
#endif

// drift_init - initialise drift controller
// drift_init - 初始化漂移控制器
bool ModeDrift::init(bool ignore_checks)
{
    return true;
}

// drift_run - runs the drift controller
// should be called at 100hz or more
// drift_run - 运行漂移控制器
// 应该以100Hz或更高的频率调用
void ModeDrift::run()
{
    static float braker = 0.0f;  // 制动器
    static float roll_input = 0.0f;  // 横滚输入

    // convert pilot input to lean angles
    // 将飞行员输入转换为倾斜角度
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // Grab inertial velocity
    // 获取惯性速度
    const Vector3f& vel = inertial_nav.get_velocity_neu_cms();

    // rotate roll, pitch input from north facing to vehicle's perspective
    // 将横滚、俯仰输入从北向旋转到飞行器视角
    float roll_vel =  vel.y * ahrs.cos_yaw() - vel.x * ahrs.sin_yaw(); // body roll vel 机体横滚速度
    float pitch_vel = vel.y * ahrs.sin_yaw() + vel.x * ahrs.cos_yaw(); // body pitch vel 机体俯仰速度

    // gain scheduling for yaw
    // 偏航的增益调度
    float pitch_vel2 = MIN(fabsf(pitch_vel), 2000);
    float target_yaw_rate = target_roll * (1.0f - (pitch_vel2 / 5000.0f)) * g2.command_model_acro_y.get_rate() / 45.0;

    roll_vel = constrain_float(roll_vel, -DRIFT_SPEEDLIMIT, DRIFT_SPEEDLIMIT);
    pitch_vel = constrain_float(pitch_vel, -DRIFT_SPEEDLIMIT, DRIFT_SPEEDLIMIT);

    roll_input = roll_input * .96f + (float)channel_yaw->get_control_in() * .04f;

    // convert user input into desired roll velocity
    // 将用户输入转换为期望的横滚速度
    float roll_vel_error = roll_vel - (roll_input / DRIFT_SPEEDGAIN);

    // roll velocity is feed into roll acceleration to minimize slip
    // 将横滚速度反馈到横滚加速度以最小化滑移
    target_roll = roll_vel_error * -DRIFT_SPEEDGAIN;
    target_roll = constrain_float(target_roll, -4500.0f, 4500.0f);

    // If we let go of sticks, bring us to a stop
    // 如果我们松开摇杆，使飞行器停止
    if (is_zero(target_pitch)) {
        // .14/ (.03 * 100) = 4.6 seconds till full braking
        // .14 / (.03 * 100) = 4.6秒直到完全制动
        braker += .03f;
        braker = MIN(braker, DRIFT_SPEEDGAIN);
        target_pitch = pitch_vel * braker;
    } else {
        braker = 0.0f;
    }

    if (!motors->armed()) {
        // Motors should be Stopped
        // 电机应该停止
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero) {
        // Attempting to Land
        // 尝试着陆
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        // 电机停止
        attitude_control->reset_yaw_target_and_rate(false);
        attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        // 已着陆
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        // 在油门大于零时清除着陆标志
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        // 不做任何操作
        break;
    }

    // call attitude controller
    // 调用姿态控制器
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle with angle boost
    // 输出飞行员的油门，并带有角度增益
    const float assisted_throttle = get_throttle_assist(vel.z, get_pilot_desired_throttle());
    attitude_control->set_throttle_out(assisted_throttle, true, g.throttle_filt);
}

// get_throttle_assist - return throttle output (range 0 ~ 1) based on pilot input and z-axis velocity
// get_throttle_assist - 根据飞行员输入和z轴速度返回油门输出（范围0~1）
float ModeDrift::get_throttle_assist(float velz, float pilot_throttle_scaled)
{
    // throttle assist - adjusts throttle to slow the vehicle's vertical velocity
    //      Only active when pilot's throttle is between 213 ~ 787
    //      Assistance is strongest when throttle is at mid, drops linearly to no assistance at 213 and 787
    // 油门辅助 - 调整油门以减缓飞行器的垂直速度
    //      仅在飞行员的油门在213~787之间时激活
    //      当油门在中间位置时辅助最强，在213和787处线性下降到无辅助
    float thr_assist = 0.0f;
    if (pilot_throttle_scaled > DRIFT_THR_MIN && pilot_throttle_scaled < DRIFT_THR_MAX) {
        // calculate throttle assist gain
        // 计算油门辅助增益
        thr_assist = 1.2f - ((float)fabsf(pilot_throttle_scaled - 0.5f) / 0.24f);
        thr_assist = constrain_float(thr_assist, 0.0f, 1.0f) * -DRIFT_THR_ASSIST_GAIN * velz;

        // ensure throttle assist never adjusts the throttle by more than 300 pwm
        // 确保油门辅助不会将油门调整超过300 pwm
        thr_assist = constrain_float(thr_assist, -DRIFT_THR_ASSIST_MAX, DRIFT_THR_ASSIST_MAX);
    }
    
    return constrain_float(pilot_throttle_scaled + thr_assist, 0.0f, 1.0f);
}
#endif
