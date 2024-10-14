#include "Copter.h"

#if MODE_FLIP_ENABLED

/*
 * 翻转飞行模式的初始化和运行调用
 *      原始实现由Jose Julio在2010年完成
 *      由Jason Short在2011年为AC2进行了适配和更新
 *
 *      控制:
 *          RC7_OPTION - RC12_OPTION参数必须设置为"Flip"(AUXSW_FLIP),即"2"
 *          飞行员切换到Stabilize、Acro或AltHold飞行模式,并将ch7/ch8开关置于ON位置
 *          默认情况下,飞行器将向右翻滚,但如果横滚或俯仰摇杆稍微向左、前或后倾斜,它将朝那个方向翻转
 *          飞行器应在2.5秒内完成翻滚,然后返回到触发翻转前的原始飞行模式
 *          飞行员可以通过关闭ch7/ch8或将横滚摇杆向左或向右移动>40度来手动退出翻转
 *
 *      状态机方法:
 *          FlipState::Start (当飞行器倾斜<45度时) : 以400度/秒的速度向右翻滚,增加油门
 *          FlipState::Roll (当飞行器在+45度~-90度之间) : 以400度/秒的速度向右翻滚,减少油门
 *          FlipState::Recover (当飞行器在-90度和原始目标角度之间) : 使用地球坐标系角度控制器将飞行器恢复到原始姿态
 */

#define FLIP_THR_INC        0.20f   // FlipState::Start阶段的油门增加量(45度倾斜角度以下)
#define FLIP_THR_DEC        0.24f   // FlipState::Roll阶段的油门减少量(45度~-90度翻滚之间)
#define FLIP_ROTATION_RATE  40000   // 旋转速率请求,单位为百分之一度/秒(即400度/秒)
#define FLIP_TIMEOUT_MS     2500    // 2.5秒后超时。飞行器将切换回原始飞行模式
#define FLIP_RECOVERY_ANGLE 500     // 当翻滚角度回到原始角度的5度以内时,认为恢复成功

#define FLIP_ROLL_RIGHT      1      // 用于设置flip_dir,表示向右翻滚
#define FLIP_ROLL_LEFT      -1      // 用于设置flip_dir,表示向左翻滚

#define FLIP_PITCH_BACK      1      // 用于设置flip_dir,表示向后俯仰
#define FLIP_PITCH_FORWARD  -1      // 用于设置flip_dir,表示向前俯仰

// flip_init - 初始化翻转控制器
bool ModeFlip::init(bool ignore_checks)
{
    // 只允许从某些飞行模式进行翻转,例如ACRO、Stabilize、AltHold或FlowHold飞行模式
    if (!copter.flightmode->allows_flip()) {
        return false;
    }

    // 如果在acro或stabilize模式下,确保油门高于零
    if (copter.ap.throttle_zero && (copter.flightmode->mode_number() == Mode::Number::ACRO || copter.flightmode->mode_number() == Mode::Number::STABILIZE)) {
        return false;
    }

    // 确保横滚输入小于40度
    if (abs(channel_roll->get_control_in()) >= 4000) {
        return false;
    }

    // 只有在飞行时才允许翻转
    if (!motors->armed() || copter.ap.land_complete) {
        return false;
    }

    // 捕获原始飞行模式,以便在完成后返回
    orig_control_mode = copter.flightmode->mode_number();

    // 初始化状态
    _state = FlipState::Start;
    start_time_ms = millis();

    roll_dir = pitch_dir = 0;

    // 根据飞行员的横滚和俯仰摇杆选择方向
    if (channel_pitch->get_control_in() > 300) {
        pitch_dir = FLIP_PITCH_BACK;
    } else if (channel_pitch->get_control_in() < -300) {
        pitch_dir = FLIP_PITCH_FORWARD;
    } else if (channel_roll->get_control_in() >= 0) {
        roll_dir = FLIP_ROLL_RIGHT;
    } else {
        roll_dir = FLIP_ROLL_LEFT;
    }

    // 记录翻转开始
    LOGGER_WRITE_EVENT(LogEvent::FLIP_START);

    // 捕获当前姿态,将在FlipState::Recovery阶段使用
    const float angle_max = copter.aparm.angle_max;
    orig_attitude.x = constrain_float(ahrs.roll_sensor, -angle_max, angle_max);
    orig_attitude.y = constrain_float(ahrs.pitch_sensor, -angle_max, angle_max);
    orig_attitude.z = ahrs.yaw_sensor;

    return true;
}

// run - 运行翻转控制器
// 应该以100Hz或更高的频率调用
void ModeFlip::run()
{
    // 如果飞行员输入横滚>40度或超时,则放弃翻转
    if (!motors->armed() || (abs(channel_roll->get_control_in()) >= 4000) || (abs(channel_pitch->get_control_in()) >= 4000) || ((millis() - start_time_ms) > FLIP_TIMEOUT_MS)) {
        _state = FlipState::Abandon;
    }

    // 获取飞行员期望的油门
    float throttle_out = get_pilot_desired_throttle();

    // 将电机设置为全范围
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // 根据旋转方向和轴获取校正后的角度
    // 我们翻转flip_angle的符号以最小化代码重复
    int32_t flip_angle;

    if (roll_dir != 0) {
        flip_angle = ahrs.roll_sensor * roll_dir;
    } else {
        flip_angle = ahrs.pitch_sensor * pitch_dir;
    }

    // 状态机
    switch (_state) {

    case FlipState::Start:
        // 45度以下请求400度/秒的横滚或俯仰
        attitude_control->input_rate_bf_roll_pitch_yaw(FLIP_ROTATION_RATE * roll_dir, FLIP_ROTATION_RATE * pitch_dir, 0.0);

        // 增加油门
        throttle_out += FLIP_THR_INC;

        // 超过45度倾斜角度进入下一阶段
        if (flip_angle >= 4500) {
            if (roll_dir != 0) {
                // 我们正在翻滚
            _state = FlipState::Roll;
            } else {
                // 我们正在俯仰
                _state = FlipState::Pitch_A;
        }
        }
        break;

    case FlipState::Roll:
        // 在45度~-90度之间请求400度/秒的横滚
        attitude_control->input_rate_bf_roll_pitch_yaw(FLIP_ROTATION_RATE * roll_dir, 0.0, 0.0);
        // 减少油门
        throttle_out = MAX(throttle_out - FLIP_THR_DEC, 0.0f);

        // 超过-90度进入恢复阶段
        if ((flip_angle < 4500) && (flip_angle > -9000)) {
            _state = FlipState::Recover;
        }
        break;

    case FlipState::Pitch_A:
        // 在45度~-90度之间请求400度/秒的俯仰
        attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, FLIP_ROTATION_RATE * pitch_dir, 0.0);
        // 减少油门
        throttle_out = MAX(throttle_out - FLIP_THR_DEC, 0.0f);

        // 检查横滚是否倒置
        if ((labs(ahrs.roll_sensor) > 9000) && (flip_angle > 4500)) {
            _state = FlipState::Pitch_B;
        }
        break;

    case FlipState::Pitch_B:
        // 在45度~-90度之间请求400度/秒的俯仰
        attitude_control->input_rate_bf_roll_pitch_yaw(0.0, FLIP_ROTATION_RATE * pitch_dir, 0.0);
        // 减少油门
        throttle_out = MAX(throttle_out - FLIP_THR_DEC, 0.0f);

        // 检查横滚是否恢复
        if ((labs(ahrs.roll_sensor) < 9000) && (flip_angle > -4500)) {
            _state = FlipState::Recover;
        }
        break;

    case FlipState::Recover: {
        // 使用最初捕获的地球坐标系角度目标进行恢复
        attitude_control->input_euler_angle_roll_pitch_yaw(orig_attitude.x, orig_attitude.y, orig_attitude.z, false);

        // 增加油门以恢复任何失去的高度
        throttle_out += FLIP_THR_INC;

        float recovery_angle;
        if (roll_dir != 0) {
            // 我们正在翻滚
            recovery_angle = fabsf(orig_attitude.x - (float)ahrs.roll_sensor);
        } else {
            // 我们正在俯仰
            recovery_angle = fabsf(orig_attitude.y - (float)ahrs.pitch_sensor);
        }

        // 检查是否成功恢复
        if (fabsf(recovery_angle) <= FLIP_RECOVERY_ANGLE) {
            // 恢复原始飞行模式
            if (!copter.set_mode(orig_control_mode, ModeReason::FLIP_COMPLETE)) {
                // 这种情况不应该发生,但以防万一
                copter.set_mode(Mode::Number::STABILIZE, ModeReason::UNKNOWN);
            }
            // 记录成功完成
            LOGGER_WRITE_EVENT(LogEvent::FLIP_END);
        }
        break;

    }
    case FlipState::Abandon:
        // 恢复原始飞行模式
        if (!copter.set_mode(orig_control_mode, ModeReason::FLIP_COMPLETE)) {
            // 这种情况不应该发生,但以防万一
            copter.set_mode(Mode::Number::STABILIZE, ModeReason::UNKNOWN);
        }
        // 记录放弃翻转
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIP, LogErrorCode::FLIP_ABANDONED);
        break;
    }

    // 输出飞行员的油门,不带角度增益
    attitude_control->set_throttle_out(throttle_out, false, g.throttle_filt);
}

#endif
