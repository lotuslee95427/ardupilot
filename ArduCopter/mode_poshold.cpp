#include "Copter.h"

#if MODE_POSHOLD_ENABLED

/*
 * Init and run calls for PosHold flight mode
 *     PosHold tries to improve upon regular loiter by mixing the pilot input with the loiter controller
 *     PosHold 试图通过将飞行员输入与悬停控制器混合来改进常规悬停
 */

#define POSHOLD_SPEED_0                         10      // speed below which it is always safe to switch to loiter
                                                        // 低于此速度时，切换到悬停模式总是安全的

// 400hz loop update rate
// 400hz 循环更新率
#define POSHOLD_BRAKE_TIME_ESTIMATE_MAX         (600*4) // max number of cycles the brake will be applied before we switch to loiter
                                                        // 在切换到悬停模式之前，刹车将应用的最大循环次数
#define POSHOLD_BRAKE_TO_LOITER_TIMER           (150*4) // Number of cycles to transition from brake mode to loiter mode.  Must be lower than POSHOLD_LOITER_STAB_TIMER
                                                        // 从刹车模式过渡到悬停模式的循环次数。必须低于 POSHOLD_LOITER_STAB_TIMER
#define POSHOLD_WIND_COMP_START_TIMER           (150*4) // Number of cycles to start wind compensation update after loiter is engaged
                                                        // 悬停启动后开始风补偿更新的循环次数
#define POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER   (50*4)  // Set it from 100 to 200, the number of centiseconds loiter and manual commands are mixed to make a smooth transition.
                                                        // 设置为 100 到 200，悬停和手动命令混合的百分秒数，以实现平滑过渡
#define POSHOLD_SMOOTH_RATE_FACTOR              0.0125f // filter applied to pilot's roll/pitch input as it returns to center.  A lower number will cause the roll/pitch to return to zero more slowly if the brake_rate is also low.
                                                        // 滤波器应用于飞行员的横滚/俯仰输入，当其返回到中心时。如果刹车率也较低，则较低的数字会导致横滚/俯仰更慢地返回到零
#define POSHOLD_WIND_COMP_TIMER_10HZ            40      // counter value used to reduce wind compensation to 10hz
                                                        // 用于将风补偿减少到 10hz 的计数器值
#define LOOP_RATE_FACTOR                        4       // used to adapt PosHold params to loop_rate
                                                        // 用于将 PosHold 参数适应循环速率
#define TC_WIND_COMP                            0.0025f // Time constant for poshold_update_wind_comp_estimate()
                                                        // poshold_update_wind_comp_estimate() 的时间常数

// definitions that are independent of main loop rate
// 独立于主循环速率的定义
#define POSHOLD_STICK_RELEASE_SMOOTH_ANGLE      1800    // max angle required (in centi-degrees) after which the smooth stick release effect is applied
                                                        // 需要的最大角度（以百分度为单位），之后应用平滑摇杆释放效果
#define POSHOLD_WIND_COMP_ESTIMATE_SPEED_MAX    10      // wind compensation estimates will only run when velocity is at or below this speed in cm/s
                                                        // 风补偿估计仅在速度为 cm/s 或更低时运行
#define POSHOLD_WIND_COMP_LEAN_PCT_MAX          0.6666f // wind compensation no more than 2/3rds of angle max to ensure pilot can always override
                                                        // 风补偿不超过最大角度的 2/3，以确保飞行员始终可以覆盖

// poshold_init - initialise PosHold controller
// poshold_init - 初始化 PosHold 控制器
bool ModePosHold::init(bool ignore_checks)
{
    // set vertical speed and acceleration limits
    // 设置垂直速度和加速度限制
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise the vertical position controller
    // 初始化垂直位置控制器
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // initialise lean angles to current attitude
    // 将倾斜角度初始化为当前姿态
    pilot_roll = 0.0f;
    pilot_pitch = 0.0f;

    // compute brake_gain
    // 计算刹车增益
    brake.gain = (15.0f * (float)g.poshold_brake_rate + 95.0f) * 0.01f;

    if (copter.ap.land_complete) {
        // if landed begin in loiter mode
        // 如果着陆，则开始悬停模式
        roll_mode = RPMode::LOITER;
        pitch_mode = RPMode::LOITER;
    } else {
        // if not landed start in pilot override to avoid hard twitch
        // 如果未着陆，则以飞行员覆盖模式启动以避免剧烈抖动
        roll_mode = RPMode::PILOT_OVERRIDE;
        pitch_mode = RPMode::PILOT_OVERRIDE;
    }

    // initialise loiter
    // 初始化悬停
    loiter_nav->clear_pilot_desired_acceleration();
    loiter_nav->init_target();

    // initialise wind_comp each time PosHold is switched on
    // 每次切换到 PosHold 时初始化 wind_comp
    init_wind_comp_estimate();

    return true;
}

// poshold_run - runs the PosHold controller
// should be called at 100hz or more
// poshold_run - 运行 PosHold 控制器
// 应以 100hz 或更高频率调用
void ModePosHold::run()
{
    float controller_to_pilot_roll_mix; // mix of controller and pilot controls.  0 = fully last controller controls, 1 = fully pilot controls
                                        // 控制器和飞行员控制的混合。0 = 完全由最后一个控制器控制，1 = 完全由飞行员控制
    float controller_to_pilot_pitch_mix;    // mix of controller and pilot controls.  0 = fully last controller controls, 1 = fully pilot controls
                                            // 控制器和飞行员控制的混合。0 = 完全由最后一个控制器控制，1 = 完全由飞行员控制
    const Vector3f& vel = inertial_nav.get_velocity_neu_cms();

    // set vertical speed and acceleration limits
    // 设置垂直速度和加速度限制
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    loiter_nav->clear_pilot_desired_acceleration();

    // apply SIMPLE mode transform to pilot inputs
    // 将 SIMPLE 模式转换应用于飞行员输入
    update_simple_mode();

    // convert pilot input to lean angles
    // 将飞行员输入转换为倾斜角度
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max_cd());

    // get pilot's desired yaw rate
    // 获取飞行员期望的偏航率
    float target_yaw_rate = get_pilot_desired_yaw_rate();

    // get pilot desired climb rate (for alt-hold mode and take-off)
    // 获取飞行员期望的爬升率（用于高度保持模式和起飞）
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // relax loiter target if we might be landed
    // 如果我们可能着陆，则放松悬停目标
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Pos Hold State Machine Determination
    // Pos Hold 状态机确定
    AltHoldModeState poshold_state = get_alt_hold_state(target_climb_rate);

    // state machine
    // 状态机
    switch (poshold_state) {

    case AltHoldModeState::MotorStopped:
        // 电机停止
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
                                                 // 强制油门输出衰减到零
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();

        // set poshold state to pilot override
        // 将 poshold 状态设置为飞行员覆盖
        roll_mode = RPMode::PILOT_OVERRIDE;
        pitch_mode = RPMode::PILOT_OVERRIDE;

        // initialise wind compensation estimate
        // 初始化风补偿估计
        init_wind_comp_estimate();
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        // 着陆地面怠速
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();
        attitude_control->reset_yaw_target_and_rate();
        init_wind_comp_estimate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        // 着陆起飞前
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
                                                 // 强制油门输出衰减到零

        // set poshold state to pilot override
        // 将 poshold 状态设置为飞行员覆盖
        roll_mode = RPMode::PILOT_OVERRIDE;
        pitch_mode = RPMode::PILOT_OVERRIDE;
        break;

    case AltHoldModeState::Takeoff:
        // 起飞
        // initiate take-off
        // 启动起飞
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        // 获取避障调整后的爬升率
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // set position controller targets adjusted for pilot input
        // 设置根据飞行员输入调整的位置控制器目标
        takeoff.do_pilot_takeoff(target_climb_rate);

        // init and update loiter although pilot is controlling lean angles
        // 初始化并更新悬停，尽管飞行员正在控制倾斜角度
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();

        // set poshold state to pilot override
        // 将 poshold 状态设置为飞行员覆盖
        roll_mode = RPMode::PILOT_OVERRIDE;
        pitch_mode = RPMode::PILOT_OVERRIDE;
        break;

    case AltHoldModeState::Flying:
        // 飞行
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // get avoidance adjusted climb rate
        // 获取避障调整后的爬升率
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

#if AP_RANGEFINDER_ENABLED
        // update the vertical offset based on the surface measurement
        // 根据表面测量更新垂直偏移
        copter.surface_tracking.update_surface_offset();
#endif

        // Send the commanded climb rate to the position controller
        // 将命令的爬升率发送到位置控制器
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        break;
    }

    // poshold specific behaviour to calculate desired roll, pitch angles
    // poshold 特定行为以计算所需的横滚、俯仰角度
    // convert inertial nav earth-frame velocities to body-frame
    // 将惯性导航地球框架速度转换为机体框架
    // To-Do: move this to AP_Math (or perhaps we already have a function to do this)
    // 待办事项：将其移动到 AP_Math（或者我们可能已经有一个函数来执行此操作）
    float vel_fw = vel.x*ahrs.cos_yaw() + vel.y*ahrs.sin_yaw();
    float vel_right = -vel.x*ahrs.sin_yaw() + vel.y*ahrs.cos_yaw();

    // If not in LOITER, retrieve latest wind compensation lean angles related to current yaw
    // 如果不在悬停模式，检索与当前偏航相关的最新风补偿倾斜角度
    if (roll_mode != RPMode::LOITER || pitch_mode != RPMode::LOITER) {
        get_wind_comp_lean_angles(wind_comp_roll, wind_comp_pitch);
    }

    // Roll state machine
    // 横滚状态机
    //  Each state (aka mode) is responsible for:
    //  每个状态（即模式）负责：
    //      1. dealing with pilot input
    //      1. 处理飞行员输入
    //      2. calculating the final roll output to the attitude controller
    //      2. 计算最终的横滚输出到姿态控制器
    //      3. checking if the state (aka mode) should be changed and if 'yes' perform any required initialisation for the new state
    //      3. 检查是否应更改状态（即模式），如果是，则执行新状态所需的任何初始化
    switch (roll_mode) {

        case RPMode::PILOT_OVERRIDE:
            // update pilot desired roll angle using latest radio input
            // 使用最新的无线电输入更新飞行员期望的横滚角度
            //  this filters the input so that it returns to zero no faster than the brake-rate
            //  这会过滤输入，使其返回到零的速度不超过刹车率
            update_pilot_lean_angle(pilot_roll, target_roll);

            // switch to BRAKE mode for next iteration if no pilot input
            // 如果没有飞行员输入，则在下一次迭代中切换到刹车模式
            if (is_zero(target_roll) && (fabsf(pilot_roll) < 2 * g.poshold_brake_rate)) {
                // initialise BRAKE mode
                // 初始化刹车模式
                roll_mode = RPMode::BRAKE;        // Set brake roll mode
                                                  // 设置刹车横滚模式
                brake.roll = 0.0f;                  // initialise braking angle to zero
                                                  // 将刹车角度初始化为零
                brake.angle_max_roll = 0.0f;        // reset brake_angle_max so we can detect when vehicle begins to flatten out during braking
                                                  // 重置 brake_angle_max 以便我们可以检测到车辆在刹车过程中何时开始变平
                brake.timeout_roll = POSHOLD_BRAKE_TIME_ESTIMATE_MAX; // number of cycles the brake will be applied, updated during braking mode.
                                                                      // 刹车将应用的循环次数，在刹车模式期间更新
                brake.time_updated_roll = false;   // flag the braking time can be re-estimated
                                                  // 标记刹车时间可以重新估算
            }

            // final lean angle should be pilot input plus wind compensation
            // 最终的倾斜角度应为飞行员输入加上风补偿
            roll = pilot_roll + wind_comp_roll;
            break;

        case RPMode::BRAKE:
        case RPMode::BRAKE_READY_TO_LOITER:
            // calculate brake.roll angle to counter-act velocity
            // 计算刹车横滚角度以抵消速度
            update_brake_angle_from_velocity(brake.roll, vel_right);

            // update braking time estimate
            // 更新刹车时间估算
            if (!brake.time_updated_roll) {
                // check if brake angle is increasing
                // 检查刹车角度是否在增加
                if (fabsf(brake.roll) >= brake.angle_max_roll) {
                    brake.angle_max_roll = fabsf(brake.roll);
                } else {
                    // braking angle has started decreasing so re-estimate braking time
                    // 刹车角度已开始减小，因此重新估算刹车时间
                    brake.timeout_roll = 1+(uint16_t)(LOOP_RATE_FACTOR*15L*(int32_t)(fabsf(brake.roll))/(10L*(int32_t)g.poshold_brake_rate));  // the 1.2 (12/10) factor has to be tuned in flight, here it means 120% of the "normal" time.
                                                                                                                                             // 1.2（12/10）因子必须在飞行中调整，这里表示“正常”时间的 120%
                    brake.time_updated_roll = true;
                }
            }

            // if velocity is very low reduce braking time to 0.5seconds
            // 如果速度非常低，将刹车时间减少到 0.5 秒
            if ((fabsf(vel_right) <= POSHOLD_SPEED_0) && (brake.timeout_roll > 50*LOOP_RATE_FACTOR)) {
                brake.timeout_roll = 50*LOOP_RATE_FACTOR;
            }

            // reduce braking timer
            // 减少刹车计时器
            if (brake.timeout_roll > 0) {
                brake.timeout_roll--;
            } else {
                // indicate that we are ready to move to Loiter.
                // 表示我们准备好进入悬停模式
                // Loiter will only actually be engaged once both roll_mode and pitch_mode are changed to RPMode::BRAKE_READY_TO_LOITER
                // 只有当 roll_mode 和 pitch_mode 都更改为 RPMode::BRAKE_READY_TO_LOITER 时，悬停才会实际启动
                //  logic for engaging loiter is handled below the roll and pitch mode switch statements
                //  启动悬停的逻辑在横滚和俯仰模式切换语句下处理
                roll_mode = RPMode::BRAKE_READY_TO_LOITER;
            }

            // final lean angle is braking angle + wind compensation angle
            // 最终的倾斜角度是刹车角度 + 风补偿角度
            roll = brake.roll + wind_comp_roll;

            // check for pilot input
            // 检查飞行员输入
            if (!is_zero(target_roll)) {
                // init transition to pilot override
                // 初始化过渡到飞行员覆盖
                roll_controller_to_pilot_override();
            }
            break;

        case RPMode::BRAKE_TO_LOITER:
        case RPMode::LOITER:
            // these modes are combined roll-pitch modes and are handled below
            // 这些模式是组合的横滚-俯仰模式，并在下面处理
            break;

        case RPMode::CONTROLLER_TO_PILOT_OVERRIDE:
            // update pilot desired roll angle using latest radio input
            // 使用最新的无线电输入更新飞行员期望的横滚角度
            //  this filters the input so that it returns to zero no faster than the brake-rate
            //  这会过滤输入，使其返回到零的速度不超过刹车率
            update_pilot_lean_angle(pilot_roll, target_roll);

            // count-down loiter to pilot timer
            // 倒计时悬停到飞行员计时器
            if (controller_to_pilot_timer_roll > 0) {
                controller_to_pilot_timer_roll--;
            } else {
                // when timer runs out switch to full pilot override for next iteration
                // 当计时器耗尽时，切换到完全飞行员覆盖以进行下一次迭代
                roll_mode = RPMode::PILOT_OVERRIDE;
            }

            // calculate controller_to_pilot mix ratio
            // 计算控制器到飞行员的混合比率
            controller_to_pilot_roll_mix = (float)controller_to_pilot_timer_roll / (float)POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;

            // mix final loiter lean angle and pilot desired lean angles
            // 混合最终的悬停倾斜角度和飞行员期望的倾斜角度
            roll = mix_controls(controller_to_pilot_roll_mix, controller_final_roll, pilot_roll + wind_comp_roll);
            break;
    }

    // Pitch state machine
    // 俯仰状态机
    //  Each state (aka mode) is responsible for:
    //  每个状态（即模式）负责：
    //      1. dealing with pilot input
    //      1. 处理飞行员输入
    //      2. calculating the final pitch output to the attitude contpitcher
    //      2. 计算姿态控制器的最终俯仰输出
    //      3. checking if the state (aka mode) should be changed and if 'yes' perform any required initialisation for the new state
    //      3. 检查是否应更改状态（即模式），如果是，则执行新状态所需的任何初始化
    switch (pitch_mode) {

        case RPMode::PILOT_OVERRIDE:
            // update pilot desired pitch angle using latest radio input
            // 使用最新的无线电输入更新飞行员期望的俯仰角度
            //  this filters the input so that it returns to zero no faster than the brake-rate
            //  这会过滤输入，使其返回到零的速度不超过刹车率
            update_pilot_lean_angle(pilot_pitch, target_pitch);

            // switch to BRAKE mode for next iteration if no pilot input
            // 如果没有飞行员输入，则在下一次迭代中切换到刹车模式
            if (is_zero(target_pitch) && (fabsf(pilot_pitch) < 2 * g.poshold_brake_rate)) {
                // initialise BRAKE mode
                // 初始化刹车模式
                pitch_mode = RPMode::BRAKE;       // set brake pitch mode
                                                   // 设置刹车俯仰模式
                brake.pitch = 0.0f;                 // initialise braking angle to zero
                                                   // 将刹车角度初始化为零
                brake.angle_max_pitch = 0.0f;       // reset brake_angle_max so we can detect when vehicle begins to flatten out during braking
                                                   // 重置 brake_angle_max，以便我们可以检测到车辆在刹车过程中何时开始变平
                brake.timeout_pitch = POSHOLD_BRAKE_TIME_ESTIMATE_MAX; // number of cycles the brake will be applied, updated during braking mode.
                                                                       // 刹车将应用的循环次数，在刹车模式期间更新
                brake.time_updated_pitch = false;   // flag the braking time can be re-estimated
                                                   // 标记刹车时间可以重新估算
            }

            // final lean angle should be pilot input plus wind compensation
            // 最终的倾斜角度应为飞行员输入加上风补偿
            pitch = pilot_pitch + wind_comp_pitch;
            break;

        case RPMode::BRAKE:
        case RPMode::BRAKE_READY_TO_LOITER:
            // calculate brake_pitch angle to counter-act velocity
            // 计算刹车俯仰角度以抵消速度
            update_brake_angle_from_velocity(brake.pitch, -vel_fw);

            // update braking time estimate
            // 更新刹车时间估算
            if (!brake.time_updated_pitch) {
                // check if brake angle is increasing
                // 检查刹车角度是否在增加
                if (fabsf(brake.pitch) >= brake.angle_max_pitch) {
                    brake.angle_max_pitch = fabsf(brake.pitch);
                } else {
                    // braking angle has started decreasing so re-estimate braking time
                    // 刹车角度已开始减小，因此重新估算刹车时间
                    brake.timeout_pitch = 1+(uint16_t)(LOOP_RATE_FACTOR*15L*(int32_t)(fabsf(brake.pitch))/(10L*(int32_t)g.poshold_brake_rate));  // the 1.2 (12/10) factor has to be tuned in flight, here it means 120% of the "normal" time.
                                                                                                                                             // 1.2（12/10）因子必须在飞行中调整，这里表示“正常”时间的 120%
                    brake.time_updated_pitch = true;
                }
            }

            // if velocity is very low reduce braking time to 0.5seconds
            // 如果速度非常低，将刹车时间减少到 0.5 秒
            if ((fabsf(vel_fw) <= POSHOLD_SPEED_0) && (brake.timeout_pitch > 50*LOOP_RATE_FACTOR)) {
                brake.timeout_pitch = 50*LOOP_RATE_FACTOR;
            }

            // reduce braking timer
            // 减少刹车计时器
            if (brake.timeout_pitch > 0) {
                brake.timeout_pitch--;
            } else {
                // indicate that we are ready to move to Loiter.
                // 表示我们准备好进入悬停模式
                // Loiter will only actually be engaged once both pitch_mode and pitch_mode are changed to RPMode::BRAKE_READY_TO_LOITER
                // 只有当 pitch_mode 和 pitch_mode 都更改为 RPMode::BRAKE_READY_TO_LOITER 时，悬停才会实际启动
                //  logic for engaging loiter is handled below the pitch and pitch mode switch statements
                //  启动悬停的逻辑在俯仰和俯仰模式切换语句下处理
                pitch_mode = RPMode::BRAKE_READY_TO_LOITER;
            }

            // final lean angle is braking angle + wind compensation angle
            // 最终的倾斜角度是刹车角度 + 风补偿角度
            pitch = brake.pitch + wind_comp_pitch;

            // check for pilot input
            // 检查飞行员输入
            if (!is_zero(target_pitch)) {
                // init transition to pilot override
                // 初始化过渡到飞行员覆盖
                pitch_controller_to_pilot_override();
            }
            break;

        case RPMode::BRAKE_TO_LOITER:
        case RPMode::LOITER:
            // these modes are combined pitch-pitch modes and are handled below
            // 这些模式是组合的俯仰-俯仰模式，并在下面处理
            break;

        case RPMode::CONTROLLER_TO_PILOT_OVERRIDE:
            // update pilot desired pitch angle using latest radio input
            // 使用最新的无线电输入更新飞行员期望的俯仰角度
            //  this filters the input so that it returns to zero no faster than the brake-rate
            //  这会过滤输入，使其返回到零的速度不超过刹车率
            update_pilot_lean_angle(pilot_pitch, target_pitch);

            // count-down loiter to pilot timer
            // 倒计时悬停到飞行员计时器
            if (controller_to_pilot_timer_pitch > 0) {
                controller_to_pilot_timer_pitch--;
            } else {
                // when timer runs out switch to full pilot override for next iteration
                // 当计时器耗尽时，切换到完全飞行员覆盖以进行下一次迭代
                pitch_mode = RPMode::PILOT_OVERRIDE;
            }

            // calculate controller_to_pilot mix ratio
            // 计算控制器到飞行员的混合比率
            controller_to_pilot_pitch_mix = (float)controller_to_pilot_timer_pitch / (float)POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;

            // mix final loiter lean angle and pilot desired lean angles
            // 混合最终的悬停倾斜角度和飞行员期望的倾斜角度
            pitch = mix_controls(controller_to_pilot_pitch_mix, controller_final_pitch, pilot_pitch + wind_comp_pitch);
            break;
    }

    //
    // Shared roll & pitch states (RPMode::BRAKE_TO_LOITER and RPMode::LOITER)
    // 共享的横滚和俯仰状态（RPMode::BRAKE_TO_LOITER 和 RPMode::LOITER）
    //

    // switch into LOITER mode when both roll and pitch are ready
    // 当横滚和俯仰都准备好时切换到悬停模式
    if (roll_mode == RPMode::BRAKE_READY_TO_LOITER && pitch_mode == RPMode::BRAKE_READY_TO_LOITER) {
        roll_mode = RPMode::BRAKE_TO_LOITER;
        pitch_mode = RPMode::BRAKE_TO_LOITER;
        brake.to_loiter_timer = POSHOLD_BRAKE_TO_LOITER_TIMER;
        // init loiter controller
        // 初始化悬停控制器
        loiter_nav->init_target(inertial_nav.get_position_xy_cm());
        // set delay to start of wind compensation estimate updates
        // 设置风补偿估算更新的延迟
        wind_comp_start_timer = POSHOLD_WIND_COMP_START_TIMER;
    }

    // roll-mode is used as the combined roll+pitch mode when in BRAKE_TO_LOITER or LOITER modes
    // 在刹车到悬停或悬停模式下，横滚模式用作组合的横滚+俯仰模式
    if (roll_mode == RPMode::BRAKE_TO_LOITER || roll_mode == RPMode::LOITER) {

        // force pitch mode to be same as roll_mode just to keep it consistent (it's not actually used in these states)
        // 强制俯仰模式与横滚模式相同以保持一致（实际上在这些状态下不使用）
        pitch_mode = roll_mode;

        // handle combined roll+pitch mode
        // 处理组合的横滚+俯仰模式
        switch (roll_mode) {
            case RPMode::BRAKE_TO_LOITER: {
                // reduce brake_to_loiter timer
                // 减少刹车到悬停计时器
                if (brake.to_loiter_timer > 0) {
                    brake.to_loiter_timer--;
                } else {
                    // progress to full loiter on next iteration
                    // 在下一次迭代中进入完全悬停
                    roll_mode = RPMode::LOITER;
                    pitch_mode = RPMode::LOITER;
                }

                // mix of brake and loiter controls.  0 = fully brake
                // controls, 1 = fully loiter controls
                // 刹车和悬停控制的混合。0 = 完全刹车控制，1 = 完全悬停控制
                const float brake_to_loiter_mix = (float)brake.to_loiter_timer / (float)POSHOLD_BRAKE_TO_LOITER_TIMER;

                // calculate brake.roll and pitch angles to counter-act velocity
                // 计算刹车横滚和俯仰角度以抵消速度
                update_brake_angle_from_velocity(brake.roll, vel_right);
                update_brake_angle_from_velocity(brake.pitch, -vel_fw);

                // run loiter controller
                // 运行悬停控制器
                loiter_nav->update(false);

                // calculate final roll and pitch output by mixing loiter and brake controls
                // 通过混合悬停和刹车控制计算最终的横滚和俯仰输出
                roll = mix_controls(brake_to_loiter_mix, brake.roll + wind_comp_roll, loiter_nav->get_roll());
                pitch = mix_controls(brake_to_loiter_mix, brake.pitch + wind_comp_pitch, loiter_nav->get_pitch());

                // check for pilot input
                // 检查飞行员输入
                if (!is_zero(target_roll) || !is_zero(target_pitch)) {
                    // if roll input switch to pilot override for roll
                    // 如果有横滚输入，切换到飞行员覆盖横滚
                    if (!is_zero(target_roll)) {
                        // init transition to pilot override
                        // 初始化过渡到飞行员覆盖
                        roll_controller_to_pilot_override();
                        // switch pitch-mode to brake (but ready to go back to loiter anytime)
                        // 切换俯仰模式到刹车（但随时准备返回悬停）
                        // no need to reset brake.pitch here as wind comp has not been updated since last brake.pitch computation
                        // 这里不需要重置刹车俯仰，因为自上次刹车俯仰计算以来风补偿没有更新
                        pitch_mode = RPMode::BRAKE_READY_TO_LOITER;
                    }
                    // if pitch input switch to pilot override for pitch
                    // 如果有俯仰输入，切换到飞行员覆盖俯仰
                    if (!is_zero(target_pitch)) {
                        // init transition to pilot override
                        // 初始化过渡到飞行员覆盖
                        pitch_controller_to_pilot_override();
                        if (is_zero(target_roll)) {
                            // switch roll-mode to brake (but ready to go back to loiter anytime)
                            // 切换横滚模式到刹车（但随时准备返回悬停）
                            // no need to reset brake.roll here as wind comp has not been updated since last brake.roll computation
                            // 这里不需要重置刹车横滚，因为自上次刹车横滚计算以来风补偿没有更新
                            roll_mode = RPMode::BRAKE_READY_TO_LOITER;
                        }
                    }
                }
                break;
            }
            case RPMode::LOITER:
                // run loiter controller
                // 运行悬停控制器
                loiter_nav->update(false);

                // set roll angle based on loiter controller outputs
                // 根据悬停控制器输出设置横滚角度
                roll = loiter_nav->get_roll();
                pitch = loiter_nav->get_pitch();

                // update wind compensation estimate
                // 更新风补偿估算
                update_wind_comp_estimate();

                // check for pilot input
                // 检查飞行员输入
                if (!is_zero(target_roll) || !is_zero(target_pitch)) {
                    // if roll input switch to pilot override for roll
                    // 如果有横滚输入，切换到飞行员覆盖横滚
                    if (!is_zero(target_roll)) {
                        // init transition to pilot override
                        // 初始化过渡到飞行员覆盖
                        roll_controller_to_pilot_override();
                        // switch pitch-mode to brake (but ready to go back to loiter anytime)
                        // 切换俯仰模式到刹车（但随时准备返回悬停）
                        pitch_mode = RPMode::BRAKE_READY_TO_LOITER;
                        // reset brake.pitch because wind_comp is now different and should give the compensation of the whole previous loiter angle
                        // 重置刹车俯仰，因为风补偿现在不同了，应该给出整个先前悬停角度的补偿
                        brake.pitch = 0.0f;
                    }
                    // if pitch input switch to pilot override for pitch
                    // 如果有俯仰输入，切换到飞行员覆盖俯仰
                    if (!is_zero(target_pitch)) {
                        // init transition to pilot override
                        // 初始化过渡到飞行员覆盖
                        pitch_controller_to_pilot_override();
                        // if roll not overriden switch roll-mode to brake (but be ready to go back to loiter any time)
                        // 如果横滚没有被覆盖，切换横滚模式到刹车（但随时准备返回悬停）
                        if (is_zero(target_roll)) {
                            roll_mode = RPMode::BRAKE_READY_TO_LOITER;
                            brake.roll = 0.0f;
                        }
                            // if roll not overridden switch roll-mode to brake (but be ready to go back to loiter any time)
                    }
                }
                break;

            default:
                // do nothing for uncombined roll and pitch modes
                // 对于未组合的横滚和俯仰模式不做任何处理
                break;
        }
    }

    // constrain target pitch/roll angles
    // 限制目标俯仰/横滚角度
    float angle_max = copter.aparm.angle_max;
    roll = constrain_float(roll, -angle_max, angle_max);
    pitch = constrain_float(pitch, -angle_max, angle_max);

    // call attitude controller
    // 调用姿态控制器
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(roll, pitch, target_yaw_rate);

    // run the vertical position controller and set output throttle
    // 运行垂直位置控制器并设置输出油门
    pos_control->update_z_controller();
}
// poshold_update_pilot_lean_angle - update the pilot's filtered lean angle with the latest raw input received
// poshold_update_pilot_lean_angle - 使用最新的原始输入更新飞行员的滤波倾斜角度
void ModePosHold::update_pilot_lean_angle(float &lean_angle_filtered, float &lean_angle_raw)
{
    // if raw input is large or reversing the vehicle's lean angle immediately set the fitlered angle to the new raw angle
    // 如果原始输入较大或反转了飞行器的倾斜角度，则立即将滤波角度设置为新的原始角度
    if ((lean_angle_filtered > 0 && lean_angle_raw < 0) || (lean_angle_filtered < 0 && lean_angle_raw > 0) || (fabsf(lean_angle_raw) > POSHOLD_STICK_RELEASE_SMOOTH_ANGLE)) {
        lean_angle_filtered = lean_angle_raw;
    } else {
        // lean_angle_raw must be pulling lean_angle_filtered towards zero, smooth the decrease
        // lean_angle_raw 必须将 lean_angle_filtered 拉向零，平滑减少
        if (lean_angle_filtered > 0) {
            // reduce the filtered lean angle at 5% or the brake rate (whichever is faster).
            // 以 5% 或刹车率（以较快者为准）减少滤波倾斜角度
            lean_angle_filtered -= MAX(lean_angle_filtered * POSHOLD_SMOOTH_RATE_FACTOR, MAX(1.0f, g.poshold_brake_rate/(float)LOOP_RATE_FACTOR));
            // do not let the filtered angle fall below the pilot's input lean angle.
            // 不要让滤波角度低于飞行员的输入倾斜角度
            // the above line pulls the filtered angle down and the below line acts as a catch
            // 上面的行将滤波角度拉低，下面的行起到捕捉作用
            lean_angle_filtered = MAX(lean_angle_filtered, lean_angle_raw);
        }else{
            lean_angle_filtered += MAX(-lean_angle_filtered * POSHOLD_SMOOTH_RATE_FACTOR, MAX(1.0f, g.poshold_brake_rate/(float)LOOP_RATE_FACTOR));
            lean_angle_filtered = MIN(lean_angle_filtered, lean_angle_raw);
        }
    }
}

// mix_controls - mixes two controls based on the mix_ratio
// mix_controls - 根据 mix_ratio 混合两个控制
//  mix_ratio of 1 = use first_control completely, 0 = use second_control completely, 0.5 = mix evenly
//  mix_ratio 为 1 = 完全使用 first_control，0 = 完全使用 second_control，0.5 = 平均混合
float ModePosHold::mix_controls(float mix_ratio, float first_control, float second_control)
{
    mix_ratio = constrain_float(mix_ratio, 0.0f, 1.0f);
    return mix_ratio * first_control + (1.0f - mix_ratio) * second_control;
}

// update_brake_angle_from_velocity - updates the brake_angle based on the vehicle's velocity and brake_gain
// update_brake_angle_from_velocity - 根据飞行器的速度和刹车增益更新刹车角度
//  brake_angle is slewed with the wpnav.poshold_brake_rate and constrained by the wpnav.poshold_braking_angle_max
//  brake_angle 以 wpnav.poshold_brake_rate 进行平滑处理，并受 wpnav.poshold_braking_angle_max 限制
//  velocity is assumed to be in the same direction as lean angle so for pitch you should provide the velocity backwards (i.e. -ve forward velocity)
//  假设速度与倾斜角度方向相同，因此对于俯仰，您应该提供反向速度（即负向前速度）
void ModePosHold::update_brake_angle_from_velocity(float &brake_angle, float velocity)
{
    float lean_angle;
    float brake_rate = g.poshold_brake_rate;

    brake_rate /= (float)LOOP_RATE_FACTOR;
    if (brake_rate <= 1.0f) {
        brake_rate = 1.0f;
    }

    // calculate velocity-only based lean angle
    // 计算仅基于速度的倾斜角度
    if (velocity >= 0) {
        lean_angle = -brake.gain * velocity * (1.0f + 500.0f / (velocity + 60.0f));
    } else {
        lean_angle = -brake.gain * velocity * (1.0f + 500.0f / (-velocity + 60.0f));
    }

    // do not let lean_angle be too far from brake_angle
    // 不要让倾斜角度偏离刹车角度太远
    brake_angle = constrain_float(lean_angle, brake_angle - brake_rate, brake_angle + brake_rate);

    // constrain final brake_angle
    // 限制最终刹车角度
    brake_angle = constrain_float(brake_angle, -(float)g.poshold_brake_angle_max, (float)g.poshold_brake_angle_max);
}

// initialise wind compensation estimate back to zero
// 初始化风补偿估算回零
void ModePosHold::init_wind_comp_estimate()
{
    wind_comp_ef.zero();
    wind_comp_timer = 0;
    wind_comp_roll = 0.0f;
    wind_comp_pitch = 0.0f;
}

// update_wind_comp_estimate - updates wind compensation estimate
// update_wind_comp_estimate - 更新风补偿估算
//  should be called at the maximum loop rate when loiter is engaged
//  悬停启动时应以最大循环速率调用
void ModePosHold::update_wind_comp_estimate()
{
    // check wind estimate start has not been delayed
    // 检查风估算启动是否未延迟
    if (wind_comp_start_timer > 0) {
        wind_comp_start_timer--;
        return;
    }

    // check horizontal velocity is low
    // 检查水平速度是否较低
    if (inertial_nav.get_speed_xy_cms() > POSHOLD_WIND_COMP_ESTIMATE_SPEED_MAX) {
        return;
    }

    // get position controller accel target
    // 获取位置控制器加速度目标
    const Vector3f& accel_target = pos_control->get_accel_target_cmss();

    // update wind compensation in earth-frame lean angles
    // 更新地球坐标系倾斜角度中的风补偿
    if (is_zero(wind_comp_ef.x)) {
        // if wind compensation has not been initialised set it immediately to the pos controller's desired accel in north direction
        // 如果风补偿尚未初始化，则立即将其设置为位置控制器在北方向上的期望加速度
        wind_comp_ef.x = accel_target.x;
    } else {
        // low pass filter the position controller's lean angle output
        // 对位置控制器的倾斜角度输出进行低通滤波
        wind_comp_ef.x = (1.0f-TC_WIND_COMP)*wind_comp_ef.x + TC_WIND_COMP*accel_target.x;
    }
    if (is_zero(wind_comp_ef.y)) {
        // if wind compensation has not been initialised set it immediately to the pos controller's desired accel in north direction
        // 如果风补偿尚未初始化，则立即将其设置为位置控制器在北方向上的期望加速度
        wind_comp_ef.y = accel_target.y;
    } else {
        // low pass filter the position controller's lean angle output
        // 对位置控制器的倾斜角度输出进行低通滤波
        wind_comp_ef.y = (1.0f-TC_WIND_COMP)*wind_comp_ef.y + TC_WIND_COMP*accel_target.y;
    }

    // limit acceleration
    // 限制加速度
    const float accel_lim_cmss = tanf(radians(POSHOLD_WIND_COMP_LEAN_PCT_MAX * copter.aparm.angle_max * 0.01f)) * 981.0f;
    const float wind_comp_ef_len = wind_comp_ef.length();
    if (!is_zero(accel_lim_cmss) && (wind_comp_ef_len > accel_lim_cmss)) {
        wind_comp_ef *= accel_lim_cmss / wind_comp_ef_len;
    }
}

// get_wind_comp_lean_angles - retrieve wind compensation angles in body frame roll and pitch angles
// get_wind_comp_lean_angles - 获取机体坐标系中的风补偿角度（横滚和俯仰角度）
//  should be called at the maximum loop rate
//  应以最大循环速率调用
void ModePosHold::get_wind_comp_lean_angles(float &roll_angle, float &pitch_angle)
{
    // reduce rate to 10hz
    // 将速率降低到 10hz
    wind_comp_timer++;
    if (wind_comp_timer < POSHOLD_WIND_COMP_TIMER_10HZ) {
        return;
    }
    wind_comp_timer = 0;

    // convert earth frame desired accelerations to body frame roll and pitch lean angles
    // 将地球坐标系的期望加速度转换为机体坐标系的横滚和俯仰倾斜角度
    roll_angle = atanf((-wind_comp_ef.x*ahrs.sin_yaw() + wind_comp_ef.y*ahrs.cos_yaw())/(GRAVITY_MSS*100))*(18000.0f/M_PI);
    pitch_angle = atanf(-(wind_comp_ef.x*ahrs.cos_yaw() + wind_comp_ef.y*ahrs.sin_yaw())/(GRAVITY_MSS*100))*(18000.0f/M_PI);
}

// roll_controller_to_pilot_override - initialises transition from a controller submode (brake or loiter) to a pilot override on roll axis
// roll_controller_to_pilot_override - 初始化从控制器子模式（刹车或悬停）到飞行员覆盖横滚轴的过渡
void ModePosHold::roll_controller_to_pilot_override()
{
    roll_mode = RPMode::CONTROLLER_TO_PILOT_OVERRIDE;
    controller_to_pilot_timer_roll = POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;
    // initialise pilot_roll to 0, wind_comp will be updated to compensate and poshold_update_pilot_lean_angle function shall not smooth this transition at next iteration. so 0 is the right value
    // 将 pilot_roll 初始化为 0，风补偿将更新以进行补偿，poshold_update_pilot_lean_angle 函数在下一次迭代中不会平滑此过渡。因此 0 是正确的值
    pilot_roll = 0.0f;
    // store final controller output for mixing with pilot input
    // 存储最终控制器输出以与飞行员输入混合
    controller_final_roll = roll;
}

// pitch_controller_to_pilot_override - initialises transition from a controller submode (brake or loiter) to a pilot override on roll axis
// pitch_controller_to_pilot_override - 初始化从控制器子模式（刹车或悬停）到飞行员覆盖横滚轴的过渡
void ModePosHold::pitch_controller_to_pilot_override()
{
    pitch_mode = RPMode::CONTROLLER_TO_PILOT_OVERRIDE;
    controller_to_pilot_timer_pitch = POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;
    // initialise pilot_pitch to 0, wind_comp will be updated to compensate and update_pilot_lean_angle function shall not smooth this transition at next iteration. so 0 is the right value
    // 将 pilot_pitch 初始化为 0，风补偿将更新以进行补偿，update_pilot_lean_angle 函数在下一次迭代中不会平滑此过渡。因此 0 是正确的值
    pilot_pitch = 0.0f;
    // store final loiter outputs for mixing with pilot input
    // 存储最终悬停输出以与飞行员输入混合
    controller_final_pitch = pitch;
}

#endif
