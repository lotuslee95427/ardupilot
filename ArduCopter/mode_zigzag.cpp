#include "Copter.h"

#if MODE_ZIGZAG_ENABLED

/*
* Init and run calls for zigzag flight mode
* 初始化和运行之字形飞行模式的调用
*/

#define ZIGZAG_WP_RADIUS_CM 300 // 之字形航点半径，单位：厘米
#define ZIGZAG_LINE_INFINITY -1 // 之字形线条无限

const AP_Param::GroupInfo ModeZigZag::var_info[] = {
    // @Param: AUTO_ENABLE
    // @DisplayName: ZigZag auto enable/disable
    // @Description: Allows you to enable (1) or disable (0) ZigZag auto feature
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    // 之字形自动启用/禁用
    AP_GROUPINFO_FLAGS("AUTO_ENABLE", 1, ModeZigZag, _auto_enabled, 0, AP_PARAM_FLAG_ENABLE),

#if HAL_SPRAYER_ENABLED
    // @Param: SPRAYER
    // @DisplayName: Auto sprayer in ZigZag
    // @Description: Enable the auto sprayer in ZigZag mode. SPRAY_ENABLE = 1 and SERVOx_FUNCTION = 22(SprayerPump) / 23(SprayerSpinner) also must be set. This makes the sprayer on while moving to destination A or B. The sprayer will stop if the vehicle reaches destination or the flight mode is changed from ZigZag to other.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    // 之字形模式下的自动喷洒器
    AP_GROUPINFO("SPRAYER", 2, ModeZigZag, _spray_enabled, 0),
#endif // HAL_SPRAYER_ENABLED

    // @Param: WP_DELAY
    // @DisplayName: The delay for zigzag waypoint
    // @Description: Waiting time after reached the destination
    // @Units: s
    // @Range: 0 127
    // @User: Advanced
    // 之字形航点的延迟
    AP_GROUPINFO("WP_DELAY", 3, ModeZigZag, _wp_delay, 0),

    // @Param: SIDE_DIST
    // @DisplayName: Sideways distance in ZigZag auto
    // @Description: The distance to move sideways in ZigZag mode
    // @Units: m
    // @Range: 0.1 100
    // @User: Advanced
    // 之字形自动模式中的侧向距离
    AP_GROUPINFO("SIDE_DIST", 4, ModeZigZag, _side_dist, 4),

    // @Param: DIRECTION
    // @DisplayName: Sideways direction in ZigZag auto
    // @Description: The direction to move sideways in ZigZag mode
    // @Values: 0:forward, 1:right, 2:backward, 3:left
    // @User: Advanced
    // 之字形自动模式中的侧向方向
    AP_GROUPINFO("DIRECTION", 5, ModeZigZag, _direction, 0),

    // @Param: LINE_NUM
    // @DisplayName: Total number of lines
    // @Description: Total number of lines for ZigZag auto if 1 or more. -1: Infinity, 0: Just moving to sideways
    // @Range: -1 32767
    // @User: Advanced
    // 总线条数
    AP_GROUPINFO("LINE_NUM", 6, ModeZigZag, _line_num, 0),

    AP_GROUPEND
};

// 构造函数
ModeZigZag::ModeZigZag(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// 初始化之字形控制器
bool ModeZigZag::init(bool ignore_checks)
{
    if (!copter.failsafe.radio) {
        // apply simple mode transform to pilot inputs
        // 将简单模式转换应用于飞行员输入
        update_simple_mode();

        // convert pilot input to lean angles
        // 将飞行员输入转换为倾斜角度
        float target_roll, target_pitch;
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // process pilot's roll and pitch input
        // 处理飞行员的横滚和俯仰输入
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        // 清除飞行员期望的加速度，以防发生无线电失效事件且我们未能切换到RTL
        loiter_nav->clear_pilot_desired_acceleration();
    }
    loiter_nav->init_target();

    // set vertical speed and acceleration limits
    // 设置垂直速度和加速度限制
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise the vertical position controller
    // 初始化垂直位置控制器
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // initialise waypoint state
    // 初始化航点状态
    stage = STORING_POINTS;
    dest_A.zero();
    dest_B.zero();

    // initialize zigzag auto
    // 初始化之字形自动模式
    init_auto();

    return true;
}

// perform cleanup required when leaving zigzag mode
// 执行离开之字形模式时所需的清理工作
void ModeZigZag::exit()
{
    // The sprayer will stop if the flight mode is changed from ZigZag to other
    // 如果飞行模式从之字形模式更改为其他模式，喷洒器将停止
    spray(false);
}

// run the zigzag controller
// should be called at 100hz or more
// 运行之字形控制器
// 应该以100hz或更高的频率调用
void ModeZigZag::run()
{
    // set vertical speed and acceleration limits
    // 设置垂直速度和加速度限制
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // set the direction and the total number of lines
    // 设置方向和总线条数
    zigzag_direction = (Direction)constrain_int16(_direction, 0, 3);
    line_num = constrain_int16(_line_num, ZIGZAG_LINE_INFINITY, 32767);

    // auto control
    // 自动控制
    if (stage == AUTO) {
        if (is_disarmed_or_landed() || !motors->get_interlock()) {
            // vehicle should be under manual control when disarmed or landed
            // 当车辆解除武装或着陆时，应处于手动控制状态
            return_to_manual_control(false);
        } else if (reached_destination()) {
            // if vehicle has reached destination switch to manual control or moving to A or B
            // 如果车辆已到达目的地，则切换到手动控制或移动到A或B
            AP_Notify::events.waypoint_complete = 1;
            if (is_auto) {
                if (line_num == ZIGZAG_LINE_INFINITY || line_count < line_num) {
                    if (auto_stage == AutoState::SIDEWAYS) {
                        save_or_move_to_destination((ab_dest_stored == Destination::A) ? Destination::B : Destination::A);
                    } else {
                        // spray off
                        // 关闭喷洒
                        spray(false);
                        move_to_side();
                    }
                } else {
                    init_auto();
                    return_to_manual_control(true);
                }
            } else {
                return_to_manual_control(true);
            }
        } else {
            auto_control();
        }
    }

    // manual control
    // 手动控制
    if (stage == STORING_POINTS || stage == MANUAL_REGAIN) {
        // receive pilot's inputs, do position and attitude control
        // 接收飞行员的输入，进行位置和姿态控制
        manual_control();
    }
}

// save current position as A or B.  If both A and B have been saved move to the one specified
// 将当前位置保存为A或B。如果A和B都已保存，则移动到指定的一个
void ModeZigZag::save_or_move_to_destination(Destination ab_dest)
{
    // get current position as an offset from EKF origin
    // 获取当前位置作为EKF原点的偏移量
    const Vector2f curr_pos {inertial_nav.get_position_xy_cm()};

    // handle state machine changes
    // 处理状态机变化
    switch (stage) {

        case STORING_POINTS:
            if (ab_dest == Destination::A) {
                // store point A
                // 存储点A
                dest_A = curr_pos;
                gcs().send_text(MAV_SEVERITY_INFO, "%s: point A stored", name());
                LOGGER_WRITE_EVENT(LogEvent::ZIGZAG_STORE_A);
            } else {
                // store point B
                // 存储点B
                dest_B = curr_pos;
                gcs().send_text(MAV_SEVERITY_INFO, "%s: point B stored", name());
                LOGGER_WRITE_EVENT(LogEvent::ZIGZAG_STORE_B);
            }
            // if both A and B have been stored advance state
            // 如果A和B都已存储，则推进状态
            if (!dest_A.is_zero() && !dest_B.is_zero() && !is_zero((dest_B - dest_A).length_squared())) {
                stage = MANUAL_REGAIN;
                spray(false);
            } else if (!dest_A.is_zero() || !dest_B.is_zero()) {
                // if only A or B have been stored, spray on
                // 如果仅存储了A或B，则打开喷洒
                spray(true);
            }
            break;

        case AUTO:
        case MANUAL_REGAIN:
            // A and B have been defined, move vehicle to destination A or B
            // A和B已定义，将车辆移动到目的地A或B
            Vector3f next_dest;
            bool terr_alt;
            if (calculate_next_dest(ab_dest, stage == AUTO, next_dest, terr_alt)) {
                wp_nav->wp_and_spline_init();
                if (wp_nav->set_wp_destination(next_dest, terr_alt)) {
                    stage = AUTO;
                    auto_stage = AutoState::AB_MOVING;
                    ab_dest_stored = ab_dest;
                    // spray on while moving to A or B
                    // 移动到A或B时打开喷洒
                    spray(true);
                    reach_wp_time_ms = 0;
                    if (is_auto == false || line_num == ZIGZAG_LINE_INFINITY) {
                        gcs().send_text(MAV_SEVERITY_INFO, "%s: moving to %s", name(), (ab_dest == Destination::A) ? "A" : "B");
                    } else {
                        line_count++;
                        gcs().send_text(MAV_SEVERITY_INFO, "%s: moving to %s (line %d/%d)", name(), (ab_dest == Destination::A) ? "A" : "B", line_count, line_num);
                    }
                }
            }
            break;
    }
}

// move to side
// 移动到侧面
void ModeZigZag::move_to_side()
{
    if (!dest_A.is_zero() && !dest_B.is_zero() && !is_zero((dest_B - dest_A).length_squared())) {
        Vector3f next_dest;
        bool terr_alt;
        if (calculate_side_dest(next_dest, terr_alt)) {
            wp_nav->wp_and_spline_init();
            if (wp_nav->set_wp_destination(next_dest, terr_alt)) {
                stage = AUTO;
                auto_stage = AutoState::SIDEWAYS;
                current_dest = next_dest;
                current_terr_alt = terr_alt;
                reach_wp_time_ms = 0;
                char const *dir[] = {"forward", "right", "backward", "left"};
                gcs().send_text(MAV_SEVERITY_INFO, "%s: moving to %s", name(), dir[(uint8_t)zigzag_direction]);
            }
        }
    }
}

// return manual control to the pilot
// 将手动控制权交还给飞行员
void ModeZigZag::return_to_manual_control(bool maintain_target)
{
    if (stage == AUTO) {
        stage = MANUAL_REGAIN;
        spray(false);
        loiter_nav->clear_pilot_desired_acceleration();
        if (maintain_target) {
            const Vector3f& wp_dest = wp_nav->get_wp_destination();
            loiter_nav->init_target(wp_dest.xy());
#if AP_RANGEFINDER_ENABLED
            if (wp_nav->origin_and_destination_are_terrain_alt()) {
                copter.surface_tracking.set_target_alt_cm(wp_dest.z);
            }
#endif
        } else {
            loiter_nav->init_target();
        }
        is_auto = false;
        gcs().send_text(MAV_SEVERITY_INFO, "%s: manual control", name());
    }
}

// fly the vehicle to closest point on line perpendicular to dest_A or dest_B
// 将车辆飞到垂直于dest_A或dest_B的线上的最近点
void ModeZigZag::auto_control()
{
    // process pilot's yaw input
    // 处理飞行员的偏航输入
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        // 获取飞行员期望的偏航速率
        target_yaw_rate = get_pilot_desired_yaw_rate();
    }

    // set motors to full range
    // 将电机设置为全范围
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    // 运行航点控制器
    const bool wpnav_ok = wp_nav->update_wpnav();

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    // WP_Nav已设置垂直位置控制目标
    // 运行垂直位置控制器并设置输出油门
    pos_control->update_z_controller();

    // call attitude controller
    // roll & pitch from waypoint controller, yaw rate from pilot
    // 调用姿态控制器
    // 横滚和俯仰来自航点控制器，偏航速率来自飞行员
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);

    // if wpnav failed (because of lack of terrain data) switch back to pilot control for next iteration
    // 如果wpnav失败（由于缺乏地形数据），则切换回飞行员控制以进行下一次迭代
    if (!wpnav_ok) {
        return_to_manual_control(false);
    }
}

// manual_control - process manual control
// 手动控制 - 处理手动控制
void ModeZigZag::manual_control()
{
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;

    // process pilot inputs unless we are in radio failsafe
    // 处理飞行员输入，除非我们处于无线电失效状态
    if (!copter.failsafe.radio) {
        float target_roll, target_pitch;
        // apply SIMPLE mode transform to pilot inputs
        // 将简单模式转换应用于飞行员输入
        update_simple_mode();

        // convert pilot input to lean angles
        // 将飞行员输入转换为倾斜角度
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // process pilot's roll and pitch input
        // 处理飞行员的横滚和俯仰输入
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
        // get pilot's desired yaw rate
        // 获取飞行员期望的偏航速率
        target_yaw_rate = get_pilot_desired_yaw_rate();

        // get pilot desired climb rate
        // 获取飞行员期望的爬升速率
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        // make sure the climb rate is in the given range, prevent floating point errors
        // 确保爬升速率在给定范围内，防止浮点错误
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we
        // do not switch to RTL for some reason
        // 清除飞行员期望的加速度，以防发生无线电失效事件且我们未能切换到RTL
        loiter_nav->clear_pilot_desired_acceleration();
    }

    // relax loiter target if we might be landed
    // 如果我们可能着陆，则放松悬停目标
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Loiter State Machine Determination
    // 悬停状态机确定
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    // althold state machine
    // 高度保持状态机
    switch (althold_state) {

    case AltHoldModeState::MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        loiter_nav->init_target();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);
        break;

    case AltHoldModeState::Takeoff:
        // initiate take-off
        // 启动起飞
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        // 获取避障调整后的爬升速率
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // run loiter controller
        // 运行悬停控制器
        loiter_nav->update();

        // call attitude controller
        // 调用姿态控制器
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);

        // set position controller targets adjusted for pilot input
        // 设置根据飞行员输入调整的位置控制器目标
        takeoff.do_pilot_takeoff(target_climb_rate);
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        loiter_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHoldModeState::Flying:
        // set motors to full range
        // 将电机设置为全范围
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // run loiter controller
        // 运行悬停控制器
        loiter_nav->update();

        // call attitude controller
        // 调用姿态控制器
#include "Copter.h"
#include "Copter.h"

#if MODE_ZIGZAG_ENABLED

/*
* Init and run calls for zigzag flight mode
* 初始化和运行之字形飞行模式的调用
*/

#define ZIGZAG_WP_RADIUS_CM 300 // 之字形航点半径，单位：厘米
#define ZIGZAG_LINE_INFINITY -1 // 之字形线条无限

const AP_Param::GroupInfo ModeZigZag::var_info[] = {
    // @Param: AUTO_ENABLE
    // @DisplayName: ZigZag auto enable/disable
    // @Description: Allows you to enable (1) or disable (0) ZigZag auto feature
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    // 之字形自动启用/禁用
    AP_GROUPINFO_FLAGS("AUTO_ENABLE", 1, ModeZigZag, _auto_enabled, 0, AP_PARAM_FLAG_ENABLE),

#if HAL_SPRAYER_ENABLED
    // @Param: SPRAYER
    // @DisplayName: Auto sprayer in ZigZag
    // @Description: Enable the auto sprayer in ZigZag mode. SPRAY_ENABLE = 1 and SERVOx_FUNCTION = 22(SprayerPump) / 23(SprayerSpinner) also must be set. This makes the sprayer on while moving to destination A or B. The sprayer will stop if the vehicle reaches destination or the flight mode is changed from ZigZag to other.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    // 之字形模式下的自动喷洒器
    AP_GROUPINFO("SPRAYER", 2, ModeZigZag, _spray_enabled, 0),
#endif // HAL_SPRAYER_ENABLED

    // @Param: WP_DELAY
    // @DisplayName: The delay for zigzag waypoint
    // @Description: Waiting time after reached the destination
    // @Units: s
    // @Range: 0 127
    // @User: Advanced
    // 之字形航点的延迟
    AP_GROUPINFO("WP_DELAY", 3, ModeZigZag, _wp_delay, 0),

    // @Param: SIDE_DIST
    // @DisplayName: Sideways distance in ZigZag auto
    // @Description: The distance to move sideways in ZigZag mode
    // @Units: m
    // @Range: 0.1 100
    // @User: Advanced
    // 之字形自动模式中的侧向距离
    AP_GROUPINFO("SIDE_DIST", 4, ModeZigZag, _side_dist, 4),

    // @Param: DIRECTION
    // @DisplayName: Sideways direction in ZigZag auto
    // @Description: The direction to move sideways in ZigZag mode
    // @Values: 0:forward, 1:right, 2:backward, 3:left
    // @User: Advanced
    // 之字形自动模式中的侧向方向
    AP_GROUPINFO("DIRECTION", 5, ModeZigZag, _direction, 0),

    // @Param: LINE_NUM
    // @DisplayName: Total number of lines
    // @Description: Total number of lines for ZigZag auto if 1 or more. -1: Infinity, 0: Just moving to sideways
    // @Range: -1 32767
    // @User: Advanced
    // 之字形自动模式中的总线数
    AP_GROUPINFO("LINE_NUM", 6, ModeZigZag, _line_num, 0),

    AP_GROUPEND
};

// ModeZigZag构造函数
ModeZigZag::ModeZigZag(void) : Mode()
{
    // 设置对象默认参数
    AP_Param::setup_object_defaults(this, var_info);
}

// 初始化之字形控制器
bool ModeZigZag::init(bool ignore_checks)
{
    if (!copter.failsafe.radio) {
        // 将简单模式变换应用于飞行员输入
        update_simple_mode();

        // 将飞行员输入转换为倾斜角度
        float target_roll, target_pitch;
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // 处理飞行员的横滚和俯仰输入
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
    } else {
        // 清除飞行员期望的加速度，以防无线电失效事件发生且由于某种原因未切换到RTL
        loiter_nav->clear_pilot_desired_acceleration();
    }
    // 初始化悬停目标
    loiter_nav->init_target();

    // 设置垂直速度和加速度限制
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // 初始化垂直位置控制器
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // 初始化航点状态
    stage = STORING_POINTS;
    dest_A.zero();
    dest_B.zero();

    // 初始化之字形自动模式
    init_auto();

    return true;
}

// 执行离开之字形模式时所需的清理工作
void ModeZigZag::exit()
{
    // 如果飞行模式从之字形更改为其他模式，喷洒器将停止
    spray(false);
}

// 运行之字形控制器
// 应该以100hz或更高的频率调用
void ModeZigZag::run()
{
    // 设置垂直速度和加速度限制
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // 设置方向和总线数
    zigzag_direction = (Direction)constrain_int16(_direction, 0, 3);
    line_num = constrain_int16(_line_num, ZIGZAG_LINE_INFINITY, 32767);

    // 自动控制
    if (stage == AUTO) {
        if (is_disarmed_or_landed() || !motors->get_interlock()) {
            // 当解除武装或着陆时，车辆应处于手动控制状态
            return_to_manual_control(false);
        } else if (reached_destination()) {
            // 如果车辆到达目的地，则切换到手动控制或移动到A或B
            AP_Notify::events.waypoint_complete = 1;
            if (is_auto) {
                if (line_num == ZIGZAG_LINE_INFINITY || line_count < line_num) {
                    if (auto_stage == AutoState::SIDEWAYS) {
                        save_or_move_to_destination((ab_dest_stored == Destination::A) ? Destination::B : Destination::A);
                    } else {
                        // 关闭喷洒
                        spray(false);
                        move_to_side();
                    }
                } else {
                    init_auto();
                    return_to_manual_control(true);
                }
            } else {
                return_to_manual_control(true);
            }
        } else {
            auto_control();
        }
    }

    // 手动控制
    if (stage == STORING_POINTS || stage == MANUAL_REGAIN) {
        // 接收飞行员的输入，进行位置和姿态控制
        manual_control();
    }
}
// save current position as A or B.  If both A and B have been saved move to the one specified
// 将当前位置保存为A或B。如果A和B都已保存，则移动到指定的一个
void ModeZigZag::save_or_move_to_destination(Destination ab_dest)
{
    // get current position as an offset from EKF origin
    // 获取当前位置作为EKF原点的偏移量
    const Vector2f curr_pos {inertial_nav.get_position_xy_cm()};

    // handle state machine changes
    // 处理状态机变化
    switch (stage) {

        case STORING_POINTS:
            if (ab_dest == Destination::A) {
                // store point A
                // 存储点A
                dest_A = curr_pos;
                gcs().send_text(MAV_SEVERITY_INFO, "%s: point A stored", name());
                LOGGER_WRITE_EVENT(LogEvent::ZIGZAG_STORE_A);
            } else {
                // store point B
                // 存储点B
                dest_B = curr_pos;
                gcs().send_text(MAV_SEVERITY_INFO, "%s: point B stored", name());
                LOGGER_WRITE_EVENT(LogEvent::ZIGZAG_STORE_B);
            }
            // if both A and B have been stored advance state
            // 如果A和B都已存储，则推进状态
            if (!dest_A.is_zero() && !dest_B.is_zero() && !is_zero((dest_B - dest_A).length_squared())) {
                stage = MANUAL_REGAIN;
                spray(false);
            } else if (!dest_A.is_zero() || !dest_B.is_zero()) {
                // if only A or B have been stored, spray on
                // 如果仅存储了A或B，则打开喷洒
                spray(true);
            }
            break;

        case AUTO:
        case MANUAL_REGAIN:
            // A and B have been defined, move vehicle to destination A or B
            // A和B已定义，将车辆移动到目的地A或B
            Vector3f next_dest;
            bool terr_alt;
            if (calculate_next_dest(ab_dest, stage == AUTO, next_dest, terr_alt)) {
                wp_nav->wp_and_spline_init();
                if (wp_nav->set_wp_destination(next_dest, terr_alt)) {
                    stage = AUTO;
                    auto_stage = AutoState::AB_MOVING;
                    ab_dest_stored = ab_dest;
                    // spray on while moving to A or B
                    // 移动到A或B时打开喷洒
                    spray(true);
                    reach_wp_time_ms = 0;
                    if (is_auto == false || line_num == ZIGZAG_LINE_INFINITY) {
                        gcs().send_text(MAV_SEVERITY_INFO, "%s: moving to %s", name(), (ab_dest == Destination::A) ? "A" : "B");
                    } else {
                        line_count++;
                        gcs().send_text(MAV_SEVERITY_INFO, "%s: moving to %s (line %d/%d)", name(), (ab_dest == Destination::A) ? "A" : "B", line_count, line_num);
                    }
                }
            }
            break;
    }
}

// move to side
// 移动到侧面
void ModeZigZag::move_to_side()
{
    if (!dest_A.is_zero() && !dest_B.is_zero() && !is_zero((dest_B - dest_A).length_squared())) {
        Vector3f next_dest;
        bool terr_alt;
        if (calculate_side_dest(next_dest, terr_alt)) {
            wp_nav->wp_and_spline_init();
            if (wp_nav->set_wp_destination(next_dest, terr_alt)) {
                stage = AUTO;
                auto_stage = AutoState::SIDEWAYS;
                current_dest = next_dest;
                current_terr_alt = terr_alt;
                reach_wp_time_ms = 0;
                char const *dir[] = {"forward", "right", "backward", "left"};
                gcs().send_text(MAV_SEVERITY_INFO, "%s: moving to %s", name(), dir[(uint8_t)zigzag_direction]);
            }
        }
    }
}

// return manual control to the pilot
// 将手动控制权交还给飞行员
void ModeZigZag::return_to_manual_control(bool maintain_target)
{
    if (stage == AUTO) {
        stage = MANUAL_REGAIN;
        spray(false);
        loiter_nav->clear_pilot_desired_acceleration();
        if (maintain_target) {
            const Vector3f& wp_dest = wp_nav->get_wp_destination();
            loiter_nav->init_target(wp_dest.xy());
#if AP_RANGEFINDER_ENABLED
            if (wp_nav->origin_and_destination_are_terrain_alt()) {
                copter.surface_tracking.set_target_alt_cm(wp_dest.z);
            }
#endif
        } else {
            loiter_nav->init_target();
        }
        is_auto = false;
        gcs().send_text(MAV_SEVERITY_INFO, "%s: manual control", name());
    }
}

// fly the vehicle to closest point on line perpendicular to dest_A or dest_B
// 将车辆飞到垂直于dest_A或dest_B的线上的最近点
void ModeZigZag::auto_control()
{
    // process pilot's yaw input
    // 处理飞行员的偏航输入
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        // 获取飞行员的期望偏航速率
        target_yaw_rate = get_pilot_desired_yaw_rate();
    }

    // set motors to full range
    // 将电机设置为全范围
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    // 运行航点控制器
    const bool wpnav_ok = wp_nav->update_wpnav();

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    // WP_Nav已设置垂直位置控制目标
    // 运行垂直位置控制器并设置输出油门
    pos_control->update_z_controller();

    // call attitude controller
    // roll & pitch from waypoint controller, yaw rate from pilot
    // 调用姿态控制器
    // 滚转和俯仰来自航点控制器，偏航速率来自飞行员
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);

    // if wpnav failed (because of lack of terrain data) switch back to pilot control for next iteration
    // 如果wpnav失败（由于缺乏地形数据），则在下一次迭代中切换回飞行员控制
    if (!wpnav_ok) {
        return_to_manual_control(false);
    }
}

// manual_control - process manual control
// 手动控制 - 处理手动控制
void ModeZigZag::manual_control()
{
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;

    // process pilot inputs unless we are in radio failsafe
    // 处理飞行员输入，除非我们处于无线电失效保护状态
    if (!copter.failsafe.radio) {
        float target_roll, target_pitch;
        // apply SIMPLE mode transform to pilot inputs
        // 将SIMPLE模式转换应用于飞行员输入
        update_simple_mode();

        // convert pilot input to lean angles
        // 将飞行员输入转换为倾斜角度
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // process pilot's roll and pitch input
        // 处理飞行员的滚转和俯仰输入
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
        // get pilot's desired yaw rate
        // 获取飞行员的期望偏航速率
        target_yaw_rate = get_pilot_desired_yaw_rate();

        // get pilot desired climb rate
        // 获取飞行员期望的爬升速率
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        // make sure the climb rate is in the given range, prevent floating point errors
        // 确保爬升速率在给定范围内，防止浮点错误
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we
        // do not switch to RTL for some reason
        // 清除飞行员期望的加速度，以防发生无线电失效保护事件，并且由于某种原因我们没有切换到RTL
        loiter_nav->clear_pilot_desired_acceleration();
    }

    // relax loiter target if we might be landed
    // 如果我们可能着陆，则放松悬停目标
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Loiter State Machine Determination
    // 悬停状态机确定
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    // althold state machine
    // 悬停状态机
    switch (althold_state) {

    case AltHoldModeState::MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        loiter_nav->init_target();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);
        break;

    case AltHoldModeState::Takeoff:
        // initiate take-off
        // 启动起飞
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        // 获取避障调整后的爬升速率
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // run loiter controller
        // 运行悬停控制器
        loiter_nav->update();

        // call attitude controller
        // 调用姿态控制器
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);

        // set position controller targets adjusted for pilot input
        // 设置根据飞行员输入调整的位置控制器目标
        takeoff.do_pilot_takeoff(target_climb_rate);
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        loiter_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHoldModeState::Flying:
        // set motors to full range
        // 将电机设置为全范围
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // run loiter controller
        // 运行悬停控制器
        loiter_nav->update();

        // call attitude controller
        // 调用姿态控制器
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);

        // get avoidance adjusted climb rate
        // 获取避障调整后的爬升速率
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

#if AP_RANGEFINDER_ENABLED
        // update the vertical offset based on the surface measurement
        // 根据地表测量更新垂直偏移
        copter.surface_tracking.update_surface_offset();
#endif

        // Send the commanded climb rate to the position controller
        // 将命令的爬升速率发送到位置控制器
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        break;
    }

    // run the vertical position controller and set output throttle
    // 运行垂直位置控制器并设置输出油门
    pos_control->update_z_controller();
}
// return true if vehicle is within a small area around the destination
// 如果车辆在目的地周围的小区域内，则返回true
bool ModeZigZag::reached_destination()
{
    // check if wp_nav believes it has reached the destination
    // 检查wp_nav是否认为已经到达目的地
    if (!wp_nav->reached_wp_destination()) {
        return false;
    }

    // check distance to destination
    // 检查到目的地的距离
    if (wp_nav->get_wp_distance_to_destination() > ZIGZAG_WP_RADIUS_CM) {
        return false;
    }

    // wait at time which is set in zigzag_wp_delay
    // 等待在zigzag_wp_delay中设置的时间
    uint32_t now = AP_HAL::millis();
    if (reach_wp_time_ms == 0) {
        reach_wp_time_ms = now;
    }
    return ((now - reach_wp_time_ms) >= (uint16_t)constrain_int16(_wp_delay, 0, 127) * 1000);
}

// calculate next destination according to vector A-B and current position
// 根据向量A-B和当前位置计算下一个目的地
// use_wpnav_alt should be true if waypoint controller's altitude target should be used, false for position control or current altitude target
// 如果应使用航点控制器的高度目标，则use_wpnav_alt应为true；对于位置控制或当前高度目标，则为false
// terrain_alt is returned as true if the next_dest should be considered a terrain alt
// 如果next_dest应被视为地形高度，则返回true
bool ModeZigZag::calculate_next_dest(Destination ab_dest, bool use_wpnav_alt, Vector3f& next_dest, bool& terrain_alt) const
{
    // define start_pos as either destination A or B
    // 将start_pos定义为目的地A或B
    Vector2f start_pos = (ab_dest == Destination::A) ? dest_A : dest_B;

    // calculate vector from A to B
    // 计算从A到B的向量
    Vector2f AB_diff = dest_B - dest_A;

    // check distance between A and B
    // 检查A和B之间的距离
    if (is_zero(AB_diff.length_squared())) {
        return false;
    }

    // get distance from vehicle to start_pos
    // 获取从车辆到start_pos的距离
    const Vector2f curr_pos2d {inertial_nav.get_position_xy_cm()};
    Vector2f veh_to_start_pos = curr_pos2d - start_pos;

    // lengthen AB_diff so that it is at least as long as vehicle is from start point
    // we need to ensure that the lines perpendicular to AB are long enough to reach the vehicle
    // 延长AB_diff，使其至少与车辆到起点的距离一样长
    // 我们需要确保垂直于AB的线足够长以到达车辆
    float scalar = 1.0f;
    if (veh_to_start_pos.length_squared() > AB_diff.length_squared()) {
        scalar = veh_to_start_pos.length() / AB_diff.length();
    }

    // create a line perpendicular to AB but originating at start_pos
    // 创建一条垂直于AB但起点为start_pos的线
    Vector2f perp1 = start_pos + Vector2f(-AB_diff[1] * scalar, AB_diff[0] * scalar);
    Vector2f perp2 = start_pos + Vector2f(AB_diff[1] * scalar, -AB_diff[0] * scalar);

    // find the closest point on the perpendicular line
    // 找到垂直线上最近的点
    const Vector2f closest2d = Vector2f::closest_point(curr_pos2d, perp1, perp2);
    next_dest.x = closest2d.x;
    next_dest.y = closest2d.y;

    if (use_wpnav_alt) {
        // get altitude target from waypoint controller
        // 从航点控制器获取高度目标
        terrain_alt = wp_nav->origin_and_destination_are_terrain_alt();
        next_dest.z = wp_nav->get_wp_destination().z;
    } else {
        // if we have a downward facing range finder then use terrain altitude targets
        // 如果我们有一个向下的测距仪，则使用地形高度目标
        terrain_alt = copter.rangefinder_alt_ok() && wp_nav->rangefinder_used_and_healthy();
        if (terrain_alt) {
#if AP_RANGEFINDER_ENABLED
            if (!copter.surface_tracking.get_target_alt_cm(next_dest.z)) {
                next_dest.z = copter.rangefinder_state.alt_cm_filt.get();
            }
#endif
        } else {
            next_dest.z = pos_control->is_active_z() ? pos_control->get_pos_target_z_cm() : inertial_nav.get_position_z_up_cm();
        }
    }

    return true;
}

// calculate side destination according to vertical vector A-B and current position
// 根据垂直向量A-B和当前位置计算侧面目的地
// terrain_alt is returned as true if the next_dest should be considered a terrain alt
// 如果next_dest应被视为地形高度，则返回true
bool ModeZigZag::calculate_side_dest(Vector3f& next_dest, bool& terrain_alt) const
{
    // calculate vector from A to B
    // 计算从A到B的向量
    Vector2f AB_diff = dest_B - dest_A;

    // calculate a vertical right or left vector for AB from the current yaw direction
    // 根据当前偏航方向计算AB的垂直向右或向左向量
    Vector2f AB_side;
    if (zigzag_direction == Direction::RIGHT || zigzag_direction == Direction::LEFT) {
        float yaw_ab_sign = (-ahrs.sin_yaw() * AB_diff[1]) + (ahrs.cos_yaw() * -AB_diff[0]);
        if (is_positive(yaw_ab_sign * (zigzag_direction == Direction::RIGHT ? 1 : -1))) {
            AB_side = Vector2f(AB_diff[1], -AB_diff[0]);
        } else {
            AB_side = Vector2f(-AB_diff[1], AB_diff[0]);
        }
    } else {
        float yaw_ab_sign = (ahrs.cos_yaw() * AB_diff[1]) + (ahrs.sin_yaw() * -AB_diff[0]);
        if (is_positive(yaw_ab_sign * (zigzag_direction == Direction::FORWARD ? 1 : -1))) {
            AB_side = Vector2f(AB_diff[1], -AB_diff[0]);
        } else {
            AB_side = Vector2f(-AB_diff[1], AB_diff[0]);
        }
    }

    // check distance the vertical vector between A and B
    // 检查A和B之间的垂直向量的距离
    if (is_zero(AB_side.length_squared())) {
        return false;
    }

    // adjust AB_side length to zigzag_side_dist
    // 将AB_side长度调整为zigzag_side_dist
    float scalar = constrain_float(_side_dist, 0.1f, 100.0f) * 100 / safe_sqrt(AB_side.length_squared());

    // get distance from vehicle to start_pos
    // 获取从车辆到start_pos的距离
    const Vector2f curr_pos2d {inertial_nav.get_position_xy_cm()};
    next_dest.x = curr_pos2d.x + (AB_side.x * scalar);
    next_dest.y = curr_pos2d.y + (AB_side.y * scalar);

    // if we have a downward facing range finder then use terrain altitude targets
    // 如果我们有一个向下的测距仪，则使用地形高度目标
    terrain_alt = copter.rangefinder_alt_ok() && wp_nav->rangefinder_used_and_healthy();
    if (terrain_alt) {
#if AP_RANGEFINDER_ENABLED
        if (!copter.surface_tracking.get_target_alt_cm(next_dest.z)) {
            next_dest.z = copter.rangefinder_state.alt_cm_filt.get();
        }
#endif
    } else {
        next_dest.z = pos_control->is_active_z() ? pos_control->get_pos_target_z_cm() : inertial_nav.get_position_z_up_cm();
    }

    return true;
}

// run zigzag auto feature which is automate both AB and sideways
// 运行之字形自动功能，自动化AB和侧向移动
void ModeZigZag::run_auto()
{
    // exit immediately if we are disabled
    // 如果我们被禁用，则立即退出
    if (!_auto_enabled) {
        return;
    }

    // make sure both A and B point are registered and not when moving to A or B
    // 确保A和B点都已注册，并且不在移动到A或B时
    if (stage != MANUAL_REGAIN) {
        return;
    }

    is_auto = true;
    // resume if zigzag auto is suspended
    // 如果之字形自动暂停，则恢复
    if (is_suspended && line_count <= line_num) {
        // resume the stage when it was suspended
        // 恢复暂停时的阶段
        if (auto_stage == AutoState::AB_MOVING) {
            line_count--;
            save_or_move_to_destination(ab_dest_stored);
        } else if (auto_stage == AutoState::SIDEWAYS) {
            wp_nav->wp_and_spline_init();
            if (wp_nav->set_wp_destination(current_dest, current_terr_alt)) {
                stage = AUTO;
                reach_wp_time_ms = 0;
                char const *dir[] = {"forward", "right", "backward", "left"};
                gcs().send_text(MAV_SEVERITY_INFO, "%s: moving to %s", name(), dir[(uint8_t)zigzag_direction]);
            }
        }
    } else {
        move_to_side();
    }
}

// suspend zigzag auto
// 暂停之字形自动
void ModeZigZag::suspend_auto()
{
    // exit immediately if we are disabled
    // 如果我们被禁用，则立即退出
    if (!_auto_enabled) {
        return;
    }

    if (auto_stage != AutoState::MANUAL) {
        is_suspended = true;
        return_to_manual_control(true);
    }
}

// initialize zigzag auto
// 初始化之字形自动
void ModeZigZag::init_auto()
{
    is_auto = false;
    auto_stage = AutoState::MANUAL;
    line_count = 0;
    is_suspended = false;
}

// spray on / off
// 喷洒开/关
void ModeZigZag::spray(bool b)
{
#if HAL_SPRAYER_ENABLED
    // 如果启用了喷洒器
    if (_spray_enabled) {
        // 运行喷洒器
        copter.sprayer.run(b);
    }
#endif
}

uint32_t ModeZigZag::wp_distance() const
{
    // 如果是自动模式，返回到达目的地的航点距离，否则返回0
    return is_auto ? wp_nav->get_wp_distance_to_destination() : 0;
}

int32_t ModeZigZag::wp_bearing() const
{
    // 如果是自动模式，返回到达目的地的航点方位角，否则返回0
    return is_auto ? wp_nav->get_wp_bearing_to_destination() : 0;
}

float ModeZigZag::crosstrack_error() const
{
    // 如果是自动模式，返回航点的横向误差，否则返回0
    return is_auto ? wp_nav->crosstrack_error() : 0;
}

#endif // MODE_ZIGZAG_ENABLED
