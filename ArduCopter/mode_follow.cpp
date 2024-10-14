#include "Copter.h"

#if MODE_FOLLOW_ENABLED

/*
 * mode_follow.cpp - 通过系统ID跟随另一个支持mavlink的飞行器
 *
 * TODO: 使用摇杆控制在球面上移动
 * TODO: 使用摇杆控制改变球面直径
 * TODO: 添加"通道7选项"以锁定"指向的"目标
 * TODO: 改进围绕移动点的盘旋效果；可能需要PID控制器？也许可以使用悬停控制器？
 * TODO: 使用目标飞行器的速度和加速度来外推其位置
 * TODO: 确保AP_AVOIDANCE_ENABLED为true，因为我们依赖它的速度限制功能
 *
 * 中文注释：
 * 这是跟随模式的主要实现文件。该模式允许飞行器跟随另一个指定的飞行器。
 * 目前还有一些待实现的功能，包括：
 * - 通过摇杆控制在目标周围的球面上移动
 * - 通过摇杆控制改变跟随距离（球面直径）
 * - 添加一个选项，可以锁定当前指向的目标
 * - 改进围绕移动目标的盘旋效果，可能需要引入PID控制器或使用现有的悬停控制器
 * - 根据目标飞行器的速度和加速度来预测其未来位置
 * - 确保启用了避障功能，因为跟随模式依赖于其速度限制功能
 */

// 初始化跟随模式
bool ModeFollow::init(const bool ignore_checks)
{
    // 检查跟随模式是否启用
    if (!g2.follow.enabled()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Set FOLL_ENABLE = 1");
        return false;
    }

#if HAL_MOUNT_ENABLED
    AP_Mount *mount = AP_Mount::get_singleton();
    // follow the lead vehicle using sysid
    // 使用系统ID跟随领航飞行器
    if (g2.follow.option_is_enabled(AP_Follow::Option::MOUNT_FOLLOW_ON_ENTER) && mount != nullptr) {
        mount->set_target_sysid(g2.follow.get_target_sysid());
    }
#endif

    // re-use guided mode
    // 重用引导模式的初始化
    return ModeGuided::init(ignore_checks);
}

// 执行离开跟随模式时所需的清理工作
void ModeFollow::exit()
{
    g2.follow.clear_offsets_if_required();
}

// 跟随模式的主运行函数
void ModeFollow::run()
{
    // if not armed set throttle to zero and exit immediately
    // 如果未解锁或已着陆，将油门设为零并立即退出
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    // 将电机设置为全范围
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // re-use guided mode's velocity controller
    // Note: this is safe from interference from GCSs and companion computer's whose guided mode
    //       position and velocity requests will be ignored while the vehicle is not in guided mode

    // 重用引导模式的速度控制器
    // 注意：这可以避免来自GCS和伴随计算机的引导模式干扰
    //       当飞行器不在引导模式时，位置和速度请求将被忽略

    // variables to be sent to velocity controller
    // 用于发送到速度控制器的变量
    Vector3f desired_velocity_neu_cms;
    bool use_yaw = false;
    float yaw_cd = 0.0f;

    Vector3f dist_vec;  // vector to lead vehicle
                        // 指向领航飞行器的向量

    Vector3f dist_vec_offs;  // vector to lead vehicle + offset
                             // 指向领航飞行器的向量加上偏移量

    Vector3f vel_of_target;  // velocity of lead vehicle
                             // 领航飞行器的速度向量
    if (g2.follow.get_target_dist_and_vel_ned(dist_vec, dist_vec_offs, vel_of_target)) {
        // convert dist_vec_offs to cm in NEU
        // 将dist_vec_offs转换为NEU坐标系下的厘米单位
        const Vector3f dist_vec_offs_neu(dist_vec_offs.x * 100.0f, dist_vec_offs.y * 100.0f, -dist_vec_offs.z * 100.0f);

        // calculate desired relative velocity vector in cm/s in NEU
        // 计算NEU坐标系下的期望相对速度向量，单位为厘米/秒
        const float kp = g2.follow.get_pos_p().kP();
        desired_velocity_neu_cms = dist_vec_offs_neu * kp;

        // create horizontal desired velocity vector (required for slow down calculations)
        // 创建水平期望速度向量（用于减速计算）
        Vector2f desired_velocity_xy_cms(desired_velocity_neu_cms.x, desired_velocity_neu_cms.y);

        // create horizontal unit vector towards target (required for slow down calculations)
        // 创建指向目标的水平单位向量（用于减速计算）
        Vector2f dir_to_target_xy(desired_velocity_xy_cms.x, desired_velocity_xy_cms.y);
        if (!dir_to_target_xy.is_zero()) {
            dir_to_target_xy.normalize();
        }

        // slow down horizontally as we approach target (use 1/2 of maximum deceleration for gentle slow down)
        // 接近目标时水平减速（使用最大减速度的1/2以实现平缓减速）
        const float dist_to_target_xy = Vector2f(dist_vec_offs_neu.x, dist_vec_offs_neu.y).length();
        copter.avoid.limit_velocity_2D(pos_control->get_pos_xy_p().kP().get(), pos_control->get_max_accel_xy_cmss() * 0.5f, desired_velocity_xy_cms, dir_to_target_xy, dist_to_target_xy, copter.G_Dt);
        // copy horizontal velocity limits back to 3d vector
        // 将水平速度限制复制回3D向量
        desired_velocity_neu_cms.xy() = desired_velocity_xy_cms;

        // limit vertical desired_velocity_neu_cms to slow as we approach target (we use 1/2 of maximum deceleration for gentle slow down)
        // 限制垂直期望速度以在接近目标时减速（使用最大减速度的1/2以实现平缓减速）
        const float des_vel_z_max = copter.avoid.get_max_speed(pos_control->get_pos_z_p().kP().get(), pos_control->get_max_accel_z_cmss() * 0.5f, fabsf(dist_vec_offs_neu.z), copter.G_Dt);
        desired_velocity_neu_cms.z = constrain_float(desired_velocity_neu_cms.z, -des_vel_z_max, des_vel_z_max);

        // Add the target velocity baseline.
        // 添加目标速度基线
        desired_velocity_neu_cms.xy() += vel_of_target.xy() * 100.0f;
        desired_velocity_neu_cms.z += -vel_of_target.z * 100.0f;

        // scale desired velocity to stay within horizontal speed limit
        // 缩放期望速度以保持在水平速度限制内
        desired_velocity_neu_cms.xy().limit_length(pos_control->get_max_speed_xy_cms());

        // limit desired velocity to be between maximum climb and descent rates
        // 限制期望速度在最大爬升和下降速率之间
        desired_velocity_neu_cms.z = constrain_float(desired_velocity_neu_cms.z, -fabsf(pos_control->get_max_speed_down_cms()), pos_control->get_max_speed_up_cms());

        // limit the velocity for obstacle/fence avoidance
        // 限制速度以避开障碍物/围栏
        copter.avoid.adjust_velocity(desired_velocity_neu_cms, pos_control->get_pos_xy_p().kP().get(), pos_control->get_max_accel_xy_cmss(), pos_control->get_pos_z_p().kP().get(), pos_control->get_max_accel_z_cmss(), G_Dt);

        // calculate vehicle heading
        // 计算飞行器航向
        switch (g2.follow.get_yaw_behave()) {
            case AP_Follow::YAW_BEHAVE_FACE_LEAD_VEHICLE: {
                if (dist_vec.xy().length_squared() > 1.0) {
                    yaw_cd = get_bearing_cd(Vector2f{}, dist_vec.xy());
                    use_yaw = true;
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_SAME_AS_LEAD_VEHICLE: {
                float target_hdg = 0.0f;
                if (g2.follow.get_target_heading_deg(target_hdg)) {
                    yaw_cd = target_hdg * 100.0f;
                    use_yaw = true;
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_DIR_OF_FLIGHT: {
                if (desired_velocity_neu_cms.xy().length_squared() > (100.0 * 100.0)) {
                    yaw_cd = get_bearing_cd(Vector2f{}, desired_velocity_neu_cms.xy());
                    use_yaw = true;
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_NONE:
            default:
                // do nothing
               break;

        }
    }

    // log output at 10hz
    // 以10Hz的频率记录输出
    uint32_t now = AP_HAL::millis();
    bool log_request = false;
    if ((now - last_log_ms >= 100) || (last_log_ms == 0)) {
        log_request = true;
        last_log_ms = now;
    }
    // re-use guided mode's velocity controller (takes NEU)
    // 重用引导模式的速度控制器（使用NEU坐标系）
    ModeGuided::set_velocity(desired_velocity_neu_cms, use_yaw, yaw_cd, false, 0.0f, false, log_request);

    ModeGuided::run();
}

// 获取到航点的距离（厘米）
uint32_t ModeFollow::wp_distance() const
{
    return g2.follow.get_distance_to_target() * 100;
}

// 获取到航点的方位角（厘度）
int32_t ModeFollow::wp_bearing() const
{
    return g2.follow.get_bearing_to_target() * 100;
}

/*
  get target position for mavlink reporting
  获取用于mavlink报告的目标位置
 */
bool ModeFollow::get_wp(Location &loc) const
{
    float dist = g2.follow.get_distance_to_target();
    float bearing = g2.follow.get_bearing_to_target();
    loc = copter.current_loc;
    loc.offset_bearing(bearing, dist);
    return true;
}

#endif // MODE_FOLLOW_ENABLED
