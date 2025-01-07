#include "Plane.h"

/**
 *
 * 检测 EKF 或惯性导航系统的故障，触发对飞行员的警报
 * 并帮助采取应对措施
 *
 */

#ifndef EKF_CHECK_ITERATIONS_MAX
 # define EKF_CHECK_ITERATIONS_MAX          10      // 1秒（即10Hz下的10次迭代）的不良方差表示故障
#endif

#ifndef EKF_CHECK_WARNING_TIME
 # define EKF_CHECK_WARNING_TIME            (30*1000)   // 警告文本消息最多每30秒发送一次到地面站
#endif

////////////////////////////////////////////////////////////////////////////////
// EKF_check 结构体
////////////////////////////////////////////////////////////////////////////////
static struct {
    uint8_t fail_count;         // EKF 或 DCM 超出容差的迭代次数
    bool bad_variance;          // 如果 EKF 应被视为不可信（fail_count 已超过 EKF_CHECK_ITERATIONS_MAX），则为 true
    uint32_t last_warn_time;    // 上次警告的系统时间（毫秒）。用于限制发送到 GCS 的文本警告频率
    bool failsafe_on;           // 当导航失效保护开启时为 true
} ekf_check_state;

// ekf_check - 检测 EKF 方差是否超出容差并触发失效保护
// 应以 10Hz 的频率调用
void Plane::ekf_check()
{
    // 确保 EKF_CHECK_ITERATIONS_MAX 至少为 7
    static_assert(EKF_CHECK_ITERATIONS_MAX >= 7, "EKF_CHECK_ITERATIONS_MAX 必须至少为 7");

    // 如果 EKF 还没有原点，立即退出 - 假设原点永远不会被取消设置
    Location temp_loc;
    if (!ahrs.get_origin(temp_loc)) {
        return;
    }

    // 如果电机未解锁，或 EKF 检查被禁用，立即返回
    bool ekf_check_disabled = !plane.arming.is_armed() || (g2.fs_ekf_thresh <= 0.0f);
#if HAL_QUADPLANE_ENABLED
    if (!quadplane.in_vtol_posvel_mode()) {
        ekf_check_disabled = true;
    }
#endif
    if (ekf_check_disabled) {
        ekf_check_state.fail_count = 0;
        ekf_check_state.bad_variance = false;
        AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;
        failsafe_ekf_off_event();   // 清除失效保护
        return;
    }

    // 比较罗盘和速度方差与阈值
    if (ekf_over_threshold()) {
        // 如果罗盘尚未被标记为不良
        if (!ekf_check_state.bad_variance) {
            // 增加计数器
            ekf_check_state.fail_count++;
            if (ekf_check_state.fail_count == (EKF_CHECK_ITERATIONS_MAX-2)) {
                // 距离声明 EKF 失效保护还有两次迭代，询问 EKF 是否可以重置偏航以解决问题
                ahrs.request_yaw_reset();
            }
            if (ekf_check_state.fail_count == (EKF_CHECK_ITERATIONS_MAX-1)) {
                // 即将声明 EKF 失效保护，询问 EKF 是否可以切换通道以解决问题
                ahrs.check_lane_switch();
            }
            // 如果计数器超过最大值，则触发失效保护
            if (ekf_check_state.fail_count >= EKF_CHECK_ITERATIONS_MAX) {
                // 限制计数器不要攀升太高
                ekf_check_state.fail_count = EKF_CHECK_ITERATIONS_MAX;
                ekf_check_state.bad_variance = true;
                LOGGER_WRITE_ERROR(LogErrorSubsystem::EKFCHECK, LogErrorCode::EKFCHECK_BAD_VARIANCE);
                // 发送消息到地面站
                if ((AP_HAL::millis() - ekf_check_state.last_warn_time) > EKF_CHECK_WARNING_TIME) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL,"EKF variance");
                    ekf_check_state.last_warn_time = AP_HAL::millis();
                }
                failsafe_ekf_event();
            }
        }
    } else {
        // 减少计数器
        if (ekf_check_state.fail_count > 0) {
            ekf_check_state.fail_count--;

            // 如果罗盘被标记为不良且计数器达到零，则清除标记
            if (ekf_check_state.bad_variance && ekf_check_state.fail_count == 0) {
                ekf_check_state.bad_variance = false;
                LOGGER_WRITE_ERROR(LogErrorSubsystem::EKFCHECK, LogErrorCode::EKFCHECK_VARIANCE_CLEARED);
                // 清除失效保护
                failsafe_ekf_off_event();
            }
        }
    }

    // 设置 AP_Notify 标志
    AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;

    // 待办：将 EKF 方差添加到扩展状态
}

// ekf_over_threshold - 如果 EKF 的方差超过容差，则返回 true
bool Plane::ekf_over_threshold()
{
    // 如果禁用，立即返回 false
    if (g2.fs_ekf_thresh <= 0.0f) {
        return false;
    }

    // 获取相对于创新测试限制归一化的 EKF 创新
    // 大于 1.0 的值意味着 EKF 已拒绝该传感器数据
    float position_variance, vel_variance, height_variance, tas_variance;
    Vector3f mag_variance;
    if (!ahrs.get_variances(vel_variance, position_variance, height_variance, mag_variance, tas_variance)) {
        return false;
    };

    // 如果任何单轴超过限制，EKF 会拒绝所有磁力计轴
    // 因此取所有轴的最大值
    const float mag_max = fmaxf(fmaxf(mag_variance.x,mag_variance.y),mag_variance.z);

    // 根据严重程度为每个超过阈值的项分配分数
    uint8_t over_thresh_count = 0;
    if (mag_max >= g2.fs_ekf_thresh) {
        over_thresh_count++;
    }

    if (vel_variance >= (2.0f * g2.fs_ekf_thresh)) {
        over_thresh_count += 2;
    } else if (vel_variance >= g2.fs_ekf_thresh) {
        over_thresh_count++;
    }

    // 位置是最重要的，如果位置失败，则接受其他传感器较低的分数
    if ((position_variance >= g2.fs_ekf_thresh && over_thresh_count >= 1) || over_thresh_count >= 2) {
        return true;
    }

    return false;
}

// failsafe_ekf_event - 执行 EKF 失效保护
void Plane::failsafe_ekf_event()
{
    // 如果 EKF 失效保护已触发，立即返回
    if (ekf_check_state.failsafe_on) {
        return;
    }

    // EKF 失效保护事件已发生
    ekf_check_state.failsafe_on = true;
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_EKFINAV, LogErrorCode::FAILSAFE_OCCURRED);

    // 如果不在需要位置的 VTOL 模式下，则无需执行任何操作
#if HAL_QUADPLANE_ENABLED
    if (!quadplane.in_vtol_posvel_mode()) {
        return;
    }

    if (quadplane.in_vtol_auto()) {
        // 飞行员不通过摇杆控制，因此切换到 QLAND 模式
        plane.set_mode(mode_qland, ModeReason::EKF_FAILSAFE);
    } else {
        // 飞行员通过摇杆控制，因此回退到 QHOVER 模式
        plane.set_mode(mode_qhover, ModeReason::EKF_FAILSAFE);
    }
#endif
}

// failsafe_ekf_off_event - EKF 失效保护清除时要采取的行动
void Plane::failsafe_ekf_off_event(void)
{
    // 如果不在 EKF 失效保护状态，立即返回
    if (!ekf_check_state.failsafe_on) {
        return;
    }

    ekf_check_state.failsafe_on = false;
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_EKFINAV, LogErrorCode::FAILSAFE_RESOLVED);
}
