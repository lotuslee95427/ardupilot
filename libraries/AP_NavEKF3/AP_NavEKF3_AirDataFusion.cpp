#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF3.h"
#include "AP_NavEKF3_core.h"
#include <AP_DAL/AP_DAL.h>

/********************************************************
*                   RESET FUNCTIONS                     *
********************************************************/

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/

/*
 * Fuse true airspeed measurements using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m
 * 使用Matlab符号工具箱生成的显式代数方程融合真实空速测量。
 * 用于生成这些方程和其他滤波器方程的脚本文件可以在这里找到:
 * https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m
*/
void NavEKF3_core::FuseAirspeed()
{
    // declarations
    // 声明变量
    ftype vn;
    ftype ve;
    ftype vd;
    ftype vwn;
    ftype vwe;
    ftype SH_TAS[3];
    ftype SK_TAS[2];
    Vector24 H_TAS = {};
    ftype VtasPred;

    // copy required states to local variable names
    // 将所需状态复制到局部变量名
    vn = stateStruct.velocity.x;
    ve = stateStruct.velocity.y;
    vd = stateStruct.velocity.z;
    vwn = stateStruct.wind_vel.x;
    vwe = stateStruct.wind_vel.y;

    // calculate the predicted airspeed
    // 计算预测的空速
    VtasPred = norm((ve - vwe) , (vn - vwn) , vd);
    // perform fusion of True Airspeed measurement
    // 执行真实空速测量的融合
    if (VtasPred > 1.0f)
    {
        // calculate observation innovation and variance
        // 计算观测创新和方差
        innovVtas = VtasPred - tasDataDelayed.tas;

        // calculate observation jacobians
        // 计算观测雅可比矩阵
        SH_TAS[0] = 1.0f/VtasPred;
        SH_TAS[1] = (SH_TAS[0]*(2.0f*ve - 2.0f*vwe))*0.5f;
        SH_TAS[2] = (SH_TAS[0]*(2.0f*vn - 2.0f*vwn))*0.5f;
        H_TAS[4] = SH_TAS[2];
        H_TAS[5] = SH_TAS[1];
        H_TAS[6] = vd*SH_TAS[0];
        H_TAS[22] = -SH_TAS[2];
        H_TAS[23] = -SH_TAS[1];
        // calculate Kalman gains
        // 计算卡尔曼增益
        ftype temp = (tasDataDelayed.tasVariance + SH_TAS[2]*(P[4][4]*SH_TAS[2] + P[5][4]*SH_TAS[1] - P[22][4]*SH_TAS[2] - P[23][4]*SH_TAS[1] + P[6][4]*vd*SH_TAS[0]) + SH_TAS[1]*(P[4][5]*SH_TAS[2] + P[5][5]*SH_TAS[1] - P[22][5]*SH_TAS[2] - P[23][5]*SH_TAS[1] + P[6][5]*vd*SH_TAS[0]) - SH_TAS[2]*(P[4][22]*SH_TAS[2] + P[5][22]*SH_TAS[1] - P[22][22]*SH_TAS[2] - P[23][22]*SH_TAS[1] + P[6][22]*vd*SH_TAS[0]) - SH_TAS[1]*(P[4][23]*SH_TAS[2] + P[5][23]*SH_TAS[1] - P[22][23]*SH_TAS[2] - P[23][23]*SH_TAS[1] + P[6][23]*vd*SH_TAS[0]) + vd*SH_TAS[0]*(P[4][6]*SH_TAS[2] + P[5][6]*SH_TAS[1] - P[22][6]*SH_TAS[2] - P[23][6]*SH_TAS[1] + P[6][6]*vd*SH_TAS[0]));
        if (temp >= tasDataDelayed.tasVariance) {
            SK_TAS[0] = 1.0f / temp;
            faultStatus.bad_airspeed = false;
        } else {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we reset the covariance matrix and try again next measurement
            // 计算条件不好,因此我们不能在这一步执行融合
            // 我们重置协方差矩阵并在下一次测量时重试
            CovarianceInit();
            faultStatus.bad_airspeed = true;
            return;
        }
        SK_TAS[1] = SH_TAS[1];

        if (tasDataDelayed.allowFusion && !airDataFusionWindOnly) {
            // 如果允许融合且不是仅风速融合模式,计算卡尔曼增益
            Kfusion[0] = SK_TAS[0]*(P[0][4]*SH_TAS[2] - P[0][22]*SH_TAS[2] + P[0][5]*SK_TAS[1] - P[0][23]*SK_TAS[1] + P[0][6]*vd*SH_TAS[0]);
            Kfusion[1] = SK_TAS[0]*(P[1][4]*SH_TAS[2] - P[1][22]*SH_TAS[2] + P[1][5]*SK_TAS[1] - P[1][23]*SK_TAS[1] + P[1][6]*vd*SH_TAS[0]);
            Kfusion[2] = SK_TAS[0]*(P[2][4]*SH_TAS[2] - P[2][22]*SH_TAS[2] + P[2][5]*SK_TAS[1] - P[2][23]*SK_TAS[1] + P[2][6]*vd*SH_TAS[0]);
            Kfusion[3] = SK_TAS[0]*(P[3][4]*SH_TAS[2] - P[3][22]*SH_TAS[2] + P[3][5]*SK_TAS[1] - P[3][23]*SK_TAS[1] + P[3][6]*vd*SH_TAS[0]);
            Kfusion[4] = SK_TAS[0]*(P[4][4]*SH_TAS[2] - P[4][22]*SH_TAS[2] + P[4][5]*SK_TAS[1] - P[4][23]*SK_TAS[1] + P[4][6]*vd*SH_TAS[0]);
            Kfusion[5] = SK_TAS[0]*(P[5][4]*SH_TAS[2] - P[5][22]*SH_TAS[2] + P[5][5]*SK_TAS[1] - P[5][23]*SK_TAS[1] + P[5][6]*vd*SH_TAS[0]);
            Kfusion[6] = SK_TAS[0]*(P[6][4]*SH_TAS[2] - P[6][22]*SH_TAS[2] + P[6][5]*SK_TAS[1] - P[6][23]*SK_TAS[1] + P[6][6]*vd*SH_TAS[0]);
            Kfusion[7] = SK_TAS[0]*(P[7][4]*SH_TAS[2] - P[7][22]*SH_TAS[2] + P[7][5]*SK_TAS[1] - P[7][23]*SK_TAS[1] + P[7][6]*vd*SH_TAS[0]);
            Kfusion[8] = SK_TAS[0]*(P[8][4]*SH_TAS[2] - P[8][22]*SH_TAS[2] + P[8][5]*SK_TAS[1] - P[8][23]*SK_TAS[1] + P[8][6]*vd*SH_TAS[0]);
            Kfusion[9] = SK_TAS[0]*(P[9][4]*SH_TAS[2] - P[9][22]*SH_TAS[2] + P[9][5]*SK_TAS[1] - P[9][23]*SK_TAS[1] + P[9][6]*vd*SH_TAS[0]);
        } else {
            // zero indexes 0 to 9
            // 将索引0到9置零
            zero_range(&Kfusion[0], 0, 9);
        }

        if (tasDataDelayed.allowFusion && !inhibitDelAngBiasStates && !airDataFusionWindOnly) {
            // 如果允许融合且不禁止角速度偏差状态且不是仅风速融合模式
            Kfusion[10] = SK_TAS[0]*(P[10][4]*SH_TAS[2] - P[10][22]*SH_TAS[2] + P[10][5]*SK_TAS[1] - P[10][23]*SK_TAS[1] + P[10][6]*vd*SH_TAS[0]);
            Kfusion[11] = SK_TAS[0]*(P[11][4]*SH_TAS[2] - P[11][22]*SH_TAS[2] + P[11][5]*SK_TAS[1] - P[11][23]*SK_TAS[1] + P[11][6]*vd*SH_TAS[0]);
            Kfusion[12] = SK_TAS[0]*(P[12][4]*SH_TAS[2] - P[12][22]*SH_TAS[2] + P[12][5]*SK_TAS[1] - P[12][23]*SK_TAS[1] + P[12][6]*vd*SH_TAS[0]);
        } else {
            // zero indexes 10 to 12
            // 将索引10到12置零
            zero_range(&Kfusion[0], 10, 12);
        }

        if (tasDataDelayed.allowFusion && !inhibitDelVelBiasStates && !airDataFusionWindOnly) {
            // 如果允许融合且不禁止速度偏差状态且不是仅风速融合模式
            for (uint8_t index = 0; index < 3; index++) {
                const uint8_t stateIndex = index + 13;
                if (!dvelBiasAxisInhibit[index]) {
                    Kfusion[stateIndex] = SK_TAS[0]*(P[stateIndex][4]*SH_TAS[2] - P[stateIndex][22]*SH_TAS[2] + P[stateIndex][5]*SK_TAS[1] - P[stateIndex][23]*SK_TAS[1] + P[stateIndex][6]*vd*SH_TAS[0]);
                } else {
                    Kfusion[stateIndex] = 0.0f;
                }
            }
        } else {
            // zero indexes 13 to 15
            // 将索引13到15置零
            zero_range(&Kfusion[0], 13, 15);
        }

        // zero Kalman gains to inhibit magnetic field state estimation
        // 将卡尔曼增益置零以抑制磁场状态估计
        if (tasDataDelayed.allowFusion && !inhibitMagStates && !airDataFusionWindOnly) {
            Kfusion[16] = SK_TAS[0]*(P[16][4]*SH_TAS[2] - P[16][22]*SH_TAS[2] + P[16][5]*SK_TAS[1] - P[16][23]*SK_TAS[1] + P[16][6]*vd*SH_TAS[0]);
            Kfusion[17] = SK_TAS[0]*(P[17][4]*SH_TAS[2] - P[17][22]*SH_TAS[2] + P[17][5]*SK_TAS[1] - P[17][23]*SK_TAS[1] + P[17][6]*vd*SH_TAS[0]);
            Kfusion[18] = SK_TAS[0]*(P[18][4]*SH_TAS[2] - P[18][22]*SH_TAS[2] + P[18][5]*SK_TAS[1] - P[18][23]*SK_TAS[1] + P[18][6]*vd*SH_TAS[0]);
            Kfusion[19] = SK_TAS[0]*(P[19][4]*SH_TAS[2] - P[19][22]*SH_TAS[2] + P[19][5]*SK_TAS[1] - P[19][23]*SK_TAS[1] + P[19][6]*vd*SH_TAS[0]);
            Kfusion[20] = SK_TAS[0]*(P[20][4]*SH_TAS[2] - P[20][22]*SH_TAS[2] + P[20][5]*SK_TAS[1] - P[20][23]*SK_TAS[1] + P[20][6]*vd*SH_TAS[0]);
            Kfusion[21] = SK_TAS[0]*(P[21][4]*SH_TAS[2] - P[21][22]*SH_TAS[2] + P[21][5]*SK_TAS[1] - P[21][23]*SK_TAS[1] + P[21][6]*vd*SH_TAS[0]);
        } else {
            // zero indexes 16 to 21
            // 将索引16到21置零
            zero_range(&Kfusion[0], 16, 21);
        }

        if (tasDataDelayed.allowFusion && !inhibitWindStates && !treatWindStatesAsTruth) {
            // 如果允许融合且不禁止风速状态且不将风速状态视为真值
            Kfusion[22] = SK_TAS[0]*(P[22][4]*SH_TAS[2] - P[22][22]*SH_TAS[2] + P[22][5]*SK_TAS[1] - P[22][23]*SK_TAS[1] + P[22][6]*vd*SH_TAS[0]);
            Kfusion[23] = SK_TAS[0]*(P[23][4]*SH_TAS[2] - P[23][22]*SH_TAS[2] + P[23][5]*SK_TAS[1] - P[23][23]*SK_TAS[1] + P[23][6]*vd*SH_TAS[0]);
        } else {
            // zero indexes 22 to 23 = 2
            // 将索引22到23置零
            zero_range(&Kfusion[0], 22, 23);
        }

        // calculate measurement innovation variance
        // 计算测量创新方差
        varInnovVtas = 1.0f/SK_TAS[0];

        // calculate the innovation consistency test ratio
        // 计算创新一致性测试比率
        tasTestRatio = sq(innovVtas) / (sq(MAX(0.01f * (ftype)frontend->_tasInnovGate, 1.0f)) * varInnovVtas);

        // fail if the ratio is > 1, but don't fail if bad IMU data
        // 如果比率>1则失败,但如果IMU数据不好则不失败
        const bool isConsistent = (tasTestRatio < 1.0f) || badIMUdata;
        tasTimeout = (imuSampleTime_ms - lastTasPassTime_ms) > frontend->tasRetryTime_ms;
        if (!isConsistent) {
            lastTasFailTime_ms = imuSampleTime_ms;
        } else {
            lastTasFailTime_ms = 0;
        }

        // test the ratio before fusing data, forcing fusion if airspeed and position are timed out as we have no choice but to try and use airspeed to constrain error growth
        // 在融合数据之前测试比率,如果空速和位置超时则强制融合,因为我们别无选择只能尝试使用空速来约束误差增长
        if (tasDataDelayed.allowFusion && (isConsistent || (tasTimeout && posTimeout))) {

            // restart the counter
            // 重启计数器
            lastTasPassTime_ms = imuSampleTime_ms;

            // correct the state vector
            // 修正状态向量
            for (uint8_t j= 0; j<=stateIndexLim; j++) {
                statesArray[j] = statesArray[j] - Kfusion[j] * innovVtas;
            }
            stateStruct.quat.normalize();

            // correct the covariance P = (I - K*H)*P
            // take advantage of the empty columns in KH to reduce the
            // number of operations
            // 修正协方差P = (I - K*H)*P
            // 利用KH中的空列来减少操作次数
            for (unsigned i = 0; i<=stateIndexLim; i++) {
                for (unsigned j = 0; j<=3; j++) {
                    KH[i][j] = 0.0f;
                }
                for (unsigned j = 4; j<=6; j++) {
                    KH[i][j] = Kfusion[i] * H_TAS[j];
                }
                for (unsigned j = 7; j<=21; j++) {
                    KH[i][j] = 0.0f;
                }
                for (unsigned j = 22; j<=23; j++) {
                    KH[i][j] = Kfusion[i] * H_TAS[j];
                }
            }
            for (unsigned j = 0; j<=stateIndexLim; j++) {
                for (unsigned i = 0; i<=stateIndexLim; i++) {
                    ftype res = 0;
                    res += KH[i][4] * P[4][j];
                    res += KH[i][5] * P[5][j];
                    res += KH[i][6] * P[6][j];
                    res += KH[i][22] * P[22][j];
                    res += KH[i][23] * P[23][j];
                    KHP[i][j] = res;
                }
            }
            for (unsigned i = 0; i<=stateIndexLim; i++) {
                for (unsigned j = 0; j<=stateIndexLim; j++) {
                    P[i][j] = P[i][j] - KHP[i][j];
                }
            }
        }
        // force the covariance matrix to be symmetrical and limit the variances to prevent ill-conditioning.
        // 强制协方差矩阵对称并限制方差以防止病态
        ForceSymmetry();
        ConstrainVariances();
    }
}

// select fusion of true airspeed measurements
// 选择真实空速测量的融合
void NavEKF3_core::SelectTasFusion()
{
    // Check if the magnetometer has been fused on that time step and the filter is running at faster than 200 Hz
    // If so, don't fuse measurements on this time step to reduce frame over-runs
    // Only allow one time slip to prevent high rate magnetometer data locking out fusion of other measurements
    // 检查磁力计是否在该时间步上已融合且滤波器运行速度快于200 Hz
    // 如果是,则不在此时间步上融合测量以减少帧超限
    // 仅允许一次时间滑移以防止高速率磁力计数据锁定其他测量的融合
    if (magFusePerformed && dtIMUavg < 0.005f && !airSpdFusionDelayed) {
        airSpdFusionDelayed = true;
        return;
    } else {
        airSpdFusionDelayed = false;
    }

    // get true airspeed measurement
    // 获取真实空速测量
    readAirSpdData();

    // if the filter is initialised, wind states are not inhibited and we have data to fuse, then perform TAS fusion
    // 如果滤波器已初始化,风速状态未被禁止且我们有数据要融合,则执行TAS融合
    if (tasDataToFuse && statesInitialised && !inhibitWindStates) {
        FuseAirspeed();
        tasDataToFuse = false;
        prevTasStep_ms = imuSampleTime_ms;
    }
}


// select fusion of synthetic sideslip measurements or body frame drag
// synthetic sidelip fusion only works for fixed wing aircraft and relies on the average sideslip being close to zero
// body frame drag only works for bluff body multi rotor vehices with thrust forces aligned with the Z axis
// it requires a stable wind for best results and should not be used for aerobatic flight
// 选择合成侧滑测量或机体坐标系阻力的融合
// 合成侧滑融合仅适用于固定翼飞机,并依赖于平均侧滑接近零
// 机体坐标系阻力仅适用于推力力与Z轴对齐的钝体多旋翼飞行器
// 它需要稳定的风以获得最佳结果,不应用于特技飞行
void NavEKF3_core::SelectBetaDragFusion()
{
    // Check if the magnetometer has been fused on that time step and the filter is running at faster than 200 Hz
    // If so, don't fuse measurements on this time step to reduce frame over-runs
    // Only allow one time slip to prevent high rate magnetometer data preventing fusion of other measurements
    // 检查磁力计是否在该时间步上已融合且滤波器运行速度快于200 Hz
    // 如果是,则不在此时间步上融合测量以减少帧超限
    // 仅允许一次时间滑移以防止高速率磁力计数据阻止其他测量的融合
    if (magFusePerformed && dtIMUavg < 0.005f && !sideSlipFusionDelayed) {
        sideSlipFusionDelayed = true;
        return;
    } else {
        sideSlipFusionDelayed = false;
    }

    // set true when the fusion time interval has triggered
    // 当融合时间间隔触发时设为true
    bool f_timeTrigger = ((imuSampleTime_ms - prevBetaDragStep_ms) >= frontend->betaAvg_ms);

    // use of air data to constrain drift is necessary if we have limited sensor data or are doing inertial dead reckoning
    // 如果我们有限的传感器数据或正在进行惯性航位推算,则需要使用空气数据来约束漂移
    bool is_dead_reckoning = ((imuSampleTime_ms - lastGpsPosPassTime_ms) > frontend->deadReckonDeclare_ms) &&
                             ((imuSampleTime_ms - lastVelPassTime_ms) > frontend->deadReckonDeclare_ms);
    const bool noYawSensor = !use_compass() && !using_noncompass_for_yaw();
    const bool f_required = (noYawSensor && (frontend->_betaMask & (1<<1))) || is_dead_reckoning;

    // set true when sideslip fusion is feasible (requires zero sideslip assumption to be valid and use of wind states)
    // 当侧滑融合可行时设为true(需要零侧滑假设有效且使用风速状态)
    const bool f_beta_feasible = (assume_zero_sideslip() && !inhibitWindStates);

    // use synthetic sideslip fusion if feasible, required and enough time has lapsed since the last fusion
    // 如果可行、需要且自上次融合以来已经过足够时间,则使用合成侧滑融合
    if (f_beta_feasible && f_timeTrigger) {
        // unless air data is required to constrain drift, it is only used to update wind state estimates
        // 除非需要空气数据来约束漂移,否则它仅用于更新风速状态估计
        if (f_required || (frontend->_betaMask & (1<<0))) {
            // we are required to correct all states
            // 我们需要修正所有状态
            airDataFusionWindOnly = false;
        } else {
            // we are required to correct only wind states
            // 我们仅需要修正风速状态
            airDataFusionWindOnly = true;
        }
        FuseSideslip();
        prevBetaDragStep_ms = imuSampleTime_ms;
    }

#if EK3_FEATURE_DRAG_FUSION
    // fusion of XY body frame aero specific forces is done at a slower rate and only if alternative methods of wind estimation are not available
    // XY机体坐标系空气动力学特定力的融合以较慢的速率进行,且仅在无法使用其他风速估计方法时进行
    if (!inhibitWindStates && storedDrag.recall(dragSampleDelayed,imuDataDelayed.time_ms)) {
        FuseDragForces();
    }
    dragTimeout = (imuSampleTime_ms - lastDragPassTime_ms) > frontend->dragFailTimeLimit_ms;
#endif
}

/*
 * Fuse sythetic sideslip measurement of zero using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m
 * 使用Matlab符号工具箱生成的显式代数方程融合零合成侧滑测量。
 * 用于生成这些方程和其他滤波器方程的脚本文件可以在这里找到:
 * https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m
*/
void NavEKF3_core::FuseSideslip()
{
    // declarations
    // 声明变量
    ftype q0;
    ftype q1;
    ftype q2;
    ftype q3;
    ftype vn;
    ftype ve;
    ftype vd;
    ftype vwn;
    ftype vwe;
    const ftype R_BETA = 0.03f; // assume a sideslip angle RMS of ~10 deg
    Vector13 SH_BETA;
    Vector8 SK_BETA;
    Vector3F vel_rel_wind;
    Vector24 H_BETA;

    // copy required states to local variable names
    // 将所需状态复制到局部变量名
    q0 = stateStruct.quat[0];
    q1 = stateStruct.quat[1];
    q2 = stateStruct.quat[2];
    q3 = stateStruct.quat[3];
    vn = stateStruct.velocity.x;
    ve = stateStruct.velocity.y;
    vd = stateStruct.velocity.z;
    vwn = stateStruct.wind_vel.x;
    vwe = stateStruct.wind_vel.y;

    // calculate predicted wind relative velocity in NED
    // 计算NED坐标系中预测的相对风速
    vel_rel_wind.x = vn - vwn;
    vel_rel_wind.y = ve - vwe;
    vel_rel_wind.z = vd;

    // rotate into body axes
    // 旋转到机体坐标系
    vel_rel_wind = prevTnb * vel_rel_wind;

    // perform fusion of assumed sideslip  = 0
    // 执行假设侧滑角=0的融合
    if (vel_rel_wind.x > 5.0f)
    {
        // Calculate observation jacobians
        // 计算观测雅可比矩阵
        // Calculate observation jacobians and Kalman gains
        // 计算观测雅可比矩阵和卡尔曼增益
        SH_BETA[0] = (vn - vwn)*(sq(q0) + sq(q1) - sq(q2) - sq(q3)) - vd*(2*q0*q2 - 2*q1*q3) + (ve - vwe)*(2*q0*q3 + 2*q1*q2);
        if (fabsF(SH_BETA[0]) <= 1e-9f) {
            faultStatus.bad_sideslip = true;
            return;
        } else {
            faultStatus.bad_sideslip = false;
        }
        // 计算中间变量SH_BETA，用于后续计算
        SH_BETA[1] = (ve - vwe)*(sq(q0) - sq(q1) + sq(q2) - sq(q3)) + vd*(2*q0*q1 + 2*q2*q3) - (vn - vwn)*(2*q0*q3 - 2*q1*q2);
        SH_BETA[2] = vn - vwn;
        SH_BETA[3] = ve - vwe;
        SH_BETA[4] = 1/sq(SH_BETA[0]);
        SH_BETA[5] = 1/SH_BETA[0];
        SH_BETA[6] = SH_BETA[5]*(sq(q0) - sq(q1) + sq(q2) - sq(q3));
        SH_BETA[7] = sq(q0) + sq(q1) - sq(q2) - sq(q3);
        SH_BETA[8] = 2*q0*SH_BETA[3] - 2*q3*SH_BETA[2] + 2*q1*vd;
        SH_BETA[9] = 2*q0*SH_BETA[2] + 2*q3*SH_BETA[3] - 2*q2*vd;
        SH_BETA[10] = 2*q2*SH_BETA[2] - 2*q1*SH_BETA[3] + 2*q0*vd;
        SH_BETA[11] = 2*q1*SH_BETA[2] + 2*q2*SH_BETA[3] + 2*q3*vd;
        SH_BETA[12] = 2*q0*q3;

        // 计算观测雅可比矩阵H_BETA
        H_BETA[0] = SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9];
        H_BETA[1] = SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11];
        H_BETA[2] = SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10];
        H_BETA[3] = - SH_BETA[5]*SH_BETA[9] - SH_BETA[1]*SH_BETA[4]*SH_BETA[8];
        H_BETA[4] = - SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) - SH_BETA[1]*SH_BETA[4]*SH_BETA[7];
        H_BETA[5] = SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2);
        H_BETA[6] = SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3);
        // 将H_BETA的7到21索引设置为0
        for (uint8_t i=7; i<=21; i++) {
            H_BETA[i] = 0.0f;
        }
        H_BETA[22] = SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7];
        H_BETA[23] = SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2) - SH_BETA[6];

        // Calculate Kalman gains
        // 计算卡尔曼增益
        ftype temp = (R_BETA - (SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7])*(P[22][4]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][4]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][4]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[23][4]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][4]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][4]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][4]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][4]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][4]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) + (SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7])*(P[22][22]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][22]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][22]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[23][22]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][22]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][22]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][22]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][22]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][22]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) + (SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2))*(P[22][5]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][5]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][5]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[23][5]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][5]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][5]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][5]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][5]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][5]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) - (SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2))*(P[22][23]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][23]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][23]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[23][23]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][23]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][23]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][23]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][23]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][23]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) + (SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9])*(P[22][0]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][0]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][0]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[23][0]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][0]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][0]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][0]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][0]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][0]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) + (SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11])*(P[22][1]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][1]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][1]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[23][1]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][1]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][1]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][1]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][1]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][1]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) + (SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10])*(P[22][2]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][2]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][2]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[23][2]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][2]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][2]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][2]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][2]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][2]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) - (SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8])*(P[22][3]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][3]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][3]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[23][3]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][3]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][3]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][3]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][3]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][3]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) + (SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))*(P[22][6]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][6]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][6]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[23][6]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][6]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][6]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][6]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][6]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][6]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))));
        if (temp >= R_BETA) {
            SK_BETA[0] = 1.0f / temp;
            faultStatus.bad_sideslip = false;
        } else {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we reset the covariance matrix and try again next measurement
            // 计算条件不好,无法进行融合,重置协方差矩阵并等待下一次测量
            CovarianceInit();
            faultStatus.bad_sideslip = true;
            return;
        }
        // 计算中间变量SK_BETA
        SK_BETA[1] = SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7];
        SK_BETA[2] = SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2);
        SK_BETA[3] = SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3);
        SK_BETA[4] = SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11];
        SK_BETA[5] = SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9];
        SK_BETA[6] = SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10];
        SK_BETA[7] = SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8];

        // 如果不仅仅使用风速状态进行空气数据融合
        if (!airDataFusionWindOnly) {
            // 计算卡尔曼增益
            Kfusion[0] = SK_BETA[0]*(P[0][0]*SK_BETA[5] + P[0][1]*SK_BETA[4] - P[0][4]*SK_BETA[1] + P[0][5]*SK_BETA[2] + P[0][2]*SK_BETA[6] + P[0][6]*SK_BETA[3] - P[0][3]*SK_BETA[7] + P[0][22]*SK_BETA[1] - P[0][23]*SK_BETA[2]);
            Kfusion[1] = SK_BETA[0]*(P[1][0]*SK_BETA[5] + P[1][1]*SK_BETA[4] - P[1][4]*SK_BETA[1] + P[1][5]*SK_BETA[2] + P[1][2]*SK_BETA[6] + P[1][6]*SK_BETA[3] - P[1][3]*SK_BETA[7] + P[1][22]*SK_BETA[1] - P[1][23]*SK_BETA[2]);
            Kfusion[2] = SK_BETA[0]*(P[2][0]*SK_BETA[5] + P[2][1]*SK_BETA[4] - P[2][4]*SK_BETA[1] + P[2][5]*SK_BETA[2] + P[2][2]*SK_BETA[6] + P[2][6]*SK_BETA[3] - P[2][3]*SK_BETA[7] + P[2][22]*SK_BETA[1] - P[2][23]*SK_BETA[2]);
            Kfusion[3] = SK_BETA[0]*(P[3][0]*SK_BETA[5] + P[3][1]*SK_BETA[4] - P[3][4]*SK_BETA[1] + P[3][5]*SK_BETA[2] + P[3][2]*SK_BETA[6] + P[3][6]*SK_BETA[3] - P[3][3]*SK_BETA[7] + P[3][22]*SK_BETA[1] - P[3][23]*SK_BETA[2]);
            Kfusion[4] = SK_BETA[0]*(P[4][0]*SK_BETA[5] + P[4][1]*SK_BETA[4] - P[4][4]*SK_BETA[1] + P[4][5]*SK_BETA[2] + P[4][2]*SK_BETA[6] + P[4][6]*SK_BETA[3] - P[4][3]*SK_BETA[7] + P[4][22]*SK_BETA[1] - P[4][23]*SK_BETA[2]);
            Kfusion[5] = SK_BETA[0]*(P[5][0]*SK_BETA[5] + P[5][1]*SK_BETA[4] - P[5][4]*SK_BETA[1] + P[5][5]*SK_BETA[2] + P[5][2]*SK_BETA[6] + P[5][6]*SK_BETA[3] - P[5][3]*SK_BETA[7] + P[5][22]*SK_BETA[1] - P[5][23]*SK_BETA[2]);
            Kfusion[6] = SK_BETA[0]*(P[6][0]*SK_BETA[5] + P[6][1]*SK_BETA[4] - P[6][4]*SK_BETA[1] + P[6][5]*SK_BETA[2] + P[6][2]*SK_BETA[6] + P[6][6]*SK_BETA[3] - P[6][3]*SK_BETA[7] + P[6][22]*SK_BETA[1] - P[6][23]*SK_BETA[2]);
            Kfusion[7] = SK_BETA[0]*(P[7][0]*SK_BETA[5] + P[7][1]*SK_BETA[4] - P[7][4]*SK_BETA[1] + P[7][5]*SK_BETA[2] + P[7][2]*SK_BETA[6] + P[7][6]*SK_BETA[3] - P[7][3]*SK_BETA[7] + P[7][22]*SK_BETA[1] - P[7][23]*SK_BETA[2]);
            Kfusion[8] = SK_BETA[0]*(P[8][0]*SK_BETA[5] + P[8][1]*SK_BETA[4] - P[8][4]*SK_BETA[1] + P[8][5]*SK_BETA[2] + P[8][2]*SK_BETA[6] + P[8][6]*SK_BETA[3] - P[8][3]*SK_BETA[7] + P[8][22]*SK_BETA[1] - P[8][23]*SK_BETA[2]);
            Kfusion[9] = SK_BETA[0]*(P[9][0]*SK_BETA[5] + P[9][1]*SK_BETA[4] - P[9][4]*SK_BETA[1] + P[9][5]*SK_BETA[2] + P[9][2]*SK_BETA[6] + P[9][6]*SK_BETA[3] - P[9][3]*SK_BETA[7] + P[9][22]*SK_BETA[1] - P[9][23]*SK_BETA[2]);
        } else {
            // zero indexes 0 to 9
            // 将索引0到9置零
            zero_range(&Kfusion[0], 0, 9);
        }

        if (!inhibitDelAngBiasStates && !airDataFusionWindOnly) {
            // 如果不禁止角速度偏差状态且不是仅风速融合模式
            Kfusion[10] = SK_BETA[0]*(P[10][0]*SK_BETA[5] + P[10][1]*SK_BETA[4] - P[10][4]*SK_BETA[1] + P[10][5]*SK_BETA[2] + P[10][2]*SK_BETA[6] + P[10][6]*SK_BETA[3] - P[10][3]*SK_BETA[7] + P[10][22]*SK_BETA[1] - P[10][23]*SK_BETA[2]);
            Kfusion[11] = SK_BETA[0]*(P[11][0]*SK_BETA[5] + P[11][1]*SK_BETA[4] - P[11][4]*SK_BETA[1] + P[11][5]*SK_BETA[2] + P[11][2]*SK_BETA[6] + P[11][6]*SK_BETA[3] - P[11][3]*SK_BETA[7] + P[11][22]*SK_BETA[1] - P[11][23]*SK_BETA[2]);
            Kfusion[12] = SK_BETA[0]*(P[12][0]*SK_BETA[5] + P[12][1]*SK_BETA[4] - P[12][4]*SK_BETA[1] + P[12][5]*SK_BETA[2] + P[12][2]*SK_BETA[6] + P[12][6]*SK_BETA[3] - P[12][3]*SK_BETA[7] + P[12][22]*SK_BETA[1] - P[12][23]*SK_BETA[2]);
        } else {
            // zero indexes 10 to 12 = 3
            // 将索引10到12置零
            zero_range(&Kfusion[0], 10, 12);
        }

        if (!inhibitDelVelBiasStates && !airDataFusionWindOnly) {
            // 如果不禁止速度偏差状态且不是仅风速融合模式
            for (uint8_t index = 0; index < 3; index++) {
                const uint8_t stateIndex = index + 13;
                if (!dvelBiasAxisInhibit[index]) {
                    // 如果该轴的速度偏差未被禁止,计算卡尔曼增益
                    Kfusion[stateIndex] = SK_BETA[0]*(P[stateIndex][0]*SK_BETA[5] + P[stateIndex][1]*SK_BETA[4] - P[stateIndex][4]*SK_BETA[1] + P[stateIndex][5]*SK_BETA[2] + P[stateIndex][2]*SK_BETA[6] + P[stateIndex][6]*SK_BETA[3] - P[stateIndex][3]*SK_BETA[7] + P[stateIndex][22]*SK_BETA[1] - P[stateIndex][23]*SK_BETA[2]);
                } else {
                    // 如果该轴的速度偏差被禁止,将增益置零
                    Kfusion[stateIndex] = 0.0f;
                }
            }
        } else {
            // zero indexes 13 to 15
            // 将索引13到15置零
            zero_range(&Kfusion[0], 13, 15);
        }

        // zero Kalman gains to inhibit magnetic field state estimation
        // 将卡尔曼增益置零以抑制磁场状态估计
        if (!inhibitMagStates && !airDataFusionWindOnly) {
            // 如果不禁止磁场状态且不是仅风速融合模式
            Kfusion[16] = SK_BETA[0]*(P[16][0]*SK_BETA[5] + P[16][1]*SK_BETA[4] - P[16][4]*SK_BETA[1] + P[16][5]*SK_BETA[2] + P[16][2]*SK_BETA[6] + P[16][6]*SK_BETA[3] - P[16][3]*SK_BETA[7] + P[16][22]*SK_BETA[1] - P[16][23]*SK_BETA[2]);
            Kfusion[17] = SK_BETA[0]*(P[17][0]*SK_BETA[5] + P[17][1]*SK_BETA[4] - P[17][4]*SK_BETA[1] + P[17][5]*SK_BETA[2] + P[17][2]*SK_BETA[6] + P[17][6]*SK_BETA[3] - P[17][3]*SK_BETA[7] + P[17][22]*SK_BETA[1] - P[17][23]*SK_BETA[2]);
            Kfusion[18] = SK_BETA[0]*(P[18][0]*SK_BETA[5] + P[18][1]*SK_BETA[4] - P[18][4]*SK_BETA[1] + P[18][5]*SK_BETA[2] + P[18][2]*SK_BETA[6] + P[18][6]*SK_BETA[3] - P[18][3]*SK_BETA[7] + P[18][22]*SK_BETA[1] - P[18][23]*SK_BETA[2]);
            Kfusion[19] = SK_BETA[0]*(P[19][0]*SK_BETA[5] + P[19][1]*SK_BETA[4] - P[19][4]*SK_BETA[1] + P[19][5]*SK_BETA[2] + P[19][2]*SK_BETA[6] + P[19][6]*SK_BETA[3] - P[19][3]*SK_BETA[7] + P[19][22]*SK_BETA[1] - P[19][23]*SK_BETA[2]);
            Kfusion[20] = SK_BETA[0]*(P[20][0]*SK_BETA[5] + P[20][1]*SK_BETA[4] - P[20][4]*SK_BETA[1] + P[20][5]*SK_BETA[2] + P[20][2]*SK_BETA[6] + P[20][6]*SK_BETA[3] - P[20][3]*SK_BETA[7] + P[20][22]*SK_BETA[1] - P[20][23]*SK_BETA[2]);
            Kfusion[21] = SK_BETA[0]*(P[21][0]*SK_BETA[5] + P[21][1]*SK_BETA[4] - P[21][4]*SK_BETA[1] + P[21][5]*SK_BETA[2] + P[21][2]*SK_BETA[6] + P[21][6]*SK_BETA[3] - P[21][3]*SK_BETA[7] + P[21][22]*SK_BETA[1] - P[21][23]*SK_BETA[2]);
        } else {
            // zero indexes 16 to 21
            // 将索引16到21置零
            zero_range(&Kfusion[0], 16, 21);
        }

        if (!inhibitWindStates && !treatWindStatesAsTruth) {
            // 如果不禁止风速状态且不将风速状态视为真值
            Kfusion[22] = SK_BETA[0]*(P[22][0]*SK_BETA[5] + P[22][1]*SK_BETA[4] - P[22][4]*SK_BETA[1] + P[22][5]*SK_BETA[2] + P[22][2]*SK_BETA[6] + P[22][6]*SK_BETA[3] - P[22][3]*SK_BETA[7] + P[22][22]*SK_BETA[1] - P[22][23]*SK_BETA[2]);
            Kfusion[23] = SK_BETA[0]*(P[23][0]*SK_BETA[5] + P[23][1]*SK_BETA[4] - P[23][4]*SK_BETA[1] + P[23][5]*SK_BETA[2] + P[23][2]*SK_BETA[6] + P[23][6]*SK_BETA[3] - P[23][3]*SK_BETA[7] + P[23][22]*SK_BETA[1] - P[23][23]*SK_BETA[2]);
        } else {
            // zero indexes 22 to 23
            // 将索引22到23置零
            zero_range(&Kfusion[0], 22, 23);
        }

        // calculate predicted sideslip angle and innovation using small angle approximation
        // 使用小角度近似计算预测的侧滑角和创新
        innovBeta = constrain_ftype(vel_rel_wind.y / vel_rel_wind.x, -0.5f, 0.5f);

        // correct the state vector
        // 修正状态向量
        for (uint8_t j= 0; j<=stateIndexLim; j++) {
            statesArray[j] = statesArray[j] - Kfusion[j] * innovBeta;
        }
        stateStruct.quat.normalize();

        // correct the covariance P = (I - K*H)*P
        // take advantage of the empty columns in KH to reduce the
        // number of operations
        // 修正协方差P = (I - K*H)*P
        // 利用KH中的空列来减少操作次数
        for (unsigned i = 0; i<=stateIndexLim; i++) {
            for (unsigned j = 0; j<=6; j++) {
                KH[i][j] = Kfusion[i] * H_BETA[j];
            }
            for (unsigned j = 7; j<=21; j++) {
                KH[i][j] = 0.0f;
            }
            for (unsigned j = 22; j<=23; j++) {
                KH[i][j] = Kfusion[i] * H_BETA[j];
            }
        }
        for (unsigned j = 0; j<=stateIndexLim; j++) {
            for (unsigned i = 0; i<=stateIndexLim; i++) {
                ftype res = 0;
                res += KH[i][0] * P[0][j];
                res += KH[i][1] * P[1][j];
                res += KH[i][2] * P[2][j];
                res += KH[i][3] * P[3][j];
                res += KH[i][4] * P[4][j];
                res += KH[i][5] * P[5][j];
                res += KH[i][6] * P[6][j];
                res += KH[i][22] * P[22][j];
                res += KH[i][23] * P[23][j];
                KHP[i][j] = res;
            }
        }
        for (unsigned i = 0; i<=stateIndexLim; i++) {
            for (unsigned j = 0; j<=stateIndexLim; j++) {
                P[i][j] = P[i][j] - KHP[i][j];
            }
        }
    }

    // force the covariance matrix to be symmetrical and limit the variances to prevent ill-conditioning.
    // 强制协方差矩阵对称并限制方差以防止病态
    ForceSymmetry();
    ConstrainVariances();
}

#if EK3_FEATURE_DRAG_FUSION
/*
 * Fuse X and Y body axis specific forces using explicit algebraic equations generated with SymPy.
 * See AP_NavEKF3/derivation/main.py for derivation
 * Output for change reference: AP_NavEKF3/derivation/generated/acc_bf_generated.cpp
 * 使用SymPy生成的显式代数方程融合X和Y体轴比力。
 * 推导过程见AP_NavEKF3/derivation/main.py
 * 变更参考输出见AP_NavEKF3/derivation/generated/acc_bf_generated.cpp
*/
void NavEKF3_core::FuseDragForces()
{
    // drag model parameters
    // 阻力模型参数
    const ftype bcoef_x = frontend->_ballisticCoef_x;
    const ftype bcoef_y = frontend->_ballisticCoef_y;
    const ftype mcoef = frontend->_momentumDragCoef.get();
    const bool using_bcoef_x = bcoef_x > 1.0f;
    const bool using_bcoef_y = bcoef_y > 1.0f;
    const bool using_mcoef = mcoef > 0.001f;

    ZERO_FARRAY(Kfusion);
    Vector24 Hfusion; // Observation Jacobians // 观测雅可比矩阵
    const ftype R_ACC = sq(fmaxF(frontend->_dragObsNoise, 0.5f));
    const ftype density_ratio = sqrtF(dal.get_EAS2TAS());
    const ftype rho = fmaxF(1.225f * density_ratio, 0.1f); // air density // 空气密度

    // get latest estimated orientation
    // 获取最新估计的姿态
    const ftype &q0 = stateStruct.quat[0];
    const ftype &q1 = stateStruct.quat[1];
    const ftype &q2 = stateStruct.quat[2];
    const ftype &q3 = stateStruct.quat[3];

    // get latest velocity in earth frame
    // 获取最新的地球坐标系速度
    const ftype &vn = stateStruct.velocity.x;
    const ftype &ve = stateStruct.velocity.y;
    const ftype &vd = stateStruct.velocity.z;

    // get latest wind velocity in earth frame
    // 获取最新的地球坐标系风速
    const ftype &vwn = stateStruct.wind_vel.x;
    const ftype &vwe = stateStruct.wind_vel.y;

    // predicted specific forces
    // calculate relative wind velocity in earth frame and rotate into body frame
    // 预测的比力
    // 计算地球坐标系中的相对风速并旋转到机体坐标系
    const Vector3F rel_wind_earth(vn - vwn, ve - vwe, vd);
    const Vector3F rel_wind_body = prevTnb * rel_wind_earth;

    // perform sequential fusion of XY specific forces
    // 依次融合XY方向的比力
    for (uint8_t axis_index = 0; axis_index < 2; axis_index++) {
        // correct accel data for bias
        // 修正加速度数据的偏差
        const ftype mea_acc = dragSampleDelayed.accelXY[axis_index]  - stateStruct.accel_bias[axis_index] / dtEkfAvg;

        // Acceleration in m/s/s predicted using vehicle and wind velocity estimates
        // Initialised to measured value and updated later using available drag model
        // 使用载具和风速估计预测的加速度(m/s/s)
        // 初始化为测量值,稍后使用可用的阻力模型更新
        ftype predAccel = mea_acc;

        // predicted sign of drag force
        // 预测的阻力方向
        const ftype dragForceSign = is_positive(rel_wind_body[axis_index]) ? -1.0f : 1.0f;

        if (axis_index == 0) {
            // drag can be modelled as an arbitrary  combination of bluff body drag that proportional to
            // speed squared, and rotor momentum drag that is proportional to speed.
            // 阻力可以建模为与速度平方成正比的钝体阻力和与速度成正比的旋翼动量阻力的任意组合
            ftype Kacc; // Derivative of specific force wrt airspeed // 比力对空速的导数
            if (using_mcoef && using_bcoef_x) {
                // mixed bluff body and propeller momentum drag
                // 混合钝体和螺旋桨动量阻力
                const ftype airSpd = (bcoef_x / rho) * (- mcoef + sqrtF(sq(mcoef) + 2.0f * (rho / bcoef_x) * fabsF(mea_acc)));
                Kacc = fmaxF(1e-1f, (rho / bcoef_x) * airSpd + mcoef * density_ratio);
                predAccel = (0.5f / bcoef_x) * rho * sq(rel_wind_body[0]) * dragForceSign - rel_wind_body[0] * mcoef * density_ratio;
            } else if (using_mcoef) {
                // propeller momentum drag only
                // 仅螺旋桨动量阻力
                Kacc = fmaxF(1e-1f, mcoef * density_ratio);
                predAccel = - rel_wind_body[0] * mcoef * density_ratio;
            } else if (using_bcoef_x) {
                // bluff body drag only
                // 仅钝体阻力
                const ftype airSpd = sqrtF((2.0f * bcoef_x * fabsF(mea_acc)) / rho);
                Kacc = fmaxF(1e-1f, (rho / bcoef_x) * airSpd);
                predAccel = (0.5f / bcoef_x) * rho * sq(rel_wind_body[0]) * dragForceSign;
            } else {
                // skip this axis
                // 跳过这个轴
                continue;
            }

            // intermediate variables
            // 中间变量
            const ftype HK0 = vn - vwn;
            const ftype HK1 = ve - vwe;
            const ftype HK2 = HK0*q0 + HK1*q3 - q2*vd;
            const ftype HK3 = 2*Kacc;
            const ftype HK4 = HK0*q1 + HK1*q2 + q3*vd;
            const ftype HK5 = HK0*q2 - HK1*q1 + q0*vd;
            const ftype HK6 = -HK0*q3 + HK1*q0 + q1*vd;
            const ftype HK7 = sq(q0) + sq(q1) - sq(q2) - sq(q3);
            const ftype HK8 = HK7*Kacc;
            const ftype HK9 = q0*q3 + q1*q2;
            const ftype HK10 = HK3*HK9;
            const ftype HK11 = q0*q2 - q1*q3;
            const ftype HK12 = 2*HK9;
            const ftype HK13 = 2*HK11;
            const ftype HK14 = 2*HK4;
            const ftype HK15 = 2*HK2;
            const ftype HK16 = 2*HK5;
            const ftype HK17 = 2*HK6;
            const ftype HK18 = -HK12*P[0][23] + HK12*P[0][5] - HK13*P[0][6] + HK14*P[0][1] + HK15*P[0][0] - HK16*P[0][2] + HK17*P[0][3] - HK7*P[0][22] + HK7*P[0][4];
            const ftype HK19 = HK12*P[5][23];
            const ftype HK20 = -HK12*P[23][23] - HK13*P[6][23] + HK14*P[1][23] + HK15*P[0][23] - HK16*P[2][23] + HK17*P[3][23] + HK19 - HK7*P[22][23] + HK7*P[4][23];
            const ftype HK21 = sq(Kacc);
            const ftype HK22 = HK12*HK21;
            const ftype HK23 = HK12*P[5][5] - HK13*P[5][6] + HK14*P[1][5] + HK15*P[0][5] - HK16*P[2][5] + HK17*P[3][5] - HK19 + HK7*P[4][5] - HK7*P[5][22];
            const ftype HK24 = HK12*P[5][6] - HK12*P[6][23] - HK13*P[6][6] + HK14*P[1][6] + HK15*P[0][6] - HK16*P[2][6] + HK17*P[3][6] + HK7*P[4][6] - HK7*P[6][22];
            const ftype HK25 = HK7*P[4][22];
            const ftype HK26 = -HK12*P[4][23] + HK12*P[4][5] - HK13*P[4][6] + HK14*P[1][4] + HK15*P[0][4] - HK16*P[2][4] + HK17*P[3][4] - HK25 + HK7*P[4][4];
            const ftype HK27 = HK21*HK7;
            const ftype HK28 = -HK12*P[22][23] + HK12*P[5][22] - HK13*P[6][22] + HK14*P[1][22] + HK15*P[0][22] - HK16*P[2][22] + HK17*P[3][22] + HK25 - HK7*P[22][22];
            const ftype HK29 = -HK12*P[1][23] + HK12*P[1][5] - HK13*P[1][6] + HK14*P[1][1] + HK15*P[0][1] - HK16*P[1][2] + HK17*P[1][3] - HK7*P[1][22] + HK7*P[1][4];
            const ftype HK30 = -HK12*P[2][23] + HK12*P[2][5] - HK13*P[2][6] + HK14*P[1][2] + HK15*P[0][2] - HK16*P[2][2] + HK17*P[2][3] - HK7*P[2][22] + HK7*P[2][4];
            const ftype HK31 = -HK12*P[3][23] + HK12*P[3][5] - HK13*P[3][6] + HK14*P[1][3] + HK15*P[0][3] - HK16*P[2][3] + HK17*P[3][3] - HK7*P[3][22] + HK7*P[3][4];
            // const ftype HK32 = Kacc/(-HK13*HK21*HK24 + HK14*HK21*HK29 + HK15*HK18*HK21 - HK16*HK21*HK30 + HK17*HK21*HK31 - HK20*HK22 + HK22*HK23 + HK26*HK27 - HK27*HK28 + R_ACC);

            // calculate innovation variance and exit if badly conditioned
            // 计算创新方差,如果条件不好则退出
            innovDragVar.x = (-HK13*HK21*HK24 + HK14*HK21*HK29 + HK15*HK18*HK21 - HK16*HK21*HK30 + HK17*HK21*HK31 - HK20*HK22 + HK22*HK23 + HK26*HK27 - HK27*HK28 + R_ACC);
            if (innovDragVar.x < R_ACC) {
                return;
            }
            const ftype HK32 = Kacc / innovDragVar.x;

            // Observation Jacobians
            // 观测雅可比矩阵
            Hfusion[0] = -HK2*HK3;
            Hfusion[1] = -HK3*HK4;
            Hfusion[2] = HK3*HK5;
            Hfusion[3] = -HK3*HK6;
            Hfusion[4] = -HK8;
            Hfusion[5] = -HK10;
            Hfusion[6] = HK11*HK3;
            Hfusion[22] = HK8;
            Hfusion[23] = HK10;

            // Kalman gains
            // Don't allow modification of any states other than wind velocity - we only need a wind estimate.
            // See AP_NavEKF3/derivation/generated/acc_bf_generated.cpp for un-implemented Kalman gain equations.
            // 卡尔曼增益
            // 不允许修改除风速以外的任何状态 - 我们只需要风速估计。
            // 未实现的卡尔曼增益方程见AP_NavEKF3/derivation/generated/acc_bf_generated.cpp
            Kfusion[22] = -HK28*HK32;
            Kfusion[23] = -HK20*HK32;


        } else if (axis_index == 1) {
            // drag can be modelled as an arbitrary combination of bluff body drag that proportional to
            // speed squared, and rotor momentum drag that is proportional to speed.
            // 阻力可以建模为与速度平方成正比的钝体阻力和与速度成正比的旋翼动量阻力的任意组合
            ftype Kacc; // Derivative of specific force wrt airspeed // 比力对空速的导数
            if (using_mcoef && using_bcoef_y) {
                // mixed bluff body and propeller momentum drag
                // 混合钝体和螺旋桨动量阻力
                const ftype airSpd = (bcoef_y / rho) * (- mcoef + sqrtF(sq(mcoef) + 2.0f * (rho / bcoef_y) * fabsF(mea_acc)));
                Kacc = fmaxF(1e-1f, (rho / bcoef_y) * airSpd + mcoef * density_ratio);
                predAccel = (0.5f / bcoef_y) * rho * sq(rel_wind_body[1]) * dragForceSign - rel_wind_body[1] * mcoef * density_ratio;
            } else if (using_mcoef) {
                // propeller momentum drag only
                // 仅螺旋桨动量阻力
                Kacc = fmaxF(1e-1f, mcoef * density_ratio);
                predAccel = - rel_wind_body[1] * mcoef * density_ratio;
            } else if (using_bcoef_y) {
                // bluff body drag only
                // 仅钝体阻力
                const ftype airSpd = sqrtF((2.0f * bcoef_y * fabsF(mea_acc)) / rho);
                Kacc = fmaxF(1e-1f, (rho / bcoef_y) * airSpd);
                predAccel = (0.5f / bcoef_y) * rho * sq(rel_wind_body[1]) * dragForceSign;
            } else {
                // nothing more to do
                // 没有更多要做的
                return;
            }

            // intermediate variables
            // 中间变量
            const ftype HK0 = ve - vwe;  // 东向相对风速 (地面速度减去风速)
            const ftype HK1 = vn - vwn;  // 北向相对风速 (地面速度减去风速)
            const ftype HK2 = HK0*q0 - HK1*q3 + q1*vd;  // 四元数旋转矩阵的第一行与相对风速的点积
            const ftype HK3 = 2*Kacc;  // 比力对空速导数的2倍
            const ftype HK4 = -HK0*q1 + HK1*q2 + q0*vd;  // 四元数旋转矩阵的第二行与相对风速的点积
            const ftype HK5 = HK0*q2 + HK1*q1 + q3*vd;  // 四元数旋转矩阵的第三行与相对风速的点积
            const ftype HK6 = HK0*q3 + HK1*q0 - q2*vd;  // 四元数旋转矩阵的第四行与相对风速的点积
            const ftype HK7 = q0*q3 - q1*q2;  // 四元数旋转矩阵的(1,2)元素
            const ftype HK8 = HK3*HK7;  // 旋转矩阵元素与比力导数的乘积
            const ftype HK9 = sq(q0) - sq(q1) + sq(q2) - sq(q3);  // 四元数旋转矩阵的(2,2)元素
            const ftype HK10 = HK9*Kacc;  // 旋转矩阵元素与比力导数的乘积
            const ftype HK11 = q0*q1 + q2*q3;  // 四元数旋转矩阵的(1,3)元素
            const ftype HK12 = 2*HK11;  // 旋转矩阵元素的2倍
            const ftype HK13 = 2*HK7;  // 旋转矩阵元素的2倍
            const ftype HK14 = 2*HK5;  // 相对风速点积的2倍
            const ftype HK15 = 2*HK2;  // 相对风速点积的2倍
            const ftype HK16 = 2*HK4;  // 相对风速点积的2倍
            const ftype HK17 = 2*HK6;  // 相对风速点积的2倍

            // 以下是协方差矩阵的计算
            const ftype HK18 = HK12*P[0][6] + HK13*P[0][22] - HK13*P[0][4] + HK14*P[0][2] + HK15*P[0][0] + HK16*P[0][1] - HK17*P[0][3] - HK9*P[0][23] + HK9*P[0][5];
            const ftype HK19 = sq(Kacc);  // 比力导数的平方
            const ftype HK20 = HK12*P[6][6] - HK13*P[4][6] + HK13*P[6][22] + HK14*P[2][6] + HK15*P[0][6] + HK16*P[1][6] - HK17*P[3][6] + HK9*P[5][6] - HK9*P[6][23];
            const ftype HK21 = HK13*P[4][22];  // 协方差矩阵元素与旋转矩阵元素的乘积
            const ftype HK22 = HK12*P[6][22] + HK13*P[22][22] + HK14*P[2][22] + HK15*P[0][22] + HK16*P[1][22] - HK17*P[3][22] - HK21 - HK9*P[22][23] + HK9*P[5][22];
            const ftype HK23 = HK13*HK19;  // 旋转矩阵元素与比力导数平方的乘积
            const ftype HK24 = HK12*P[4][6] - HK13*P[4][4] + HK14*P[2][4] + HK15*P[0][4] + HK16*P[1][4] - HK17*P[3][4] + HK21 - HK9*P[4][23] + HK9*P[4][5];
            const ftype HK25 = HK9*P[5][23];  // 协方差矩阵元素与旋转矩阵元素的乘积
            const ftype HK26 = HK12*P[5][6] - HK13*P[4][5] + HK13*P[5][22] + HK14*P[2][5] + HK15*P[0][5] + HK16*P[1][5] - HK17*P[3][5] - HK25 + HK9*P[5][5];
            const ftype HK27 = HK19*HK9;  // 比力导数平方与旋转矩阵元素的乘积
            const ftype HK28 = HK12*P[6][23] + HK13*P[22][23] - HK13*P[4][23] + HK14*P[2][23] + HK15*P[0][23] + HK16*P[1][23] - HK17*P[3][23] + HK25 - HK9*P[23][23];
            const ftype HK29 = HK12*P[2][6] + HK13*P[2][22] - HK13*P[2][4] + HK14*P[2][2] + HK15*P[0][2] + HK16*P[1][2] - HK17*P[2][3] - HK9*P[2][23] + HK9*P[2][5];
            const ftype HK30 = HK12*P[1][6] + HK13*P[1][22] - HK13*P[1][4] + HK14*P[1][2] + HK15*P[0][1] + HK16*P[1][1] - HK17*P[1][3] - HK9*P[1][23] + HK9*P[1][5];
            const ftype HK31 = HK12*P[3][6] + HK13*P[3][22] - HK13*P[3][4] + HK14*P[2][3] + HK15*P[0][3] + HK16*P[1][3] - HK17*P[3][3] - HK9*P[3][23] + HK9*P[3][5];

            // 计算创新方差
            innovDragVar.y = (HK12*HK19*HK20 + HK14*HK19*HK29 + HK15*HK18*HK19 + HK16*HK19*HK30 - HK17*HK19*HK31 + HK22*HK23 - HK23*HK24 + HK26*HK27 - HK27*HK28 + R_ACC);
            if (innovDragVar.y < R_ACC) {
                // calculation is badly conditioned
                // 计算条件不好
                return;
            }
            const ftype HK32 = Kacc / innovDragVar.y;  // 卡尔曼增益的标量部分

            // Observation Jacobians
            // 观测雅可比矩阵
            Hfusion[0] = -HK2*HK3;  // 对四元数q0的偏导数
            Hfusion[1] = -HK3*HK4;  // 对四元数q1的偏导数
            Hfusion[2] = -HK3*HK5;  // 对四元数q2的偏导数
            Hfusion[3] = HK3*HK6;   // 对四元数q3的偏导数
            Hfusion[4] = HK8;       // 对速度的偏导数
            Hfusion[5] = -HK10;     // 对位置的偏导数
            Hfusion[6] = -HK11*HK3; // 对姿态角的偏导数
            Hfusion[22] = -HK8;     // 对风速x分量的偏导数
            Hfusion[23] = HK10;     // 对风速y分量的偏导数

            // Kalman gains
            // Don't allow modification of any states other than wind velocity at this stage of development - we only need a wind estimate.
            // See AP_NavEKF3/derivation/generated/acc_bf_generated.cpp for un-implemented Kalman gain equations.
            // 卡尔曼增益
            // 在开发的这个阶段，不允许修改除风速以外的任何状态 - 我们只需要风速估计
            // 未实现的卡尔曼增益方程见AP_NavEKF3/derivation/generated/acc_bf_generated.cpp
            Kfusion[22] = -HK22*HK32;  // 风速x分量的卡尔曼增益
            Kfusion[23] = -HK28*HK32;  // 风速y分量的卡尔曼增益
        }

        innovDrag[axis_index] = predAccel - mea_acc;  // 计算创新序列
        dragTestRatio[axis_index] = sq(innovDrag[axis_index]) / (25.0f * innovDragVar[axis_index]);  // 计算创新一致性检验比率

        // if the innovation consistency check fails then don't fuse the sample
        // 如果创新一致性检验失败，则不融合该样本
        if (dragTestRatio[axis_index] > 1.0f) {
            return;
        }

        // correct the state vector
        // 修正状态向量
        for (uint8_t j= 0; j<=stateIndexLim; j++) {
            statesArray[j] = statesArray[j] - Kfusion[j] * innovDrag[axis_index];
        }
        stateStruct.quat.normalize();  // 归一化四元数

        // correct the covariance P = (I - K*H)*P
        // take advantage of the empty columns in KH to reduce the
        // number of operations
        // 修正协方差矩阵 P = (I - K*H)*P
        // 利用KH中的空列来减少运算次数
        for (unsigned i = 0; i<=stateIndexLim; i++) {
            for (unsigned j = 0; j<=6; j++) {
                KH[i][j] = Kfusion[i] * Hfusion[j];
            }
            for (unsigned j = 7; j<=21; j++) {
                KH[i][j] = 0.0f;
            }
            for (unsigned j = 22; j<=23; j++) {
                KH[i][j] = Kfusion[i] * Hfusion[j];
            }
        }
        for (unsigned j = 0; j<=stateIndexLim; j++) {
            for (unsigned i = 0; i<=stateIndexLim; i++) {
                ftype res = 0;
                res += KH[i][0] * P[0][j];
                res += KH[i][1] * P[1][j];
                res += KH[i][2] * P[2][j];
                res += KH[i][3] * P[3][j];
                res += KH[i][4] * P[4][j];
                res += KH[i][5] * P[5][j];
                res += KH[i][6] * P[6][j];
                res += KH[i][22] * P[22][j];
                res += KH[i][23] * P[23][j];
                KHP[i][j] = res;
            }
        }
        for (unsigned i = 0; i<=stateIndexLim; i++) {
            for (unsigned j = 0; j<=stateIndexLim; j++) {
                P[i][j] = P[i][j] - KHP[i][j];
            }
        }
    }

    // record time of successful fusion
    // 记录成功融合的时间
    lastDragPassTime_ms = imuSampleTime_ms;
}
#endif // EK3_FEATURE_DRAG_FUSION

/********************************************************
*                   MISC FUNCTIONS                      *
********************************************************/
