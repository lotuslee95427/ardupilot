/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       APM_Compass库示例代码(HMC5843传感器).
 *       作者: Jordi MuÒoz and Jose Julio. DIYDrones.com
 */

// 包含所需的头文件
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>

// 获取HAL(硬件抽象层)实例
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// 创建板级配置对象
static AP_BoardConfig board_config;

// 定义一个虚拟载具类,用于提供AHRS和气压计支持
class DummyVehicle {
public:
    AP_AHRS ahrs;  // 需要AHRS,自PR #10890后添加
    AP_Baro baro;  // 指南针需要基于位置设置磁场模型
#if AP_COMPASS_EXTERNALAHRS_ENABLED
    AP_ExternalAHRS eAHRS; // 外部AHRS支持
#endif  // AP_COMPASS_EXTERNALAHRS_ENABLED
};

// 创建全局对象
static DummyVehicle vehicle;
static Compass compass;      // 指南针对象
static AP_SerialManager serial_manager;  // 串口管理器

uint32_t timer;  // 用于控制读取时间间隔的计时器

// 初始化函数,仅在启动时调用一次
static void setup()
{
    hal.console->printf("Compass library test\n");

    // 初始化各个组件
    board_config.init();
    vehicle.ahrs.init();
    compass.init();
    hal.console->printf("init done - %u compasses detected\n", compass.get_count());

    // 设置补偿值以消除周围干扰
    compass.set_and_save_offsets(0, Vector3f(0, 0, 0));
    // 设置磁北和真北之间的偏角
    compass.set_declination(ToRad(0.0f));

    hal.scheduler->delay(1000);
    timer = AP_HAL::micros();
}

// 主循环函数
static void loop()
{
    // 记录检测到的指南针数量
    static const uint8_t compass_count = compass.get_count();
    // 用于存储每个指南针三个轴的最小值、最大值和偏移值
    static float min[COMPASS_MAX_INSTANCES][3];
    static float max[COMPASS_MAX_INSTANCES][3];
    static float offset[COMPASS_MAX_INSTANCES][3];

    // 每100ms(10Hz)读取一次指南针数据
    if ((AP_HAL::micros() - timer) > 100000L) {
        timer = AP_HAL::micros();
        compass.read();
        const uint32_t read_time = AP_HAL::micros() - timer;

        // 遍历所有指南针
        for (uint8_t i = 0; i < compass_count; i++) {
            float heading;

            hal.console->printf("Compass #%u: ", i);

            // 检查指南针健康状态
            if (!compass.healthy()) {
                hal.console->printf("not healthy\n");
                continue;
            }

            // 创建方向余弦矩阵(DCM)
            Matrix3f dcm_matrix;
            // 本例中使用roll=0,pitch=0
            dcm_matrix.from_euler(0, 0, 0);
            heading = compass.calculate_heading(dcm_matrix, i);

            // 获取磁场向量
            const Vector3f &mag = compass.get_field(i);

            // 记录最小值
            min[i][0] = MIN(mag.x, min[i][0]);
            min[i][1] = MIN(mag.y, min[i][1]);
            min[i][2] = MIN(mag.z, min[i][2]);

            // 记录最大值
            max[i][0] = MAX(mag.x, max[i][0]);
            max[i][1] = MAX(mag.y, max[i][1]);
            max[i][2] = MAX(mag.z, max[i][2]);

            // 计算偏移值
            offset[i][0] = -(max[i][0] + min[i][0]) / 2;
            offset[i][1] = -(max[i][1] + min[i][1]) / 2;
            offset[i][2] = -(max[i][2] + min[i][2]) / 2;

            // 显示航向角和原始磁场数据
            hal.console->printf("Heading: %.2f (%3d, %3d, %3d)",
                                (double)ToDeg(heading),
                                (int)mag.x,
                                (int)mag.y,
                                (int)mag.z);

            // 显示计算出的偏移值
            hal.console->printf(" offsets(%.2f, %.2f, %.2f)",
                                (double)offset[i][0],
                                (double)offset[i][1],
                                (double)offset[i][2]);

            // 显示读取时间
            hal.console->printf(" t=%u", (unsigned)read_time);

            hal.console->printf("\n");
        }
    } else {
        // 如果距离上次读取时间不足100ms,延时1ms
        hal.scheduler->delay(1);
    }
}

// 主程序入口点
AP_HAL_MAIN();
