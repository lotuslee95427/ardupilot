# CM4PILOT 集成飞控板

# CM4PILOT是一款低成本、小巧的飞控板,集成了树莓派CM4计算模块

# 展示飞控板实物图片
<div align=left><img width="600"  src=CM4pilot_inshell.jpg/></div>
<div align=left><img width="500"  src=CM4Pilot_structure.jpg/></div>


## 主要特性

# 硬件架构特点
 - 树莓派CM4 + Ardupilot,驾驶舱结构中的伴飞计算机
 - 小巧轻便,尺寸58mm X 50mm X 18mm,重量26g(不含外壳)
 - Broadcom BCM2711四核Cortex-A72 (ARM v8) 64位SoC,主频1.5GHz
 - STM32F405微控制器

# 传感器配置
 - IMU: BMI088惯性测量单元
 - Mag: LIS3MDLTR磁力计
 - Baro: BMP280气压计
 - RTC: PCF85063实时时钟

# 接口配置
 - 2个2通道MIPI CSI摄像头接口
 - 2个microSD卡槽
 - 1个电源接口(模拟)
 - FMU: 6个UART串口和USB接口
 - CM4: 2个UART串口、4个USB2.0和1个OTG接口
 - 1个I2C接口
 - 1个CAN接口
 - 1个SBUS输入和8路PWM输出(支持DShot)
 - 外部SPI接口

# 板载功能
 - 板载蜂鸣器
 - 板载RGB LED
 - 板载128M Flash用于日志记录

## UART串口映射

# 串口功能分配
 - SERIAL0 -> USB(OTG1)
 - SERIAL1 -> USART1(数传1)(支持DMA)
 - SERIAL2 -> USART3 (CM4)(支持DMA)
 - SERIAL3 -> UART4 (GPS)(支持DMA)
 - SERIAL4 -> UART6 (GPS2)(支持DMA)
 - SERIAL5 -> USART2 (SBUS)(遥控输入,不支持DMA)

## 遥控器输入

# RC输入配置在SBUS引脚(UART2_RX),支持除串行协议外的所有RC协议

## PWM输出

# PWM输出能力说明
CM4PILOT支持最多8路PWM输出。所有输出都支持DShot(不支持BDshot)。
PWM分为4组:

# PWM分组说明
 - PWM 1~4 在第1组
 - PWM 5,6 在第2组
 - PWM 7,8 在第3组
 - 板载蜂鸣器在第4组

## GPIO功能

# GPIO功能说明
所有8个PWM通道都可用作GPIO功能。
下面显示了这些PWM通道在ArduPilot中的引脚编号:

# PWM通道对应的GPIO引脚编号表
| PWM通道 | 引脚编号 | PWM通道 | 引脚编号 |
| ------------ | ---- | ------------ | ---- |
| PWM1         | 50   | PWM8         | 57   |
| PWM2         | 51   | 
| PWM3         | 52   | 
| PWM4         | 53   | 
| PWM5         | 54   | 
| PWM6         | 55   | 
| PWM7         | 56   | 

## 电池监测

# 默认电池监测参数配置
正确的电池设置参数默认为:
 
 - BATT_MONITOR 4
 - BATT_VOLT_PIN 11
 - BATT_CURR_PIN 12
 - BATT_VOLT_SCALE 10.1
 - BATT_CURR_SCALE 17.0

## 指南针

# 指南针配置说明
CM4PILOT有一个内置的LIS3MDLTR指南针,也可以通过I2C接口的SDA和SCL引脚连接外部指南针。

## 固件加载

# 固件获取说明
这些板子的固件可以在 https://firmware.ardupilot.org 的"ACNS-CM4PILOT"子文件夹中找到。

# 初始固件加载说明
初始固件加载可以通过DFU方式完成,按住boot按钮插入USB。然后使用你喜欢的DFU工具加载"xxx_bl.hex"固件。

# 后续更新说明
之后,你可以使用Mission Planner更新固件。

## 引脚定义和尺寸

# 展示引脚定义和尺寸图
<div align=left><img width="600"  src=CM4Pilot_Pinout.jpg/></div>
<div align=left><img width="500"  src=CM4pilot_size.jpg/></div>
