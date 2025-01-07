# ACNS-F405AIO 集成飞控板

# 简介
ACNS-F405AIO是一款低成本、小巧的多旋翼飞控板,板载集成了4个BLheli_s电调。

## 主要特性

# 硬件架构特点
 - STM32F405RET微控制器
 - IMU: BMI160、ICM42688惯性测量单元
 - Mag: LIS3MDLTR磁力计
 - Baro: BMP280气压计
 - 1个microSD卡槽
 - 6个UART串口和USB接口
 - 1个I2C接口
 - 1个CAN接口
 - 1个SBUS输入和8路PWM输出(4个内置电调,4个外部PWM接口)
 - 1个外部SPI接口
 - 板载RGB LED
 - 板载128M Flash存储
 - 4个BLheli_s电调,支持3-4S,30A,电机顺序匹配Ardupilot X型机架配置
 - 小巧轻便,尺寸39mm X 39mm X 10mm,重量9g(不含外壳)
   
## UART串口映射

# 串口功能分配
 - SERIAL0 -> USB(OTG1)
 - SERIAL1 -> USART1(数传1)(支持DMA)
 - SERIAL2 -> USART3(数传2)(支持DMA)
 - SERIAL3 -> UART4(GPS)(支持DMA)
 - SERIAL4 -> UART6(GPS2)(支持DMA)
 - SERIAL5 -> USART2(SBUS)(遥控输入,不支持DMA)

## 遥控器输入

# RC输入配置在SBUS引脚(UART2_RX),支持除串行协议外的所有RC协议

## PWM输出

# PWM输出能力说明
ACNS-F405AIO支持最多8路PWM输出。所有输出都支持DShot(不支持BDshot)。
PWM分为3组:

# PWM分组说明
 - PWM 1~4 在第1组(4个电机)
 - PWM 5,6 在第2组(外部PWM)
 - PWM 7,8 在第3组(外部PWM)

## GPIO功能

# GPIO功能说明
4个外部PWM通道可用作GPIO功能。
下面显示了这些PWM通道在ArduPilot中的引脚编号:

# PWM通道对应的GPIO引脚编号表
| PWM通道 | 引脚编号 |
| ------------ | ---- | 
| PWM5         | 54   |
| PWM6         | 55   | 
| PWM7         | 56   | 
| PWM8         | 57   | 

## 电池监测

# 默认电池监测参数配置
正确的电池设置参数默认为:
 
 - BATT_MONITOR 4
 - BATT_VOLT_PIN 11
 - BATT_CURR_PIN 12
 - BATT_VOLT_SCALE 9.2
 - BATT_CURR_SCALE 50.0

## 指南针

# 指南针配置说明
ACNS-F405AIO有一个内置的LIS3MDLTR指南针,也可以通过I2C接口的SDA和SCL引脚连接外部指南针。

## 固件加载

# 固件获取说明
这些板子的固件可以在 https://firmware.ardupilot.org 的"ACNS-F405AIO"子文件夹中找到。

# 初始固件加载说明
初始固件加载可以通过DFU方式完成,按住boot按钮插入USB。然后使用你喜欢的DFU工具加载"xxx_bl.hex"固件。

# 后续更新说明
之后,你可以使用Mission Planner更新固件。

## 引脚定义和尺寸图
<div align=center>
<img width="500" src=F405AIO_top.jpg/>

<img width="500" src=F405AIO_bottom.jpg/>
