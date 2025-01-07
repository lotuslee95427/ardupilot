# Airvolute DCS2.Pilot板载FMU说明文档

# 概述
DroneCore 2.0是一款模块化的AI驱动开源飞控系统,专为复杂应用场景设计。它集成了高性能计算处理能力、冗余连接、小尺寸和轻量化等特点。该系统将载板、伴随计算机和电源分配板的功能整合到一个紧凑的外形中。

系统通常使用"CUBE"飞控作为主FMU,但也可以使用板载STM32H743作为FMU。本文档描述了板载FMU的硬件定义和固件配置,固件可从ArduPilot固件服务器获取。

更多关于DCS2.Pilot板的信息请参考:
https://docs.airvolute.com/dronecore-autopilot/dcs2

## 购买渠道
info@airvolute.com

## 主要特性
- MCU: STM32H743 微控制器
- IMU: BMI088 惯性测量单元 
- 气压计: BMP390
- 2个UART串口
- 2路CAN总线
- 4路PWM输出
- PPM遥控输入
- 外部SPI和I2C接口
- SD卡接口
- 板载USB与Jetson主机连接
- 以太网接口

## DCS2.Pilot外设接口图
[外设接口示意图]

## DCS2.Pilot板载FMU相关接口定义
### 顶部接口
[顶部接口示意图]

#### PPM接口(遥控输入)
- 接口类型:JST GH 1.25mm间距,3针
- 匹配连接器:JST GHR-03V-S
- RC输入配置在PPM_SBUS_PROT引脚,连接到UART3_RX和TIM3_CH1
- 支持所有单向遥控协议,需设置SERIAL3_PROTOCOL为RCIN
- 由于与主FMU共享,默认在此副FMU上禁用
- 5V供电限流1A

[接口引脚定义表格]

### 底部接口
[底部接口示意图]

#### FMU SEC接口
- 接口类型:JST GH 1.25mm间距,12针
- 匹配连接器:JST GHR-12V-S
- 支持4路PWM输出,直接连接STM32H743
- 支持所有PWM协议及DShot和双向DShot
- PWM分两组:1,2为一组;3,4为一组
- 同组通道需使用相同输出频率
- 5V供电限流1A

[接口引脚定义表格]

#### 外部传感器接口
- 接口类型:BM23PF0.8-10DS-0.35V
- 匹配连接器:BM23PF0.8-10DP-0.35V 
- 支持通过I2C和SPI连接外部IMU
- 5V供电限流1.9A

[接口引脚定义表格]

#### 以太网扩展接口
- 接口型号:505110-1692
- 通过板载交换机连接到FMU
- RMII总线,速率100Mbps

#### SD卡接口
- 接口型号:MEM2085-00-115-00-A
- 支持标准microSD卡
- 主要用于存储飞行数据和日志

## 其他接口
### CAN总线接口
- 两路CAN总线:CAN1和CAN2
- 支持高达1Mbps速率,FD模式下8Mbps
- 接口在DCS2.Adapter_board上,可根据需求定制
- JST GH 1.25mm间距,4针
- 5V供电限流1.9A

[接口引脚定义表格]

## UART映射
- SERIAL0: USB接口(默认115200波特率)
- SERIAL1: UART1(FMU SEC)(默认57600波特率,Mavlink2协议)
- SERIAL2: UART2(FMU SEC)(默认57600波特率,Mavlink2协议) 
- SERIAL3: UART3(仅RX,PPM接口)(默认禁用)

UART无硬件流控,UART1/2连接到FMU_SEC接口

## 固件烧录
初次烧录bootloader需通过SWD接口,之后可通过板载USB与Jetson主机连接烧录固件
