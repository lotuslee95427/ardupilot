# AnyLeaf Mercury H7 飞控板说明文档

## 概述
这是由[AnyLeaf](http://www.anyleaf.org/)生产的Mercury H7飞控板的说明文档。该文档描述了飞控板的硬件特性、接口定义和使用说明。

## 主要特性
    处理器
        STM32H743 32位处理器
    传感器
        ICM42688加速度计/陀螺仪,配备专用32.768kHz晶振
        DPS310气压计
    供电系统
        支持2S-6S锂电池输入,带电压监测
        9V/3A输出用于供电图传
        5V/2A输出用于供电舵机和其他电子设备
        3.3V/500mA输出用于供电电子设备
    接口
        8路双向DSHOT或PWM电机输出
        1个CAN-FD接口用于外设连接
        1个DJI格式图传接口
        4个UART串口用于外设连接(默认启用其中3个)
        1个I2C总线用于外设连接
        USB-C接口
        所有UART支持硬件反相
        板载ExpressLRS遥控接收机,用于控制和/或遥测数据
    尺寸规格
        尺寸: 37.5 x 37.5mm
        重量: 8g

## 引脚定义

![Anyleaf H7底部引脚图](anyleaf_h7_diagram_bottom.jpg)
![Anyleaf H7顶部接口图](anyleaf_h7_diagram_top.jpg)

引脚和接口定义已在PCB上标注,以下为特殊说明:
- 板载ELRS接收机连接到UART2的PA2(FC发送)和PA3(FC接收)引脚
- 电调遥测连接到UART3接收引脚(PD9)
- OSD HDL(DJI手控器互操作)连接到UART1接收引脚(PB7)

## UART映射
所有UART均支持DMA传输
 
 - SERIAL0 -> USB接口
 - SERIAL1 -> UART1(外部焊盘和DJI接口的SBUS引脚,默认MAVLINK2协议)
 - SERIAL2 -> UART2(DJI接口遥测,默认DisplayPort协议)
 - SERIAL3 -> UART3(电调接口遥测引脚或外部焊盘,默认电调遥测协议)
 - SERIAL4 -> USART4(外部焊盘,默认GPS协议)
 - SERIAL5 -> UART7(仅用于板载ELRS接收机,RCIN协议)
 - SERIAL6 -> UART8(用户自定义,外部焊盘)

## CAN-FD接口

飞控板包含一个4引脚DroneCAN标准CAN接口。支持64字节帧长度和最高5Mbps数据速率。可用于连接GPS、指南针、电源监测、传感器、电机、舵机等CAN外设。

## 遥控输入

飞控板集成2.4GHz ExpressLRS接收机,可接收控制信号,发送或接收MavLink遥测数据。要启用所有ELRS功能,需要将RC5通道设置为解锁开关(可通过多个RC5_OPTIONS实现),或将发射机的5通道映射为遥测解锁状态。详见示例:https://youtu.be/YO2yA1fmZBs

如需使用DJI接口的SBUS,需将SERIAL5_PROTOCOL改为0,SERIAL1_PROTOCOL改为23以启用遥控输入。
   
## OSD支持

飞控板在6引脚DJI兼容JST SH接口上提供MSP-DisplayPort输出。

## 电机输出

电机1-8支持双向DSHOT和PWM输出。

以下电机组内必须统一使用PWM或DShot:
电机1-4 组1
电机5-6 组2
电机7-8 组3

## 指南针

飞控板不包含内置指南针,但可通过CAN接口或底部I2C焊盘连接外置指南针。

## 固件烧录
固件可在https://firmware.ardupilot.org的"Anyleaf H7"子目录下获取。

首次烧录需按住BOOT按键并插入USB进入DFU模式,使用DFU工具烧录"AnyleafH7_bl.hex"引导程序。

之后可使用Mission Planner或QGroundControl更新固件。
