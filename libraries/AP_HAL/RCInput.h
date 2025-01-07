#pragma once

#include "AP_HAL_Namespace.h"

// 定义遥控器输入信号的最小脉冲宽度(微秒)
#define RC_INPUT_MIN_PULSEWIDTH 900
// 定义遥控器输入信号的最大脉冲宽度(微秒) 
#define RC_INPUT_MAX_PULSEWIDTH 2100

// RC输入抽象类,定义了处理遥控器输入信号的接口
class AP_HAL::RCInput {
public:
    /**
     * 初始化RC输入系统
     * 需要在平台HAL实例初始化时调用,以确保RCInput实现类型
     * 和初始化参数(如ISRRegistry)对程序员可知
     * (在C++类型系统中很难描述这种依赖关系)
     */
    virtual void init() = 0;
    
    // 清理资源的虚函数
    virtual void teardown() {};

    /**
     * 检查是否有新的输入数据
     * 如果自上次调用new_input()后有新数据,返回true
     */
    virtual bool new_input(void) = 0;

    /**
     * 返回最近一次读取中有效通道的数量
     */
    virtual uint8_t  num_channels() = 0;

    /* 读取单个通道的值 */
    virtual uint16_t read(uint8_t ch) = 0;

    /* 读取多个通道的值到数组中,返回有效的通道数量 */
    virtual uint8_t read(uint16_t* periods, uint8_t len) = 0;

    /* 获取接收机的RSSI(信号强度)值,如果可用的话
       返回值: -1表示未知, 0表示无连接, 255表示信号最强 */
    virtual int16_t get_rssi(void) { return -1; }
    
    /* 获取接收机链路质量
       返回值: -1表示未知, 较大值表示链路质量更好 */
    virtual int16_t get_rx_link_quality(void) { return -1; }
    
    /* 返回描述RC输入协议的字符串 */
    virtual const char *protocol() const = 0;

    /**
     * 通道值覆盖功能:这些功能可能不太优雅,但目前端口工作需要它们
     * v的取值说明:
     *  v == -1 -> 不改变此通道
     *  v == 0  -> 不覆盖此通道
     *  v > 0   -> 将v设置为覆盖值
     */

    /* 执行接收机对频绑定 */
    virtual bool rc_bind(int dsmMode) { return false; }

    /* 启用或禁用RC输入的脉冲输入
       当我们通过UART解码R/C信号时,用于减少系统负载 */
    virtual void pulse_input_enable(bool enable) { }
};
