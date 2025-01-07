// 防止头文件重复包含
#pragma once

// 包含日志结构相关头文件
#include <AP_Logger/LogStructure.h>
// 包含UART驱动相关头文件
#include "UARTDriver.h"

// 定义HAL层的日志ID,包含UART消息ID
#define LOG_IDS_FROM_HAL \
    LOG_UART_MSG

// @LoggerMessage: UART
// @Description: UART统计信息
// @Field: TimeUS: 系统启动后的时间(微秒)
// @Field: I: UART实例编号
// @Field: Tx: 发送数据速率(字节/秒)
// @Field: Rx: 接收数据速率(字节/秒)
// PACKED关键字确保结构体紧凑对齐
struct PACKED log_UART {
    LOG_PACKET_HEADER;      // 日志数据包头
    uint64_t time_us;       // 时间戳(微秒)
    uint8_t instance;       // UART实例编号
    float tx_rate;          // 发送速率
    float rx_rate;          // 接收速率
};

// 如果未启用UART统计功能
#if !HAL_UART_STATS_ENABLED
#define LOG_STRUCTURE_FROM_HAL
#else
// 定义UART日志数据结构
// LOG_UART_MSG: 消息ID
// sizeof(log_UART): 结构体大小
// "UART": 日志名称
// "QBff": 数据格式(Q:uint64_t, B:uint8_t, f:float)
// "TimeUS,I,Tx,Rx": 字段名称
// "s#BB": 单位(s:秒, #:无单位, B:字节)
// "F---": 倍数标志
#define LOG_STRUCTURE_FROM_HAL                          \
    { LOG_UART_MSG, sizeof(log_UART),                   \
      "UART","QBff","TimeUS,I,Tx,Rx", "s#BB", "F---" },
#endif
