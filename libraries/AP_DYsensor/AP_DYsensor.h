#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

class AP_DYsensor {
public:
    AP_DYsensor();

    // 初始化传感器
    void init(void);

    // 主循环更新函数
    void update(void);

    // 健康状态检查
    bool healthy() const { return _healthy; }

    // 添加参数支持
    static const struct AP_Param::GroupInfo var_info[];
    
    // 添加配置参数
    AP_Int8 enabled;

private:
    // 串口指针
    AP_HAL::UARTDriver *_uart;
    
    // 健康状态标志
    bool _healthy;
    
    // 上次读取时间
    uint32_t _last_read_ms;
    
    // 读取传感器数据
    void _read_sensor();
    
    // 解析接收到的数据
    void _parse_data();
    
    // 验证校验和
    bool _verify_checksum();
    
    // 数据包长度定义
    static const uint8_t PACKET_LENGTH = 32;  // 总长度
    static const uint8_t SYNC_BYTE1 = 0xEE;  // 同步字1
    static const uint8_t SYNC_BYTE2 = 0x16;  // 同步字2
    
    // 接收缓冲区
    uint8_t _rx_buffer[PACKET_LENGTH];
    uint8_t _rx_index;
    
    // 解析状态
    enum class ParseState {
        WAITING_SYNC1,
        WAITING_SYNC2,
        COLLECTING_DATA,
    } _parse_state;
}; 