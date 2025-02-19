#pragma once

#include <AP_HAL/AP_HAL.h>

class AP_DYsensor {
public:
    AP_DYsensor();
    
    // 初始化函数
    void init(void);
    
    // 定期调用的更新函数
    void update(void);
    
    // 发送数据函数
    void send_data(void);

private:
    // UART指针
    AP_HAL::UARTDriver *_uart;
    
    // 数据包相关常量
    static const uint8_t REC_PACKET_LENGTH = 32;  // 数据包总长度
    static const uint8_t REC_SYNC_BYTE1 = 0xEE;   // 同步字1
    static const uint8_t REC_SYNC_BYTE2 = 0x16;   // 同步字2
    
    static const uint8_t SEND_PACKET_LENGTH = 16;   // 数据包总长度
    static const uint8_t SEND_SYNC_BYTE1 = 0xEB;   // 同步字1
    static const uint8_t SEND_SYNC_BYTE2 = 0x90;   // 同步字2

   //解析状态枚举
    enum class ParseState {
        WAITING_SYNC1,    // 等待同步字1
        WAITING_SYNC2,    // 等待同步字2
        COLLECTING_DATA   // 收集数据
    };
    
    // 当前解析状态
    ParseState _parse_state = ParseState::WAITING_SYNC1;
    
    // 接收缓冲区
    uint8_t _rx_buffer[REC_PACKET_LENGTH];
    uint8_t _rx_index;
    
    // 发送缓冲区
    uint8_t _tx_buffer[SEND_PACKET_LENGTH];
    
    // 校验和验证
    bool _verify_checksum(void);
    
    // 计算校验和
    uint8_t _calculate_checksum(const uint8_t *buffer,const uint8_t PACKET_LENGTH);
    
    // 解析数据
    void _parse_data(void);
}; 