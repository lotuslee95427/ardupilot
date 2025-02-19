#include "AP_DYsensor.h"

extern const AP_HAL::HAL& hal;

AP_DYsensor::AP_DYsensor()
{
    _uart = nullptr;
    _rx_index = 0;
    
    // 初始化发送缓冲区
    _tx_buffer[0] = SEND_SYNC_BYTE1;  // 同步字1
    _tx_buffer[1] = SEND_SYNC_BYTE2;  // 同步字2
    
    // 初始化数据位（示例数据）
    for (uint8_t i = 2; i < SEND_PACKET_LENGTH-1; i++) {
        _tx_buffer[i] = i;  // 填充示例数据
    }
}

void AP_DYsensor::init()
{
    // 获取SERIAL5的UART驱动
    _uart = hal.serial(5);
    
    if (_uart != nullptr) {
        // 设置波特率为115200
        _uart->begin(115200);
        hal.console->printf("DYsensor: initialized on Serial5\n");
    }
}

void AP_DYsensor::update()
{
    if (_uart == nullptr) {
        return;
    }

    send_data();

    // 检查是否有新数据
    while (_uart->available()) {
        uint8_t c = _uart->read();
        
        // 状态机处理数据
        switch (_parse_state) {
            case ParseState::WAITING_SYNC1:
                if (c == REC_SYNC_BYTE1) {
                    _rx_buffer[0] = c;
                    _rx_index = 1;
                    _parse_state = ParseState::WAITING_SYNC2;
                }
                break;
                
            case ParseState::WAITING_SYNC2:
                if (c == REC_SYNC_BYTE2) {
                    _rx_buffer[1] = c;
                    _rx_index = 2;
                    _parse_state = ParseState::COLLECTING_DATA;
                } else {
                    _parse_state = ParseState::WAITING_SYNC1;
                }
                break;
                
            case ParseState::COLLECTING_DATA:
                _rx_buffer[_rx_index++] = c;
                
                if (_rx_index >= REC_PACKET_LENGTH) {
                    if (_verify_checksum()) {
                        _parse_data();
                        hal.console->printf("DYsensor received valid packet: ");
                        for (uint8_t i = 0; i < REC_PACKET_LENGTH; i++) {
                            hal.console->printf("%02x ", _rx_buffer[i]);
                        }
                        hal.console->printf("\n");
                    } else {
                        hal.console->printf("DYsensor checksum error\n");
                    }
                    _parse_state = ParseState::WAITING_SYNC1;
                }
                break;
        }
    }
}

void AP_DYsensor::send_data()
{
    if (_uart == nullptr) {
        return;
    }
    
    // 计算并设置校验和
    _tx_buffer[15] = _calculate_checksum(_tx_buffer,SEND_PACKET_LENGTH);
    
    // 发送数据
    _uart->write(_tx_buffer, SEND_PACKET_LENGTH);
    
/*     hal.console->printf("DYsensor sent packet: ");
    for (uint8_t i = 0; i < SEND_PACKET_LENGTH; i++) {
        hal.console->printf("%02x ", _tx_buffer[i]);
    }
    hal.console->printf("\n"); */
}

bool AP_DYsensor::_verify_checksum()
{
    return (_rx_buffer[31] == _calculate_checksum(_rx_buffer,REC_PACKET_LENGTH));
}

uint8_t AP_DYsensor::_calculate_checksum(const uint8_t *buffer,const uint8_t PACKET_LENGTH)
{
    uint8_t sum = 0;
    // 计算前15字节的和
    for (uint8_t i = 0; i < PACKET_LENGTH-1; i++) {
        sum += buffer[i];
    }
    return sum;  // 自动取低8位
}

void AP_DYsensor::_parse_data()
{
    // 数据已经通过校验和验证，可以进行处理
    // 这里可以添加具体的数据处理代码
} 