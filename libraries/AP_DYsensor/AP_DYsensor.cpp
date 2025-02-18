#include "AP_DYsensor.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#define DYSENSOR_DEBUG 1

AP_DYsensor::AP_DYsensor() :
    _uart(nullptr),
    _healthy(false),
    _last_read_ms(0),
    _rx_index(0),
    _parse_state(ParseState::WAITING_SYNC1)
{
}

void AP_DYsensor::init()
{
    _uart = hal.serial(5);
    
    if (_uart != nullptr) {
        _uart->begin(115200);
#if DYSENSOR_DEBUG
        hal.console->printf("DYsensor: initialized on Serial5\n");
#endif
    } else {
#if DYSENSOR_DEBUG
        hal.console->printf("DYsensor: failed to initialize Serial5\n");
#endif
    }
}

void AP_DYsensor::update()
{
    if (_uart == nullptr) {
        return;
    }

    _read_sensor();
}

void AP_DYsensor::_read_sensor()
{
    // 检查是否有新数据可读
    while (_uart->available()) {
        uint8_t c = _uart->read();
        
        switch (_parse_state) {
            case ParseState::WAITING_SYNC1:
                if (c == SYNC_BYTE1) {
                    _rx_buffer[0] = c;
                    _rx_index = 1;
                    _parse_state = ParseState::WAITING_SYNC2;
#if DYSENSOR_DEBUG
                    hal.console->printf("DYsensor: Found SYNC1\n");
#endif
                }
                break;
                
            case ParseState::WAITING_SYNC2:
                if (c == SYNC_BYTE2) {
                    _rx_buffer[1] = c;
                    _rx_index = 2;
                    _parse_state = ParseState::COLLECTING_DATA;
#if DYSENSOR_DEBUG
                    hal.console->printf("DYsensor: Found SYNC2\n");
#endif
                } else {
                    _parse_state = ParseState::WAITING_SYNC1;
                }
                break;
                
            case ParseState::COLLECTING_DATA:
                _rx_buffer[_rx_index++] = c;
                
                if (_rx_index >= PACKET_LENGTH) {
                    if (_verify_checksum()) {
                        _parse_data();
#if DYSENSOR_DEBUG
                        hal.console->printf("DYsensor: Received packet: ");
                        for(uint8_t i=0; i<PACKET_LENGTH; i++) {
                            hal.console->printf("0x%02x ", _rx_buffer[i]);
                        }
                        hal.console->printf("\n");
#endif
                    } else {
#if DYSENSOR_DEBUG
                        hal.console->printf("DYsensor: Checksum error\n");
#endif
                    }
                    _parse_state = ParseState::WAITING_SYNC1;
                }
                break;
        }
    }
}

bool AP_DYsensor::_verify_checksum()
{
    uint8_t sum = 0;
    // 计算前31字节的和
    for (uint8_t i = 0; i < PACKET_LENGTH-1; i++) {
        sum += _rx_buffer[i];
    }
    // 检查校验和是否匹配
    return (sum == _rx_buffer[PACKET_LENGTH-1]);
}

void AP_DYsensor::_parse_data()
{
    // 数据有效，更新健康状态
    _healthy = true;
    _last_read_ms = AP_HAL::millis();
    
    // 这里可以添加具体的数据解析代码
    // 数据在 _rx_buffer[2] 到 _rx_buffer[30] 中

    
}

const AP_Param::GroupInfo AP_DYsensor::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: DYsensor Enable
    // @Description: Enable DYsensor
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE", 1, AP_DYsensor, enabled, 0),
    
    AP_GROUPEND
}; 