/*
  基于SPI总线的JEDEC Flash存储设备的日志记录实现
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_Logger_Block.h"

#if HAL_LOGGING_FLASH_JEDEC_ENABLED

// AP_Logger_Flash_JEDEC类继承自AP_Logger_Block,用于实现JEDEC Flash存储设备的日志记录
class AP_Logger_Flash_JEDEC : public AP_Logger_Block {
public:
    // 构造函数,初始化父类
    AP_Logger_Flash_JEDEC(AP_Logger &front, LoggerMessageWriter_DFLogStart *writer) :
        AP_Logger_Block(front, writer) {}
    
    // 探测函数,创建AP_Logger_Flash_JEDEC实例
    static AP_Logger_Backend  *probe(AP_Logger &front,
                                     LoggerMessageWriter_DFLogStart *ls) {
        return NEW_NOTHROW AP_Logger_Flash_JEDEC(front, ls);
    }
    
    // 初始化Flash设备
    void              Init(void) override;
    
    // 检查Flash是否可用
    bool              CardInserted() const override { return !flash_died && df_NumPages > 0; }

private:
    // 将缓冲区数据写入Flash页
    void              BufferToPage(uint32_t PageAdr) override;
    // 将Flash页数据读入缓冲区
    void              PageToBuffer(uint32_t PageAdr) override;
    // 擦除扇区
    void              SectorErase(uint32_t SectorAdr) override;
    // 擦除4KB扇区
    void              Sector4kErase(uint32_t SectorAdr) override;
    // 开始整片擦除
    void              StartErase() override;
    // 检查是否正在擦除
    bool              InErase() override;
    // 发送命令和地址
    void              send_command_addr(uint8_t cmd, uint32_t address);
    // 等待Flash就绪
    void              WaitReady();
    // 检查Flash是否忙
    bool              Busy();
    // 读取状态寄存器
    uint8_t           ReadStatusReg();
    // 进入4字节地址模式
    void              Enter4ByteAddressMode(void);

    // 使能写操作
    void              WriteEnable();
    // 获取扇区数量
    bool              getSectorCount(void);

    // SPI设备对象
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev;
    // SPI设备信号量
    AP_HAL::Semaphore *dev_sem;

    bool flash_died;          // Flash是否失效
    uint32_t erase_start_ms;  // 擦除开始时间
    uint8_t erase_cmd;        // 擦除命令
    bool use_32bit_address;   // 是否使用32位地址
    bool read_cache_valid;    // 读缓存是否有效
};

#endif // HAL_LOGGING_FLASH_JEDEC_ENABLED
