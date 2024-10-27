/*
  基于SPI的块存储设备的日志记录实现
 */
#pragma once

#include "AP_Logger_config.h"

#if HAL_LOGGING_FLASH_W25NXX_ENABLED

#include <AP_HAL/AP_HAL.h>

#include "AP_Logger_Block.h"

// W25NXX Flash存储器的日志记录类
class AP_Logger_W25NXX : public AP_Logger_Block {
public:
    // 构造函数
    AP_Logger_W25NXX(AP_Logger &front, LoggerMessageWriter_DFLogStart *writer) :
        AP_Logger_Block(front, writer) {}
    
    // 探测并创建W25NXX日志记录器实例
    static AP_Logger_Backend  *probe(AP_Logger &front,
                                     LoggerMessageWriter_DFLogStart *ls) {
        return NEW_NOTHROW AP_Logger_W25NXX(front, ls);
    }
    
    // 初始化Flash设备
    void              Init(void) override;
    
    // 检查Flash卡是否可用
    bool              CardInserted() const override { return !flash_died && df_NumPages > 0; }

private:
    // 将缓冲区数据写入指定页
    void              BufferToPage(uint32_t PageAdr) override;
    
    // 将指定页数据读入缓冲区
    void              PageToBuffer(uint32_t PageAdr) override;
    
    // 擦除指定扇区
    void              SectorErase(uint32_t SectorAdr) override;
    
    // 擦除4K大小的扇区
    void              Sector4kErase(uint32_t SectorAdr) override;
    
    // 开始擦除操作
    void              StartErase() override;
    
    // 检查是否正在擦除
    bool              InErase() override;
    
    // 发送命令和地址到Flash
    void              send_command_addr(uint8_t cmd, uint32_t address);
    
    // 等待Flash就绪
    void              WaitReady();
    
    // 检查Flash是否忙
    bool              Busy();
    
    // 读取状态寄存器指定位
    uint8_t           ReadStatusRegBits(uint8_t bits);
    
    // 写入状态寄存器
    void              WriteStatusReg(uint8_t reg, uint8_t bits);

    // 使能写操作
    void              WriteEnable();
    
    // 获取扇区数量
    bool              getSectorCount(void);

    // SPI设备对象
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev;
    
    // SPI设备信号量
    AP_HAL::Semaphore *dev_sem;

    // Flash块数量
    uint32_t flash_blockNum;

    bool flash_died;          // Flash是否失效
    uint32_t erase_start_ms;  // 擦除开始时间(毫秒)
    uint16_t erase_block;     // 当前擦除的块号
    bool read_cache_valid;    // 读缓存是否有效
};

#endif // HAL_LOGGING_FLASH_W25NXX_ENABLED
