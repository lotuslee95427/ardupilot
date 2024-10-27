/*
  基于SPI总线的JEDEC Flash存储设备的日志记录实现
  
  该模块实现了将日志数据写入JEDEC标准的Flash存储设备的功能
  通过SPI总线与Flash芯片通信
  支持多种厂商的Flash芯片,如Macronix、Micron、Winbond等
  提供数据读写、擦除等基本操作
*/

#include "AP_Logger_config.h"

#if HAL_LOGGING_FLASH_JEDEC_ENABLED

#include <AP_HAL/AP_HAL.h>

#include "AP_Logger_Flash_JEDEC.h"

#include <stdio.h>

extern const AP_HAL::HAL& hal;

// JEDEC Flash命令定义
#define JEDEC_WRITE_ENABLE           0x06    // 写使能命令
#define JEDEC_WRITE_DISABLE          0x04    // 写禁止命令
#define JEDEC_READ_STATUS            0x05    // 读状态寄存器命令
#define JEDEC_WRITE_STATUS           0x01    // 写状态寄存器命令
#define JEDEC_READ_DATA              0x03    // 读数据命令
#define JEDEC_FAST_READ              0x0b    // 快速读取命令
#define JEDEC_DEVICE_ID              0x9F    // 读取设备ID命令
#define JEDEC_PAGE_WRITE             0x02    // 页写入命令

// 擦除相关命令
#define JEDEC_BULK_ERASE             0xC7    // 整片擦除命令
#define JEDEC_SECTOR4_ERASE          0x20    // 4KB扇区擦除命令
#define JEDEC_BLOCK32_ERASE          0x52    // 32KB块擦除命令
#define JEDEC_BLOCK64_ERASE          0xD8    // 64KB块擦除命令

// 状态寄存器位定义
#define JEDEC_STATUS_BUSY            0x01    // 忙状态位
#define JEDEC_STATUS_WRITEPROTECT    0x02    // 写保护状态位
#define JEDEC_STATUS_BP0             0x04    // 块保护位0
#define JEDEC_STATUS_BP1             0x08    // 块保护位1
#define JEDEC_STATUS_BP2             0x10    // 块保护位2
#define JEDEC_STATUS_TP              0x20    // 临时保护位
#define JEDEC_STATUS_SEC             0x40    // 安全位
#define JEDEC_STATUS_SRP0            0x80    // 状态寄存器保护位0

/*
  Flash设备ID定义,来自betaflight的flash_m25p16.c
  
  格式为:制造商ID + 存储器类型 + 容量
*/
#define JEDEC_ID_MACRONIX_MX25L3206E   0xC22016  // Macronix 32Mbit
#define JEDEC_ID_MACRONIX_MX25L6406E   0xC22017  // Macronix 64Mbit
#define JEDEC_ID_MACRONIX_MX25L25635E  0xC22019  // Macronix 256Mbit
#define JEDEC_ID_MICRON_M25P16         0x202015  // Micron 16Mbit
#define JEDEC_ID_MICRON_N25Q064        0x20BA17  // Micron 64Mbit
#define JEDEC_ID_MICRON_N25Q128        0x20ba18  // Micron 128Mbit
#define JEDEC_ID_WINBOND_W25Q16        0xEF4015  // Winbond 16Mbit
#define JEDEC_ID_WINBOND_W25Q32        0xEF4016  // Winbond 32Mbit
#define JEDEC_ID_WINBOND_W25X32        0xEF3016  // Winbond 32Mbit
#define JEDEC_ID_WINBOND_W25Q64        0xEF4017  // Winbond 64Mbit
#define JEDEC_ID_WINBOND_W25Q128       0xEF4018  // Winbond 128Mbit
#define JEDEC_ID_WINBOND_W25Q256       0xEF4019  // Winbond 256Mbit
#define JEDEC_ID_WINBOND_W25Q128_2     0xEF7018  // Winbond 128Mbit另一型号
#define JEDEC_ID_CYPRESS_S25FL128L     0x016018  // Cypress 128Mbit

// 初始化Flash设备
void AP_Logger_Flash_JEDEC::Init()
{
    // 获取SPI设备实例
    dev = hal.spi->get_device("dataflash");
    if (!dev) {
        AP_HAL::panic("PANIC: AP_Logger SPIDeviceDriver not found");
        return;
    }

    // 获取设备信号量
    dev_sem = dev->get_semaphore();

    // 获取扇区数量,失败则标记Flash故障
    if (!getSectorCount()) {
        flash_died = true;
        return;
    }

    // 如果需要,进入4字节地址模式
    if (use_32bit_address) {
        Enter4ByteAddressMode();
    }

    flash_died = false;

    // 调用父类初始化
    AP_Logger_Block::Init();
}

/*
  等待Flash设备就绪
  检查忙状态标志是否清除
 */
void AP_Logger_Flash_JEDEC::WaitReady()
{
    if (flash_died) {
        return;
    }

    uint32_t t = AP_HAL::millis();
    while (Busy()) {
        hal.scheduler->delay_microseconds(100);
        // 超时5秒则标记Flash故障
        if (AP_HAL::millis() - t > 5000) {
            printf("DataFlash: flash_died\n");
            flash_died = true;
            break;
        }
    }
}

// 获取Flash扇区数量并配置相关参数
bool AP_Logger_Flash_JEDEC::getSectorCount(void)
{
    WaitReady();

    WITH_SEMAPHORE(dev_sem);

    // 读取制造商ID
    uint8_t cmd = JEDEC_DEVICE_ID;
    uint8_t buf[4];
    dev->transfer(&cmd, 1, buf, 4);

    uint32_t id = buf[0] << 16 | buf[1] << 8 | buf[2];

    uint32_t blocks = 0;

    // 根据不同的Flash型号配置参数
    switch (id) {
    case JEDEC_ID_WINBOND_W25Q16:
    case JEDEC_ID_MICRON_M25P16:
        blocks = 32;
        df_PagePerBlock = 256;
        df_PagePerSector = 16;
        break;
    case JEDEC_ID_WINBOND_W25Q32:
    case JEDEC_ID_WINBOND_W25X32:
    case JEDEC_ID_MACRONIX_MX25L3206E:
        blocks = 64;
        df_PagePerBlock = 256;
        df_PagePerSector = 16;
        break;
    case JEDEC_ID_MICRON_N25Q064:
    case JEDEC_ID_WINBOND_W25Q64:
    case JEDEC_ID_MACRONIX_MX25L6406E:
        blocks = 128;
        df_PagePerBlock = 256;
        df_PagePerSector = 16;
        break;
    case JEDEC_ID_MICRON_N25Q128:
    case JEDEC_ID_WINBOND_W25Q128:
    case JEDEC_ID_WINBOND_W25Q128_2:
    case JEDEC_ID_CYPRESS_S25FL128L:
        blocks = 256;
        df_PagePerBlock = 256;
        df_PagePerSector = 16;
        break;
    case JEDEC_ID_WINBOND_W25Q256:
    case JEDEC_ID_MACRONIX_MX25L25635E:
        blocks = 512;
        df_PagePerBlock = 256;
        df_PagePerSector = 16;
        use_32bit_address = true;
        break;
    default:
        hal.scheduler->delay(2000);
        printf("Unknown SPI Flash 0x%08x\n", id);
        return false;
    }

    // 配置页大小和总页数
    df_PageSize = 256;
    df_NumPages = blocks * df_PagePerBlock;
    erase_cmd = JEDEC_BLOCK64_ERASE;

    printf("SPI Flash 0x%08x found pages=%u erase=%uk\n",
           id, df_NumPages, (df_PagePerBlock * (uint32_t)df_PageSize)/1024);
    return true;
}

// 读取状态寄存器
uint8_t AP_Logger_Flash_JEDEC::ReadStatusReg()
{
    WITH_SEMAPHORE(dev_sem);
    uint8_t cmd = JEDEC_READ_STATUS;
    uint8_t status;
    dev->transfer(&cmd, 1, &status, 1);
    return status;
}

// 检查Flash是否处于忙状态
bool AP_Logger_Flash_JEDEC::Busy()
{
    return (ReadStatusReg() & (JEDEC_STATUS_BUSY | JEDEC_STATUS_SRP0)) != 0;
}

// 进入4字节地址模式
void AP_Logger_Flash_JEDEC::Enter4ByteAddressMode(void)
{
    WITH_SEMAPHORE(dev_sem);

    const uint8_t cmd = 0xB7;
    dev->transfer(&cmd, 1, nullptr, 0);
}

/*
  发送带地址的命令
  command: 命令字节
  PageAdr: 页地址
*/
void AP_Logger_Flash_JEDEC::send_command_addr(uint8_t command, uint32_t PageAdr)
{
    uint8_t cmd[5];
    cmd[0] = command;
    if (use_32bit_address) {
        // 4字节地址模式
        cmd[1] = (PageAdr >> 24) & 0xff;
        cmd[2] = (PageAdr >> 16) & 0xff;
        cmd[3] = (PageAdr >>  8) & 0xff;
        cmd[4] = (PageAdr >>  0) & 0xff;
    } else {
        // 3字节地址模式
        cmd[1] = (PageAdr >> 16) & 0xff;
        cmd[2] = (PageAdr >>  8) & 0xff;
        cmd[3] = (PageAdr >>  0) & 0xff;
    }

    dev->transfer(cmd, use_32bit_address?5:4, nullptr, 0);
}

// 将Flash页数据读入缓冲区
void AP_Logger_Flash_JEDEC::PageToBuffer(uint32_t pageNum)
{
    // 检查页号有效性
    if (pageNum == 0 || pageNum > df_NumPages+1) {
        printf("Invalid page read %u\n", pageNum);
        memset(buffer, 0xFF, df_PageSize);
        df_Read_PageAdr = pageNum;
        return;
    }

    // 如果页已在缓存中则直接返回
    if (pageNum == df_Read_PageAdr && read_cache_valid) {
        return;
    }

    df_Read_PageAdr = pageNum;

    WaitReady();

    uint32_t PageAdr = (pageNum-1) * df_PageSize;

    WITH_SEMAPHORE(dev_sem);
    dev->set_chip_select(true);
    send_command_addr(JEDEC_READ_DATA, PageAdr);
    dev->transfer(nullptr, 0, buffer, df_PageSize);
    dev->set_chip_select(false);

    read_cache_valid = true;
}

// 将缓冲区数据写入Flash页
void AP_Logger_Flash_JEDEC::BufferToPage(uint32_t pageNum)
{
    // 检查页号有效性
    if (pageNum == 0 || pageNum > df_NumPages+1) {
        printf("Invalid page write %u\n", pageNum);
        return;
    }

    // 如果写入的不是当前缓存页,则使缓存无效
    if (pageNum != df_Read_PageAdr) {
        read_cache_valid = false;
    }

    WriteEnable();

    WITH_SEMAPHORE(dev_sem);

    uint32_t PageAdr = (pageNum-1) * df_PageSize;

    dev->set_chip_select(true);
    send_command_addr(JEDEC_PAGE_WRITE, PageAdr);
    dev->transfer(buffer, df_PageSize, nullptr, 0);
    dev->set_chip_select(false);
}

/*
  擦除一个扇区(大小因硬件而异)
*/
void AP_Logger_Flash_JEDEC::SectorErase(uint32_t blockNum)
{
    WriteEnable();

    WITH_SEMAPHORE(dev_sem);

    uint32_t PageAdr = blockNum * df_PageSize * df_PagePerBlock;
    send_command_addr(erase_cmd, PageAdr);
}

/*
  擦除一个4KB扇区
*/
void AP_Logger_Flash_JEDEC::Sector4kErase(uint32_t sectorNum)
{
    WriteEnable();

    WITH_SEMAPHORE(dev_sem);
    uint32_t SectorAddr = sectorNum * df_PageSize * df_PagePerSector;
    send_command_addr(JEDEC_SECTOR4_ERASE, SectorAddr);
}

// 开始整片擦除
void AP_Logger_Flash_JEDEC::StartErase()
{
    WriteEnable();

    WITH_SEMAPHORE(dev_sem);

    uint8_t cmd = JEDEC_BULK_ERASE;
    dev->transfer(&cmd, 1, nullptr, 0);

    erase_start_ms = AP_HAL::millis();
    printf("Dataflash: erase started\n");
}

// 检查是否正在擦除
bool AP_Logger_Flash_JEDEC::InErase()
{
    if (erase_start_ms && !Busy()) {
        printf("Dataflash: erase done (%u ms)\n", AP_HAL::millis() - erase_start_ms);
        erase_start_ms = 0;
    }
    return erase_start_ms != 0;
}

// 使能写操作
void AP_Logger_Flash_JEDEC::WriteEnable(void)
{
    WaitReady();
    WITH_SEMAPHORE(dev_sem);
    uint8_t b = JEDEC_WRITE_ENABLE;
    dev->transfer(&b, 1, nullptr, 0);
}

#endif // HAL_LOGGING_FLASH_JEDEC_ENABLED
