/*
  基于SPI的DataFlash块存储设备的日志记录实现
*/

#include <AP_HAL/AP_HAL.h>

#include "AP_Logger_W25NXX.h"

#if HAL_LOGGING_FLASH_W25NXX_ENABLED

#include <stdio.h>

extern const AP_HAL::HAL& hal;

// JEDEC标准命令定义
#define JEDEC_WRITE_ENABLE           0x06    // 写使能
#define JEDEC_WRITE_DISABLE          0x04    // 写禁止
#define JEDEC_READ_STATUS            0x05    // 读状态寄存器
#define JEDEC_WRITE_STATUS           0x01    // 写状态寄存器
#define JEDEC_READ_DATA              0x03    // 读数据
#define JEDEC_PAGE_DATAa_READ         0x13   // 页数据读取
#define JEDEC_FAST_READ              0x0b    // 快速读取
#define JEDEC_DEVICE_ID              0x9F    // 读取设备ID
#define JEDEC_PAGE_WRITE             0x02    // 页写入
#define JEDEC_PROGRAM_EXECUTE        0x10    // 执行编程

#define JEDEC_DEVICE_RESET           0xFF    // 设备复位
#define JEDEC_BLOCK_ERASE            0xD8    // 128K块擦除

// 状态寄存器位定义
#define JEDEC_STATUS_BUSY            0x01    // 忙状态位
#define JEDEC_STATUS_WRITEPROTECT    0x02    // 写保护状态位

// W25NXX特定寄存器地址
#define W25NXX_STATUS_REG           0xC0    // 状态寄存器
#define W25NXX_PROT_REG             0xA0    // 保护寄存器
#define W25NXX_CONF_REG             0xB0    // 配置寄存器
#define W25NXX_STATUS_EFAIL         0x04    // 擦除失败标志
#define W25NXX_STATUS_PFAIL         0x08    // 编程失败标志

// 保护寄存器位定义
#define W25NXX_PROT_SRP1_ENABLE          (1 << 0)    // 状态寄存器保护1使能
#define W25NXX_PROT_WP_E_ENABLE          (1 << 1)    // 写保护使能
#define W25NXX_PROT_TB_ENABLE            (1 << 2)    // 顶部/底部保护使能
#define W25NXX_PROT_PB0_ENABLE           (1 << 3)    // 保护块0使能
#define W25NXX_PROT_PB1_ENABLE           (1 << 4)    // 保护块1使能
#define W25NXX_PROT_PB2_ENABLE           (1 << 5)    // 保护块2使能
#define W25NXX_PROT_PB3_ENABLE           (1 << 6)    // 保护块3使能
#define W25NXX_PROT_SRP2_ENABLE          (1 << 7)    // 状态寄存器保护2使能

// 配置寄存器位定义
#define W25NXX_CONFIG_ECC_ENABLE         (1 << 4)    // ECC使能
#define W25NXX_CONFIG_BUFFER_READ_MODE   (1 << 3)    // 缓冲读取模式使能

// 操作超时时间定义
#define W25NXX_TIMEOUT_PAGE_READ_US        60        // 页读取超时时间(ECC使能时)
#define W25NXX_TIMEOUT_PAGE_PROGRAM_US     700       // 页编程超时时间
#define W25NXX_TIMEOUT_BLOCK_ERASE_MS      10        // 块擦除超时时间
#define W25NXX_TIMEOUT_RESET_MS            500       // 复位超时时间

// 设备容量定义
#define W25N01G_NUM_BLOCKS                  1024     // W25N01G的块数量
#define W25N02K_NUM_BLOCKS                  2048     // W25N02K的块数量

// JEDEC设备ID定义
#define JEDEC_ID_WINBOND_W25N01GV      0xEFAA21    // W25N01GV的JEDEC ID
#define JEDEC_ID_WINBOND_W25N02KV      0xEFAA22    // W25N02KV的JEDEC ID

// 初始化函数
void AP_Logger_W25NXX::Init()
{
    // 获取SPI设备
    dev = hal.spi->get_device("dataflash");
    if (!dev) {
        AP_HAL::panic("PANIC: AP_Logger W25NXX device not found");
        return;
    }

    dev_sem = dev->get_semaphore();

    // 获取扇区数量,失败则标记flash故障
    if (!getSectorCount()) {
        flash_died = true;
        return;
    }

    flash_died = false;

    // 复位设备
    WaitReady();
    {
        WITH_SEMAPHORE(dev_sem);
        uint8_t b = JEDEC_DEVICE_RESET;
        dev->transfer(&b, 1, nullptr, 0);
    }
    hal.scheduler->delay(W25NXX_TIMEOUT_RESET_MS);

    // 禁用写保护
    WriteStatusReg(W25NXX_PROT_REG, 0);
    // 使能ECC和缓冲模式
    WriteStatusReg(W25NXX_CONF_REG, W25NXX_CONFIG_ECC_ENABLE|W25NXX_CONFIG_BUFFER_READ_MODE);

    // 打印状态寄存器值
    printf("W25NXX status: SR-1=0x%x, SR-2=0x%x, SR-3=0x%x\n",
        ReadStatusRegBits(W25NXX_PROT_REG),
        ReadStatusRegBits(W25NXX_CONF_REG),
        ReadStatusRegBits(W25NXX_STATUS_REG));

    AP_Logger_Block::Init();
}

/*
  等待忙标志清除
 */
void AP_Logger_W25NXX::WaitReady()
{
    if (flash_died) {
        return;
    }

    uint32_t t = AP_HAL::millis();
    while (Busy()) {
        hal.scheduler->delay_microseconds(100);
        if (AP_HAL::millis() - t > 5000) {
            printf("DataFlash: flash_died\n");
            flash_died = true;
            break;
        }
    }
}

// 获取扇区数量并初始化设备参数
bool AP_Logger_W25NXX::getSectorCount(void)
{
    WaitReady();

    WITH_SEMAPHORE(dev_sem);

    // 读取制造商ID
    uint8_t cmd = JEDEC_DEVICE_ID;
    uint8_t buf[4];
    dev->transfer(&cmd, 1, buf, 4);

    uint32_t id = buf[1] << 16 | buf[2] << 8 | buf[3];

    // 根据设备ID设置参数
    switch (id) {
    case JEDEC_ID_WINBOND_W25N01GV:
        df_PageSize = 2048;          // 页大小
        df_PagePerBlock = 64;        // 每块的页数
        df_PagePerSector = 64;       // 每扇区的页数(使扇区等同于块)
        flash_blockNum = W25N01G_NUM_BLOCKS;
        break;
    case JEDEC_ID_WINBOND_W25N02KV:
        df_PageSize = 2048;
        df_PagePerBlock = 64;
        df_PagePerSector = 64;
        flash_blockNum = W25N02K_NUM_BLOCKS;
        break;

    default:
        hal.scheduler->delay(2000);
        printf("Unknown SPI Flash 0x%08x\n", id);
        return false;
    }

    df_NumPages = flash_blockNum * df_PagePerBlock;

    printf("SPI Flash 0x%08x found pages=%u\n", id, df_NumPages);
    return true;
}

// 读取状态寄存器位
uint8_t AP_Logger_W25NXX::ReadStatusRegBits(uint8_t bits)
{
    WITH_SEMAPHORE(dev_sem);
    uint8_t cmd[2] { JEDEC_READ_STATUS, bits };
    uint8_t status;
    dev->transfer(cmd, 2, &status, 1);
    return status;
}

// 写状态寄存器
void AP_Logger_W25NXX::WriteStatusReg(uint8_t reg, uint8_t bits)
{
    WaitReady();
    WITH_SEMAPHORE(dev_sem);
    uint8_t cmd[3] = {JEDEC_WRITE_STATUS, reg, bits};
    dev->transfer(cmd, 3, nullptr, 0);
}

// 检查设备是否忙
bool AP_Logger_W25NXX::Busy()
{
    uint8_t status = ReadStatusRegBits(W25NXX_STATUS_REG);

    // 检查编程和擦除失败标志
    if ((status & W25NXX_STATUS_PFAIL) != 0) {
        printf("Program failure!\n");
    }
    if ((status & W25NXX_STATUS_EFAIL) != 0) {
        printf("Erase failure!\n");
    }

    return (status & JEDEC_STATUS_BUSY) != 0;
}

/*
  发送带地址的命令
*/
void AP_Logger_W25NXX::send_command_addr(uint8_t command, uint32_t PageAdr)
{
    uint8_t cmd[4];
    cmd[0] = command;
    cmd[1] = (PageAdr >>  16) & 0xff;  // 地址高字节
    cmd[2] = (PageAdr >>  8) & 0xff;   // 地址中字节
    cmd[3] = (PageAdr >>  0) & 0xff;   // 地址低字节

    dev->transfer(cmd, 4, nullptr, 0);
}

// 将页数据读入缓冲区
void AP_Logger_W25NXX::PageToBuffer(uint32_t pageNum)
{
    if (pageNum == 0 || pageNum > df_NumPages+1) {
        printf("Invalid page read %u\n", pageNum);
        memset(buffer, 0xFF, df_PageSize);
        df_Read_PageAdr = pageNum;
        return;
    }

    // 如果已经读取了这个页面
    if (pageNum == df_Read_PageAdr && read_cache_valid) {
        return;
    }

    df_Read_PageAdr = pageNum;

    WaitReady();

    uint32_t PageAdr = (pageNum-1);

    {
        WITH_SEMAPHORE(dev_sem);
        // 将页数据读入内部缓冲区
        send_command_addr(JEDEC_PAGE_DATA_READ, PageAdr);
    }

    // 从内部缓冲区读入我们的缓冲区
    WaitReady();
    {
        WITH_SEMAPHORE(dev_sem);
        dev->set_chip_select(true);
        uint8_t cmd[4];
        cmd[0] = JEDEC_READ_DATA;
        cmd[1] = (0 >>  8) & 0xff;  // 列地址高字节
        cmd[2] = (0 >>  0) & 0xff;  // 列地址低字节
        cmd[3] = 0;                 // 空字节
        dev->transfer(cmd, 4, nullptr, 0);
        dev->transfer(nullptr, 0, buffer, df_PageSize);
        dev->set_chip_select(false);

        read_cache_valid = true;
    }
}

// 将缓冲区数据写入页
void AP_Logger_W25NXX::BufferToPage(uint32_t pageNum)
{
    if (pageNum == 0 || pageNum > df_NumPages+1) {
        printf("Invalid page write %u\n", pageNum);
        return;
    }

    // 如果写入的不是缓存页
    if (pageNum != df_Read_PageAdr) {
        read_cache_valid = false;
    }

    WriteEnable();

    uint32_t PageAdr = (pageNum-1);
    {
        WITH_SEMAPHORE(dev_sem);

        // 将我们的缓冲区写入内部缓冲区
        dev->set_chip_select(true);

        uint8_t cmd[3];
        cmd[0] = JEDEC_PAGE_WRITE;
        cmd[1] = (0 >>  8) & 0xff;  // 列地址高字节
        cmd[2] = (0 >>  0) & 0xff;  // 列地址低字节

        dev->transfer(cmd, 3, nullptr, 0);
        dev->transfer(buffer, df_PageSize, nullptr, 0);
        dev->set_chip_select(false);
    }

    // 将内部缓冲区数据写入页
    {
        WITH_SEMAPHORE(dev_sem);
        send_command_addr(JEDEC_PROGRAM_EXECUTE, PageAdr);
    }
}

/*
  擦除一个扇区(大小因硬件而异)
*/
void AP_Logger_W25NXX::SectorErase(uint32_t blockNum)
{
    WriteEnable();
    WITH_SEMAPHORE(dev_sem);

    uint32_t PageAdr = blockNum  * df_PagePerBlock;
    send_command_addr(JEDEC_BLOCK_ERASE, PageAdr);
}

/*
  擦除一个4k扇区
*/
void AP_Logger_W25NXX::Sector4kErase(uint32_t sectorNum)
{
    SectorErase(sectorNum);
}

// 开始擦除操作
void AP_Logger_W25NXX::StartErase()
{
    WriteEnable();

    WITH_SEMAPHORE(dev_sem);

    // 只擦除第一个块,其他块将在InErase中擦除
    send_command_addr(JEDEC_BLOCK_ERASE, 0);

    erase_block = 1;
    erase_start_ms = AP_HAL::millis();
    printf("Dataflash: erase started\n");
}

// 检查擦除是否在进行中
bool AP_Logger_W25NXX::InErase()
{
    if (erase_start_ms && !Busy()) {
        if (erase_block < flash_blockNum) {
            SectorErase(erase_block++);
        } else {
            printf("Dataflash: erase done (%u ms)\n", AP_HAL::millis() - erase_start_ms);
            erase_start_ms = 0;
            erase_block = 0;
        }
    }
    return erase_start_ms != 0;
}

// 使能写操作
void AP_Logger_W25NXX::WriteEnable(void)
{
    WaitReady();
    WITH_SEMAPHORE(dev_sem);
    uint8_t b = JEDEC_WRITE_ENABLE;
    dev->transfer(&b, 1, nullptr, 0);
}

#endif // HAL_LOGGING_FLASH_W25NXX_ENABLED
