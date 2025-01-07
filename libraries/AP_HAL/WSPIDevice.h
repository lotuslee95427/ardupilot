/*
 * 本文件是自由软件:你可以根据自由软件基金会发布的GNU通用公共许可证的条款,
 * 即许可证的第3版或(您选择的)任何后来的版本重新发布和/或修改它。
 *
 * 本文件的发布是希望它能起到作用。但没有任何保证；甚至没有隐含的适销性或
 * 适合特定用途的保证。更多细节请参见GNU通用公共许可证。
 *
 * 您应该已经收到了GNU通用公共许可证的副本。如果没有，
 * 请参阅<http://www.gnu.org/licenses/>。
 *
 * 代码作者: Andy Piper 和 Siddharth Bharat Purohit
 */

// 防止头文件重复包含
#pragma once

#include <inttypes.h>

#include "AP_HAL_Namespace.h"
#include "Device.h"
#include "utility/OwnPtr.h"

// 是否使用WSPI默认配置
#ifndef HAL_USE_WSPI_DEFAULT_CFG
#define HAL_USE_WSPI_DEFAULT_CFG 1
#endif

namespace AP_HAL
{

// 底层HAL实现可以重写这些配置
#if HAL_USE_WSPI_DEFAULT_CFG
namespace WSPI
{
#if HAL_USE_QUADSPI
// QUADSPI模式下的配置参数定义
// 命令模式掩码和各种模式定义
constexpr uint32_t CFG_CMD_MODE_MASK        =   (3LU << 8LU);     // 命令模式掩码
constexpr uint32_t CFG_CMD_MODE_NONE        =   (0LU << 8LU);     // 无命令模式
constexpr uint32_t CFG_CMD_MODE_ONE_LINE    =   (1LU << 8LU);     // 单线命令模式
constexpr uint32_t CFG_CMD_MODE_TWO_LINES   =   (2LU << 8LU);     // 双线命令模式
constexpr uint32_t CFG_CMD_MODE_FOUR_LINES  =   (3LU << 8LU);     // 四线命令模式

// 命令大小配置
constexpr uint32_t CFG_CMD_SIZE_MASK     =  0LU;                  // 命令大小掩码
constexpr uint32_t CFG_CMD_SIZE_8        =  0LU;                  // 8位命令大小

// 地址模式配置
constexpr uint32_t CFG_ADDR_MODE_MASK        =  (3LU << 10LU);    // 地址模式掩码
constexpr uint32_t CFG_ADDR_MODE_NONE        =  (0LU << 10LU);    // 无地址模式
constexpr uint32_t CFG_ADDR_MODE_ONE_LINE    =  (1LU << 10LU);    // 单线地址模式
constexpr uint32_t CFG_ADDR_MODE_TWO_LINES   =  (2LU << 10LU);    // 双线地址模式
constexpr uint32_t CFG_ADDR_MODE_FOUR_LINES  =  (3LU << 10LU);    // 四线地址模式

// 地址大小配置
constexpr uint32_t CFG_ADDR_SIZE_MASK   =  (3LU << 12LU);         // 地址大小掩码
constexpr uint32_t CFG_ADDR_SIZE_8      =  (0LU << 12LU);         // 8位地址
constexpr uint32_t CFG_ADDR_SIZE_16     =  (1LU << 12LU);         // 16位地址
constexpr uint32_t CFG_ADDR_SIZE_24     =  (2LU << 12LU);         // 24位地址
constexpr uint32_t CFG_ADDR_SIZE_32     =  (3LU << 12LU);         // 32位地址

// 备用模式配置
constexpr uint32_t CFG_ALT_MODE_MASK        =  (3LU << 14LU);     // 备用模式掩码
constexpr uint32_t CFG_ALT_MODE_NONE        =  (0LU << 14LU);     // 无备用模式
constexpr uint32_t CFG_ALT_MODE_ONE_LINE    =  (1LU << 14LU);     // 单线备用模式
constexpr uint32_t CFG_ALT_MODE_TWO_LINES   =  (2LU << 14LU);     // 双线备用模式
constexpr uint32_t CFG_ALT_MODE_FOUR_LINES  =  (3LU << 14LU);     // 四线备用模式

// 备用DDR模式
constexpr uint32_t CFG_ALT_DDR              =  (1LU << 31LU);     // 备用DDR模式使能

// 备用大小配置
constexpr uint32_t CFG_ALT_SIZE_MASK        =  (3LU << 16LU);     // 备用大小掩码
constexpr uint32_t CFG_ALT_SIZE_8           =  (0LU << 16LU);     // 8位备用大小
constexpr uint32_t CFG_ALT_SIZE_16          =  (1LU << 16LU);     // 16位备用大小
constexpr uint32_t CFG_ALT_SIZE_24          =  (2LU << 16LU);     // 24位备用大小
constexpr uint32_t CFG_ALT_SIZE_32          =  (3LU << 16LU);     // 32位备用大小

// 数据模式配置
constexpr uint32_t CFG_DATA_MODE_MASK       =  (3LU << 24LU);     // 数据模式掩码
constexpr uint32_t CFG_DATA_MODE_NONE       =  (0LU << 24LU);     // 无数据模式
constexpr uint32_t CFG_DATA_MODE_ONE_LINE   =  (1LU << 24LU);     // 单线数据模式
constexpr uint32_t CFG_DATA_MODE_TWO_LINES  =  (2LU << 24LU);     // 双线数据模式
constexpr uint32_t CFG_DATA_MODE_FOUR_LINES =  (3LU << 24LU);     // 四线数据模式

// 数据DDR模式
constexpr uint32_t CFG_DATA_DDR             =  (1LU << 31LU);     // 数据DDR模式使能

// 发送指令优化模式
constexpr uint32_t CFG_SIOO                 =  (1LU << 28LU);     // 发送指令优化使能

#else   // OCTOSPI模式下的配置参数定义
// 命令模式配置
constexpr uint32_t CFG_CMD_MODE_MASK        =   (7LU << 0LU);     // 命令模式掩码
constexpr uint32_t CFG_CMD_MODE_NONE        =   (0LU << 0LU);     // 无命令模式
constexpr uint32_t CFG_CMD_MODE_ONE_LINE    =   (1LU << 0LU);     // 单线命令模式
constexpr uint32_t CFG_CMD_MODE_TWO_LINES   =   (2LU << 0LU);     // 双线命令模式
constexpr uint32_t CFG_CMD_MODE_FOUR_LINES  =   (3LU << 0LU);     // 四线命令模式
constexpr uint32_t CFG_CMD_MODE_EIGHT_LINES  =  (4LU << 0LU);     // 八线命令模式

// 命令大小配置
constexpr uint32_t CFG_CMD_SIZE_MASK     =  (3LU << 4LU);         // 命令大小掩码
constexpr uint32_t CFG_CMD_SIZE_8        =  (0LU << 4LU);         // 8位命令大小

// 地址模式配置
constexpr uint32_t CFG_ADDR_MODE_MASK        =  (7LU << 8LU);     // 地址模式掩码
constexpr uint32_t CFG_ADDR_MODE_NONE        =  (0LU << 8LU);     // 无地址模式
constexpr uint32_t CFG_ADDR_MODE_ONE_LINE    =  (1LU << 8LU);     // 单线地址模式
constexpr uint32_t CFG_ADDR_MODE_TWO_LINES   =  (2LU << 8LU);     // 双线地址模式
constexpr uint32_t CFG_ADDR_MODE_FOUR_LINES  =  (3LU << 8LU);     // 四线地址模式
constexpr uint32_t CFG_ADDR_MODE_EIGHT_LINES  = (4LU << 8LU);     // 八线地址模式

// 地址大小配置
constexpr uint32_t CFG_ADDR_SIZE_MASK   =  (3LU << 12LU);         // 地址大小掩码
constexpr uint32_t CFG_ADDR_SIZE_8      =  (0LU << 12LU);         // 8位地址
constexpr uint32_t CFG_ADDR_SIZE_16     =  (1LU << 12LU);         // 16位地址
constexpr uint32_t CFG_ADDR_SIZE_24     =  (2LU << 12LU);         // 24位地址
constexpr uint32_t CFG_ADDR_SIZE_32     =  (3LU << 12LU);         // 32位地址

// 备用模式配置
constexpr uint32_t CFG_ALT_MODE_MASK        =  (7LU << 16LU);     // 备用模式掩码
constexpr uint32_t CFG_ALT_MODE_NONE        =  (0LU << 16LU);     // 无备用模式
constexpr uint32_t CFG_ALT_MODE_ONE_LINE    =  (1LU << 16LU);     // 单线备用模式
constexpr uint32_t CFG_ALT_MODE_TWO_LINES   =  (2LU << 16LU);     // 双线备用模式
constexpr uint32_t CFG_ALT_MODE_FOUR_LINES  =  (3LU << 16LU);     // 四线备用模式
constexpr uint32_t CFG_ALT_MODE_EIGHT_LINES  = (4LU << 16LU);     // 八线备用模式

// 备用DDR模式
constexpr uint32_t CFG_ALT_DDR              =  (1LU << 19LU);     // 备用DDR模式使能

// 备用大小配置
constexpr uint32_t CFG_ALT_SIZE_MASK        =  (3LU << 20LU);     // 备用大小掩码
constexpr uint32_t CFG_ALT_SIZE_8           =  (0LU << 20LU);     // 8位备用大小
constexpr uint32_t CFG_ALT_SIZE_16          =  (1LU << 20LU);     // 16位备用大小
constexpr uint32_t CFG_ALT_SIZE_24          =  (2LU << 20LU);     // 24位备用大小
constexpr uint32_t CFG_ALT_SIZE_32          =  (3LU << 20LU);     // 32位备用大小

// 数据模式配置
constexpr uint32_t CFG_DATA_MODE_MASK       =  (7LU << 24LU);     // 数据模式掩码
constexpr uint32_t CFG_DATA_MODE_NONE       =  (0LU << 24LU);     // 无数据模式
constexpr uint32_t CFG_DATA_MODE_ONE_LINE   =  (1LU << 24LU);     // 单线数据模式
constexpr uint32_t CFG_DATA_MODE_TWO_LINES  =  (2LU << 24LU);     // 双线数据模式
constexpr uint32_t CFG_DATA_MODE_FOUR_LINES =  (3LU << 24LU);     // 四线数据模式
constexpr uint32_t CFG_DATA_MODE_EIGHT_LINES=  (4LU << 24LU);     // 八线数据模式

// 数据DDR模式
constexpr uint32_t CFG_DATA_DDR             =  (1LU << 27LU);     // 数据DDR模式使能

// 发送指令优化模式
constexpr uint32_t CFG_SIOO                 =  (1LU << 31LU);     // 发送指令优化使能

#endif // HAL_USE_QUADSPI
}
#endif //#if HAL_USE_WSPI_DEFAULT_CFG

// WSPI设备类,继承自Device基类
class WSPIDevice : public Device
{
public:
    // 构造函数,设置总线类型为WSPI
    WSPIDevice() : Device(BUS_TYPE_WSPI) { }

    /* 实现Device类的transfer()方法 */
    virtual bool transfer(const uint8_t *send, uint32_t send_len,
                          uint8_t *recv, uint32_t recv_len) override = 0;

    // 为即将到来的传输调用设置命令头
    virtual void set_cmd_header(const CommandHeader& cmd_hdr) override = 0;

    // 检查设备是否忙
    virtual bool is_busy() = 0;

    // 获取信号量
    virtual AP_HAL::Semaphore* get_semaphore() override = 0;

protected:
    uint32_t _trx_flags;  // 传输标志
};

// WSPI设备管理器类
class WSPIDeviceManager
{
public:
    // 根据名称获取WSPI设备
    virtual OwnPtr<WSPIDevice> get_device(const char *name)
    {
        return nullptr;
    }

    /* 返回当前注册的WSPI设备数量 */
    virtual uint8_t get_count() const
    {
        return 0;
    }

    /* 获取指定索引的WSPI设备名称 */
    virtual const char *get_device_name(uint8_t idx) const
    {
        return nullptr;
    }
};

}
