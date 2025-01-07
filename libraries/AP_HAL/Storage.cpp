// 包含必要的头文件
#include "AP_HAL.h"
#include "Storage.h"
#include <AP_Math/AP_Math.h>

/*
  默认的存储擦除方法
  该方法会将存储区域的所有内容清零
 */
bool AP_HAL::Storage::erase(void)
{
    // 定义一个16字节的缓冲区数组,初始化为0
    uint8_t blk[16] {};
    // 定义偏移量变量
    uint32_t ofs;
    
    // 循环遍历整个存储区域
    for (ofs=0; ofs<HAL_STORAGE_SIZE; ofs += sizeof(blk)) {
        // 计算本次要写入的字节数,取缓冲区大小和剩余空间的较小值
        uint32_t n = MIN(sizeof(blk), HAL_STORAGE_SIZE - ofs);
        // 将缓冲区的数据(全0)写入到存储区域
        write_block(ofs, blk, n);
    }
    // 擦除完成,返回成功
    return true;
}
