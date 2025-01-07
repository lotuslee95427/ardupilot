#pragma once

#include <stdarg.h>
#include <AP_Common/AP_Common.h> // 用于FMT_PRINTF宏定义
#include "AP_HAL_Namespace.h"
#include <AP_Logger/AP_Logger_config.h>

// 是否启用堆内存管理功能
#ifndef ENABLE_HEAP 
#define ENABLE_HEAP 0
#endif

class ExpandingString;

// AP_HAL::Util类提供一些通用的工具函数和系统功能
class AP_HAL::Util {
public:
    // 格式化字符串输出函数,类似snprintf
    int snprintf(char* str, size_t size,
                 const char *format, ...) FMT_PRINTF(4, 5);

    // 带va_list参数的格式化字符串输出函数
    int vsnprintf(char* str, size_t size,
                  const char *format, va_list ap);

    // 设置软件解锁状态
    virtual void set_soft_armed(const bool b);
    // 获取软件解锁状态
    bool get_soft_armed() const { return soft_armed; }

    // 返回最后一次解锁状态改变的时间(毫秒)
    uint32_t get_last_armed_change() const { return last_armed_change_ms; };

    // 检查是否由看门狗复位导致的重启
    virtual bool was_watchdog_reset() const { return false; }

    // 检查在看门狗复位时安全开关是否关闭
    bool was_watchdog_safety_off() const {
        return was_watchdog_reset() && persistent_data.safety_state == SAFETY_ARMED;
    }

    // 检查是否在解锁状态下发生看门狗复位
    bool was_watchdog_armed() const { return was_watchdog_reset() && persistent_data.armed; }

    // 获取自定义日志目录路径
    virtual const char* get_custom_log_directory() const { return nullptr; }
    // 获取自定义地形数据目录路径
    virtual const char* get_custom_terrain_directory() const { return nullptr;  }
    // 获取自定义存储目录路径
    virtual const char *get_custom_storage_directory() const { return nullptr;  }

    // 获取AP_Param的自定义默认值文件路径
    virtual const char* get_custom_defaults_file() const {
        return HAL_PARAM_DEFAULTS_PATH;
    }

    // 启动时将命令行参数写入EEPROM
    virtual void set_cmdline_parameters() {};

    // 安全开关状态枚举
    enum safety_state : uint8_t {
        SAFETY_NONE,      // 无安全开关
        SAFETY_DISARMED,  // 已锁定
        SAFETY_ARMED,     // 已解锁
    };

    /*
      持久化数据结构。在看门狗复位后会恢复这些数据。
      只有在was_watchdog_reset()返回true时才应该读取这些数据。
      注意在STM32上这个结构限制为76字节
     */
    struct PersistentData {
        float roll_rad, pitch_rad, yaw_rad;  // 姿态角(弧度)
        int32_t home_lat, home_lon, home_alt_cm; // Home点位置
        uint32_t fault_addr;    // 故障地址
        uint32_t fault_icsr;    // 中断控制状态寄存器
        uint32_t fault_lr;      // 链接寄存器
        uint32_t internal_errors; // 内部错误
        uint16_t internal_error_count; // 内部错误计数
        uint16_t internal_error_last_line; // 最后一次内部错误行号
        uint32_t spi_count;     // SPI计数
        uint32_t i2c_count;     // I2C计数
        uint32_t i2c_isr_count; // I2C中断计数
        uint16_t waypoint_num;  // 航点编号
        uint16_t last_mavlink_msgid; // 最后一条MAVLink消息ID
        uint16_t last_mavlink_cmd;   // 最后一条MAVLink命令
        uint16_t semaphore_line;     // 信号量行号
        uint16_t fault_line;         // 故障行号
        uint8_t fault_type;          // 故障类型
        uint8_t fault_thd_prio;      // 故障线程优先级
        char thread_name4[4];        // 线程名称
        int8_t scheduler_task;       // 调度器任务
        bool armed;                  // 解锁状态
        enum safety_state safety_state; // 安全开关状态
        bool boot_to_dfu;           // 是否需要重启进入DFU模式
    };
    struct PersistentData persistent_data;
    // last_persistent_data仅在发生看门狗复位时填充
    struct PersistentData last_persistent_data;

    /*
      获取安全开关状态(如果有)
     */
    virtual enum safety_state safety_switch_state(void) { return SAFETY_NONE; }

    /*
      设置硬件RTC时间(UTC微秒)
     */
    virtual void set_hw_rtc(uint64_t time_utc_usec) = 0;

    /*
      获取系统时钟(UTC微秒)
     */
    virtual uint64_t get_hw_rtc() const = 0;

    // Bootloader刷写结果枚举
    enum class FlashBootloader {
        OK=0,             // 成功
        NO_CHANGE=1,      // 无变化
        FAIL=2,           // 失败
        NOT_AVAILABLE=3,  // 不可用
        NOT_SIGNED=4,     // 未签名
    };

    // 刷写bootloader(通常从ROMFS中获取)
    virtual FlashBootloader flash_bootloader() { return FlashBootloader::NOT_AVAILABLE; }

    /*
      获取系统标识符(如序列号)
      如果系统标识符不可用则返回false
      Buf应该填充可打印的字符串并以null结尾
     */
    virtual bool get_system_id(char buf[50]) { return false; }
    virtual bool get_system_id_unformatted(uint8_t buf[], uint8_t &len) { return false; }

    /**
       返回命令行参数(如果有)
     */
    virtual void commandline_arguments(uint8_t &argc, char * const *&argv) { argc = 0; }

    // 初始化蜂鸣器
    virtual bool toneAlarm_init(uint8_t types) { return false;}
    // 设置蜂鸣器音调
    virtual void toneAlarm_set_buzzer_tone(float frequency, float volume, uint32_t duration_ms) {}

    /* IMU加热系统支持 */
    virtual void set_imu_temp(float current) {}
    virtual void set_imu_target_temp(int8_t *target) {}
    
    // 内存类型枚举
    enum Memory_Type {
        MEM_DMA_SAFE,    // DMA安全内存
        MEM_FAST,        // 快速内存
        MEM_FILESYSTEM   // 文件系统内存
    };
    // 分配指定类型的内存,如果不支持则返回普通内存
    virtual void *malloc_type(size_t size, Memory_Type mem_type) { return calloc(1, size); }
    // 释放指定类型的内存
    virtual void free_type(void *ptr, size_t size, Memory_Type mem_type) { return free(ptr); }

#if ENABLE_HEAP
    // 堆内存管理函数,注意一旦分配就不能释放
    virtual void *allocate_heap_memory(size_t size) = 0;
    virtual void *heap_realloc(void *heap, void *ptr, size_t old_size, size_t new_size) = 0;
#if USE_LIBC_REALLOC
    virtual void *std_realloc(void *ptr, size_t new_size) { return realloc(ptr, new_size); }
#else
    virtual void *std_realloc(void *ptr, size_t new_size) = 0;
#endif // USE_LIBC_REALLOC
#endif // ENABLE_HEAP

    /**
       返回可用内存字节数,如果未知则返回4096
     */
    virtual uint32_t available_memory(void) { return 4096; }

    // 尝试捕获处理器,通常用于进入调试器
    virtual bool trap() const { return false; }

    // 获取运行线程信息
    virtual void thread_info(ExpandingString &str) {}

    // 获取DMA竞争信息
    virtual void dma_info(ExpandingString &str) {}

    // 获取内存分配信息
    virtual void mem_info(ExpandingString &str) {}

    // 从bootloader扇区加载持久化参数
    virtual bool load_persistent_params(ExpandingString &str) const { return false; }

    // 根据名称获取持久化参数
    virtual bool get_persistent_param_by_name(const char *name, char* value, size_t& len) const {
        return false;
    }

#if HAL_UART_STATS_ENABLED
    // 获取UART I/O信息
    virtual void uart_info(ExpandingString &str) {}

#if HAL_LOGGING_ENABLED
    // 记录每个串口的UART日志
    virtual void uart_log() {};
#endif
#endif // HAL_UART_STATS_ENABLED

    // 获取定时器频率信息
    virtual void timer_info(ExpandingString &str) {}

    // 生成随机值
    virtual bool get_random_vals(uint8_t* data, size_t size) { return false; }

    // 生成真随机值,会阻塞直到有足够的熵
    virtual bool get_true_random_vals(uint8_t* data, size_t size, uint32_t timeout_us) { return false; }

    // 记录栈使用信息
    virtual void log_stack_info(void) {}

#if AP_CRASHDUMP_ENABLED
    // 获取最后一次崩溃转储大小
    virtual size_t last_crash_dump_size() const { return 0; }
    // 获取最后一次崩溃转储指针
    virtual void* last_crash_dump_ptr() const { return nullptr; }
#endif

#if HAL_ENABLE_DFU_BOOT
    // 重启进入DFU模式
    virtual void boot_to_dfu(void) {}
#endif
protected:
    // 软件解锁状态初始为false,这样在车辆代码完全启动前执行器不会输出任何值
    bool soft_armed = false;
    // 最后一次解锁状态改变时间(毫秒)
    uint32_t last_armed_change_ms;
};
