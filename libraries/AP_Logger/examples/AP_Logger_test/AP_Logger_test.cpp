/*
 * AP_Logger库的示例代码
 * 最初基于Jordi MuÒoz和Jose Julio的代码
 */

// 包含必要的头文件
#include <AP_HAL/AP_HAL.h>        // HAL(硬件抽象层)
#include <AP_Scheduler/AP_Scheduler.h>  // 调度器
#include <AP_Logger/AP_Logger.h>   // 日志记录器
#include <GCS_MAVLink/GCS_Dummy.h> // MAVLink地面站通信
#include <stdio.h>

// 获取HAL实例
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// 定义测试日志消息ID
#define LOG_TEST_MSG 1

// 定义测试日志数据结构
struct PACKED log_Test {
    LOG_PACKET_HEADER;  // 日志包头
    uint16_t v1, v2, v3, v4;  // 4个16位无符号整数
    int32_t  l1, l2;         // 2个32位有符号整数
};

// 定义日志结构数组
static const struct LogStructure log_structure[] = {
    LOG_COMMON_STRUCTURES,  // 包含通用日志结构
    { LOG_TEST_MSG, sizeof(log_Test),       
      "TEST",              // 消息名称
      "HHHHii",           // 格式字符串(H=uint16_t, i=int32_t)
      "V1,V2,V3,V4,L1,L2", // 字段名称
      "------",           // 单位(本例中未使用)
      "------"            // 数据乘数(本例中未使用)
    }
};

// 定义要写入的数据包数量
#define NUM_PACKETS 500

// 日志编号
static uint16_t log_num;

// 定义测试类
class AP_LoggerTest {
public:
    void setup();
    void loop();

private:
    // 日志记录掩码
    AP_Int32 log_bitmask;
    // 日志记录器实例
    AP_Logger logger;
    // 调度器实例
    AP_Scheduler scheduler;
};

// 创建测试类实例
static AP_LoggerTest loggertest;

// 初始化函数
void AP_LoggerTest::setup(void)
{
    hal.console->printf("Logger Log Test 1.0\n");

    // 设置日志记录掩码为全部启用
    log_bitmask.set((uint32_t)-1);
    // 初始化日志记录器
    logger.init(log_bitmask, log_structure, ARRAY_SIZE(log_structure));
    logger.set_vehicle_armed(true);
    logger.Write_Message("AP_Logger Test");

#ifdef DEBUG_RATES
    // 打印日志结构的调试信息
    hal.console->printf("| Type | Size | 10Hz(bs) | 25Hz(bs) | 400Hz(Kbs) |\n");
    for (uint16_t i = 0; i < ARRAY_SIZE(log_structure); i++) {
        LogStructure log = log_structure[i];
        hal.console->printf("| %-6s | %3d | %4d | %4d | %2dk |\n", log.name, log.msg_len,
            log.msg_len * 10, log.msg_len * 25, log.msg_len * 400 / 1000);
    }
#endif
    // 延时20ms
    hal.scheduler->delay(20);

    // 查找最后一个日志文件编号
    log_num = logger.find_last_log();
    hal.console->printf("Using log number %u\n", log_num);
    hal.console->printf("Writing to flash... wait...\n");

    // 记录写入时间统计
    uint32_t total_micros = 0;
    uint16_t i;

    // 循环写入测试数据包
    for (i = 0; i < NUM_PACKETS; i++) {
        uint32_t start = AP_HAL::micros();
        // 创建测试数据包        
        struct log_Test pkt = {
            LOG_PACKET_HEADER_INIT(LOG_TEST_MSG),
            v1    : (uint16_t)(2000 + i),
            v2    : (uint16_t)(2001 + i),
            v3    : (uint16_t)(2002 + i),
            v4    : (uint16_t)(2003 + i),
            l1    : (int32_t)(i * 5000),
            l2    : (int32_t)(i * 16268)
        };
        // 写入数据块
        logger.WriteBlock(&pkt, sizeof(pkt));
        total_micros += AP_HAL::micros() - start;
        hal.scheduler->delay(20);
    }

    // 打印平均写入时间
    hal.console->printf("Average write time %.1f usec/byte\n", 
                       (double)total_micros/((double)i*sizeof(struct log_Test)));

    // 获取当前时间
    uint64_t now = AP_HAL::micros64();
    
    // 测试Write函数
    hal.console->printf("Testing Write\n");
    logger.Write("MARY",
                 "TimeUS,GoodValue",  // 字段名
                 "sm",                // 单位
                 "F0",                // 乘数
                 "Qf",                // 格式(Q=uint64_t, f=float)
                 now,
                 -1.5673);
                 
    // 测试WriteCritical函数
    hal.console->printf("Testing WriteCritical\n");
    logger.WriteCritical("BOB",
                         "TimeUS,GreatValue",
                         "sm",
                         "F0",
                         "Qf",
                         now,
                         17.3);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    // 在SITL和Linux平台上刷新日志
    logger.flush();
#endif

    // 设置飞行器为未解锁状态
    logger.set_vehicle_armed(false);
}

// 主循环函数
void AP_LoggerTest::loop(void)
{
    hal.console->printf("\nTest complete.\n");
    hal.scheduler->delay(20000);
}

/*
  兼容旧式pde构建方式
 */
void setup(void);
void loop(void);

void setup()
{
    loggertest.setup();
}

void loop()
{
    loggertest.loop();
}

// MAVLink参数组定义
const struct AP_Param::GroupInfo        GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};
// 创建地面站通信实例
GCS_Dummy _gcs;

// 主程序入口点
AP_HAL_MAIN();
