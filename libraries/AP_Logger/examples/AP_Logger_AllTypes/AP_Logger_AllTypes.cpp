/*
 * 写入两个日志,每个日志包含每种属性类型的样本
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <stdio.h>

// 获取HAL实例
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// 二进制日志消息格式字符串中的格式字符
// 定义第一种日志结构体,包含各种基本数据类型
struct PACKED log_TYP1 {
    LOG_PACKET_HEADER;  // 日志包头
    uint64_t time_us;   // 时间戳(微秒)
    int16_t    a[32];   // 16位有符号整型数组
    int8_t     b;       // 8位有符号整型
    uint8_t    B;       // 8位无符号整型
    int16_t    h;       // 16位有符号整型
    uint16_t   H;       // 16位无符号整型
    int32_t    i;       // 32位有符号整型
    uint32_t   I;       // 32位无符号整型
    float      f;       // 单精度浮点数
    double     d;       // 双精度浮点数
    char       n[4];    // 4字节字符数组
    char       N[16];   // 16字节字符数组
    char       Z[64];   // 64字节字符数组
};
// 确保log_TYP1结构体大小小于256字节
static_assert(sizeof(log_TYP1) < 256, "log_TYP1 is oversize");

// 定义第二种日志结构体,包含更多数据类型
struct PACKED log_TYP2 {
    LOG_PACKET_HEADER;  // 日志包头
    uint64_t time_us;   // 时间戳(微秒)
    int16_t   c;        // int16_t * 100
    uint16_t  C;        // uint16_t * 100
    int32_t   e;        // int32_t * 100
    uint32_t  E;        // uint32_t * 100
    int32_t   L;        // 纬度/经度
    uint8_t   M;        // 飞行模式
    int64_t   q;        // 64位有符号整型
    uint64_t  Q;        // 64位无符号整型
    Float16_t g;        // 16位浮点数
};

// 定义日志消息类型枚举
enum MyLogMessages {
    LOG_TYP1_MSG,
    LOG_TYP2_MSG,
};

// 定义日志结构数组,包含所有日志格式定义
static const struct LogStructure log_structure[] = {
    // 格式消息定义
    { LOG_FORMAT_MSG,
      sizeof(log_Format),
      "FMT",
      "BBnNZ",
      "Type,Length,Name,Format,Columns",
      "-b---",
      "-----" },
    // 单位消息定义  
    { LOG_UNIT_MSG, sizeof(log_Unit),
      "UNIT", "QbZ",      "TimeUS,Id,Label", "s--","F--" },
    // 格式单位消息定义
    { LOG_FORMAT_UNITS_MSG, sizeof(log_Format_Units),
      "FMTU", "QBNN",      "TimeUS,FmtType,UnitIds,MultIds","s---", "F---" },
    // 乘数消息定义
    { LOG_MULT_MSG, sizeof(log_Format_Multiplier),
      "MULT", "Qbd",      "TimeUS,Id,Mult", "s--","F--" },

    // TYP1消息定义
    { LOG_TYP1_MSG,
      sizeof(log_TYP1),
      "TYP1",
      "QabBhHiIfdnNZ",
      "TimeUS,a,b,B,h,H,i,I,f,d,n,N,Z",
      "s------------",
      "F------------"
    },
    // TYP2消息定义
    { LOG_TYP2_MSG,
      sizeof(log_TYP2),
      "TYP2",
      "QcCeELMqQg",
      "TimeUS,c,C,e,E,L,M,q,Q,g",
      "s---------",
      "F---------"
    },
    // 文本消息定义
    { LOG_MESSAGE_MSG,
      sizeof(log_Message),
      "MSG",
      "QZ",
      "TimeUS,Message",
      "s-",
      "F-"}
};

// 这些与上面日志结构中的条目相同。保持独立以维持上面结构与LogStructure.h中结构的视觉相似性
#define TYP1_FMT "QabBhHiIfdnNZ"
#define TYP1_LBL "TimeUS,b,B,h,H,i,I,f,d,n,N,Z"
#define TYP2_FMT "QcCeELMqQg"
#define TYP2_LBL "TimeUS,c,C,e,E,L,M,q,Q,g"

// 日志编号
static uint16_t log_num;

// 定义测试类
class AP_LoggerTest_AllTypes : public AP_HAL::HAL::Callbacks {
public:
    void setup() override;
    void loop() override;

private:
    // 日志位掩码
    AP_Int32 log_bitmask;
    // 日志记录器实例
    AP_Logger logger;
    // 调度器实例
    AP_Scheduler scheduler;

    // 写入类型消息
    void Log_Write_TypeMessages();
    void Log_Write_TypeMessages_Log_Write();

    // 刷新日志记录器
    void flush_logger(AP_Logger &logger);
};

// 刷新日志记录器函数实现
void AP_LoggerTest_AllTypes::flush_logger(AP_Logger &_logger)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    _logger.flush();
#else
    // 在stm32等平台上flush不可用,因为这可能是一个危险的操作
    // 但如果等待足够长时间(写入时为2秒,见AP_Logger_File::_io_timer)
    // 数据会被写出
    hal.scheduler->delay(3000);
#endif
}

// 写入类型消息函数实现
void AP_LoggerTest_AllTypes::Log_Write_TypeMessages()
{
    // 查找最后一个日志编号
    log_num = logger.find_last_log();
    hal.console->printf("Using log number %u\n", log_num);

    hal.console->printf("Writing out a few messages to get formats out...");
    logger.Write_Message("Start 1");

    // 创建并写入TYP1类型消息
    struct log_TYP1 typ1{
        LOG_PACKET_HEADER_INIT(LOG_TYP1_MSG),
        time_us : AP_HAL::micros64(),
        a : { -32768, 32767, 1, -1, 0, 19 }, // int16[32]
        b : -17, // int8_t
        B : 42,  // uint8_t
        h : -12372,  // int16_t
        H : 19812,   // uint16_t
        i : -98234729,   // int32_t
        I : 74627293,    // uint32_t
        f : 35.87654,  // float
        d : 67.7393274658293,   // double
        n : { 'A', 'B', 'C', 'D' }, // char[4]
        // char[16]:
        N : { 'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P' },
        // char[64]:
        Z : { 'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P',
              'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P',
              'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P',
              'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P' }
    };
    // 写入TYP1消息
    if (!logger.WriteBlock_first_succeed(&typ1, sizeof(typ1))) {
        abort();
    }

    // 创建16位浮点数
    Float16_t f16;
    f16.set(13.54f);
    // 创建并写入TYP2类型消息
    struct log_TYP2 typ2 = {
        LOG_PACKET_HEADER_INIT(LOG_TYP2_MSG),
        time_us : AP_HAL::micros64(),
        c : -9823, // int16_t * 100
        C : 5436,  // uint16_t * 100
        e : -9209238,  // int32_t * 100
        E : 19239872,  // uint32_t * 100
        L : -3576543,   // uint32_t latitude/longitude;
        M : 5,          //   uint8_t;   // flight mode;
        q : -98239832498328,   // int64_t
        Q : 3432345232233432,  // uint64_t
        g : f16              // float16_t
    };
    logger.WriteBlock(&typ2, sizeof(typ2));

    // 刷新日志
    flush_logger(logger);

    // 停止日志记录
    logger.StopLogging();
}

// 使用Log_Write方式写入类型消息
void AP_LoggerTest_AllTypes::Log_Write_TypeMessages_Log_Write()
{
    // 查找最后一个日志编号
    log_num = logger.find_last_log();
    hal.console->printf("Using log number for Log_Write %u\n", log_num);

    hal.console->printf("Writing out a few messages to get formats out...");
    logger.Write_Message("Start 2");

    // 写入字符串消息
    logger.Write("TYPn",
                 "TimeUS,Str",
                 "Qn",
                 AP_HAL::micros64(),
                 "ABCD");

    // 写入数组消息
    const int16_t a[32] = { -32768, 32767, 1, -1, 0, 17 };

    logger.Write("TYPa",
                 "TimeUS,Arr",
                 "Qa",
                 AP_HAL::micros64(),
                 a);

    // 写入TYP3消息(与TYP1格式相同)
    logger.Write("TYP3",
                 TYP1_LBL,
                 TYP1_FMT,
                 AP_HAL::micros64(),
                 a, // int16[32]
                 -17, // int8_t
                 42,  // uint8_t
                 -12372,  // int16_t
                 19812,   // uint16_t
                 -98234729,   // int32_t
                 74627293,    // uint32_t
                 35.87654f,  // float
                 (double)67.7393274658293,   // double
                 "ABCD", // char[4]
                 // char[16]:
                 "ABCDEFGHIJKLMNOP",
                 // char[64]:
                 "ABCDEFGHIJKLMNOPABCDEFGHIJKLMNOPABCDEFGHIJKLMNOPABCDEFGHIJKLMNOP"
        );

    // 写入TYP4消息(与TYP2格式相同)
    logger.Write("TYP4",
                 TYP2_LBL,
                 TYP2_FMT,
                 AP_HAL::micros64(),
                 -9823, // int16_t * 100
                 5436,  // uint16_t * 100
                 -9209238,  // int32_t * 100
                 19239872,  // uint32_t * 100
                 -3576543,   // uint32_t latitude/longitude;
                 5,          //   uint8_t;   // flight mode;
                 -98239832498328,   // int64_t
                 3432345232233432,   // uint64_t
                 13.54               // float16_t
        );

    // 写入包含NaN值的消息
    logger.Write("NANS", "f,d,bf,bd", "fdfd",  logger.quiet_nanf(), logger.quiet_nan(), NAN, NAN);

    // 刷新日志
    flush_logger(logger);

    // 停止日志记录
    logger.StopLogging();
}

// 设置函数
void AP_LoggerTest_AllTypes::setup(void)
{
    hal.console->printf("Logger All Types 1.0\n");

    // 设置日志位掩码为全1
    log_bitmask.set((uint32_t)-1);
    // 初始化日志记录器
    logger.init(log_bitmask, log_structure, ARRAY_SIZE(log_structure));
    // 设置载具为已解锁状态
    logger.set_vehicle_armed(true);
    logger.Write_Message("AP_Logger Test");

    // 测试延时
    hal.scheduler->delay(20);

    // 执行测试
    Log_Write_TypeMessages();
    Log_Write_TypeMessages_Log_Write();

    hal.console->printf("tests done\n");
}

// 循环函数
void AP_LoggerTest_AllTypes::loop(void)
{
    hal.console->printf("all done\n");
    hal.scheduler->delay(1000);
}

// GCS参数组定义
const struct AP_Param::GroupInfo        GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};
// GCS实例
GCS_Dummy _gcs;

// 创建测试实例
static AP_LoggerTest_AllTypes loggertest;

// 主回调函数
AP_HAL_MAIN_CALLBACKS(&loggertest);
