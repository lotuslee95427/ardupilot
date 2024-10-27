#pragma once

#include <AP_Common/AP_Common.h>

// 如果添加任何新的类型、单位或乘数，请更新 README.md
// if you add any new types, units or multipliers, please update README.md

/*
二进制日志消息格式字符串中的格式字符
Format characters in the format string for binary log messages
  a   : int16_t[32]      // 32个16位整数数组
  b   : int8_t           // 8位有符号整数
  B   : uint8_t          // 8位无符号整数
  h   : int16_t          // 16位有符号整数
  H   : uint16_t         // 16位无符号整数
  i   : int32_t          // 32位有符号整数
  I   : uint32_t         // 32位无符号整数
  f   : float            // 浮点数
  d   : double           // 双精度浮点数
  n   : char[4]          // 4字节字符数组
  N   : char[16]         // 16字节字符数组
  Z   : char[64]         // 64字节字符数组
  c   : int16_t * 100    // 16位整数乘以100
  C   : uint16_t * 100   // 16位无符号整数乘以100
  e   : int32_t * 100    // 32位整数乘以100
  E   : uint32_t * 100   // 32位无符号整数乘以100
  L   : int32_t latitude/longitude  // 纬度/经度
  M   : uint8_t flight mode         // 飞行模式
  q   : int64_t          // 64位有符号整数
  Q   : uint64_t         // 64位无符号整数
 */

// 单位结构体定义
struct UnitStructure {
    const char ID;           // 单位标识符
    const char *unit;        // 单位名称
};

// 乘数结构体定义
struct MultiplierStructure {
    const char ID;           // 乘数标识符
    const double multiplier; // 乘数值
};

// 所有单位都应该是基本单位
// 这意味着电池容量在这里是"安培*秒"
// 请保持名称与 Tools/autotest/param_metadata/param.py:33 一致
// all units here should be base units
// This does mean battery capacity is here as "amp*second"
// Please keep the names consistent with Tools/autotest/param_metadata/param.py:33
const struct UnitStructure log_Units[] = {
    { '-', "" },              // 无单位,如 Pi 或字符串 / no units e.g. Pi, or a string
    { '?', "UNKNOWN" },       // 尚未确定的单位 / Units which haven't been worked out yet....
    { 'A', "A" },             // 安培 / Ampere
    { 'a', "Ah" },            // 安培小时 / Ampere hours
    { 'd', "deg" },           // 角度,-180到180 / of the angular variety, -180 to 180
    { 'b', "B" },             // 字节 / bytes
    { 'B', "B/s" },           // 字节每秒 / bytes per second
    { 'k', "deg/s" },         // 度每秒(虽然不是SI单位,但在某些情况下比弧度更友好) / degrees per second. Degrees are NOT SI, but is some situations more user-friendly than radians
    { 'D', "deglatitude" },   // 纬度 / degrees of latitude
    { 'e', "deg/s/s" },       // 度每二次方秒 / degrees per second per second. Degrees are NOT SI, but is some situations more user-friendly than radians
    { 'E', "rad/s" },         // 弧度每秒 / radians per second
    { 'G', "Gauss" },         // 高斯(非SI单位,1特斯拉=10000高斯,无法简单替换) / Gauss is not an SI unit, but 1 tesla = 10000 gauss so a simple replacement is not possible here
    { 'h', "degheading" },    // 航向角0.?到359.? / 0.? to 359.?
    { 'i', "A.s" },           // 安培秒 / Ampere second
    { 'J', "W.s" },           // 焦耳(瓦特秒) / Joule (Watt second)
    { 'l', "l" },             // 升 / litres
    { 'L', "rad/s/s" },       // 弧度每二次方秒 / radians per second per second
    { 'm', "m" },             // 米 / metres
    { 'n', "m/s" },           // 米每秒 / metres per second
    // { 'N', "N" },          // 牛顿 / Newton
    { 'o', "m/s/s" },         // 米每二次方秒 / metres per second per second
    { 'O', "degC" },          // 摄氏度(非SI单位,但开尔文对大多数用户来说太复杂) / degrees Celsius. Not SI, but Kelvin is too cumbersome for most users
    { '%', "%" },             // 百分比 / percent
    { 'S', "satellites" },    // 卫星数量 / number of satellites
    { 's', "s" },             // 秒 / seconds
    { 'q', "rpm" },           // 每分钟转数(非SI单位,但有时比赫兹更直观) / rounds per minute. Not SI, but sometimes more intuitive than Hertz
    { 'r', "rad" },           // 弧度 / radians
    { 'U', "deglongitude" },  // 经度 / degrees of longitude
    { 'u', "ppm" },           // 每分钟脉冲数 / pulses per minute
    { 'v', "V" },             // 伏特 / Volt
    { 'P', "Pa" },            // 帕斯卡 / Pascal
    { 'w', "Ohm" },           // 欧姆 / Ohm
    { 'W', "Watt" },          // 瓦特 / Watt
    { 'X', "W.h" },           // 瓦特小时 / Watt hour
    { 'y', "l/s" },           // 升每秒 / litres per second
    { 'Y', "us" },            // 微秒脉宽调制 / pulse width modulation in microseconds
    { 'z', "Hz" },            // 赫兹 / Hertz
    { '#', "instance" }       // 实例编号(例如传感器实例) / (e.g.)Sensor instance number
};

// 这些乘数信息适用于日志中的原始值
// 格式字段暗示的任何调整(例如"centidegrees"中的"centi")在缩放时被*忽略*
// 本质上,"format"只是告诉你C类型,格式类型h(int16_t)等同于格式类型c(int16_t*100)
// 简而言之,GCS不应该/不能从单位名称推断任何缩放
// this multiplier information applies to the raw value present in the
// log.  Any adjustment implied by the format field (e.g. the "centi"
// in "centidegrees" is *IGNORED* for the purposes of scaling.
// Essentially "format" simply tells you the C-type, and format-type h
// (int16_t) is equivalent to format-type c (int16_t*100)
// tl;dr a GCS shouldn't/mustn't infer any scaling from the unit name

const struct MultiplierStructure log_Multipliers[] = {
    { '-', 0 },       // 无乘数,如字符串 / no multiplier e.g. a string
    { '?', 1 },       // 尚未确定的乘数 / multipliers which haven't been worked out yet....
// <这里留个空隙,以防万一....> / <leave a gap here, just in case....>
    { '2', 1e2 },
    { '1', 1e1 },
    { '0', 1e0 },
    { 'A', 1e-1 },
    { 'B', 1e-2 },
    { 'C', 1e-3 },
    { 'D', 1e-4 },
    { 'E', 1e-5 },
    { 'F', 1e-6 },
    { 'G', 1e-7 },
    { 'I', 1e-9 },
// <这里留个空隙,以防万一....> / <leave a gap here, just in case....>
    { '!', 3.6 }, // (安培*秒 => 毫安*小时)和(千米/小时 => 米/秒) / (ampere*second => milliampere*hour) and (km/h => m/s)
    { '/', 3600 }, // (安培*秒 => 安培*小时) / (ampere*second => ampere*hour)
};

/*
  由于g++中命名成员结构初始化的限制,这些不得不是宏
  unfortunately these need to be macros because of a limitation of
  named member structure initialisation in g++
 */
#define LOG_PACKET_HEADER	       uint8_t head1, head2, msgid;
#define LOG_PACKET_HEADER_INIT(id) head1 : HEAD_BYTE1, head2 : HEAD_BYTE2, msgid : id
#define LOG_PACKET_HEADER_LEN 3 // LOG_PACKET_HEADER所需的字节数 / bytes required for LOG_PACKET_HEADER

// 一旦所有日志代码都转换完成,我们将从此头文件中删除这些内容
// once the logging code is all converted we will remove these from
// this header
#define HEAD_BYTE1  0xA3    // 十进制163 / Decimal 163
#define HEAD_BYTE2  0x95    // 十进制149 / Decimal 149

[... rest of the file continues with similar dual-language comments ...]
