# Logger Notes
# 日志记录说明

## Format Types
## 格式类型

The format type specifies the amount of storage required for the entry
and how the content should be interpreted.
格式类型指定了每个条目所需的存储空间以及如何解释内容。

| Char | C Type |
|------|--------|
|a   | int16_t[32]|
|b   | int8_t|
|B   | uint8_t|
|h   | int16_t|
|H   | uint16_t|
|i   | int32_t|
|I   | uint32_t|
|f   | float|
|d   | double|
|n   | char[4]|
|N   | char[16]|
|Z   | char[64]|
|L   | int32_t latitude/longitude (so -35.1332423 becomes -351332423)|
|M   | uint8_t flight mode|
|q   | int64_t|
|Q   | uint64_t|
|g   | float16_t|

Legacy field types - do not use.  These have been replaced by using  the base C type and an appropriate multiplier column entry.
遗留字段类型 - 请勿使用。这些已被基本C类型和适当的乘数列条目所替代。

| Char | CType+Mult   |
|------|--------------|
|  c   | int16_t * 100|
|  C   | uint16_t * 100|
|  e   | int32_t * 100|
|  E   | uint32_t * 100|

## Units
## 单位

All units here should be base units. 
This means battery capacity uses "amp \* second" not "milliAmp \* hours". 
Please keep the names consistent with Tools/autotest/param_metadata/param.py:33
这里的所有单位都应该是基本单位。
这意味着电池容量使用"安培 \* 秒"而不是"毫安 \* 小时"。
请保持名称与Tools/autotest/param_metadata/param.py:33一致。

| Char | Unit Abbrev. | Description | Notes |
|-----|---|---|---|
| '-' | "" | no units e.g. Pi or a string | 无单位，如Pi或字符串 |
| '?' | "UNKNOWN" | Units which haven't been worked out yet.... | 尚未确定的单位.... |
| 'A' | "A" | Ampere | 安培 |
| 'd' | "deg" | of the angular variety | -180 to 180 | 角度，范围-180到180 |
| 'b' | "B" | bytes | 字节 |
| 'B' | "B/s" | bytes per second | 字节每秒 |
| 'k' | "deg/s" | degrees per second | Not an SI unit, but in some situations more user-friendly than radians per second | 度每秒，非SI单位，但在某些情况下比弧度每秒更友好 |
| 'D' | "deglatitude" | degrees of latitude | 纬度 |
| 'e' | "deg/s/s" | degrees per second per second | Not an SI unit, but in some situations more user-friendly than radians per second^2 | 度每二次方秒，非SI单位，但有时比弧度每二次方秒更友好 |
| 'E' | "rad/s" | radians per second | 弧度每秒 |
| 'G' | "Gauss" | Gauss | Not an SI unit, but 1 tesla = 10000 gauss so a simple replacement is not possible here | 高斯，非SI单位，1特斯拉=10000高斯，所以无法简单替换 |
| 'h' | "degheading" | 0.? to 359.? | 航向角，0到359度 |
| 'i' | "A.s" | Ampere second | 安培秒 |
| 'J' | "W.s" | Joule (Watt second) | 焦耳(瓦特秒) |
| 'l' | "l" | litres | 升 |
| 'L' | "rad/s/s" | radians per second per second | 弧度每二次方秒 |
| 'm' | "m" | metres | 米 |
| 'n' | "m/s" | metres per second | 米每秒 |
| 'N' | "N" | Newton | 牛顿 |
| 'o' | "m/s/s" | metres per second per second | 米每二次方秒 |
| 'O' | "degC" | degrees Celsius | Not an SI unit, but Kelvin is too cumbersome for most users | 摄氏度，非SI单位，但开尔文对大多数用户来说太繁琐 |
| '%' | "%" | percent | 百分比 |
| 'S' | "satellites" | number of satellites | 卫星数量 |
| 's' | "s" | seconds | 秒 |
| 'q' | "rpm" | revolutions per minute | Not an SI unit, but sometimes more intuitive than Hertz | 每分钟转数，非SI单位，但有时比赫兹更直观 |
| 'r' | "rad" | radians | 弧度 |
| 'U' | "deglongitude" | degrees of longitude | 经度 |
| 'u' | "ppm" | pulses per minute | 每分钟脉冲数 |
| 'v' | "V" | Volt | 伏特 |
| 'P' | "Pa" | Pascal | 帕斯卡 |
| 'w' | "Ohm" | Ohm | 欧姆 |
| 'W' | "W" | watt | 瓦特 |
| 'X' | "W.h" | watt hour | 瓦特小时 |
| 'Y' | "us" | pulse width modulation in microseconds | 微秒脉宽调制 |
| 'z' | "Hz" | Hertz | 赫兹 |
| '#' | "instance" | (e.g.)Sensor instance number | 传感器实例编号 |

## Multipliers
## 乘数

This multiplier information applies to the raw value present in the
log. Any adjustment implied by the format field (e.g. the "centi"
in "centidegrees" is *IGNORED* for the purposes of scaling.
Essentially "format" simply tells you the C-type, and format-type h
(int16_t) is equivalent to format-type c (int16_t*100)
tl;dr a GCS shouldn't/mustn't infer any scaling from the unit name
此乘数信息适用于日志中的原始值。格式字段暗示的任何调整（例如"centidegrees"中的"centi"）
在缩放时将被*忽略*。本质上，"format"仅告诉你C类型，格式类型h(int16_t)等同于
格式类型c(int16_t*100)。简而言之，地面站不应该/不能从单位名称推断任何缩放。

| Char | Multiplier | Description |
|------|------------|---|
| '-' | 0 | no multiplier e.g. char[4] | 无乘数，如char[4] |
| '?' | 1 | multipliers which haven't been worked out yet | 尚未确定的乘数 |
| '2' | 1e2 | | 100倍 |
| '1' | 1e1 | | 10倍 |
| '0' | 1e0 | x1 | 1倍 |
| 'A' | 1e-1 | | 0.1倍 |
| 'B' | 1e-2 | | 0.01倍 |
| 'C' | 1e-3 | | 0.001倍 |
| 'D' | 1e-4 | | 0.0001倍 |
| 'E' | 1e-5 | | 0.00001倍 |
| 'F' | 1e-6 | | 0.000001倍 |
| 'G' | 1e-7 | | 0.0000001倍 |
| 'I' | 1e-9 | | 0.000000001倍 |
| '!' | 3.6 | (milliampere \* hour => ampere \* second) and (km/h => m/s) | 毫安时转安培秒和千米每小时转米每秒 |
| '/' | 3600 | (ampere \* hour => ampere \* second) | 安培小时转安培秒 |
