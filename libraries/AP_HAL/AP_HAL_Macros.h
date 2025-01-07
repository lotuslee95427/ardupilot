#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

/*
  macros to allow code to build on multiple platforms more easily
  宏定义,使代码更容易在多个平台上构建
 */

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX || HAL_WITH_EKF_DOUBLE || AP_SIM_ENABLED
/*
  allow double maths on Linux and SITL to avoid problems with system headers
  在Linux和SITL上允许双精度数学运算,以避免系统头文件的问题
 */
  #if !defined(ALLOW_DOUBLE_MATH_FUNCTIONS)
    #define ALLOW_DOUBLE_MATH_FUNCTIONS
  #endif
#endif

// we need to include math.h here for newer compilers (eg. g++ 7.3.1 for stm32)
// 这里需要包含math.h用于较新的编译器(例如stm32的g++ 7.3.1)
#include <math.h>

#if !defined(ALLOW_DOUBLE_MATH_FUNCTIONS)
/* give warnings if we use double precision maths functions without
   specifying ALLOW_DOUBLE_TRIG_FUNCTIONS. Code should use the
   equivalent f function instead (eg. use cosf() instead of
   cos()). Individual cpp files that really do need double precision
   should define ALLOW_DOUBLE_TRIG_FUNCTIONS before including
   AP_Math.h

   如果在未指定ALLOW_DOUBLE_TRIG_FUNCTIONS的情况下使用双精度数学函数,则给出警告。
   代码应该使用等效的f函数(例如使用cosf()而不是cos())。
   确实需要双精度的单个cpp文件应该在包含AP_Math.h之前定义ALLOW_DOUBLE_TRIG_FUNCTIONS
*/
#define sin(x) DO_NOT_USE_DOUBLE_MATHS()    // 禁用sin双精度函数
#define cos(x) DO_NOT_USE_DOUBLE_MATHS()    // 禁用cos双精度函数  
#define tan(x) DO_NOT_USE_DOUBLE_MATHS()    // 禁用tan双精度函数
#define acos(x) DO_NOT_USE_DOUBLE_MATHS()   // 禁用acos双精度函数
#define asin(x) DO_NOT_USE_DOUBLE_MATHS()   // 禁用asin双精度函数
#define atan(x) DO_NOT_USE_DOUBLE_MATHS()   // 禁用atan双精度函数
#define atan2(x,y) DO_NOT_USE_DOUBLE_MATHS() // 禁用atan2双精度函数
#define exp(x) DO_NOT_USE_DOUBLE_MATHS()    // 禁用exp双精度函数
#define pow(x,y) DO_NOT_USE_DOUBLE_MATHS()  // 禁用pow双精度函数
#define sqrt(x) DO_NOT_USE_DOUBLE_MATHS()   // 禁用sqrt双精度函数
#define log2(x) DO_NOT_USE_DOUBLE_MATHS()   // 禁用log2双精度函数
#define log10(x) DO_NOT_USE_DOUBLE_MATHS()  // 禁用log10双精度函数
#define ceil(x) DO_NOT_USE_DOUBLE_MATHS()   // 禁用ceil双精度函数
#define floor(x) DO_NOT_USE_DOUBLE_MATHS()  // 禁用floor双精度函数
#define round(x) DO_NOT_USE_DOUBLE_MATHS()  // 禁用round双精度函数
#define fmax(x,y) DO_NOT_USE_DOUBLE_MATHS() // 禁用fmax双精度函数
#if !HAL_NUM_CAN_IFACES
// we should do log() and fabs() as well, but can't because of a conflict in uavcan
// 我们也应该禁用log()和fabs(),但由于uavcan中的冲突而不能这样做
#define log(x) DO_NOT_USE_DOUBLE_MATHS()    // 禁用log双精度函数
#define fabs(x) DO_NOT_USE_DOUBLE_MATHS()   // 禁用fabs双精度函数
#endif
#endif
