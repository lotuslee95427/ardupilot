/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

// 包含HAL头文件
#include "HAL.h"

// 如果未定义AP_MAIN,则将其定义为main
#ifndef AP_MAIN
#define AP_MAIN main
#endif

// 定义AP_HAL_MAIN宏,用于创建主程序入口点
// 创建一个包含setup和loop回调函数的FunCallbacks对象
// 定义C语言风格的main函数
// 调用hal.run运行程序,传入命令行参数和回调函数
#define AP_HAL_MAIN() \
    AP_HAL::HAL::FunCallbacks callbacks(setup, loop); \
    extern "C" {                               \
    int AP_MAIN(int argc, char* const argv[]); \
    int AP_MAIN(int argc, char* const argv[]) { \
        hal.run(argc, argv, &callbacks); \
        return 0; \
    } \
    }

// 定义AP_HAL_MAIN_CALLBACKS宏,允许传入自定义的回调函数
// 定义C语言风格的main函数
// 调用hal.run运行程序,传入命令行参数和自定义回调函数
#define AP_HAL_MAIN_CALLBACKS(CALLBACKS) extern "C" { \
    int AP_MAIN(int argc, char* const argv[]); \
    int AP_MAIN(int argc, char* const argv[]) { \
        hal.run(argc, argv, CALLBACKS); \
        return 0; \
    } \
    }
