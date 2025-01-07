/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_Common/AP_Common.h>

// Returns the max number of bytes that can be sent since the last call given byte/s rate limit
// 根据给定的字节/秒速率限制返回自上次调用以来可以发送的最大字节数
class DataRateLimit {
public:
    // 计算在给定速率限制下可发送的最大字节数
    uint32_t max_bytes(const float bytes_per_sec);
private:
    uint32_t last_us;      // 上次调用的时间戳(微秒)
    float remainder;       // 剩余的小数部分字节数
};
