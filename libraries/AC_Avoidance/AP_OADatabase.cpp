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

// 包含配置头文件
#include "AC_Avoidance_config.h"

#if AP_OADATABASE_ENABLED

#include "AP_OADatabase.h"

// 包含所需的头文件
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL& hal;

// 设置默认的超时时间(秒)
#ifndef AP_OADATABASE_TIMEOUT_SECONDS_DEFAULT
    #define AP_OADATABASE_TIMEOUT_SECONDS_DEFAULT   10
#endif

// 设置默认的数据库大小
#ifndef AP_OADATABASE_SIZE_DEFAULT
    #define AP_OADATABASE_SIZE_DEFAULT          100
#endif

// 设置默认的队列大小
#ifndef AP_OADATABASE_QUEUE_SIZE_DEFAULT
    #define AP_OADATABASE_QUEUE_SIZE_DEFAULT 80
#endif

// 设置距离家的距离阈值(米)
#ifndef AP_OADATABASE_DISTANCE_FROM_HOME
    #define AP_OADATABASE_DISTANCE_FROM_HOME 3
#endif

// 参数定义
const AP_Param::GroupInfo AP_OADatabase::var_info[] = {

    // @Param: SIZE
    // @DisplayName: OADatabase maximum number of points
    // @Description: OADatabase maximum number of points. Set to 0 to disable the OA Database. Larger means more points but is more cpu intensive to process
    // @Range: 0 10000
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("SIZE", 1, AP_OADatabase, _database_size_param, AP_OADATABASE_SIZE_DEFAULT),

    // @Param: EXPIRE
    // @DisplayName: OADatabase item timeout
    // @Description: OADatabase item timeout. The time an item will linger without any updates before it expires. Zero means never expires which is useful for a sent-once static environment but terrible for dynamic ones.
    // @Units: s
    // @Range: 0 127
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXPIRE", 2, AP_OADatabase, _database_expiry_seconds, AP_OADATABASE_TIMEOUT_SECONDS_DEFAULT),

    // @Param: QUEUE_SIZE
    // @DisplayName: OADatabase queue maximum number of points
    // @Description: OADatabase queue maximum number of points. This in an input buffer size. Larger means it can handle larger bursts of incoming data points to filter into the database. No impact on cpu, only RAM. Recommend larger for faster datalinks or for sensors that generate a lot of data.
    // @Range: 1 200
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("QUEUE_SIZE", 3, AP_OADatabase, _queue_size_param, AP_OADATABASE_QUEUE_SIZE_DEFAULT),

    // @Param: OUTPUT
    // @DisplayName: OADatabase output level
    // @Description: OADatabase output level to configure which database objects are sent to the ground station. All data is always available internally for avoidance algorithms.
    // @Values: 0:Disabled,1:Send only HIGH importance items,2:Send HIGH and NORMAL importance items,3:Send all items
    // @User: Advanced
    AP_GROUPINFO("OUTPUT", 4, AP_OADatabase, _output_level, (float)OutputLevel::HIGH),

    // @Param: BEAM_WIDTH
    // @DisplayName: OADatabase beam width
    // @Description: Beam width of incoming lidar data
    // @Units: deg
    // @Range: 1 10
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("BEAM_WIDTH", 5, AP_OADatabase, _beam_width, 5.0f),

    // @Param: RADIUS_MIN
    // @DisplayName: OADatabase Minimum  radius
    // @Description: Minimum radius of objects held in database
    // @Units: m
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("RADIUS_MIN", 6, AP_OADatabase, _radius_min, 0.01f),

    // @Param: DIST_MAX
    // @DisplayName: OADatabase Distance Maximum
    // @Description: Maximum distance of objects held in database.  Set to zero to disable the limits
    // @Units: m
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("DIST_MAX", 7, AP_OADatabase, _dist_max, 0.0f),

    // @Param{Copter}: ALT_MIN
    // @DisplayName: OADatabase minimum altitude above home before storing obstacles
    // @Description: OADatabase will reject obstacles if vehicle's altitude above home is below this parameter, in a 3 meter radius around home. Set 0 to disable this feature.
    // @Units: m
    // @Range: 0 4
    // @User: Advanced
    AP_GROUPINFO_FRAME("ALT_MIN", 8, AP_OADatabase, _min_alt, 0.0f, AP_PARAM_FRAME_COPTER | AP_PARAM_FRAME_HELI | AP_PARAM_FRAME_TRICOPTER),

    AP_GROUPEND
};

// 构造函数
AP_OADatabase::AP_OADatabase()
{
    // 确保单例模式
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_OADatabase must be singleton");
    }
    _singleton = this;

    // 设置参数默认值
    AP_Param::setup_object_defaults(this, var_info);
}

// 初始化函数
void AP_OADatabase::init()
{
    init_database();
    init_queue();

    // 使用至少1度的光束宽度初始化标量
    dist_to_radius_scalar = tanf(radians(MAX(_beam_width, 1.0f)));

    if (!healthy()) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "DB init failed . Sizes queue:%u, db:%u", (unsigned int)_queue.size, (unsigned int)_database.size);
        delete _queue.items;
        delete[] _database.items;
        return;
    }
}

// 更新函数
void AP_OADatabase::update()
{
    if (!healthy()) {
        return;
    }

    process_queue();
    database_items_remove_all_expired();
}

// 将位置信息推入数据库队列
void AP_OADatabase::queue_push(const Vector3f &pos, uint32_t timestamp_ms, float distance)
{
    if (!healthy()) {
        return;
    }

    // 检查此障碍物是否因为在家附近高度过低而需要被拒绝
#if APM_BUILD_COPTER_OR_HELI
    if (!is_zero(_min_alt)) { 
        Vector3f current_pos;
        if (!AP::ahrs().get_relative_position_NED_home(current_pos)) {
            // 无法获取载具位置
            return;
        }
        if (current_pos.xy().length() < AP_OADATABASE_DISTANCE_FROM_HOME) {
            // 载具在家附近的小范围内
            if (-current_pos.z < _min_alt) {
                // 载具低于最小高度
                return;
            }
        }
    }
#endif
    
    // 忽略太远的物体
    if ((_dist_max > 0.0f) && (distance > _dist_max)) {
        return;
    }

    const OA_DbItem item = {pos, timestamp_ms, MAX(_radius_min, distance * dist_to_radius_scalar), 0, AP_OADatabase::OA_DbItemImportance::Normal};
    {
        WITH_SEMAPHORE(_queue.sem);
        _queue.items->push(item);
    }
}

// 初始化队列
void AP_OADatabase::init_queue()
{
    _queue.size = _queue_size_param;
    if (_queue.size == 0) {
        return;
    }

    _queue.items = NEW_NOTHROW ObjectBuffer<OA_DbItem>(_queue.size);
    if (_queue.items != nullptr && _queue.items->get_size() == 0) {
        // 分配失败
        delete _queue.items;
        _queue.items = nullptr;
    }
}

// 初始化数据库
void AP_OADatabase::init_database()
{
    _database.size = _database_size_param;
    if (_database_size_param == 0) {
        return;
    }

    _database.items = NEW_NOTHROW OA_DbItem[_database.size];
}

// 根据重要性获取应发送到GCS的通道位掩码
uint8_t AP_OADatabase::get_send_to_gcs_flags(const OA_DbItemImportance importance)
{
    switch (importance) {
    case OA_DbItemImportance::Low:
        if (_output_level >= OutputLevel::ALL) {
            return 0xFF;
        }
        break;

    case OA_DbItemImportance::Normal:
        if (_output_level >= OutputLevel::HIGH_AND_NORMAL) {
            return 0xFF;
        }
        break;

    case OA_DbItemImportance::High:
        if (_output_level >= OutputLevel::HIGH) {
            return 0xFF;
        }
        break;
    }
    return 0x0;
}

// 处理队列中的数据
bool AP_OADatabase::process_queue()
{
    if (!healthy()) {
        return false;
    }

    // 处理队列,将条目移入数据库
    const uint16_t queue_available = MIN(_queue.items->available(), 100U);
    if (queue_available == 0) {
        return false;
    }

    for (uint16_t queue_index=0; queue_index<queue_available; queue_index++) {
        OA_DbItem item;

        bool pop_success;
        {
            WITH_SEMAPHORE(_queue.sem);
            pop_success = _queue.items->pop(item);
        }
        if (!pop_success) {
            return false;
        }

        item.send_to_gcs = get_send_to_gcs_flags(item.importance);

        // 将项目与数据库中的所有项目进行比较。如果找到类似项目,更新现有项目,否则添加为新项目
        bool found = false;
        for (uint16_t i=0; i<_database.count; i++) {
            if (is_close_to_item_in_database(i, item)) {
                database_item_refresh(i, item.timestamp_ms, item.radius);
                found = true;
                break;
            }
        }

        if (!found) {
            database_item_add(item);
        }
    }
    return (_queue.items->available() > 0);
}

// 向数据库添加项目
void AP_OADatabase::database_item_add(const OA_DbItem &item)
{
    if (_database.count >= _database.size) {
        return;
    }
    _database.items[_database.count] = item;
    _database.items[_database.count].send_to_gcs = get_send_to_gcs_flags(_database.items[_database.count].importance);
    _database.count++;
}

// 从数据库中移除项目
void AP_OADatabase::database_item_remove(const uint16_t index)
{
    if (index >= _database.count || _database.count == 0) {
        // 索引超出范围
        return;
    }

    // 半径为0表示我们不再关心它(即已过期)
    _database.items[index].radius = 0;
    _database.items[index].send_to_gcs = get_send_to_gcs_flags(_database.items[index].importance);

    _database.count--;
    if (_database.count == 0) {
        return;
    }

    if (index != _database.count) {
        // 将数组中的最后一个对象复制到过期对象上
        _database.items[index] = _database.items[_database.count];
        _database.items[index].send_to_gcs = get_send_to_gcs_flags(_database.items[index].importance);
    }
}

// 刷新数据库中的项目
void AP_OADatabase::database_item_refresh(const uint16_t index, const uint32_t timestamp_ms, const float radius)
{
    if (index >= _database.count) {
        // 索引超出范围
        return;
    }

    const bool is_different =
            (!is_equal(_database.items[index].radius, radius)) ||
            (timestamp_ms - _database.items[index].timestamp_ms >= 500);

    if (is_different) {
        // 更新接近物体的时间戳和半径,使其停留更长时间
        // 并触发重新发送到GCS
        _database.items[index].timestamp_ms = timestamp_ms;
        _database.items[index].radius = radius;
        _database.items[index].send_to_gcs = get_send_to_gcs_flags(_database.items[index].importance);
    }
}

// 移除所有过期的数据库项目
void AP_OADatabase::database_items_remove_all_expired()
{
    // 计算数据库中所有项目的年龄

    if (_database_expiry_seconds <= 0) {
        // 零意味着永不过期。这不是正常行为,但也许你想发送一次静态环境,而不需要不断更新
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t expiry_ms = (uint32_t)_database_expiry_seconds * 1000;
    uint16_t index = 0;
    while (index < _database.count) {
        if (now_ms - _database.items[index].timestamp_ms > expiry_ms) {
            database_item_remove(index);
        } else {
            index++;
        }
    }
}

// 检查数据库中是否已存在类似对象。如果存在,对象计时器也会重置
bool AP_OADatabase::is_close_to_item_in_database(const uint16_t index, const OA_DbItem &item) const
{
    if (index >= _database.count) {
        // 索引超出范围
        return false;
    }

    const float distance_sq = (_database.items[index].pos - item.pos).length_squared();
    return ((distance_sq < sq(item.radius)) || (distance_sq < sq(_database.items[index].radius)));
}

#if HAL_GCS_ENABLED
// 发送ADSB_VEHICLE mavlink消息
void AP_OADatabase::send_adsb_vehicle(mavlink_channel_t chan, uint16_t interval_ms)
{
    // 确保数据库的send_to_gcs字段足够大
    static_assert(MAVLINK_COMM_NUM_BUFFERS <= sizeof(OA_DbItem::send_to_gcs) * 8,
                  "AP_OADatabase's OA_DBItem.send_to_gcs bitmask must be large enough to hold MAVLINK_COMM_NUM_BUFFERS");

    if ((_output_level <= OutputLevel::NONE) || !healthy()) {
        return;
    }

    const uint8_t chan_as_bitmask = 1 << chan;
    const char callsign[9] = "OA_DB";

    // 计算应该发送多少消息
    const uint32_t now_ms = AP_HAL::millis();
    uint16_t num_to_send = 1;
    uint16_t num_sent = 0;
    if ((_last_send_to_gcs_ms[chan] != 0) && (interval_ms > 0)) {
        uint32_t diff_ms = now_ms - _last_send_to_gcs_ms[chan];
        num_to_send = MAX(diff_ms / interval_ms, 1U);
    }
    _last_send_to_gcs_ms[chan] = now_ms;

    // 发送未发送的对象,直到输出缓冲区已满或已发送足够数量
    for (uint16_t i=0; i < _database.count; i++) {
        if (!HAVE_PAYLOAD_SPACE(chan, ADSB_VEHICLE) || (num_sent >= num_to_send)) {
            // 暂时完成
            return;
        }

        const uint16_t idx = _next_index_to_send[chan];

        // 准备发送下一个对象
        _next_index_to_send[chan]++;
        if (_next_index_to_send[chan] >= _database.count) {
            _next_index_to_send[chan] = 0;
        }

        if ((_database.items[idx].send_to_gcs & chan_as_bitmask) == 0) {
            continue;
        }

        // 将对象位置从EKF原点的偏移转换为Location
        const Location item_loc(Vector3f(_database.items[idx].pos.x * 100.0f, _database.items[idx].pos.y * 100.0f, _database.items[idx].pos.z * 100.0f), Location::AltFrame::ABOVE_ORIGIN);

        mavlink_msg_adsb_vehicle_send(chan,
            idx,
            item_loc.lat,
            item_loc.lng,
            0,                          // altitude_type
            item_loc.alt,               
            0,                          // heading
            0,                          // hor_velocity
            0,                          // ver_velocity
            callsign,                   // callsign
            255,                        // emitter_type
            0,                          // tslc
            0,                          // flags
            (uint16_t)(_database.items[idx].radius * 100.f));   // squawk

        // 取消标记要发送到gcs的项目
        _database.items[idx].send_to_gcs &= ~chan_as_bitmask;

        // 更新发送到GCS的最高索引
        _highest_index_sent[chan] = MAX(idx, _highest_index_sent[chan]);

        // 更新发送计数
        num_sent++;
    }

    // 清除过期项目,以防数据库大小缩小
    while (_highest_index_sent[chan] > _database.count) {
        if (!HAVE_PAYLOAD_SPACE(chan, ADSB_VEHICLE) || (num_sent >= num_to_send)) {
            // 暂时完成
            return;
        }

        const uint16_t idx = _highest_index_sent[chan];
        _highest_index_sent[chan]--;

        if (_database.items[idx].importance != OA_DbItemImportance::High) {
            continue;
        }

        mavlink_msg_adsb_vehicle_send(chan,
            idx,        // id
            0,          // latitude
            0,          // longitude
            0,          // altitude_type
            0,          // altitude
            0,          // heading
            0,          // hor_velocity
            0,          // ver_velocity
            callsign,   // callsign
            255,        // emitter_type
            0,          // tslc
            0,          // flags
            0);         // squawk

        // 更新发送计数
        num_sent++;
    }
}
#endif  // HAL_GCS_ENABLED

// 单例实例
AP_OADatabase *AP_OADatabase::_singleton;

namespace AP {
AP_OADatabase *oadatabase()
{
    return AP_OADatabase::get_singleton();
}

}

#endif  // AP_OADATABASE_ENABLED
