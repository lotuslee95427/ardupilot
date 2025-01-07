#pragma once

#include "AC_Avoidance_config.h"

#if AP_OADATABASE_ENABLED

#include <AP_HAL/Semaphores.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Param/AP_Param.h>

// 障碍物数据库类,用于存储和管理障碍物信息
class AP_OADatabase {
public:

    AP_OADatabase();

    CLASS_NO_COPY(AP_OADatabase); /* 不允许复制 */

    // 获取单例实例
    static AP_OADatabase *get_singleton() {
        return _singleton;
    }

    // 障碍物重要性枚举
    enum OA_DbItemImportance {
        Low,        // 低重要性
        Normal,     // 普通重要性
        High,       // 高重要性
    };

    // 障碍物数据库项结构体
    struct OA_DbItem {
        Vector3f pos;           // 障碍物相对EKF原点的位置偏移(米)
        uint32_t timestamp_ms;  // 障碍物最后更新的系统时间
        float radius;           // 障碍物半径(米)
        uint8_t send_to_gcs;    // 需要发送此障碍物详情的mavlink通道位掩码
        OA_DbItemImportance importance;  // 障碍物重要性
    };

    void init();   // 初始化
    void update(); // 更新

    // 将障碍物推入数据库。pos是相对EKF原点的偏移(米),angle是角度(度),distance是距离(米)
    void queue_push(const Vector3f &pos, uint32_t timestamp_ms, float distance);

    // 返回数据库是否健康
    bool healthy() const { return (_queue.items != nullptr) && (_database.items != nullptr); }

    // 获取数据库中的项。当i >= _database.count时结果未定义
    const OA_DbItem& get_item(uint32_t i) const { return _database.items[i]; }

    // 获取数据库中的项目数量
    uint16_t database_count() const { return _database.count; }

    // 清空队列并尝试放入数据库。如果还有更多工作要做则返回true
    bool process_queue();

    // 发送ADSB_VEHICLE mavlink消息
    void send_adsb_vehicle(mavlink_channel_t chan, uint16_t interval_ms);

    static const struct AP_Param::GroupInfo var_info[];

private:

    // 初始化函数
    void init_queue();      // 初始化队列
    void init_database();   // 初始化数据库

    // 数据库项管理
    void database_item_add(const OA_DbItem &item);     // 添加项
    void database_item_refresh(const uint16_t index, const uint32_t timestamp_ms, const float radius);  // 刷新项
    void database_item_remove(const uint16_t index);   // 移除项
    void database_items_remove_all_expired();          // 移除所有过期项

    // 根据重要性获取应该发送到的GCS通道位掩码
    // 如果应该发送则返回0xFF(发送到所有通道),否则返回0
    uint8_t get_send_to_gcs_flags(const OA_DbItemImportance importance);

    // 判断数据库中索引为"index"的项是否接近"item"
    bool is_close_to_item_in_database(const uint16_t index, const OA_DbItem &item) const;

    // 用于_OUTPUT参数的枚举
    enum class OutputLevel {
        NONE = 0,               // 不输出
        HIGH = 1,               // 只输出高重要性
        HIGH_AND_NORMAL = 2,    // 输出高和普通重要性
        ALL = 3                 // 输出所有
    };

    // 参数
    AP_Int16        _queue_size_param;                      // 队列大小
    AP_Int16        _database_size_param;                   // 数据库大小
    AP_Int8         _database_expiry_seconds;               // 障碍物超时时间(秒)
    AP_Enum<OutputLevel> _output_level;                     // 控制哪些项应该发送到GCS
    AP_Float        _beam_width;                            // 将激光雷达读数转换为障碍物半径时使用的波束宽度
    AP_Float        _radius_min;                            // 障碍物最小半径(米)
    AP_Float        _dist_max;                              // 障碍物最大距离(米)
    AP_Float        _min_alt;                               // 数据库最小载具高度检查(米)

    // 队列结构
    struct {
        ObjectBuffer<OA_DbItem> *items;                     // 线程安全的传入队列,用于存储从近距离传感器获取的点
        uint16_t        size;                               // _queue_size_param的缓存值
        HAL_Semaphore   sem;                                // 用于多线程访问队列的信号量
    } _queue;
    float dist_to_radius_scalar;                            // 用于将距离和波束宽度转换为障碍物半径的标量

    // 数据库结构
    struct {
        OA_DbItem       *items;                             // 数据库中的障碍物数组
        uint16_t        count;                              // items数组中的障碍物数量
        uint16_t        size;                               // 初始化后固定的_database_size_param缓存值
    } _database;

    uint16_t _next_index_to_send[MAVLINK_COMM_NUM_BUFFERS]; // 下一个要发送到GCS的_database中的对象索引
    uint16_t _highest_index_sent[MAVLINK_COMM_NUM_BUFFERS]; // 发送到GCS的_database中的最高索引
    uint32_t _last_send_to_gcs_ms[MAVLINK_COMM_NUM_BUFFERS];// 最后一次调用send_adsb_vehicle的系统时间

    static AP_OADatabase *_singleton;                        // 单例指针
};

// AP命名空间
namespace AP {
    AP_OADatabase *oadatabase();  // 获取数据库实例
};

#endif  // AP_OADATABASE_ENABLED
