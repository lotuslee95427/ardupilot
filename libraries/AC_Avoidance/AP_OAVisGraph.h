#pragma once

#include "AC_Avoidance_config.h"

#if AP_OAPATHPLANNER_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Common/AP_ExpandingArray.h>

/*
 * Visibility graph used by Dijkstra's algorithm for path planning around fence, stay-out zones and moving obstacles
 * 用于Dijkstra算法的可见性图,用于围栏、禁区和移动障碍物周围的路径规划
 */
class AP_OAVisGraph {
public:
    AP_OAVisGraph();

    CLASS_NO_COPY(AP_OAVisGraph);  /* Do not allow copies */ // 不允许拷贝

    // types of items held in graph
    // 图中保存的项目类型
    enum OAType : uint8_t {
        OATYPE_SOURCE = 0,         // 源点
        OATYPE_DESTINATION,        // 目标点
        OATYPE_INTERMEDIATE_POINT, // 中间点
    };

    // support up to 255 items of each type
    // 每种类型最多支持255个项目
    typedef uint8_t oaid_num;

    // id for uniquely identifying objects held in visibility graphs and paths
    // 用于唯一标识可见性图和路径中对象的ID
    class OAItemID {
    public:
        OAType id_type;    // 项目类型
        oaid_num id_num;   // 项目编号
        bool operator ==(const OAItemID &i) const { return ((id_type == i.id_type) && (id_num == i.id_num)); }
    };

    struct VisGraphItem {
        OAItemID id1;       // first item's id 第一个项目的ID
        OAItemID id2;       // second item's id 第二个项目的ID
        float distance_cm;  // distance between the items 项目之间的距离(厘米)
    };

    // clear all elements from graph
    // 清除图中所有元素
    void clear() { _num_items = 0; }

    // get number of items in visibility graph table
    // 获取可见性图表中的项目数量
    uint16_t num_items() const { return _num_items; }

    // add item to visiblity graph, returns true on success, false if graph is full
    // 向可见性图添加项目,成功返回true,图满时返回false
    bool add_item(const OAItemID &id1, const OAItemID &id2, float distance_cm);

    // allow accessing graph as an array, 0 indexed
    // Note: no protection against out-of-bounds accesses so use with num_items()
    // 允许像数组一样访问图,从0开始索引
    // 注意:没有越界访问保护,所以要配合num_items()使用
    const VisGraphItem& operator[](uint16_t i) const { return _items[i]; }

private:

    AP_ExpandingArray<VisGraphItem> _items;  // 可扩展的项目数组
    uint16_t _num_items;                     // 项目总数
};

#endif  // AP_OAPATHPLANNER_ENABLED
