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

// 包含避障配置头文件
#include "AC_Avoidance_config.h"

#if AP_OAPATHPLANNER_ENABLED

// 包含可见性图头文件
#include "AP_OAVisGraph.h"

// 构造函数初始化扩展数组,每个块使用20个元素
AP_OAVisGraph::AP_OAVisGraph() :
    _items(20)
{
}

// 向可见性图添加项目,成功返回true,图满时返回false
bool AP_OAVisGraph::add_item(const OAItemID &id1, const OAItemID &id2, float distance_cm)
{
    // 不超过65k个项目
    if (_num_items == UINT16_MAX) {
        return false;
    }

    // 确保数组中有空间
    if (!_items.expand_to_hold(_num_items+1)) {
        return false;
    }

    // 添加项目
    _items[_num_items] = {id1, id2, distance_cm};
    _num_items++;
    return true;
}

#endif  // AP_OAPATHPLANNER_ENABLED
