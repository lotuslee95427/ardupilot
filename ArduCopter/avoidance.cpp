#include "Copter.h"

// 根据高度检查是否应该启用简单避障功能
void Copter::low_alt_avoidance()
{
#if AP_AVOIDANCE_ENABLED
    // 定义变量存储当前高度(厘米)
    int32_t alt_cm;
    
    // 尝试获取测距仪插值高度
    if (!get_rangefinder_height_interpolated_cm(alt_cm)) {
        // 如果无法获取有效的测距仪读数,启用避障功能
        avoid.proximity_alt_avoidance_enable(true);
        return;
    }

    // 默认启用避障
    bool enable_avoidance = true;
    
    // 如果当前高度低于最小避障高度(转换为厘米),禁用避障
    if (alt_cm < avoid.get_min_alt() * 100.0f) {
        enable_avoidance = false;
    }
    
    // 根据计算结果设置避障功能的启用状态
    avoid.proximity_alt_avoidance_enable(enable_avoidance);
#endif
}
