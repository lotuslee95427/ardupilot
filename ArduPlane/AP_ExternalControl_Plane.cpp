/*
  飞机外部控制库
 */


#include "AP_ExternalControl_Plane.h"
#if AP_EXTERNAL_CONTROL_ENABLED

#include "Plane.h"

/*
  设置盘旋点的目标全球位置。
*/
bool AP_ExternalControl_Plane::set_global_position(const Location& loc)
{

    // set_target_location 已经检查飞机是否准备好进行外部控制。
    // 它不检查是否正在飞行或已解锁，只检查是否处于引导模式。
    return plane.set_target_location(loc);
}

#endif // AP_EXTERNAL_CONTROL_ENABLED
