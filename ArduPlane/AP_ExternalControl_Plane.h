/*
  飞机外部控制库
 */
#pragma once

#include <AP_ExternalControl/AP_ExternalControl.h>

#if AP_EXTERNAL_CONTROL_ENABLED

#include <AP_Common/Location.h>

class AP_ExternalControl_Plane : public AP_ExternalControl {
public:
    /*
      设置盘旋点的全球目标位置。
    */
    bool set_global_position(const Location& loc) override WARN_IF_UNUSED;
};

#endif // AP_EXTERNAL_CONTROL_ENABLED
