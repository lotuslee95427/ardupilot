/*
   本程序是自由软件：您可以根据自由软件基金会发布的GNU通用公共许可证的条款，
   即许可证的第3版或（您选择的）任何后来的版本重新发布它和/或修改它。

   本程序的发布是希望它能起到作用。但没有任何保证；甚至没有隐含的适销对路或适合特定用途的保证。
   更多细节请参见GNU通用公共许可证。

   您应该已经收到一份GNU通用公共许可证的副本。如果没有，请看<http://www.gnu.org/licenses/>。
 */
#pragma once

/*
  飞机高级故障保护支持
 */

#if AP_ADVANCEDFAILSAFE_ENABLED
#include <AP_AdvancedFailsafe/AP_AdvancedFailsafe.h>

/*
  飞机特定的AP_AdvancedFailsafe类
 */
class AP_AdvancedFailsafe_Plane : public AP_AdvancedFailsafe
{
public:

    // 使用基类的构造函数
    using AP_AdvancedFailsafe::AP_AdvancedFailsafe;

    // 调用此函数将所有输出设置为终止状态
    void terminate_vehicle(void) override;
    
protected:
    // 设置FMU固件停止运行时的故障保护值
    void setup_IO_failsafe(void) override;

    // 返回AFS映射的控制模式
    enum control_mode afs_mode(void) override;

    // 在数据链路丢失时强制进入自动模式
    void set_mode_auto(void) override;
};

#endif // AP_ADVANCEDFAILSAFE_ENABLED
