// 用户特定配置文件。config.h中列出的任何项目都可以在这里被覆盖。

// 取消注释以下行以禁用功能（列出的闪存大小是针对APM2板，在Pixhawk和其他板上的节省会被低估）
//#define LOGGING_ENABLED       0            // 禁用日志记录以节省11K闪存空间
//#define MOUNT                 0            // 禁用相机云台以节省8K闪存空间
//#define AUTOTUNE_ENABLED      0            // 禁用自动调谐功能以节省7K闪存
//#define NAV_GUIDED            0            // 禁用外部导航计算机通过MAV_CMD_NAV_GUIDED任务命令控制飞行器的能力
//#define MODE_ACRO_ENABLED     0            // 禁用特技模式支持
//#define MODE_AUTO_ENABLED     0            // 禁用自动模式支持
//#define MODE_BRAKE_ENABLED    0            // 禁用刹车模式支持
//#define MODE_CIRCLE_ENABLED   0            // 禁用圆圈模式支持
//#define MODE_DRIFT_ENABLED    0            // 禁用漂移模式支持
//#define MODE_FLIP_ENABLED     0            // 禁用翻转模式支持
//#define MODE_FOLLOW_ENABLED   0            // 禁用跟随模式支持
//#define MODE_GUIDED_ENABLED   0            // 禁用引导模式支持
//#define MODE_GUIDED_NOGPS_ENABLED   0      // 禁用无GPS引导模式支持
//#define MODE_LOITER_ENABLED   0            // 禁用悬停模式支持
//#define MODE_POSHOLD_ENABLED  0            // 禁用定点模式支持
//#define MODE_RTL_ENABLED      0            // 禁用返航模式支持
//#define MODE_SMARTRTL_ENABLED 0            // 禁用智能返航模式支持
//#define MODE_SPORT_ENABLED    0            // 禁用运动模式支持
//#define MODE_SYSTEMID_ENABLED 0            // 禁用系统识别模式支持
//#define MODE_THROW_ENABLED    0            // 禁用抛飞模式支持
//#define MODE_ZIGZAG_ENABLED   0            // 禁用之字形模式支持
//#define OSD_ENABLED           0            // 禁用屏幕显示支持

// 以下功能在所有板上默认禁用
//#define CAL_ALWAYS_REBOOT                         // 罗盘或加速度计校准完成后，飞行控制器将重启
//#define DISALLOW_GCS_MODE_CHANGE_DURING_RC_FAILSAFE   // 在遥控器故障保护期间禁止地面站更改模式。避免像Solo这样的飞行器在遥控和遥测使用同一链路时的竞争条件
//#define ADVANCED_FAILSAFE     1             // 启用高级故障保护，允许在故障保护事件中执行部分任务

// 其他设置
//#define THROTTLE_IN_DEADBAND   100                // 重新定义油门死区大小（PWM值，0 ~ 1000）

// 用户钩子：用于用户开发的代码
// 将您的变量定义放入UserVariables.h文件中（或另一个文件名，然后更改下面的#define）。
//#define USERHOOK_VARIABLES "UserVariables.h"
// 将您的自定义代码放入UserCode.cpp中，函数名称与下面列出的相匹配，并确保取消注释相应的#define
//#define USERHOOK_INIT userhook_init();                      // 用于在启动时运行一次的代码
//#define USERHOOK_FASTLOOP userhook_FastLoop();            // 用于以100Hz运行的代码
//#define USERHOOK_50HZLOOP userhook_50Hz();                  // 用于以50Hz运行的代码
//#define USERHOOK_MEDIUMLOOP userhook_MediumLoop();        // 用于以10Hz运行的代码
//#define USERHOOK_SLOWLOOP userhook_SlowLoop();            // 用于以3.3Hz运行的代码
//#define USERHOOK_SUPERSLOWLOOP userhook_SuperSlowLoop();  // 用于以1Hz运行的代码
//#define USERHOOK_AUXSWITCH 1                        // 用于处理用户辅助开关的代码
//#define USER_PARAMS_ENABLED 1                       // 启用用户参数
