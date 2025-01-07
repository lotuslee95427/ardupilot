# 这是AnyleafH7飞控板的硬件定义文件
# 该文件由chibios_hwdef.py脚本处理,用于配置STM32H743芯片的硬件引脚和外设

# 定义MCU类型为STM32H7系列,具体型号为STM32H743
MCU STM32H7xx STM32H743xx

# 开发板ID为1146,用于固件校验和加载
APJ_BOARD_ID 1146

# 外部晶振频率为16MHz,用于系统时钟配置
OSCILLATOR_HZ 16000000

# Flash总容量为2048KB
FLASH_SIZE_KB 2048

# USB OTG1接口配置
PA11 OTG_FS_DM OTG1    # USB D-引脚
PA12 OTG_FS_DP OTG1    # USB D+引脚

# SWD调试接口配置
PA13 JTMS-SWDIO SWD    # SWDIO数据线
PA14 JTCK-SWCLK SWD    # SWCLK时钟线

# 启用SLCAN功能,允许通过USB-CAN适配器访问CAN总线
define AP_CAN_SCLAN_ENABLED 1
# 配置USB OTG2接口使用SLCAN协议
define HAL_OTG2_PROTOCOL SerialProtocol_SLCAN