# 这是AeroFox-PMU的主要硬件配置文件,定义了MCU、存储、定时器等基本硬件参数

# 定义MCU类型为STM32L431系列微控制器
MCU STM32L431 STM32L431xx

# bootloader从40KB处开始固件(36KB bootloader + 4KB 参数存储)
FLASH_RESERVE_START_KB 40
FLASH_SIZE_KB 256              # Flash总容量为256KB

# 在Flash的第18和19页存储参数
STORAGE_FLASH_PAGE 18
define HAL_STORAGE_SIZE 800    # 参数存储区大小为800字节

# ChibiOS系统定时器配置
STM32_ST_USE_TIMER 15         # 使用定时器15作为系统定时器
define CH_CFG_ST_RESOLUTION 16 # 定时器分辨率为16位

# 开发板ID为1080,用于固件校验和加载
APJ_BOARD_ID 1080

# 外部晶振频率为8MHz
OSCILLATOR_HZ 8000000

# 设置为外设模式
env AP_PERIPH 1

# 配置串口输出为SD1,波特率57600
STDOUT_SERIAL SD1
STDOUT_BAUDRATE 57600

# 系统配置参数
define HAL_NO_GPIO_IRQ                # 禁用GPIO中断
define SERIAL_BUFFERS_SIZE 512        # 串口缓冲区大小为512字节
define PORT_INT_REQUIRED_STACK 64     # 中断处理所需栈空间
define DMA_RESERVE_SIZE 0             # 不预留DMA空间

# 配置中断服务程序(ISR)处理程序栈大小为0x300
MAIN_STACK 0x300

# 配置主线程栈大小为0xA00
PROCESS_STACK 0xA00

# 设置一个小的默认参数文件,最大参数数量为512
define AP_PARAM_MAX_EMBEDDED_PARAM 512

# 配置活动指示LED,使用PB3引脚,高电平点亮
PB3 LED OUTPUT HIGH
define HAL_LED_ON 1

# 配置SWD调试接口引脚
PA13 JTMS-SWDIO SWD            # SWDIO数据线
PA14 JTCK-SWCLK SWD            # SWCLK时钟线

# 配置CAN总线
CAN_ORDER 1                     # 设置CAN总线优先级
PB8 CAN1_RX CAN1               # CAN1接收引脚
PB9 CAN1_TX CAN1               # CAN1发送引脚

# 设置CAN消息池大小为6000字节
define HAL_CAN_POOL_SIZE 6000

# 允许通过串口0发送重启命令,用于快速开发调试
define HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT 0

# 由于RAM不足,保持ROMFS不压缩,避免运行时解压bootloader
env ROMFS_UNCOMPRESSED True
