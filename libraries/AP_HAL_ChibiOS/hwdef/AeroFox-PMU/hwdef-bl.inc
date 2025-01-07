# 这是AeroFox-PMU电源管理单元的bootloader硬件配置文件

# 定义MCU类型为STM32L431系列
MCU STM32L431 STM32L431xx

# 开发板ID,用于固件校验和加载,1080是分配给AeroFox-PMU的ID号
APJ_BOARD_ID 1080

# 设置外部晶振频率为8MHz
OSCILLATOR_HZ 8000000

# 设置为外设模式
env AP_PERIPH 1

# Flash相关配置
FLASH_RESERVE_START_KB 0        # Flash起始保留区大小为0KB
FLASH_BOOTLOADER_LOAD_KB 36     # Bootloader占用36KB Flash空间
FLASH_SIZE_KB 256              # Flash总大小为256KB

# 应用程序代码起始偏移量为4KB
APP_START_OFFSET_KB 4

# 配置蓝色LED作为bootloader指示灯,低电平点亮
PB3 LED_BOOTLOADER OUTPUT LOW  # blue
define HAL_LED_ON 0

# 配置CAN总线引脚
PB8 CAN1_RX CAN1               # CAN1接收引脚
PB9 CAN1_TX CAN1               # CAN1发送引脚
# PC13 GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW  # CAN静默模式控制引脚(已注释)
CAN_ORDER 1                     # 设置CAN总线优先级

# Bootloader相关配置
define BOOTLOADER_BAUDRATE 57600    # 设置波特率
define HAL_BOOTLOADER_TIMEOUT 1000  # 设置超时时间为1000ms

# 系统配置
define HAL_USE_SERIAL FALSE         # 禁用串口功能
define HAL_NO_GPIO_IRQ             # 禁用GPIO中断
define SERIAL_BUFFERS_SIZE 32      # 设置串口缓冲区大小为32字节
define HAL_USE_EMPTY_IO TRUE       # 使用空IO
define PORT_INT_REQUIRED_STACK 64   # 设置中断所需栈空间
define DMA_RESERVE_SIZE 0          # 不预留DMA空间

# 设置主栈和进程栈大小为2KB
MAIN_STACK 0x800
PROCESS_STACK 0x800

# 配置SWD调试接口引脚
PA13 JTMS-SWDIO SWD            # SWDIO数据线
PA14 JTCK-SWCLK SWD            # SWCLK时钟线
