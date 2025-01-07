#pragma once

// 定义开发板名称为Linux / Define board name as Linux
#define HAL_BOARD_NAME "Linux"
// 定义CPU等级为1000级 / Define CPU class as 1000
#define HAL_CPU_CLASS HAL_CPU_CLASS_1000
// 定义内存等级为1000级 / Define memory class as 1000
#define HAL_MEM_CLASS HAL_MEM_CLASS_1000
// 启用操作系统套接字 / Enable OS sockets
#define HAL_OS_SOCKETS 1
// 定义存储大小为16384字节 / Define storage size as 16384 bytes
#define HAL_STORAGE_SIZE            16384
// 定义可用存储大小等于总存储大小 / Define available storage size equal to total storage size
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE

// make sensor selection clearer
// 定义I2C IMU探测宏,用于添加IMU后端 / Define I2C IMU probe macro for adding IMU backend
#define PROBE_IMU_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,GET_I2C_DEVICE(bus, addr),##args))
// 定义双I2C IMU探测宏 / Define dual I2C IMU probe macro
#define PROBE_IMU_I2C2(driver, bus, addr1, addr2, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.i2c_mgr->get_device(bus, addr1),hal.i2c_mgr->get_device(bus, addr2),##args))
// 定义SPI IMU探测宏 / Define SPI IMU probe macro
#define PROBE_IMU_SPI(driver, devname, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname),##args))
// 定义双SPI IMU探测宏 / Define dual SPI IMU probe macro
#define PROBE_IMU_SPI2(driver, devname1, devname2, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname1),hal.spi->get_device(devname2),##args))

// 定义I2C气压计探测宏 / Define I2C barometer probe macro
#define PROBE_BARO_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_Baro_ ## driver::probe(*this,std::move(GET_I2C_DEVICE(bus, addr)),##args))
// 定义SPI气压计探测宏 / Define SPI barometer probe macro
#define PROBE_BARO_SPI(driver, devname, args ...) ADD_BACKEND(AP_Baro_ ## driver::probe(*this,std::move(hal.spi->get_device(devname)),##args))

// 定义I2C罗盘探测宏 / Define I2C compass probe macro
#define PROBE_MAG_I2C(driver, bus, addr, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe(GET_I2C_DEVICE(bus, addr),##args))
// 定义SPI罗盘探测宏 / Define SPI compass probe macro
#define PROBE_MAG_SPI(driver, devname, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe(hal.spi->get_device(devname),##args))
// 定义IMU内置罗盘探测宏 / Define IMU integrated compass probe macro
#define PROBE_MAG_IMU(driver, imudev, imu_instance, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe_ ## imudev(imu_instance,##args))
// 定义IMU内置I2C罗盘探测宏 / Define IMU integrated I2C compass probe macro
#define PROBE_MAG_IMU_I2C(driver, imudev, bus, addr, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe_ ## imudev(GET_I2C_DEVICE(bus,addr),##args))

// 根据不同的开发板子类型定义不同的配置 / Define different configurations based on board subtypes
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NONE
    // 定义日志目录 / Define log directory
    #define HAL_BOARD_LOG_DIRECTORY "logs"
    // 定义地形数据目录 / Define terrain data directory
    #define HAL_BOARD_TERRAIN_DIRECTORY "terrain"
    // 定义存储目录 / Define storage directory
    #define HAL_BOARD_STORAGE_DIRECTORY "."
    // 禁用默认IMU / Disable default IMU
    #define HAL_INS_DEFAULT HAL_INS_NONE
    // 禁用默认气压计 / Disable default barometer
    #define HAL_BARO_DEFAULT HAL_BARO_NONE
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD
    #if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF
      #define HAL_INS_PROBE_LIST PROBE_IMU_SPI(Invensense, "mpu9250", ROTATION_ROLL_180_YAW_270)
    #else
      #define HAL_INS_PROBE_LIST PROBE_IMU_SPI(Invensense, "mpu9250", ROTATION_ROLL_180_YAW_90)
    #endif
    #define HAL_BARO_PROBE_LIST PROBE_BARO_SPI(MS56XX, "ms5611")
    #define HAL_MAG_PROBE_LIST PROBE_MAG_IMU(AK8963, mpu9250, 0, ROTATION_NONE)
    #define HAL_PROBE_EXTERNAL_I2C_COMPASSES
    #define AP_NOTIFY_GPIO_LED_3_ENABLED 1
    #define HAL_GPIO_A_LED_PIN        61
    #define HAL_GPIO_B_LED_PIN        48
    #define HAL_GPIO_C_LED_PIN        117

[... rest of the file continues with similar configuration blocks for different board subtypes ...]
