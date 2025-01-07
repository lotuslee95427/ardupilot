/*
 * 版权所有 (C) 2015-2016  Intel Corporation. 保留所有权利。
 *
 * 本文件是自由软件:你可以在自由软件基金会发布的GNU通用公共许可证的条款下
 * 重新分发和/或修改它,可以选择使用版本3或更高版本的许可证。
 *
 * 本文件的发布是希望它能有用,但不提供任何保证;甚至没有对适销性或特定用途
 * 适用性的暗示保证。详细信息请参见GNU通用公共许可证。
 *
 * 你应该随程序收到一份GNU通用公共许可证的副本。如果没有,
 * 请参见<http://www.gnu.org/licenses/>。
 */
#pragma once

#include <inttypes.h>

#include "AP_HAL_Namespace.h"
#include "utility/functor.h"
#include "AP_HAL_Boards.h"

#if CONFIG_HAL_BOARD != HAL_BOARD_QURT
// 由于hexagon SDK中的include错误,我们需要utility来使用std::move,但在QURT上不需要
#include <utility>
#endif

/*
 * 这是一个抽象I2C和SPI设备的接口类
 */
class AP_HAL::Device {
public:
    // 定义总线类型枚举
    enum BusType {
        BUS_TYPE_UNKNOWN = 0,  // 未知总线类型
        BUS_TYPE_I2C     = 1,  // I2C总线
        BUS_TYPE_SPI     = 2,  // SPI总线
        BUS_TYPE_UAVCAN  = 3,  // UAVCAN总线
        BUS_TYPE_SITL    = 4,  // SITL模拟总线
        BUS_TYPE_MSP     = 5,  // MSP总线
        BUS_TYPE_SERIAL  = 6,  // 串行总线
        BUS_TYPE_WSPI    = 7,  // 宽SPI总线
    };

    // 定义速度枚举
    enum Speed {
        SPEED_HIGH,  // 高速
        SPEED_LOW,   // 低速
    };

    // 用于支持宽SPI(如四线SPI)设备通信的命令头结构
    struct CommandHeader {
        uint32_t  cmd;    // 命令阶段数据
        uint32_t  cfg;    // 传输配置字段
        uint32_t  addr;   // 地址阶段数据
        uint32_t  alt;    // 备用阶段数据
        uint32_t  dummy;  // 要插入的空周期数
    };

    // 定义周期性回调函数类型
    FUNCTOR_TYPEDEF(PeriodicCb, void);
    typedef void* PeriodicHandle;

    // 寄存器读写回调函数
    // 返回:void 参数:寄存器地址,寄存器数据,数据大小,方向(读:false,写:true)
    FUNCTOR_TYPEDEF(RegisterRWCb, void, uint8_t, uint8_t*, uint32_t, bool);
    typedef void* RegisterRWHandle;

    // Bank选择回调函数类型
    FUNCTOR_TYPEDEF(BankSelectCb, bool, uint8_t);

    // 构造函数,设置总线类型
    Device(enum BusType type)
    {
        _bus_id.devid_s.bus_type = type;
    }

    // 返回总线类型
    enum BusType bus_type(void) const {
        return _bus_id.devid_s.bus_type;
    }

    // 返回总线编号
    uint8_t bus_num(void) const {
        return _bus_id.devid_s.bus;
    }

    // 返回24位总线标识符
    uint32_t get_bus_id(void) const {
        return _bus_id.devid;
    }

    // 返回总线地址
    uint8_t get_bus_address(void) const {
        return _bus_id.devid_s.address;
    }

    // 在设备类中设置设备类型(如 AP_COMPASS_TYPE_LSM303D)
    void set_device_type(uint8_t devtype);

    // 析构函数
    virtual ~Device() {
        delete[] _checked.regs;
    }

    /*
     * 更改设备地址。注意这是7位地址,不包含读/写位。
     * 仅适用于I2C设备
     */
    virtual void set_address(uint8_t address) {};
    
    /*
     * 设置未来传输的速度。根据总线类型,速度可能会在同一总线上的所有设备间共享。
     *
     * 返回:如果速度设置成功或平台未实现此功能则返回true;否则返回false。
     */
    virtual bool set_speed(Speed speed)  = 0;

    /*
     * 核心传输函数。执行单次总线事务,向从设备发送send_len字节并接收recv_len字节。
     *
     * 返回:传输成功返回true,失败返回false。
     */
    virtual bool transfer(const uint8_t *send, uint32_t send_len,
                          uint8_t *recv, uint32_t recv_len) = 0;

    /*
     * 在事务开始前设置所需标志
     * 用于宽SPI通信接口如双/四/八线SPI
     */
    virtual void set_cmd_header(const CommandHeader& cmd_hdr) {}

    /*
     * 设置外设进入就地执行模式
     * 仅适用于宽SPI设置
     */
    virtual bool enter_xip_mode(void** map_ptr) { return false; }
    virtual bool exit_xip_mode() { return false; }

    /**
     * #transfer()的包装函数,用于从first_reg开始读取recv_len个寄存器到recv指向的数组。
     * 在执行传输前,通过#set_read_flag(uint8_t)设置的读标志会与first_reg进行OR运算。
     *
     * 返回:传输成功返回true,失败返回false。
     */
    bool read_registers(uint8_t first_reg, uint8_t *recv, uint32_t recv_len);

    /**
     * #transfer()的包装函数,用于向寄存器reg写入一个字节。
     * 传输时按reg和val的顺序发送。
     *
     * 返回:传输成功返回true,失败返回false。
     */
    bool write_register(uint8_t reg, uint8_t val, bool checked=false);
    
    /*
     * 设置寄存器读写回调函数
     */
    virtual void set_register_rw_callback(RegisterRWCb register_rw_callback) {
        _register_rw_callback = register_rw_callback;
    }

    /**
     * #transfer()的包装函数,先调用bank选择回调函数,然后执行传输
     *
     * 返回:传输成功返回true,失败返回false。
     */
    bool transfer_bank(uint8_t bank, const uint8_t *send, uint32_t send_len,
                          uint8_t *recv, uint32_t recv_len);

    /**
     * #transfer_bank()的包装函数,从first_reg开始读取recv_len个寄存器到recv指向的数组。
     * 在执行传输前,通过#set_read_flag(uint8_t)设置的读标志会与first_reg进行OR运算。
     *
     * 返回:传输成功返回true,失败返回false。
     */
    bool read_bank_registers(uint8_t bank, uint8_t first_reg, uint8_t *recv, uint32_t recv_len);

    /**
     * #transfer_bank()的包装函数,用于向bank中的寄存器reg写入一个字节。
     * 传输时按reg和val的顺序发送。
     *
     * 返回:传输成功返回true,失败返回false。
     */
    bool write_bank_register(uint8_t bank, uint8_t reg, uint8_t val, bool checked=false);

    /**
     * 在bank中为检查寄存器设置值
     */
    void set_checked_register(uint8_t bank, uint8_t reg, uint8_t val);

    /**
     * 为检查寄存器设置值
     */
    void set_checked_register(uint8_t reg, uint8_t val);

    /**
     * 设置寄存器值检查。frequency参数设置检查频率。
     * 如果设为10,则每10次调用check_next_register才会检查一次寄存器
     */
    bool setup_checked_registers(uint8_t num_regs, uint8_t frequency=10);

    /**
     * 检查下一个寄存器值是否正确。
     * 如果值不正确或寄存器检查未设置则返回false
     */
    bool check_next_register(void);

    // 检查寄存器结构体
    struct checkreg {
        uint8_t bank;    // bank编号
        uint8_t regnum;  // 寄存器编号
        uint8_t value;   // 寄存器值
    };
    
    /**
     * 检查下一个寄存器值是否正确,并返回失败信息。
     * 如果值不正确或寄存器检查未设置则返回false
     */
    bool check_next_register(struct checkreg &fail);
    
    /**
     * #transfer()的包装函数,用于从设备读取一系列字节。
     * 与#read_registers()方法不同,不写入任何值,
     * 因此不包含通过#set_read_flag()设置的读标志
     */
    bool read(uint8_t *recv, uint32_t recv_len)
    {
        return transfer(nullptr, 0, recv, recv_len);
    }

    /*
     * 获取此设备所在总线的信号量。
     * 这主要供驱动程序在初始化阶段使用。
     */
    virtual AP_HAL::Semaphore *get_semaphore() = 0;

    /*
     * 为此总线注册周期性回调函数。同一总线上的所有回调都在同一线程中执行,
     * 且已获取锁。换句话说,回调不在主线程(或注册回调的线程)中执行,
     * 而是在每个总线专用的独立线程中执行。
     *
     * 注册周期性回调后,不应再从其他上下文使用其他函数。
     * 如果确实需要这样做,必须获取锁。
     *
     * 返回:此周期性回调的句柄。要取消回调,
     * 调用#unregister_callback()或在回调中返回false。
     */
    virtual PeriodicHandle register_periodic_callback(uint32_t period_usec, PeriodicCb) = 0;

    /*
     * 调整通过#register_periodic_callback注册的周期性回调的时间。
     * 注意时间将从此调用时刻重新计算,并在@period_usec后到期。
     *
     * 返回:如果成功调整周期性回调则返回true,否则返回false。
     */
    virtual bool adjust_periodic_callback(PeriodicHandle h, uint32_t period_usec) = 0;

    /*
     * 取消此总线上的周期性回调。
     *
     * 返回:如果成功注销回调则返回true,否则返回false。
     */
    virtual bool unregister_callback(PeriodicHandle h) { return false; }

    /*
     * 设置用于寄存器检查期间bank选择的回调函数
     */
    virtual void setup_bankselect_callback(BankSelectCb bank_select) {
        _bank_select = bank_select;
    }

    /*
     * 注销bank选择回调函数
     */
    virtual void deregister_bankselect_callback() {
        _bank_select = nullptr;
    }

    /*
     * 允许设置DMA传输完成后调用的回调函数。
     * 如果设置了此回调,任何读/写操作将在设置传输后直接返回,
     * 且在回调本身调用register_completion_callback(0)之前不能释放总线信号量
     */
    virtual void register_completion_callback(AP_HAL::MemberProc proc) {}
    virtual void register_completion_callback(AP_HAL::Proc proc) {}
    
    /*
     * 支持直接控制SPI片选。
     * 对于具有特定延迟的非常规SPI传输模式的设备是必需的
     */
    virtual bool set_chip_select(bool set) { return false; }

    /**
     * 连接在I2C或SPI总线上的某些设备需要在寄存器地址上设置一个位来执行读操作。
     * 此函数设置#read_registers()使用的标志。标志的默认值为零。
     */
    void set_read_flag(uint8_t flag);

    /**
     * 根据总线类型、总线编号、总线地址和设备类型生成总线ID。
     * 这用于不使用标准HAL设备类型的设备,如UAVCAN设备
     */
    static uint32_t make_bus_id(enum BusType bus_type, uint8_t bus, uint8_t address, uint8_t devtype);

    /**
     * 返回相同总线连接但新设备类型的总线ID。
     * 这用于辅助总线连接
     */
    static uint32_t change_bus_id(uint32_t old_id, uint8_t devtype);

    /**
     * 返回带有新devtype的总线ID
     */
    uint32_t get_bus_id_devtype(uint8_t devtype) const;

    /**
     * 获取总线类型
     */
    static enum BusType devid_get_bus_type(uint32_t dev_id);

    // 获取总线编号
    static uint8_t devid_get_bus(uint32_t dev_id);

    // 获取总线地址
    static uint8_t devid_get_address(uint32_t dev_id);

    // 获取设备类型
    static uint8_t devid_get_devtype(uint32_t dev_id);

    /* 设置传输重试次数 */
    virtual void set_retries(uint8_t retries) {};

protected:
    uint8_t _read_flag = 0;  // 读标志

    /*
     * 分解的设备元素。使用位域来保持整体值足够小,
     * 以便能够在float中准确表示,这使得可以通过MAVLink
     * 参数协议传输而不丢失信息。
     */
    struct DeviceStructure {
        enum BusType bus_type : 3;  // 总线类型
        uint8_t bus: 5;             // 总线类型的实例编号
        uint8_t address;            // 总线地址(如I2C地址)
        uint8_t devtype;            // 设备类特定的设备类型
    };

    // 设备ID联合体
    union DeviceId {
        struct DeviceStructure devid_s;
        uint32_t devid;
    };

    union DeviceId _bus_id;

    // 设置设备地址(如i2c总线地址或spi片选)
    void set_device_address(uint8_t address) {
        _bus_id.devid_s.address = address;
    }

    // 设置设备总线编号
    void set_device_bus(uint8_t bus) {
        _bus_id.devid_s.bus = bus;
    }

private:
    BankSelectCb _bank_select;           // bank选择回调
    RegisterRWCb _register_rw_callback;  // 寄存器读写回调
    
    // 检查寄存器相关结构
    struct {
        uint8_t n_allocated;         // 已分配数量
        uint8_t n_set;              // 已设置数量
        uint8_t next;               // 下一个检查位置
        uint8_t frequency;          // 检查频率
        uint8_t counter;            // 计数器
        struct checkreg last_reg_fail;  // 最后失败的寄存器
        struct checkreg *regs;          // 寄存器数组
    } _checked;
};
