/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */

// 包含必要的头文件
#include <hal.h>
#include "GPIO.h"

#include <AP_BoardConfig/AP_BoardConfig.h>
#include "hwdef/common/stm32_util.h"
#include <AP_InternalError/AP_InternalError.h>
#ifndef HAL_BOOTLOADER_BUILD
#include <SRV_Channel/SRV_Channel.h>
#endif
#ifndef HAL_NO_UARTDRIVER
#include <GCS_MAVLink/GCS.h>
#endif
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Math/AP_Math.h>

// 使用ChibiOS命名空间
using namespace ChibiOS;

#if HAL_WITH_IO_MCU
#include <AP_IOMCU/AP_IOMCU.h>
extern AP_IOMCU iomcu;
#endif

// 从hwdef.dat定义的GPIO引脚表结构体
struct gpio_entry {
    uint8_t pin_num;                                  // 引脚编号
    bool enabled;                                     // 是否启用
    uint8_t pwm_num;                                 // PWM通道号
    ioline_t pal_line;                               // ChibiOS PAL线
    AP_HAL::GPIO::irq_handler_fn_t fn;              // GPIO中断回调函数
    thread_reference_t thd_wait;                     // 等待线程引用
    bool is_input;                                   // 是否为输入引脚
    uint8_t mode;                                    // 引脚模式
    uint16_t isr_quota;                             // 中断配额
    uint8_t isr_disabled_ticks;                      // 中断禁用计数
    AP_HAL::GPIO::INTERRUPT_TRIGGER_TYPE isr_mode;   // 中断触发模式
};

// 如果定义了HAL_GPIO_PINS,则启用GPIO引脚
#ifdef HAL_GPIO_PINS
#define HAVE_GPIO_PINS 1
static struct gpio_entry _gpio_tab[] = HAL_GPIO_PINS;
#else
#define HAVE_GPIO_PINS 0
#endif


/*
  map a user pin number to a GPIO table entry
  将用户引脚号映射到GPIO表项
 */
static struct gpio_entry *gpio_by_pin_num(uint8_t pin_num, bool check_enabled=true)
{
#if HAVE_GPIO_PINS
    for (uint8_t i=0; i<ARRAY_SIZE(_gpio_tab); i++) {
        const auto &t = _gpio_tab[i];
        if (pin_num == t.pin_num) {
            if (check_enabled && t.pwm_num != 0 && !t.enabled) {
                return NULL;
            }
            return &_gpio_tab[i];
        }
    }
#endif
    return NULL;
}

// 中断回调函数声明
static void pal_interrupt_cb(void *arg);
static void pal_interrupt_cb_functor(void *arg);

// GPIO类构造函数
GPIO::GPIO()
{}

// GPIO初始化函数
void GPIO::init()
{
#if !APM_BUILD_TYPE(APM_BUILD_iofirmware) && !defined(HAL_BOOTLOADER_BUILD)
#if HAL_WITH_IO_MCU || HAVE_GPIO_PINS
    uint8_t chan_offset = 0;
#endif
#if HAL_WITH_IO_MCU
    // 如果启用了IO MCU,设置GPIO掩码
    if (AP_BoardConfig::io_enabled()) {
        uint8_t GPIO_mask = 0;
        for (uint8_t i=0; i<8; i++) {
            if (SRV_Channels::is_GPIO(i)) {
                GPIO_mask |= 1U << i;
            }
        }
        iomcu.set_GPIO_mask(GPIO_mask);
        chan_offset = 8;
    }
#endif
    // 自动禁用用于PWM输出的引脚
#if HAVE_GPIO_PINS
    for (uint8_t i=0; i<ARRAY_SIZE(_gpio_tab); i++) {
        struct gpio_entry *g = &_gpio_tab[i];
        if (g->pwm_num != 0) {
            g->enabled = SRV_Channels::is_GPIO((g->pwm_num-1)+chan_offset);
        }
    }
#endif // HAVE_GPIO_PINS
#endif // HAL_BOOTLOADER_BUILD
#ifdef HAL_PIN_ALT_CONFIG
    setup_alt_config();
#endif
}

#ifdef HAL_PIN_ALT_CONFIG
// 选择的替代配置
uint8_t GPIO::alt_config;

/*
  alternative config table, selected using BRD_ALT_CONFIG
  使用BRD_ALT_CONFIG选择的替代配置表
 */
static const struct alt_config {
    uint8_t alternate;        // 替代配置编号
    uint16_t mode;           // 引脚模式
    ioline_t line;           // ChibiOS PAL线
    PERIPH_TYPE periph_type; // 外设类型
    uint8_t periph_instance; // 外设实例
} alternate_config[] HAL_PIN_ALT_CONFIG;

/*
  change pin configuration based on ALT() lines in hwdef.dat
  根据hwdef.dat中的ALT()行更改引脚配置
 */
void GPIO::setup_alt_config(void)
{
    AP_BoardConfig *bc = AP::boardConfig();
    if (!bc) {
        return;
    }
    alt_config = bc->get_alt_config();
    if (alt_config == 0) {
        // 使用默认配置
        return;
    }
    for (uint8_t i=0; i<ARRAY_SIZE(alternate_config); i++) {
        const struct alt_config &alt = alternate_config[i];
        if (alt_config == alt.alternate) {
            if (alt.periph_type == PERIPH_TYPE::GPIO) {
                // 在GPIO表中启用引脚
#if HAVE_GPIO_PINS
                for (uint8_t j=0; j<ARRAY_SIZE(_gpio_tab); j++) {
                    struct gpio_entry *g = &_gpio_tab[j];
                    if (g->pal_line == alt.line) {
                        g->enabled = true;
                        break;
                    }
                }
#endif // HAVE_GPIO_PINS
                continue;
            }
            const iomode_t mode = alt.mode & ~PAL_STM32_HIGH;
            const uint8_t odr = (alt.mode & PAL_STM32_HIGH)?1:0;
            palSetLineMode(alt.line, mode);
            palWriteLine(alt.line, odr);
        }
    }
}
#endif // HAL_PIN_ALT_CONFIG

/*
   resolve an ioline_t to take account of alternative
   configurations. This allows drivers to get the right ioline_t for an
   alternative config. Note that this may return 0, meaning the pin is
   not mapped to this peripheral in the active config
   
   解析ioline_t以考虑替代配置。这允许驱动程序获取替代配置的正确ioline_t。
   注意这可能返回0,表示在当前配置中该引脚未映射到此外设。
*/
ioline_t GPIO::resolve_alt_config(ioline_t base, PERIPH_TYPE ptype, uint8_t instance)
{
#ifdef HAL_PIN_ALT_CONFIG
    if (alt_config == 0) {
        // 保持不变
        return base;
    }
    for (uint8_t i=0; i<ARRAY_SIZE(alternate_config); i++) {
        const struct alt_config &alt = alternate_config[i];
        if (alt_config == alt.alternate) {
            if (ptype == alt.periph_type && instance == alt.periph_instance) {
                // 我们已经用不同的线重新配置了这个外设
                return alt.line;
            }
        }
    }
    // 现在搜索通过BRD_ALT_CONFIG配置关闭的引脚
    for (uint8_t i=0; i<ARRAY_SIZE(alternate_config); i++) {
        const struct alt_config &alt = alternate_config[i];
        if (alt_config == alt.alternate) {
            if (alt.line == base) {
                // 在此配置中此线不再可用
                return 0;
            }
        }
    }
#endif
    return base;
}


// 设置引脚模式(输入/输出)
void GPIO::pinMode(uint8_t pin, uint8_t output)
{
    struct gpio_entry *g = gpio_by_pin_num(pin);
    if (g) {
        if (!output && g->is_input &&
            (g->mode == PAL_MODE_INPUT_PULLUP ||
             g->mode == PAL_MODE_INPUT_PULLDOWN)) {
            // 已经设置
            return;
        }
        g->mode = output?PAL_MODE_OUTPUT_PUSHPULL:PAL_MODE_INPUT;
#if defined(STM32F7) || defined(STM32H7) || defined(STM32F4) || defined(STM32G4) || defined(STM32L4) || defined(STM32L4PLUS)
        if (g->mode == PAL_MODE_OUTPUT_PUSHPULL) {
            // 如果已设置则保留OPENDRAIN
            iomode_t old_mode = palReadLineMode(g->pal_line);
            if ((old_mode & PAL_MODE_OUTPUT_OPENDRAIN) == PAL_MODE_OUTPUT_OPENDRAIN) {
                g->mode = PAL_MODE_OUTPUT_OPENDRAIN;
            }
        }
#endif
        palSetLineMode(g->pal_line, g->mode);
        g->is_input = !output;
    }
}


// 读取引脚状态
uint8_t GPIO::read(uint8_t pin)
{
    struct gpio_entry *g = gpio_by_pin_num(pin);
    if (g) {
        return palReadLine(g->pal_line);
    }
#if HAL_WITH_IO_MCU
    if (AP_BoardConfig::io_enabled() && iomcu.valid_GPIO_pin(pin)) {
        return iomcu.read_virtual_GPIO(pin);
    }
#endif
    return 0;
}

// 写入引脚状态
void GPIO::write(uint8_t pin, uint8_t value)
{
    struct gpio_entry *g = gpio_by_pin_num(pin);
    if (g) {
        if (g->is_input) {
            // 控制上拉/下拉
            g->mode = value==1?PAL_MODE_INPUT_PULLUP:PAL_MODE_INPUT_PULLDOWN;
            palSetLineMode(g->pal_line, g->mode);
        } else if (value == PAL_LOW) {
            palClearLine(g->pal_line);
        } else {
            palSetLine(g->pal_line);
        }
        return;
    }
#if HAL_WITH_IO_MCU
    if (AP_BoardConfig::io_enabled() && iomcu.valid_GPIO_pin(pin)) {
        iomcu.write_GPIO(pin, value);
    }
#endif
}

// 切换引脚状态
void GPIO::toggle(uint8_t pin)
{
    struct gpio_entry *g = gpio_by_pin_num(pin);
    if (g) {
        palToggleLine(g->pal_line);
        return;
    }
#if HAL_WITH_IO_MCU
    if (AP_BoardConfig::io_enabled() && iomcu.valid_GPIO_pin(pin)) {
        iomcu.toggle_GPIO(pin);
    }
#endif
}

/* Alternative interface: */
// 获取数字IO对象
AP_HAL::DigitalSource* GPIO::channel(uint16_t pin)
{
    struct gpio_entry *g = gpio_by_pin_num(pin);
    if (g != nullptr) {
        return NEW_NOTHROW DigitalSource(g->pal_line);
    }
#if HAL_WITH_IO_MCU
    if (AP_BoardConfig::io_enabled() && iomcu.valid_GPIO_pin(pin)) {
        return NEW_NOTHROW IOMCU_DigitalSource(pin);
    }
#endif
    return nullptr;
}

extern const AP_HAL::HAL& hal;

/*
   Attach an interrupt handler to a GPIO pin number. The pin number
   must be one specified with a GPIO() marker in hwdef.dat
   
   将中断处理程序附加到GPIO引脚号。引脚号必须在hwdef.dat中用GPIO()标记指定
 */
bool GPIO::attach_interrupt(uint8_t pin,
                            irq_handler_fn_t fn,
                            INTERRUPT_TRIGGER_TYPE mode)
{
    struct gpio_entry *g = gpio_by_pin_num(pin, false);
    if (!g) {
        return false;
    }
    g->isr_disabled_ticks = 0;
    g->isr_quota = 0;
    if (!_attach_interrupt(g->pal_line,
                           palcallback_t(fn?pal_interrupt_cb_functor:nullptr),
                           g,
                           mode)) {
        return false;
    }
    g->fn = fn;
    g->isr_mode = mode;
    return true;
}

/*
   Attach an interrupt handler to ioline_t
   将中断处理程序附加到ioline_t
 */
bool GPIO::_attach_interrupt(ioline_t line, AP_HAL::Proc p, uint8_t mode)
{
    return _attach_interrupt(line, palcallback_t(p?pal_interrupt_cb:nullptr), (void*)p, mode);
}

// 附加中断处理程序
bool GPIO::attach_interrupt(uint8_t pin,
                            AP_HAL::Proc proc,
                            INTERRUPT_TRIGGER_TYPE mode) {
    struct gpio_entry *g = gpio_by_pin_num(pin, false);
    if (!g) {
        return false;
    }
    g->isr_disabled_ticks = 0;
    g->isr_quota = 0;
    g->isr_mode = mode;
    return _attach_interrupt(g->pal_line, proc, mode);
}

// 在中断上下文中附加中断
bool GPIO::_attach_interruptI(ioline_t line, palcallback_t cb, void *p, uint8_t mode)
{
    uint32_t chmode = 0;
    switch(mode) {
        case INTERRUPT_FALLING:
            chmode = PAL_EVENT_MODE_FALLING_EDGE;
            break;
        case INTERRUPT_RISING:
            chmode = PAL_EVENT_MODE_RISING_EDGE;
            break;
        case INTERRUPT_BOTH:
            chmode = PAL_EVENT_MODE_BOTH_EDGES;
            break;
        default:
            if (p) {
                return false;
            }
            break;
    }

    palevent_t *pep = pal_lld_get_line_event(line);
    if (pep->cb && p != nullptr) {
        // 该引脚已被用于回调
        return false;
    }

    if (!p) {
        chmode = PAL_EVENT_MODE_DISABLED;
    }

    palDisableLineEventI(line);
    palSetLineCallbackI(line, cb, p);
    palEnableLineEventI(line, chmode);

    return true;
}

// 附加中断
bool GPIO::_attach_interrupt(ioline_t line, palcallback_t cb, void *p, uint8_t mode)
{
    osalSysLock();
    bool ret = _attach_interruptI(line, cb, p, mode);
    osalSysUnlock();
    return ret;
}

// 检查USB是否连接
bool GPIO::usb_connected(void)
{
    return _usb_connected;
}

// 数字IO源构造函数
DigitalSource::DigitalSource(ioline_t _line) :
    line(_line)
{}

// 设置数字IO源模式
void DigitalSource::mode(uint8_t output)
{
    palSetLineMode(line, output);
}

// 读取数字IO源状态
uint8_t DigitalSource::read()
{
    return palReadLine(line);
}

// 写入数字IO源状态
void DigitalSource::write(uint8_t value)
{
    palWriteLine(line, value);
}

// 切换数字IO源状态
void DigitalSource::toggle()
{
    palToggleLine(line);
}

#if HAL_WITH_IO_MCU
// IOMCU数字IO源构造函数
IOMCU_DigitalSource::IOMCU_DigitalSource(uint8_t _pin) :
    pin(_pin)
{}

// 写入IOMCU数字IO源状态
void IOMCU_DigitalSource::write(uint8_t value)
{
    iomcu.write_GPIO(pin, value);
}

// 切换IOMCU数字IO源状态
void IOMCU_DigitalSource::toggle()
{
    iomcu.toggle_GPIO(pin);
}
#endif // HAL_WITH_IO_MCU

// 中断回调函数
static void pal_interrupt_cb(void *arg)
{
    if (arg != nullptr) {
        ((AP_HAL::Proc)arg)();
    }
}

// 中断回调函数仿函数
static void pal_interrupt_cb_functor(void *arg)
{
    const uint32_t now = AP_HAL::micros();

    struct gpio_entry *g = (gpio_entry *)arg;
    if (g == nullptr) {
        // what?
        return;
    }
    if (!(g->fn)) {
        return;
    }
    if (g->isr_quota >= 1) {
        /*
          we have an interrupt quota enabled for this pin. If the
          quota remaining drops to 1 without it being refreshed in
          timer_tick then we disable the interrupt source. This is to
          prevent CPU overload due to very high GPIO interrupt counts
          
          我们为此引脚启用了中断配额。如果剩余配额降至1且在timer_tick中未刷新,
          则禁用中断源。这是为了防止由于GPIO中断计数过高而导致CPU过载
         */
        if (g->isr_quota == 1) {
            osalSysLockFromISR();
            palDisableLineEventI(g->pal_line);
            osalSysUnlockFromISR();
            return;
        }
        g->isr_quota--;
    }
    (g->fn)(g->pin_num, palReadLine(g->pal_line), now);
}

/*
  handle interrupt from pin change for wait_pin()
  处理wait_pin()的引脚变化中断
 */
static void pal_interrupt_wait(void *arg)
{
    osalSysLockFromISR();
    struct gpio_entry *g = (gpio_entry *)arg;
    if (g == nullptr || g->thd_wait == nullptr) {
        osalSysUnlockFromISR();
        return;
    }
    osalThreadResumeI(&g->thd_wait, MSG_OK);
    osalSysUnlockFromISR();
}

/*
  block waiting for a pin to change. Return true on pin change, false on timeout
  阻塞等待引脚变化。引脚变化时返回true,超时返回false
*/
bool GPIO::wait_pin(uint8_t pin, INTERRUPT_TRIGGER_TYPE mode, uint32_t timeout_us)
{
    struct gpio_entry *g = gpio_by_pin_num(pin);
    if (!g) {
        return false;
    }

    osalSysLock();
    if (g->thd_wait) {
        // 只允许单个等待者
        osalSysUnlock();
        return false;
    }

    if (!_attach_interruptI(g->pal_line,
                           palcallback_t(pal_interrupt_wait),
                           g,
                           mode)) {
        osalSysUnlock();
        return false;
    }

    // 不允许太长或太短的超时时间
    timeout_us = constrain_uint32(TIME_US2I(timeout_us), CH_CFG_ST_TIMEDELTA, TIME_US2I(30000U));

    msg_t msg = osalThreadSuspendTimeoutS(&g->thd_wait, timeout_us);
    _attach_interruptI(g->pal_line,
                       palcallback_t(nullptr),
                       nullptr,
                       mode);
    osalSysUnlock();

    return msg == MSG_OK;
}

// 检查引脚号是否有效
bool GPIO::valid_pin(uint8_t pin) const
{
    if (gpio_by_pin_num(pin) != nullptr) {
        return true;
    }
#if HAL_WITH_IO_MCU
    if (AP_BoardConfig::io_enabled() && iomcu.valid_GPIO_pin(pin)) {
        return true;
    }
#endif
    return false;
}

// 返回与GPIO引脚关联的舵机通道。成功时返回true并填充servo_ch参数
// servo_ch使用从零开始的索引
bool GPIO::pin_to_servo_channel(uint8_t pin, uint8_t& servo_ch) const
{
#if HAL_WITH_IO_MCU || HAVE_GPIO_PINS
    uint8_t fmu_chan_offset = 0;
#endif
#if HAL_WITH_IO_MCU
    if (AP_BoardConfig::io_enabled()) {
        // 检查这是否是主引脚之一
        uint8_t main_servo_ch = pin;
        if (iomcu.convert_pin_number(main_servo_ch)) {
            servo_ch = main_servo_ch;
            return true;
        }
        // 使用IOMCU时,本地(FMU)通道从8开始
        fmu_chan_offset = 8;
    }
#endif

#if HAVE_GPIO_PINS
    // 在_gpio_tab中搜索匹配的引脚
    for (uint8_t i=0; i<ARRAY_SIZE(_gpio_tab); i++) {
        if (_gpio_tab[i].pin_num == pin) {
            if (_gpio_tab[i].pwm_num == 0) {
                return false;
            }
            servo_ch = _gpio_tab[i].pwm_num-1+fmu_chan_offset;
            return true;
        }
    }
#endif // HAVE_GPIO_PINS
    return false;
}

#if defined(STM32F7) || defined(STM32H7) || defined(STM32F4) || defined(STM32F3) || defined(STM32G4) || defined(STM32L4) || defined(STM32L4PLUS)

// 允许保存和恢复引脚设置
bool GPIO::get_mode(uint8_t pin, uint32_t &mode)
{
    auto *p = gpio_by_pin_num(pin);
    if (!p) {
        return false;
    }
    mode = uint32_t(palReadLineMode(p->pal_line));
    return true;
}

void GPIO::set_mode(uint8_t pin, uint32_t mode)
{
    auto *p = gpio_by_pin_num(pin);
    if (p) {
        palSetLineMode(p->pal_line, ioline_t(mode));
    }
}
#endif

#ifndef IOMCU_FW
/*
  timer to setup interrupt quotas for a 100ms period from
  monitor thread
  
  从监视线程设置100ms周期的中断配额的定时器
*/
void GPIO::timer_tick()
{
    // 允许GPIO中断源每秒最多100k次中断,即每100ms调用timer_tick()时10k次
#if HAVE_GPIO_PINS
    const uint16_t quota = 10000U;
    for (uint8_t i=0; i<ARRAY_SIZE(_gpio_tab); i++) {
        if (_gpio_tab[i].isr_quota != 1) {
            // 为下一个tick重置配额
            _gpio_tab[i].isr_quota = quota;
            continue;
        }
        // 自上次检查以来此引脚的ISR配额已用完。
        // 这不是真正的内部错误,但我们使用INTERNAL_ERROR()来获取报告机制

        if (_gpio_tab[i].isr_disabled_ticks == 0) {
#ifndef HAL_NO_UARTDRIVER
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"ISR flood on pin %u", _gpio_tab[i].pin_num);
#endif
            // 仅在解锁时触发内部错误
            if (hal.util->get_soft_armed()) {
                INTERNAL_ERROR(AP_InternalError::error_t::gpio_isr);
            }
        }
        if (hal.util->get_soft_armed()) {
            // 在解锁前不开始计数
            _gpio_tab[i].isr_disabled_ticks = 1;
            continue;
        }

        // 增加禁用计数,不要溢出
        if (_gpio_tab[i].isr_disabled_ticks < UINT8_MAX) {
            _gpio_tab[i].isr_disabled_ticks++;
        }

        // 100 * 100ms = 10秒
        const uint8_t ISR_retry_ticks = 100U;
        if ((_gpio_tab[i].isr_disabled_ticks > ISR_retry_ticks) && (_gpio_tab[i].fn != nullptr)) {
            // 尝试重新启用
#ifndef HAL_NO_UARTDRIVER
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Retrying pin %d after ISR flood", _gpio_tab[i].pin_num);
#endif
            if (attach_interrupt(_gpio_tab[i].pin_num, _gpio_tab[i].fn, _gpio_tab[i].isr_mode)) {
                // 成功,重置配额
                _gpio_tab[i].isr_quota = quota;
            } else {
                // 失败,重置禁用计数以便稍后重试
                _gpio_tab[i].isr_disabled_ticks = 1;
            }
        }
    }
#endif // HAVE_GPIO_PINS
}

// 检查ISR洪水
bool GPIO::arming_checks(size_t buflen, char *buffer) const
{
#if HAVE_GPIO_PINS
    for (uint8_t i=0; i<ARRAY_SIZE(_gpio_tab); i++) {
        if (_gpio_tab[i].isr_disabled_ticks != 0) {
            hal.util->snprintf(buffer, buflen, "Pin %u disabled (ISR flood)", _gpio_tab[i].pin_num);
            return false;
        }
    }
#endif // HAVE_GPIO_PINS
    return true;
}
#endif // IOMCU_FW
