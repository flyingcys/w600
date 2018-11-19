/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>

#include "board.h"
#include "drv_pin.h"

#ifdef RT_USING_PIN
#define __WM_PIN(index, gpio_index) (gpio_index)
#define __WM_PIN_DEFAULT -1

/* WM chip GPIO map*/
static const rt_int16_t pins[] =
{
    __WM_PIN(0, __WM_PIN_DEFAULT),
#if (WM60X_PIN_NUMBERS == 33)
        __WM_PIN(1, __WM_PIN_DEFAULT),
        __WM_PIN(2, __WM_PIN_DEFAULT),
        __WM_PIN(3, __WM_PIN_DEFAULT),
        __WM_PIN(4, __WM_PIN_DEFAULT),
        __WM_PIN(5, __WM_PIN_DEFAULT),
        __WM_PIN(6, __WM_PIN_DEFAULT),
        __WM_PIN(7, __WM_PIN_DEFAULT),
        __WM_PIN(8, __WM_PIN_DEFAULT),
        __WM_PIN(9, __WM_PIN_DEFAULT),
        __WM_PIN(10, __WM_PIN_DEFAULT),
        __WM_PIN(11, __WM_PIN_DEFAULT),
        __WM_PIN(12, __WM_PIN_DEFAULT),
        __WM_PIN(13, WM_IO_PA_00),
        __WM_PIN(14, WM_IO_PA_01),
        __WM_PIN(15, WM_IO_PA_04),
        __WM_PIN(16, __WM_PIN_DEFAULT),
        __WM_PIN(17, WM_IO_PA_05),
        __WM_PIN(18, WM_IO_PB_13),
        __WM_PIN(19, WM_IO_PB_14),
        __WM_PIN(20, WM_IO_PB_15),
        __WM_PIN(21, WM_IO_PB_16),
        __WM_PIN(22, WM_IO_PB_17),
        __WM_PIN(23, WM_IO_PB_18),
        __WM_PIN(24, __WM_PIN_DEFAULT),
        __WM_PIN(25, __WM_PIN_DEFAULT),
        __WM_PIN(26, WM_IO_PB_06),
        __WM_PIN(27, WM_IO_PB_07),
        __WM_PIN(28, WM_IO_PB_08),
        __WM_PIN(29, WM_IO_PB_09),
        __WM_PIN(30, WM_IO_PB_10),
        __WM_PIN(31, WM_IO_PB_11),
        __WM_PIN(32, WM_IO_PB_12),
        __WM_PIN(33, __WM_PIN_DEFAULT),
#elif(WM60X_PIN_NUMBERS == 69)
        __WM_PIN(1, WM_IO_PB_19),
        __WM_PIN(2, WM_IO_PB_10),
        __WM_PIN(3, WM_IO_PB_21),
        __WM_PIN(4, WM_IO_PB_22),
        __WM_PIN(5, WM_IO_PB_23),
        __WM_PIN(6, WM_IO_PB_24),
        __WM_PIN(7, WM_IO_PB_25),
        __WM_PIN(8, WM_IO_PB_26),
        __WM_PIN(9, __WM_PIN_DEFAULT),
        __WM_PIN(10, __WM_PIN_DEFAULT),
        __WM_PIN(11, __WM_PIN_DEFAULT),
        __WM_PIN(12, __WM_PIN_DEFAULT),
        __WM_PIN(13, __WM_PIN_DEFAULT),
        __WM_PIN(14, __WM_PIN_DEFAULT),
        __WM_PIN(15, __WM_PIN_DEFAULT),
        __WM_PIN(16, __WM_PIN_DEFAULT),
        __WM_PIN(17, __WM_PIN_DEFAULT),
        __WM_PIN(18, __WM_PIN_DEFAULT),
        __WM_PIN(19, __WM_PIN_DEFAULT),
        __WM_PIN(20, __WM_PIN_DEFAULT),
        __WM_PIN(21, __WM_PIN_DEFAULT),
        __WM_PIN(22, __WM_PIN_DEFAULT),
        __WM_PIN(23, WM_IO_PA_00),
        __WM_PIN(24, WM_IO_PA_01),
        __WM_PIN(25, WM_IO_PA_02),
        __WM_PIN(26, WM_IO_PA_03),
        __WM_PIN(27, WM_IO_PA_04),
        __WM_PIN(28, __WM_PIN_DEFAULT),
        __WM_PIN(29, WM_IO_PA_05),
        __WM_PIN(30, WM_IO_PA_13),
        __WM_PIN(31, WM_IO_PA_14),
        __WM_PIN(32, WM_IO_PA_15),
        __WM_PIN(33, WM_IO_PA_06),
        __WM_PIN(34, __WM_PIN_DEFAULT),
        __WM_PIN(35, WM_IO_PA_07),
        __WM_PIN(36, WM_IO_PA_08),
        __WM_PIN(37, WM_IO_PA_09),
        __WM_PIN(38, WM_IO_PA_10),
        __WM_PIN(39, WM_IO_PA_11),
        __WM_PIN(40, WM_IO_PA_12),
        __WM_PIN(41, WM_IO_PB_28),
        __WM_PIN(42, __WM_PIN_DEFAULT),
        __WM_PIN(43, WM_IO_PB_13),
        __WM_PIN(44, WM_IO_PB_14),
        __WM_PIN(45, WM_IO_PB_15),
        __WM_PIN(46, __WM_PIN_DEFAULT),
        __WM_PIN(47, WM_IO_PB_16),
        __WM_PIN(48, WM_IO_PB_17),
        __WM_PIN(49, WM_IO_PB_18),
        __WM_PIN(50, __WM_PIN_DEFAULT),
        __WM_PIN(51, WM_IO_PB_30),
        __WM_PIN(52, WM_IO_PB_31),
        __WM_PIN(53, WM_IO_PB_27),
        __WM_PIN(54, WM_IO_PB_00),
        __WM_PIN(55, WM_IO_PB_01),
        __WM_PIN(56, WM_IO_PB_02),
        __WM_PIN(57, WM_IO_PB_03),
        __WM_PIN(58, __WM_PIN_DEFAULT),
        __WM_PIN(59, WM_IO_PB_04),
        __WM_PIN(60, WM_IO_PB_05),
        __WM_PIN(61, WM_IO_PB_06),
        __WM_PIN(62, WM_IO_PB_07),
        __WM_PIN(63, WM_IO_PB_08),
        __WM_PIN(64, WM_IO_PB_09),
        __WM_PIN(65, WM_IO_PB_10),
        __WM_PIN(66, WM_IO_PB_11),
        __WM_PIN(67, WM_IO_PB_12),
        __WM_PIN(68, __WM_PIN_DEFAULT),
        __WM_PIN(69, __WM_PIN_DEFAULT),
#endif
};
 
#define WM_PIN_NUM(items) (sizeof(items) / sizeof(items[0]))
static rt_int16_t wm_get_pin(rt_base_t pin_index)
{
    rt_int16_t gpio_pin = __WM_PIN_DEFAULT;
    if (pin_index < WM_PIN_NUM(pins))
    {
        gpio_pin = pins[pin_index];
    }
		rt_kprintf("pin index:%d\r\n", gpio_pin);
    return gpio_pin;
}

static void wm_pin_mode(struct rt_device *device, rt_base_t pin, rt_base_t mode)
{
    rt_int16_t gpio_pin;
    gpio_pin = wm_get_pin(pin);
    if (gpio_pin == __WM_PIN_DEFAULT)
    {
        return;
    }
    if (mode == PIN_MODE_INPUT)
    {
        tls_gpio_cfg((enum tls_io_name)gpio_pin, WM_GPIO_DIR_INPUT, WM_GPIO_ATTR_FLOATING);
    }
    else if (mode == PIN_MODE_INPUT_PULLUP)
    {
        tls_gpio_cfg((enum tls_io_name)gpio_pin, WM_GPIO_DIR_INPUT, WM_GPIO_ATTR_PULLHIGH);
    }
    else if (mode == PIN_MODE_INPUT_PULLDOWN)
    {
        tls_gpio_cfg((enum tls_io_name)gpio_pin, WM_GPIO_DIR_INPUT, WM_GPIO_ATTR_PULLLOW);
    }
    else if (mode == PIN_MODE_OUTPUT)
    {
				rt_kprintf("pin mode index:%d\r\n", gpio_pin);
        tls_gpio_cfg((enum tls_io_name)gpio_pin, WM_GPIO_DIR_OUTPUT, WM_GPIO_ATTR_PULLHIGH);
    }
    return;
}

static void wm_pin_write(struct rt_device *device, rt_base_t pin, rt_base_t value)
{
    rt_int16_t gpio_pin;
    gpio_pin = wm_get_pin(pin);
    if (gpio_pin == __WM_PIN_DEFAULT)
    {
        return;
    }
    tls_gpio_write((enum tls_io_name)gpio_pin, value);
		rt_kprintf("pin write index:%d\r\n", gpio_pin);
    return;
}

static int wm_pin_read(struct rt_device *device, rt_base_t pin)
{
    rt_int16_t gpio_pin;
    gpio_pin = wm_get_pin(pin);
    if (gpio_pin == __WM_PIN_DEFAULT)
    {
        return PIN_LOW;
    }
		rt_kprintf("pin read index:%d\r\n", gpio_pin);
    return tls_gpio_read((enum tls_io_name)gpio_pin);
}

static rt_err_t wm_pin_attach_irq(struct rt_device *device, rt_int32_t pin,
                                  rt_uint32_t mode, void (*hdr)(void *args), void *args)
{
    rt_int16_t gpio_pin;
    rt_base_t level;

    gpio_pin = wm_get_pin(pin);
    if (gpio_pin == __WM_PIN_DEFAULT)
    {
        return RT_ENOSYS;
    }

    level = rt_hw_interrupt_disable();
    /*irq mode set*/
    switch (mode)
    {
    case PIN_IRQ_MODE_RISING:
        tls_gpio_irq_cfg((enum tls_io_name)gpio_pin, WM_GPIO_IRQ_TRIG_RISING_EDGE);
        break;
    case PIN_IRQ_MODE_FALLING:
        tls_gpio_irq_cfg((enum tls_io_name)gpio_pin, WM_GPIO_IRQ_TRIG_FALLING_EDGE);
        break;
    case PIN_IRQ_MODE_RISING_FALLING:
        tls_gpio_irq_cfg((enum tls_io_name)gpio_pin, WM_GPIO_IRQ_TRIG_DOUBLE_EDGE);
        break;
    case PIN_IRQ_MODE_HIGH_LEVEL:
        tls_gpio_irq_cfg((enum tls_io_name)gpio_pin, WM_GPIO_IRQ_TRIG_HIGH_LEVEL);
        break;
    case PIN_IRQ_MODE_LOW_LEVEL:
        tls_gpio_irq_cfg((enum tls_io_name)gpio_pin, WM_GPIO_IRQ_TRIG_LOW_LEVEL);
        break;
    default:
        rt_hw_interrupt_enable(level);
        return RT_ENOSYS;
    }

    tls_gpio_isr_register((enum tls_io_name)gpio_pin, hdr, args);
    rt_hw_interrupt_enable(level);
    return RT_EOK;
}

static rt_err_t wm_pin_detach_irq(struct rt_device *device, rt_int32_t pin)
{
    return RT_EOK;
}

static rt_err_t wm_pin_irq_enable(struct rt_device *device, rt_base_t pin, rt_uint32_t enabled)
{
    rt_int16_t gpio_pin;
    rt_base_t level;
    gpio_pin = wm_get_pin(pin);
    if (gpio_pin == __WM_PIN_DEFAULT)
    {
        return RT_ENOSYS;
    }
    level = rt_hw_interrupt_disable();
    if (enabled == PIN_IRQ_ENABLE)
    {
        tls_gpio_irq_enable((enum tls_io_name)gpio_pin);
        rt_hw_interrupt_enable(level);
        return RT_EOK;
    }
    else if (enabled == PIN_IRQ_DISABLE)
    {
        tls_gpio_irq_disable((enum tls_io_name)gpio_pin);
        rt_hw_interrupt_enable(level);
        return RT_EOK;
    }
    else
    {
        rt_hw_interrupt_enable(level);
        return RT_ENOSYS;
    }
}

struct rt_pin_ops _wm_pin_ops =
    {
        wm_pin_mode,
        wm_pin_write,
        wm_pin_read,
        wm_pin_attach_irq,
        wm_pin_detach_irq,
        wm_pin_irq_enable};

int wm_pin_init(void)
{
    int ret = rt_device_pin_register("pin", &_wm_pin_ops, RT_NULL);
    return ret;
}
INIT_BOARD_EXPORT(wm_pin_init);

void WM_GPIOA_IRQHandler(void)
{
    rt_interrupt_enter();
    GPIOA_IRQHandler();
    rt_interrupt_leave();
}

void WM_GPIOB_IRQHandler(void)
{
    rt_interrupt_enter();
    GPIOB_IRQHandler();
    rt_interrupt_leave();
}
#endif
