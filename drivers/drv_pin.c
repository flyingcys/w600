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

#include "drv_pin.h"

static void hw_pin_mode(struct rt_device *device, rt_base_t pin, rt_base_t mode)
{

    if(mode == PIN_MODE_INPUT)
    {
//        tls_gpio_cfg(, WM_GPIO_DIR_INPUT, WM_GPIO_ATTR_FLOATING);
    }
    else if(mode == PIN_MODE_INPUT_PULLUP)
    {

    }
    else if(mode == PIN_MODE_INPUT_PULLDOWN)
    {

    }
    else if(mode == PIN_MODE_OUTPUT)
    {
//        tls_gpio_cfg(, );
    }
    return;
}

static void hw_pin_write(struct rt_device *device, rt_base_t pin, rt_base_t value)
{
    #if 0
    const struct pin_index *index;

    index = get_pin(pin);
    if(index == RT_NULL)
    {

    }
    #endif
    tls_gpio_write(WM_IO_PB_18, value);
    return;
}

static int hw_pin_read(struct rt_device *device, rt_base_t pin)
{
    int value;
    #if 0
    const struct pin_index *index;

    value = PIN_LOW;
    
    index = get_pin(pin);
    if(index == RT_NULL)
    {
        return value;
    }
    #endif
    value = tls_gpio_read(WM_IO_PB_18);
    return value;
}

static rt_err_t hw_pin_attach_irq(struct rt_device *device, rt_int32_t pin,
       rt_uint32_t mode, void (*hdr)(void *args), void *args)
{
    return RT_EOK;
}

static rt_err_t hw_pin_detach_irq(struct rt_device *device, rt_int32_t pin)
{
    return RT_EOK;
}

static rt_err_t hw_pin_irq_enable(struct rt_device *device, rt_base_t pin, rt_uint32_t enabled)
{
#if 0
    if(enabled == PIN_IRQ_ENABLE)
    {
        switch()
        {
        case :
            tls_gpio_irq_enable(gpio_pin, WM_GPIO_IRQ_TRIG_RISING_EDGE);
            break;

        case :
            break;
        }
    }

	tls_gpio_isr_register(gpio_pin, demo_gpio_isr_callback, NULL);
    #endif
    return RT_EOK;
}

struct rt_pin_ops _hw_pin_ops =
{
    hw_pin_mode,
    hw_pin_write,
    hw_pin_read,

    hw_pin_attach_irq,
    hw_pin_detach_irq,
    hw_pin_irq_enable
};

int hw_pin_init(void)
{
    int ret = rt_device_pin_register("pin", &_hw_pin_ops , RT_NULL);

    return ret;
}
INIT_BOARD_EXPORT(hw_pin_init);

