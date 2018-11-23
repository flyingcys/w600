/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-22      fanwenl      1st version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "drv_pwm.h"
#include "pin_map.h"

#ifdef RT_USING_PWM

static rt_err_t wm_pwm_control(struct rt_device_pwm *device, int cmd, void *arg)
{
    struct rt_pwm_configuration *configuration = (struct rt_pwm_configuration *)arg;

    int ret = WM_FAILED;
    rt_uint32_t freq_hz = 0;
    rt_uint8_t duty = 0;
    rt_uint32_t channel = 0;
    
    channel = configuration->channel - 1;
    
    if(channel > 4)
       return RT_EINVAL;
    
    switch (cmd)
    {
    case PWM_CMD_ENABLE:
        ret = tls_pwm_start(channel);
        if(ret == WM_SUCCESS)
            return RT_EOK;
        else
            return RT_ERROR;
    case PWM_CMD_DISABLE:
        ret = tls_pwm_stop(channel);
        if(ret == WM_SUCCESS)
            return RT_EOK;
        else
            return RT_ERROR;
    case PWM_CMD_SET:
        freq_hz = 1000000 / configuration->period;
        duty = (rt_uint8_t)((configuration->pulse * 255.0) / configuration->period);
        if(freq_hz > 156250)
            return RT_ERROR;
        ret = tls_pwm_init(channel, freq_hz, duty,0);
        if(ret == WM_SUCCESS)
            return RT_EOK;
        else
            return RT_ERROR;
    case PWM_CMD_GET:
        tls_pwm_get_info(channel, (u32_t *)&freq_hz, &duty);
        if(freq_hz)
        {
            configuration->period = 1000000 / freq_hz;
            configuration->pulse = (duty * configuration->period ) / 255;
            return RT_EOK;
        }
        else
            return RT_ERROR;
    default:
        return RT_EINVAL;
    }
}

static struct rt_pwm_ops drv_ops =
{
    wm_pwm_control
};
static struct rt_device_pwm wm_pwm;
int wm_pwm_init(void)
{
    rt_int16_t gpio_pin;
    /*io config*/
#ifdef USING_PWM_CH1
    gpio_pin = wm_get_pin(WM_PWM_CH1_PIN);
    if(gpio_pin > 0)
    {
        wm_pwm1_config((enum tls_io_name)gpio_pin);
    }
#endif
#ifdef USING_PWM_CH2
    gpio_pin = wm_get_pin(WM_PWM_CH2_PIN);
    if(gpio_pin > 0)
    {
        wm_pwm2_config((enum tls_io_name)gpio_pin);
    }
#endif
#ifdef USING_PWM_CH3
    gpio_pin = wm_get_pin(WM_PWM_CH3_PIN);
    if(gpio_pin > 0)
    {
        wm_pwm3_config((enum tls_io_name)gpio_pin);
    }
#endif
#ifdef USING_PWM_CH4
    gpio_pin = wm_get_pin(WM_PWM_CH4_PIN);
    if(gpio_pin > 0)
    {
        wm_pwm4_config((enum tls_io_name)gpio_pin);
    }
#endif
#ifdef USING_PWM_CH5
    gpio_pin = wm_get_pin(WM_PWM_CH5_PIN);
    if(gpio_pin > 0)
    {
        wm_pwm5_config((enum tls_io_name)gpio_pin);
    }
#endif
#if defined(USING_PWM_CH1) || defined(USING_PWM_CH2) || defined(USING_PWM_CH3) || \
    defined(USING_PWM_CH4) || defined(USING_PWM_CH5)
    rt_device_pwm_register(&wm_pwm, "pwm", &drv_ops, 0);
#endif
    return RT_EOK;
}
INIT_DEVICE_EXPORT(wm_pwm_init);

/* pwm test cmd */
static void pwm_test(int argc, char **argv)
{
    rt_device_t pwmd = rt_device_find(argv[1]);
    struct rt_pwm_configuration cfg =
        {
            .channel = 0,  /* 0-n */
            .period = 100, /* unit:ns 1ns~4.29s:1Ghz~0.23hz */
            .pulse = 50    /* unit:ns (pulse<=period) */
        };
    for (int i = 1; i < 6; i++)
    {
        cfg.channel = i;
        rt_device_control(pwmd, PWM_CMD_SET, &cfg);
        rt_device_control(pwmd, PWM_CMD_ENABLE, &cfg);
    }
}
MSH_CMD_EXPORT(pwm_test, pwm_test pwm1);

#endif
