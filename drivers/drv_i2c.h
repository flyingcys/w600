/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-9      fanwenl      1st version
 */
#ifndef __DRV_I2C_H__
#define __DRV_I2C_H__

#include <rtdevice.h>
#include "wm_i2c.h"
#include "wm_io.h"
#include "wm_gpio_afsel.h"

struct wm_i2c_bus
{
    struct rt_i2c_bus_device parent;
    struct rt_i2c_msg *msg;
    rt_uint32_t msg_cnt;
    volatile rt_uint32_t msg_ptr;
    volatile rt_uint32_t dptr;
    rt_uint32_t wait_stop;
};

int rt_hw_i2c_init(void);
#endif