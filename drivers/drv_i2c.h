/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */
#ifndef __DRV_I2C_H__
#define __DRV_I2C_H__

#include <rtdevice.h>
#include "wm_i2c.h"
#include "wm_io.h"

#define I2C_HW_FREQ		(200000)
#define WM_HW_I2C_SCL   WM_IO_PB_21
#define WM_HW_I2C_SDA   WM_IO_PB_22

struct wm_i2c_bus
{
    struct rt_i2c_bus_device parent;
    struct rt_i2c_msg *msg;
    rt_uint32_t msg_cnt;
    volatile rt_uint32_t msg_ptr;
    volatile rt_uint32_t dptr;
    rt_uint32_t wait_stop;
};
#endif