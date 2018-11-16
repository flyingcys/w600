/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-12      fanwenl      1st version
 */

#ifndef __DRV_SPI_BUS_H__
#define __DRV_SPI_BUS_H__

#include <rtthread.h>
#include <rtdevice.h>
#include "wm_hostspi.h"
#include "wm_spi_hal.h"
#include "wm_io.h"
#include "wm_gpio_afsel.h"

#define WM_SPI_CLK      WM_IO_PB_16
#define WM_SPI_MOSI     WM_IO_PB_18
#define WM_SPI_MISO     WM_IO_PB_17

struct wm_sw_spi_cs
{
    rt_uint32_t pin;
};

int rt_hw_spi_bus_init(void);
rt_err_t wm_spi_bus_attach_device(const char *bus_name, const char *device_name, rt_uint32_t pin);

#endif
