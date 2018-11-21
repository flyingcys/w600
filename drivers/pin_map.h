/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-21     fanwenl       first version
 */
#ifndef __PIN_MAP_H__
#define __PIN_MAP_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>
#include "wm_io.h"

rt_int16_t wm_get_pin(rt_base_t pin_index);

#endif