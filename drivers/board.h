/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-09-15     flyingcys    add w600
 */
 
#ifndef __BOARD_H__
#define __BOARD_H__

#include "wm_type_def.h"
#include "misc.h"
#include "wm_config.h"
#include "wm_cpu.h"
#include "wm_crypto_hard.h"
#include "wm_debug.h"
#include "rtconfig.h"

#if defined(SOC_W600_A8xx)
    #define WM60X_PIN_NUMBERS 33
#elif defined(SOC_W601_A8xx)
    #define WM60X_PIN_NUMBERS 69
#endif

#ifdef __CC_ARM
extern int Image$$RW_IRAM1$$ZI$$Limit;
#define HEAP_BEGIN    (&Image$$RW_IRAM1$$ZI$$Limit)
#elif __ICCARM__
#pragma section="HEAP"
#define HEAP_BEGIN    (__segment_end("HEAP"))
#else
extern int __bss_end;
#define HEAP_BEGIN    (&__bss_end)
#endif

#define HEAP_END           (0x20038000UL)

void rt_hw_board_init(void);

#endif
