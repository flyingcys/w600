 /*
  * Copyright (c) 2006-2018, RT-Thread Development Team
  *
  * SPDX-License-Identifier: Apache-2.0
  *
  * Change Logs:
  * Date           Author       Notes
  * 2018-09-15     flyingcys    add w600 uart drvice
  */
#include <rthw.h>
#include <rtthread.h>


#ifndef __DRV_UART_H__
#define __DRV_UART_H__

#include "wm_io.h"
#include "wm_uart.h"
#include "wm_debug.h"
#include "wm_gpio_afsel.h"

#define   WM600_UART0    (TLS_UART_REGS_T *) HR_UART0_BASE_ADDR
#define   WM600_UART1    (TLS_UART_REGS_T *) HR_UART1_BASE_ADDR
#define   WM600_UART2    (TLS_UART_REGS_T *) HR_UART2_BASE_ADDR

int rt_hw_uart_init(void);

#endif /* __DRV_UART_H__ */
