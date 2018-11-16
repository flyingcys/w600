/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-09-15     flyingcys    add w600
 */
 
#include <stdint.h>
#include <rthw.h>
#include <rtthread.h>

#include "board.h"
#include "drv_uart.h"


#define FW_MAJOR_VER           0x03
#define FW_MINOR_VER           0x00
#define FW_PATCH_VER           0x00

const char FirmWareVer[4] = {
	'G',
	FW_MAJOR_VER,  /* Main version */
	FW_MINOR_VER, /* Subversion */
	FW_PATCH_VER  /* Internal version */
	};
const char HwVer[6] = {
	'H',
	0x1,
	0x0,
	0x0,
	0x0,
	0x0
};

void disp_version_info(void)
{
    extern const char WiFiVer[];
    TLS_DBGPRT_INFO("\n\n");
    TLS_DBGPRT_INFO("****************************************************************\n");
    TLS_DBGPRT_INFO("*                                                              *\n");
    TLS_DBGPRT_INFO("* Copyright (C) 2014 WinnerMicro Co. Ltd.                      *\n");
    TLS_DBGPRT_INFO("* All rights reserved.                                         *\n");
    TLS_DBGPRT_INFO("* WinnerMicro Firmware Version: %x.%x.%X                         *\n",
           FirmWareVer[1], FirmWareVer[2], FirmWareVer[3]);
    TLS_DBGPRT_INFO("* WinnerMicro Hardware Version: %x.%x.%x.%x.%x                      *\n",
           HwVer[1], HwVer[2], HwVer[3],HwVer[4],HwVer[5]);
    TLS_DBGPRT_INFO("*                                                              *\n");
    TLS_DBGPRT_INFO("* WinnerMicro Wi-Fi Lib Version: %x.%x.%x                         *\n",
           WiFiVer[0], WiFiVer[1], WiFiVer[2]);
    TLS_DBGPRT_INFO("****************************************************************\n");
}


void wm_sys_clk_config(void)
{
    tls_sys_clk sysclk;

    tls_sys_clk_set(CPU_CLK_80M);
    tls_sys_clk_get(&sysclk);
    SysTick_Config(sysclk.cpuclk * UNIT_MHZ / RT_TICK_PER_SECOND);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
}

void wm_gpio_config(void)
{
	/* must call first */
	wm_gpio_af_disable();	

	/* UART0_TX-PA04 UART0_RX-PA05 */
	wm_uart0_tx_config(WM_IO_PA_04);
	wm_uart0_rx_config(WM_IO_PA_05);

	/* UART1_RX-PB11  UART1_TX-PB12 */	
	wm_uart1_rx_config(WM_IO_PB_11);
	wm_uart1_tx_config(WM_IO_PB_12);	

	/*MASTER SPI configuratioin*/
//	wm_spi_cs_config(WM_IO_PB_15);
//	wm_spi_ck_config(WM_IO_PB_16);
//	wm_spi_di_config(WM_IO_PB_17);
//	wm_spi_do_config(WM_IO_PB_18);
}

void wm6_peripheral_init(void)
{
#if (TLS_CONFIG_LS_SPI)	
    //tls_spi_init();
    //tls_spifls_init();
#endif
}

/**
 * This is the timer interrupt service routine.
 *
 */
void OS_CPU_SysTickHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}

/**
 * This function will initial board.
 */
void rt_hw_board_init(void)
{
    wm_sys_clk_config();
    
#ifdef RT_USING_HEAP
    rt_system_heap_init((void*)HEAP_BEGIN, (void*)HEAP_END);
#endif

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

#ifdef RT_USING_CONSOLE
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif

	/* must call first to configure gpio Alternate functions according the hardware design */
	wm_gpio_config();

#if TLS_CONFIG_HARD_CRYPTO
    tls_crypto_init();
#endif

    tls_fls_init();
}

/*@}*/
