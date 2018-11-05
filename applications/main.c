/*
 * File      : main.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-09-15     flyingcys    add w600 wifi driver
 */
#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>

const unsigned int HZ = RT_TICK_PER_SECOND;


struct tls_ethif * tls_netif_get_ethif(void)
{
    rt_kprintf("===============%s %d\r\n", __FUNCTION__, __LINE__);


}

int tls_os_get_type(void)
{
    rt_kprintf("===============%s %d\r\n", __FUNCTION__, __LINE__);
    
}


err_t tls_dhcp_stop(void)
{
    rt_kprintf("===============%s %d\r\n", __FUNCTION__, __LINE__);
}

u8 tls_get_isr_count(void)
{
    rt_kprintf("===============%s %d\r\n", __FUNCTION__, __LINE__);

}

int tls_wifi_get_oneshot_flag(void)
{
//    rt_kprintf("===============%s %d\r\n", __FUNCTION__, __LINE__);
    return 0;
}

void * mem_alloc_debug(u32 size)
{
    return rt_malloc(size);
}
#if 0
void *tls_mem_alloc(uint32_t size)
{
    return rt_malloc(size);
}
#endif
void mem_free_debug(void *p)
{
    rt_free(p);
}

#if 0
void tls_mem_free(void *p)
{
    rt_free(p);
}

void *MEMCPY(void *dst, const void *src, rt_ubase_t count)
{
    return memcpy(dst, src, count);
}
#endif

int main(void)
{
    /* user app entry */
    rt_kprintf("build time: %s %s\n", __DATE__, __TIME__);
    
    rt_kprintf("Hello RT-Thread!\n");
    rt_wlan_set_mode(RT_WLAN_DEVICE_STA_NAME, RT_WLAN_STATION);
    
    while (1)
    {
    	rt_thread_delay(RT_TICK_PER_SECOND);
    }    
    return 0;
}