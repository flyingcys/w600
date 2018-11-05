/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-09-15     flyingcys    add w600 wifi driver
 */
 
#ifndef __DRV_WIFI_H__
#define __DRV_WIFI_H__

#include "wm_type_def.h"
#include "wm_wifi.h"

/* Forward declarations. */
extern struct netif *tls_get_netif(void);
extern uint8_t* tls_wifi_buffer_acquire(int total_len);
extern void tls_wifi_buffer_release(rt_bool_t is_apsta, rt_uint8_t* buffer);
extern uint8_t *wpa_supplicant_get_mac(void);
extern int tls_hw_set_multicast_key(rt_uint8_t* addr);
extern int tls_hw_del_multicast_key(rt_uint8_t* addr);
#if TLS_CONFIG_AP
extern rt_uint8_t *hostapd_get_mac(void);
#endif
int ethernetif_input(const rt_uint8_t *bssid, rt_uint8_t *buf, rt_uint32_t buf_len);

int drv_wifi_init(void);
    
#endif /* __DRV_WIFI_H__ */

