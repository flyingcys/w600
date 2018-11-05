/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-09-15     flyingcys    add w600 wifi driver
 */
 
#include <rtthread.h>
#include "drv_wifi.h"
#include <wlan_dev.h>

#define DBG_ENABLE
#define DBG_LEVEL DBG_LOG
#define DBG_SECTION_NAME  "WIFI"
#define DBG_COLOR
#include <rtdbg.h>

#include "wm_ram_config.h"
#include "wm_debug.h"
#include "lwip/ip_addr.h"
//#define ETH_RX_DUMP
//#define ETH_TX_DUMP
//#define MINI_DUMP

#define     MAX_ADDR_LEN        6

struct drv_wifi
{
    struct rt_wlan_device *wlan;

    rt_uint8_t dev_addr[MAX_ADDR_LEN];
    
};

static const struct rt_wlan_dev_ops ops;

static struct drv_wifi wifi_sta;
static struct drv_wifi wifi_ap;

#if defined(ETH_RX_DUMP) ||  defined(ETH_TX_DUMP)
static void packet_dump(const char *msg, const void *ptr, rt_uint32_t len)
{
    rt_uint32_t j;
    rt_uint8_t *p = (rt_uint8_t *)ptr;
    rt_kprintf("%s %d byte\n", msg, len);

#ifdef MINI_DUMP
	return;
#endif

    for (j = 0; j < len; j++)
    {
        if ((j % 8) == 0)
        {
            rt_kprintf("  ");
        }
        
        if ((j % 16) == 0)
        {
            rt_kprintf("\r\n");
        }
        rt_kprintf("%02x ", *p ++);
    }

    rt_kprintf("\n\n");
}
#endif /* dump */

static void wm_wlan_status_changed(rt_uint8_t status)
{
    rt_kprintf("status:%d\r\n", status);
    switch(status)
    {
        case WIFI_JOIN_SUCCESS:             
            rt_kprintf("NETIF_WIFI_JOIN_SUCCESS\r\n");
            rt_wlan_dev_indicate_event_handle(wifi_sta.wlan, RT_WLAN_DEV_EVT_CONNECT, RT_NULL);
            break;
        
        case WIFI_JOIN_FAILED:
            rt_kprintf("NETIF_WIFI_JOIN_FAILED\r\n");
            rt_wlan_dev_indicate_event_handle(wifi_sta.wlan, RT_WLAN_DEV_EVT_CONNECT_FAIL, RT_NULL);
            break;
            
        case WIFI_DISCONNECTED:
            rt_kprintf("NETIF_WIFI_DISCONNECTED\r\n");
            rt_wlan_dev_indicate_event_handle(wifi_sta.wlan, RT_WLAN_DEV_EVT_DISCONNECT, RT_NULL);
            break;
        
        case WIFI_SOFTAP_SUCCESS:
            rt_kprintf("WIFI_SOFTAP_SUCCESS\r\n");
//            rt_wlan_dev_indicate_event_handle(wifi_sta.wlan, RT_WLAN_DEV_EVT_AP_START, RT_NULL);
            break;
            
        case WIFI_SOFTAP_FAILED:
            rt_kprintf("WIFI_SOFTAP_FAILED\r\n");
//            rt_wlan_dev_indicate_event_handle(wifi_sta.wlan, RT_WLAN_DEV_EVT_AP_ASSOCIATE_FAILED, RT_NULL);
            break;
            
        case WIFI_SOFTAP_CLOSED:
            rt_kprintf("WIFI_SOFTAP_CLOSED\r\n");
//            rt_wlan_dev_indicate_event_handle(wifi_sta.wlan, RT_WLAN_DEV_EVT_DISCONNECT, RT_NULL);
            break;

        default:
            break;
    }
}

static rt_err_t wm_wlan_init(void)
{
    extern rt_uint8_t tx_gain_group[];
    rt_uint8_t mac_addr[6] = {0x00,0x25,0x08,0x09,0x01,0x0F};
    
    /*PARAM GAIN,MAC default*/
	tls_ft_param_init();
    tls_param_load_factory_default();
    tls_param_init(); /*add param to init sysparam_lock sem*/
    
    tls_get_tx_gain(&tx_gain_group[0]);
    TLS_DBGPRT_INFO("tx gain ");
    TLS_DBGPRT_DUMP((char *)(&tx_gain_group[0]), 12);
    TLS_DBGPRT_INFO("mac addr ");
    TLS_DBGPRT_DUMP((char *)(&mac_addr[0]), 6);
    
    if(tls_wifi_mem_cfg(WIFI_MEM_START_ADDR, 7, 7)) /*wifi tx&rx mem customized interface*/
    {
        TLS_DBGPRT_INFO("wl mem initial failured\n");
    }
    
    tls_get_mac_addr(&mac_addr[0]);
    if(tls_wl_init(NULL, &mac_addr[0], NULL) == NULL){
        TLS_DBGPRT_INFO("wl driver initial failured\n");
    }
    if (wpa_supplicant_init(mac_addr)) {
        TLS_DBGPRT_INFO("supplicant initial failured\n");
    }
    memcpy(wifi_sta.dev_addr, wpa_supplicant_get_mac(), MAX_ADDR_LEN);
    rt_kprintf("mac:%02x-%02x-%02x-%02x-%02x-%02x\r\n", wifi_sta.dev_addr[0], wifi_sta.dev_addr[1], wifi_sta.dev_addr[2], wifi_sta.dev_addr[3], wifi_sta.dev_addr[4] ,wifi_sta.dev_addr[5]);
    return RT_EOK;
}

static void wm_wlan_promisc_dataframe_callback(u8* data, u32 data_len)
{

}

rt_err_t rthw_wlan_init(struct rt_wlan_device *wlan)
{
    wm_wlan_init();
    
    tls_ethernet_data_rx_callback(ethernetif_input);
    tls_wifi_status_change_cb_register(wm_wlan_status_changed);

    return RT_EOK;
}

rt_err_t rthw_wlan_mode(struct rt_wlan_device *wlan, rt_wlan_mode_t mode)
{
    LOG_D("F:%s L:%d", __FUNCTION__, __LINE__);

    return RT_EOK;
}

static rt_wlan_security_t wm_wlan_security_map()
{
}

void wm_wlan_scan_callback(void)
{
    int buflen = 2000;
    char *buf = RT_NULL;
    int err;

    struct rt_wlan_info wlan_info = {0};
    struct rt_wlan_buff buff;
    
    buf = rt_malloc(buflen);
    if(RT_NULL == buf)
    {
        LOG_E("rt_malloc failed...\r\n");
        return;
    }
    
    err = tls_wifi_get_scan_rslt((u8 *)buf, buflen);
    if(err != WM_SUCCESS)
    {
        rt_free(buf);
        return;
    }

    struct tls_scan_bss_t *scan_res = (struct tls_scan_bss_t *)buf;
    struct tls_bss_info_t *bss_info = (struct tls_bss_info_t *)scan_res->bss;
    if(scan_res->count <= 0)
    {
        return;
    }

    int i;
    for(i = 0; i < scan_res->count; i ++)
    {
        memset(&wlan_info, 0, sizeof(wlan_info));
        
        memcpy(&wlan_info.bssid[0], bss_info->bssid, 6);
        memcpy(wlan_info.ssid.val, bss_info->ssid, bss_info->ssid_len);
        wlan_info.ssid.len = bss_info->ssid_len;
        
        wlan_info.channel = (rt_int16_t)bss_info->channel;
        wlan_info.rssi = (rt_int16_t)bss_info->rssi;

        bss_info ++;
        
        buff.data = &wlan_info;
        buff.len = sizeof(wlan_info);
        rt_wlan_dev_indicate_event_handle(wifi_sta.wlan, RT_WLAN_DEV_EVT_SCAN_REPORT, &buff);
        
    }
    
    rt_wlan_dev_indicate_event_handle(wifi_sta.wlan, RT_WLAN_DEV_EVT_SCAN_DONE, RT_NULL);
}

rt_err_t rthw_wlan_scan(struct rt_wlan_device *wlan, struct rt_scan_info *scan_info)
{
    int ret;
    
    LOG_D("F:%s L:%d", __FUNCTION__, __LINE__);
    /* register scan complt callback*/
    tls_wifi_scan_result_cb_register(wm_wlan_scan_callback);
    
    /* trigger the scan */
    ret = tls_wifi_scan();
    if((ret == WM_WIFI_SCANNING_BUSY) || (ret == WM_FAILED))
        return -RT_ERROR;
    
    return RT_EOK;
}

rt_err_t rthw_wlan_join(struct rt_wlan_device *wlan, struct rt_sta_info *sta_info)
{
    LOG_D("F:%s L:%d", __FUNCTION__, __LINE__);

	tls_wifi_disconnect();

//	tls_wifi_connect((u8 *)ssid, strlen(ssid), (u8 *)pwd, strlen(pwd));

    tls_wifi_connect((u8 *)sta_info->ssid.val, sta_info->ssid.len, (u8 *)sta_info->key.val, sta_info->key.len);
//return tls_wifi_connect_by_bssid(bssid, pwd, (pwd == NULL) ? 0 : strlen((char *)pwd));
//    ret = tls_wifi_connect_by_ssid_bssid((u8*)result.ssid, result.ssid_length, ak_bssid, (u8*)result.pwd, result.pwd_length);

    return RT_EOK;
}

rt_err_t rthw_wlan_softap(struct rt_wlan_device *wlan, struct rt_ap_info *ap_info)
{
    LOG_D("F:%s L:%d", __FUNCTION__, __LINE__);


    return RT_EOK;
}

rt_err_t rthw_wlan_disconnect(struct rt_wlan_device *wlan)
{
    LOG_D("F:%s L:%d", __FUNCTION__, __LINE__);

	tls_wifi_disconnect();
    
    return RT_EOK;
}

rt_err_t rthw_wlan_ap_stop(struct rt_wlan_device *wlan)
{
    LOG_D("F:%s L:%d", __FUNCTION__, __LINE__);

    return RT_EOK;
}

rt_err_t rthw_wlan_ap_deauth(struct rt_wlan_device *wlan, rt_uint8_t mac[])
{
    LOG_D("F:%s L:%d", __FUNCTION__, __LINE__);

    return RT_EOK;
}

rt_err_t rthw_wlan_scan_stop(struct rt_wlan_device *wlan)
{
    LOG_D("F:%s L:%d", __FUNCTION__, __LINE__);

    return RT_EOK;
}

int rthw_wlan_get_rssi(struct rt_wlan_device *wlan)
{
    LOG_D("F:%s L:%d", __FUNCTION__, __LINE__);

    return RT_EOK;
}

rt_err_t rthw_wlan_set_powersave(struct rt_wlan_device *wlan, int level)
{
    LOG_D("F:%s L:%d", __FUNCTION__, __LINE__);

    return RT_EOK;
}

int rthw_wlan_get_powersave(struct rt_wlan_device *wlan)
{
    LOG_D("F:%s L:%d", __FUNCTION__, __LINE__);

    return RT_EOK;
}


rt_err_t rthw_wlan_cfg_promisc(struct rt_wlan_device *wlan, rt_bool_t start)
{
    LOG_D("F:%s L:%d", __FUNCTION__, __LINE__);

    if(RT_TRUE == start)
    {
        tls_wifi_data_recv_cb_register(wm_wlan_promisc_dataframe_callback);
    }
    else
    {
        tls_wifi_data_recv_cb_register(RT_NULL);
    }
    
    return RT_EOK;
}

rt_err_t rthw_wlan_cfg_filter(struct rt_wlan_device *wlan, struct rt_wlan_filter *filter)
{
    LOG_D("F:%s L:%d", __FUNCTION__, __LINE__);

    return RT_EOK;
}

rt_err_t rthw_wlan_set_channel(struct rt_wlan_device *wlan, int channel)
{
    LOG_D("F:%s L:%d", __FUNCTION__, __LINE__);

    tls_wifi_change_chanel(channel);
    
    return RT_EOK;
}

int rthw_wlan_get_channel(struct rt_wlan_device *wlan)
{
    LOG_D("F:%s L:%d", __FUNCTION__, __LINE__);

    return RT_EOK;
}

rt_err_t rthw_wlan_set_country(struct rt_wlan_device *wlan, rt_country_code_t country_code)
{
    LOG_D("F:%s L:%d", __FUNCTION__, __LINE__);

    return RT_EOK;
}

rt_country_code_t rthw_wlan_get_country(struct rt_wlan_device *wlan)
{
    LOG_D("F:%s L:%d", __FUNCTION__, __LINE__);

    return RT_EOK;
}

rt_err_t rthw_wlan_set_mac(struct rt_wlan_device *wlan, rt_uint8_t mac[])
{
    LOG_D("F:%s L:%d", __FUNCTION__, __LINE__);
    

    return RT_EOK;
}

rt_err_t rthw_wlan_get_mac(struct rt_wlan_device *wlan, rt_uint8_t mac[])
{
    LOG_D("F:%s L:%d", __FUNCTION__, __LINE__);

    memcpy(mac, wpa_supplicant_get_mac(), MAX_ADDR_LEN);
    
    return RT_EOK;
}

int rthw_wlan_recv(struct rt_wlan_device *wlan, void *buff, int len)
{
    return RT_EOK;
}

int rthw_wlan_send(struct rt_wlan_device *wlan, void *buff, int len)
{
    struct pbuf *q = NULL;
    int datalen = 0;
    rt_uint8_t *dst = tls_wifi_buffer_acquire(len);
    if(dst == NULL)
        return -RT_ENOMEM;
    
#if ETH_PAD_SIZE
    pbuf_header(p, -ETH_PAD_SIZE);    /* Drop the padding word */
#endif
        
#if defined(ETH_TX_DUMP)
    packet_dump("Tx", buff, len);
#endif

    MEMCPY(dst, buff, len);

#if TLS_CONFIG_AP
    if (netif != tls_get_netif())
        tls_wifi_buffer_release(true, dst);
    else
#endif
        tls_wifi_buffer_release(false, dst);
    
#if ETH_PAD_SIZE
    pbuf_header(p, ETH_PAD_SIZE);    /* Reclaim the padding word */
#endif

    return RT_EOK;
}

static const struct rt_wlan_dev_ops ops = 
{
    .wlan_init              = rthw_wlan_init,
    .wlan_mode              = rthw_wlan_mode,
    .wlan_scan              = rthw_wlan_scan,
    .wlan_join              = rthw_wlan_join,
    .wlan_softap            = rthw_wlan_softap,
    .wlan_disconnect        = rthw_wlan_disconnect,
    .wlan_ap_stop           = rthw_wlan_ap_stop,
    .wlan_ap_deauth         = rthw_wlan_ap_deauth,
    .wlan_scan_stop         = rthw_wlan_scan_stop,
    .wlan_get_rssi          = rthw_wlan_get_rssi,
    .wlan_set_powersave     = rthw_wlan_set_powersave,
    .wlan_get_powersave     = rthw_wlan_get_powersave,
    .wlan_cfg_promisc       = rthw_wlan_cfg_promisc,
    .wlan_cfg_filter        = rthw_wlan_cfg_filter,
    .wlan_set_channel       = rthw_wlan_set_channel,
    .wlan_get_channel       = rthw_wlan_get_channel,
    .wlan_set_country       = rthw_wlan_set_country,
    .wlan_get_country       = rthw_wlan_get_country,
    .wlan_set_mac           = rthw_wlan_set_mac,
    .wlan_get_mac           = rthw_wlan_get_mac,
    .wlan_recv              = rthw_wlan_recv,
    .wlan_send              = rthw_wlan_send,
};

int drv_wifi_init(void)
{
    struct rt_wlan_device *wlan;

    memset(&wifi_sta, 0, sizeof(wifi_sta));
    wlan = rt_wlan_dev_register(RT_WLAN_DEVICE_STA_NAME, &ops, 0, &wifi_sta);
    wifi_sta.wlan = wlan;

    return RT_EOK;
}
INIT_DEVICE_EXPORT(drv_wifi_init);

err_t tls_netif_set_addr(ip_addr_t *ipaddr,
                        ip_addr_t *netmask,
                        ip_addr_t *gw)
{
    rt_kprintf("%s %d\r\n", __FUNCTION__, __LINE__);
    return 0;
}

int ethernetif_input(const rt_uint8_t *bssid, rt_uint8_t *buf, rt_uint32_t buf_len)
{
    if(rt_wlan_dev_report_data(wifi_sta.wlan, (void *)buf, buf_len) == RT_EOK)
        return 0;

    return -1;
}

