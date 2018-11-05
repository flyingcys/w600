/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */

#include <rtdevice.h>
#include <rtthread.h>
#include "drv_rtc.h"
#include <time.h>

static time_t get_timestamp(void)
{
    struct tm tm_new = {0};

	int ctrl1 = 0;
	int ctrl2 = 0;

	ctrl1 = tls_reg_read32(HR_PMU_RTC_CTRL1);
	ctrl2 = tls_reg_read32(HR_PMU_RTC_CTRL2);
	tm_new.tm_year = ((int)((int)ctrl2 & 0x00007f00) >> 8);
	tm_new.tm_mon  = (ctrl2 & 0x0000000f);
	tm_new.tm_mday = (ctrl1 & 0x1f000000) >> 24;
	tm_new.tm_hour = (ctrl1 & 0x001f0000) >> 16;
	tm_new.tm_min  = (ctrl1 & 0x00003f00) >>  8;
	tm_new.tm_sec  =  ctrl1 & 0x0000003f;
    
    return mktime(&tm_new);
}

static int set_timestamp(time_t timestamp)
{
    int ctrl1 = 0;
    int ctrl2 = 0;

	ctrl2  = tls_reg_read32(HR_PMU_RTC_CTRL2);	/* disable */
	ctrl2 &= ~(1 << 16);
	tls_reg_write32(HR_PMU_RTC_CTRL2, ctrl2);

    ctrl1 |= timestamp.tm_sec;
    ctrl1 |= timestamp.tm_min  << 8;
    ctrl1 |= timestamp.tm_hour << 16;
    ctrl1 |= timestamp.tm_mday << 24;
    tls_reg_write32(HR_PMU_RTC_CTRL1, ctrl1);

    ctrl2  = 0;
    ctrl2 |= timestamp.tm_mon;
    ctrl2 |= timestamp.tm_year << 8;
    tls_reg_write32(HR_PMU_RTC_CTRL2, ctrl2);

	ctrl2  = tls_reg_read32(HR_PMU_RTC_CTRL2);		/* enable */
	ctrl2 |= (1 << 16);
	tls_reg_write32(HR_PMU_RTC_CTRL2, ctrl2);
    
    return RT_EOK;
}

static rt_err_t hw_rtc_init(rt_device_t dev)
{

}

static rt_err_t hw_rtc_open(rt_device_t dev, rt_uint16_t oflag)
{
    tls_reg_write32(HR_PMU_RTC_CTRL1, tls_reg_read32(HR_PMU_RTC_CTRL1) | (BIT(31)));

    tls_irq_disable(PMU_RTC_INT);

    return RT_EOK;
}

static rt_err_t hw_rtc_close(rt_device_t dev)
{
    tls_reg_write32(HR_PMU_RTC_CTRL1, tls_reg_read32(HR_PMU_RTC_CTRL1) & (~BIT(31)));

    tls_irq_disable(PMU_RTC_INT);

    return RT_EOK;
}

static rt_size_t hw_rtc_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    hw_rtc_control(dev, RT_DEVICE_CTRL_RTC_GET_TIME, buffer);
    return size;
}

static rt_size_t hw_rtc_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    hw_rtc_control(dev, RT_DEVICE_CTRL_RTC_SET_TIME, buffer);
    return size;
}

static rt_err_t hw_rtc_control(rt_device_t dev, int cmd, void *args)
{
    RT_ASSERT(dev != RT_NULL);

    switch(cmd)
    {
    case RT_DEVICE_CTRL_RTC_GET_TIME:
        *(rt_uint32_t *)args = get_timestamp();
        break;

    case RT_DEVICE_CTRL_RTC_SET_TIME:
        set_timestamp(*(time_t *)args);
        break;

    default:
        return RT_EINVAL;
        break;
    }
    return RT_EOK;
}

static struct rt_device device = 
{
    .type    = RT_Device_Class_RTC, 
    .init    = hw_rtc_init, 
    .open    = hw_rtc_open, 
    .close   = hw_rtc_close, 
    .read    = hw_rtc_read,
    .write   = hw_rtc_write,
    .control = hw_rtc_control, 
    .user_data = RT_NULL,
};
    
int rt_hw_rtc_init(void)
{
    rt_err_t ret = RT_EOK;
    
    ret = rt_device_register(&device, "rtc", RT_DEVICE_FLAG_RDWR);
    if(ret != RT_EOK)
        return ret;

    rt_device_open(&device, RT_DEVICE_FLAG_RDWR);
    return;
}
INIT_DEVICE_EXPORT(rt_hw_rtc_init);
