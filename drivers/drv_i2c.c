/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-9      fanwenl      1st version
 */

#include <rtdevice.h>
#include <rtthread.h>
#include "drv_i2c.h"

#ifdef RT_USING_I2C

static struct wm_i2c_bus wm_i2c;

static rt_size_t wm_i2c_mst_xfer(struct rt_i2c_bus_device *bus,
                                        struct rt_i2c_msg msgs[],
                                               rt_uint32_t num);
static rt_size_t wm_i2c_slv_xfer(struct rt_i2c_bus_device *bus,
                                       struct rt_i2c_msg msgs[],
                                               rt_uint32_t num);
static rt_err_t wm_i2c_bus_control(struct rt_i2c_bus_device *bus,
                                                      rt_uint32_t,
                                                     rt_uint32_t);

static const struct rt_i2c_bus_device_ops wm_i2c_ops =
{
     wm_i2c_mst_xfer,
     wm_i2c_slv_xfer,
     wm_i2c_bus_control,
};


static rt_size_t wm_i2c_mst_xfer(struct rt_i2c_bus_device *bus,
                                       struct rt_i2c_msg msgs[],
                                                rt_uint32_t num)
{
    struct wm_i2c_bus *wm_i2c;
    rt_size_t i;
    uint8_t addr_msb;
    RT_ASSERT(bus != RT_NULL);
    wm_i2c = (struct wm_i2c_bus *) bus;

    wm_i2c->msg = msgs;
    wm_i2c->msg_ptr = 0;
    wm_i2c->msg_cnt = num;
    wm_i2c->dptr = 0;

    for (i = 0; i < num; i++)
    {
        if (wm_i2c->msg[i].flags & RT_I2C_RD)
        {
            if(wm_i2c->msg[i].flags & RT_I2C_ADDR_10BIT)
            {
                addr_msb = (wm_i2c->msg[i].addr >> 7) | 0xF1; 
                tls_i2c_write_byte(addr_msb, 1);   
                tls_i2c_wait_ack();
                tls_i2c_write_byte((uint8_t)wm_i2c->msg[i].addr, 1);   
                tls_i2c_wait_ack();
            }
            else
            {
                tls_i2c_write_byte((wm_i2c->msg[i].addr << 1) | wm_i2c->msg[i].flags, 1);   
                tls_i2c_wait_ack();
            }
            while(wm_i2c->msg[i].len > 1)
            {
                *wm_i2c->msg[i].buf++=tls_i2c_read_byte(1,0);	
                wm_i2c->msg[i].len--;
            }
 
               *wm_i2c->msg[i].buf = tls_i2c_read_byte(0,1);
        }
        else
        {
            if(wm_i2c->msg[i].flags & RT_I2C_ADDR_10BIT)
            {
                addr_msb = ((wm_i2c->msg[i].addr >> 7) | 0xF0) & 0xFE; 
                tls_i2c_write_byte(addr_msb, 1);   
                tls_i2c_wait_ack();
                tls_i2c_write_byte((uint8_t)wm_i2c->msg[i].addr, 1);   
                tls_i2c_wait_ack();
            }
            else
            {
                tls_i2c_write_byte((wm_i2c->msg[i].addr << 1) | wm_i2c->msg[i].flags, 1);   
                tls_i2c_wait_ack();
            }
            while(wm_i2c->msg[i].len > 0)
            {
                tls_i2c_write_byte(*wm_i2c->msg[i].buf, 0);
                tls_i2c_wait_ack();
                wm_i2c->msg[i].len--;
                wm_i2c->msg[i].buf++;
            }
            tls_i2c_stop();
        }
    }
    wm_i2c->msg = RT_NULL;
    wm_i2c->msg_ptr = 0;
    wm_i2c->msg_cnt = 0;
    wm_i2c->dptr = 0;
    
    return i;
}
static rt_size_t wm_i2c_slv_xfer(struct rt_i2c_bus_device *bus,
                                       struct rt_i2c_msg msgs[],
                                                rt_uint32_t num)
{
     return 0;
}
static rt_err_t wm_i2c_bus_control(struct rt_i2c_bus_device *bus,
                                                  rt_uint32_t cmd,
                                                  rt_uint32_t arg)
{
     return RT_ERROR;
}

void WM_I2C_IRQHandler(void)
{
    extern void I2C_IRQHandler(void);
    /* enter interrupt */
    rt_interrupt_enter();

    I2C_IRQHandler();
    /* leave interrupt */
    rt_interrupt_leave();
}

int rt_hw_i2c_init(void)
{
    wm_i2c_scl_config(WM_HW_I2C_SCL);
    wm_i2c_sda_config(WM_HW_I2C_SDA);

    tls_i2c_init(I2C_HW_FREQ);
        
    wm_i2c.parent.ops = (void *)&wm_i2c_ops;
    rt_i2c_bus_device_register(&wm_i2c.parent, "i2c");
    
    return 0;
}
INIT_DEVICE_EXPORT(rt_hw_i2c_init); 
 
#endif /*RT_USING_I2C*/