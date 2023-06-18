/******************************************************************************
 * Copyright 2020-2021 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include <firmament.h>

#include "drv_usbd_cdc.h"
#include "hal/usb/usbd_cdc.h"
#include "module/utils/ringbuffer.h"

#include "at32f435_437.h"
#include "at32f435_437_clock.h"
#include "at_surf_f437_board_otg.h"

static struct usbd_cdc_dev usbd_dev;

void cdc_transmit_complete(void);
void cdc_receive_complete(void);

/**
  * @brief  this function handles otgfs interrupt.
  * @param  none
  * @retval none
  */
void OTG_IRQ_HANDLER(void)
{
  rt_interrupt_enter();
  usbd_irq_handler(&otg_core_struct);
  rt_interrupt_leave();
}

static rt_size_t usbd_cdc_read(usbd_cdc_dev_t usbd, rt_off_t pos, void* buf, rt_size_t size)
{
    rt_size_t rb;

    RT_ASSERT(usbd->rx_rb != NULL);

    rb = ringbuffer_get(usbd->rx_rb, buf, size);

    return rb;
}

static rt_size_t usbd_cdc_write(usbd_cdc_dev_t usbd, rt_off_t pos, const void* buf, rt_size_t size)
{
  uint32_t timeout;
  timeout = 5000000;
  do
  {
    /* send data to host */
    if(usb_vcp_send_data(&otg_core_struct.dev, buf, size) == SUCCESS)
    {
      return size;  
    }
  }while(timeout --);
  return 0;
}

void drv_usbd_cdc_disconnect_cb(void)
{
    hal_usbd_cdc_notify_status(&usbd_dev, USBD_STATUS_DISCONNECT);
}

void drv_usbd_cdc_connect_cb(void)
{
    hal_usbd_cdc_notify_status(&usbd_dev, USBD_STATUS_CONNECT);
}

/**
  * @brief  usb host cdc class transmit complete
  * @param  uhost: to the structure of usbh_core_type
  * @retval status: usb_sts_type status
  */
void cdc_transmit_complete(void)
{ 
  hal_usbd_cdc_notify_status(&usbd_dev, USBD_STATUS_TX_COMPLETE);
}

/**
  * @brief  usb host cdc class reception complete
  * @param  uhost: to the structure of usbh_core_type
  * @retval status: usb_sts_type status
  */
void cdc_receive_complete(void)
{
    if (usbd_dev.rx_rb == NULL) {
    /* usbd is not initialized */
    return;
    }
    uint16_t data_len = 0;
    data_len = usb_vcp_get_rxdata(&otg_core_struct.dev, usb_buffer);
    (void)ringbuffer_put(usbd_dev.rx_rb, usb_buffer, data_len);
    hal_usbd_cdc_notify_status(&usbd_dev, USBD_STATUS_RX);
}

struct usbd_cdc_ops usbd_ops = {
    .dev_init = NULL,
    .dev_read = usbd_cdc_read,
    .dev_write = usbd_cdc_write,
    .dev_control = NULL
};

rt_err_t drv_usb_cdc_init(void)
{
    rt_err_t err;
    usbd_dev.ops = &usbd_ops;
    otg_init();
    err = hal_usbd_cdc_register(&usbd_dev, "usbd0", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE | RT_DEVICE_FLAG_INT_RX, RT_NULL);
    if (err != RT_EOK) {
        return err;
    }
    // hal_usbd_cdc_notify_status(&usbd_dev, USBD_STATUS_CONNECT);
    return RT_EOK;
}
