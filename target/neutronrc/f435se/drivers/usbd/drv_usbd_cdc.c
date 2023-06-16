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

// #include "cdc_acm_core.h"
// #include "drv_usbd_int.h"

// #include "at32f435_437_board.h"
// #include "at32f435_437_clock.h"
#include "usb_conf.h"
#include "usb_core.h"
#include "usbh_int.h"
#include "usbh_user.h"
#include "usbh_cdc_class.h"

static struct usbd_cdc_dev usbd_dev;
/* usb global struct define */
otg_core_type otg_core_struct;
uint32_t rx_data[512];

/**
  * @brief  this function handles otgfs interrupt.
  * @param  none
  * @retval none
  */
void OTG_IRQ_HANDLER(void)
{
  rt_interrupt_enter();
  usbh_irq_handler(&otg_core_struct);
  rt_interrupt_leave();
}

/**
  * @brief  usb delay millisecond function.
  * @param  ms: number of millisecond delay
  * @retval none
  */
void usb_delay_ms(uint32_t ms)
{
  /* user can define self delay function */
  delay_ms(ms);
}

/**
  * @brief  usb delay microsecond function.
  * @param  us: number of microsecond delay
  * @retval none
  */
void usb_delay_us(uint32_t us)
{
  delay_us(us);
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
    cdc_start_transmission(&otg_core_struct.host, (uint8_t *)buf, size);
    return size;
}

/**
  * @brief  usb host cdc class transmit complete
  * @param  uhost: to the structure of usbh_core_type
  * @retval status: usb_sts_type status
  */
void cdc_transmit_complete(usbh_core_type *uhost)
{ 
  hal_usbd_cdc_notify_status(&usbd_dev, USBD_STATUS_TX_COMPLETE);
}

/**
  * @brief  usb host cdc class reception complete
  * @param  uhost: to the structure of usbh_core_type
  * @retval status: usb_sts_type status
  */
void cdc_receive_complete(usbh_core_type *uhost)
{
    usbh_core_type *puhost = (usbh_core_type *)uhost;
    usbh_cdc_type *pcdc = (usbh_cdc_type *)puhost->class_handler->pdata;
    if (usbd_dev.rx_rb == NULL) {
    /* usbd is not initialized */
    return;
    }
    (void)ringbuffer_put(usbd_dev.rx_rb, rx_data, uhost->hch[pcdc->data_interface.in_channel].trans_count);
    hal_usbd_cdc_notify_status(&usbd_dev, USBD_STATUS_RX);
    cdc_start_reception(&otg_core_struct.host, (uint8_t *)rx_data, 64);
}

/**
  * @brief  this function config gpio.
  * @param  none
  * @retval none
  */
static void usb_gpio_config(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(OTG_PIN_GPIO_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);

  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;

  /* dp and dm */
  gpio_init_struct.gpio_pins = OTG_PIN_DP | OTG_PIN_DM;
  gpio_init(OTG_PIN_GPIO, &gpio_init_struct);

  gpio_pin_mux_config(OTG_PIN_GPIO, OTG_PIN_DP_SOURCE, OTG_PIN_MUX);
  gpio_pin_mux_config(OTG_PIN_GPIO, OTG_PIN_DM_SOURCE, OTG_PIN_MUX);

#ifdef USB_SOF_OUTPUT_ENABLE
  crm_periph_clock_enable(OTG_PIN_SOF_GPIO_CLOCK, TRUE);
  gpio_init_struct.gpio_pins = OTG_PIN_SOF;
  gpio_init(OTG_PIN_SOF_GPIO, &gpio_init_struct);
  gpio_pin_mux_config(OTG_PIN_SOF_GPIO, OTG_PIN_SOF_SOURCE, OTG_PIN_MUX);
#endif

  /* otgfs use vbus pin */
#ifndef USB_VBUS_IGNORE
  gpio_init_struct.gpio_pins = OTG_PIN_VBUS;
  gpio_init_struct.gpio_pull = GPIO_PULL_DOWN;
  gpio_pin_mux_config(OTG_PIN_GPIO, OTG_PIN_VBUS_SOURCE, OTG_PIN_MUX);
  gpio_init(OTG_PIN_GPIO, &gpio_init_struct);
#endif

#ifdef USBH_5V_POWER_SWITCH
  crm_periph_clock_enable(OTG_PIN_POWER_SWITCH_CLOCK, TRUE);
  gpio_bits_set(OTG_PIN_POWER_SWITCH_GPIO, OTG_PIN_POWER_SWITCH);

  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_OPEN_DRAIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_pins = OTG_PIN_POWER_SWITCH;
  gpio_init(OTG_PIN_POWER_SWITCH_GPIO, &gpio_init_struct);
#endif
}


/**
  * @brief  usb 48M clock select
  * @param  clk_s:USB_CLK_HICK, USB_CLK_HEXT
  * @retval none
  */
void usb_clock48m_select(usb_clk48_s clk_s)
{
  if(clk_s == USB_CLK_HICK)
  {
    crm_usb_clock_source_select(CRM_USB_CLOCK_SOURCE_HICK);

    /* enable the acc calibration ready interrupt */
    crm_periph_clock_enable(CRM_ACC_PERIPH_CLOCK, TRUE);

    /* update the c1\c2\c3 value */
    acc_write_c1(7980);
    acc_write_c2(8000);
    acc_write_c3(8020);

    /* open acc calibration */
    acc_calibration_mode_enable(ACC_CAL_HICKTRIM, TRUE);
  }
  else
  {
    switch(system_core_clock)
    {
      /* 48MHz */
      case 48000000:
        crm_usb_clock_div_set(CRM_USB_DIV_1);
        break;

      /* 72MHz */
      case 72000000:
        crm_usb_clock_div_set(CRM_USB_DIV_1_5);
        break;

      /* 96MHz */
      case 96000000:
        crm_usb_clock_div_set(CRM_USB_DIV_2);
        break;

      /* 120MHz */
      case 120000000:
        crm_usb_clock_div_set(CRM_USB_DIV_2_5);
        break;

      /* 144MHz */
      case 144000000:
        crm_usb_clock_div_set(CRM_USB_DIV_3);
        break;

      /* 168MHz */
      case 168000000:
        crm_usb_clock_div_set(CRM_USB_DIV_3_5);
        break;

      /* 192MHz */
      case 192000000:
        crm_usb_clock_div_set(CRM_USB_DIV_4);
        break;

      /* 216MHz */
      case 216000000:
        crm_usb_clock_div_set(CRM_USB_DIV_4_5);
        break;

      /* 240MHz */
      case 240000000:
        crm_usb_clock_div_set(CRM_USB_DIV_5);
        break;

      /* 264MHz */
      case 264000000:
        crm_usb_clock_div_set(CRM_USB_DIV_5_5);
        break;

      /* 288MHz */
      case 288000000:
        crm_usb_clock_div_set(CRM_USB_DIV_6);
        break;

      default:
        break;

    }
  }
}

static void usb_rcu_config(void)
{
  /* enable otgfs clock */
  crm_periph_clock_enable(OTG_CLOCK, TRUE);

  /* select usb 48m clcok source */
  usb_clock48m_select(USB_CLK_HEXT);
}

static void usb_intr_config(void)
{
    //   /* enable otgfs irq */
    nvic_irq_enable(OTG_IRQ, 2, 0);
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
    usb_gpio_config();
    usb_rcu_config();
    /* init usb */
    usbh_init(&otg_core_struct,
            USB_FULL_SPEED_CORE_ID,
            USB_ID,
            &uhost_cdc_class_handler,
            &usbh_user_handle);
    err = hal_usbd_cdc_register(&usbd_dev, "usbd0", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE | RT_DEVICE_FLAG_INT_RX, RT_NULL);
    if (err != RT_EOK) {
        return err;
    }
    usb_intr_config();
    hal_usbd_cdc_notify_status(&usbd_dev, USBD_STATUS_CONNECT);
    cdc_start_reception(&otg_core_struct.host, (uint8_t *)rx_data, 64);
    return RT_EOK;
}
