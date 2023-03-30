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
#include "drv_spi.h"
#include "hal/spi/spi.h"

// #define SPI_USE_DMA

// struct gd32_spi_bus {
//     struct rt_spi_bus parent;
//     uint32_t spi_periph;
//     struct rt_spi_configuration bus_config;
// #ifdef SPI_USE_DMA
//     //TODO
// #endif
// };

struct at32_spi
{
    struct at32_spi_config *config;
    uint32_t spi_periph;
    struct rt_spi_bus spi_bus;  // hal level
};

struct at32_spi_config
{
    spi_type *spi_x;
    const char *spi_name;
    IRQn_Type irqn;
    struct dma_config *dma_rx;
    struct dma_config *dma_tx;
    rt_uint16_t spi_dma_flag;
};

// struct gd32_spi_cs {
//     uint32_t gpio_periph;
//     uint32_t pin;
// };

struct at32_spi_cs
{
    gpio_type *gpio_x;
    uint32_t gpio_pin;
};

// /**
//  * @brief Configure spi device
//  * 
//  * @param device SPI device
//  * @param configuration SPI device configuration
//  * @return rt_err_t RT_EOK for success
//  */
// static rt_err_t configure(struct rt_spi_device* device,
//                           struct rt_spi_configuration* configuration)
// {
//     struct gd32_spi_bus* gd32_spi_bus = (struct gd32_spi_bus*)device->bus;
//     spi_parameter_struct spi_init_struct;

//     if (gd32_spi_bus->bus_config.mode == configuration->mode
//         && gd32_spi_bus->bus_config.data_width == configuration->data_width
//         && gd32_spi_bus->bus_config.max_hz == configuration->max_hz) {
//         /* same configuration, do not need re-configure */
//         return RT_EOK;
//     }

//     if (configuration->data_width <= 8) {
//         spi_init_struct.frame_size = SPI_FRAMESIZE_8BIT;
//     } else if (configuration->data_width <= 16) {
//         spi_init_struct.frame_size = SPI_FRAMESIZE_16BIT;
//     } else {
//         return RT_EIO;
//     }

//     /* baudrate */
//     {
//         uint32_t max_hz;
//         uint32_t PCLK;

//         if (gd32_spi_bus->spi_periph == SPI1 || gd32_spi_bus->spi_periph == SPI2) {
//             /* PCLK1 60MHz max. */
//             PCLK = rcu_clock_freq_get(CK_APB1);
//         } else {
//             /* PCLK2 120MHz max. */
//             PCLK = rcu_clock_freq_get(CK_APB2);
//         }

//         max_hz = configuration->max_hz;

//         if (max_hz >= PCLK / 2) {
//             spi_init_struct.prescale = SPI_PSC_2;
//         } else if (max_hz >= PCLK / 4) {
//             spi_init_struct.prescale = SPI_PSC_4;
//         } else if (max_hz >= PCLK / 8) {
//             spi_init_struct.prescale = SPI_PSC_8;
//         } else if (max_hz >= PCLK / 16) {
//             spi_init_struct.prescale = SPI_PSC_16;
//         } else if (max_hz >= PCLK / 32) {
//             spi_init_struct.prescale = SPI_PSC_32;
//         } else if (max_hz >= PCLK / 64) {
//             spi_init_struct.prescale = SPI_PSC_64;
//         } else if (max_hz >= PCLK / 128) {
//             spi_init_struct.prescale = SPI_PSC_128;
//         } else {
//             /*  min prescaler 256 */
//             spi_init_struct.prescale = SPI_PSC_256;
//         }
//     } /* baudrate */

//     /* CPOL */
//     if (configuration->mode & RT_SPI_CPOL) {
//         /* CPHA */
//         if (configuration->mode & RT_SPI_CPHA) {
//             spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
//         } else {
//             spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_1EDGE;
//         }
//     } else {
//         /* CPHA */
//         if (configuration->mode & RT_SPI_CPHA) {
//             spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE;
//         } else {
//             spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
//         }
//     }

//     /* MSB or LSB */
//     if (configuration->mode & RT_SPI_MSB) {
//         spi_init_struct.endian = SPI_ENDIAN_MSB;
//     } else {
//         spi_init_struct.endian = SPI_ENDIAN_LSB;
//     }

//     /* first disable spi */
//     spi_disable(gd32_spi_bus->spi_periph);
//     spi_i2s_deinit(gd32_spi_bus->spi_periph);
//     /* configure SPI parameter */
//     spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
//     spi_init_struct.device_mode = SPI_MASTER;
//     spi_init_struct.nss = SPI_NSS_SOFT;
//     spi_init(gd32_spi_bus->spi_periph, &spi_init_struct);
//     /* enable SPI */
//     spi_enable(gd32_spi_bus->spi_periph);
//     /* update SPI bus configuration */
//     gd32_spi_bus->bus_config = *configuration;

//     return RT_EOK;
// }


static rt_err_t configure(struct rt_spi_device* device,
                          struct rt_spi_configuration* configuration)
{
    struct rt_spi_bus * spi_bus = (struct rt_spi_bus *)device->bus;
    struct at32_spi *instance = (struct at32_spi *)spi_bus->parent.user_data;

    spi_init_type spi_init_struct;

    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(configuration != RT_NULL);

    at32_msp_spi_init(instance->config->spi_x);

    /* data_width */
    if(configuration->data_width <= 8)
    {
        spi_init_struct.frame_bit_num = SPI_FRAME_8BIT;
    }
    else if(configuration->data_width <= 16)
    {
        spi_init_struct.frame_bit_num = SPI_FRAME_16BIT;
    }
    else
    {
        return -RT_EIO;
    }

    /* baudrate */
    {
        uint32_t spi_apb_clock;
        uint32_t max_hz;
        crm_clocks_freq_type clocks_struct;

        max_hz = configuration->max_hz;

        crm_clocks_freq_get(&clocks_struct);
        LOG_D("sys freq: %d\n", clocks_struct.sclk_freq);
        LOG_D("max freq: %d\n", max_hz);

        if (instance->config->spi_x == SPI1)
        {
            spi_apb_clock = clocks_struct.apb2_freq;
            LOG_D("pclk2 freq: %d\n", clocks_struct.apb2_freq);
        }
        else
        {
            spi_apb_clock = clocks_struct.apb1_freq;
            LOG_D("pclk1 freq: %d\n", clocks_struct.apb1_freq);
        }

        if(max_hz >= (spi_apb_clock / 2))
        {
            spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_2;
        }
        else if (max_hz >= (spi_apb_clock / 4))
        {
            spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_4;
        }
        else if (max_hz >= (spi_apb_clock / 8))
        {
            spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_8;
        }
        else if (max_hz >= (spi_apb_clock / 16))
        {
            spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_16;
        }
        else if (max_hz >= (spi_apb_clock / 32))
        {
            spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_32;
        }
        else if (max_hz >= (spi_apb_clock / 64))
        {
            spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_64;
        }
        else if (max_hz >= (spi_apb_clock / 128))
        {
            spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_128;
        }
        else
        {
            /* min prescaler 256 */
            spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_256;
        }
    } /* baudrate */

    switch(configuration->mode & RT_SPI_MODE_3)
    {
    case RT_SPI_MODE_0:
        spi_init_struct.clock_phase = SPI_CLOCK_PHASE_1EDGE;
        spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_LOW;
        break;
    case RT_SPI_MODE_1:
        spi_init_struct.clock_phase = SPI_CLOCK_PHASE_2EDGE;
        spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_LOW;
        break;
    case RT_SPI_MODE_2:
        spi_init_struct.clock_phase = SPI_CLOCK_PHASE_1EDGE;
        spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_HIGH;
        break;
    case RT_SPI_MODE_3:
        spi_init_struct.clock_phase = SPI_CLOCK_PHASE_2EDGE;
        spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_HIGH;
        break;
    }

    /* msb or lsb */
    if(configuration->mode & RT_SPI_MSB)
    {
        spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_MSB;
    }
    else
    {
        spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_LSB;
    }

    spi_init_struct.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
    spi_init_struct.master_slave_mode = SPI_MODE_MASTER;
    spi_init_struct.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
    /* init spi */
    spi_init(instance->config->spi_x, &spi_init_struct);

    /* enable spi */
    spi_enable(instance->config->spi_x, TRUE);
    /* disable spi crc */
    spi_crc_enable(instance->config->spi_x, FALSE);

    return RT_EOK;
};
// /**
//  * @brief SPI transfer function
//  * 
//  * @param device SPI device instance
//  * @param message SPI message to be transfered
//  * @return rt_uint32_t bytes have been transfered
//  */
// static rt_uint32_t transfer(struct rt_spi_device* device, struct rt_spi_message* message)
// {
//     struct gd32_spi_bus* gd32_spi_bus = (struct gd32_spi_bus*)device->bus;
//     struct rt_spi_configuration* config = &device->config;
//     struct gd32_spi_cs* gd32_spi_cs = (struct gd32_spi_cs*)device->parent.user_data;
//     rt_uint32_t size = message->length;

//     /* take CS */
//     if (message->cs_take) {
//         gpio_bit_reset(gd32_spi_cs->gpio_periph, gd32_spi_cs->pin);
//     }

// #ifdef SPI_USE_DMA
//     #error Not support SPI DMA.
// #endif

//     {
//         if (config->data_width <= 8) {
//             const rt_uint8_t* send_ptr = message->send_buf;
//             rt_uint8_t* recv_ptr = message->recv_buf;

//             while (size--) {
//                 rt_uint8_t data = 0xFF;

//                 if (send_ptr != RT_NULL) {
//                     data = *send_ptr++;
//                 }

//                 /* Wait until the transmit buffer is empty */
//                 while (RESET == spi_i2s_flag_get(gd32_spi_bus->spi_periph, SPI_FLAG_TBE))
//                     ;

//                 /* Send the byte */
//                 spi_i2s_data_transmit(gd32_spi_bus->spi_periph, data);

//                 /* Wait until a data is received */
//                 while (RESET == spi_i2s_flag_get(gd32_spi_bus->spi_periph, SPI_FLAG_RBNE))
//                     ;

//                 /* Get the received data */
//                 data = spi_i2s_data_receive(gd32_spi_bus->spi_periph);

//                 if (recv_ptr != RT_NULL) {
//                     *recv_ptr++ = data;
//                 }
//             }
//         } else if (config->data_width <= 16) {
//             const rt_uint16_t* send_ptr = message->send_buf;
//             rt_uint16_t* recv_ptr = message->recv_buf;

//             while (size--) {
//                 rt_uint16_t data = 0xFFFF;

//                 if (send_ptr != RT_NULL) {
//                     data = *send_ptr++;
//                 }

//                 /* Wait until the transmit buffer is empty */
//                 while (RESET == spi_i2s_flag_get(gd32_spi_bus->spi_periph, SPI_FLAG_TBE))
//                     ;

//                 /* Send the byte */
//                 spi_i2s_data_transmit(gd32_spi_bus->spi_periph, data);

//                 /* Wait until a data is received */
//                 while (RESET == spi_i2s_flag_get(gd32_spi_bus->spi_periph, SPI_FLAG_RBNE))
//                     ;

//                 /* Get the received data */
//                 data = spi_i2s_data_receive(gd32_spi_bus->spi_periph);

//                 if (recv_ptr != RT_NULL) {
//                     *recv_ptr++ = data;
//                 }
//             }
//         }
//     }

//     /* release CS */
//     if (message->cs_release) {
//         gpio_bit_set(gd32_spi_cs->gpio_periph, gd32_spi_cs->pin);
//     }

//     return message->length;
// }



static rt_uint32_t xfer(struct rt_spi_device* device, struct rt_spi_message* message)
{
    struct rt_spi_bus * at32_spi_bus = (struct rt_spi_bus *)device->bus;
    struct at32_spi *instance = (struct at32_spi *)at32_spi_bus->parent.user_data;
    struct rt_spi_configuration *config = &device->config;
    struct at32_spi_cs * at32_spi_cs = device->parent.user_data;
    rt_size_t message_length = 0, already_send_length = 0;
    rt_uint16_t send_length = 0;
    rt_uint8_t *recv_buf;
    const rt_uint8_t *send_buf;

    RT_ASSERT(device != NULL);
    RT_ASSERT(message != NULL);

    /* take cs */
    if(message->cs_take)
    {
        gpio_bits_reset(at32_spi_cs->gpio_x, at32_spi_cs->gpio_pin);
        LOG_D("spi take cs\n");
    }

    message_length = message->length;
    recv_buf = message->recv_buf;
    send_buf = message->send_buf;
    while (message_length)
    {
        /* the HAL library use uint16 to save the data length */
        if (message_length > 65535)
        {
            send_length = 65535;
            message_length = message_length - 65535;
        }
        else
        {
            send_length = message_length;
            message_length = 0;
        }

        /* calculate the start address */
        already_send_length = message->length - send_length - message_length;
        send_buf = (rt_uint8_t *)message->send_buf + already_send_length;
        recv_buf = (rt_uint8_t *)message->recv_buf + already_send_length;

        /* start once data exchange in dma mode */
        if (message->send_buf && message->recv_buf)
        {
            if ((instance->config->spi_dma_flag & RT_DEVICE_FLAG_DMA_RX) && \
                (instance->config->spi_dma_flag & RT_DEVICE_FLAG_DMA_TX))
            {
                _spi_dma_receive(instance, (uint8_t *)recv_buf, send_length);
                _spi_dma_transmit(instance, (uint8_t *)send_buf, send_length);
                /* wait transfer complete */
                while(spi_i2s_flag_get(instance->config->spi_x, SPI_I2S_BF_FLAG) != RESET);
                while((instance->config->dma_tx->dma_done == RT_FALSE) || (instance->config->dma_rx->dma_done == RT_FALSE));
                /* clear rx overrun flag */
                spi_i2s_flag_clear(instance->config->spi_x, SPI_I2S_ROERR_FLAG);
                spi_enable(instance->config->spi_x, FALSE);
                spi_enable(instance->config->spi_x, TRUE);
            }
            else
            {
                _spi_polling_receive_transmit(instance, (uint8_t *)recv_buf, (uint8_t *)send_buf, send_length, config->data_width);
            }
        }
        else if (message->send_buf)
        {
            if (instance->config->spi_dma_flag & RT_DEVICE_FLAG_DMA_TX)
            {
                _spi_dma_transmit(instance, (uint8_t *)send_buf, send_length);
                /* wait transfer complete */
                while(spi_i2s_flag_get(instance->config->spi_x, SPI_I2S_BF_FLAG) != RESET);
                while(instance->config->dma_tx->dma_done == RT_FALSE);
                /* clear rx overrun flag */
                spi_i2s_flag_clear(instance->config->spi_x, SPI_I2S_ROERR_FLAG);
                spi_enable(instance->config->spi_x, FALSE);
                spi_enable(instance->config->spi_x, TRUE);
            }
            else
            {
                _spi_polling_receive_transmit(instance, RT_NULL, (uint8_t *)send_buf, send_length, config->data_width);
            }

            if (message->cs_release && (device->config.mode & RT_SPI_3WIRE))
            {
                /* release the cs by disable spi when using 3 wires spi */
                spi_enable(instance->config->spi_x, FALSE);
            }
        }
        else
        {
            memset((void *)recv_buf, 0xff, send_length);
            if (instance->config->spi_dma_flag & RT_DEVICE_FLAG_DMA_RX)
            {
                _spi_dma_receive(instance, (uint8_t *)recv_buf, send_length);
                _spi_dma_transmit(instance, (uint8_t *)recv_buf, send_length);
                /* wait transfer complete */
                while(spi_i2s_flag_get(instance->config->spi_x, SPI_I2S_BF_FLAG) != RESET);
                while((instance->config->dma_tx->dma_done == RT_FALSE) || (instance->config->dma_rx->dma_done == RT_FALSE));
                /* clear rx overrun flag */
                spi_i2s_flag_clear(instance->config->spi_x, SPI_I2S_ROERR_FLAG);
                spi_enable(instance->config->spi_x, FALSE);
                spi_enable(instance->config->spi_x, TRUE);
            }
            else
            {
                /* clear the old error flag */
                spi_i2s_flag_clear(instance->config->spi_x, SPI_I2S_ROERR_FLAG);
                _spi_polling_receive_transmit(instance, (uint8_t *)recv_buf, (uint8_t *)recv_buf, send_length, config->data_width);
            }
        }
    }

    /* release cs */
    if(message->cs_release)
    {
        gpio_bits_set(at32_spi_cs->gpio_x, at32_spi_cs->gpio_pin);
        LOG_D("spi release cs\n");
    }

    return message->length;
}

// static struct rt_spi_ops gd32_spi_ops = {
//     configure,
//     transfer
// };

static struct rt_spi_ops at32_spi_ops =
{
    configure,
    xfer
};


// /** \brief init and register gd32 spi bus.
//  *
//  * \param SPI: GD32 SPI, e.g: SPI1,SPI2,SPI3.
//  * \param stm32_spi: stm32 spi bus struct.
//  * \param spi_bus_name: spi bus name, e.g: "spi1"
//  * \return rt_err_t RT_EOK for success
//  */
// static rt_err_t gd32_spi_register(uint32_t spi_periph,
//                                   struct gd32_spi_bus* gd32_spi,
//                                   const char* spi_bus_name)
// {
//     if (spi_periph == SPI0) {
//         gd32_spi->spi_periph = SPI0;

//         /* SPI0 configure */
//         rcu_periph_clock_enable(RCU_GPIOA);
//         rcu_periph_clock_enable(RCU_GPIOB);
//         /* Peripheral clock enable */
//         rcu_periph_clock_enable(RCU_SPI0);

//         /*  SPI0 GPIO Configuration
//         PA5 ------> SPI0_SCK
//         PA6 ------> SPI0_MISO
//         PB5 ------> SPI0_MOSI */
//         gpio_af_set(GPIOA, GPIO_AF_5, GPIO_PIN_5 | GPIO_PIN_6);
//         gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5 | GPIO_PIN_6);
//         gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5 | GPIO_PIN_6);
//         gpio_af_set(GPIOB, GPIO_AF_5, GPIO_PIN_5);
//         gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
//         gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5);

// #ifdef SPI_USE_DMA
//         //TODO
// #endif
//     } else if (spi_periph == SPI1) {
//         gd32_spi->spi_periph = SPI1;

//         /* SPI0 configure */
//         rcu_periph_clock_enable(RCU_GPIOB);
//         /* Peripheral clock enable */
//         rcu_periph_clock_enable(RCU_SPI1);

//         /*  SPI0 GPIO Configuration
//         PB10 ------> SPI0_SCK
//         PB14 ------> SPI0_MISO
//         PB15 ------> SPI0_MOSI */
//         gpio_af_set(GPIOB, GPIO_AF_5, GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15);
//         gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15);
//         gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15);

// #ifdef SPI_USE_DMA
//         //TODO
// #endif
//     } else {
//         return RT_ENOSYS;
//     }
//     return rt_spi_bus_register(&gd32_spi->parent, spi_bus_name, &gd32_spi_ops);
// }

/** \brief init and register gd32 spi bus.
 *
 * \param SPI: GD32 SPI, e.g: SPI1,SPI2,SPI3.
 * \param stm32_spi: stm32 spi bus struct.
 * \param spi_bus_name: spi bus name, e.g: "spi1"
 * \return rt_err_t RT_EOK for success
 */
static rt_err_t at32_spi_register(uint32_t spi_periph,
                                  struct gd32_spi_bus* gd32_spi,
                                  const char* spi_bus_name){


}

// /**
//  * @brief Initialize spi bus and device
//  * 
//  * @return rt_err_t RT_EOK for success
//  */
// rt_err_t drv_spi_init(void)
// {
//     static struct gd32_spi_bus gd32_spi0;
//     static struct gd32_spi_bus gd32_spi1;

//     /* register SPI0 bus */
//     RT_TRY(gd32_spi_register(SPI0, &gd32_spi0, "spi0"));

//     /* attach spi_device_0 (BMI088 ACCEL) to spi0 */
//     {
//         static struct rt_spi_device rt_spi0_dev0;
//         static struct gd32_spi_cs spi0_cs0 = { .gpio_periph = GPIOC, .pin = GPIO_PIN_14 };

//         /* enable cs pin clock */
//         rcu_periph_clock_enable(RCU_GPIOC);
//         /* configure cs pin gpio */
//         gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_14);
//         gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_14);
//         /* set CS pin by default */
//         gpio_bit_set(spi0_cs0.gpio_periph, spi0_cs0.pin);

//         RT_TRY(rt_spi_bus_attach_device(&rt_spi0_dev0, "spi0_dev0", "spi0", (void*)&spi0_cs0));
//     }

//     /* attach spi_device_1 (BMI088 GYRO) to spi0 */
//     {
//         static struct rt_spi_device rt_spi0_dev1;
//         static struct gd32_spi_cs spi0_cs1 = { .gpio_periph = GPIOC, .pin = GPIO_PIN_15 };

//         /* enable cs pin clock */
//         rcu_periph_clock_enable(RCU_GPIOC);
//         /* configure cs pin gpio */
//         gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_15);
//         gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_15);
//         /* set CS pin by default */
//         gpio_bit_set(spi0_cs1.gpio_periph, spi0_cs1.pin);

//         RT_TRY(rt_spi_bus_attach_device(&rt_spi0_dev1, "spi0_dev1", "spi0", (void*)&spi0_cs1));
//     }

//     /* attach spi_device_2 (BMM150 MAG) to spi0 */
//     {
//         static struct rt_spi_device rt_spi0_dev2;
//         static struct gd32_spi_cs spi0_cs2 = { .gpio_periph = GPIOC, .pin = GPIO_PIN_2 };

//         /* enable cs pin clock */
//         rcu_periph_clock_enable(RCU_GPIOC);
//         /* configure cs pin gpio */
//         gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_2);
//         gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
//         /* set CS pin by default */
//         gpio_bit_set(spi0_cs2.gpio_periph, spi0_cs2.pin);

//         RT_TRY(rt_spi_bus_attach_device(&rt_spi0_dev2, "spi0_dev2", "spi0", (void*)&spi0_cs2));
//     }

//     /* attach spi_device_3 (SPL06 Baro) to spi0 */
//     {
//         static struct rt_spi_device rt_spi0_dev3;
//         static struct gd32_spi_cs spi0_cs3 = { .gpio_periph = GPIOE, .pin = GPIO_PIN_15 };

//         /* enable cs pin clock */
//         rcu_periph_clock_enable(RCU_GPIOE);
//         /* configure cs pin gpio */
//         gpio_mode_set(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_15);
//         gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_15);
//         /* set CS pin by default */
//         gpio_bit_set(spi0_cs3.gpio_periph, spi0_cs3.pin);

//         RT_TRY(rt_spi_bus_attach_device(&rt_spi0_dev3, "spi0_dev3", "spi0", (void*)&spi0_cs3));
//     }

//     /* attach spi_device_4 (ICM42688 IMU) to spi0 */
//     {
//         static struct rt_spi_device rt_spi0_dev4;
//         static struct gd32_spi_cs spi0_cs4 = { .gpio_periph = GPIOE, .pin = GPIO_PIN_12 };

//         /* enable cs pin clock */
//         rcu_periph_clock_enable(RCU_GPIOE);
//         /* configure cs pin gpio */
//         gpio_mode_set(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_12);
//         gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);
//         /* set CS pin by default */
//         gpio_bit_set(spi0_cs4.gpio_periph, spi0_cs4.pin);

//         RT_TRY(rt_spi_bus_attach_device(&rt_spi0_dev4, "spi0_dev4", "spi0", (void*)&spi0_cs4));
//     }

//     /* register SPI1 bus */
//     RT_TRY(gd32_spi_register(SPI1, &gd32_spi1, "spi1"));

//     /* attach spi_device_0 (SPI1 FRAM) to spi1 */
//     {
//         static struct rt_spi_device rt_spi1_dev0;
//         static struct gd32_spi_cs spi1_cs0 = { .gpio_periph = GPIOD, .pin = GPIO_PIN_10 };

//         /* enable cs pin clock */
//         rcu_periph_clock_enable(RCU_GPIOD);
//         /* configure cs pin gpio */
//         gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_10);
//         gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
//         /* set CS pin by default */
//         gpio_bit_set(spi1_cs0.gpio_periph, spi1_cs0.pin);

//         RT_TRY(rt_spi_bus_attach_device(&rt_spi1_dev0, "spi1_dev0", "spi1", (void*)&spi1_cs0));
//     }

//     /* attach spi_device_1 (ICM20948 IMU) to spi1 */
//     {
//         static struct rt_spi_device rt_spi1_dev1;
//         static struct gd32_spi_cs spi1_cs1 = { .gpio_periph = GPIOE, .pin = GPIO_PIN_2 };

//         /* enable cs pin clock */
//         rcu_periph_clock_enable(RCU_GPIOE);
//         /* configure cs pin gpio */
//         gpio_mode_set(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_2);
//         gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
//         /* set CS pin by default */
//         gpio_bit_set(spi1_cs1.gpio_periph, spi1_cs1.pin);

//         RT_TRY(rt_spi_bus_attach_device(&rt_spi1_dev1, "spi1_dev1", "spi1", (void*)&spi1_cs1));
//     }

//     /* attach spi_device_2 (MS5611 Barometer) to spi1 */
//     {
//         static struct rt_spi_device rt_spi1_dev2;
//         static struct gd32_spi_cs spi1_cs2 = { .gpio_periph = GPIOE, .pin = GPIO_PIN_3 };

//         /* enable cs pin clock */
//         rcu_periph_clock_enable(RCU_GPIOE);
//         /* configure cs pin gpio */
//         gpio_mode_set(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_3);
//         gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
//         /* set CS pin by default */
//         gpio_bit_set(spi1_cs2.gpio_periph, spi1_cs2.pin);

//         RT_TRY(rt_spi_bus_attach_device(&rt_spi1_dev2, "spi1_dev2", "spi1", (void*)&spi1_cs2));
//     }

//     /* attach spi_device_3 (SPI1 CS1) to spi1 */
//     {
//         static struct rt_spi_device rt_spi1_dev3;
//         static struct gd32_spi_cs spi1_cs3 = { .gpio_periph = GPIOC, .pin = GPIO_PIN_13 };

//         /* enable cs pin clock */
//         rcu_periph_clock_enable(RCU_GPIOC);
//         /* configure cs pin gpio */
//         gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_13);
//         gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
//         /* set CS pin by default */
//         gpio_bit_set(spi1_cs3.gpio_periph, spi1_cs3.pin);

//         RT_TRY(rt_spi_bus_attach_device(&rt_spi1_dev3, "spi1_dev3", "spi1", (void*)&spi1_cs3));
//     }

//     /* attach spi_device_4 (SPI1 CS2) to spi1 */
//     {
//         static struct rt_spi_device rt_spi1_dev4;
//         static struct gd32_spi_cs spi1_cs4 = { .gpio_periph = GPIOE, .pin = GPIO_PIN_10 };

//         /* enable cs pin clock */
//         rcu_periph_clock_enable(RCU_GPIOE);
//         /* configure cs pin gpio */
//         gpio_mode_set(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_10);
//         gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
//         /* set CS pin by default */
//         gpio_bit_set(spi1_cs4.gpio_periph, spi1_cs4.pin);

//         RT_TRY(rt_spi_bus_attach_device(&rt_spi1_dev4, "spi1_dev4", "spi1", (void*)&spi1_cs4));
//     }

//     return RT_EOK;
// }

rt_err_t drv_spi_init(void)
{
    
}