/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-05-16     shelton      first version
 * 2022-11-10     shelton      support spi dma
 * 2023-01-31     shelton      add support f421/f425
 */

#include <string.h>
#include "hal/spi/spi.h"
#include "drv_spi.h"
#include "drv_common.h"
#include "drv_config.h"
#include "drv_dma.h"
#include "board_device.h"

struct at32_spi_bus {
    struct rt_spi_bus parent;
    spi_type *spi_periph;
    struct rt_spi_configuration bus_config;
#ifdef SPI_USE_DMA
    //TODO
#endif
};

struct at32_spi_cs {
    gpio_type* gpio_periph;
    uint16_t pin;
};

static rt_err_t configure(struct rt_spi_device* device,
                          struct rt_spi_configuration* configuration)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(configuration != RT_NULL);

    struct at32_spi_bus * at32_spi_bus = (struct at32_spi_bus *)device->bus;
    if (at32_spi_bus->bus_config.mode == configuration->mode
        && at32_spi_bus->bus_config.data_width == configuration->data_width
        && at32_spi_bus->bus_config.max_hz == configuration->max_hz) {
        /* same configuration, do not need re-configure */
        return RT_EOK;
    }

    spi_init_type spi_init_struct;

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

        // TODO : should check this
        if (at32_spi_bus->spi_periph  == SPI1)
        {
            spi_apb_clock = clocks_struct.apb1_freq;
            LOG_D("pclk2 freq: %d\n", clocks_struct.apb2_freq);
        }
        else
        {
            spi_apb_clock = clocks_struct.apb2_freq;
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
    spi_init(at32_spi_bus->spi_periph , &spi_init_struct);

    /* enable spi */
    spi_enable(at32_spi_bus->spi_periph , TRUE);
    /* disable spi crc */
    spi_crc_enable(at32_spi_bus->spi_periph , FALSE);

    return RT_EOK;
};

static void _spi_polling_receive_transmit(spi_type* spi_periph,rt_uint8_t *recv_buf, rt_uint8_t *send_buf, \
                                          rt_uint32_t size, rt_uint8_t data_mode)
{
    /* data frame length 8 bit */
    if(data_mode <= 8)
    {
        const rt_uint8_t *send_ptr = send_buf;
        rt_uint8_t * recv_ptr = recv_buf;

        LOG_D("spi poll transfer start: %d\n", size);

        while(size--)
        {
            rt_uint8_t data = 0xFF;

            if(send_ptr != RT_NULL)
            {
                data = *send_ptr++;
            }

            /* wait until the transmit buffer is empty */
            while(spi_i2s_flag_get(spi_periph, SPI_I2S_TDBE_FLAG) == RESET);
            /* send the byte */
            spi_i2s_data_transmit(spi_periph, data);

            /* wait until a data is received */
            while(spi_i2s_flag_get(spi_periph, SPI_I2S_RDBF_FLAG) == RESET);
            /* get the received data */
            data = spi_i2s_data_receive(spi_periph);

            if(recv_ptr != RT_NULL)
            {
                *recv_ptr++ = data;
            }
        }
        LOG_D("spi poll transfer finsh\n");
    }
    /* data frame length 16 bit */
    else if(data_mode <= 16)
    {
        const rt_uint16_t * send_ptr = (rt_uint16_t *)send_buf;
        rt_uint16_t * recv_ptr = (rt_uint16_t *)recv_buf;

        while(size--)
        {
            rt_uint16_t data = 0xFF;

            if(send_ptr != RT_NULL)
            {
                data = *send_ptr++;
            }

            /* wait until the transmit buffer is empty */
            while(spi_i2s_flag_get(spi_periph, SPI_I2S_TDBE_FLAG) == RESET);
            /* send the byte */
            spi_i2s_data_transmit(spi_periph, data);

            /* wait until a data is received */
            while(spi_i2s_flag_get(spi_periph, SPI_I2S_RDBF_FLAG) == RESET);
            /* get the received data */
            data = spi_i2s_data_receive(spi_periph);

            if(recv_ptr != RT_NULL)
            {
                *recv_ptr++ = data;
            }
        }
    }
}

static rt_uint32_t transfer(struct rt_spi_device* device, struct rt_spi_message* message)
{
    RT_ASSERT(device != NULL);
    RT_ASSERT(message != NULL);

    struct at32_spi_bus * at32_spi_bus = (struct at32_spi_bus *)device->bus;
    struct rt_spi_configuration *config = &device->config;
    struct at32_spi_cs* at32_spi_cs = (struct at32_spi_cs*)device->parent.user_data;

    rt_size_t message_length = 0, already_send_length = 0;
    rt_uint16_t send_length = 0;
    rt_uint8_t *recv_buf;
    const rt_uint8_t *send_buf;

    //struct at32_spi *instance = (struct at32_spi *)at32_spi_bus->parent.user_data;
    /* take cs */
    if(message->cs_take)
    {
        gpio_bits_reset(at32_spi_cs->gpio_periph, at32_spi_cs->pin);
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
        _spi_polling_receive_transmit(at32_spi_bus->spi_periph, (uint8_t *)recv_buf, (uint8_t *)send_buf, send_length, config->data_width);
    }

    /* release cs */
    if(message->cs_release)
    {
        gpio_bits_set(at32_spi_cs->gpio_periph, at32_spi_cs->pin);
        LOG_D("spi release cs\n");
    }

    return message->length;
}

static struct rt_spi_ops at32_spi_ops =
{
    configure,
    transfer
};


/** \brief init and register at32 spi bus.
 *
 * \param SPI: at32 SPI, e.g: SPI1,SPI2,SPI3.
 * \param stm32_spi: stm32 spi bus struct.
 * \param spi_bus_name: spi bus name, e.g: "spi1"
 * \return rt_err_t RT_EOK for success
 */
static rt_err_t at32_spi_register(spi_type* spi_periph,
                                  struct at32_spi_bus* at32_spi,
                                  const char* spi_bus_name)
{
    if (spi_periph == SPI2) {
        at32_spi->spi_periph = SPI2;
        gpio_init_type gpio_initstructure;

        crm_periph_clock_enable(CRM_SPI2_PERIPH_CLOCK, TRUE);
        crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
        crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);

        gpio_initstructure.gpio_out_type       = GPIO_OUTPUT_PUSH_PULL;
        gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;

        /* sck */
        gpio_initstructure.gpio_pull           = GPIO_PULL_UP;
        gpio_initstructure.gpio_mode           = GPIO_MODE_MUX;
        gpio_initstructure.gpio_pins           = GPIO_PINS_1;
        gpio_init(GPIOD, &gpio_initstructure);
        gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE1, GPIO_MUX_6);

        /* miso */
        gpio_initstructure.gpio_pull           = GPIO_PULL_UP;
        gpio_initstructure.gpio_pins           = GPIO_PINS_2;
        gpio_init(GPIOC, &gpio_initstructure);
        gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE2, GPIO_MUX_5);

        /* mosi */
        gpio_initstructure.gpio_pull           = GPIO_PULL_UP;
        gpio_initstructure.gpio_pins           = GPIO_PINS_4;
        gpio_init(GPIOD, &gpio_initstructure);
        gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE4, GPIO_MUX_6);

#ifdef SPI_USE_DMA
        //TODO
#endif
    } else {
        return RT_ENOSYS;
    }
    return rt_spi_bus_register(&at32_spi->parent, spi_bus_name, &at32_spi_ops);
}

/**
 * @brief Initialize spi bus and device
 * 
 * @return rt_err_t RT_EOK for success
 */
rt_err_t drv_spi_init(void)
{
    static struct at32_spi_bus at32_spi2;

    /* register SPI0 bus */
    RT_TRY(at32_spi_register(SPI2, &at32_spi2, "spi2"));

    /* attach spi_device_0 to spi2 */
    {
        static struct rt_spi_device rt_spi2_dev0;
        static struct at32_spi_cs spi2_cs0 = { .gpio_periph = GPIOD, .pin = GPIO_PINS_0 };
    
        crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
        gpio_init_type gpio_initstructure;
        /* software cs, pd0 as a general io to control flash cs */
        gpio_initstructure.gpio_out_type       = GPIO_OUTPUT_PUSH_PULL;
        gpio_initstructure.gpio_pull           = GPIO_PULL_UP;
        gpio_initstructure.gpio_mode           = GPIO_MODE_OUTPUT;
        gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
        gpio_initstructure.gpio_pins           = GPIO_PINS_0;
        gpio_init(GPIOD, &gpio_initstructure);

        /* set CS pin by default */
        gpio_bits_set(spi2_cs0.gpio_periph, spi2_cs0.pin);

        RT_TRY(rt_spi_bus_attach_device(&rt_spi2_dev0, "spi2_dev0", "spi2", (void*)&spi2_cs0));
    }

    return RT_EOK;
}
