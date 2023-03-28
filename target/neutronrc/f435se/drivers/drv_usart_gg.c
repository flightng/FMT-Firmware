/******************************************************************************
 * Copyright 2020 The Firmament Authors. All Rights Reserved.
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

#include "drv_usart.h"
#include "hal/serial/serial.h"

#include "drv_common.h"
#include "drv_usart.h"
#include "drv_config.h"
#include "board_device.h"
#include "hal/serial/serial.h"

#define SET_BIT(REG, BIT)   ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)  ((REG) & (BIT))

#define UART_ENABLE_IRQ(n)  NVIC_EnableIRQ((n))
#define UART_DISABLE_IRQ(n) NVIC_DisableIRQ((n))

#define USING_UART0
#define USING_UART1
#define USING_UART2
#define USING_UART3
#define USING_UART4
#define USING_UART5
#define USING_UART6
#define USING_UART7

#define SERIAL3_DEFAULT_CONFIG                    \
    {                                             \
        BAUD_RATE_115200,    /* 115200 bits/s */  \
            DATA_BITS_8,     /* 8 databits */     \
            STOP_BITS_1,     /* 1 stopbit */      \
            PARITY_NONE,     /* No parity  */     \
            BIT_ORDER_LSB,   /* LSB first sent */ \
            NRZ_NORMAL,      /* Normal mode */    \
            SERIAL_RB_BUFSZ, /* Buffer size */    \
            0                                     \
    }


/* GD32 uart driver */
// Todo: compress uart info
// struct gd32_uart {
//     uint32_t uart_periph;
//     IRQn_Type irqn;
//     rcu_periph_enum per_clk;
//     rcu_periph_enum tx_gpio_clk;
//     rcu_periph_enum rx_gpio_clk;
//     uint32_t tx_port;
//     uint16_t tx_af;
//     uint16_t tx_pin;
//     uint32_t rx_port;
//     uint16_t rx_af;
//     uint16_t rx_pin;
//     struct gd32_uart_dma {
//         /* dma instance */
//         uint32_t dma_periph;
//         /* dma clock */
//         rcu_periph_enum clock;
//         /* dma rx channel */
//         dma_channel_enum rx_ch;
//         /* dma rx irq */
//         uint8_t rx_irq;
//         /* dma tx channel */
//         dma_channel_enum tx_ch;
//         /* dma tx irq */
//         uint8_t tx_irq;
//         /* dma peripheral select */
//         dma_subperipheral_enum sub_periph;
//         /* setting receive len */
//         rt_size_t setting_recv_len;
//         /* last receive index */
//         rt_size_t last_recv_index;
//     } dma;
// };

struct at32_uart {
    char *name;
    usart_type *uart_x;
    IRQn_Type irqn;
    struct dma_config *dma_rx;
    rt_size_t last_index;
    struct dma_config *dma_tx;
    rt_uint16_t uart_dma_flag;
    struct serial_device serial;
    
    crm_periph_clock_type per_clk;
    crm_periph_clock_type tx_gpio_clk;
    crm_periph_clock_type rx_gpio_clk;

    uint32_t tx_port;
    uint16_t tx_af;
    uint16_t tx_pin;
    uint32_t rx_port;
    uint16_t rx_af;
    uint16_t rx_pin;
};

// /**
//  * Serial port receive idle process. This need add to uart idle ISR.
//  *
//  * @param serial serial device
//  */
// static void dma_uart_rx_idle_isr(struct serial_device* serial)
// {
//     struct gd32_uart* uart = (struct gd32_uart*)serial->parent.user_data;
//     rt_size_t recv_total_index, recv_len;
//     rt_base_t level;
//     uint32_t remain_bytes;

//     /* disable interrupt */
//     level = rt_hw_interrupt_disable();
//     /* check remain bytes to receive */
//     remain_bytes = dma_transfer_number_get(uart->dma.dma_periph, uart->dma.rx_ch);
//     /* total received bytes */
//     recv_total_index = uart->dma.setting_recv_len - remain_bytes;
//     /* received bytes at this time */
//     recv_len = recv_total_index - uart->dma.last_recv_index;
//     /* update last received total bytes */
//     uart->dma.last_recv_index = recv_total_index;
//     /* enable interrupt */
//     rt_hw_interrupt_enable(level);

//     if (recv_len) {
//         /* high-level ISR routine */
//         hal_serial_isr(serial, SERIAL_EVENT_RX_DMADONE | (recv_len << 8));
//     }
// }

// /**
//  * DMA receive done process. This need add to DMA receive done ISR.
//  *
//  * @param serial serial device
//  */
// static void dma_rx_done_isr(struct serial_device* serial)
// {
//     struct gd32_uart* uart = (struct gd32_uart*)serial->parent.user_data;
//     rt_size_t recv_len;
//     rt_base_t level;

//     /* disable interrupt */
//     level = rt_hw_interrupt_disable();
//     /* received bytes at this time */
//     recv_len = uart->dma.setting_recv_len - uart->dma.last_recv_index;
//     /* reset last recv index */
//     uart->dma.last_recv_index = 0;
//     /* enable interrupt */
//     rt_hw_interrupt_enable(level);

//     if (recv_len) {
//         /* high-level ISR routine */
//         hal_serial_isr(serial, SERIAL_EVENT_RX_DMADONE | (recv_len << 8));
//     }
// }

#ifdef RT_SERIAL_USING_DMA
void dma_rx_isr(struct serial_device *serial)
{
    volatile rt_uint32_t reg_sts = 0, index = 0;
    rt_size_t recv_total_index, recv_len;
    rt_base_t level;
    struct at32_uart *instance;
    instance = (struct at32_uart *) serial->parent.user_data;
    RT_ASSERT(instance != RT_NULL);

    reg_sts = instance->dma_rx->dma_x->sts;
    index = instance->dma_rx->channel_index;

    if (((reg_sts & (DMA_FDT_FLAG << (4 * (index - 1)))) != RESET) ||
        ((reg_sts & (DMA_HDT_FLAG << (4 * (index - 1)))) != RESET))
    {
        /* clear dma flag */
        instance->dma_rx->dma_x->clr |= (rt_uint32_t)(DMA_FDT_FLAG << (4 * (index - 1))) | (DMA_HDT_FLAG << (4 * (index - 1)));

        level = rt_hw_interrupt_disable();
        recv_total_index = serial->config.bufsz - dma_data_number_get(instance->dma_rx->dma_channel);
        if (recv_total_index == 0)
        {
            recv_len = serial->config.bufsz - instance->last_index;
        }
        else
        {
            recv_len = recv_total_index - instance->last_index;
        }
        instance->last_index = recv_total_index;
        rt_hw_interrupt_enable(level);

        if (recv_len)
        {
            rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_DMADONE | (recv_len << 8));
        }
    }
}

// /**
//  * DMA transmit done process. This need add to DMA receive done ISR.
//  *
//  * @param serial serial device
//  */
// static void dma_tx_done_isr(struct serial_device* serial)
// {
//     /* high-level ISR routine */
//     hal_serial_isr(serial, SERIAL_EVENT_TX_DMADONE);
// }

void dma_tx_isr(struct serial_device *serial)
{
    volatile rt_uint32_t reg_sts = 0, index = 0;
    rt_size_t trans_total_index;
    rt_base_t level;
    RT_ASSERT(serial != RT_NULL);
    struct at32_uart *instance;
    instance = (struct at32_uart *) serial->parent.user_data;
    RT_ASSERT(instance != RT_NULL);

    reg_sts = instance->dma_tx->dma_x->sts;
    index = instance->dma_tx->channel_index;

    if ((reg_sts & (DMA_FDT_FLAG << (4 * (index - 1)))) != RESET)
    {
        /* mark dma flag */
        instance->dma_tx->dma_done = RT_TRUE;
        /* clear dma flag */
        instance->dma_tx->dma_x->clr |= (rt_uint32_t)(DMA_FDT_FLAG << (4 * (index - 1)));
        /* disable dma tx channel */
        dma_channel_enable(instance->dma_tx->dma_channel, FALSE);

        level = rt_hw_interrupt_disable();
        trans_total_index = dma_data_number_get(instance->dma_tx->dma_channel);
        rt_hw_interrupt_enable(level);

        if (trans_total_index == 0)
        {
            rt_hw_serial_isr(serial, RT_SERIAL_EVENT_TX_DMADONE);
        }
    }
}
#endif

// static void uart_isr(struct serial_device* serial)
// {
//     struct gd32_uart* uart = (struct gd32_uart*)serial->parent.user_data;

//     if ((usart_interrupt_flag_get(uart->uart_periph, USART_INT_FLAG_RBNE) != RESET) && (usart_flag_get(uart->uart_periph, USART_FLAG_RBNE) != RESET)) {
//         /* high-level ISR routine */
//         hal_serial_isr(serial, SERIAL_EVENT_RX_IND);
//         /* Clear RXNE interrupt flag */
//         usart_flag_clear(uart->uart_periph, USART_FLAG_RBNE);
//     }

//     if ((usart_interrupt_flag_get(uart->uart_periph, USART_INT_FLAG_IDLE) != RESET) && (usart_flag_get(uart->uart_periph, USART_FLAG_IDLE) != RESET)) {
//         dma_uart_rx_idle_isr(serial);
//         /* Read USART_DATA to clear IDLE interrupt flag */
//         usart_data_receive(uart->uart_periph);
//     }

//     if ((usart_interrupt_flag_get(uart->uart_periph, USART_INT_FLAG_TC) != RESET) && (usart_flag_get(uart->uart_periph, USART_FLAG_TC) != RESET)) {
//         /* Clear TC interrupt flag */
//         usart_flag_clear(uart->uart_periph, USART_FLAG_TC);
//     }
// }

static void usart_isr(struct serial_device *serial) {
    struct at32_uart *instance;
#ifdef RT_SERIAL_USING_DMA
    rt_size_t recv_total_index, recv_len;
    rt_base_t level;
#endif
    RT_ASSERT(serial != RT_NULL);

    instance = (struct at32_uart *) serial->parent.user_data;
    RT_ASSERT(instance != RT_NULL);

    if (usart_flag_get(instance->uart_x, USART_RDBF_FLAG) != RESET) {
        hal_serial_isr(serial, SERIAL_EVENT_RX_IND);
    }
#ifdef RT_SERIAL_USING_DMA
    else if (usart_flag_get(instance->uart_x, USART_IDLEF_FLAG) != RESET)
    {
        /* clear idle flag */
        usart_data_receive(instance->uart_x);

        level = rt_hw_interrupt_disable();
        recv_total_index = serial->config.bufsz - dma_data_number_get(instance->dma_rx->dma_channel);
        recv_len = recv_total_index - instance->last_index;
        instance->last_index = recv_total_index;
        rt_hw_interrupt_enable(level);

        if (recv_len)
        {
            rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_DMADONE | (recv_len << 8));
        }
    }
#endif
    else
    {
        if (usart_flag_get(instance->uart_x, USART_CTSCF_FLAG) != RESET) {
            usart_flag_clear(instance->uart_x, USART_CTSCF_FLAG);
        }

        if (usart_flag_get(instance->uart_x, USART_BFF_FLAG) != RESET) {
            usart_flag_clear(instance->uart_x, USART_BFF_FLAG);
        }

        if (usart_flag_get(instance->uart_x, USART_TDC_FLAG) != RESET) {
            usart_flag_clear(instance->uart_x, USART_TDC_FLAG);
        }
    }
}

// #ifdef USING_UART6
// static struct serial_device serial0;
// static struct gd32_uart uart6 = {
//     .uart_periph = UART6,
//     .irqn = UART6_IRQn,
//     .per_clk = RCU_UART6,
//     .tx_gpio_clk = RCU_GPIOE,
//     .rx_gpio_clk = RCU_GPIOE,
//     .tx_port = GPIOE,
//     .tx_af = GPIO_AF_8,
//     .tx_pin = GPIO_PIN_8,
//     .rx_port = GPIOE,
//     .rx_af = GPIO_AF_8,
//     .rx_pin = GPIO_PIN_7,
// };

// void UART6_IRQHandler(void)
// {
//     /* enter interrupt */
//     rt_interrupt_enter();
//     /* uart isr routine */
//     uart_isr(&serial0);
//     /* leave interrupt */
//     rt_interrupt_leave();
// }
// #endif /* USING_UART6 */

// #ifdef USING_UART1
// static struct serial_device serial1;
// static struct gd32_uart uart1 = {
//     .uart_periph = USART1,
//     .irqn = USART1_IRQn,
//     .per_clk = RCU_USART1,
//     .tx_gpio_clk = RCU_GPIOD,
//     .rx_gpio_clk = RCU_GPIOD,
//     .tx_port = GPIOD,
//     .tx_af = GPIO_AF_7,
//     .tx_pin = GPIO_PIN_5,
//     .rx_port = GPIOD,
//     .rx_af = GPIO_AF_7,
//     .rx_pin = GPIO_PIN_6,
//     .dma = {
//         .dma_periph = DMA0,
//         .clock = RCU_DMA0,
//         .rx_ch = DMA_CH5,
//         .rx_irq = DMA0_Channel5_IRQn,
//         .tx_ch = DMA_CH6,
//         .tx_irq = DMA0_Channel6_IRQn,
//         .sub_periph = DMA_SUBPERI4,
//     }
// };

// void USART1_IRQHandler(void)
// {
//     /* enter interrupt */
//     rt_interrupt_enter();
//     /* uart isr routine */
//     uart_isr(&serial1);
//     /* leave interrupt */
//     rt_interrupt_leave();
// }

// void DMA0_Channel6_IRQHandler(void)
// {
//     /* enter interrupt */
//     rt_interrupt_enter();
//     if (dma_interrupt_flag_get(uart1.dma.dma_periph, uart1.dma.tx_ch, DMA_INT_FLAG_FTF)) {
//         dma_tx_done_isr(&serial1);
//         dma_interrupt_flag_clear(uart1.dma.dma_periph, uart1.dma.tx_ch, DMA_INT_FLAG_FTF);
//     }
//     /* leave interrupt */
//     rt_interrupt_leave();
// }

// void DMA0_Channel5_IRQHandler(void)
// {
//     /* enter interrupt */
//     rt_interrupt_enter();
//     if (dma_interrupt_flag_get(uart1.dma.dma_periph, uart1.dma.rx_ch, DMA_INT_FLAG_FTF)) {
//         dma_rx_done_isr(&serial1);
//         dma_interrupt_flag_clear(uart1.dma.dma_periph, uart1.dma.rx_ch, DMA_INT_FLAG_FTF);
//     }
//     /* leave interrupt */
//     rt_interrupt_leave();
// }
// #endif /* USING_UART1 */

#ifdef BSP_USING_UART1
static struct serial_device serial0;
static struct at32_uart uart1 = {
    .uart_x = USART1,
    .irqn = USART1_IRQn,

    .per_clk = CRM_USART1_PERIPH_CLOCK,
    .tx_gpio_clk = CRM_GPIOA_PERIPH_CLOCK,
    .rx_gpio_clk = CRM_GPIOA_PERIPH_CLOCK,
    .tx_port = (gpio_type *)GPIOA,

    //.tx_af = GPIO_AF_8,
    .tx_pin = GPIO_PINS_9,
    .rx_port = (gpio_type *)GPIOA,
    //.rx_af = GPIO_AF_8,
    .rx_pin = GPIO_PINS_10,
};


void USART1_IRQHandler(void) {
    rt_interrupt_enter();

    usart_isr(&serial0);

    rt_interrupt_leave();
}
#endif

#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART1_RX_USING_DMA)
void UART1_RX_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    dma_rx_isr(&serial0);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(RT_SERIAL_USING_DMA) && defined(BSP_UART1_RX_USING_DMA) */

#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART1_TX_USING_DMA)
void UART1_TX_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    dma_tx_isr(&serial0);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(RT_SERIAL_USING_DMA) && defined(BSP_UART1_TX_USING_DMA) */

// #ifdef USING_UART2
// static struct serial_device serial2;
// static struct gd32_uart uart2 = {
//     .uart_periph = USART2,
//     .irqn = USART2_IRQn,
//     .per_clk = RCU_USART2,
//     .tx_gpio_clk = RCU_GPIOD,
//     .rx_gpio_clk = RCU_GPIOD,
//     .tx_port = GPIOD,
//     .tx_af = GPIO_AF_7,
//     .tx_pin = GPIO_PIN_8,
//     .rx_port = GPIOD,
//     .rx_af = GPIO_AF_7,
//     .rx_pin = GPIO_PIN_9,
//     .dma = {
//         .dma_periph = DMA0,
//         .clock = RCU_DMA0,
//         .rx_ch = DMA_CH1,
//         .rx_irq = DMA0_Channel1_IRQn,
//         .tx_ch = DMA_CH3,
//         .tx_irq = DMA0_Channel3_IRQn,
//         .sub_periph = DMA_SUBPERI4,
//     }
// };

// void USART2_IRQHandler(void)
// {
//     /* enter interrupt */
//     rt_interrupt_enter();
//     /* uart isr routine */
//     uart_isr(&serial2);
//     /* leave interrupt */
//     rt_interrupt_leave();
// }

// void DMA0_Channel3_IRQHandler(void)
// {
//     /* enter interrupt */
//     rt_interrupt_enter();
//     if (dma_interrupt_flag_get(uart2.dma.dma_periph, uart2.dma.tx_ch, DMA_INT_FLAG_FTF)) {
//         dma_tx_done_isr(&serial2);
//         dma_interrupt_flag_clear(uart2.dma.dma_periph, uart2.dma.tx_ch, DMA_INT_FLAG_FTF);
//     }
//     /* leave interrupt */
//     rt_interrupt_leave();
// }

// void DMA0_Channel1_IRQHandler(void)
// {
//     /* enter interrupt */
//     rt_interrupt_enter();
//     if (dma_interrupt_flag_get(uart2.dma.dma_periph, uart2.dma.rx_ch, DMA_INT_FLAG_FTF)) {
//         dma_rx_done_isr(&serial2);
//         dma_interrupt_flag_clear(uart2.dma.dma_periph, uart2.dma.rx_ch, DMA_INT_FLAG_FTF);
//     }
//     /* leave interrupt */
//     rt_interrupt_leave();
// }
// #endif /* USING_UART2 */

// #ifdef USING_UART0
// static struct serial_device serial3;
// static struct gd32_uart uart0 = {
//     .uart_periph = USART0,
//     .irqn = USART0_IRQn,
//     .per_clk = RCU_USART0,
//     .tx_gpio_clk = RCU_GPIOB,
//     .rx_gpio_clk = RCU_GPIOB,
//     .tx_port = GPIOB,
//     .tx_af = GPIO_AF_7,
//     .tx_pin = GPIO_PIN_6,
//     .rx_port = GPIOB,
//     .rx_af = GPIO_AF_7,
//     .rx_pin = GPIO_PIN_7,
// };

// void USART0_IRQHandler(void)
// {
//     /* enter interrupt */
//     rt_interrupt_enter();
//     /* uart isr routine */
//     uart_isr(&serial3);
//     /* leave interrupt */
//     rt_interrupt_leave();
// }
// #endif /* USING_UART0 */

// #ifdef USING_UART7
// static struct serial_device serial4;
// static struct gd32_uart uart7 = {
//     .uart_periph = UART7,
//     .irqn = UART7_IRQn,
//     .per_clk = RCU_UART7,
//     .tx_gpio_clk = RCU_GPIOE,
//     .rx_gpio_clk = RCU_GPIOE,
//     .tx_port = GPIOE,
//     .tx_af = GPIO_AF_8,
//     .tx_pin = GPIO_PIN_1,
//     .rx_port = GPIOE,
//     .rx_af = GPIO_AF_8,
//     .rx_pin = GPIO_PIN_0,
//     // .dma = {
//     //     .dma_periph = DMA0,
//     //     .clock = RCU_DMA0,
//     //     .rx_ch = DMA_CH1,
//     //     .rx_irq = DMA0_Channel1_IRQn,
//     //     .tx_ch = DMA_CH3,
//     //     .tx_irq = DMA0_Channel3_IRQn,
//     //     .sub_periph = DMA_SUBPERI4,
//     // }
// };

// void UART7_IRQHandler(void)
// {
//     /* enter interrupt */
//     rt_interrupt_enter();
//     /* uart isr routine */
//     uart_isr(&serial4);
//     /* leave interrupt */
//     rt_interrupt_leave();
// }
// #endif /* USING_UART7 */

// static void _dma_transmit(struct gd32_uart* uart, rt_uint8_t* buf, rt_size_t size)
// {
//     /* wait current transfers are finished */
//     while (READ_BIT(DMA_CHCTL(uart->dma.dma_periph, uart->dma.tx_ch), BIT(0)))
//         ;

//     dma_memory_address_config(uart->dma.dma_periph, uart->dma.tx_ch, 0, (uint32_t)buf);
//     dma_transfer_number_config(uart->dma.dma_periph, uart->dma.tx_ch, size);

//     /* enable DMA channel transfer complete interrupt */
//     dma_interrupt_enable(uart->dma.dma_periph, uart->dma.tx_ch, DMA_CHXCTL_FTFIE);
//     /* enable DMA channel7 */
//     dma_channel_enable(uart->dma.dma_periph, uart->dma.tx_ch);
// }



// static void _dma_tx_config(struct gd32_uart* uart)
// {
//     dma_single_data_parameter_struct dma_init_struct;

//     /* enable dma tx interrupt */
//     nvic_irq_enable(uart->dma.tx_irq, 0, 1);

//     dma_deinit(uart->dma.dma_periph, uart->dma.tx_ch);
//     dma_init_struct.direction = DMA_MEMORY_TO_PERIPH;
//     dma_init_struct.memory0_addr = 0x00000000U; /* will be configured later */
//     dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
//     dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
//     dma_init_struct.number = 0x00000000U; /* will be configured later */
//     dma_init_struct.periph_addr = (uint32_t)&USART_DATA(uart->uart_periph);
//     dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
//     dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
//     dma_init_struct.circular_mode = DMA_CIRCULAR_MODE_DISABLE;
//     dma_single_data_mode_init(uart->dma.dma_periph, uart->dma.tx_ch, &dma_init_struct);

//     /* configure DMA mode */
//     dma_channel_subperipheral_select(uart->dma.dma_periph, uart->dma.tx_ch, uart->dma.sub_periph);
//     /* USART DMA enable for transmission */
//     usart_dma_transmit_config(uart->uart_periph, USART_DENT_ENABLE);
// }

// static void _dma_rx_config(struct gd32_uart* uart, rt_uint8_t* buf, rt_size_t size)
// {
//     dma_single_data_parameter_struct dma_init_struct;

//     /* set expected receive length */
//     uart->dma.setting_recv_len = size;

//     nvic_irq_enable(uart->dma.rx_irq, 0, 0);
//     /* Enable USART IDLE interrupt, which is used by DMA rx */
//     usart_interrupt_enable(uart->uart_periph, USART_INT_IDLE);

//     dma_deinit(uart->dma.dma_periph, uart->dma.rx_ch);
//     dma_init_struct.direction = DMA_PERIPH_TO_MEMORY;
//     dma_init_struct.memory0_addr = (uint32_t)buf;
//     dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
//     dma_init_struct.number = size;
//     dma_init_struct.periph_addr = (uint32_t)&USART_DATA(uart->uart_periph);
//     dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
//     dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
//     dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
//     dma_init_struct.circular_mode = DMA_CIRCULAR_MODE_ENABLE;
//     dma_single_data_mode_init(uart->dma.dma_periph, uart->dma.rx_ch, &dma_init_struct);

//     /* configure DMA mode */
//     dma_channel_subperipheral_select(uart->dma.dma_periph, uart->dma.rx_ch, uart->dma.sub_periph);
//     /* enable DMA channel transfer complete interrupt */
//     dma_interrupt_enable(uart->dma.dma_periph, uart->dma.rx_ch, DMA_CHXCTL_FTFIE);
//     /* enable DMA channel */
//     dma_channel_enable(uart->dma.dma_periph, uart->dma.rx_ch);
//     /* USART DMA enable for reception */
//     usart_dma_receive_config(uart->uart_periph, USART_DENR_ENABLE);
// }

#if defined (RT_SERIAL_USING_DMA)
static void _dma_base_channel_check(struct at32_uart *instance)
{
    dma_channel_type *rx_channel = instance->dma_rx->dma_channel;
    dma_channel_type *tx_channel = instance->dma_tx->dma_channel;

    instance->dma_rx->dma_done = RT_TRUE;
    instance->dma_rx->dma_x = (dma_type *)((rt_uint32_t)rx_channel & ~0xFF);
    instance->dma_rx->channel_index = ((((rt_uint32_t)rx_channel & 0xFF) - 8) / 0x14) + 1;

    instance->dma_tx->dma_done = RT_TRUE;
    instance->dma_tx->dma_x = (dma_type *)((rt_uint32_t)tx_channel & ~0xFF);
    instance->dma_tx->channel_index = ((((rt_uint32_t)tx_channel & 0xFF) - 8) / 0x14) + 1;
}
#endif

// static void at32_uart_get_dma_config(void)
// {
// #ifdef BSP_USING_UART1
//     uart_config[UART1_INDEX].uart_dma_flag = 0;
// #ifdef BSP_UART1_RX_USING_DMA
//     uart_config[UART1_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_RX;
//     static struct dma_config uart1_dma_rx = UART1_RX_DMA_CONFIG;
//     uart_config[UART1_INDEX].dma_rx = &uart1_dma_rx;
// #endif
// #ifdef BSP_UART1_TX_USING_DMA
//     uart_config[UART1_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_TX;
//     static struct dma_config uart1_dma_tx = UART1_TX_DMA_CONFIG;
//     uart_config[UART1_INDEX].dma_tx = &uart1_dma_tx;
// #endif
// #endif

// #ifdef BSP_USING_UART2
//     uart_config[UART2_INDEX].uart_dma_flag = 0;
// #ifdef BSP_UART2_RX_USING_DMA
//     uart_config[UART2_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_RX;
//     static struct dma_config uart2_dma_rx = UART2_RX_DMA_CONFIG;
//     uart_config[UART2_INDEX].dma_rx = &uart2_dma_rx;
// #endif
// #ifdef BSP_UART2_TX_USING_DMA
//     uart_config[UART2_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_TX;
//     static struct dma_config uart2_dma_tx = UART2_TX_DMA_CONFIG;
//     uart_config[UART2_INDEX].dma_tx = &uart2_dma_tx;
// #endif
// #endif

// #ifdef BSP_USING_UART3
//     uart_config[UART3_INDEX].uart_dma_flag = 0;
// #ifdef BSP_UART3_RX_USING_DMA
//     uart_config[UART3_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_RX;
//     static struct dma_config uart3_dma_rx = UART3_RX_DMA_CONFIG;
//     uart_config[UART3_INDEX].dma_rx = &uart3_dma_rx;
// #endif
// #ifdef BSP_UART3_TX_USING_DMA
//     uart_config[UART3_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_TX;
//     static struct dma_config uart3_dma_tx = UART3_TX_DMA_CONFIG;
//     uart_config[UART3_INDEX].dma_tx = &uart3_dma_tx;
// #endif
// #endif

// #ifdef BSP_USING_UART4
//     uart_config[UART4_INDEX].uart_dma_flag = 0;
// #ifdef BSP_UART4_RX_USING_DMA
//     uart_config[UART4_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_RX;
//     static struct dma_config uart4_dma_rx = UART4_RX_DMA_CONFIG;
//     uart_config[UART4_INDEX].dma_rx = &uart4_dma_rx;
// #endif
// #ifdef BSP_UART4_TX_USING_DMA
//     uart_config[UART4_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_TX;
//     static struct dma_config uart4_dma_tx = UART4_TX_DMA_CONFIG;
//     uart_config[UART4_INDEX].dma_tx = &uart4_dma_tx;
// #endif
// #endif

// #ifdef BSP_USING_UART5
//     uart_config[UART5_INDEX].uart_dma_flag = 0;
// #ifdef BSP_UART5_RX_USING_DMA
//     uart_config[UART5_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_RX;
//     static struct dma_config uart5_dma_rx = UART5_RX_DMA_CONFIG;
//     uart_config[UART5_INDEX].dma_rx = &uart5_dma_rx;
// #endif
// #ifdef BSP_UART5_TX_USING_DMA
//     uart_config[UART5_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_TX;
//     static struct dma_config uart5_dma_tx = UART5_TX_DMA_CONFIG;
//     uart_config[UART5_INDEX].dma_tx = &uart5_dma_tx;
// #endif
// #endif

// #ifdef BSP_USING_UART6
//     uart_config[UART6_INDEX].uart_dma_flag = 0;
// #ifdef BSP_UART6_RX_USING_DMA
//     uart_config[UART6_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_RX;
//     static struct dma_config uart6_dma_rx = UART6_RX_DMA_CONFIG;
//     uart_config[UART6_INDEX].dma_rx = &uart6_dma_rx;
// #endif
// #ifdef BSP_UART6_TX_USING_DMA
//     uart_config[UART6_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_TX;
//     static struct dma_config uart6_dma_tx = UART6_TX_DMA_CONFIG;
//     uart_config[UART6_INDEX].dma_tx = &uart6_dma_tx;
// #endif
// #endif

// #ifdef BSP_USING_UART7
//     uart_config[UART7_INDEX].uart_dma_flag = 0;
// #ifdef BSP_UART7_RX_USING_DMA
//     uart_config[UART7_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_RX;
//     static struct dma_config uart7_dma_rx = UART7_RX_DMA_CONFIG;
//     uart_config[UART7_INDEX].dma_rx = &uart7_dma_rx;
// #endif
// #ifdef BSP_UART7_TX_USING_DMA
//     uart_config[UART7_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_TX;
//     static struct dma_config uart7_dma_tx = UART7_TX_DMA_CONFIG;
//     uart_config[UART7_INDEX].dma_tx = &uart7_dma_tx;
// #endif
// #endif

// #ifdef BSP_USING_UART8
//     uart_config[UART8_INDEX].uart_dma_flag = 0;
// #ifdef BSP_UART8_RX_USING_DMA
//     uart_config[UART8_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_RX;
//     static struct dma_config uart8_dma_rx = UART8_RX_DMA_CONFIG;
//     uart_config[UART8_INDEX].dma_rx = &uart8_dma_rx;
// #endif
// #ifdef BSP_UART8_TX_USING_DMA
//     uart_config[UART8_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_TX;
//     static struct dma_config uart8_dma_tx = UART8_TX_DMA_CONFIG;
//     uart_config[UART8_INDEX].dma_tx = &uart8_dma_tx;
// #endif
// #endif
// }

#ifdef RT_SERIAL_USING_DMA
static void at32_dma_config(struct serial_device *serial, rt_ubase_t flag)
{
    dma_init_type dma_init_struct;
    dma_channel_type *dma_channel = NULL;
    struct rt_serial_rx_fifo *rx_fifo;
    struct at32_uart *instance;
    struct dma_config *dma_config;

    RT_ASSERT(serial != RT_NULL);
    instance = (struct at32_uart *) serial->parent.user_data;
    RT_ASSERT(instance != RT_NULL);

    RT_ASSERT(flag == RT_DEVICE_FLAG_DMA_TX || flag == RT_DEVICE_FLAG_DMA_RX);

    if (RT_DEVICE_FLAG_DMA_RX == flag)
    {
        dma_channel = instance->dma_rx->dma_channel;
        dma_config = instance->dma_rx;
    }
    else /* RT_DEVICE_FLAG_DMA_TX == flag */
    {
        dma_channel = instance->dma_tx->dma_channel;
        dma_config = instance->dma_tx;
    }

    crm_periph_clock_enable(dma_config->dma_clock, TRUE);
    dma_default_para_init(&dma_init_struct);
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
    dma_init_struct.priority = DMA_PRIORITY_MEDIUM;

    if (RT_DEVICE_FLAG_DMA_RX == flag)
    {
        dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
        dma_init_struct.loop_mode_enable = TRUE;
    }
    else if (RT_DEVICE_FLAG_DMA_TX == flag)
    {
        dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
        dma_init_struct.loop_mode_enable = FALSE;
    }

    dma_reset(dma_channel);
    dma_init(dma_channel, &dma_init_struct);
#if defined (SOC_SERIES_AT32F425)
    dma_flexible_config(dma_config->dma_x, dma_config->flex_channel, \
                       (dma_flexible_request_type)dma_config->request_id);
#endif
#if defined (SOC_SERIES_AT32F435) || defined (SOC_SERIES_AT32F437)
    dmamux_enable(dma_config->dma_x, TRUE);
    dmamux_init(dma_config->dmamux_channel, (dmamux_requst_id_sel_type)dma_config->request_id);
#endif
    /* enable interrupt */
    if (flag == RT_DEVICE_FLAG_DMA_RX)
    {
        rx_fifo = (struct rt_serial_rx_fifo *)serial->serial_rx;
        /* start dma transfer */
        _uart_dma_receive(instance, rx_fifo->buffer, serial->config.bufsz);
    }

    /* dma irq should set in dma tx mode */
    nvic_irq_enable(dma_config->dma_irqn, 0, 0);
    nvic_irq_enable(instance->irqn, 1, 0);
}
#endif

// static void _close_usart(struct serial_device* serial)
// {
//     struct gd32_uart* uart = (struct gd32_uart*)serial->parent.user_data;

//     if (serial->parent.open_flag & RT_DEVICE_FLAG_INT_RX) {
//         /* disable interrupt */
//         usart_interrupt_disable(uart->uart_periph, USART_INT_RBNE);
//     }

//     if (serial->parent.open_flag & RT_DEVICE_FLAG_DMA_RX) {
//         /* disable DMA channel transfer complete interrupt */
//         dma_interrupt_disable(uart->dma.dma_periph, uart->dma.rx_ch, DMA_CHXCTL_FTFIE);
//         /* disable rx idle interrupt */
//         usart_interrupt_disable(uart->uart_periph, USART_INT_IDLE);
//         /* disable DMA channel */
//         dma_channel_disable(uart->dma.dma_periph, uart->dma.rx_ch);
//         /* USART DMA disable for reception */
//         usart_dma_receive_config(uart->uart_periph, USART_DENR_DISABLE);
//     }

//     if (serial->parent.open_flag & RT_DEVICE_FLAG_DMA_TX) {
//         /* disable DMA channel transfer complete interrupt */
//         dma_interrupt_disable(uart->dma.dma_periph, uart->dma.tx_ch, DMA_CHXCTL_FTFIE);
//         /* disable DMA channel */
//         dma_channel_disable(uart->dma.dma_periph, uart->dma.tx_ch);
//         /* USART DMA disable for transmission */
//         usart_dma_receive_config(uart->uart_periph, USART_DENT_ENABLE);
//     }
//     /* reset last recv index */
//     uart->dma.last_recv_index = 0;
// }

// static void gd32_uart_gpio_init(struct gd32_uart* uart)
// {
//     /* enable USART clock */
//     rcu_periph_clock_enable(uart->tx_gpio_clk);
//     rcu_periph_clock_enable(uart->rx_gpio_clk);
//     rcu_periph_clock_enable(uart->per_clk);
//     if (uart->dma.clock == RCU_DMA0 || uart->dma.clock == RCU_DMA1) {
//         /* enable DMA1 clock */
//         rcu_periph_clock_enable(uart->dma.clock);
//     }

//     /* connect port to USARTx_Tx */
//     gpio_af_set(uart->tx_port, uart->tx_af, uart->tx_pin);
//     /* connect port to USARTx_Rx */
//     gpio_af_set(uart->rx_port, uart->rx_af, uart->rx_pin);

//     /* configure USART Tx as alternate function push-pull */
//     gpio_mode_set(uart->tx_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, uart->tx_pin);
//     gpio_output_options_set(uart->tx_port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, uart->tx_pin);

//     /* configure USART Rx as alternate function push-pull */
//     gpio_mode_set(uart->rx_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, uart->rx_pin);
//     gpio_output_options_set(uart->rx_port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, uart->rx_pin);

//     NVIC_SetPriority(uart->irqn, 0);
//     NVIC_EnableIRQ(uart->irqn);
// }

static void at32_uart_gpio_init(struct at32_uart* uart)
{
    gpio_init_type gpio_init_struct;

    // #if defined (__GNUC__) && !defined (__clang__)
    // setvbuf(stdout, NULL, _IONBF, 0);
    // #endif
    /* enable the uart and gpio clock */
    crm_periph_clock_enable(uart->tx_gpio_clk, TRUE);
    crm_periph_clock_enable(uart->rx_gpio_clk, TRUE);
    crm_periph_clock_enable(uart->per_clk, TRUE);

    gpio_default_para_init(&gpio_init_struct);

    /* configure the uart tx pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = uart->tx_pin;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init((gpio_type *)uart->tx_port, &gpio_init_struct);

    gpio_init_struct.gpio_pins = uart->rx_pin;
    gpio_init((gpio_type *)uart->rx_port, &gpio_init_struct);

    gpio_pin_mux_config((gpio_type *)uart->rx_port, GPIO_PINS_SOURCE9, GPIO_MUX_7);
    gpio_pin_mux_config((gpio_type *)uart->rx_port, GPIO_PINS_SOURCE10, GPIO_MUX_7);
}

// static rt_err_t usart_configure(struct serial_device* serial, struct serial_configure* cfg)
// {
//     struct gd32_uart* uart;

//     RT_ASSERT(serial != RT_NULL);
//     RT_ASSERT(cfg != RT_NULL);

//     uart = (struct gd32_uart*)serial->parent.user_data;

//     gd32_uart_gpio_init(uart);

//     usart_baudrate_set(uart->uart_periph, cfg->baud_rate);

//     switch (cfg->data_bits) {
//     case DATA_BITS_9:
//         usart_word_length_set(uart->uart_periph, USART_WL_9BIT);
//         break;

//     default:
//         usart_word_length_set(uart->uart_periph, USART_WL_8BIT);
//         break;
//     }

//     switch (cfg->stop_bits) {
//     case STOP_BITS_2:
//         usart_stop_bit_set(uart->uart_periph, USART_STB_2BIT);
//         break;
//     default:
//         usart_stop_bit_set(uart->uart_periph, USART_STB_1BIT);
//         break;
//     }

//     switch (cfg->parity) {
//     case PARITY_ODD:
//         usart_parity_config(uart->uart_periph, USART_PM_ODD);
//         break;
//     case PARITY_EVEN:
//         usart_parity_config(uart->uart_periph, USART_PM_EVEN);
//         break;
//     default:
//         usart_parity_config(uart->uart_periph, USART_PM_NONE);
//         break;
//     }

//     usart_receive_config(uart->uart_periph, USART_RECEIVE_ENABLE);
//     usart_transmit_config(uart->uart_periph, USART_TRANSMIT_ENABLE);
//     usart_enable(uart->uart_periph);

//     return RT_EOK;
// }

static rt_err_t at32_configure(struct serial_device *serial,
    struct serial_configure *cfg) {
    struct at32_uart *instance = (struct at32_uart *) serial->parent.user_data;
    usart_data_bit_num_type data_bit;
    usart_stop_bit_num_type stop_bit;
    usart_parity_selection_type parity_mode;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    RT_ASSERT(instance != RT_NULL);

    at32_uart_gpio_init(instance);

    usart_receiver_enable(instance->uart_x, TRUE);
    usart_transmitter_enable(instance->uart_x, TRUE);

    usart_hardware_flow_control_set(instance->uart_x, USART_HARDWARE_FLOW_NONE);

    switch (cfg->data_bits) {
    case DATA_BITS_8:
        data_bit = USART_DATA_8BITS;
        break;
    case DATA_BITS_9:
        data_bit = USART_DATA_9BITS;
        break;
    default:
        data_bit = USART_DATA_8BITS;
        break;
    }

    switch (cfg->stop_bits) {
    case STOP_BITS_1:
        stop_bit = USART_STOP_1_BIT;
        break;
    case STOP_BITS_2:
        stop_bit = USART_STOP_2_BIT;
        break;
    default:
        stop_bit = USART_STOP_1_BIT;
        break;
    }

    switch (cfg->parity) {
    case PARITY_NONE:
        parity_mode = USART_PARITY_NONE;
        break;
    case PARITY_ODD:
        parity_mode = USART_PARITY_ODD;
        break;
    case PARITY_EVEN:
        parity_mode = USART_PARITY_EVEN;
        break;
    default:
        parity_mode = USART_PARITY_NONE;
        break;
    }

#ifdef RT_SERIAL_USING_DMA
    if (!(serial->parent.open_flag & RT_DEVICE_OFLAG_OPEN)) {
        instance->last_index = 0;
    }
#endif

    usart_parity_selection_config(instance->uart_x, parity_mode);
    usart_init(instance->uart_x, cfg->baud_rate, data_bit, stop_bit);
    usart_enable(instance->uart_x, TRUE);

    return RT_EOK;
}

// static rt_err_t usart_control(struct serial_device* serial, int cmd, void* arg)
// {
//     struct gd32_uart* uart;
//     rt_uint32_t ctrl_arg = (rt_uint32_t)(arg);

//     RT_ASSERT(serial != RT_NULL);
//     uart = (struct gd32_uart*)serial->parent.user_data;

//     switch (cmd) {
//     case RT_DEVICE_CTRL_CLR_INT:
//         if (ctrl_arg == RT_DEVICE_FLAG_INT_RX) {
//             /* disable rx irq */
//             NVIC_DisableIRQ(uart->irqn);
//             /* disable interrupt */
//             usart_interrupt_disable(uart->uart_periph, USART_INT_RBNE);
//         }
//         break;

//     case RT_DEVICE_CTRL_SET_INT:
//         if (ctrl_arg == RT_DEVICE_FLAG_INT_RX) {
//             /* enable rx irq */
//             NVIC_EnableIRQ(uart->irqn);
//             /* enable interrupt */
//             usart_interrupt_enable(uart->uart_periph, USART_INT_RBNE);
//         }
//         break;

//         /* USART DMA config */
//     case RT_DEVICE_CTRL_CONFIG:
//         if (ctrl_arg == RT_DEVICE_FLAG_DMA_RX) {
//             struct serial_rx_fifo* rx_fifo = (struct serial_rx_fifo*)serial->serial_rx;
//             struct gd32_uart* uart = (struct gd32_uart*)serial->parent.user_data;

//             if (READ_BIT(DMA_CHCTL(uart->dma.dma_periph, uart->dma.rx_ch), BIT(0))) {
//                 return RT_EBUSY;
//             }

//             _dma_rx_config(uart, rx_fifo->buffer, serial->config.bufsz);
//         }

//         if (ctrl_arg == RT_DEVICE_FLAG_DMA_TX) {
//             struct gd32_uart* uart = (struct gd32_uart*)serial->parent.user_data;

//             if (READ_BIT(DMA_CHCTL(uart->dma.dma_periph, uart->dma.tx_ch), 0)) {
//                 return RT_EBUSY;
//             }

//             _dma_tx_config(uart);
//         }
//         break;

//     case RT_DEVICE_CTRL_SUSPEND:
//         _close_usart(serial);
//         break;

//     default:
//         break;
//     }

//     return RT_EOK;
// }

static rt_err_t at32_control(struct serial_device *serial, int cmd, void *arg) {
    struct at32_uart *instance;

#ifdef RT_SERIAL_USING_DMA
    rt_ubase_t ctrl_arg = (rt_ubase_t)arg;
#endif

    RT_ASSERT(serial != RT_NULL);
    instance = (struct at32_uart *) serial->parent.user_data;
    RT_ASSERT(instance != RT_NULL);

    switch (cmd) {
    case RT_DEVICE_CTRL_CLR_INT:
        nvic_irq_disable(instance->irqn);
        usart_interrupt_enable(instance->uart_x, USART_RDBF_INT, FALSE);

#ifdef RT_SERIAL_USING_DMA
        /* disable DMA */
        if (ctrl_arg == RT_DEVICE_FLAG_DMA_RX)
        {
            nvic_irq_disable(instance->dma_rx->dma_irqn);
            dma_reset(instance->dma_rx->dma_channel);
        }
        else if(ctrl_arg == RT_DEVICE_FLAG_DMA_TX)
        {
            nvic_irq_disable(instance->dma_tx->dma_irqn);
            dma_reset(instance->dma_tx->dma_channel);
        }
#endif
        break;
    case RT_DEVICE_CTRL_SET_INT:
        nvic_irq_enable(instance->irqn, 1, 0);
        usart_interrupt_enable(instance->uart_x, USART_RDBF_INT, TRUE);
        break;
#ifdef RT_SERIAL_USING_DMA
    case RT_DEVICE_CTRL_CONFIG:
        at32_dma_config(serial, ctrl_arg);
        break;
#endif
    }

    return RT_EOK;
}

// static int usart_putc(struct serial_device* serial, char ch)
// {
//     struct gd32_uart* uart;

//     RT_ASSERT(serial != RT_NULL);
//     uart = (struct gd32_uart*)serial->parent.user_data;

//     usart_data_transmit(uart->uart_periph, ch);

//     /* wait transmit finish */
//     while ((usart_flag_get(uart->uart_periph, USART_FLAG_TBE) == RESET))
//         ;

//     return RT_EOK;
// }

// static int usart_getc(struct serial_device* serial)
// {
//     int ch = -1;
//     struct gd32_uart* uart;

//     RT_ASSERT(serial != RT_NULL);
//     uart = (struct gd32_uart*)serial->parent.user_data;

//     if (usart_flag_get(uart->uart_periph, USART_FLAG_RBNE) != RESET)
//         ch = usart_data_receive(uart->uart_periph);

//     return ch;
// }

static int at32_putc(struct serial_device *serial, char ch) {
    struct at32_uart *instance;

    RT_ASSERT(serial != RT_NULL);
    instance = (struct at32_uart *) serial->parent.user_data;
    RT_ASSERT(instance != RT_NULL);

    usart_data_transmit(instance->uart_x, (uint8_t)ch);
    while (usart_flag_get(instance->uart_x, USART_TDC_FLAG) == RESET);

    return 1;
}

static int at32_getc(struct serial_device *serial) {
    int ch;
    struct at32_uart *instance;

    RT_ASSERT(serial != RT_NULL);
    instance = (struct at32_uart *) serial->parent.user_data;
    RT_ASSERT(instance != RT_NULL);

    ch = -1;
    if (usart_flag_get(instance->uart_x, USART_RDBF_FLAG) != RESET) {
        ch = usart_data_receive(instance->uart_x) & 0xff;
    }

    return ch;
}

// static rt_size_t usart_dma_transmit(struct serial_device* serial, rt_uint8_t* buf, rt_size_t size, int direction)
// {
//     if (direction == SERIAL_DMA_TX) {
//         _dma_transmit(serial->parent.user_data, buf, size);
//         return size;
//     }

//     return 0;
// }

#ifdef RT_SERIAL_USING_DMA
static void _uart_dma_receive(struct at32_uart *instance, rt_uint8_t *buffer, rt_uint32_t size)
{
    dma_channel_type* dma_channel = instance->dma_rx->dma_channel;

    dma_channel->dtcnt = size;
    dma_channel->paddr = (rt_uint32_t)&(instance->uart_x->dt);
    dma_channel->maddr = (rt_uint32_t)buffer;
    /* enable usart interrupt */
    usart_interrupt_enable(instance->uart_x, USART_PERR_INT, TRUE);
    usart_interrupt_enable(instance->uart_x, USART_IDLE_INT, TRUE);
    /* enable transmit complete interrupt */
    dma_interrupt_enable(dma_channel, DMA_FDT_INT, TRUE);
    /* enable dma receive */
    usart_dma_receiver_enable(instance->uart_x, TRUE);

    /* enable dma channel */
    dma_channel_enable(dma_channel, TRUE);
}

static void _uart_dma_transmit(struct at32_uart *instance, rt_uint8_t *buffer, rt_uint32_t size)
{
    /* wait before transfer complete */
    while(instance->dma_tx->dma_done == RT_FALSE);

    dma_channel_type *dma_channel = instance->dma_tx->dma_channel;

    dma_channel->dtcnt = size;
    dma_channel->paddr = (rt_uint32_t)&(instance->uart_x->dt);
    dma_channel->maddr = (rt_uint32_t)buffer;

    /* enable transmit complete interrupt */
    dma_interrupt_enable(dma_channel, DMA_FDT_INT, TRUE);
    /* enable dma transmit */
    usart_dma_transmitter_enable(instance->uart_x, TRUE);

    /* mark dma flag */
    instance->dma_tx->dma_done = RT_FALSE;
    /* enable dma channel */
    dma_channel_enable(dma_channel, TRUE);
}
#endif

// /* usart driver operations */
// static const struct usart_ops __usart_ops = {
//     .configure = usart_configure,
//     .control = usart_control,
//     .putc = usart_putc,
//     .getc = usart_getc,
//     .dma_transmit = usart_dma_transmit,
// };

static const struct usart_ops at32_uart_ops = {
    .configure = at32_configure,
    .control = at32_control,
    .putc = at32_putc,
    .getc = at32_getc,
#ifdef RT_SERIAL_USING_DMA
    .dma_transmit = at32_dma_transmit,    
#endif
};

// rt_err_t drv_usart_init(void)
// {
//     rt_err_t rt_err = RT_EOK;
//     struct serial_configure config = SERIAL_DEFAULT_CONFIG;

// #ifdef USING_UART6
//     serial0.ops = &__usart_ops;
//     #ifdef SERIAL0_DEFAULT_CONFIG
//     struct serial_configure serial0_config = SERIAL0_DEFAULT_CONFIG;
//     serial0.config = serial0_config;
//     #else
//     serial0.config = config;
//     #endif

//     /* register serial device */
//     rt_err |= hal_serial_register(&serial0,
//                                   "serial0",
//                                   RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE | RT_DEVICE_FLAG_INT_RX,
//                                   &uart6);
// #endif /* USING_UART6 */

// #ifdef USING_UART1
//     serial1.ops = &__usart_ops;
//     #ifdef SERIAL1_DEFAULT_CONFIG
//     struct serial_configure serial1_config = SERIAL1_DEFAULT_CONFIG;
//     serial1.config = serial1_config;
//     #else
//     serial1.config = config;
//     #endif

//     /* register serial device */
//     rt_err |= hal_serial_register(&serial1,
//                                   "serial1",
//                                   RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX,
//                                   &uart1);
// #endif /* USING_UART2 */

// #ifdef USING_UART2
//     serial2.ops = &__usart_ops;
//     #ifdef SERIAL2_DEFAULT_CONFIG
//     struct serial_configure serial2_config = SERIAL2_DEFAULT_CONFIG;
//     serial2.config = serial2_config;
//     #else
//     serial2.config = config;
//     #endif

//     /* register serial device */
//     rt_err |= hal_serial_register(&serial2,
//                                   "serial2",
//                                   RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX,
//                                   &uart2);
// #endif /* USING_UART2 */

// #ifdef USING_UART0
//     serial3.ops = &__usart_ops;
//     #ifdef SERIAL3_DEFAULT_CONFIG
//     struct serial_configure serial3_config = SERIAL3_DEFAULT_CONFIG;
//     serial3.config = serial3_config;
//     #else
//     serial3.config = config;
//     #endif

//     /* register serial device */
//     rt_err |= hal_serial_register(&serial3,
//                                   "serial3",
//                                   RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE | RT_DEVICE_FLAG_INT_RX,
//                                   &uart0);
// #endif /* USING_UART0 */

// #ifdef USING_UART7
//     serial4.ops = &__usart_ops;
//     #ifdef SERIAL4_DEFAULT_CONFIG
//     struct serial_configure serial4_config = SERIAL4_DEFAULT_CONFIG;
//     serial4.config = serial4_config;
//     #else
//     serial4.config = config;
//     #endif

//     /* register serial device */
//     rt_err |= hal_serial_register(&serial4,
//                                   "serial4",
//                                   RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE | RT_DEVICE_FLAG_INT_RX,
//                                   &uart7);
// #endif /* USING_UART7 */

//     return rt_err;
// }

rt_err_t drv_usart_init(void)
{
    rt_err_t rt_err = RT_EOK;
    struct serial_configure config = SERIAL_DEFAULT_CONFIG;

#ifdef USING_UART1
    serial0.ops = &at32_uart_ops;
    #ifdef SERIAL0_DEFAULT_CONFIG
    struct serial_configure serial0_config = SERIAL0_DEFAULT_CONFIG;
    serial0.config = serial0_config;
    #else
    serial0.config = config;
    #endif

    /* register serial device */
    rt_err |= hal_serial_register(&serial0,
                                  "serial0",
                                  RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE | RT_DEVICE_FLAG_INT_RX,
                                  &uart1);
#endif /* USING_UART6 */
}

