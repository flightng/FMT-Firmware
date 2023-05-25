/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-05-25     gg           first version
 */
#include <firmament.h>

#include "hal/actuator/actuator.h"

// #define DRV_DBG(...) console_printf(__VA_ARGS__)
#define DRV_DBG(...)

#define PWM_FREQ_50HZ  (50)
#define PWM_FREQ_125HZ (125)
#define PWM_FREQ_250HZ (250)
#define PWM_FREQ_400HZ (400)

#define MAX_PWM_OUT_CHAN      4            // Main Out has 10 pwm channel
#define TIMER_FREQUENCY       2500000       // Timer frequency: 2.5M
#define PWM_DEFAULT_FREQUENCY PWM_FREQ_50HZ // pwm default frequqncy
#define VAL_TO_DC(_val)       ((float)(_val * __pwm_freq) / 1000000.0f)
#define DC_TO_VAL(_dc)        (1000000.0f / __pwm_freq * _dc)

#define PWM_ARR(freq) (TIMER_FREQUENCY / freq) // CCR reload value, Timer frequency = TIMER_FREQUENCY/(PWM_ARR+1)

static rt_err_t pwm_config(actuator_dev_t dev, const struct actuator_configure* cfg);
static rt_err_t pwm_control(actuator_dev_t dev, int cmd, void* arg);
static rt_size_t pwm_read(actuator_dev_t dev, rt_uint16_t chan_sel, rt_uint16_t* chan_val, rt_size_t size);
static rt_size_t pwm_write(actuator_dev_t dev, rt_uint16_t chan_sel, const rt_uint16_t* chan_val, rt_size_t size);

const static struct actuator_ops __act_ops = {
    .act_config = pwm_config,
    .act_control = pwm_control,
    .act_read = pwm_read,
    .act_write = pwm_write
};

static struct actuator_device act_dev = {
    .chan_mask = 0x3FF,
    .range = { 1000, 2000 },
    .config = {
        .protocol = ACT_PROTOCOL_PWM,
        .chan_num = MAX_PWM_OUT_CHAN,
        .pwm_config = { .pwm_freq = 50 },
        .dshot_config = { 0 } },
    .ops = &__act_ops
};

static uint32_t __pwm_freq = PWM_DEFAULT_FREQUENCY;
static float __pwm_dc[MAX_PWM_OUT_CHAN];

static void pwm_gpio_init(void)
{
    gpio_init_type gpio_init_struct;
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    gpio_init_struct.gpio_pins = GPIO_PINS_6|GPIO_PINS_7|GPIO_PINS_8|GPIO_PINS_9;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOB, &gpio_init_struct);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE6, GPIO_MUX_2);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE7, GPIO_MUX_2);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE8, GPIO_MUX_2);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE9, GPIO_MUX_2);
    gpio_bits_set(GPIOB,GPIO_PINS_6);
    gpio_bits_set(GPIOB,GPIO_PINS_7);
    gpio_bits_set(GPIOB,GPIO_PINS_8);
    gpio_bits_set(GPIOB,GPIO_PINS_9);
}

static void pwm_timer_init(void)
{
    uint16_t prescalervalue = 0;
    tmr_output_config_type tmr_oc_init_structure;
    crm_clocks_freq_type crm_clocks_freq_struct = {0};
      /* TMR4 clock enable */
    crm_periph_clock_enable(CRM_TMR4_PERIPH_CLOCK, TRUE);
    /* get system clock */
    crm_clocks_freq_get(&crm_clocks_freq_struct);
    /* compute the prescaler value */
    prescalervalue = crm_clocks_freq_struct.apb1_freq  / TIMER_FREQUENCY - 1;

    /* TMR4 time base configuration */
    tmr_base_init(TMR4, (PWM_ARR(__pwm_freq) - 1), prescalervalue);
    tmr_cnt_dir_set(TMR4, TMR_COUNT_UP);
    tmr_clock_source_div_set(TMR4, TMR_CLOCK_DIV1);

    tmr_output_default_para_init(&tmr_oc_init_structure);
    tmr_oc_init_structure.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    tmr_oc_init_structure.oc_idle_state = FALSE;
    tmr_oc_init_structure.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_oc_init_structure.oc_output_state = TRUE;

    tmr_output_channel_config(TMR4, TMR_SELECT_CHANNEL_1, &tmr_oc_init_structure);
    tmr_output_channel_buffer_enable(TMR4, TMR_SELECT_CHANNEL_1, TRUE);
    tmr_output_channel_config(TMR4, TMR_SELECT_CHANNEL_2, &tmr_oc_init_structure);
    tmr_output_channel_buffer_enable(TMR4, TMR_SELECT_CHANNEL_2, TRUE);
    tmr_output_channel_config(TMR4, TMR_SELECT_CHANNEL_3, &tmr_oc_init_structure);
    tmr_output_channel_buffer_enable(TMR4, TMR_SELECT_CHANNEL_3, TRUE);
    tmr_output_channel_config(TMR4, TMR_SELECT_CHANNEL_4, &tmr_oc_init_structure);
    tmr_output_channel_buffer_enable(TMR4, TMR_SELECT_CHANNEL_4, TRUE);

    tmr_period_buffer_enable(TMR4, TRUE);
    tmr_counter_enable(TMR4, TRUE);
}

rt_inline void __read_pwm(uint8_t chan_id, float* dc)
{
    *dc = __pwm_dc[chan_id];
}

rt_inline void __write_pwm(uint8_t chan_id, float dc)
{
    switch (chan_id) {
    case 0:
        tmr_channel_value_set(TMR4, TMR_SELECT_CHANNEL_1, PWM_ARR(__pwm_freq) * dc - 1);
        break;
    case 1:
        tmr_channel_value_set(TMR4, TMR_SELECT_CHANNEL_2, PWM_ARR(__pwm_freq) * dc - 1);
        break;
    case 2:
        tmr_channel_value_set(TMR4, TMR_SELECT_CHANNEL_3, PWM_ARR(__pwm_freq) * dc - 1);
        break;
    case 3:
        tmr_channel_value_set(TMR4, TMR_SELECT_CHANNEL_4, PWM_ARR(__pwm_freq) * dc - 1);
        break;
    default:
        return;
    }
    __pwm_dc[chan_id] = dc;
}

static rt_err_t __set_pwm_frequency(uint16_t freq)
{
    if (freq < PWM_FREQ_50HZ || freq > PWM_FREQ_400HZ) {
        /* invalid frequency */
        return RT_EINVAL;
    }

    __pwm_freq = freq;
    tmr_period_value_set(TMR4,PWM_ARR(__pwm_freq) - 1);
    /* the timer compare value should be re-configured */
    for (uint8_t i = 0; i < MAX_PWM_OUT_CHAN; i++) {
        __write_pwm(i, __pwm_dc[i]);
    }

    return RT_EOK;
}

static rt_err_t pwm_config(actuator_dev_t dev, const struct actuator_configure* cfg)
{
    DRV_DBG("aux out configured: pwm frequency:%d\n", cfg->pwm_config.pwm_freq);

    if (__set_pwm_frequency(cfg->pwm_config.pwm_freq) != RT_EOK) {
        return RT_ERROR;
    }
    /* update device configuration */
    dev->config = *cfg;

    return RT_EOK;
}

static rt_err_t pwm_control(actuator_dev_t dev, int cmd, void* arg)
{
    rt_err_t ret = RT_EOK;

    switch (cmd) {
    case ACT_CMD_CHANNEL_ENABLE:
        /* set to lowest pwm before open */
        for (uint8_t i = 0; i < MAX_PWM_OUT_CHAN; i++) {
            __write_pwm(i, VAL_TO_DC(act_dev.range[0]));
        }

        /* auto-reload preload enable */
        tmr_output_enable(TMR4, TRUE);
        break;
    case ACT_CMD_CHANNEL_DISABLE:
        /* auto-reload preload disable */
        tmr_output_enable(TMR4, FALSE);
        break;
    case ACT_CMD_SET_PROTOCOL:
        /* TODO: Support dshot */
        ret = RT_EINVAL;
        break;
    default:
        ret = RT_EINVAL;
        break;
    }

    return ret;
}

static rt_size_t pwm_read(actuator_dev_t dev, rt_uint16_t chan_sel, rt_uint16_t* chan_val, rt_size_t size)
{
    rt_uint16_t* index = chan_val;
    float dc;

    for (uint8_t i = 0; i < MAX_PWM_OUT_CHAN; i++) {
        if (chan_sel & (1 << i)) {
            __read_pwm(i, &dc);
            *index = DC_TO_VAL(dc);
            index++;
        }
    }

    return size;
}

static rt_size_t pwm_write(actuator_dev_t dev, rt_uint16_t chan_sel, const rt_uint16_t* chan_val, rt_size_t size)
{
    const rt_uint16_t* index = chan_val;
    rt_uint16_t val;
    float dc;

    for (uint8_t i = 0; i < MAX_PWM_OUT_CHAN; i++) {
        if (chan_sel & (1 << i)) {
            val = *index;
            /* calculate pwm duty cycle */
            dc = VAL_TO_DC(val);
            /* update pwm signal */
            __write_pwm(i, dc);

            index++;
        }
    }

    return size;
}

rt_err_t drv_pwm_init(void)
{
    /* init pwm gpio pin */
    pwm_gpio_init();
    /* init pwm timer, pwm output mode */
    pwm_timer_init();

    /* register actuator hal device */
    return hal_actuator_register(&act_dev, "main_out", RT_DEVICE_FLAG_RDWR, NULL);
}