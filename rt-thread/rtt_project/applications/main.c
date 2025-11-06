/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-10-19     Nations      first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

/* defined the D1 pin: PD0 */
#define LED1_PIN GET_PIN(C, 13)
#define LED2_PIN GET_PIN(A, 11)

static rt_uint8_t led0_stack[512], led1_stack[512];
static struct rt_thread led0_thread;
static struct rt_thread led1_thread;
/**
 * @brief  led0 thread entry
 */
static void led0_thread_entry(void *parameter)
{
    while (1)
    {
        rt_thread_delay(500); // delay 500ms
        rt_pin_write(LED1_PIN, PIN_HIGH);
        rt_thread_delay(500); // delay 500ms
        rt_pin_write(LED1_PIN, PIN_LOW);
    }
}

/**
 * @brief  led1 thread entry
 */
static void led1_thread_entry(void *parameter)
{
    while (1)
    {
        rt_thread_delay(250); // delay 250ms
        rt_pin_write(LED2_PIN, PIN_HIGH);
        rt_thread_delay(250); // delay 250ms
        rt_pin_write(LED2_PIN, PIN_LOW);
    }
}

int main(void)
{
    rt_err_t result;
    /* init led0 thread */
     rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);
     rt_pin_mode(LED2_PIN, PIN_MODE_OUTPUT);
    result = rt_thread_init(&led0_thread, "led0", led0_thread_entry, RT_NULL, (rt_uint8_t *)&led0_stack[0], sizeof(led0_stack), 4, 5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&led0_thread);
    }
    /* init led1 thread */
    result = rt_thread_init(&led1_thread, "led1", led1_thread_entry, RT_NULL, (rt_uint8_t *)&led1_stack[0], sizeof(led1_stack), 5, 5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&led1_thread);
    }

    while (1)
    {

        rt_thread_mdelay(500);
    }
}
