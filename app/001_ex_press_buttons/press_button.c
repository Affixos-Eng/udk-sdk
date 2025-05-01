/**
 * LEAPS - Low Energy Accurate Positioning System.
 *
 * This example just print the message on console when pressing each button
 * A, B and C respectively
 *
 * Copyright (c) 2025, LEAPS. All rights reserved.
 *
 */
#include <example_selection.h>

#if defined(TEST_BUTTONS)

/* HAL layer */
#include "hal-console.h"
#include "hal-gpio.h"
#include "hal-timer.h"

/* Define example displayed name */
#define APP_NAME "PRESS BUTTON EXAMPLE "

/* local file - variables  */
static hal_gpio_callback_t gpio_cb[3];

/* Register callback function fo each button */
static void button_A_register(hal_gpio_callback_handler_t cb);
static void button_B_register(hal_gpio_callback_handler_t cb);
static void button_C_register(hal_gpio_callback_handler_t cb);

/* Callback function for each button */
static void button_A_event(void *data);
static void button_B_event(void *data);
static void button_C_event(void *data);

/**
 * Application entry point.
 */
int press_buttons(void)
{
    /* Print example on console. */
    test_run_info((unsigned char *)APP_NAME);

    /* Enable and register callback even for each button */
    /* Name of each button - refer page:
       https://docs.leapslabs.com/udk/udk-start/device-hw-interfaces */
    button_A_register(button_A_event);
    button_B_register(button_B_event);
    button_C_register(button_C_event);

    /* Waiting for press button */
    while (1) {
    }

    /* Never reach */
    return (0);
}

/* Checking if button B is pressed or not */
static int button_B_is_released(void)
{
    return (hal_gpio_pin_read(DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(sw2), gpios)),
                    DT_GPIO_PIN(DT_ALIAS(sw2), gpios)) != 0);
}

/* Checking if button B is pressed or not */
static int button_A_is_released(void)
{
    return (hal_gpio_pin_read(DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(sw1), gpios)),
                    DT_GPIO_PIN(DT_ALIAS(sw1), gpios)) != 0);
}

/* Checking if button B is pressed or not */
static int button_C_is_released(void)
{
    return (hal_gpio_pin_read(DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(sw0), gpios)),
                    DT_GPIO_PIN(DT_ALIAS(sw0), gpios)) != 0);
}

/* Callback function for button A */
static void button_A_event(void *data)
{
    if (button_A_is_released()) {
        printk("Button A is released at %" PRIu32 "\n", hal_sys_tick());
    } else {
        printk("Button A is pressed at %" PRIu32 "\n", hal_sys_tick());
    }
}

/* Callback function for button B */
static void button_B_event(void *data)
{
    if (button_B_is_released()) {
        printk("Button B is released at %" PRIu32 "\n", hal_sys_tick());
    } else {
        printk("Button B is pressed at %" PRIu32 "\n", hal_sys_tick());
    }

}

/* Callback function for button C */
static void button_C_event(void *data)
{
    if (button_C_is_released()) {
        printk("Button C is released at %" PRIu32 "\n", hal_sys_tick());
    } else {
        printk("Button C is pressed at %" PRIu32 "\n", hal_sys_tick());
    }
}

/* Register callback function for button A */
static void button_A_register(hal_gpio_callback_handler_t cb)
{
    hal_gpio_callback_init(cb, NULL, &gpio_cb[2]);
    /* Callback is called for both press and release button */
    hal_gpio_callback_register(DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(sw1), gpios)),
            DT_GPIO_PIN(DT_ALIAS(sw1), gpios), GPIO_PULLUP,
            IRQ_TYPE_EDGE_BOTH, &gpio_cb[2]);
}

/* Register callback function for button B */
static void button_B_register(hal_gpio_callback_handler_t cb)
{
    hal_gpio_callback_init(cb, NULL, &gpio_cb[1]);
    /* Callback is called for both press and release button */
    hal_gpio_callback_register(DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(sw2), gpios)),
            DT_GPIO_PIN(DT_ALIAS(sw2), gpios), GPIO_PULLUP,
            IRQ_TYPE_EDGE_BOTH, &gpio_cb[1]);
}

/* Register callback function for button C */
static void button_C_register(hal_gpio_callback_handler_t cb)
{
    hal_gpio_callback_init(cb, NULL, &gpio_cb[0]);
    /* Callback is called for both press and release button */
    hal_gpio_callback_register(DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(sw0), gpios)),
            DT_GPIO_PIN(DT_ALIAS(sw0), gpios), GPIO_PULLUP,
            IRQ_TYPE_EDGE_BOTH, &gpio_cb[0]);
}

#endif /* #if defined(TEST_BUTTONS) */
