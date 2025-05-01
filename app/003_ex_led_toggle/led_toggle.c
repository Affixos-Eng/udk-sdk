/**
 * LEAPS - Low Energy Accurate Positioning System.
 *
 * This example will toggle all available LEDs on board with the order:
 * - Blue LED
 * - Red LED
 * - Green LED
 *
 * Copyright (c) 2025, LEAPS. All rights reserved.
 *
 */

#include <example_selection.h>

#if defined(TEST_LED)

#include "hal-gpio.h"
#include "hal-timer.h"
#include "hal-console.h"

/* Define example displayed name */
#define APP_NAME "TEST LED      "
#define SLEEP_TIME_MS           500

static void led_blue_toggle(void);
static void led_red_toggle(void);
static void led_green_toggle(void);

/**
 * Application entry point.
 */
int led_toggle(void)
{
    /* Display application name on console. */
    test_run_info((unsigned char *)APP_NAME);

    /* Set related pin for Red LED as OUTPUT */
    hal_gpio_pin_configure_output(DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(ledr), gpios)),
                                        DT_GPIO_PIN(DT_ALIAS(ledr), gpios), 0);

    /* Set related pin for Green LED as OUTPUT */
    hal_gpio_pin_configure_output(DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(ledg), gpios)),
                                         DT_GPIO_PIN(DT_ALIAS(ledg), gpios), 0);

    /* Set related pin for Blue LED as OUTPUT */
    hal_gpio_pin_configure_output(DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(ledb), gpios)),
                                         DT_GPIO_PIN(DT_ALIAS(ledb), gpios), 0);

    /* Waiting for the leb toggle */
    while (1) {
        /* Toggle blue LED */
        for (int i = 0; i < 6; i++) {
            led_blue_toggle();
            hal_delay_ms(SLEEP_TIME_MS);
        }
        /* Toggle Red LED */
        for (int i = 0; i < 6; i++) {
            led_red_toggle();
            hal_delay_ms(SLEEP_TIME_MS);
        }
        /* Toggle Green LED */
        for (int i = 0; i < 6; i++) {
            led_green_toggle();
            hal_delay_ms(SLEEP_TIME_MS);
        }
    }
}

static void led_red_toggle(void)
{
    hal_gpio_pin_toggle(DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(ledr), gpios)),
    DT_GPIO_PIN(DT_ALIAS(ledr), gpios));
}

static void led_green_toggle(void)
{
    hal_gpio_pin_toggle(DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(ledg), gpios)),
    DT_GPIO_PIN(DT_ALIAS(ledg), gpios));
}

static void led_blue_toggle(void)
{
    hal_gpio_pin_toggle(DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(ledb), gpios)),
    DT_GPIO_PIN(DT_ALIAS(ledb), gpios));
}

#endif /* #if defined(TEST_LED) */
