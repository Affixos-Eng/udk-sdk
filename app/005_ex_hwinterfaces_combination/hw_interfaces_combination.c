/**
 * LEAPS - Low Energy Accurate Positioning System.
 *
 * This example contains some combination features of supported hardware
 * interface on board:
 * - Press button A - All the LEDs will be toggle consecutively
 * - Press button B - The buzzer will emit the beep sound
 * - Press button C - The motor will generate vibration
 *
 * Copyright (c) 2025, LEAPS. All rights reserved.
 *
 */

#include <example_selection.h>

#if defined(TEST_HW_INTERFACE_COMBINATION)

#include "hal-gpio.h"
#include "hal-timer.h"
#include "hal-console.h"
#include <drivers/pwm.h>

/* Define the example displayed name */
#define APP_NAME "TEST IO COMBINATION      "

#define SLEEP_TIME_MS                      1000
#define TOGGLE_PERIOD                      50
#define SCHEDULED_PERIOD                   5
#define MOTOR_PWM_ON_PER_MS                50

/* Local variable */
static const struct device *buzzer_dev = NULL;
static const struct device *motor_dev = NULL;

static hal_gpio_callback_t gpio_cb[3];
static bool button_A_is_pressed = false;
static bool button_B_is_pressed = false;
static bool button_C_is_pressed = false;

/* Local functions */
static void leds_enable_all(void);
static void leds_toggle_all(void);

static void motor_enable(void);
static void motor_alarm(uint8_t val, int intensity);

static void buzzer_enable(void);
static void buzzer_emit_sound(void);

static void button_A_event(void *data);
static void button_B_event(void *data);
static void button_C_event(void *data);

static void button_A_register(hal_gpio_callback_handler_t cb);
static void button_B_register(hal_gpio_callback_handler_t cb);
static void button_C_register(hal_gpio_callback_handler_t cb);

/* To keep the sequence call of Qorvo's example code is unchanged */
extern void test_run_info(unsigned char *data);

/**
 * Application entry point.
 */
int test_combination(void)
{
    /* Display application name on LCD. */
    test_run_info((unsigned char *)APP_NAME);
    /* Enable all supported lebs */
    leds_enable_all();
    /* Register the callback function for each button */
    button_A_register(button_A_event);
    button_B_register(button_B_event);
    button_C_register(button_C_event);
    /* Enable the motor device */
    motor_enable();
    /* Enable the buzzer device */
    buzzer_enable();
    /* Waiting for press buttons */
    while (1) {
        /* Checking the event every 5ms */
        hal_delay_ms(SCHEDULED_PERIOD);
        /* If each button is pressed */
        if (button_A_is_pressed == true) {
            leds_toggle_all();
            /* Reset button state */
            button_A_is_pressed = false;
        }
        if (button_B_is_pressed == true) {
            buzzer_emit_sound();
            /* Reset button state */
            button_B_is_pressed = false;
        }
        if (button_C_is_pressed == true) {
            /* Generate vibration alarm */
            motor_alarm(255, 4);
            /* Reset button state */
            button_C_is_pressed = false;
        }
    }
    /* Never reach */
    return 0;
}

/* Enable all supported lebs on board */
static void leds_enable_all(void)
{
    /* Set related pin for Red LED as OUTPUT */
    hal_gpio_pin_configure_output(
        DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(ledr), gpios)),
        DT_GPIO_PIN(DT_ALIAS(ledr), gpios), 0);
    /* Set related pin for Green as OUTPUT */
    hal_gpio_pin_configure_output(
        DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(ledg), gpios)),
        DT_GPIO_PIN(DT_ALIAS(ledg), gpios), 0);
    /* Set related pin for Blue LED as OUTPUT */
    hal_gpio_pin_configure_output(
        DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(ledb), gpios)),
        DT_GPIO_PIN(DT_ALIAS(ledb), gpios), 0);
}

/* Enable the buzzer device */
static void buzzer_enable(void)
{
    if (buzzer_dev == NULL) {
        buzzer_dev = \
        device_get_binding(DT_LABEL(DT_PWMS_CTLR(DT_ALIAS(pwm_buzzer0))));
        if (buzzer_dev) {
        }
    }
}

/* Enable the motor device */
static void motor_enable(void)
{
    if (motor_dev == NULL) {
        motor_dev = \
        device_get_binding(DT_LABEL(DT_PWMS_CTLR(DT_ALIAS(pwm_motor0))));
        if (motor_dev) {
        }
    }
}

/* Return if button is pressed or not */
static int button_B_is_released(void)
{
    return (hal_gpio_pin_read(DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(sw2), gpios)),
                    DT_GPIO_PIN(DT_ALIAS(sw2), gpios)) != 0);
}

static int button_A_is_released(void)
{
    return (hal_gpio_pin_read(DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(sw1), gpios)),
                    DT_GPIO_PIN(DT_ALIAS(sw1), gpios)) != 0);
}

static int button_C_is_released(void)
{
    return (hal_gpio_pin_read(DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(sw0), gpios)),
                    DT_GPIO_PIN(DT_ALIAS(sw0), gpios)) != 0);
}

/* Register the callback function for each button */
static void button_A_register(hal_gpio_callback_handler_t cb)
{
    hal_gpio_callback_init(cb, NULL, &gpio_cb[2]);
    /* Callback is called for both press and release button */
    hal_gpio_callback_register(DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(sw1), gpios)),
            DT_GPIO_PIN(DT_ALIAS(sw1), gpios), GPIO_PULLUP,
            IRQ_TYPE_EDGE_BOTH, &gpio_cb[2]);
}

static void button_B_register(hal_gpio_callback_handler_t cb)
{
    hal_gpio_callback_init(cb, NULL, &gpio_cb[1]);
    /* Callback is called for both press and release button */
    hal_gpio_callback_register(DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(sw2), gpios)),
            DT_GPIO_PIN(DT_ALIAS(sw2), gpios), GPIO_PULLUP,
            IRQ_TYPE_EDGE_BOTH, &gpio_cb[1]);
}

static void button_C_register(hal_gpio_callback_handler_t cb)
{
    hal_gpio_callback_init(cb, NULL, &gpio_cb[0]);
    /* Callback is called for both press and release button */
    hal_gpio_callback_register(DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(sw0), gpios)),
            DT_GPIO_PIN(DT_ALIAS(sw0), gpios), GPIO_PULLUP,
            IRQ_TYPE_EDGE_BOTH, &gpio_cb[0]);
}

/* Even function for front button */
static void button_A_event(void *data)
{
    if (button_A_is_released()) {
    } else {
        if (button_A_is_pressed == false)
        {
            button_A_is_pressed = true;
        }
    }
}

static void button_B_event(void *data)
{
    if (button_B_is_released()) {
    } else {
        if (button_B_is_pressed == false)
        {
            button_B_is_pressed = true;
        }
    }

}

static void button_C_event(void *data)
{
    if (button_C_is_released()) {
    } else {
        if (button_C_is_pressed == false)
        {
            button_C_is_pressed = true;
        }
    }
}

/* Setting the vibration of motor*/
static void motor_vibration_setting(uint32_t per_us, uint32_t pulse_us)
{
    if (motor_dev) {
        /* Generate the PWM to generate vibration of motor */
        pwm_set_cycles(motor_dev, DT_PWMS_CHANNEL(DT_ALIAS(pwm_motor0)),
        per_us, pulse_us, 0);
    }
}

/* Generate alarm (vibration) on/off */
static void motor_alarm(uint8_t val, int intensity)
{
    if ((val) && (intensity > 0)) {
        /* Start generate vibration */
        motor_vibration_setting(500, 350 + val * 150 / 255);
        hal_thread_delay(MOTOR_PWM_ON_PER_MS * intensity);
        /* Stop generate vibration */
        motor_vibration_setting(500, 0);
        hal_thread_delay(MOTOR_PWM_ON_PER_MS * intensity);
        /* Start generate vibration */
        motor_vibration_setting(500, 250 + val * 250 / 255);
        hal_thread_delay(MOTOR_PWM_ON_PER_MS * intensity / 2);
        /* Stop generate vibration */
        motor_vibration_setting(500, 0);
    } else {
        /* Stop generate vibration */
        motor_vibration_setting(500, 0);
    }
}

/* Toggle all LEDs */
static void leds_toggle_all (void)
{
    /* Toggle blue LED */
    for (int i = 0; i < 6; i++) {
        hal_gpio_pin_toggle(DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(ledb), gpios)),
        DT_GPIO_PIN(DT_ALIAS(ledb), gpios));
        hal_delay_ms(TOGGLE_PERIOD);
    }
    /* Toggle red LED */
    for (int i = 0; i < 6; i++) {
        hal_gpio_pin_toggle(DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(ledr), gpios)),
        DT_GPIO_PIN(DT_ALIAS(ledr), gpios));
        hal_delay_ms(TOGGLE_PERIOD);
    }
    /* Toggle green LED */
    for (int i = 0; i < 6; i++) {
        hal_gpio_pin_toggle(DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(ledg), gpios)),
        DT_GPIO_PIN(DT_ALIAS(ledg), gpios));
        hal_delay_ms(TOGGLE_PERIOD);
    }
}

/* Set the generated PWM to emit beep sound */
static void buzzer_setting_sound(uint32_t per_us, uint32_t pulse_us)
{
    if (buzzer_dev) {
        pwm_set_cycles(buzzer_dev, DT_PWMS_CHANNEL(DT_ALIAS(pwm_buzzer0)),
        per_us, pulse_us, 0);
    }
}

/* Buzzer emit beep sound */
static void buzzer_emit_sound(void)
{
    /* Start generate beep sound */
    for (int i = 0; i < 6; i++) {
        if (i % 2) {
            /* Set PWM duty = 1/2 period to generate beep sound */
            buzzer_setting_sound(10000,5000);
        } else {
            /* Set inactive level to stop generate beep sound */
            buzzer_setting_sound(10000,0);
        }
        /* Intensity for each beep sound */
        hal_delay_ms(80);
    }
    /* Set inactive level to stop generate beep sound */
    buzzer_setting_sound(10000,0);
}

#endif /* #if defined(TEST_HW_INTERFACE_COMBINATION) */
