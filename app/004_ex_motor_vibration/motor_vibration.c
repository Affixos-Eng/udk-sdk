/**
 * LEAPS - Low Energy Accurate Positioning System.
 *
 * This example will alarm the vibration from motor device
 *
 * Copyright (c) 2025, LEAPS. All rights reserved.
 *
 */

#include <example_selection.h>

#if defined(TEST_MOTOR)

#include "hal-timer.h"
#include "hal-console.h"
#include <drivers/pwm.h>

/* Define the example displayed name */
#define APP_NAME "TEST MOTTOR      "
#define SLEEP_TIME_MS              1000
#define MOTOR_PWM_ON_PER_MS        50

static const struct device *motor_dev = NULL;
static void motor_enable(void);
static void motor_alarm(uint8_t val, int intensity);

/**
 * Application entry point.
 */
int motor_vibration(void)
{
    /* Display application name on console. */
    test_run_info((unsigned char *)APP_NAME);
    /* Enable motor device */
    motor_enable();
   /* Loop to emit vibrate */
    while (1) {
        /* Generate vibration alarm */
        motor_alarm(255, 4);
        /* Waiting for 1s for next generate vibration */
        hal_delay_ms(SLEEP_TIME_MS);
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

#endif /* #if defined(TEST_MOTOR) */
