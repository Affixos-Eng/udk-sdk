/**
 * LEAPS - Low Energy Accurate Positioning System.
 *
 * This example will emit the beep sound from buzzer device
 *
 * Copyright (c) 2025, LEAPS. All rights reserved.
 *
 */

#include <example_selection.h>

#if defined(TEST_BUZZER)

/* HAL layer */
#include "hal-console.h"
#include "hal-timer.h"
/* Driver layer */
#include <drivers/pwm.h>

/* Define the example displayed name */
#define APP_NAME "TEST BUZZER EXAMPLE "
#define SLEEP_TIME_MS           1000
#define INTENSITY_SOUND         80

static void buzzer_setting_sound(uint32_t per_us, uint32_t pulse_us);
static void buzzer_enable(void);

static const struct device *buzzer_dev = NULL;

/**
 * Application entry point.
 */
int buzzer_alarm(void)
{
    /* Display application name on LCD. */
    test_run_info((unsigned char *)APP_NAME);
    /* Enable buzzer device */
    buzzer_enable();
    /* Loop to generate beep sound */
    while (1) {
        /* Waiting for 1s for emit beep sound again */
        hal_delay_ms(SLEEP_TIME_MS);
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
            hal_delay_ms(INTENSITY_SOUND);
        }
        /* Set inactive level to stop generate beep sound */
        buzzer_setting_sound(10000,0);
    }

    return 0;
}

/* Enable the buzzer device */
static void buzzer_enable(void)
{
    if (buzzer_dev == NULL) {
        buzzer_dev = device_get_binding(
                                 DT_LABEL(DT_PWMS_CTLR(DT_ALIAS(pwm_buzzer0))));
        if (buzzer_dev) {
            /* Do nothing */
        }
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

#endif /* #if defined(TEST_BUZZER) */
