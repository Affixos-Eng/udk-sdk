/**
 * LEAPS - Low Energy Accurate Positioning System.
 *
 * Hardware Abstraction Layer - GPIO.
 *
 * Copyright (c) 2025, LEAPS. All rights reserved.
 *
 */

#include "hal-gpio.h"

/* Local function */
static void gpio_callback_handler(const struct device *port,
                                  struct gpio_callback *internal, uint32_t pins)
{
    hal_gpio_callback_t *cb = CONTAINER_OF(internal,
                                                 hal_gpio_callback_t, internal);

    if (cb->user_handler) {
        cb->user_handler(cb->user_data);
    }
}

/* Described in header file */
void hal_gpio_callback_init(hal_gpio_callback_handler_t *user_handler,
                                 void *user_data, hal_gpio_callback_t *callback)
{
    callback->user_handler = user_handler;
    callback->user_data = user_data;
}

/* Described in header file */
void hal_gpio_callback_register(const char *controller, uint32_t pin,
                                                  hal_pullup_t pullup,
                                                  hal_irq_type_t irq_type,
                                                  hal_gpio_callback_t *callback)
{
    callback->dev = device_get_binding(controller);

    (void)gpio_pin_configure(callback->dev, pin, GPIO_INPUT | pullup);

    (void)gpio_pin_interrupt_configure(callback->dev, pin,
            ((irq_type == IRQ_TYPE_EDGE_FALLING) ? GPIO_INT_EDGE_TO_INACTIVE :
             (irq_type == IRQ_TYPE_EDGE_RISING) ? GPIO_INT_EDGE_TO_ACTIVE :
             GPIO_INT_EDGE_BOTH));

    gpio_init_callback(&callback->internal, gpio_callback_handler, BIT(pin));

    gpio_add_callback(callback->dev, &callback->internal);
}

/* Described in header file */
int hal_gpio_pin_configure_output(const char *controller, uint32_t pin,
                                                                 uint32_t value)
{
    int ret;
    const struct device *dev;

    dev = device_get_binding(controller);

    ret = gpio_pin_configure(dev, pin, GPIO_OUTPUT);

    if (ret) {
        return ret;
    }

    return gpio_pin_set(dev, pin, value);
}

/* Described in header file */
int hal_gpio_pin_read(const char *controller, uint32_t pin)
{
    int value;
    const struct device *dev;

    dev = device_get_binding(controller);

    value  = gpio_pin_get(dev, pin);

    return value;
}

/* Described in header file */
int hal_gpio_pin_toggle(const char *controller, uint32_t pin)
{
    const struct device *dev;

    dev = device_get_binding(controller);

    return gpio_pin_toggle(dev, pin);
}

/* Described in header file */
void hal_gpio_pin_reset_DWIC(void)
{
    const struct device *dev_gpio = device_get_binding(
                           DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(uwb0_reset), gpios)));

    gpio_pin_configure(dev_gpio, DT_GPIO_PIN(DT_ALIAS(uwb0_reset), gpios),
                                                    GPIO_OUTPUT | GPIO_PULL_UP);

    gpio_pin_set(dev_gpio, DT_GPIO_PIN(DT_ALIAS(uwb0_reset), gpios), 0);

    k_sleep(K_MSEC(2));

    gpio_pin_configure(dev_gpio, DT_GPIO_PIN(DT_ALIAS(uwb0_reset), gpios),
                                                                    GPIO_INPUT);

    k_sleep(K_MSEC(2));
}

/*------------------------------- End of file --------------------------------*/
