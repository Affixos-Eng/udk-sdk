/**
 * LEAPS - Low Energy Accurate Positioning System.
 *
 * Hardware Abstraction Layer - GPIO.
 *
 * Copyright (c) 2025, LEAPS. All rights reserved.
 *
 */

#ifndef _HAL_GPIO_H_
#define _HAL_GPIO_H_

#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/pwm.h>

/* Wrapper the reset function to keep the Qorvo's example ís unchnage in
   sequence call */
#define reset_DWIC                                       hal_gpio_pin_reset_DWIC

/* Interrupt type */
typedef enum {
    IRQ_TYPE_EDGE_RISING = 0x01,
    IRQ_TYPE_EDGE_FALLING = 0x02,
    IRQ_TYPE_EDGE_BOTH = 0x03
} hal_irq_type_t;

/* Pull up/down type */
typedef enum {
    GPIO_PULLDOWN = GPIO_PULL_DOWN,
    GPIO_PULLUP = GPIO_PULL_UP,
    GPIO_NOPULL = 0
} hal_pullup_t;

/* Callback function pointer */
typedef void hal_gpio_callback_handler_t(void* data);

/* Callback type */
typedef struct {
    const struct device *dev;
    struct gpio_callback internal;
    gpio_callback_handler_t internal_handler;
    hal_gpio_callback_handler_t *user_handler;
    void* user_data;
} hal_gpio_callback_t;

/**
 * @brief Initialize a callback structure with user's handler and user's data
 *
 * @param[in] user_handler Pointer to user's handler
 * @param[in] user_data    Pointer to user's data
 * @param[in] callback     Pointer to callback structure
 *
 * @return none.
 */
void hal_gpio_callback_init(hal_gpio_callback_handler_t *user_handler,
                                void *user_data, hal_gpio_callback_t *callback);

/**
 * @brief Register an interrupt for a GPIO using the callback structure
 *
 * @param[in] controller Pointer to GPIO controller
 * @param[in] pin        GPIO pin number
 * @param[in] pullup     GPIO pull up configuration
 * @param[in] irq_type   Type of IRQ
 * @param[in] callback   Pointer to callback structure
 *
 * @return none.
 */
void hal_gpio_callback_register(const char *controller, uint32_t pin,
                                   hal_pullup_t pullup, hal_irq_type_t irq_type,
                                                 hal_gpio_callback_t *callback);

/**
 * @brief Configure a GPIO as an output pin
 *
 * @param[in] controller Pointer to GPIO controller
 * @param[in] pin        GPIO pin number
 * @param[in] value      Output value
 *
 * @return Returns a negative value on an error else returns zero
 */
int hal_gpio_pin_configure_output(const char *controller, uint32_t pin,
                                                                uint32_t value);

/**
 * @brief Read value of a GPIO pin
 *
 * @param[in] controller Pointer to GPIO controller
 * @param[in] pin        GPIO pin number
 *
 * @return Returns value of the pin
 */
int hal_gpio_pin_read(const char *controller, uint32_t pin);

/**
 * @brief Toggle a GPIO pin
 *
 * @param[in] controller Pointer to GPIO controller
 * @param[in] pin        GPIO pin number
 *
 * @return Returns a negative value on an error else returns zero
 */
int hal_gpio_pin_toggle(const char *controller, uint32_t pin);

/**
 * @brief Reset the UWB IC
 *
 * @param[in] none
 *
 * @return none
 */
void hal_gpio_pin_reset_DWIC(void);

#endif    /* _HAL_GPIO_H_ */
