/**
 * LEAPS - Low Energy Accurate Positioning System.
 *
 * Re-define the probe interfaces base on Zephyr environment
 *
 * Copyright (c) 2025, LEAPS. All rights reserved.
 *
 */

#include "hal-timer.h"
#include "hal-spi.h"
#include "hal-gpio.h"
#include "deca_interface.h"
#include "deca_device_api.h"
#include <nrfx_gpiote.h>

/* Init k_work for IRQ handler. */
static struct k_work irq_work;

#define IRQ_PIN DT_GPIO_PIN(DT_ALIAS(uwb0_irq), gpios)

/* Local file - gpio callback  */
static hal_gpio_callback_t gpio_cb;

/* Extern uwb driver */
extern const struct dwt_driver_s dw3000_driver;
extern const struct dwt_driver_s dw3720_driver;
const struct dwt_driver_s* tmp_ptr[] = { &dw3000_driver, &dw3720_driver};

/* Extern the interface */
static void board_uwb_spi_wakeup(void);

/* Underlying spi interfaces */
static const struct dwt_spi_s dw3000_spi_fct = {
    .readfromspi = hal_spi_read,
    .writetospi = hal_spi_write,
    .writetospiwithcrc = hal_spi_writewithcrc,
    .setslowrate = hal_spi_setslowrate,
    .setfastrate = hal_spi_setfastrate
};

/* Initialize the interfaces */
const struct dwt_probe_s dw3000_probe_interf =
{
    .dw = NULL,
    .spi = (void*)&dw3000_spi_fct,
    .wakeup_device_with_io = board_uwb_spi_wakeup,
    .driver_list = (struct dwt_driver_s **)tmp_ptr,
    .dw_driver_num = (sizeof(tmp_ptr) / sizeof(tmp_ptr[0])),
};

/* Wake up UW IC interface */
static void board_uwb_spi_wakeup(void)
{
    uint32_t val;

    const struct device *dev_gpio_cs = device_get_binding(
        DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(uwb0_cs), gpios)));
    const struct device *dev_gpio_rst = device_get_binding(
        DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(uwb0_reset), gpios)));

    gpio_pin_set(dev_gpio_cs, DT_GPIO_PIN(DT_ALIAS(uwb0_cs), gpios), 0);
    hal_delay_us(200);
    gpio_pin_set(dev_gpio_cs, DT_GPIO_PIN(DT_ALIAS(uwb0_cs), gpios), 1);
    gpio_pin_configure(dev_gpio_rst,
                  DT_GPIO_PIN(DT_ALIAS(uwb0_reset), gpios), GPIO_INPUT);

    do {
        val  = gpio_pin_get(dev_gpio_rst, DT_GPIO_PIN(DT_ALIAS(uwb0_reset),
               gpios));

        gpio_pin_set(dev_gpio_cs, DT_GPIO_PIN(DT_ALIAS(uwb0_cs), gpios), 0);
        hal_delay_us(200);
        gpio_pin_set(dev_gpio_cs, DT_GPIO_PIN(DT_ALIAS(uwb0_cs), gpios), 1);
    } while (val == 0);
}

/* Wrapper function to be used by decadriver. Declared in deca_device_api.h */
void deca_sleep(unsigned int time_ms)
{
    k_msleep(time_ms);
}

/* Wrapper function to be used by decadriver. Declared in deca_device_api.h */
void deca_usleep(unsigned long time_us)
{
    k_usleep(time_us);
}


/* wrapper to compatible */
void dwt_isr_wrapper(void * data)
{
    (void) data;
    k_work_submit(&irq_work);
}

/* DW IC IRQ handler definition. */
static port_dwic_isr_t port_dwic_isr = NULL;



/* @fn      port_CheckEXT_IRQ
 * @brief   wrapper to read DW_IRQ input pin state
 * */
__INLINE uint32_t port_CheckEXT_IRQ(void)
{
    return nrf_gpio_pin_read(IRQ_PIN);
}

/* @fn      port_GetEXT_IRQStatus
 * @brief   wrapper to read a DW_IRQ pin IRQ status
 * */
__INLINE uint32_t port_GetEXT_IRQStatus(void)
{
    bool status = nrfx_gpiote_in_is_set(IRQ_PIN);

    if (status == true)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
/* @fn      port_DisableEXT_IRQ
 * @brief   wrapper to disable DW_IRQ pin IRQ
 * */
__INLINE void port_DisableEXT_IRQ(void)
{
    nrfx_gpiote_in_event_disable(IRQ_PIN);

}

/* @fn      port_EnableEXT_IRQ
 * @brief   wrapper to enable DW_IRQ pin IRQ
 * */
__INLINE void port_EnableEXT_IRQ(void)
{
    nrfx_gpiote_in_event_enable(IRQ_PIN, true);
}

static void process_deca_irq(struct k_work *work)
{
    while (port_CheckEXT_IRQ() != 0)
    {
        if (port_dwic_isr)
        {
            port_dwic_isr();
        }
        k_msleep(1);
    } /* while DW3000 IRQ line active */
    k_msleep(1);
}

/* Described in header file */
void port_set_dwic_isr(port_dwic_isr_t cb)
{
  /* Check DW IC IRQ activation status. */
    uint8_t en = port_GetEXT_IRQStatus();

    /* If needed, deactivate DW IC IRQ during the installation of the new handler. */
    port_DisableEXT_IRQ();

    k_work_init(&irq_work, process_deca_irq);

    hal_gpio_callback_init(dwt_isr_wrapper, NULL, &gpio_cb);
    hal_gpio_callback_register(DT_LABEL(DT_GPIO_CTLR(DT_ALIAS(uwb0_irq), gpios)),
            DT_GPIO_PIN(DT_ALIAS(uwb0_irq), gpios), GPIO_PULLDOWN,
            IRQ_TYPE_EDGE_RISING, &gpio_cb);

    port_dwic_isr = cb;

    if (!en)
    {
        port_EnableEXT_IRQ();
    }
}

typedef int32_t decaIrqStatus_t; /* Type for remembering IRQ status */

decaIrqStatus_t decamutexon(void)
{
    decaIrqStatus_t s = port_GetEXT_IRQStatus();

    if (s)
    {
        port_DisableEXT_IRQ(); /* Disable the external interrupt line */ 
    }
    return s; /* Return state before disable, value is used to re-enable in decamutexoff call */
}

void decamutexoff(decaIrqStatus_t s) /* Put a function here that re-enables the interrupt at the end of the critical section */
{
    if (s)
    { /* Need to check the port state as we can't use level sensitive interrupt on the STM ARM */
        port_EnableEXT_IRQ();
    }
}