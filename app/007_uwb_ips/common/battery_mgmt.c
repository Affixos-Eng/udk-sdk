/**
 * @file battery_mgmt.c
 * @brief LiPo battery monitoring
 */

#include "uwb_ips.h"
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(battery, LOG_LEVEL_INF);

#if CONFIG_UWB_IPS_BATTERY_MONITOR && DT_NODE_EXISTS(DT_PATH(zephyr_user))

/*============================================================================
 * Configuration
 *============================================================================*/

/* LiPo voltage thresholds (mV) */
#define BATTERY_FULL_MV     4200
#define BATTERY_EMPTY_MV    3300
#define BATTERY_RANGE_MV    (BATTERY_FULL_MV - BATTERY_EMPTY_MV)

/* ADC configuration */
#define ADC_RESOLUTION      12
#define ADC_REFERENCE_MV    3300
#define ADC_DIVIDER_RATIO   2  /* If using voltage divider */

static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));
static int16_t adc_buffer;

static struct adc_sequence adc_seq = {
    .buffer = &adc_buffer,
    .buffer_size = sizeof(adc_buffer),
};

static bool initialized = false;

/*============================================================================
 * Public Functions
 *============================================================================*/

int battery_init(void)
{
    if (!adc_is_ready_dt(&adc_channel)) {
        LOG_WRN("ADC device not ready");
        return -ENODEV;
    }
    
    int ret = adc_channel_setup_dt(&adc_channel);
    if (ret < 0) {
        LOG_ERR("ADC channel setup failed: %d", ret);
        return ret;
    }
    
    ret = adc_sequence_init_dt(&adc_channel, &adc_seq);
    if (ret < 0) {
        LOG_ERR("ADC sequence init failed: %d", ret);
        return ret;
    }
    
    initialized = true;
    LOG_INF("Battery monitoring initialized");
    
    return 0;
}

int battery_read(battery_status_t *status)
{
    if (!initialized) {
        status->voltage_mv = 0;
        status->percentage = 0;
        status->is_charging = false;
        return -ENODEV;
    }
    
    int ret = adc_read(adc_channel.dev, &adc_seq);
    if (ret < 0) {
        LOG_WRN("ADC read failed: %d", ret);
        return ret;
    }
    
    /* Convert ADC value to millivolts */
    int32_t val_mv = adc_buffer;
    ret = adc_raw_to_millivolts_dt(&adc_channel, &val_mv);
    if (ret < 0) {
        /* Fallback calculation */
        val_mv = (adc_buffer * ADC_REFERENCE_MV) / (1 << ADC_RESOLUTION);
    }
    
    /* Account for voltage divider if present */
    status->voltage_mv = val_mv * ADC_DIVIDER_RATIO;
    
    /* Calculate percentage */
    if (status->voltage_mv >= BATTERY_FULL_MV) {
        status->percentage = 100;
    } else if (status->voltage_mv <= BATTERY_EMPTY_MV) {
        status->percentage = 0;
    } else {
        status->percentage = (uint8_t)(((status->voltage_mv - BATTERY_EMPTY_MV) * 100) / BATTERY_RANGE_MV);
    }
    
    /* TODO: Detect charging status from GPIO if available */
    status->is_charging = false;
    
    LOG_DBG("Battery: %u mV, %u%%", status->voltage_mv, status->percentage);
    
    return 0;
}

#else /* !CONFIG_UWB_IPS_BATTERY_MONITOR */

int battery_init(void)
{
    LOG_INF("Battery monitoring disabled");
    return 0;
}

int battery_read(battery_status_t *status)
{
    status->voltage_mv = 4200;
    status->percentage = 100;
    status->is_charging = false;
    return 0;
}

#endif /* CONFIG_UWB_IPS_BATTERY_MONITOR */

