/**
 * @file uwb_config.c
 * @brief UWB transceiver initialization using LEAPS UDK-SDK
 */

#include "uwb_ips.h"
#include <zephyr/logging/log.h>

#include <deca_device_api.h>
#include <deca_probe_interface.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <hal-spi.h>
#include <hal-gpio.h>

LOG_MODULE_REGISTER(uwb_config, LOG_LEVEL_INF);

/* External declarations from SDK */
extern void test_run_info(unsigned char *data);
extern dwt_txconfig_t txconfig_options;

/* Default UWB configuration - Channel 5, 6.8 Mbps, 128 preamble */
static dwt_config_t uwb_config = {
    .chan           = 5,                /* Channel 5 (6.5 GHz) */
    .txPreambLength = DWT_PLEN_128,     /* 128 symbol preamble */
    .rxPAC          = DWT_PAC8,         /* PAC 8 for short preamble */
    .txCode         = 9,                /* TX preamble code */
    .rxCode         = 9,                /* RX preamble code */
    .sfdType        = DWT_SFD_DW_8,     /* DW 8-bit SFD */
    .dataRate       = DWT_BR_6M8,       /* 6.8 Mbps data rate */
    .phrMode        = DWT_PHRMODE_STD,  /* Standard PHR mode */
    .phrRate        = DWT_PHRRATE_STD,  /* Standard PHR rate */
    .sfdTO          = (129 + 8 - 8),    /* SFD timeout */
    .stsMode        = DWT_STS_MODE_OFF, /* STS disabled */
    .stsLength      = DWT_STS_LEN_64,   /* STS length (unused) */
    .pdoaMode       = DWT_PDOA_M0       /* PDOA off */
};

static uint16_t device_address = 0;

dwt_config_t* uwb_get_config(void)
{
    return &uwb_config;
}

void uwb_set_address(uint16_t addr)
{
    device_address = addr;
}

int uwb_init(void)
{
    LOG_INF("Initializing UWB transceiver...");
    
    /* Configure SPI for high-speed operation */
    port_set_dw_ic_spi_fastrate();
    
    /* Reset DW3000 IC */
    reset_DWIC();
    
    /* Wait for DW3000 to wake up */
    k_msleep(2);
    
    /* Probe and identify the DW3000 device */
    if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) != DWT_SUCCESS) {
        LOG_ERR("Failed to probe DW3000 device");
        return -ENODEV;
    }
    
    /* Wait for IDLE_RC state */
    int timeout = 100;
    while (!dwt_checkidlerc() && timeout--) {
        k_msleep(1);
    }
    
    if (timeout <= 0) {
        LOG_ERR("DW3000 not in IDLE_RC state");
        return -ETIMEDOUT;
    }
    
    /* Initialize the device */
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        LOG_ERR("DW3000 initialization failed");
        return -EIO;
    }
    
    LOG_INF("DW3000 initialized, device ID: 0x%08x", dwt_readdevid());
    
    /* Configure UWB parameters */
    if (dwt_configure(&uwb_config) != DWT_SUCCESS) {
        LOG_ERR("DW3000 configuration failed");
        return -EIO;
    }
    
    /* Configure TX power */
    dwt_configuretxrf(&txconfig_options);
    
    /* Set antenna delays for calibration */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    
    /* Enable LEDs for debugging (disable in production for power savings) */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    
    /* Configure RF port */
    dwt_configure_rf_port(DWT_RF_PORT_AUTO_1_2);
    
    LOG_INF("UWB transceiver configured successfully");
    LOG_INF("  Channel: %d", uwb_config.chan);
    LOG_INF("  Data rate: 6.8 Mbps");
    LOG_INF("  Preamble: 128 symbols");
    
    return 0;
}

