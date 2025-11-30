/**
 * @file anchor_main.c
 * @brief UWB IPS Anchor Main Application
 * 
 * Anchors respond to TWR ranging requests from tags and forward
 * position data through the mesh network to the gateway.
 */

#include "../common/uwb_ips.h"
#include "anchor_twr.h"
#include <zephyr/logging/log.h>
#include <shared_functions.h>
#include <shared_defines.h>

LOG_MODULE_REGISTER(anchor_main, LOG_LEVEL_INF);

/*============================================================================
 * Configuration
 *============================================================================*/

#define ANCHOR_STACK_SIZE   2048
#define ANCHOR_PRIORITY     5

/*============================================================================
 * Static Variables
 *============================================================================*/

static K_THREAD_STACK_DEFINE(anchor_stack, ANCHOR_STACK_SIZE);
static struct k_thread anchor_thread;

static bool running = false;
static uint32_t last_beacon_time = 0;

/*============================================================================
 * Anchor Main Loop
 *============================================================================*/

static void anchor_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    
    LOG_INF("Anchor thread started");
    
    while (running) {
        uint32_t now = k_uptime_get_32();
        
        /* Send beacon periodically */
        if ((now - last_beacon_time) >= CONFIG_UWB_IPS_BEACON_INTERVAL_MS) {
            anchor_send_beacon();
            last_beacon_time = now;
        }
        
        /* Process incoming messages (TWR poll, position data, etc.) */
        int ret = anchor_twr_respond();
        
        if (ret == -ETIMEDOUT) {
            /* Normal - no messages received */
        } else if (ret < 0) {
            LOG_DBG("TWR respond error: %d", ret);
        }
    }
    
    LOG_INF("Anchor thread stopped");
}

/*============================================================================
 * Main Entry Point (called by SDK's main.c as test_app())
 *============================================================================*/

int test_app(void)
{
    LOG_INF("==============================================");
    LOG_INF("UWB Indoor Positioning System - Anchor");
    LOG_INF("Version: %d.%d.%d", 
            UWB_IPS_VERSION_MAJOR, 
            UWB_IPS_VERSION_MINOR,
            UWB_IPS_VERSION_PATCH);
    LOG_INF("Device ID: %d (0x%04X)", 
            CONFIG_UWB_IPS_DEVICE_ID, 
            CONFIG_UWB_IPS_DEVICE_ID);
#if CONFIG_UWB_IPS_GATEWAY
    LOG_INF("Mode: GATEWAY");
#else
    LOG_INF("Mode: Anchor (relay)");
#endif
    LOG_INF("Beacon interval: %d ms", CONFIG_UWB_IPS_BEACON_INTERVAL_MS);
    LOG_INF("==============================================");
    
    /* Initialize mesh routing */
    mesh_init();
    
    /* Initialize UWB transceiver */
    int ret = uwb_init();
    if (ret != 0) {
        LOG_ERR("UWB init failed: %d", ret);
        return ret;
    }
    
    /* Initialize Anchor TWR */
    ret = anchor_twr_init();
    if (ret != 0) {
        LOG_ERR("Anchor TWR init failed: %d", ret);
        return ret;
    }
    
#if CONFIG_UWB_IPS_GATEWAY
    /* Initialize gateway UART interface */
    extern int gateway_uart_init(void);
    ret = gateway_uart_init();
    if (ret != 0) {
        LOG_ERR("Gateway UART init failed: %d", ret);
        return ret;
    }
#endif
    
    running = true;
    
    /* Send initial beacon */
    anchor_send_beacon();
    last_beacon_time = k_uptime_get_32();
    
    /* Start anchor thread */
    k_thread_create(&anchor_thread, anchor_stack, ANCHOR_STACK_SIZE,
                    anchor_thread_entry, NULL, NULL, NULL,
                    ANCHOR_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&anchor_thread, "anchor_main");
    
    /* Main thread monitors status */
    while (1) {
        k_msleep(30000);
        
        LOG_INF("Anchor running, uptime: %u s", k_uptime_get_32() / 1000);
    }
    
    return 0;
}

