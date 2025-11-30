/**
 * @file anchor_main.c
 * @brief UWB IPS Anchor Main Application
 * 
 * Anchors respond to UWB TWR ranging requests from tags and relay
 * position data through BLE mesh to the gateway.
 * 
 * Architecture:
 * - UWB: Used ONLY for TWR ranging (responding to tags)
 * - BLE: Used for beacons and position data forwarding
 */

#include "../common/uwb_ips.h"
#include "../common/ble_mesh.h"
#include "anchor_twr.h"
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <shared_functions.h>
#include <shared_defines.h>
#include <stdio.h>

LOG_MODULE_REGISTER(anchor_main, LOG_LEVEL_INF);

/*============================================================================
 * Configuration
 *============================================================================*/

#define ANCHOR_STACK_SIZE   2048
#define ANCHOR_PRIORITY     5
#define BEACON_INTERVAL_MS  CONFIG_UWB_IPS_BEACON_INTERVAL_MS

/*============================================================================
 * Static Variables
 *============================================================================*/

static K_THREAD_STACK_DEFINE(anchor_stack, ANCHOR_STACK_SIZE);
static struct k_thread anchor_thread;

static K_THREAD_STACK_DEFINE(beacon_stack, 1024);
static struct k_thread beacon_thread_data;

static bool running = false;

/* Anchor's known position (configured or stored) */
static position_3d_t anchor_position = {0};

/* Gateway UART */
#if CONFIG_UWB_IPS_GATEWAY
static const struct device *uart_dev = NULL;
#endif

/*============================================================================
 * Gateway UART Output
 *============================================================================*/

#if CONFIG_UWB_IPS_GATEWAY
static void gateway_output_position(uint16_t tag_id,
                                    const position_3d_t *position,
                                    uint8_t battery_pct)
{
    if (uart_dev == NULL) {
        return;
    }
    
    /* Output JSON format */
    char buf[128];
    int len = snprintf(buf, sizeof(buf),
                       "{\"tag\":%u,\"x\":%d,\"y\":%d,\"z\":%d,\"q\":%u,\"bat\":%u,\"ts\":%u}\n",
                       tag_id,
                       position->x_mm / 10,  /* Convert to cm */
                       position->y_mm / 10,
                       position->z_mm / 10,
                       position->quality / 10,
                       battery_pct,
                       position->timestamp_ms);
    
    for (int i = 0; i < len; i++) {
        uart_poll_out(uart_dev, buf[i]);
    }
    
    LOG_INF("Gateway TX: tag=%u pos=(%d,%d,%d)cm",
            tag_id, position->x_mm / 10, position->y_mm / 10, position->z_mm / 10);
}

static int gateway_uart_init(void)
{
    uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not ready");
        return -ENODEV;
    }
    
    LOG_INF("Gateway UART initialized");
    return 0;
}
#endif

/*============================================================================
 * BLE Position Callback (Gateway Only)
 *============================================================================*/

static void on_position_received(uint16_t tag_id,
                                 const position_3d_t *position,
                                 uint8_t battery_pct)
{
    LOG_INF("Position from tag 0x%04X: (%d, %d, %d) mm",
            tag_id, position->x_mm, position->y_mm, position->z_mm);
    
#if CONFIG_UWB_IPS_GATEWAY
    gateway_output_position(tag_id, position, battery_pct);
#endif
}

/*============================================================================
 * Beacon Thread
 *============================================================================*/

static void beacon_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    
    LOG_INF("BLE beacon thread started");
    
    bool is_gw = IS_ENABLED(CONFIG_UWB_IPS_GATEWAY);
    
    while (running) {
        /* Send BLE beacon */
        ble_mesh_send_beacon(&anchor_position, is_gw);
        
        k_msleep(BEACON_INTERVAL_MS);
    }
    
    LOG_INF("Beacon thread stopped");
}

/*============================================================================
 * UWB TWR Responder Thread
 *============================================================================*/

static void anchor_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    
    LOG_INF("UWB TWR responder thread started");
    
    while (running) {
        /* Respond to incoming TWR poll messages */
        int ret = anchor_twr_respond();
        
        if (ret == -ETIMEDOUT) {
            /* Normal - no TWR request received */
        } else if (ret < 0) {
            LOG_DBG("TWR respond error: %d", ret);
        }
    }
    
    LOG_INF("UWB TWR responder thread stopped");
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
    LOG_INF("Beacon interval: %d ms", BEACON_INTERVAL_MS);
    LOG_INF("Mesh: BLE Advertisement-based");
    LOG_INF("==============================================");
    
    int ret;
    bool is_gateway = IS_ENABLED(CONFIG_UWB_IPS_GATEWAY);
    
    /* TODO: Load anchor position from config/storage */
    /* For now, use device ID to offset position for testing */
    anchor_position.x_mm = (CONFIG_UWB_IPS_DEVICE_ID - 1001) * 3000;  /* 3m spacing */
    anchor_position.y_mm = 0;
    anchor_position.z_mm = 2000;  /* 2m height */
    anchor_position.quality = 1000;
    
    LOG_INF("Anchor position: (%d, %d, %d) mm",
            anchor_position.x_mm, anchor_position.y_mm, anchor_position.z_mm);
    
#if CONFIG_UWB_IPS_GATEWAY
    /* Initialize gateway UART */
    ret = gateway_uart_init();
    if (ret != 0) {
        LOG_ERR("Gateway UART init failed: %d", ret);
        return ret;
    }
#endif
    
    /* Initialize BLE mesh */
    ret = ble_mesh_init(CONFIG_UWB_IPS_DEVICE_ID, is_gateway);
    if (ret != 0) {
        LOG_ERR("BLE mesh init failed: %d", ret);
        return ret;
    }
    
    /* Set position callback (gateway receives and outputs) */
    ble_mesh_set_position_callback(on_position_received);
    
    /* Start BLE mesh (scanning + forwarding) */
    ret = ble_mesh_start();
    if (ret != 0) {
        LOG_ERR("BLE mesh start failed: %d", ret);
        return ret;
    }
    
    /* Initialize UWB transceiver (for TWR responses) */
    ret = uwb_init();
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
    
    running = true;
    
    /* Start BLE beacon thread */
    k_thread_create(&beacon_thread_data, beacon_stack, K_THREAD_STACK_SIZEOF(beacon_stack),
                    beacon_thread_entry, NULL, NULL, NULL,
                    ANCHOR_PRIORITY + 1, 0, K_NO_WAIT);
    k_thread_name_set(&beacon_thread_data, "ble_beacon");
    
    /* Start UWB TWR responder thread */
    k_thread_create(&anchor_thread, anchor_stack, ANCHOR_STACK_SIZE,
                    anchor_thread_entry, NULL, NULL, NULL,
                    ANCHOR_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&anchor_thread, "uwb_twr");
    
    /* Main thread monitors status */
    while (1) {
        k_msleep(30000);
        
        LOG_INF("Anchor running, uptime: %u s", k_uptime_get_32() / 1000);
    }
    
    return 0;
}
