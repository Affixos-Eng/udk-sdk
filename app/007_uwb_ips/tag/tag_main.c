/**
 * @file tag_main.c
 * @brief UWB IPS Tag Main Application
 * 
 * Tags perform DS-TWR ranging with anchors, calculate their position
 * using trilateration, and send position updates to the network.
 */

#include "../common/uwb_ips.h"
#include "tag_twr.h"
#include <zephyr/logging/log.h>
#include <shared_functions.h>
#include <shared_defines.h>

LOG_MODULE_REGISTER(tag_main, LOG_LEVEL_INF);

/*============================================================================
 * Configuration
 *============================================================================*/

#define TAG_STACK_SIZE      2048
#define TAG_PRIORITY        5
#define MIN_ANCHORS_FOR_POS 3

/*============================================================================
 * Static Variables
 *============================================================================*/

static K_THREAD_STACK_DEFINE(tag_stack, TAG_STACK_SIZE);
static struct k_thread tag_thread;

static bool running = false;

/*============================================================================
 * Tag Main Loop
 *============================================================================*/

static void tag_run_cycle(void)
{
    /* Get list of known anchors */
    anchor_info_t anchors[CONFIG_UWB_IPS_MAX_ANCHORS];
    uint8_t num_anchors = tag_get_anchors(anchors, CONFIG_UWB_IPS_MAX_ANCHORS);
    
    if (num_anchors < MIN_ANCHORS_FOR_POS) {
        LOG_WRN("Not enough anchors (%u/%u), waiting...", 
                num_anchors, MIN_ANCHORS_FOR_POS);
        return;
    }
    
    LOG_DBG("Starting ranging cycle with %u anchors", num_anchors);
    
    /* Perform TWR ranging with all anchors */
    twr_measurement_t measurements[CONFIG_UWB_IPS_MAX_ANCHORS];
    uint8_t num_valid = 0;
    
    tag_twr_range_multiple(anchors, num_anchors, measurements, &num_valid);
    
    if (num_valid < MIN_ANCHORS_FOR_POS) {
        LOG_WRN("Too few valid measurements (%u/%u)", num_valid, MIN_ANCHORS_FOR_POS);
        return;
    }
    
    /* Calculate position using trilateration */
    position_3d_t position = {0};
    
    int ret = position_calculate(measurements, num_valid, anchors, &position);
    
    if (ret != 0) {
        LOG_WRN("Position calculation failed: %d", ret);
        return;
    }
    
    /* Read battery status */
    battery_status_t battery = {0};
#if CONFIG_UWB_IPS_BATTERY_MONITOR
    battery_read(&battery);
#else
    battery.percentage = 100;
    battery.voltage_mv = 4200;
    battery.is_charging = false;
#endif
    
    /* Log position */
    LOG_INF("Position: (%.3f, %.3f, %.3f) m, quality: %u",
            position.x_mm / 1000.0f,
            position.y_mm / 1000.0f,
            position.z_mm / 1000.0f,
            position.quality);
    
    /* Find nearest anchor (prefer gateway) and send position */
    uint16_t best_anchor = 0;
    float best_distance = 999999.0f;
    
    for (uint8_t i = 0; i < num_valid; i++) {
        if (measurements[i].valid) {
            /* Prefer gateway anchors */
            float effective_dist = measurements[i].distance_m;
            for (uint8_t j = 0; j < num_anchors; j++) {
                if (anchors[j].id == measurements[i].anchor_id && anchors[j].is_gateway) {
                    effective_dist *= 0.5f;  /* Prioritize gateway */
                    break;
                }
            }
            
            if (effective_dist < best_distance) {
                best_distance = effective_dist;
                best_anchor = measurements[i].anchor_id;
            }
        }
    }
    
    if (best_anchor != 0) {
        tag_send_position(best_anchor, &position, &battery);
    }
}

static void tag_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    
    LOG_INF("Tag thread started");
    
    uint32_t cycle_interval_ms = 1000 / CONFIG_UWB_IPS_UPDATE_RATE_HZ;
    
    while (running) {
        uint32_t start = k_uptime_get_32();
        
        tag_run_cycle();
        
        /* Calculate remaining sleep time */
        uint32_t elapsed = k_uptime_get_32() - start;
        if (elapsed < cycle_interval_ms) {
            k_msleep(cycle_interval_ms - elapsed);
        }
    }
    
    LOG_INF("Tag thread stopped");
}

/*============================================================================
 * Beacon Listener Thread
 *============================================================================*/

#define BEACON_STACK_SIZE   1024
#define BEACON_PRIORITY     6

static K_THREAD_STACK_DEFINE(beacon_stack, BEACON_STACK_SIZE);
static struct k_thread beacon_thread;

static void beacon_listener_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    
    LOG_INF("Beacon listener started");
    
    uint8_t rx_buffer[32];
    
    while (running) {
        /* Enable RX for beacon listening */
        dwt_setpreambledetecttimeout(0);
        dwt_setrxtimeout(5000);  /* 5ms timeout */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        
        uint32_t status;
        waitforsysstatus(&status, NULL, 
                         (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR), 0);
        
        if (status & DWT_INT_RXFCG_BIT_MASK) {
            dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
            
            uint16_t frame_len = dwt_getframelength(0);
            if (frame_len <= sizeof(rx_buffer)) {
                dwt_readrxdata(rx_buffer, frame_len, 0);
                
                /* Check if it's a beacon (func_code at offset 9) */
                if (frame_len >= 10 && rx_buffer[9] == MSG_TYPE_ANCHOR_BEACON) {
                    tag_process_beacon(rx_buffer, frame_len, 0);
                }
            }
        } else {
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }
        
        k_msleep(100);  /* Brief pause between listens */
    }
}

/*============================================================================
 * Main Entry Point (called by SDK's main.c as test_app())
 *============================================================================*/

int test_app(void)
{
    LOG_INF("==============================================");
    LOG_INF("UWB Indoor Positioning System - Tag");
    LOG_INF("Version: %d.%d.%d", 
            UWB_IPS_VERSION_MAJOR, 
            UWB_IPS_VERSION_MINOR,
            UWB_IPS_VERSION_PATCH);
    LOG_INF("Device ID: %d (0x%04X)", 
            CONFIG_UWB_IPS_DEVICE_ID, 
            CONFIG_UWB_IPS_DEVICE_ID);
    LOG_INF("Update rate: %d Hz", CONFIG_UWB_IPS_UPDATE_RATE_HZ);
    LOG_INF("==============================================");
    
    /* Initialize UWB transceiver */
    int ret = uwb_init();
    if (ret != 0) {
        LOG_ERR("UWB init failed: %d", ret);
        return ret;
    }
    
    /* Initialize Tag TWR */
    ret = tag_twr_init();
    if (ret != 0) {
        LOG_ERR("Tag TWR init failed: %d", ret);
        return ret;
    }
    
    /* Initialize battery monitoring */
#if CONFIG_UWB_IPS_BATTERY_MONITOR
    ret = battery_init();
    if (ret != 0) {
        LOG_WRN("Battery init failed: %d (continuing anyway)", ret);
    }
#endif
    
    running = true;
    
    /* Start beacon listener thread */
    k_thread_create(&beacon_thread, beacon_stack, BEACON_STACK_SIZE,
                    beacon_listener_entry, NULL, NULL, NULL,
                    BEACON_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&beacon_thread, "beacon_listener");
    
    /* Wait for some anchors to be discovered */
    LOG_INF("Waiting for anchor beacons...");
    k_msleep(5000);
    
    /* Start main tag thread */
    k_thread_create(&tag_thread, tag_stack, TAG_STACK_SIZE,
                    tag_thread_entry, NULL, NULL, NULL,
                    TAG_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&tag_thread, "tag_main");
    
    /* Main thread can sleep or do housekeeping */
    while (1) {
        k_msleep(10000);
        
        /* Periodic status log */
        anchor_info_t anchors[CONFIG_UWB_IPS_MAX_ANCHORS];
        uint8_t num_anchors = tag_get_anchors(anchors, CONFIG_UWB_IPS_MAX_ANCHORS);
        LOG_INF("Known anchors: %u", num_anchors);
    }
    
    return 0;
}

