/**
 * @file tag_main.c
 * @brief UWB IPS Tag Main Application
 * 
 * Tags perform DS-TWR ranging with anchors (via UWB), calculate their 
 * position using trilateration, and send position updates via BLE mesh.
 * 
 * Architecture:
 * - UWB: Used ONLY for TWR ranging
 * - BLE: Used for receiving anchor beacons and sending position data
 */

#include "../common/uwb_ips.h"
#include "../common/ble_mesh.h"
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

/* Anchor list from BLE beacons */
#define MAX_ANCHORS 8
static anchor_info_t known_anchors[MAX_ANCHORS];
static uint8_t num_known_anchors = 0;
static K_MUTEX_DEFINE(anchor_mutex);

/*============================================================================
 * BLE Beacon Callback
 *============================================================================*/

static void on_anchor_beacon(const anchor_info_t *anchor)
{
    k_mutex_lock(&anchor_mutex, K_FOREVER);
    
    /* Look for existing entry */
    for (uint8_t i = 0; i < num_known_anchors; i++) {
        if (known_anchors[i].id == anchor->id) {
            /* Update existing */
            known_anchors[i] = *anchor;
            k_mutex_unlock(&anchor_mutex);
            return;
        }
    }
    
    /* Add new anchor if room */
    if (num_known_anchors < MAX_ANCHORS) {
        known_anchors[num_known_anchors++] = *anchor;
        LOG_INF("Discovered anchor 0x%04X via BLE at (%d, %d, %d) mm",
                anchor->id,
                anchor->position.x_mm,
                anchor->position.y_mm,
                anchor->position.z_mm);
    }
    
    k_mutex_unlock(&anchor_mutex);
}

static uint8_t get_known_anchors(anchor_info_t *anchors, uint8_t max_count)
{
    uint32_t now = k_uptime_get_32();
    uint8_t count = 0;
    
    k_mutex_lock(&anchor_mutex, K_FOREVER);
    
    for (uint8_t i = 0; i < num_known_anchors && count < max_count; i++) {
        /* Skip stale anchors (not seen in 30 seconds) */
        if ((now - known_anchors[i].last_seen_ms) > 30000) {
            continue;
        }
        anchors[count++] = known_anchors[i];
    }
    
    k_mutex_unlock(&anchor_mutex);
    
    return count;
}

/*============================================================================
 * Tag Main Loop
 *============================================================================*/

static void tag_run_cycle(void)
{
    /* Get list of known anchors from BLE beacons */
    anchor_info_t anchors[CONFIG_UWB_IPS_MAX_ANCHORS];
    uint8_t num_anchors = get_known_anchors(anchors, CONFIG_UWB_IPS_MAX_ANCHORS);
    
    if (num_anchors < MIN_ANCHORS_FOR_POS) {
        LOG_WRN("Not enough anchors (%u/%u), waiting for BLE beacons...", 
                num_anchors, MIN_ANCHORS_FOR_POS);
        return;
    }
    
    LOG_DBG("Starting UWB ranging cycle with %u anchors", num_anchors);
    
    /* Perform UWB TWR ranging with all anchors */
    twr_measurement_t measurements[CONFIG_UWB_IPS_MAX_ANCHORS];
    uint8_t num_valid = 0;
    
    tag_twr_range_multiple(anchors, num_anchors, measurements, &num_valid);
    
    if (num_valid < MIN_ANCHORS_FOR_POS) {
        LOG_WRN("Too few valid UWB measurements (%u/%u)", num_valid, MIN_ANCHORS_FOR_POS);
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
    
    /* Send position via BLE mesh */
    ret = ble_mesh_send_position(&position, &battery);
    if (ret != 0) {
        LOG_WRN("BLE position broadcast failed: %d", ret);
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
    LOG_INF("Mesh: BLE Advertisement-based");
    LOG_INF("==============================================");
    
    int ret;
    
    /* Initialize BLE mesh (for beacons and position broadcast) */
    ret = ble_mesh_init(CONFIG_UWB_IPS_DEVICE_ID, false);
    if (ret != 0) {
        LOG_ERR("BLE mesh init failed: %d", ret);
        return ret;
    }
    
    /* Set callback for anchor beacons */
    ble_mesh_set_beacon_callback(on_anchor_beacon);
    
    /* Start BLE mesh (scanning for beacons) */
    ret = ble_mesh_start();
    if (ret != 0) {
        LOG_ERR("BLE mesh start failed: %d", ret);
        return ret;
    }
    
    /* Initialize UWB transceiver (for TWR ranging only) */
    ret = uwb_init();
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
    
    /* Wait for BLE anchor beacons */
    LOG_INF("Waiting for BLE anchor beacons...");
    k_msleep(5000);
    
    /* Start main tag thread (UWB ranging + BLE position broadcast) */
    k_thread_create(&tag_thread, tag_stack, TAG_STACK_SIZE,
                    tag_thread_entry, NULL, NULL, NULL,
                    TAG_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&tag_thread, "tag_main");
    
    /* Main thread monitors status */
    while (1) {
        k_msleep(10000);
        
        /* Periodic status log */
        anchor_info_t anchors[MAX_ANCHORS];
        uint8_t num = get_known_anchors(anchors, MAX_ANCHORS);
        LOG_INF("Known anchors (via BLE): %u", num);
    }
    
    return 0;
}
