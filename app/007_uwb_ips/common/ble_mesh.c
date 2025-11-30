/**
 * @file ble_mesh.c
 * @brief BLE Advertisement-based Mesh Implementation
 */

#include "ble_mesh.h"
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(ble_mesh, LOG_LEVEL_INF);

/*============================================================================
 * Configuration
 *============================================================================*/

#define ADV_DATA_MAX_LEN    24
#define MANUF_DATA_OFFSET   2   /* Skip length and type bytes */

/*============================================================================
 * Static Variables
 *============================================================================*/

static uint16_t my_device_id = 0;
static bool is_gateway_node = false;
static bool mesh_running = false;
static uint8_t tx_seq_num = 0;

/* Callbacks */
static ble_mesh_position_cb_t position_callback = NULL;
static ble_mesh_beacon_cb_t beacon_callback = NULL;

/* Duplicate detection cache */
typedef struct {
    uint16_t tag_id;
    uint8_t seq_num;
    uint32_t timestamp;
    bool valid;
} duplicate_entry_t;

static duplicate_entry_t duplicate_cache[BLE_MESH_DUPLICATE_CACHE];
static K_MUTEX_DEFINE(cache_mutex);

/* Advertising data buffer */
static uint8_t adv_data_buf[ADV_DATA_MAX_LEN];

/* Advertising parameters */
static struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(
    BT_LE_ADV_OPT_USE_IDENTITY,
    BT_GAP_ADV_FAST_INT_MIN_2,  /* 100ms */
    BT_GAP_ADV_FAST_INT_MAX_2,  /* 150ms */
    NULL
);

/* Scan parameters */
static struct bt_le_scan_param scan_param = {
    .type = BT_LE_SCAN_TYPE_PASSIVE,
    .options = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
    .interval = BT_GAP_SCAN_FAST_INTERVAL,  /* 100ms */
    .window = BT_GAP_SCAN_FAST_WINDOW,      /* 50ms */
};

/*============================================================================
 * Duplicate Detection
 *============================================================================*/

static bool is_duplicate(uint16_t tag_id, uint8_t seq_num)
{
    uint32_t now = k_uptime_get_32();
    bool found = false;
    int oldest_idx = 0;
    uint32_t oldest_time = UINT32_MAX;
    
    k_mutex_lock(&cache_mutex, K_FOREVER);
    
    for (int i = 0; i < BLE_MESH_DUPLICATE_CACHE; i++) {
        if (duplicate_cache[i].valid) {
            /* Check for match */
            if (duplicate_cache[i].tag_id == tag_id &&
                duplicate_cache[i].seq_num == seq_num) {
                /* Check if still within timeout */
                if ((now - duplicate_cache[i].timestamp) < BLE_MESH_DUPLICATE_TIMEOUT) {
                    found = true;
                    break;
                }
            }
            
            /* Track oldest entry */
            if (duplicate_cache[i].timestamp < oldest_time) {
                oldest_time = duplicate_cache[i].timestamp;
                oldest_idx = i;
            }
        } else {
            oldest_idx = i;
            oldest_time = 0;
        }
    }
    
    /* If not found, add to cache */
    if (!found) {
        duplicate_cache[oldest_idx].tag_id = tag_id;
        duplicate_cache[oldest_idx].seq_num = seq_num;
        duplicate_cache[oldest_idx].timestamp = now;
        duplicate_cache[oldest_idx].valid = true;
    }
    
    k_mutex_unlock(&cache_mutex);
    
    return found;
}

/*============================================================================
 * BLE Scan Callback
 *============================================================================*/

static void scan_recv_cb(const bt_addr_le_t *addr, int8_t rssi,
                         uint8_t adv_type, struct net_buf_simple *buf)
{
    /* Look for manufacturer-specific data */
    while (buf->len > 1) {
        uint8_t len = net_buf_simple_pull_u8(buf);
        if (len == 0 || len > buf->len) {
            break;
        }
        
        uint8_t type = net_buf_simple_pull_u8(buf);
        len--;
        
        if (type == BT_DATA_MANUFACTURER_DATA && len >= 4) {
            /* Check company ID */
            uint16_t company_id = net_buf_simple_pull_le16(buf);
            len -= 2;
            
            if (company_id != BLE_MESH_COMPANY_ID || len < 2) {
                net_buf_simple_pull(buf, len);
                continue;
            }
            
            uint8_t msg_id = net_buf_simple_pull_u8(buf);
            uint8_t ttl = net_buf_simple_pull_u8(buf);
            len -= 2;
            
            if (msg_id == BLE_MSG_TAG_POSITION && len >= sizeof(ble_tag_position_t) - 2) {
                /* Parse position message */
                uint16_t tag_id = net_buf_simple_pull_le16(buf);
                uint8_t seq_num = net_buf_simple_pull_u8(buf);
                int16_t x_cm = net_buf_simple_pull_le16(buf);
                int16_t y_cm = net_buf_simple_pull_le16(buf);
                int16_t z_cm = net_buf_simple_pull_le16(buf);
                uint8_t quality = net_buf_simple_pull_u8(buf);
                uint8_t battery_pct = net_buf_simple_pull_u8(buf);
                
                /* Check for duplicate */
                if (is_duplicate(tag_id, seq_num)) {
                    LOG_DBG("Duplicate position from tag 0x%04X seq %u", tag_id, seq_num);
                    return;
                }
                
                LOG_INF("Position from tag 0x%04X: (%d, %d, %d) cm, TTL=%u",
                        tag_id, x_cm, y_cm, z_cm, ttl);
                
                if (is_gateway_node) {
                    /* Gateway: deliver to callback */
                    if (position_callback) {
                        position_3d_t pos = {
                            .x_mm = x_cm * 10,
                            .y_mm = y_cm * 10,
                            .z_mm = z_cm * 10,
                            .quality = quality * 10,  /* Scale to 0-1000 */
                            .timestamp_ms = k_uptime_get_32()
                        };
                        position_callback(tag_id, &pos, battery_pct);
                    }
                } else if (ttl > 1) {
                    /* Relay: forward with decremented TTL */
                    ble_tag_position_t fwd_msg = {
                        .msg_id = BLE_MSG_TAG_POSITION,
                        .ttl = ttl - 1,
                        .tag_id = tag_id,
                        .seq_num = seq_num,
                        .x_cm = x_cm,
                        .y_cm = y_cm,
                        .z_cm = z_cm,
                        .quality = quality,
                        .battery_pct = battery_pct
                    };
                    
                    /* Small delay to avoid collision */
                    k_msleep(10 + (my_device_id % 20));
                    
                    /* Rebroadcast */
                    struct bt_data ad[] = {
                        BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
                        BT_DATA(BT_DATA_MANUFACTURER_DATA, 
                               (uint8_t[]){
                                   BLE_MESH_COMPANY_ID & 0xFF,
                                   (BLE_MESH_COMPANY_ID >> 8) & 0xFF,
                                   fwd_msg.msg_id, fwd_msg.ttl,
                                   fwd_msg.tag_id & 0xFF, (fwd_msg.tag_id >> 8) & 0xFF,
                                   fwd_msg.seq_num,
                                   fwd_msg.x_cm & 0xFF, (fwd_msg.x_cm >> 8) & 0xFF,
                                   fwd_msg.y_cm & 0xFF, (fwd_msg.y_cm >> 8) & 0xFF,
                                   fwd_msg.z_cm & 0xFF, (fwd_msg.z_cm >> 8) & 0xFF,
                                   fwd_msg.quality, fwd_msg.battery_pct
                               }, 15)
                    };
                    
                    bt_le_adv_stop();
                    bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
                    k_msleep(50);  /* Send a few advertisements */
                    bt_le_adv_stop();
                    
                    LOG_DBG("Forwarded position, TTL now %u", ttl - 1);
                }
                
            } else if (msg_id == BLE_MSG_ANCHOR_BEACON && len >= sizeof(ble_anchor_beacon_t) - 2) {
                /* Parse beacon message */
                uint16_t anchor_id = net_buf_simple_pull_le16(buf);
                int16_t x_cm = net_buf_simple_pull_le16(buf);
                int16_t y_cm = net_buf_simple_pull_le16(buf);
                int16_t z_cm = net_buf_simple_pull_le16(buf);
                uint8_t is_gw = net_buf_simple_pull_u8(buf);
                uint8_t flags = net_buf_simple_pull_u8(buf);
                
                LOG_DBG("Beacon from anchor 0x%04X at (%d, %d, %d) cm",
                        anchor_id, x_cm, y_cm, z_cm);
                
                if (beacon_callback) {
                    anchor_info_t info = {
                        .id = anchor_id,
                        .position = {
                            .x_mm = x_cm * 10,
                            .y_mm = y_cm * 10,
                            .z_mm = z_cm * 10,
                            .quality = 1000,
                            .timestamp_ms = k_uptime_get_32()
                        },
                        .is_gateway = is_gw,
                        .last_rssi = rssi,
                        .last_seen_ms = k_uptime_get_32()
                    };
                    beacon_callback(&info);
                }
            } else {
                net_buf_simple_pull(buf, len);
            }
        } else {
            net_buf_simple_pull(buf, len);
        }
    }
}

/*============================================================================
 * Public Functions
 *============================================================================*/

int ble_mesh_init(uint16_t device_id, bool is_gateway)
{
    int err;
    
    my_device_id = device_id;
    is_gateway_node = is_gateway;
    
    /* Clear duplicate cache */
    memset(duplicate_cache, 0, sizeof(duplicate_cache));
    
    /* Initialize Bluetooth */
    err = bt_enable(NULL);
    if (err && err != -EALREADY) {
        LOG_ERR("Bluetooth init failed: %d", err);
        return err;
    }
    
    LOG_INF("BLE mesh initialized, device_id=0x%04X, gateway=%d",
            device_id, is_gateway);
    
    return 0;
}

int ble_mesh_start(void)
{
    int err;
    
    /* Start scanning */
    err = bt_le_scan_start(&scan_param, scan_recv_cb);
    if (err && err != -EALREADY) {
        LOG_ERR("Scan start failed: %d", err);
        return err;
    }
    
    mesh_running = true;
    LOG_INF("BLE mesh started");
    
    return 0;
}

int ble_mesh_stop(void)
{
    bt_le_scan_stop();
    bt_le_adv_stop();
    mesh_running = false;
    
    LOG_INF("BLE mesh stopped");
    return 0;
}

int ble_mesh_send_position(const position_3d_t *position,
                           const battery_status_t *battery)
{
    ble_tag_position_t msg = {
        .msg_id = BLE_MSG_TAG_POSITION,
        .ttl = BLE_MESH_MAX_TTL,
        .tag_id = my_device_id,
        .seq_num = tx_seq_num++,
        .x_cm = (int16_t)(position->x_mm / 10),
        .y_cm = (int16_t)(position->y_mm / 10),
        .z_cm = (int16_t)(position->z_mm / 10),
        .quality = (uint8_t)(position->quality / 10),
        .battery_pct = battery->percentage
    };
    
    uint8_t manuf_data[] = {
        BLE_MESH_COMPANY_ID & 0xFF,
        (BLE_MESH_COMPANY_ID >> 8) & 0xFF,
        msg.msg_id, msg.ttl,
        msg.tag_id & 0xFF, (msg.tag_id >> 8) & 0xFF,
        msg.seq_num,
        msg.x_cm & 0xFF, (msg.x_cm >> 8) & 0xFF,
        msg.y_cm & 0xFF, (msg.y_cm >> 8) & 0xFF,
        msg.z_cm & 0xFF, (msg.z_cm >> 8) & 0xFF,
        msg.quality, msg.battery_pct
    };
    
    struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
        BT_DATA(BT_DATA_MANUFACTURER_DATA, manuf_data, sizeof(manuf_data))
    };
    
    /* Stop scanning temporarily */
    bt_le_scan_stop();
    
    /* Send multiple advertisements for reliability */
    int err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed: %d", err);
        bt_le_scan_start(&scan_param, scan_recv_cb);
        return err;
    }
    
    /* Keep advertising for ~100ms (a few intervals) */
    k_msleep(100);
    
    bt_le_adv_stop();
    
    /* Resume scanning */
    bt_le_scan_start(&scan_param, scan_recv_cb);
    
    LOG_DBG("Sent position: (%d, %d, %d) cm, seq=%u",
            msg.x_cm, msg.y_cm, msg.z_cm, msg.seq_num);
    
    return 0;
}

int ble_mesh_send_beacon(const position_3d_t *anchor_pos, bool is_gateway)
{
    ble_anchor_beacon_t msg = {
        .msg_id = BLE_MSG_ANCHOR_BEACON,
        .ttl = 1,  /* Beacons are not relayed */
        .anchor_id = my_device_id,
        .x_cm = (int16_t)(anchor_pos->x_mm / 10),
        .y_cm = (int16_t)(anchor_pos->y_mm / 10),
        .z_cm = (int16_t)(anchor_pos->z_mm / 10),
        .is_gateway = is_gateway ? 1 : 0,
        .flags = 0
    };
    
    uint8_t manuf_data[] = {
        BLE_MESH_COMPANY_ID & 0xFF,
        (BLE_MESH_COMPANY_ID >> 8) & 0xFF,
        msg.msg_id, msg.ttl,
        msg.anchor_id & 0xFF, (msg.anchor_id >> 8) & 0xFF,
        msg.x_cm & 0xFF, (msg.x_cm >> 8) & 0xFF,
        msg.y_cm & 0xFF, (msg.y_cm >> 8) & 0xFF,
        msg.z_cm & 0xFF, (msg.z_cm >> 8) & 0xFF,
        msg.is_gateway, msg.flags
    };
    
    struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
        BT_DATA(BT_DATA_MANUFACTURER_DATA, manuf_data, sizeof(manuf_data))
    };
    
    /* Stop scanning temporarily */
    bt_le_scan_stop();
    
    int err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Beacon advertising failed: %d", err);
        bt_le_scan_start(&scan_param, scan_recv_cb);
        return err;
    }
    
    k_msleep(50);
    bt_le_adv_stop();
    
    /* Resume scanning */
    bt_le_scan_start(&scan_param, scan_recv_cb);
    
    LOG_DBG("Sent beacon: anchor 0x%04X at (%d, %d, %d) cm",
            msg.anchor_id, msg.x_cm, msg.y_cm, msg.z_cm);
    
    return 0;
}

void ble_mesh_set_position_callback(ble_mesh_position_cb_t callback)
{
    position_callback = callback;
}

void ble_mesh_set_beacon_callback(ble_mesh_beacon_cb_t callback)
{
    beacon_callback = callback;
}

bool ble_mesh_is_running(void)
{
    return mesh_running;
}

