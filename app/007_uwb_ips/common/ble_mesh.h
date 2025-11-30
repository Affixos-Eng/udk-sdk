/**
 * @file ble_mesh.h
 * @brief BLE Advertisement-based Mesh for Position Data
 * 
 * Uses BLE non-connectable advertisements to flood position data
 * from tags through anchors to the gateway.
 */

#ifndef BLE_MESH_H
#define BLE_MESH_H

#include "uwb_ips.h"
#include <stdbool.h>
#include <stdint.h>

/*============================================================================
 * Configuration
 *============================================================================*/

#define BLE_MESH_COMPANY_ID         0xFFFF  /* Development ID (change for production) */
#define BLE_MESH_MAX_TTL            3       /* Maximum hops */
#define BLE_MESH_DUPLICATE_CACHE    32      /* Duplicate detection cache size */
#define BLE_MESH_DUPLICATE_TIMEOUT  5000    /* Duplicate timeout (ms) */

/* Advertisement intervals */
#define BLE_MESH_ADV_INTERVAL_TAG   100     /* Tag position broadcast (ms) */
#define BLE_MESH_ADV_INTERVAL_ANCHOR 1000   /* Anchor beacon (ms) */
#define BLE_MESH_SCAN_INTERVAL      100     /* Scan interval (ms) */
#define BLE_MESH_SCAN_WINDOW        50      /* Scan window (ms) */

/* Network ID - allows multiple networks to coexist without mixing traffic */
#ifndef CONFIG_UWB_IPS_NETWORK_ID
#define CONFIG_UWB_IPS_NETWORK_ID   0x01    /* Default network ID */
#endif

/*============================================================================
 * Message Types
 *============================================================================*/

#define BLE_MSG_TAG_POSITION        0x01
#define BLE_MSG_ANCHOR_BEACON       0x02
#define BLE_MSG_GATEWAY_ACK         0x03

/*============================================================================
 * Message Structures
 *============================================================================*/

/**
 * @brief Tag position message (14 bytes)
 * 
 * Network ID is included to allow multiple networks to coexist.
 */
typedef struct __attribute__((packed)) {
    uint8_t msg_id;         /**< BLE_MSG_TAG_POSITION (0x01) */
    uint8_t network_id;     /**< Network ID for filtering */
    uint8_t ttl;            /**< Time-to-live (hops remaining) */
    uint16_t tag_id;        /**< Source tag ID */
    uint8_t seq_num;        /**< Sequence number (0-255) */
    int16_t x_cm;           /**< X position in centimeters */
    int16_t y_cm;           /**< Y position in centimeters */
    int16_t z_cm;           /**< Z position in centimeters */
    uint8_t quality;        /**< Position quality (0-100) */
    uint8_t battery_pct;    /**< Battery percentage */
} ble_tag_position_t;

/**
 * @brief Anchor beacon message (13 bytes)
 * 
 * Network ID is included to allow multiple networks to coexist.
 */
typedef struct __attribute__((packed)) {
    uint8_t msg_id;         /**< BLE_MSG_ANCHOR_BEACON (0x02) */
    uint8_t network_id;     /**< Network ID for filtering */
    uint8_t ttl;            /**< Always 1 (no rebroadcast) */
    uint16_t anchor_id;     /**< Anchor device ID */
    int16_t x_cm;           /**< Known X position */
    int16_t y_cm;           /**< Known Y position */
    int16_t z_cm;           /**< Known Z position */
    uint8_t is_gateway;     /**< 1 if gateway anchor */
    uint8_t flags;          /**< Status flags */
} ble_anchor_beacon_t;

/*============================================================================
 * Callback Types
 *============================================================================*/

/**
 * @brief Callback for gateway to receive position data
 * @param tag_id Source tag ID
 * @param position Received position
 * @param battery_pct Battery percentage
 */
typedef void (*ble_mesh_position_cb_t)(uint16_t tag_id,
                                        const position_3d_t *position,
                                        uint8_t battery_pct);

/**
 * @brief Callback for tags to receive anchor beacons
 * @param anchor Anchor information from beacon
 */
typedef void (*ble_mesh_beacon_cb_t)(const anchor_info_t *anchor);

/*============================================================================
 * Public Functions
 *============================================================================*/

/**
 * @brief Initialize BLE mesh
 * @param device_id This device's ID
 * @param network_id Network ID for multi-network filtering (1-255)
 * @param is_gateway True if this is the gateway anchor
 * @return 0 on success, negative error code on failure
 */
int ble_mesh_init(uint16_t device_id, uint8_t network_id, bool is_gateway);

/**
 * @brief Start BLE mesh operation
 * @return 0 on success
 */
int ble_mesh_start(void);

/**
 * @brief Stop BLE mesh operation
 * @return 0 on success
 */
int ble_mesh_stop(void);

/**
 * @brief Send tag position via BLE advertisement
 * @param position Calculated position
 * @param battery Battery status
 * @return 0 on success
 */
int ble_mesh_send_position(const position_3d_t *position,
                           const battery_status_t *battery);

/**
 * @brief Send anchor beacon via BLE advertisement
 * @param anchor_pos Anchor's known position
 * @param is_gateway True if this anchor is the gateway
 * @return 0 on success
 */
int ble_mesh_send_beacon(const position_3d_t *anchor_pos, bool is_gateway);

/**
 * @brief Set callback for received position data (gateway only)
 * @param callback Function to call when position received
 */
void ble_mesh_set_position_callback(ble_mesh_position_cb_t callback);

/**
 * @brief Set callback for received anchor beacons (tags only)
 * @param callback Function to call when beacon received
 */
void ble_mesh_set_beacon_callback(ble_mesh_beacon_cb_t callback);

/**
 * @brief Check if BLE mesh is running
 * @return true if running
 */
bool ble_mesh_is_running(void);

/**
 * @brief Get current network ID
 * @return Network ID (1-255)
 */
uint8_t ble_mesh_get_network_id(void);

#endif /* BLE_MESH_H */

