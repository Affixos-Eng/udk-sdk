/**
 * @file uwb_ips.h
 * @brief UWB Indoor Positioning System - Main Header
 * 
 * This firmware uses the LEAPS UDK-SDK for DW3000 driver support.
 * TWR (Two-Way Ranging) is used for distance measurement.
 */

#ifndef UWB_IPS_H
#define UWB_IPS_H

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

/* Use SDK driver API */
#include <deca_device_api.h>
#include <deca_types.h>

/*============================================================================
 * Configuration Constants
 *============================================================================*/

#define UWB_IPS_VERSION_MAJOR   4
#define UWB_IPS_VERSION_MINOR   0
#define UWB_IPS_VERSION_PATCH   0

/* Device ID ranges */
#define DEVICE_ID_TAG_MIN       1
#define DEVICE_ID_TAG_MAX       1000
#define DEVICE_ID_ANCHOR_MIN    1001
#define DEVICE_ID_ANCHOR_MAX    2000
#define UWB_BROADCAST_ID        0xFFFF

/* TWR timing constants (microseconds) */
#define POLL_TX_TO_RESP_RX_DLY_UUS  300
#define RESP_RX_TO_FINAL_TX_DLY_UUS 300
#define POLL_RX_TO_RESP_TX_DLY_UUS  900
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
#define RESP_RX_TIMEOUT_UUS         300
#define FINAL_RX_TIMEOUT_UUS        220
#define PREAMBLE_TIMEOUT_PAC        5

/* Antenna delays (calibration values) */
#define TX_ANT_DLY  16385
#define RX_ANT_DLY  16385

/* Message types */
#define MSG_TYPE_TWR_POLL           0x21
#define MSG_TYPE_TWR_RESPONSE       0x10
#define MSG_TYPE_TWR_FINAL          0x23
#define MSG_TYPE_TAG_POSITION       0x30
#define MSG_TYPE_ANCHOR_BEACON      0x31
#define MSG_TYPE_GATEWAY_DATA       0x40

/* Use SDK's SPEED_OF_LIGHT from shared_defines.h */
#include <shared_defines.h>

/*============================================================================
 * Data Types
 *============================================================================*/

/**
 * @brief 3D position with quality metric
 */
typedef struct {
    int32_t x_mm;           /**< X position in millimeters */
    int32_t y_mm;           /**< Y position in millimeters */
    int32_t z_mm;           /**< Z position in millimeters */
    uint16_t quality;       /**< Quality indicator (0-1000) */
    uint32_t timestamp_ms;  /**< Timestamp when calculated */
} position_3d_t;

/**
 * @brief Battery status
 */
typedef struct {
    uint16_t voltage_mv;    /**< Voltage in millivolts */
    uint8_t percentage;     /**< Battery percentage (0-100) */
    bool is_charging;       /**< Charging status */
} battery_status_t;

/**
 * @brief TWR measurement result
 */
typedef struct {
    uint16_t anchor_id;     /**< Anchor device ID */
    float distance_m;       /**< Measured distance in meters */
    int8_t rssi;            /**< Received signal strength */
    uint8_t quality;        /**< Measurement quality (0-100) */
    bool valid;             /**< Measurement validity flag */
} twr_measurement_t;

/**
 * @brief Anchor information (for tags)
 */
typedef struct {
    uint16_t id;            /**< Anchor device ID */
    position_3d_t position; /**< Known anchor position */
    int8_t last_rssi;       /**< Last RSSI from beacon */
    uint32_t last_seen_ms;  /**< When last beacon received */
    uint8_t hop_to_gateway; /**< Hops to reach gateway */
    bool is_gateway;        /**< Is this the gateway anchor */
} anchor_info_t;

/**
 * @brief Routing table entry
 */
typedef struct {
    uint16_t dest_id;       /**< Destination (gateway) */
    uint16_t next_hop_id;   /**< Next hop anchor */
    uint8_t hop_count;      /**< Total hops to gateway */
    int8_t rssi;            /**< Link quality */
    uint32_t last_update_ms;/**< Route age */
} routing_entry_t;

/*============================================================================
 * TWR Message Structures (IEEE 802.15.4 compliant)
 *============================================================================*/

/* Frame header common to all TWR messages */
#define TWR_FRAME_CTRL      0x8841  /* Data frame, 16-bit addressing */
#define TWR_PAN_ID          0xDECA  /* Qorvo PAN ID */

/**
 * @brief TWR Poll message (10 bytes + FCS)
 */
typedef struct __attribute__((packed)) {
    uint8_t frame_ctrl[2];  /* 0x41, 0x88 */
    uint8_t seq_num;
    uint8_t pan_id[2];      /* 0xCA, 0xDE */
    uint8_t dest_addr[2];
    uint8_t src_addr[2];
    uint8_t func_code;      /* MSG_TYPE_TWR_POLL */
} twr_poll_msg_t;

/**
 * @brief TWR Response message (13 bytes + FCS)
 */
typedef struct __attribute__((packed)) {
    uint8_t frame_ctrl[2];
    uint8_t seq_num;
    uint8_t pan_id[2];
    uint8_t dest_addr[2];
    uint8_t src_addr[2];
    uint8_t func_code;      /* MSG_TYPE_TWR_RESPONSE */
    uint8_t activity_code;  /* 0x02 = continue ranging */
    uint8_t activity_param[2];
} twr_response_msg_t;

/**
 * @brief TWR Final message (22 bytes + FCS)
 */
typedef struct __attribute__((packed)) {
    uint8_t frame_ctrl[2];
    uint8_t seq_num;
    uint8_t pan_id[2];
    uint8_t dest_addr[2];
    uint8_t src_addr[2];
    uint8_t func_code;      /* MSG_TYPE_TWR_FINAL */
    uint8_t poll_tx_ts[4];  /* Poll TX timestamp (32-bit truncated) */
    uint8_t resp_rx_ts[4];  /* Response RX timestamp */
    uint8_t final_tx_ts[4]; /* Final TX timestamp */
} twr_final_msg_t;

/**
 * @brief Tag position broadcast message
 */
typedef struct __attribute__((packed)) {
    uint8_t frame_ctrl[2];
    uint8_t seq_num;
    uint8_t pan_id[2];
    uint8_t dest_addr[2];
    uint8_t src_addr[2];
    uint8_t func_code;      /* MSG_TYPE_TAG_POSITION */
    int32_t x_mm;
    int32_t y_mm;
    int32_t z_mm;
    uint16_t quality;
    uint8_t battery_pct;
    uint8_t is_charging;
} tag_position_msg_t;

/**
 * @brief Anchor beacon message
 */
typedef struct __attribute__((packed)) {
    uint8_t frame_ctrl[2];
    uint8_t seq_num;
    uint8_t pan_id[2];
    uint8_t dest_addr[2];   /* Broadcast: 0xFF, 0xFF */
    uint8_t src_addr[2];
    uint8_t func_code;      /* MSG_TYPE_ANCHOR_BEACON */
    int32_t x_mm;
    int32_t y_mm;
    int32_t z_mm;
    uint8_t is_gateway;
    uint8_t hop_to_gateway;
} anchor_beacon_msg_t;

/*============================================================================
 * Function Prototypes - UWB Configuration
 *============================================================================*/

/**
 * @brief Initialize UWB transceiver using SDK driver
 * @return 0 on success, negative error code on failure
 */
int uwb_init(void);

/**
 * @brief Get default UWB configuration
 * @return Pointer to dwt_config_t structure
 */
dwt_config_t* uwb_get_config(void);

/**
 * @brief Set device address
 * @param addr 16-bit device address
 */
void uwb_set_address(uint16_t addr);

/*============================================================================
 * Function Prototypes - Position Calculation
 *============================================================================*/

/**
 * @brief Calculate position using trilateration
 * @param measurements Array of TWR measurements
 * @param count Number of measurements
 * @param anchors Array of anchor positions
 * @param position Output position
 * @return 0 on success, negative error code on failure
 */
int position_calculate(const twr_measurement_t *measurements,
                       uint8_t count,
                       const anchor_info_t *anchors,
                       position_3d_t *position);

/*============================================================================
 * Function Prototypes - Battery Management
 *============================================================================*/

/**
 * @brief Initialize battery monitoring
 * @return 0 on success, negative error code on failure
 */
int battery_init(void);

/**
 * @brief Read battery status
 * @param status Output battery status
 * @return 0 on success, negative error code on failure
 */
int battery_read(battery_status_t *status);

/*============================================================================
 * Function Prototypes - Mesh Routing
 *============================================================================*/

/**
 * @brief Initialize mesh routing table
 */
void mesh_init(void);

/**
 * @brief Update routing table from beacon
 * @param source_id Source anchor ID
 * @param gateway_id Gateway ID
 * @param hop_count Hop count from source to gateway
 * @param rssi Signal strength
 */
void mesh_update_route(uint16_t source_id, uint16_t gateway_id,
                       uint8_t hop_count, int8_t rssi);

/**
 * @brief Get next hop toward gateway
 * @param next_hop Output next hop anchor ID
 * @return 0 on success, negative error code if no route
 */
int mesh_get_next_hop(uint16_t *next_hop);

/*============================================================================
 * Function Prototypes - Utilities
 *============================================================================*/

/**
 * @brief Calculate CRC16
 * @param data Input data
 * @param length Data length
 * @return CRC16 value
 */
uint16_t uwb_crc16(const uint8_t *data, uint16_t length);

/**
 * @brief Pack 16-bit address into byte array
 */
static inline void pack_addr(uint8_t *dest, uint16_t addr) {
    dest[0] = addr & 0xFF;
    dest[1] = (addr >> 8) & 0xFF;
}

/**
 * @brief Unpack 16-bit address from byte array
 */
static inline uint16_t unpack_addr(const uint8_t *src) {
    return src[0] | (src[1] << 8);
}

#endif /* UWB_IPS_H */

