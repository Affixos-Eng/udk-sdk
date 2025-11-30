/**
 * @file tag_twr.h
 * @brief Tag TWR (Two-Way Ranging) interface using LEAPS SDK
 */

#ifndef TAG_TWR_H
#define TAG_TWR_H

#include "../common/uwb_ips.h"

/**
 * @brief Initialize tag TWR functionality
 * @return 0 on success, negative error code on failure
 */
int tag_twr_init(void);

/**
 * @brief Perform DS-TWR ranging with a specific anchor
 * @param anchor_id Target anchor ID
 * @param distance_m Output distance in meters
 * @param rssi Output RSSI value
 * @return 0 on success, negative error code on failure
 */
int tag_twr_range(uint16_t anchor_id, float *distance_m, int8_t *rssi);

/**
 * @brief Range with multiple anchors
 * @param anchors Array of anchor info
 * @param num_anchors Number of anchors
 * @param measurements Output measurement results
 * @param num_valid Output number of valid measurements
 * @return 0 on success
 */
int tag_twr_range_multiple(const anchor_info_t *anchors, 
                           uint8_t num_anchors,
                           twr_measurement_t *measurements,
                           uint8_t *num_valid);

/**
 * @brief Send position to anchor for gateway forwarding
 * @param anchor_id Destination anchor
 * @param position Calculated position
 * @param battery Battery status
 * @return 0 on success
 */
int tag_send_position(uint16_t anchor_id, 
                      const position_3d_t *position,
                      const battery_status_t *battery);

/**
 * @brief Process received anchor beacon
 * @param rx_buffer Received message buffer
 * @param length Message length
 * @param rssi Received signal strength
 */
void tag_process_beacon(const uint8_t *rx_buffer, uint16_t length, int8_t rssi);

/**
 * @brief Get list of known anchors
 * @param anchors Output anchor list
 * @param max_count Maximum anchors to return
 * @return Number of known anchors
 */
uint8_t tag_get_anchors(anchor_info_t *anchors, uint8_t max_count);

#endif /* TAG_TWR_H */

