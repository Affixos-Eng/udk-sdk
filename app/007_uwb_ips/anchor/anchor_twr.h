/**
 * @file anchor_twr.h
 * @brief Anchor TWR (Two-Way Ranging Responder) interface
 */

#ifndef ANCHOR_TWR_H
#define ANCHOR_TWR_H

#include "../common/uwb_ips.h"

/**
 * @brief Initialize anchor TWR functionality
 * @return 0 on success, negative error code on failure
 */
int anchor_twr_init(void);

/**
 * @brief Run anchor responder (blocking, handles one exchange)
 * @return 0 on success, negative error code on failure
 */
int anchor_twr_respond(void);

/**
 * @brief Send anchor beacon
 * @return 0 on success
 */
int anchor_send_beacon(void);

/**
 * @brief Handle received position message from tag
 * @param rx_buffer Received message
 * @param length Message length
 */
void anchor_handle_position(const uint8_t *rx_buffer, uint16_t length);

/**
 * @brief Forward data to gateway (via mesh or UART)
 * @param data Data to forward
 * @param length Data length
 * @return 0 on success
 */
int anchor_forward_to_gateway(const uint8_t *data, uint16_t length);

#endif /* ANCHOR_TWR_H */

