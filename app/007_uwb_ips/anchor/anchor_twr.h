/**
 * @file anchor_twr.h
 * @brief Anchor TWR (Two-Way Ranging Responder) interface
 * 
 * Handles UWB TWR ranging only. Position forwarding is done via BLE mesh.
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
 * 
 * Listens for TWR poll messages and responds. Does not handle position
 * forwarding - that's done via BLE mesh in anchor_main.c.
 * 
 * @return 0 on success, -ETIMEDOUT on no message, negative error otherwise
 */
int anchor_twr_respond(void);

#endif /* ANCHOR_TWR_H */
