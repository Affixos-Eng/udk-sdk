/**
 * @file anchor_twr.c
 * @brief Anchor TWR (DS-TWR Responder) implementation using LEAPS UDK-SDK
 * 
 * Based on ex_05b_ds_twr_resp from LEAPS SDK examples.
 */

#include "anchor_twr.h"
#include <zephyr/logging/log.h>
#include <string.h>

#include <deca_device_api.h>
#include <shared_defines.h>
#include <shared_functions.h>

LOG_MODULE_REGISTER(anchor_twr, LOG_LEVEL_INF);

/*============================================================================
 * Constants and Configuration
 *============================================================================*/

/* Message frame indices */
#define ALL_MSG_SN_IDX              2
#define ALL_MSG_COMMON_LEN          10
#define FINAL_MSG_POLL_TX_TS_IDX    10
#define FINAL_MSG_RESP_RX_TS_IDX    14
#define FINAL_MSG_FINAL_TX_TS_IDX   18

/*============================================================================
 * Static Variables
 *============================================================================*/

/* TWR message templates */
static uint8_t rx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, MSG_TYPE_TWR_POLL, 0, 0 };
static uint8_t tx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, MSG_TYPE_TWR_RESPONSE, 0x02, 0, 0, 0, 0 };
static uint8_t rx_final_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, MSG_TYPE_TWR_FINAL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

/* Beacon message */
static uint8_t tx_beacon_msg[sizeof(anchor_beacon_msg_t)];

/* RX buffer */
#define RX_BUF_LEN 32
static uint8_t rx_buffer[RX_BUF_LEN];

/* Frame sequence number */
static uint8_t frame_seq_nb = 0;

/* Device configuration */
static uint16_t my_address = 0;
static position_3d_t my_position = {0};
static bool is_gateway = false;
static uint8_t hop_to_gateway = 0;

/*============================================================================
 * Helper Functions - use SDK's functions
 *============================================================================*/

/* SDK provides: get_tx_timestamp_u64(), get_rx_timestamp_u64(), final_msg_get_ts() */

/*============================================================================
 * Public Functions
 *============================================================================*/

int anchor_twr_init(void)
{
    LOG_INF("Initializing Anchor TWR...");
    
    /* Get device configuration */
    my_address = CONFIG_UWB_IPS_DEVICE_ID;
    is_gateway = IS_ENABLED(CONFIG_UWB_IPS_GATEWAY);
    
    if (is_gateway) {
        hop_to_gateway = 0;
        LOG_INF("This anchor is the GATEWAY");
    } else {
        hop_to_gateway = 255;  /* Unknown until we hear from gateway */
    }
    
    /* Set address in message templates */
    pack_addr(&tx_resp_msg[7], my_address);
    
    /* TODO: Load position from config or storage */
    my_position.x_mm = 0;
    my_position.y_mm = 0;
    my_position.z_mm = 0;
    
    LOG_INF("Anchor TWR initialized, address: 0x%04X", my_address);
    
    return 0;
}

int anchor_twr_respond(void)
{
    uint32_t status_reg;
    uint64_t poll_rx_ts, resp_tx_ts, final_rx_ts;
    
    /* Enable RX with no timeout (wait indefinitely for poll) */
    dwt_setpreambledetecttimeout(0);
    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    
    /* Wait for poll message or other frame */
    waitforsysstatus(&status_reg, NULL, 
                     (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR), 0);
    
    if (!(status_reg & DWT_INT_RXFCG_BIT_MASK)) {
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        return -ETIMEDOUT;
    }
    
    /* Clear good RX event */
    dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
    
    /* Read received frame */
    uint16_t frame_len = dwt_getframelength(0);
    if (frame_len > RX_BUF_LEN) {
        return -EMSGSIZE;
    }
    
    dwt_readrxdata(rx_buffer, frame_len, 0);
    
    /* Check message type at offset 9 */
    if (frame_len < 10) {
        return -EINVAL;
    }
    
    uint8_t msg_type = rx_buffer[9];
    
    /* Handle position message from tag */
    if (msg_type == MSG_TYPE_TAG_POSITION) {
        anchor_handle_position(rx_buffer, frame_len);
        return 0;
    }
    
    /* Handle routing update from other anchor */
    if (msg_type == MSG_TYPE_ANCHOR_BEACON) {
        /* Process beacon for routing table */
        const anchor_beacon_msg_t *beacon = (const anchor_beacon_msg_t *)rx_buffer;
        if (beacon->is_gateway && !is_gateway) {
            uint16_t gateway_id = unpack_addr(beacon->src_addr);
            mesh_update_route(gateway_id, gateway_id, 0, 0);
            hop_to_gateway = 1;
        }
        return 0;
    }
    
    /* Check if it's a TWR poll for us */
    if (msg_type != MSG_TYPE_TWR_POLL) {
        return 0;  /* Not a poll, ignore */
    }
    
    /* Verify destination is us */
    uint16_t dest = unpack_addr(&rx_buffer[5]);
    if (dest != my_address && dest != UWB_BROADCAST_ID) {
        return 0;  /* Not for us */
    }
    
    uint16_t source = unpack_addr(&rx_buffer[7]);
    
    LOG_DBG("Poll received from 0x%04X", source);
    
    /* Get poll RX timestamp */
    poll_rx_ts = get_rx_timestamp_u64();
    
    /* Calculate response TX time */
    uint32_t resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(resp_tx_time);
    
    /* Set timing for final RX */
    dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
    dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
    dwt_setpreambledetecttimeout(PREAMBLE_TIMEOUT_PAC);
    
    /* Build response message */
    pack_addr(&tx_resp_msg[5], source);  /* Dest = poll source */
    tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    
    dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
    dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);
    
    /* Send response with RX enabled after */
    int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
    
    if (ret != DWT_SUCCESS) {
        LOG_WRN("Delayed TX failed for response");
        return -EIO;
    }
    
    /* Wait for final message */
    waitforsysstatus(&status_reg, NULL, 
                     (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR), 0);
    
    frame_seq_nb++;
    
    if (!(status_reg & DWT_INT_RXFCG_BIT_MASK)) {
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        LOG_DBG("No final message received");
        return -ETIMEDOUT;
    }
    
    /* Clear events */
    dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK | DWT_INT_TXFRS_BIT_MASK);
    
    /* Read final message */
    frame_len = dwt_getframelength(0);
    if (frame_len > RX_BUF_LEN) {
        return -EMSGSIZE;
    }
    
    dwt_readrxdata(rx_buffer, frame_len, 0);
    
    /* Verify it's a final message */
    if (rx_buffer[9] != MSG_TYPE_TWR_FINAL) {
        LOG_DBG("Unexpected message type: 0x%02X", rx_buffer[9]);
        return -EINVAL;
    }
    
    /* Get timestamps */
    resp_tx_ts = get_tx_timestamp_u64();
    final_rx_ts = get_rx_timestamp_u64();
    
    /* Extract timestamps from final message */
    uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
    final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
    final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
    final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);
    
    /* Calculate Time of Flight using DS-TWR formula */
    uint32_t poll_rx_ts_32 = (uint32_t)poll_rx_ts;
    uint32_t resp_tx_ts_32 = (uint32_t)resp_tx_ts;
    uint32_t final_rx_ts_32 = (uint32_t)final_rx_ts;
    
    double Ra = (double)(resp_rx_ts - poll_tx_ts);
    double Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
    double Da = (double)(final_tx_ts - resp_rx_ts);
    double Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
    
    int64_t tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));
    
    double tof = tof_dtu * DWT_TIME_UNITS;
    double distance = tof * SPEED_OF_LIGHT;
    
    LOG_INF("TWR with tag 0x%04X: %.3f m", source, distance);
    
    return 0;
}

int anchor_send_beacon(void)
{
    anchor_beacon_msg_t *beacon = (anchor_beacon_msg_t *)tx_beacon_msg;
    
    beacon->frame_ctrl[0] = 0x41;
    beacon->frame_ctrl[1] = 0x88;
    beacon->seq_num = frame_seq_nb++;
    beacon->pan_id[0] = 0xCA;
    beacon->pan_id[1] = 0xDE;
    pack_addr(beacon->dest_addr, UWB_BROADCAST_ID);
    pack_addr(beacon->src_addr, my_address);
    beacon->func_code = MSG_TYPE_ANCHOR_BEACON;
    beacon->x_mm = my_position.x_mm;
    beacon->y_mm = my_position.y_mm;
    beacon->z_mm = my_position.z_mm;
    beacon->is_gateway = is_gateway ? 1 : 0;
    beacon->hop_to_gateway = hop_to_gateway;
    
    dwt_writetxdata(sizeof(anchor_beacon_msg_t), tx_beacon_msg, 0);
    dwt_writetxfctrl(sizeof(anchor_beacon_msg_t) + FCS_LEN, 0, 0);
    
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    
    waitforsysstatus(NULL, NULL, DWT_INT_TXFRS_BIT_MASK, 0);
    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
    
    LOG_DBG("Beacon sent");
    
    return 0;
}

void anchor_handle_position(const uint8_t *rx_buffer, uint16_t length)
{
    if (length < sizeof(tag_position_msg_t)) {
        return;
    }
    
    const tag_position_msg_t *msg = (const tag_position_msg_t *)rx_buffer;
    
    uint16_t tag_id = unpack_addr(msg->src_addr);
    
    LOG_INF("Position from tag 0x%04X: (%.3f, %.3f, %.3f) m, battery: %u%%",
            tag_id,
            msg->x_mm / 1000.0f,
            msg->y_mm / 1000.0f,
            msg->z_mm / 1000.0f,
            msg->battery_pct);
    
    /* Forward to gateway */
    anchor_forward_to_gateway(rx_buffer, length);
}

int anchor_forward_to_gateway(const uint8_t *data, uint16_t length)
{
    if (is_gateway) {
        /* We are the gateway - send via UART */
#if CONFIG_UWB_IPS_GATEWAY
        extern int gateway_send_uart(const uint8_t *data, uint16_t length);
        return gateway_send_uart(data, length);
#else
        return 0;
#endif
    } else {
        /* Forward via UWB mesh */
        uint16_t next_hop;
        int ret = mesh_get_next_hop(&next_hop);
        
        if (ret != 0) {
            LOG_WRN("No route to gateway");
            return -ENETUNREACH;
        }
        
        /* Send to next hop */
        dwt_writetxdata(length, data, 0);
        dwt_writetxfctrl(length + FCS_LEN, 0, 0);
        
        dwt_starttx(DWT_START_TX_IMMEDIATE);
        
        waitforsysstatus(NULL, NULL, DWT_INT_TXFRS_BIT_MASK, 0);
        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
        
        LOG_DBG("Forwarded to anchor 0x%04X", next_hop);
        
        return 0;
    }
}

