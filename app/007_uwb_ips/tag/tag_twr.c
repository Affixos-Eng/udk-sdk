/**
 * @file tag_twr.c
 * @brief Tag TWR (DS-TWR Initiator) implementation using LEAPS UDK-SDK
 * 
 * Based on ex_05a_ds_twr_init from LEAPS SDK examples.
 */

#include "tag_twr.h"
#include <zephyr/logging/log.h>
#include <string.h>

#include <deca_device_api.h>
#include <shared_defines.h>
#include <shared_functions.h>

LOG_MODULE_REGISTER(tag_twr, LOG_LEVEL_INF);

/*============================================================================
 * Constants and Configuration
 *============================================================================*/

#define MAX_KNOWN_ANCHORS   16
#define ANCHOR_TIMEOUT_MS   30000  /* Remove anchors not seen for 30 seconds */

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
static uint8_t tx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, MSG_TYPE_TWR_POLL };
static uint8_t rx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, MSG_TYPE_TWR_RESPONSE, 0x02, 0, 0 };
static uint8_t tx_final_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, MSG_TYPE_TWR_FINAL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

/* Position message template */
static uint8_t tx_position_msg[sizeof(tag_position_msg_t)];

/* RX buffer */
#define RX_BUF_LEN 24
static uint8_t rx_buffer[RX_BUF_LEN];

/* Frame sequence number */
static uint8_t frame_seq_nb = 0;

/* Known anchors table */
static anchor_info_t known_anchors[MAX_KNOWN_ANCHORS];
static uint8_t num_known_anchors = 0;
static K_MUTEX_DEFINE(anchor_mutex);

/* Device address */
static uint16_t my_address = 0;

/*============================================================================
 * Helper Functions
 *============================================================================*/

/* Use SDK's functions: get_tx_timestamp_u64(), get_rx_timestamp_u64() */

static void set_ts_in_msg(uint8_t *ts_field, uint64_t ts)
{
    final_msg_set_ts(ts_field, ts);
}

/*============================================================================
 * Public Functions
 *============================================================================*/

int tag_twr_init(void)
{
    LOG_INF("Initializing Tag TWR...");
    
    /* Set timing parameters for DS-TWR */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_setpreambledetecttimeout(PREAMBLE_TIMEOUT_PAC);
    
    /* Get device address from config */
    my_address = CONFIG_UWB_IPS_DEVICE_ID;
    
    /* Set source address in message templates */
    pack_addr(&tx_poll_msg[7], my_address);
    pack_addr(&tx_final_msg[7], my_address);
    
    LOG_INF("Tag TWR initialized, address: 0x%04X", my_address);
    
    return 0;
}

int tag_twr_range(uint16_t anchor_id, float *distance_m, int8_t *rssi)
{
    uint32_t status_reg;
    uint64_t poll_tx_ts, resp_rx_ts, final_tx_ts;
    
    LOG_DBG("Ranging with anchor 0x%04X", anchor_id);
    
    /* Set destination address in poll message */
    pack_addr(&tx_poll_msg[5], anchor_id);
    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    
    /* Write poll frame and start TX with response expected */
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg) + FCS_LEN, 0, 1);  /* +2 for FCS, ranging bit set */
    
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    
    /* Wait for TX complete and then RX */
    waitforsysstatus(&status_reg, NULL, 
                     (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR), 0);
    
    frame_seq_nb++;
    
    if (!(status_reg & DWT_INT_RXFCG_BIT_MASK)) {
        /* RX error or timeout */
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | DWT_INT_TXFRS_BIT_MASK);
        LOG_DBG("No response from anchor 0x%04X", anchor_id);
        return -ETIMEDOUT;
    }
    
    /* Clear RX good frame event */
    dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK | DWT_INT_TXFRS_BIT_MASK);
    
    /* Read received response */
    uint16_t frame_len = dwt_getframelength(0);
    if (frame_len > RX_BUF_LEN) {
        LOG_WRN("Response frame too long: %u", frame_len);
        return -EMSGSIZE;
    }
    
    dwt_readrxdata(rx_buffer, frame_len, 0);
    
    /* Verify response message */
    rx_buffer[ALL_MSG_SN_IDX] = 0;  /* Clear seq number for comparison */
    
    /* Set expected source/dest in template for verification */
    pack_addr(&rx_resp_msg[5], my_address);      /* Dest should be us */
    pack_addr(&rx_resp_msg[7], anchor_id);       /* Source should be anchor */
    
    if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) != 0) {
        LOG_DBG("Unexpected response format");
        return -EINVAL;
    }
    
    /* Get timestamps */
    poll_tx_ts = get_tx_timestamp_u64();
    resp_rx_ts = get_rx_timestamp_u64();
    
    /* Calculate final TX time */
    uint32_t final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(final_tx_time);
    
    /* Predicted final TX timestamp including antenna delay */
    final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
    
    /* Build and send final message */
    pack_addr(&tx_final_msg[5], anchor_id);
    tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    set_ts_in_msg(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
    set_ts_in_msg(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
    set_ts_in_msg(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);
    
    dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
    dwt_writetxfctrl(sizeof(tx_final_msg) + FCS_LEN, 0, 1);
    
    int ret = dwt_starttx(DWT_START_TX_DELAYED);
    
    if (ret != DWT_SUCCESS) {
        LOG_WRN("Delayed TX failed for final message");
        return -EIO;
    }
    
    /* Wait for TX complete */
    waitforsysstatus(NULL, NULL, DWT_INT_TXFRS_BIT_MASK, 0);
    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
    
    frame_seq_nb++;
    
    /* Calculate distance from timestamps
     * Note: In DS-TWR, the responder calculates distance. However, we have
     * all timestamps, so we can calculate here too for local display.
     * 
     * The responder in our system will also calculate and log, but tag
     * position calculation uses these distances.
     */
    
    /* Simple distance estimate from round-trip time */
    double round_trip = (double)(resp_rx_ts - poll_tx_ts);
    double tof_dtu = round_trip / 2.0;  /* Simplified - proper calc in responder */
    double tof_sec = tof_dtu * DWT_TIME_UNITS;
    
    *distance_m = (float)(tof_sec * SPEED_OF_LIGHT);
    *rssi = 0;  /* TODO: Read RSSI from diagnostics */
    
    LOG_DBG("Range to 0x%04X: %.3f m", anchor_id, *distance_m);
    
    return 0;
}

int tag_twr_range_multiple(const anchor_info_t *anchors, 
                           uint8_t num_anchors,
                           twr_measurement_t *measurements,
                           uint8_t *num_valid)
{
    *num_valid = 0;
    
    for (uint8_t i = 0; i < num_anchors && i < CONFIG_UWB_IPS_MAX_ANCHORS; i++) {
        float distance;
        int8_t rssi;
        
        int ret = tag_twr_range(anchors[i].id, &distance, &rssi);
        
        measurements[i].anchor_id = anchors[i].id;
        measurements[i].distance_m = distance;
        measurements[i].rssi = rssi;
        measurements[i].valid = (ret == 0);
        measurements[i].quality = (ret == 0) ? 100 : 0;
        
        if (ret == 0) {
            (*num_valid)++;
        }
        
        /* Small delay between ranging exchanges */
        k_msleep(10);
    }
    
    return 0;
}

int tag_send_position(uint16_t anchor_id, 
                      const position_3d_t *position,
                      const battery_status_t *battery)
{
    tag_position_msg_t *msg = (tag_position_msg_t *)tx_position_msg;
    
    /* Build IEEE 802.15.4 header */
    msg->frame_ctrl[0] = 0x41;
    msg->frame_ctrl[1] = 0x88;
    msg->seq_num = frame_seq_nb++;
    msg->pan_id[0] = 0xCA;
    msg->pan_id[1] = 0xDE;
    pack_addr(msg->dest_addr, anchor_id);
    pack_addr(msg->src_addr, my_address);
    msg->func_code = MSG_TYPE_TAG_POSITION;
    
    /* Position data */
    msg->x_mm = position->x_mm;
    msg->y_mm = position->y_mm;
    msg->z_mm = position->z_mm;
    msg->quality = position->quality;
    msg->battery_pct = battery->percentage;
    msg->is_charging = battery->is_charging ? 1 : 0;
    
    /* Send */
    dwt_writetxdata(sizeof(tag_position_msg_t), tx_position_msg, 0);
    dwt_writetxfctrl(sizeof(tag_position_msg_t) + FCS_LEN, 0, 0);
    
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    
    waitforsysstatus(NULL, NULL, DWT_INT_TXFRS_BIT_MASK, 0);
    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
    
    LOG_DBG("Sent position to anchor 0x%04X", anchor_id);
    
    return 0;
}

void tag_process_beacon(const uint8_t *rx_buffer, uint16_t length, int8_t rssi)
{
    if (length < sizeof(anchor_beacon_msg_t)) {
        return;
    }
    
    const anchor_beacon_msg_t *beacon = (const anchor_beacon_msg_t *)rx_buffer;
    
    if (beacon->func_code != MSG_TYPE_ANCHOR_BEACON) {
        return;
    }
    
    uint16_t anchor_id = unpack_addr(beacon->src_addr);
    
    LOG_DBG("Beacon from anchor 0x%04X at (%d, %d, %d) mm",
            anchor_id, beacon->x_mm, beacon->y_mm, beacon->z_mm);
    
    k_mutex_lock(&anchor_mutex, K_FOREVER);
    
    /* Look for existing entry */
    for (uint8_t i = 0; i < num_known_anchors; i++) {
        if (known_anchors[i].id == anchor_id) {
            /* Update existing */
            known_anchors[i].position.x_mm = beacon->x_mm;
            known_anchors[i].position.y_mm = beacon->y_mm;
            known_anchors[i].position.z_mm = beacon->z_mm;
            known_anchors[i].is_gateway = beacon->is_gateway;
            known_anchors[i].hop_to_gateway = beacon->hop_to_gateway;
            known_anchors[i].last_rssi = rssi;
            known_anchors[i].last_seen_ms = k_uptime_get_32();
            k_mutex_unlock(&anchor_mutex);
            return;
        }
    }
    
    /* Add new anchor if room */
    if (num_known_anchors < MAX_KNOWN_ANCHORS) {
        anchor_info_t *anchor = &known_anchors[num_known_anchors];
        anchor->id = anchor_id;
        anchor->position.x_mm = beacon->x_mm;
        anchor->position.y_mm = beacon->y_mm;
        anchor->position.z_mm = beacon->z_mm;
        anchor->is_gateway = beacon->is_gateway;
        anchor->hop_to_gateway = beacon->hop_to_gateway;
        anchor->last_rssi = rssi;
        anchor->last_seen_ms = k_uptime_get_32();
        num_known_anchors++;
        
        LOG_INF("Discovered anchor 0x%04X", anchor_id);
    }
    
    k_mutex_unlock(&anchor_mutex);
}

uint8_t tag_get_anchors(anchor_info_t *anchors, uint8_t max_count)
{
    uint32_t now = k_uptime_get_32();
    uint8_t count = 0;
    
    k_mutex_lock(&anchor_mutex, K_FOREVER);
    
    for (uint8_t i = 0; i < num_known_anchors && count < max_count; i++) {
        /* Skip stale anchors */
        if ((now - known_anchors[i].last_seen_ms) > ANCHOR_TIMEOUT_MS) {
            continue;
        }
        
        anchors[count++] = known_anchors[i];
    }
    
    k_mutex_unlock(&anchor_mutex);
    
    return count;
}

