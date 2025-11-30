/**
 * @file anchor_gateway.c
 * @brief Gateway UART interface for forwarding position data to host
 */

#include "../common/uwb_ips.h"
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gateway, LOG_LEVEL_INF);

#if CONFIG_UWB_IPS_GATEWAY

/*============================================================================
 * UART Frame Format
 *============================================================================*/

#define UART_FRAME_START    0xAA
#define UART_FRAME_END      0x55

/* Frame: [START][TYPE][LEN_LO][LEN_HI][PAYLOAD...][CRC_LO][CRC_HI][END] */

typedef struct __attribute__((packed)) {
    uint8_t start;
    uint8_t msg_type;
    uint16_t length;
} uart_header_t;

typedef struct __attribute__((packed)) {
    uint16_t crc;
    uint8_t end;
} uart_footer_t;

/*============================================================================
 * Static Variables
 *============================================================================*/

static const struct device *uart_dev = NULL;

/*============================================================================
 * Public Functions
 *============================================================================*/

int gateway_uart_init(void)
{
    /* Get UART device */
    uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not ready");
        return -ENODEV;
    }
    
    LOG_INF("Gateway UART initialized");
    
    return 0;
}

int gateway_send_uart(const uint8_t *data, uint16_t length)
{
    if (uart_dev == NULL) {
        return -ENODEV;
    }
    
    /* Determine message type */
    uint8_t msg_type = MSG_TYPE_TAG_POSITION;
    if (length >= 10) {
        msg_type = data[9];  /* Function code at offset 9 */
    }
    
    /* Calculate CRC of payload */
    uint16_t crc = uwb_crc16(data, length);
    
    /* Send header */
    uart_poll_out(uart_dev, UART_FRAME_START);
    uart_poll_out(uart_dev, msg_type);
    uart_poll_out(uart_dev, length & 0xFF);
    uart_poll_out(uart_dev, (length >> 8) & 0xFF);
    
    /* Send payload */
    for (uint16_t i = 0; i < length; i++) {
        uart_poll_out(uart_dev, data[i]);
    }
    
    /* Send footer */
    uart_poll_out(uart_dev, crc & 0xFF);
    uart_poll_out(uart_dev, (crc >> 8) & 0xFF);
    uart_poll_out(uart_dev, UART_FRAME_END);
    
    LOG_DBG("Sent %u bytes via UART", length);
    
    return 0;
}

#else /* !CONFIG_UWB_IPS_GATEWAY */

int gateway_uart_init(void)
{
    return 0;
}

int gateway_send_uart(const uint8_t *data, uint16_t length)
{
    ARG_UNUSED(data);
    ARG_UNUSED(length);
    return -ENOTSUP;
}

#endif /* CONFIG_UWB_IPS_GATEWAY */

