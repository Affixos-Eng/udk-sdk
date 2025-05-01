/**
 * LEAPS - Low Energy Accurate Positioning System.
 *
 * Hardware Abstraction Layer - SPI.
 *
 * Copyright (c) 2025, LEAPS. All rights reserved.
 *
 */

#include "hal-spi.h"

/* header for decamutex<on/off> */
#include "deca_device_api.h"

static const struct device *cs_dev = NULL;
static const struct device *spi_dev = NULL;
static struct spi_config spi_cfg_slowrate;
static struct spi_config spi_cfg_fastrate;
static struct spi_config spi_cfg;
static struct spi_cs_control spi_cs;

/* TX buffer. */
static uint8_t m_tx_buf[MAX_SPI_BUFFER_LENGTH] __attribute__((aligned(4)));

/* RX buffer. */
static uint8_t m_rx_buf[MAX_SPI_BUFFER_LENGTH] __attribute__((aligned(4)));

static struct spi_buf rx = {
    .buf = m_rx_buf,
    .len = 0,
};

static struct spi_buf_set rx_bufs = {
    .buffers = &rx,
    .count = 1,
};

static struct spi_buf tx = {
    .buf = m_tx_buf,
    .len = 0,
};

static struct spi_buf_set tx_bufs = {
    .buffers = &tx,
    .count = 1,
};

/* Described in header file */
int hal_spi_init(void)
{
    if (spi_dev) {
        return -1;
    }

    spi_dev = device_get_binding(DT_LABEL(DT_ALIAS(uwb0_spi)));

    if (NULL == spi_dev) {
        return -1;
    }

    cs_dev = device_get_binding(UWB_CS_PIN_CTRL);

    if (NULL == cs_dev) {
        return -1;
    }

    spi_cs.gpio.port = cs_dev;
    spi_cs.delay = 0;
    spi_cs.gpio.pin = UWB_CS_PIN_NUM;

    /* To remove 1st zero byte from uwb */
    gpio_pin_set(spi_cs.gpio.port, spi_cs.gpio.pin, 1);
    gpio_pin_set(spi_cs.gpio.port, spi_cs.gpio.pin, 0);

    spi_cfg_slowrate.cs = (const struct spi_cs_control*) &spi_cs;
    spi_cfg_fastrate.cs = (const struct spi_cs_control*) &spi_cs;

    spi_cfg_slowrate.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8);
    spi_cfg_slowrate.frequency = UWB_SPI_MIN_FREQ;

    spi_cfg_fastrate.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8);
    spi_cfg_fastrate.frequency = UWB_SPI_MAX_FREQ;

    /* Default value is slow rate */
    spi_cfg = spi_cfg_slowrate;

    return 0;
}

/* Described in header file */
int hal_spi_read(uint16_t hdr_len, /*const*/ uint8_t *hdr_buf,
                 uint16_t read_len, uint8_t *read_buf)
{
    int err;
    if (spi_dev == NULL) {
        printk("hal_spi_read - spi_dev NULL\n");
        return -1;
    }
    tx.len = hdr_len + read_len;
    rx.len = tx.len;

    if (rx.len > sizeof(m_tx_buf)) {
        printk("hal_spi_read - overflow\n");
        return -1;
    }

    memmove(m_tx_buf, hdr_buf, hdr_len);

    /* underlying reading data */
    err = spi_transceive(spi_dev, &spi_cfg, &tx_bufs, &rx_bufs);
    if (err) {
        printk("hal_spi_read - error: %d\n", err);
        return -1;
    }

    memcpy(read_buf, &m_rx_buf[hdr_len], read_len);

    return 0;
}

/* Described in header file */
int hal_spi_write(uint16_t hdr_len, const uint8_t *hdr_buf,
                 uint16_t body_len, const uint8_t *body_buf)
{
    int err = 0;
    uint32_t m_length = hdr_len + body_len; /**< Transfer length. */

    if (m_length > sizeof(m_tx_buf)) {
        printk("hal_spi_write - error: %d\n", err);
        return -1;
    }

    memmove(m_tx_buf, hdr_buf, hdr_len);
    memmove(m_tx_buf + hdr_len, body_buf, body_len);

    tx.len = m_length;
    rx.len = m_length;

    err = spi_transceive(spi_dev, &spi_cfg, &tx_bufs, &rx_bufs);

    if (err != 0) {
        printk("hal_spi_write - error: %d\n", err);
        return -1;
    }

    return 0;
}

/* Described in header file */
int hal_spi_writewithcrc(uint16_t hdr_len, const uint8_t *hdr_buf,
                       uint16_t body_len, const uint8_t *body_buf, uint8_t crc8)
{
    int err = 0;
    decaIrqStatus_t stat;
    uint32_t m_length = hdr_len + body_len + 1; /**< Transfer length. */

    if (m_length > MAX_SPI_BUFFER_LENGTH) {
        printk("hal_spi_writewithcrc - error: %d\n", err);
        return -1;
    }

    stat = decamutexon();

    memmove(m_tx_buf, hdr_buf, hdr_len);
    memmove(m_tx_buf + hdr_len, body_buf, body_len);
    m_tx_buf[hdr_len + body_len] = crc8;

    tx.len = m_length;

    err = spi_transceive(spi_dev, &spi_cfg, &tx_bufs, NULL);
    if (err != 0) {
        decamutexoff(stat);
        printk("hal_spi_writewithcrc - error: %d\n", err);
        return -1;
    }

    decamutexoff(stat);
    return 0;
}

/* Described in header file */
void hal_spi_setslowrate(void)
{
    spi_cfg = spi_cfg_slowrate;
}

/* Described in header file */
void hal_spi_setfastrate(void)
{
    spi_cfg = spi_cfg_fastrate;
}

/* Described in header file */
int hal_spi_init_fastrate(void)
{
    int err = 0;

    /* Initialize spi with default slow rate */
    err = hal_spi_init();

    if (err != 0) {
        return -1;
    }
    /* Switch to fast rate */
    spi_cfg = spi_cfg_fastrate;

    return 0;
}

/* End of file */
