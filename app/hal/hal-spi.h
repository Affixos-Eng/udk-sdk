/**
 * LEAPS - Low Energy Accurate Positioning System.
 *
 * Hardware Abstraction Layer - SPI.
 *
 * Copyright (c) 2025, LEAPS. All rights reserved.
 *
 */
#ifndef __HAL_SPI_H__
#define __HAL_SPI_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Zephyr */
#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>

/* Max buffer length of SPI */
#define MAX_SPI_BUFFER_LENGTH 128

/* Hardware pin */
#define UWB_CS_PIN          DT_GPIO_PIN(DT_NODELABEL(spi3), cs_gpios)
#define UWB_CS_PIN_FLAGS    DT_GPIO_FLAGS(DT_NODELABEL(spi3), cs_gpios)
#define UWB_CS_PIN_CTRL     DT_LABEL(DT_GPIO_CTLR(DT_NODELABEL(spi3), cs_gpios))
#define UWB_CS_PIN_NUM      DT_GPIO_PIN(DT_ALIAS(uwb0_spi), cs_gpios)

/* Default bit rate */
#define UWB_SPI_MAX_FREQ              32000000
#define UWB_SPI_MIN_FREQ              2000000

/* Wrapper function to keep Qorvo's example code is unchanged */
#define port_set_dw_ic_spi_slowrate   hal_spi_init
#define port_set_dw_ic_spi_fastrate   hal_spi_init_fastrate

/**
 * @brief Initialize the Spi (default speed is slow rate)
 *
 * @param[in] none
 *
 * @return -1/0     if intialization is failed/success
 */
extern int hal_spi_init(void);

/**
 * @brief Initialize the Spi (default speed is fast rate), to keep the Qorvo's
 *        example code is unchanged
 *
 * @param[in] none
 *
 * @return -1/0     if intialization is failed/success
 */
extern int hal_spi_init_fastrate(void);

/**
 * @brief Reading the spi data
 *
 * @param[in] hdr_len   number of bytes header to write
 * @param[in] hdr_buf   pointer to buffer containing the 'hdr_len' bytes of
 *                      header to write
 * @param[in] read_len  number of bytes data being read
 * @param[in] read_buf  pointer to buffer containing to return the data
 *                      (NB: size required = hdr_len + read_len)
 *
 * @return -1 (DWT_ERROR)/0(DWT_SUCCESS) if reading data is failed/success
 */
extern int hal_spi_read(uint16_t hdr_len, uint8_t *hdr_buf,
                                          uint16_t read_len, uint8_t *read_buf);

/**
 * @brief Write the spi data
 *
 * @param[in] hdr_len    number of bytes header being written
 * @param[in] hdr_buf    pointer to buffer containing the 'hdr_len' bytes of
 *                       header to be written
 * @param[in] body_len   number of bytes data being written
 * @param[in] body_buf   pointer to buffer containing the 'body_len' bytes od
 *                       data to be written
 *
 * @return -1 (DWT_ERROR)/0(DWT_SUCCESS)     if write data is failed/success
 */
extern int hal_spi_write(uint16_t hdr_len, const uint8_t *hdr_buf,
                                    uint16_t body_len, const uint8_t *body_buf);

/**
 * @brief
 *
 * @param[in] headerLength    number of bytes header being written
 * @param[in] headerBuffer    pointer to buffer containing the 'headerLength'
 *                            bytes of header to be written
 * @param[in] bodyLength      number of bytes data being written
 * @param[in] bodyBuffer      pointer to buffer containing the 'bodylength'
 *                            bytes od data to be written
 * @param[in] crc8            8-bit crc, calculated on the header and data bytes
 *
 * @return -1 (DWT_ERROR)/0(DWT_SUCCESS)     if write data is failed/success
 */
extern int hal_spi_writewithcrc(uint16_t headerLength,
                               const uint8_t *headerBuffer, uint16_t bodyLength,
                                       const uint8_t *bodyBuffer, uint8_t crc8);

/**
 * @brief Configure the spi speed is slow rate
 *
 * @param[in] none
 *
 * @return none
 */
extern void hal_spi_setslowrate(void);

/**
 * @brief Configure the spi speed is fast rate
 *
 * @param[in] none
 *
 * @return none
 */
extern void hal_spi_setfastrate(void);

#endif /* __HAL_SPI_H__ */

