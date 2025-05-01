/**
 * LEAPS - Low Energy Accurate Positioning System.
 *
 * Re-define the test_app function based on Menu Selection
 *
 * Copyright (c) 2025, LEAPS. All rights reserved.
 *
 */

#ifndef _EXMAPLE_SELECTION_H_
#define _EXMAPLE_SELECTION_H_

/* Purpose: make it common for all example to avoid change the Qorvo examples */
#include "hal-console.h"
#include "hal-timer.h"
#include "hal-spi.h"
#include "hal-gpio.h"

/**
 * @brief Wrapper the interrupt function for compatibale with Qorvo's example
 *        code (for common use also)
 * @param[in] cb Calbback handler
 */
extern void dwt_isr_wrapper(void * data);

/* Selection for example */
#if CONFIG_TEST_BUTTONS
#define TEST_BUTTONS
extern int press_buttons(void);
#define test_app press_buttons
#endif

#if CONFIG_TEST_BUZZER
#define TEST_BUZZER
extern int buzzer_alarm(void);
#define test_app buzzer_alarm
#endif

#if CONFIG_TEST_LED
#define TEST_LED
int led_toggle(void);
#define test_app led_toggle
#endif

#if CONFIG_TEST_MOTOR
#define  TEST_MOTOR
int motor_vibration(void);
#define test_app motor_vibration
#endif

#if CONFIG_TEST_HW_INTERFACE_COMBINATION
#define TEST_HW_INTERFACE_COMBINATION
int test_combination(void);
#define test_app test_combination
#endif

#if CONFIG_READING_DEV_ID
#define TEST_READING_DEV_ID
int read_dev_id(void);
#define test_app read_dev_id
#endif

#if CONFIG_TEST_OTP_WRITE
#define TEST_OTP_WRITE
int otp_write(void);
#define test_app otp_write
#endif

#if CONFIG_TEST_SPI_CRC
#define TEST_SPI_CRC
int spi_crc(void);
#define test_app spi_crc
#endif

#if CONFIG_TEST_GPIO
#define TEST_GPIO
int gpio_example(void);
#define test_app gpio_example
#endif

#if CONFIG_TEST_SIMPLE_TX
#define TEST_SIMPLE_TX
int simple_tx(void);
#define test_app simple_tx
#endif

#if CONFIG_TEST_TX_SLEEP_IDLE_RC
#define TEST_TX_SLEEP_IDLE_RC
int tx_sleep_idleRC(void);
#define test_app tx_sleep_idleRC
#endif

#if CONFIG_TEST_TX_SLEEP
#define TEST_TX_SLEEP
int tx_sleep(void);
#define test_app tx_sleep
#endif

#if CONFIG_TEST_TX_SLEEP_AUTO
#define TEST_TX_SLEEP_AUTO
int tx_sleep_auto(void);
#define test_app tx_sleep_auto
#endif

#if CONFIG_TEST_TX_SLEEP_TIMED
#define TEST_TX_SLEEP_TIMED
int tx_timed_sleep(void);
#define test_app tx_timed_sleep
#endif

#if CONFIG_TEST_TX_WITH_CCA
#define TEST_TX_WITH_CCA
int tx_with_cca(void);
#define test_app tx_with_cca
#endif

#if CONFIG_TEST_SIMPLE_TX_STS_SDC
#define TEST_SIMPLE_TX_STS_SDC
int simple_tx_sts_sdc(void);
#define test_app simple_tx_sts_sdc
#endif

#if CONFIG_TEST_SIMPLE_TX_PDOA
#define TEST_SIMPLE_TX_PDOA
int simple_tx_pdoa(void);
#define test_app simple_tx_pdoa
#endif

#if CONFIG_TEST_SIMPLE_TX_AES
#define TEST_SIMPLE_TX_AES
int simple_tx_aes(void);
#define test_app simple_tx_aes
#endif

#if CONFIG_TEST_SIMPLE_TX_AUTOMOTIVE
#define TEST_SIMPLE_TX_AUTOMOTIVE
int simple_tx_automotive(void);
#define test_app simple_tx_automotive
#endif

#if CONFIG_TEST_SIMPLE_RX_NLOS
#define TEST_SIMPLE_RX_NLOS
int simple_rx_nlos(void);
#define test_app simple_rx_nlos
#endif

#if CONFIG_TEST_SIMPLE_RX
#define TEST_SIMPLE_RX
int simple_rx(void);
#define test_app simple_rx
#endif

#if CONFIG_TEST_RX_DIAG
#define TEST_RX_DIAG
int rx_diagnostics(void);
#define test_app rx_diagnostics
#endif

#if CONFIG_TEST_RX_SNIFF
#define TEST_RX_SNIFF
int rx_sniff(void);
#define test_app rx_sniff
#endif

#if CONFIG_TEST_DOUBLE_BUFFER_RX
#define TEST_DOUBLE_BUFFER_RX
int double_buffer_rx(void);
#define test_app double_buffer_rx
#endif

#if CONFIG_TEST_RX_TRIM
#define TEST_RX_TRIM
int rx_with_xtal_trim(void);
#define test_app rx_with_xtal_trim
#endif

#if CONFIG_TEST_SIMPLE_RX_STS_SDC
#define TEST_SIMPLE_RX_STS_SDC
int simple_rx_sts_sdc(void);
#define test_app simple_rx_sts_sdc
#endif

#if CONFIG_TEST_SIMPLE_RX_PDOA
#define TEST_SIMPLE_RX_PDOA
int simple_rx_pdoa(void);
#define test_app simple_rx_pdoa
#endif

#if CONFIG_TEST_SIMPLE_RX_AES
#define TEST_SIMPLE_RX_AES
int simple_rx_aes(void);
#define test_app simple_rx_aes
#endif

#if CONFIG_TEST_RX_ADC_CAPTURE
#define TEST_RX_ADC_CAPTURE
int rx_adc_capture(void);
#define test_app rx_adc_capture
#endif

#if CONFIG_TEST_SIMPLE_RX_CIR
#define TEST_SIMPLE_RX_CIR
int simple_rx_cir(void);
#define test_app simple_rx_cir
#endif

#if CONFIG_TEST_TX_WAIT_RESP
#define TEST_TX_WAIT_RESP
int tx_wait_resp(void);
#define test_app tx_wait_resp
#endif

#if CONFIG_TEST_RX_SEND_RESP
#define TEST_RX_SEND_RESP
int rx_send_resp(void);
#define test_app rx_send_resp
#endif

#if CONFIG_TEST_TX_WAIT_RESP_INT
#define TEST_TX_WAIT_RESP_INT
int tx_wait_resp_int(void);
#define test_app tx_wait_resp_int
#endif

#if CONFIG_TEST_CONTINUOUS_WAVE
#define TEST_CONTINUOUS_WAVE
int continuous_wave_example(void);
#define test_app continuous_wave_example
#endif

#if CONFIG_TEST_CONTINUOUS_FRAME
#define TEST_CONTINUOUS_FRAME
int continuous_frame_example(void);
#define test_app continuous_frame_example
#endif

#if CONFIG_TEST_DS_TWR_INITIATOR_STS
#define TEST_DS_TWR_INITIATOR_STS
int ds_twr_initiator_sts(void);
#define test_app ds_twr_initiator_sts
#endif

#if CONFIG_TEST_DS_TWR_INITIATOR
#define TEST_DS_TWR_INITIATOR
int ds_twr_initiator(void);
#define test_app ds_twr_initiator
#endif

#if CONFIG_TEST_DS_TWR_RESPONDER_STS
#define TEST_DS_TWR_RESPONDER_STS
int ds_twr_responder_sts(void);
#define test_app ds_twr_responder_sts
#endif

#if CONFIG_TEST_DS_TWR_RESPONDER
#define TEST_DS_TWR_RESPONDER
int ds_twr_responder(void);
#define test_app ds_twr_responder
#endif

#if CONFIG_TEST_DS_TWR_STS_SDC_INITIATOR
#define TEST_DS_TWR_STS_SDC_INITIATOR
int ds_twr_sts_sdc_initiator(void);
#define test_app ds_twr_sts_sdc_initiator
#endif

#if CONFIG_TEST_DS_TWR_STS_SDC_RESPONDER
#define TEST_DS_TWR_STS_SDC_RESPONDER
int ds_twr_sts_sdc_responder(void);
#define test_app ds_twr_sts_sdc_responder
#endif

#if CONFIG_TEST_SS_TWR_INITIATOR_STS_NO_DATA
#define TEST_SS_TWR_INITIATOR_STS_NO_DATA
int ss_twr_initiator_sts_no_data(void);
#define test_app ss_twr_initiator_sts_no_data
#endif

#if CONFIG_TEST_SS_TWR_INITIATOR_STS
#define TEST_SS_TWR_INITIATOR_STS
int ss_twr_initiator_sts(void);
#define test_app ss_twr_initiator_sts
#endif

#if CONFIG_TEST_SS_TWR_INITIATOR
#define TEST_SS_TWR_INITIATOR
int ss_twr_initiator(void);
#define test_app ss_twr_initiator
#endif

#if CONFIG_TEST_SS_TWR_RESPONDER_STS_NO_DATA
#define TEST_SS_TWR_RESPONDER_STS_NO_DATA
int ss_twr_responder_sts_no_data(void);
#define test_app ss_twr_responder_sts_no_data
#endif

#if CONFIG_TEST_SS_TWR_RESPONDER_STS
#define TEST_SS_TWR_RESPONDER_STS
int ss_twr_responder_sts(void);
#define test_app ss_twr_responder_sts
#endif

#if CONFIG_TEST_SS_TWR_RESPONDER
#define TEST_SS_TWR_RESPONDER
int ss_twr_responder(void);
#define test_app ss_twr_responder
#endif

#if CONFIG_TEST_AES_SS_TWR_INITIATOR
#define TEST_AES_SS_TWR_INITIATOR
int ss_aes_twr_initiator(void);
#define test_app ss_aes_twr_initiator
#endif

#if CONFIG_TEST_AES_SS_TWR_RESPONDER
#define TEST_AES_SS_TWR_RESPONDER
int ss_aes_twr_responder(void);
#define test_app ss_aes_twr_responder
#endif

#if CONFIG_TEST_ACK_DATA_TX
#define TEST_ACK_DATA_TX
int ack_data_tx(void);
#define test_app ack_data_tx
#endif

#if CONFIG_TEST_ACK_DATA_RX
#define TEST_ACK_DATA_RX
int ack_data_rx(void);
#define test_app ack_data_rx
#endif

#if CONFIG_TEST_GPIO
#define TEST_GPIO
int gpio_example(void);
#define test_app gpio_example
#endif

#if CONFIG_TEST_LE_PEND_RX
#define TEST_LE_PEND_RX
int le_pend_rx(void);
#define test_app le_pend_rx
#endif

#if CONFIG_TEST_LE_PEND_TX
#define TEST_LE_PEND_TX
int le_pend_tx(void);
#define test_app le_pend_tx
#endif

#if CONFIG_TEST_PLL_CAL
#define TEST_PLL_CAL
int pll_cal(void);
#define test_app pll_cal
#endif

#if CONFIG_TEST_BW_CAL
#define TEST_BW_CAL
int bw_cal(void);
#define test_app bw_cal
#endif

#if CONFIG_TEST_TIMER
#define TEST_TIMER
int timer_example(void);
#define test_app timer_example
#endif

#if CONFIG_TEST_TX_POWER_ADJUSTMENT
#define TEST_TX_POWER_ADJUSTMENT
int tx_power_adjustment_example(void);
#define test_app tx_power_adjustment_example
#endif

#if CONFIG_TEST_SIMPLE_AES
#define TEST_SIMPLE_AES
int simple_aes(void);
#define test_app simple_aes
#endif

#if CONFIG_TEST_LINEAR_TX_POWER
#define TEST_LINEAR_TX_POWER
int linear_tx_power_example(void);
#define test_app linear_tx_power_example
#endif

#endif /* #ifndef _EXMAPLE_SELECTION_H_ */
