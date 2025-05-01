/**
 * LEAPS - Low Energy Accurate Positioning System.
 *
 * Hardware Abstraction Layer - TIMER.
 *
 * Copyright (c) 2025, LEAPS. All rights reserved.
 *
 */

#ifndef __HAL_TIMER_H__
#define __HAL_TIMER_H__

#include "zephyr.h"

/* Re-define the hal function for the delay function */
#define hal_sys_tick()          k_cycle_get_32()
#define hal_thread_delay(t)     k_sleep(K_MSEC(t))
#define hal_delay_us(t)         k_sleep(K_USEC(t))
#define hal_delay_ms(t)         k_sleep(K_MSEC(t))

/* Re-define this function to keep Qorvo's example code is unchanged */
#define Sleep(t)                k_sleep(K_MSEC(t))

#endif /* #ifndef __HAL_TIMER_H__ */
