/**
 * LEAPS - Low Energy Accurate Positioning System.
 *
 * Declaration of test_run_info function
 *
 * Copyright (c) 2025, LEAPS. All rights reserved.
 *
 */

#ifndef __HAL_CONSOLE_H__
#define __HAL_CONSOLE_H__

/* Zephyr lib inclusion */
#include "zephyr.h"

/**
 * @brief This function is simply a printk() call for a string. In order to keep
 *        the Qorvo's example code is unchanged.
 *
 * @param[in] data message data, this data should be NULL string.
 *
 * @return none.
 */
extern void test_run_info(unsigned char *data);

#endif /* #ifndef __HAL_CONSOLE_H__ */
