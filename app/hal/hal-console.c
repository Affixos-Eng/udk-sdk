/**
 * LEAPS - Low Energy Accurate Positioning System.
 *
 * Define test_run_info function to make it compatiable with Qorvo's examples
 * on Zephyr enviroment
 *
 * Copyright (c) 2025, LEAPS. All rights reserved.
 *
 */

#include "hal-console.h"

/* Described in header file */
void test_run_info(unsigned char *data)
{
    printk("%s\n", data);
}
