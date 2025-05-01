# LEAPS - Low Energy Accurate Positioning System.
# 
# Copyright (c) 2025, LEAPS. All rights reserved.
#
# main example 
if(CONFIG_TEST_TX_SLEEP)
target_sources(app PRIVATE tx_sleep.c)
else()
target_sources(app PRIVATE tx_sleep_idleRC.c)
endif()
