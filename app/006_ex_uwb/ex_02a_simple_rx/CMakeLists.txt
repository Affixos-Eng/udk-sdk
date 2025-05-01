# LEAPS - Low Energy Accurate Positioning System.
# 
# Copyright (c) 2025, LEAPS. All rights reserved.
#
# main example 
if(CONFIG_TEST_SIMPLE_RX_NLOS)
target_sources(app PRIVATE simple_rx_nlos.c)
else()
target_sources(app PRIVATE simple_rx.c)
endif()