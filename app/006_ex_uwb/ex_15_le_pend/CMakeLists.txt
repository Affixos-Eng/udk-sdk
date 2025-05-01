# LEAPS - Low Energy Accurate Positioning System.
# 
# Copyright (c) 2025, LEAPS. All rights reserved.
# 
# main example 
if(CONFIG_TEST_LE_PEND_RX)
  target_sources(app PRIVATE le_pend_rx.c)
else()
  target_sources(app PRIVATE le_pend_tx.c)
endif()