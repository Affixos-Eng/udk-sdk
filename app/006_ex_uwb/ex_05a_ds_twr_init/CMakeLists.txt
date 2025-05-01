# LEAPS - Low Energy Accurate Positioning System.
# 
# Copyright (c) 2025, LEAPS. All rights reserved.
# 
# main example 
if(CONFIG_TEST_DS_TWR_INITIATOR_STS)
target_sources(app PRIVATE ds_twr_initiator_sts.c)
else()
target_sources(app PRIVATE ds_twr_initiator.c)
endif()
