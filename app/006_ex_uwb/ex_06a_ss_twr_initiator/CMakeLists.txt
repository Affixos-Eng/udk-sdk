# LEAPS - Low Energy Accurate Positioning System.
# 
# Copyright (c) 2025, LEAPS. All rights reserved.
# 
# main example 
if(CONFIG_TEST_SS_TWR_INITIATOR_STS_NO_DATA)
target_sources(app PRIVATE ss_twr_initiator_sts_no_data.c)
elseif(CONFIG_TEST_SS_TWR_INITIATOR_STS)
target_sources(app PRIVATE ss_twr_initiator_sts.c)
else()
target_sources(app PRIVATE ss_twr_initiator.c)
endif()
