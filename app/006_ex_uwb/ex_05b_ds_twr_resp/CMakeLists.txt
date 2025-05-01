# LEAPS - Low Energy Accurate Positioning System.
# 
# Copyright (c) 2025, LEAPS. All rights reserved.
#
# main example 
if(CONFIG_TEST_DS_TWR_RESPONDER_STS)
target_sources(app PRIVATE ds_twr_responder_sts.c)
else()
target_sources(app PRIVATE ds_twr_responder.c)
endif()
