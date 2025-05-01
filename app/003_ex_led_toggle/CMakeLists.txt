# LEAPS - Low Energy Accurate Positioning System.
# 
# Copyright (c) 2025, LEAPS. All rights reserved.
# 
# HAL layer 
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../hal)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../hal/hal-gpio.c)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../hal/hal-console.c)
# main application
target_sources(app PRIVATE led_toggle.c)