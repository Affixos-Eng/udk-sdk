# LEAPS - Low Energy Accurate Positioning System.
# 
# Copyright (c) 2025, LEAPS. All rights reserved.
# 
# SPDX-License-Identifier: Apache-2.0
#
# This is currently not used in implementation, keep it for future maintenance.
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/dwt_uwb_driver)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/dwt_uwb_driver/dw3000)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/dwt_uwb_driver/dw3720)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/dwt_uwb_driver/lib/qmath/include)
# 
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/dwt_uwb_driver/dw3000/dw3000_device.c)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/dwt_uwb_driver/dw3720/dw3720_device.c)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/dwt_uwb_driver/lib/qmath/src/qmath.c)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/dwt_uwb_driver/deca_compat.c)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/dwt_uwb_driver/deca_interface.c)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/dwt_uwb_driver/deca_rsl.c)
