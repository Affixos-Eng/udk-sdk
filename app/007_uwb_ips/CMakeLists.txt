# UWB Indoor Positioning System
# SPDX-License-Identifier: Apache-2.0

# Include HAL from SDK
target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../hal/hal-spi.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../hal/hal-gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../hal/hal-console.c
)

# Include shared functions from UWB examples
target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../006_ex_uwb/shared_data/shared_functions.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../006_ex_uwb/shared_data/probe_interface.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../006_ex_uwb/shared_data/config_options.c
)

# Include directories
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/../006_ex_uwb/shared_data
    ${CMAKE_CURRENT_SOURCE_DIR}/../hal
)

# Common sources
target_sources(app PRIVATE
    common/uwb_config.c
    common/position_calc.c
    common/battery_mgmt.c
    common/ble_mesh.c
    common/uwb_utils.c
)

# Tag or Anchor specific
if(CONFIG_UWB_IPS_TAG)
    target_sources(app PRIVATE
        tag/tag_main.c
        tag/tag_twr.c
    )
elseif(CONFIG_UWB_IPS_ANCHOR)
    target_sources(app PRIVATE
        anchor/anchor_main.c
        anchor/anchor_twr.c
        anchor/anchor_gateway.c
    )
endif()

