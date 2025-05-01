# LEAPS - Low Energy Accurate Positioning System.
#
# Copyright (c) 2025, LEAPS. All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(example VERSION 1.0)
# include example_selection.h
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../app)
# Specify source files for the app target
target_sources(app PRIVATE main.c)
# Include directories for Zephyr and your additional directories
target_include_directories(app PRIVATE ${ZEPHYR_INCLUDE_DIRS})
# Add subdirectories
if(CONFIG_TEST_BUTTONS)
  # buttons example 
  add_subdirectory(001_ex_press_buttons)
elseif(CONFIG_TEST_BUZZER)
  # buzzer alarm beep sound
  add_subdirectory(002_ex_buzzer_alarm)
elseif(CONFIG_TEST_LED)
  # toggle LED
  add_subdirectory(003_ex_led_toggle)  
elseif(CONFIG_TEST_MOTOR)
  # motor vibrate
  add_subdirectory(004_ex_motor_vibration) 
elseif(CONFIG_TEST_HW_INTERFACE_COMBINATION)
  # combning some hw interfaces
  add_subdirectory(005_ex_hwinterfaces_combination) 
else()# otherwise is test UWB
  add_subdirectory(006_ex_uwb)
endif()