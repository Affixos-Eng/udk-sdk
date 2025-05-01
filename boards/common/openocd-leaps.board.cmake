# LEAPS - Low Energy Accurate Positioning System.
# 
# Copyright (c) 2025, LEAPS. All rights reserved.
# 
# SPDX-License-Identifier: Apache-2.0

board_set_flasher_ifnset(openocd)
board_set_debugger_ifnset(openocd)

# "load_image" or "flash write_image erase"?
if(CONFIG_X86 OR CONFIG_ARC)
  set_ifndef(OPENOCD_USE_LOAD_IMAGE YES)
endif()
if(OPENOCD_USE_LOAD_IMAGE)
  set_ifndef(OPENOCD_FLASH load_image)
else()
  set_ifndef(OPENOCD_FLASH "flash write_image erase")
endif()

set(OPENOCD_CMD_LOAD_DEFAULT "${OPENOCD_FLASH}")
set(OPENOCD_CMD_VERIFY_DEFAULT "verify_image")

if (NOT DEFINED ENV{OPENOCD_INTERFACE})
	set_ifndef(OPENOCD_INTERFACE "jlink")
else()
	set_ifndef(OPENOCD_INTERFACE $ENV{OPENOCD_INTERFACE})
endif()

if (${OPENOCD_INTERFACE} STREQUAL "jlink")
	if ($ENV{JLINK_SERIAL})
		set_ifndef(DAPID_CMD "jlink serial $ENV{JLINK_SERIAL}")
	endif()
elseif (${OPENOCD_INTERFACE} STREQUAL "cmsis-dap")
	if ($ENV{CMSIS_SERIAL})
		set_ifndef(DAPID_CMD "cmsis_dap_serial $ENV{CMSIS_SERIAL}")
	endif()
endif()
set_ifndef(DAPID_CMD "init")

board_finalize_runner_args(openocd
  --cmd-pre-init "${DAPID_CMD}"
  --cmd-pre-load "wdt_feed"
  --cmd-load "${OPENOCD_CMD_LOAD_DEFAULT}"
  --cmd-post-verify "wdt_feed"
  --cmd-verify "${OPENOCD_CMD_VERIFY_DEFAULT}"
  --cmd-post-verify "exit_debug_mode"
  )
