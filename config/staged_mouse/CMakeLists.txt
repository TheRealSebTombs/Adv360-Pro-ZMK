# SPDX-License-Identifier: MIT
if ((NOT CONFIG_ZMK_SPLIT) OR CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
  target_sources(app PRIVATE src/behavior_staged_mouse.c)
endif()
