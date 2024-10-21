# Copyright (c) 2024 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_INPUT_PINNACLE input_pinnacle.c)

target_sources_ifdef(CONFIG_ZMK_INPUT_PINNACLE_IDLE_SLEEPER app PRIVATE zmk_pinnacle_idle_sleeper.c)
