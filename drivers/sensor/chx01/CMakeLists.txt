# Copyright (c) 2023 Google LLC
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

add_subdirectory(chirpmicro)

zephyr_library_sources_ifdef(CONFIG_CH101 ch101.c)
