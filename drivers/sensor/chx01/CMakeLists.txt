# Copyright (c) 2023 Google LLC
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_include_directories(.)
zephyr_library_sources(ch_driver.c ch_common.c)

zephyr_library_sources_ifdef(CONFIG_CH101 ch101.c)
zephyr_library_sources_ifdef(CONFIG_CH101_GPR_FW ch101_gpr_fw.c)
zephyr_library_sources_ifdef(CONFIG_CH101_GPR_SR_FW ch101_gpr_sr_fw.c)
