/*
 * Copyright (c) 2023 Google LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_CH101_H
#define ZEPHYR_DRIVERS_SENSOR_CH101_H

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor/ch_common.h>

#define CH101_DATA_MEM_SIZE 0x800
#define CH101_DATA_MEM_ADDR 0x0200
#define CH101_PROG_MEM_SIZE 0x800
#define CH101_PROG_MEM_ADDR 0xF800
#define CH101_FW_SIZE	    CH101_PROG_MEM_SIZE

/** Index of first sample to use for calculating bandwidth. */
#define CH101_BANDWIDTH_INDEX_1 6
/** Index of second sample to use for calculating bandwidth. */
#define CH101_BANDWIDTH_INDEX_2 (CH101_BANDWIDTH_INDEX_1 + 1)
/** Index for calculating scale factor. */
#define CH101_SCALEFACTOR_INDEX 4

#define CH101_MAX_TICK_INTERVAL 256

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Flash a custom firmware image to the CH101
 *
 * @param dev The device to flash
 * @param firmware Firmware binary, must have size CH101_FW_SIZE
 * @return 0 on success
 * @return < 0 on failure
 */
int ch101_flash_firmware(const struct device *dev, const struct ch_firmware *firmware);

/**
 * @brief General purpose range-finder firmware
 */
extern const struct ch_firmware ch101_gpr_fw;

/**
 * @brief Short range general purpose range-finder firmware
 */
extern const struct ch_firmware ch101_gpr_sr_fw;

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_CH101_H */
