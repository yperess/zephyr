/*
 * Copyright (c) 2023 Google LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_SENSOR_MEMPOOL_H
#define ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_SENSOR_MEMPOOL_H

struct sensor_mempool_entry {
	const struct device *sensor;
	uint8_t *buffer;
};

#endif /* ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_SENSOR_MEMPOOL_H */
