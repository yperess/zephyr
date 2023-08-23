/*
 * Copyright (c) 2023 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_DATA_TYPES_H
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_DATA_TYPES_H

#include <zephyr/dsp/types.h>

#ifdef __cplusplus
extern "C" {
#endif

struct sensor_data_header {
	/**
	 * The closest timestamp for when the first frame was generated as attained by
	 * :c:func:`k_uptime_ticks`.
	 */
	uint64_t base_timestamp_ns;
	/**
	 * The number of elements in the 'readings' array.
	 *
	 * This must be at least 1
	 */
	uint16_t reading_count;
};

/**
 * Data for a sensor channel which reports on three axes. This is used by:
 * - :c:enum:`SENSOR_CHAN_ACCEL_X`
 * - :c:enum:`SENSOR_CHAN_ACCEL_Y`
 * - :c:enum:`SENSOR_CHAN_ACCEL_Z`
 * - :c:enum:`SENSOR_CHAN_ACCEL_XYZ`
 * - :c:enum:`SENSOR_CHAN_GYRO_X`
 * - :c:enum:`SENSOR_CHAN_GYRO_Y`
 * - :c:enum:`SENSOR_CHAN_GYRO_Z`
 * - :c:enum:`SENSOR_CHAN_GYRO_XYZ`
 * - :c:enum:`SENSOR_CHAN_MAGN_X`
 * - :c:enum:`SENSOR_CHAN_MAGN_Y`
 * - :c:enum:`SENSOR_CHAN_MAGN_Z`
 * - :c:enum:`SENSOR_CHAN_MAGN_XYZ`
 * - :c:enum:`SENSOR_CHAN_POS_DX`
 * - :c:enum:`SENSOR_CHAN_POS_DY`
 * - :c:enum:`SENSOR_CHAN_POS_DZ`
 */
struct sensor_three_axis_data {
	struct sensor_data_header header;
	int8_t shift;
	struct sensor_three_axis_sample_data {
		uint32_t timestamp_delta;
		union {
			q31_t values[3];
			q31_t v[3];
			struct {
				q31_t x;
				q31_t y;
				q31_t z;
			};
		};
	} readings[1];
};

/**
 * Data from a sensor where we only care about an event occurring. This is used to report triggers.
 */
struct sensor_occurrence_data {
	struct sensor_data_header header;
	struct sensor_occurrence_sample_data {
		uint32_t timestamp_delta;
	} readings[1];
};

struct sensor_q31_data {
	struct sensor_data_header header;
	int8_t shift;
	struct sensor_q31_sample_data {
		uint32_t timestamp_delta;
		union {
			q31_t value;
			q31_t light; /**< Unit: lux */
			q31_t pressure; /**< Unit: kilopascal */
			q31_t temperature; /**< Unit: degrees Celsius */
			q31_t percent; /**< Unit: percent */
			q31_t distance; /**< Unit: meters */
			q31_t density; /**< Unit: ug/m^3 */
			q31_t density_ppm; /**< Unit: parts per million */
			q31_t density_ppb; /**< Unit: parts per billion */
			q31_t resistance; /**< Unit: ohms */
			q31_t voltage; /**< Unit: volts */
			q31_t current; /**< Unit: amps */
			q31_t power; /**< Unit: watts */
			q31_t angle; /**< Unit: degrees */
			q31_t electric_charge; /**< Unit: mAh */
		};
	} readings[1];
};

/**
 * Data from a sensor that produces a byte of data. This is used by:
 * - :c:enum:`SENSOR_CHAN_PROX`
 */
struct sensor_byte_data {
	struct sensor_data_header header;
	struct sensor_byte_sample_data {
		uint32_t timestamp_delta;
		union {
			uint8_t value;
			struct {
				uint8_t is_near: 1;
				uint8_t padding: 7;
			};
		};
	} readings[1];
};

/**
 * Data from a sensor that produces a count like value. This is used by:
 * - :c:enum:`SENSOR_CHAN_GAUGE_CYCLE_COUNT`
 */
struct sensor_uint64_data {
	struct sensor_data_header header;
	struct sensor_uint64_sample_data {
		uint32_t timestamp_delta;
		uint64_t value;
	} readings[1];
};

/**
 * A union of all sensor data types.
 */
union sensor_data_union {
	struct sensor_three_axis_data three_axis;
	struct sensor_occurrence_data occurence;
	struct sensor_q31_data q31;
	struct sensor_byte_data byte;
	struct sensor_uint64_data uint64;
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_DATA_TYPES_H */
