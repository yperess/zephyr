/*
 * Copyright (c) 2018 Diego Sueiro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(app);

struct sample_stats {
	int64_t accumulator;
	uint32_t count;
	uint64_t sample_window_start;
};

static void data_ready_trigger_handler(const struct device *sensor,
				       const struct sensor_trigger *trigger)
{
	static struct sample_stats stats[SENSOR_CHAN_ALL];
	const int64_t now = k_uptime_get();
	struct sensor_value value;

	if (sensor_sample_fetch(sensor)) {
		LOG_ERR("Failed to fetch samples on data ready handler");
	}
	for (int i = 0; i < SENSOR_CHAN_ALL; ++i) {
		/* Skip 3 axis channels */
		if (i == SENSOR_CHAN_ACCEL_XYZ || i == SENSOR_CHAN_GYRO_XYZ ||
		    i == SENSOR_CHAN_MAGN_XYZ) {
			continue;
		}
		if (sensor_channel_get(sensor, i, &value) != 0) {
			continue;
		}
		/* Do something with the data */
		stats[i].accumulator += value.val1 * INT64_C(1000000) + value.val2;
		if (stats[i].count++ == 0) {
			stats[i].sample_window_start = now;
		} else if (now > stats[i].sample_window_start + CONFIG_SAMPLE_PRINT_TIMEOUT_MS) {
			int64_t micro_value = stats[i].accumulator / stats[i].count;

			value.val1 = micro_value / 1000000;
			value.val2 = (int32_t)llabs(micro_value - (value.val1 * 1000000));
			LOG_INF("chan=%d, num_samples=%u, data=%d.%06d", i, stats[i].count,
				value.val1, value.val2);

			stats[i].accumulator = 0;
			stats[i].count = 0;
		}
	}
}

#define CHIRP DT_NODELABEL(chirp)

void main(void)
{
	const struct device *chirp = DEVICE_DT_GET(CHIRP);
	struct sensor_value val;
	int rc;

	LOG_INF("\n\n********** Starting sampling run **********\n\n");

	val.val1 = 0;
	val.val2 = 500000;
	rc = sensor_attr_set(chirp, SENSOR_CHAN_DISTANCE, SENSOR_ATTR_SAMPLING_FREQUENCY, &val);
	LOG_INF("Setting sampling frequency (%d)", rc);
	if (rc) {
		return;
	}
	k_msleep(5);

	for (int i = 0; i < 5; ++i) {
		k_msleep(5000);
		rc = sensor_sample_fetch(chirp);
		LOG_INF("\n\nFetching samples (%d)", rc);
		k_msleep(5);

		if (rc) {
			continue;
		}

		rc = sensor_channel_get(chirp, SENSOR_CHAN_DISTANCE, &val);
		LOG_INF("    Getting cached distance (%d)", rc);
		k_msleep(5);
		if (rc) {
			continue;
		}

		LOG_INF("    Distance = %d.%06dm", val.val1, val.val2);
	}
//	STRUCT_SECTION_FOREACH(sensor_info, sensor)
//	{
//		struct sensor_trigger trigger = {
//			.chan = SENSOR_CHAN_ALL,
//			.type = SENSOR_TRIG_DATA_READY,
//		};
//		sensor_trigger_set(sensor->dev, &trigger, data_ready_trigger_handler);
//	}
}
