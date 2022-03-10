/*
 * Copyright (c) 2022 Google LLC.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/shell/shell.h>

#define SENSOR_GET_HELP \
	"Get sensor data. Sensor types are optional. All types are read when no types are " \
	"provided. Syntax:\n"  \
	"<device_name> <sensor_type_0> .. <sensor_type_N>"

#define SENSOR_INFO_HELP \
	"Get sensor info, such as vendor and model name, for all sensors."

static const char *sensor_type_name[SENSOR_TYPE_VENDOR_START] = {
	[SENSOR_TYPE_ACCELEROMETER] = "accel",
	[SENSOR_TYPE_GYROSCOPE] = "gyro",
};

static int handle_sensor_type_by_name(const struct shell *shell,
				      const struct device *dev,
				      const char *type_name)
{
	const struct sensor_raw_data *data;
	char *endptr;
	int err;
	uint32_t i;

	/* Attempt to parse sensor type as a number first */
	i = strtoul(type_name, &endptr, 0);

	if (*endptr != '\0') {
		/* Sensor type is not a number, look it up */
		for (i = 0; i < ARRAY_SIZE(sensor_type_name); ++i) {
			if (strcmp(type_name, sensor_type_name[i]) == 0) {
				break;
			}
		}

		if (i == ARRAY_SIZE(sensor_type_name)) {
			shell_error(shell, "Sensor type not supported (%s)", type_name);
			return -ENOTSUP;
		}
	}

	err = sensor_read_data(dev, &i, 1, &data);
	if (err != 0) {
		return err;
	}

	if (i >= ARRAY_SIZE(sensor_type_name)) {
		shell_print(shell, "sensor type=%" PRIu32 ", timestamp=%" PRIu64 ", raw value=",
			    i, data->header.base_timestamp);
		shell_hexdump(shell, data->readings, data->header.reading_count);
	} else {
		shell_print(shell, "sensor type=%" PRIu32 " %s, timestamp=%" PRIu64 ", raw value=",
			    i, sensor_type_name[i], data->header.base_timestamp);
		shell_hexdump(shell, data->readings, data->header.reading_count);
	}

	return 0;
}

static int cmd_get_sensor(const struct shell *shell, size_t argc, char *argv[])
{
	const struct device *dev;
	int err;

	dev = device_get_binding(argv[1]);
	if (dev == NULL) {
		shell_error(shell, "Device unknown (%s)", argv[1]);
		return -ENODEV;
	}

	if (argc == 2) {
		/* Read all sensor types */
		for (int i = 0; i < ARRAY_SIZE(sensor_type_name); ++i) {
			if (sensor_type_name[i] != NULL) {
				handle_sensor_type_by_name(shell, dev, sensor_type_name[i]);
			}
		}
	} else {
		for (int i = 2; i < argc; ++i) {
			err = handle_sensor_type_by_name(shell, dev, argv[i]);
			if (err != 0) {
				shell_error(shell, "Failed to read sensor type (%s)", argv[i]);
			}
		}
	}

	return 0;
}

static void sensor_type_name_get(size_t idx, struct shell_static_entry *entry);

SHELL_DYNAMIC_CMD_CREATE(dsub_sensor_type_name, sensor_type_name_get);

static void sensor_type_name_get(size_t idx, struct shell_static_entry *entry)
{
	int cnt = 0;

	entry->syntax = NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = &dsub_sensor_type_name;

	for (int i = 0; i < SENSOR_TYPE_VENDOR_START; ++i) {
		if (sensor_type_name[i] != NULL) {
			if (cnt == idx) {
				entry->syntax = sensor_type_name[i];
				break;
			}
			cnt++;
		}
	}
}

static void device_name_get(size_t idx, struct shell_static_entry *entry);

SHELL_DYNAMIC_CMD_CREATE(dsub_device_name, device_name_get);

static void device_name_get(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, NULL);

	entry->syntax = (dev != NULL) ? dev->name : NULL;
	entry->handler = NULL;
	entry->help  = NULL;
	entry->subcmd = &dsub_sensor_type_name;
}
static int cmd_get_sensor_info(const struct shell *sh, size_t argc,
		char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#ifdef CONFIG_SENSOR_INFO
	STRUCT_SECTION_FOREACH(sensor_info, sensor) {
		shell_print(sh,
			    "device name: %s, vendor: [%s], model: [%s], "
			    "friendly name: [%s]",
			    sensor->dev->name, sensor->vendor, sensor->model,
			    sensor->friendly_name);
	}
	return 0;
#else
	return -EINVAL;
#endif
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_sensor, SHELL_CMD_ARG(get, &dsub_device_name, SENSOR_GET_HELP, cmd_get_sensor, 2, 255),
	SHELL_COND_CMD(CONFIG_SENSOR_INFO, info, NULL, SENSOR_INFO_HELP, cmd_get_sensor_info),
	SHELL_SUBCMD_SET_END
	);

SHELL_CMD_REGISTER(sensor, &sub_sensor, "Sensor commands", NULL);
