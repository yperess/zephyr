/*
 * Copyright (c) 2021 Leica Geosystems AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <ztest.h>
#include <drivers/sensor.h>

const struct device *get_fuel_gauge_device(void)
{
	const struct device *dev = DEVICE_DT_GET_ANY(sbs_sbs_gauge);

	zassert_true(device_is_ready(dev), "Fuel Gauge not found");

	return dev;
}

void test_get_sensor_value(int16_t channel)
{
	struct sensor_value value;
	const struct device *dev = get_fuel_gauge_device();

	zassert_true(sensor_sample_fetch_chan(dev, channel) < 0, "Sample fetch failed");
	zassert_true(sensor_channel_get(dev, channel, &value) < 0, "Get sensor value failed");
}

void test_get_sensor_value_not_supp(int16_t channel)
{
	const struct device *dev = get_fuel_gauge_device();

	zassert_true(sensor_sample_fetch_chan(dev, channel) == -ENOTSUP, "Invalid function");
}

ZTEST(sbs_gauge_test, test_get_gauge_voltage)
{
	test_get_sensor_value(SENSOR_CHAN_GAUGE_VOLTAGE);
}

ZTEST(sbs_gauge_test, test_get_gauge_current)
{
	test_get_sensor_value(SENSOR_CHAN_GAUGE_AVG_CURRENT);
}

ZTEST(sbs_gauge_test, test_get_stdby_current)
{
	test_get_sensor_value(SENSOR_CHAN_GAUGE_STDBY_CURRENT);
}

ZTEST(sbs_gauge_test, test_get_max_load_current)
{
	test_get_sensor_value(SENSOR_CHAN_GAUGE_MAX_LOAD_CURRENT);
}

ZTEST(sbs_gauge_test, test_get_temperature)
{
	test_get_sensor_value(SENSOR_CHAN_GAUGE_TEMP);
}

ZTEST(sbs_gauge_test, test_get_soc)
{
	test_get_sensor_value(SENSOR_CHAN_GAUGE_STATE_OF_CHARGE);
}

ZTEST(sbs_gauge_test, test_get_full_charge_capacity)
{
	test_get_sensor_value(SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY);
}

ZTEST(sbs_gauge_test, test_get_rem_charge_capacity)
{
	test_get_sensor_value(SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY);
}

ZTEST(sbs_gauge_test, test_get_nom_avail_capacity)
{
	test_get_sensor_value(SENSOR_CHAN_GAUGE_NOM_AVAIL_CAPACITY);
}

ZTEST(sbs_gauge_test, test_get_full_avail_capacity)
{
	test_get_sensor_value(SENSOR_CHAN_GAUGE_FULL_AVAIL_CAPACITY);
}

ZTEST(sbs_gauge_test, test_get_average_power)
{
	test_get_sensor_value(SENSOR_CHAN_GAUGE_AVG_POWER);
}

ZTEST(sbs_gauge_test, test_get_average_time_to_empty)
{
	test_get_sensor_value(SENSOR_CHAN_GAUGE_TIME_TO_EMPTY);
}

ZTEST(sbs_gauge_test, test_get_average_time_to_full)
{
	test_get_sensor_value(SENSOR_CHAN_GAUGE_TIME_TO_FULL);
}

ZTEST(sbs_gauge_test, test_get_cycle_count)
{
	test_get_sensor_value(SENSOR_CHAN_GAUGE_CYCLE_COUNT);
}

ZTEST(sbs_gauge_test, test_get_design_voltage)
{
	test_get_sensor_value(SENSOR_CHAN_GAUGE_DESIGN_VOLTAGE);
}

ZTEST(sbs_gauge_test, test_get_desired_voltage)
{
	test_get_sensor_value(SENSOR_CHAN_GAUGE_DESIRED_VOLTAGE);
}

ZTEST(sbs_gauge_test, test_get_desired_chg_current)
{
	test_get_sensor_value(SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT);
}

ZTEST(sbs_gauge_test, test_not_supported_channel)
{
	uint8_t channel;

	for (channel = SENSOR_CHAN_ACCEL_X; channel <= SENSOR_CHAN_RPM; channel++) {
		test_get_sensor_value_not_supp(channel);
	}
	/* SOH is not defined in the SBS 1.1 specifications */
	test_get_sensor_value_not_supp(SENSOR_CHAN_GAUGE_STATE_OF_HEALTH);
}
