/*
 * Copyright 2020 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @defgroup driver_sensor_subsys_tests sensor_subsys
 * @ingroup all_tests
 * @{
 * @}
 */

#include <ztest.h>
#include <drivers/sensor.h>

/* There is no obvious way to pass this to tests, so use a global */
ZTEST_BMEM static const char *accel_label;

static enum sensor_channel channel[] = {
	SENSOR_CHAN_ACCEL_X,
	SENSOR_CHAN_ACCEL_Y,
	SENSOR_CHAN_ACCEL_Z,
	SENSOR_CHAN_GYRO_X,
	SENSOR_CHAN_GYRO_Y,
	SENSOR_CHAN_GYRO_Z,
};

ZTEST(accel_tests, test_sensor_accel_basic)
{
	const struct device *dev;

	dev = device_get_binding(accel_label);
	zassert_not_null(dev, "failed: dev '%s' is null", accel_label);

	zassert_equal(sensor_sample_fetch(dev), 0, "fail to fetch sample");

	for (int i = 0; i < ARRAY_SIZE(channel); i++) {
		struct sensor_value val;

		zassert_ok(sensor_channel_get(dev, channel[i], &val),
			   "fail to get channel");
		zassert_equal(i, val.val1, "expected %d, got %d", i, val.val1);
		zassert_true(val.val2 < 1000, "error %d is too large",
			     val.val2);
	}
}

/* Configure tests on an accelerometer device with the given label */
static void configure_tests_for_accel(const char *label)
{
	const struct device *accel = device_get_binding(label);

	PRINT("Running tests on '%s'\n", label);
	zassert_not_null(accel, "Unable to get Accelerometer device");
	k_object_access_grant(accel, k_current_get());
	accel_label = label;
}

ZTEST_USER(accel0_tests, test_sensor_accel)
{
	test_sensor_accel_basic();
}

static bool only_true_predicate(void * state)
{
	return true;
}

ZTEST_SUITE(accel_tests, only_true_predicate, NULL, NULL, NULL, NULL);

/* Neccessary to rerun tests with different config */
void accel_tests_main(void)
{
	configure_tests_for_accel(DT_LABEL(DT_ALIAS(accel_0)));
	ztest_run_test_suite(accel_tests);

#if DT_NODE_EXISTS(DT_ALIAS(accel_1))
	configure_tests_for_accel(DT_LABEL(DT_ALIAS(accel_1)));
	ztest_run_test_suite(accel_tests);
#endif /* DT_NODE_EXISTS(DT_ALIAS(accel_1)) */
}
