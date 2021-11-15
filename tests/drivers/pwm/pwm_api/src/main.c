/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <zephyr.h>
#include <ztest.h>
#include <device.h>

const struct device *get_pwm_device(void);

static void * pwn_basic_test_setup(void)
{
	const struct device *dev = get_pwm_device();

	zassert_true(device_is_ready(dev), "PWM device is not ready");
	k_object_access_grant(dev, k_current_get());

	return NULL;
}

ZTEST_SUITE(pwn_basic_test, NULL, pwn_basic_test_setup, NULL, NULL, NULL);
