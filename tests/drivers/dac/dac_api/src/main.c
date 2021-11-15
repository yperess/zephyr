/*
 * Copyright (c) 2020 Libre Solar Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <ztest.h>

extern const struct device *get_dac_device(void);

static void * dac_basic_test_setup(void)
{
	k_object_access_grant(get_dac_device(), k_current_get());

	return NULL;
}

ZTEST_SUITE(dac_basic_test, NULL, dac_basic_test_setup, NULL, NULL, NULL);
