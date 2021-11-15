/*
 * Copyright (c) 2020 Seagate Technology LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <ztest.h>

#include "test_led_api.h"

static void * led_user_test_setup(void)
{
	k_object_access_grant(get_led_controller(), k_current_get());

	return NULL;
}

ZTEST_SUITE(led_user_test, NULL, led_user_test_setup, NULL, NULL, NULL);
