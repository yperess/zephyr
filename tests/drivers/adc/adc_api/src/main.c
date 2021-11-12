/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <zephyr.h>
#include <ztest.h>

extern const struct device *get_adc_device(void);
extern struct k_poll_signal async_sig;

static void * adc_basic_test_setup(void)
{
	k_object_access_grant(get_adc_device(), k_current_get());
#ifdef CONFIG_ADC_ASYNC
	k_object_access_grant(&async_sig, k_current_get());
	k_poll_signal_init(&async_sig);
	k_thread_system_pool_assign(k_current_get());
#endif
}

ZTEST_SUITE(adc_basic_test, adc_basic_test_setup, NULL, NULL, NULL, NULL);
