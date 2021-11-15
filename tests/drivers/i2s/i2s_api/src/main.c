/*
 * Copyright (c) 2017 comsuisse AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <zephyr.h>
#include <ztest.h>
#include <device.h>
#include "i2s_api_test.h"

static void * i2s_tests_setup(void)
{
	const struct device *dev_i2s_rx;
	const struct device *dev_i2s_tx;

	k_thread_access_grant(k_current_get(),
			      &rx_mem_slab, &tx_mem_slab);

	dev_i2s_rx = device_get_binding(I2S_DEV_NAME_RX);
	if (dev_i2s_rx != NULL) {
		k_object_access_grant(dev_i2s_rx, k_current_get());
	}

	dev_i2s_tx = device_get_binding(I2S_DEV_NAME_TX);
	if (dev_i2s_tx != NULL) {
		k_object_access_grant(dev_i2s_tx, k_current_get());
	}

	return NULL;
}

ZTEST(i2s_tests, i2s_tests)
{
	test_i2s_tx_transfer_configure_0();
	test_i2s_rx_transfer_configure_0();
	test_i2s_transfer_short();
	test_i2s_transfer_long();
	test_i2s_rx_sync_start();
	test_i2s_rx_empty_timeout();
	test_i2s_transfer_restart();
	test_i2s_transfer_tx_underrun();
	test_i2s_transfer_rx_overrun();

	test_i2s_tx_transfer_configure_1();
	test_i2s_rx_transfer_configure_1();
	test_i2s_state_not_ready_neg();
	test_i2s_state_ready_neg();
	test_i2s_state_running_neg();
	test_i2s_state_stopping_neg();
	test_i2s_state_error_neg();

	test_i2s_dir_both_transfer_configure_0();
	test_i2s_dir_both_transfer_short();
	test_i2s_dir_both_transfer_long();
	test_i2s_dir_both_transfer_restart();
	test_i2s_dir_both_transfer_rx_overrun();
	test_i2s_dir_both_transfer_tx_underrun();

	test_i2s_dir_both_transfer_configure_1();
	test_i2s_dir_both_state_running_neg();
	test_i2s_dir_both_state_stopping_neg();
	test_i2s_dir_both_state_error_neg();
}

/* Neccessary to run all tests in user mode */
ZTEST_USER(i2s_tests, i2s_user_tests)
{
	test_i2s_tx_transfer_configure_0();
	test_i2s_rx_transfer_configure_0();
	test_i2s_transfer_short();
	test_i2s_transfer_long();
	test_i2s_rx_sync_start();
	test_i2s_rx_empty_timeout();
	test_i2s_transfer_restart();
	test_i2s_transfer_tx_underrun();
	test_i2s_transfer_rx_overrun();

	test_i2s_tx_transfer_configure_1();
	test_i2s_rx_transfer_configure_1();
	test_i2s_state_not_ready_neg();
	test_i2s_state_ready_neg();
	test_i2s_state_running_neg();
	test_i2s_state_stopping_neg();
	test_i2s_state_error_neg();

	test_i2s_dir_both_transfer_configure_0();
	test_i2s_dir_both_transfer_short();
	test_i2s_dir_both_transfer_long();
	test_i2s_dir_both_transfer_restart();
	test_i2s_dir_both_transfer_rx_overrun();
	test_i2s_dir_both_transfer_tx_underrun();

	test_i2s_dir_both_transfer_configure_1();
	test_i2s_dir_both_state_running_neg();
	test_i2s_dir_both_state_stopping_neg();
	test_i2s_dir_both_state_error_neg();
}

ZTEST_SUITE(i2s_tests, NULL, i2s_tests_setup, NULL, NULL, NULL);
