/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @addtogroup t_driver_uart
 * @{
 * @defgroup t_uart_async test_uart_async
 * @}
 */

#include "test_uart.h"

static void * uart_async_test_setup(void)
{
	init_test();

#ifdef CONFIG_USERSPACE
	set_permissions();
#endif
	return NULL;
}

ZTEST_SUITE(test_single_read_suite, NULL, uart_async_test_setup,
	    test_single_read_setup, NULL, NULL);
ZTEST_SUITE(test_chained_read_suite, NULL, uart_async_test_setup,
	    test_chained_read_setup, NULL, NULL);
ZTEST_SUITE(test_double_buffer_suite, NULL, uart_async_test_setup,
	    test_double_buffer_setup, NULL, NULL);
ZTEST_SUITE(test_read_abort_suite, NULL, uart_async_test_setup,
	    test_read_abort_setup, NULL, NULL);
ZTEST_SUITE(test_write_abort_suite, NULL, uart_async_test_setup,
	    test_write_abort_setup, NULL, NULL);
ZTEST_SUITE(test_forever_timeout_suite, NULL, uart_async_test_setup,
	    test_forever_timeout_setup, NULL, NULL);
ZTEST_SUITE(test_chained_write_suite, NULL, uart_async_test_setup,
	    test_chained_write_setup, NULL, NULL);
ZTEST_SUITE(test_long_buffers_suite, NULL, uart_async_test_setup,
	    test_long_buffers_setup, NULL, NULL);
