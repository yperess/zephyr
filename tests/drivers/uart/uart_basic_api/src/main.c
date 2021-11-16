/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @addtogroup t_driver_uart
 * @{
 * @defgroup t_uart_basic test_uart_basic_operations
 * @}
 */

#include <usb/usb_device.h>
#include "test_uart.h"

#ifdef CONFIG_SHELL
TC_CMD_DEFINE(test_uart_configure)
TC_CMD_DEFINE(test_uart_config_get)
TC_CMD_DEFINE(test_uart_fifo_read)
TC_CMD_DEFINE(test_uart_fifo_fill)
TC_CMD_DEFINE(test_uart_poll_in)
TC_CMD_DEFINE(test_uart_poll_out)
TC_CMD_DEFINE(test_uart_pending)

SHELL_CMD_REGISTER(test_uart_configure, NULL, NULL,
			TC_CMD_ITEM(test_uart_configure));
SHELL_CMD_REGISTER(test_uart_config_get, NULL, NULL,
			TC_CMD_ITEM(test_uart_config_get));
SHELL_CMD_REGISTER(test_uart_fifo_read, NULL, NULL,
			TC_CMD_ITEM(test_uart_fifo_read));
SHELL_CMD_REGISTER(test_uart_fifo_fill, NULL, NULL,
			TC_CMD_ITEM(test_uart_fifo_fill));
SHELL_CMD_REGISTER(test_uart_poll_in, NULL, NULL,
			TC_CMD_ITEM(test_uart_poll_in));
SHELL_CMD_REGISTER(test_uart_poll_out, NULL, NULL,
			TC_CMD_ITEM(test_uart_poll_out));
SHELL_CMD_REGISTER(test_uart_pending, NULL, NULL,
			TC_CMD_ITEM(test_uart_pending));
#endif

#ifndef CONFIG_UART_INTERRUPT_DRIVEN
void test_uart_fifo_fill(void)
{
	ztest_test_skip();
}

void test_uart_fifo_read(void)
{
	ztest_test_skip();
}

void test_uart_pending(void)
{
	ztest_test_skip();
}
#endif

static void * uart_basic_test_setup(void)
{
#if defined(CONFIG_USB_UART_CONSOLE)
	const struct device *dev;
	uint32_t dtr = 0;

	dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	if (!device_is_ready(dev) || usb_enable(NULL)) {
		return;
	}

	while (!dtr) {
		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
		k_sleep(K_MSEC(100));
	}
#endif /* CONFIG_USB_UART_CONSOLE */
	return NULL;
}

ZTEST(uart_basic_test, uart_basic_test)
{
	/* Tests all in one ztest due to test order dependencies */
#ifndef CONFIG_SHELL
	test_uart_configure();
	test_uart_config_get();
	test_uart_fifo_fill();
	test_uart_fifo_read();
	test_uart_poll_in();
	test_uart_poll_out();
	test_uart_pending);
#endi/* CONFIG_SHELL */f
}

ZTEST_SUITE(uart_basic_test, NULL, uart_basic_test_setup, NULL, NULL, NULL);
