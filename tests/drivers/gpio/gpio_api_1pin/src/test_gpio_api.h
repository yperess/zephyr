/*
 * Copyright (c) 2019 Piotr Mienkowski
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TEST_GPIO_API_H_
#define TEST_GPIO_API_H_

#include <zephyr.h>
#include <drivers/gpio.h>
#include <ztest.h>

#if DT_NODE_HAS_PROP(DT_ALIAS(led0), gpios)
#define TEST_DEV             DT_GPIO_LABEL(DT_ALIAS(led0), gpios)
#define TEST_PIN             DT_GPIO_PIN(DT_ALIAS(led0), gpios)
#define TEST_PIN_DTS_FLAGS   DT_GPIO_FLAGS(DT_ALIAS(led0), gpios)
#else
#error Unsupported board
#endif

#define TEST_GPIO_MAX_RISE_FALL_TIME_US    200

#endif /* TEST_GPIO_API_H_ */
