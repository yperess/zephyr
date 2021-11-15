/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <stdbool.h>
#include <zephyr.h>
#include <ztest.h>

#ifdef CONFIG_SHELL
TC_CMD_DEFINE(test_dma_m2m_chan0_burst8)
TC_CMD_DEFINE(test_dma_m2m_chan1_burst8)
TC_CMD_DEFINE(test_dma_m2m_chan0_burst16)
TC_CMD_DEFINE(test_dma_m2m_chan1_burst16)

SHELL_CMD_REGISTER(test_dma_m2m_chan0_burst8, NULL, NULL,
			TC_CMD_ITEM(test_dma_m2m_chan0_burst8));
SHELL_CMD_REGISTER(test_dma_m2m_chan1_burst8, NULL, NULL,
			TC_CMD_ITEM(test_dma_m2m_chan1_burst8));
SHELL_CMD_REGISTER(test_dma_m2m_chan0_burst16, NULL, NULL,
			TC_CMD_ITEM(test_dma_m2m_chan0_burst16));
SHELL_CMD_REGISTER(test_dma_m2m_chan1_burst16, NULL, NULL,
			TC_CMD_ITEM(test_dma_m2m_chan1_burst16));
#endif

bool should_dma_m2m_test(const void *state)
{
#ifndef CONFIG_SHELL
	return true;
#else
	return false;
#endif
}
ZTEST_SUITE(dma_m2m_test, should_dma_m2m_test, NULL, NULL, NULL, NULL);

