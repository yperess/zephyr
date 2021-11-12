/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ztest.h>
#include <app_memory/partitions.h>

static void * setup_mbedtls_partition(void)
{
#ifdef CONFIG_USERSPACE
	k_mem_domain_add_partition(&k_mem_domain_default, &k_mbedtls_partition);
#endif
	return NULL;
}

ZTEST_SUITE(test_mbedtls_fn, setup_mbedtls_partition, NULL, NULL, NULL, NULL);
