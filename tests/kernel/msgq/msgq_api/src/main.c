/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @defgroup kernel_message_queue_tests Message Queue
 * @ingroup all_tests
 * @{
 * @}
 */

#include <ztest.h>

#ifdef CONFIG_64BIT
#define MAX_SZ	256
#else
#define MAX_SZ	128
#endif

K_HEAP_DEFINE(test_pool, MAX_SZ * 2);

extern struct k_msgq kmsgq;
extern struct k_msgq msgq;
extern struct k_sem end_sema;
extern struct k_thread tdata;
K_THREAD_STACK_EXTERN(tstack);

ZTEST_SUITE(msgq_api_1cpu, NULL, NULL, ztest_simple_1cpu_before, ztest_simple_1cpu_after, NULL);
ZTEST_SUITE(msgq_api, NULL, NULL, NULL, NULL, NULL);

/*test case main entry*/
void test_main(void)
{
	k_thread_access_grant(k_current_get(), &kmsgq, &msgq, &end_sema,
			      &tdata, &tstack);

	k_thread_heap_assign(k_current_get(), &test_pool);

	ztest_run_test_suites(NULL);
	ztest_verify_all_test_suites_ran();
}
