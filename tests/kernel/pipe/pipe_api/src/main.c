/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @defgroup kernel_pipe_tests PIPEs
 * @ingroup all_tests
 * @{
 * @}
 */

#include <ztest.h>

/* k objects */
extern struct k_pipe pipe, kpipe, khalfpipe, put_get_pipe;
extern struct k_sem end_sema;
extern struct k_stack tstack;
extern struct k_thread tdata;
extern struct k_heap test_pool;

ZTEST_SUITE(pipe_api_1cpu, NULL, NULL, ztest_simple_1cpu_before, ztest_simple_1cpu_after, NULL);
ZTEST_SUITE(pipe_api, NULL, NULL, NULL, NULL, NULL);

/*test case main entry*/
void test_main(void)
{
	k_thread_access_grant(k_current_get(), &pipe,
			      &kpipe, &end_sema, &tdata, &tstack,
			      &khalfpipe, &put_get_pipe);

	k_thread_heap_assign(k_current_get(), &test_pool);

	ztest_run_test_suites(NULL);
	ztest_verify_all_test_suites_ran();
}
