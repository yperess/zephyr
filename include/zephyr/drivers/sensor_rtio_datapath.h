/*
 * Copyright (c) 2023 Google LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_SENSOR_RTIO_DATAPATH_H
#define ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_SENSOR_RTIO_DATAPATH_H

#include <errno.h>

#include <zephyr/drivers/sensor_mempool.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/rtio/rtio_mpsc.h>
#include <zephyr/rtio/rtio_executor_simple.h>
#include <zephyr/sys/mem_blocks.h>

struct sensor_mempool {
	struct sys_mem_blocks *buffer;
	struct rtio *rtio;
};

#define SENSOR_MEMPOOL_DEFINE(name, sq_sz, cq_sz, blk_size, num_blks, buf_align)                   \
	SYS_MEM_BLOCKS_DEFINE_STATIC(name##_buffer, blk_size, num_blks, buf_align);                \
	RTIO_EXECUTOR_SIMPLE_DEFINE(name##_exec);                                                  \
	RTIO_DEFINE(name##_rtio, (struct rtio_executor *)&name##_exec, sq_sz, cq_sz)               \
	static struct sensor_mempool name = {                                                      \
		.buffer = &name##_buffer,                                                          \
		.rtio = &name##_rtio,                                                              \
	};

typedef void (*sensor_rtio_data_handler)(struct sensor_mempool_entry *entry, int32_t result);

//static inline int sensor_rtio_consumer(struct sensor_mempool *mempool,
//				       sensor_rtio_data_handler handler)
//{
//	struct rtio_cqe *cqe = rtio_cqe_consume_block(mempool->rtio);
//	int32_t result = cqe->result;
//	void *next_buffer = NULL;
//	struct sensor_mempool_entry *entry = cqe->userdata;
//
//	__ASSERT_NO_MSG(handler != NULL);
//
//	rtio_spsc_release(mempool->rtio->cq);
//	if (sys_mem_blocks_alloc(mempool->buffer, 1, &next_buffer) == 0) {
//		/* Got a new block from the mempool */
//		sensor_set_data_buffer(entry->sensor, next_buffer);
//	}
//
//	(*handler)(entry, result);
//
//	/* Recycle the buffer by putting it back on the queue */
//	sys_mem_blocks_free(mempool->buffer, 1, (void **)&entry->buffer);
//
//	if (next_buffer != NULL) {
//		return 0;
//	}
//	if (sys_mem_blocks_alloc(mempool->buffer, 1, &next_buffer)) {
//		return -ENOMEM;
//	}
//	sensor_set_data_buffer(entry->sensor, next_buffer);
//	return 0;
//}

#endif /* ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_SENSOR_RTIO_DATAPATH_H */
