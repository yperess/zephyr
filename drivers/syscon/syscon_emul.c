/*
 * Copyright (c) Carlo Caione <ccaione@baylibre.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT syscon

#include <device.h>
#include <drivers/syscon.h>
#include <init.h>

#include "syscon_common.h"

struct syscon_emul_config {
	uintptr_t base_address;
	size_t size;
	uint8_t reg_width;
};

struct syscon_emul_data {
	uint32_t *mem;
};

static int syscon_emul_get_base(const struct device *dev, uintptr_t *addr)
{
	const struct syscon_emul_config *config = dev->config;

	*addr = config->base_address;
	return 0;
}

static int syscon_emul_read_reg(const struct device *dev, uint16_t reg, uint32_t *val)
{
	const struct syscon_emul_config *config = dev->config;
	struct syscon_emul_data *data = dev->data;

	if (syscon_sanitize_reg(&reg, config->size, config->reg_width)) {
		return -EINVAL;
	}

	*val = data->mem[reg];

	return 0;
}

static int syscon_emul_write_reg(const struct device *dev, uint16_t reg, uint32_t val)
{
	const struct syscon_emul_config *config = dev->config;
	struct syscon_emul_data *data = dev->data;

	if (syscon_sanitize_reg(&reg, config->size, config->reg_width)) {
		return -EINVAL;
	}

	data->mem[reg] = val;

	return 0;
}

static int syscon_emul_get_size(const struct device *dev, size_t *size)
{
	const struct syscon_emul_config *config = dev->config;

	*size = config->size;
	return 0;
}

static const struct syscon_driver_api syscon_driver_api = {
	.read = syscon_emul_read_reg,
	.write = syscon_emul_write_reg,
	.get_base = syscon_emul_get_base,
	.get_size = syscon_emul_get_size,
};

static int syscon_emul_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

#define SYSCON_INIT(inst)                                                                          \
	static uint32_t syscon_emul_mem_##inst[DT_REG_SIZE(DT_DRV_INST(inst))];                    \
	static const struct syscon_emul_config syscon_emul_config_##inst = {                       \
		.base_address = DT_REG_ADDR(DT_DRV_INST(inst)),                                    \
		.size = DT_REG_SIZE(DT_DRV_INST(inst)),                                            \
		.reg_width = DT_PROP(DT_DRV_INST(inst), reg_io_width),                             \
	};                                                                                         \
	static struct syscon_emul_data syscon_emul_data_##inst = {                                 \
		.mem = syscon_emul_mem_##inst,                                                     \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, syscon_emul_init, NULL, &syscon_emul_data_##inst,              \
			      &syscon_emul_config_##inst, PRE_KERNEL_1,                            \
			      CONFIG_SYSCON_INIT_PRIORITY_DEVICE, &syscon_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SYSCON_INIT);
