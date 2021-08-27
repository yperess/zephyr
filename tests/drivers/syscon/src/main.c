/*
 * Copyright 2021 Google LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <drivers/syscon.h>
#include <string.h>
#include <ztest.h>

#define SYSCON_NODELABEL(n) DT_NODELABEL(syscon##n)
#define SYSCON_LABEL(n) DT_LABEL(SYSCON_NODELABEL(n))
#define SYSCON_ADDR(n) DT_REG_ADDR(SYSCON_NODELABEL(n))
#define SYSCON_SIZE(n) DT_REG_SIZE(SYSCON_NODELABEL(n))

static void test_syscon_get_size(void)
{
	const struct device *dev;
	size_t size;

	dev = device_get_binding(SYSCON_LABEL(4));
	zassert_ok(syscon_get_size(dev, &size), NULL);
	zassert_equal(size, SYSCON_SIZE(4), NULL);

	dev = device_get_binding(SYSCON_LABEL(2));
	zassert_ok(syscon_get_size(dev, &size), NULL);
	zassert_equal(size, SYSCON_SIZE(2), NULL);

	dev = device_get_binding(SYSCON_LABEL(1));
	zassert_ok(syscon_get_size(dev, &size), NULL);
	zassert_equal(size, SYSCON_SIZE(1), NULL);
}

static void test_syscon_get_base_address(void)
{
	const struct device *dev;
	uintptr_t addr;

	dev = device_get_binding(SYSCON_LABEL(4));
	zassert_ok(syscon_get_base(dev, &addr), NULL);
	zassert_equal(addr, SYSCON_ADDR(4), NULL);

	dev = device_get_binding(SYSCON_LABEL(2));
	zassert_ok(syscon_get_base(dev, &addr), NULL);
	zassert_equal(addr, SYSCON_ADDR(2), NULL);

	dev = device_get_binding(SYSCON_LABEL(1));
	zassert_ok(syscon_get_base(dev, &addr), NULL);
	zassert_equal(addr, SYSCON_ADDR(1), NULL);
}

static void test_syscon_out_of_bounds(void)
{
	const struct device *dev;
	uint32_t reg;

	dev = device_get_binding(SYSCON_LABEL(4));
	zassert_equal(syscon_read_reg(dev, SYSCON_SIZE(4), &reg), -EINVAL,
		      "Shouldn't be able to read byte %d", SYSCON_SIZE(4));

	dev = device_get_binding(SYSCON_LABEL(2));
	zassert_equal(syscon_read_reg(dev, SYSCON_SIZE(2), &reg), -EINVAL,
		      "Shouldn't be able to read byte %d", SYSCON_SIZE(2));

	dev = device_get_binding(SYSCON_LABEL(1));
	zassert_equal(syscon_read_reg(dev, SYSCON_SIZE(1), &reg), -EINVAL,
		      "Shouldn't be able to read byte %d", SYSCON_SIZE(1));
}

static void test_syscon_read_write_inst(const struct device *dev, int size)
{
	uint32_t reg;
	uint32_t expected = 0;

	for (int i = 0; i < size; ++i) {
		if (i % 4 == 0) {
			expected = i + 1;
		} else {
			expected = (expected << 8) | (i + i);
		}

		/* Clear the register at i, syscon should account for alignment. */
		zassert_ok(syscon_write_reg(dev, i, 0), NULL);
		zassert_ok(syscon_read_reg(dev, i, &reg), NULL);
		zassert_equal(0, reg, NULL);

		/* Test write and read, syscon should account for alignment. */
		zassert_ok(syscon_write_reg(dev, i, expected), NULL);
		zassert_ok(syscon_read_reg(dev, i, &reg), NULL);
		zassert_equal(expected, reg, NULL);
	}
}

static void test_syscon_read_write(void)
{
	test_syscon_read_write_inst(device_get_binding(SYSCON_LABEL(4)), SYSCON_SIZE(4));
	test_syscon_read_write_inst(device_get_binding(SYSCON_LABEL(2)), SYSCON_SIZE(2));
	test_syscon_read_write_inst(device_get_binding(SYSCON_LABEL(1)), SYSCON_SIZE(1));
}

void test_main(void)
{
	ztest_test_suite(syscon, ztest_unit_test(test_syscon_get_size),
			 ztest_unit_test(test_syscon_get_base_address),
			 ztest_unit_test(test_syscon_out_of_bounds),
			 ztest_unit_test(test_syscon_read_write));
	ztest_run_test_suite(syscon);
}
