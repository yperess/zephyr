/*
 * Copyright (c) 2023 Google LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_CHX01_CH_COMMON_H
#define ZEPHYR_DRIVERS_SENSOR_CHX01_CH_COMMON_H

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor/ch_common.h>

enum ch_part_number {
	CH101_PART_NUMBER,
};

struct ch_common_data {
	const struct i2c_dt_spec i2c;
	const struct gpio_dt_spec *gpio_int;
	uint16_t scale_factor;
	uint16_t rtc_cal_result;
	uint32_t op_frequency;
	uint16_t interval_ms;
	uint16_t sample_interval;
	enum ch_mode mode;
};

/** Speed of sound in m/s */
#define CH_SPEEDOFSOUND_MPS (343)

/* CH-101 common definitions */

#define CH101_COMMON_REG_OPMODE	       0x01
#define CH101_COMMON_REG_TICK_INTERVAL 0x02
#define CH101_COMMON_REG_PERIOD	       0x05
#define CH101_COMMON_REG_CAL_TRIG      0x06
#define CH101_COMMON_REG_MAX_RANGE     0x07
#define CH101_COMMON_REG_TIME_PLAN     0x09
#define CH101_COMMON_REG_CAL_RESULT    0x0A
#define CH101_COMMON_REG_REV_CYCLES    0x0C
#define CH101_COMMON_REG_DCO_PERIOD    0x0E
#define CH101_COMMON_REG_RX_HOLDOFF    0x11
#define CH101_COMMON_REG_STAT_RANGE    0x12
#define CH101_COMMON_REG_STAT_COEFF    0x13
#define CH101_COMMON_REG_READY	       0x14
#define CH101_COMMON_REG_TOF_SF	       0x16
#define CH101_COMMON_REG_TOF	       0x18
#define CH101_COMMON_REG_AMPLITUDE     0x1A
#define CH101_COMMON_REG_DATA	       0x1C

int ch_common_set_mode(const struct device *dev, enum ch_part_number part_number,
		       enum ch_mode mode);

int ch_common_set_sample_interval(const struct device *dev, enum ch_part_number part_number,
				  uint16_t interval_ms);

int ch_common_get_range(const struct device *dev, enum ch_part_number part_number, uint32_t *range);

void ch_common_prepare_pulse_timer(const struct device *dev, enum ch_part_number part_number);

struct ch_common_data *ch101_get_common_data(const struct device *dev);

void ch_common_store_pt_result(const struct device *dev, enum ch_part_number part_number);

void ch_common_store_op_freq(const struct device *dev, enum ch_part_number part_number);

int ch_common_store_scale_factor(const struct device *dev, enum ch_part_number part_number);

#endif /* ZEPHYR_DRIVERS_SENSOR_CHX01_CH_COMMON_H */
