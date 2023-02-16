/*
 * Copyright (c) 2023 Google LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "ch_common.h"
#include "ch_driver.h"

LOG_MODULE_REGISTER(CH_COMMON, CONFIG_SENSOR_LOG_LEVEL);

#define MAX_PERIOD_VALUE 16

int ch_common_set_mode(const struct device *dev, enum ch_part_number part_number, enum ch_mode mode)
{
	int rc = 0;
	uint8_t opmode_reg;
	uint8_t period_reg;
	uint8_t tick_interval_reg;
	uint16_t max_tick_interval;
	struct ch_common_data *data = NULL;

	switch (part_number) {
	case CH101_PART_NUMBER:
		opmode_reg = CH101_COMMON_REG_OPMODE;
		period_reg = CH101_COMMON_REG_PERIOD;
		tick_interval_reg = CH101_COMMON_REG_TICK_INTERVAL;
		max_tick_interval = CH101_MAX_TICK_INTERVAL;
		data = ch101_get_common_data(dev);
		break;
	default:
		return -EINVAL;
	}

	switch (mode) {
	case CH_MODE_IDLE:
		rc = chdrv_write_byte(data->i2c.bus, data->i2c.addr, period_reg, 0);
		rc |= chdrv_write_word(data->i2c.bus, data->i2c.addr, tick_interval_reg,
				       max_tick_interval);
		rc |= chdrv_write_byte(data->i2c.bus, data->i2c.addr, opmode_reg, CH_MODE_IDLE);
		break;
	case CH_MODE_FREERUN:
		LOG_DBG("Setting sample interval %u ms", data->interval_ms);
		rc = ch_common_set_sample_interval(dev, part_number, data->interval_ms);
		rc |= chdrv_write_byte(data->i2c.bus, data->i2c.addr, opmode_reg, CH_MODE_FREERUN);
		break;
	case CH_MODE_TRIGGERED_TX_RX:
		rc = chdrv_write_byte(data->i2c.bus, data->i2c.addr, opmode_reg,
				      CH_MODE_TRIGGERED_TX_RX);
		break;
	case CH_MODE_TRIGGERED_RX_ONLY:
		rc = chdrv_write_byte(data->i2c.bus, data->i2c.addr, opmode_reg,
				      CH_MODE_TRIGGERED_RX_ONLY);
		break;
	default:
		return -EINVAL;
	}

	return rc == 0 ? 0 : -EIO;
}

int ch_common_set_sample_interval(const struct device *dev, enum ch_part_number part_number,
				  uint16_t interval_ms)
{
	int rc = 0;
	uint8_t period_reg;
	uint8_t tick_interval_reg;
	uint16_t max_tick_interval;
	uint32_t period;
	uint32_t tick_interval;
	struct ch_common_data *data = NULL;

	switch (part_number) {
	case CH101_PART_NUMBER:
		period_reg = CH101_COMMON_REG_PERIOD;
		tick_interval_reg = CH101_COMMON_REG_TICK_INTERVAL;
		max_tick_interval = CH101_MAX_TICK_INTERVAL;
		data = ch101_get_common_data(dev);
		break;
	default:
		return -EINVAL;
	}

	// TODO replace 100 with the pulse duration (ms)
	uint32_t sample_interval = data->rtc_cal_result * interval_ms / 100;

	if (interval_ms == 0) {
		return -EINVAL;
	}

	period = (sample_interval / 2048) + 1;
	if (period > UINT8_MAX) {
		return -EINVAL;
	}

	if (period != 0) {
		tick_interval = sample_interval / period;

		while (tick_interval > max_tick_interval && period < MAX_PERIOD_VALUE) {
			tick_interval >>= 1;
			period <<= 1;
		}
	} else {
		tick_interval = max_tick_interval;
	}

	LOG_DBG("Set period=%" PRIu32 ", tick_interval=%" PRIu32, period, tick_interval);

	rc = chdrv_write_byte(data->i2c.bus, data->i2c.addr, period_reg, (uint8_t)period);
	rc |= chdrv_write_word(data->i2c.bus, data->i2c.addr, tick_interval_reg,
			       (uint16_t)tick_interval);

	data->sample_interval = sample_interval;
	return rc == 0 ? 0 : -EIO;
}

int ch_common_store_scale_factor(const struct device *dev, enum ch_part_number part_number)
{
	int rc;
	uint8_t tof_sf_reg;
	uint16_t scale_factor;
	struct ch_common_data *data = NULL;

	switch (part_number) {
	case CH101_PART_NUMBER:
		tof_sf_reg = CH101_COMMON_REG_TOF_SF;
		data = ch101_get_common_data(dev);
		break;
	default:
		return -EINVAL;
	}

	rc = chdrv_read_word(data->i2c.bus, data->i2c.addr, tof_sf_reg, &scale_factor);
	if (rc != 0) {
		LOG_ERR("Failed to read ToF scale_factor (%d)", rc);
		data->scale_factor = 0;
		return rc;
	}

	data->scale_factor = scale_factor;
	return 0;
}

int ch_common_get_range(const struct device *dev, enum ch_part_number part_number, uint32_t *range)
{
	uint8_t tof_reg;
	uint16_t time_of_flight;
	uint16_t scale_factor;
	struct ch_common_data *data = NULL;
	int rc;

	switch (part_number) {
	case CH101_PART_NUMBER:
		tof_reg = CH101_COMMON_REG_TOF;
		data = ch101_get_common_data(dev);
		break;
	default:
		return -EINVAL;
	}

	rc = chdrv_read_word(data->i2c.bus, data->i2c.addr, tof_reg, &time_of_flight);
	if (rc != 0) {
		LOG_ERR("Failed to read ToF register 0x%02x", tof_reg);
		return rc;
	}

	if (time_of_flight == UINT16_MAX) {
		LOG_ERR("No object detected");
		return -EBUSY;
	}

	if (data->scale_factor == 0) {
		rc = ch_common_store_scale_factor(dev, part_number);
		if (rc != 0) {
			LOG_ERR("Failed to store scale factor");
			return rc;
		}
	}
	scale_factor = data->scale_factor;

	LOG_DBG("scale_factor=%u", scale_factor);
	if (scale_factor == 0) {
		*range = UINT32_MAX;
		return 0;
	}

	// TODO replace 100 with the calibration pulse ms
	uint32_t num = (CH_SPEEDOFSOUND_MPS * 100 * (uint32_t)time_of_flight);
	uint32_t den = ((uint32_t)data->rtc_cal_result * (uint32_t)scale_factor) >> 11;

	*range = (num / den);
	LOG_DBG("time_of_flight=%u", time_of_flight);
	LOG_DBG("%u / %u = %u", num, den, *range);

	/* TODO CH201 needs to double the range */

	/* TODO allow firmware to specify oversampling */
	/* range >>= data->oversample; */

	/* TODO account for pre-trigger time in RX_ONLY mode */
	return 0;
}

void ch_common_prepare_pulse_timer(const struct device *dev, enum ch_part_number part_number)
{
	uint8_t cal_trig_reg;
	struct ch_common_data *data = NULL;
	int rc;

	switch (part_number) {
	case CH101_PART_NUMBER:
		cal_trig_reg = CH101_COMMON_REG_CAL_TRIG;
		data = ch101_get_common_data(dev);
		break;
	default:
		return;
	}

	rc = chdrv_write_byte(data->i2c.bus, data->i2c.addr, cal_trig_reg, 0);
	if (rc) {
		LOG_ERR("Failed to prepare pulse timer (%d)", rc);
	}
}

void ch_common_store_pt_result(const struct device *dev, enum ch_part_number part_number)
{
	struct ch_common_data *data = NULL;
	uint8_t pt_result_reg;
	uint16_t rtc_cal_result;
	int rc;

	switch (part_number) {
	case CH101_PART_NUMBER:
		pt_result_reg = CH101_COMMON_REG_CAL_RESULT;
		data = ch101_get_common_data(dev);
		break;
	default:
		return;
	}

	rc = chdrv_read_word(data->i2c.bus, data->i2c.addr, pt_result_reg, &rtc_cal_result);
	if (rc) {
		LOG_ERR("Failed to get calibration result (%d)", rc);
		return;
	}
	data->rtc_cal_result = rtc_cal_result;
}

void ch_common_store_op_freq(const struct device *dev, enum ch_part_number part_number)
{
	struct ch_common_data *data = NULL;
	uint8_t tof_sf_reg;
	uint16_t raw_freq; // aka scale factor
	uint32_t freq_counter_cycles;
	uint32_t num;
	uint32_t den;
	uint32_t op_freq;
	int rc;

	switch (part_number) {
	case CH101_PART_NUMBER:
		tof_sf_reg = CH101_COMMON_REG_TOF_SF;
		data = ch101_get_common_data(dev);
		break;
	default:
		return;
	}

	freq_counter_cycles = 128; // dev_ptr->freqCounterCycles;

	rc = chdrv_read_word(data->i2c.bus, data->i2c.addr, tof_sf_reg, &raw_freq);
	if (rc) {
		LOG_ERR("Failed to read ToF scale_factor for op_frequency (%d)", rc);
		return;
	}

	num = (uint32_t)(((data->rtc_cal_result) * 1000U) / (16U * freq_counter_cycles)) *
	      (uint32_t)(raw_freq);
	den = (uint32_t)(100); // TODO must match the pulselength (100) in ch_driver.c
	op_freq = (num / den);

	data->op_frequency = op_freq;
}
