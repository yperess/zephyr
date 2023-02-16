/*
 * Copyright (c) 2023 Google LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "ch_common.h"
#include "ch_driver.h"

LOG_MODULE_DECLARE(CH_COMMON);

int chdrv_write_byte(const struct device *i2c, uint16_t i2c_addr, uint16_t mem_addr,
		     uint8_t data_value)
{
	/* Insert byte count (1) at start of data */
	uint8_t message[] = {sizeof(data_value), data_value};

	/* Write the data */
	return i2c_write(i2c, message, sizeof(message), i2c_addr);
}

int chdrv_write_word(const struct device *i2c, uint16_t i2c_addr, uint16_t mem_addr,
		     uint16_t data_value)
{
	/*
	 * Place byte count (2) in first byte of message
	 * Sensor is little-endian, so LSB goes in at the lower address
	 */
	uint8_t message[] = {sizeof(data_value), data_value & 0xff, (data_value >> 8) & 0xff};

	return i2c_write(i2c, message, sizeof(message), i2c_addr);
}

int chdrv_read_word(const struct device *i2c, uint16_t i2c_addr, uint16_t mem_addr,
		    uint16_t *data_value)
{
	const uint8_t write_buff[] = { mem_addr & 0xff, (mem_addr >> 8) & 0xff};
	int rc = i2c_write_read(i2c, i2c_addr, write_buff, sizeof(write_buff), data_value, sizeof(data_value));

	if (rc != 0) {
		return rc;
	}

	*data_value = sys_le16_to_cpu(*data_value);
	return 0;
}

int chdrv_reset_and_halt(const struct device *i2c)
{
	int rc;

	rc = chdrv_prog_write(i2c, CH_PROG_REG_CPU, 0x40);
	rc |= chdrv_prog_write(i2c, CH_PROG_REG_CPU, 0x11);

	return rc == 0 ? 0 : -EIO;
}

int chdrv_set_idle(const struct device *i2c)
{
	const uint8_t idle_loop[] = {0x40, 0x03, 0xff, 0xfc};
	const uint8_t val[] = {0x5a, 0x80};
	int rc = chdrv_prog_mem_write(i2c, 0xfffc, idle_loop, sizeof(idle_loop));

	if (rc != 0) {
		LOG_ERR("Failed to write idle loop program");
		return rc;
	}

	rc = chdrv_reset_and_halt(i2c);
	if (rc != 0) {
		LOG_ERR("Failed to reset");
		return rc;
	}

	rc = chdrv_prog_mem_write(i2c, 0x120, val, sizeof(val));
	if (rc != 0) {
		LOG_ERR("Failed to keep wdt stopped");
		return rc;
	}

	return 0;
}

int chdrv_fw_load(const struct device *i2c, uint16_t prog_mem_addr, const uint8_t *fw,
		  uint16_t fw_size)
{
	const uint16_t max_xfer_size =
#if CONFIG_CHX01_MAX_PROG_XFER_SIZE > 0
		MIN(CONFIG_CHX01_MAX_PROG_XFER_SIZE, fw_size)
#else
		fw_size
#endif
		;
	uint16_t bytes_left = fw_size;
	int rc;

	while (bytes_left > 0) {
		uint16_t xfer_nbytes = MIN(bytes_left, max_xfer_size);

		LOG_DBG("Writing %u/%u bytes", xfer_nbytes, fw_size);
		rc = chdrv_prog_mem_write(i2c, prog_mem_addr, fw, xfer_nbytes);
		if (rc != 0) {
			LOG_ERR("Failed to write firmware (%d)", rc);
			return rc;
		}

		fw += xfer_nbytes;
		prog_mem_addr += xfer_nbytes;
		bytes_left -= xfer_nbytes;
	}

	return 0;
}

int chdrv_prog_mem_write(const struct device *i2c, uint16_t addr, const uint8_t *message,
			 uint16_t nbytes)
{
	int rc = 0;

	__ASSERT_NO_MSG(nbytes > 0);

	rc = chdrv_prog_write(i2c, CH_PROG_REG_ADDR, addr);
	if (rc != 0) {
		LOG_ERR("Failed to set CH_PROG_REG_ADDR=0x%04x", addr);
		return rc;
	}
	if (nbytes == 1 || (nbytes == 2 && (addr & 0x1) == 0)) {
		uint16_t data = message[0];

		if (nbytes == 2) {
			data |= (message[1] << 8);
		}

		rc = chdrv_prog_write(i2c, CH_PROG_REG_DATA, data);
		if (rc != 0) {
			LOG_ERR("Failed to set CH_PROG_REG_DATA=0x%04x", data);
			return rc;
		}

		rc = chdrv_prog_write(i2c, CH_PROG_REG_CTL, (0x03 | ((nbytes == 1) ? 0x08 : 0x00)));
		if (rc != 0) {
			LOG_ERR("Failed to set CH_PROG_REG_CTL=0x%04x",
				(0x03 | ((nbytes == 1) ? 0x08 : 0x00)));
		}
	} else {
		const uint8_t burst_hdr[2] = {0xC4, 0x0B};

		rc = chdrv_prog_write(i2c, CH_PROG_REG_CNT, (nbytes - 1));
		if (rc != 0) {
			LOG_ERR("Failed to set CH_PROG_REG_CTL=0x%04x", nbytes - 1);
			return rc;
		}

		rc = i2c_write(i2c, burst_hdr, (uint32_t)sizeof(burst_hdr), CH_I2C_ADDR_PROG);
		if (rc != 0) {
			LOG_ERR("Failed to set header");
			return rc;
		}
		rc = i2c_write(i2c, message, nbytes, CH_I2C_ADDR_PROG);
		if (rc != 0) {
			LOG_ERR("Failed to set data");
		}
	}
	return rc;
}

int chdrv_prog_write(const struct device *i2c, uint8_t reg_addr, uint16_t data)
{
	/* Set register address write bit */
	reg_addr |= 0x80;

	/* Write the register address, followed by the value to be written */
	uint8_t message[] = {reg_addr, data & 0xff, (data >> 8) & 0xff};

	/* For the 2-byte registers, we also need to write MSB after the LSB */
	return i2c_write(i2c, message, 1 + CH_PROG_SIZEOF(reg_addr), CH_I2C_ADDR_PROG);
}

int chdrv_burst_write(const struct device *i2c, uint16_t mem_addr, const uint8_t *data, uint8_t len)
{
	uint8_t message[CH_I2C_MAX_WRITE_BYTES + 1];

	message[0] = mem_addr & 0xff;
	message[1] = len;
	memcpy(message + 2, data, len);

	return i2c_write(i2c, message, len + 2, CH_I2C_ADDR_PROG);
}

int chdrv_init_ram(const struct device *i2c, const struct ch_firmware *fw)
{
	__ASSERT_NO_MSG(fw != NULL);

	if (fw->init_size == 0) {
		return 0;
	}

	return chdrv_prog_mem_write(i2c, fw->init_addr, fw->init_fw, fw->init_size);
}

static int chdrv_prog_read(const struct device *i2c, uint8_t reg_addr, uint16_t *data)
{
	uint8_t nbytes = CH_PROG_SIZEOF(reg_addr);
	uint8_t message[1] = {0x7f & reg_addr};
	uint8_t read_data[2];
	int rc;

	rc = i2c_write(i2c, message, 1, CH_I2C_ADDR_PROG);
	if (rc != 0) {
		return rc;
	}

	rc = i2c_read(i2c, read_data, nbytes, CH_I2C_ADDR_PROG);
	if (rc != 0) {
		return rc;
	}

	*data = read_data[0];
	if (nbytes > 1) {
		*data |= (((uint16_t)read_data[1]) << 8);
	}

	return 0;
}

int chdrv_prog_ping(const struct device *i2c)
{
	uint16_t tmp = 0;
	int rc;

	printk("1\n");
	rc = chdrv_reset_and_halt(i2c);
	printk("reset_and_halt (%d)\n", rc);
	rc |= chdrv_prog_read(i2c, CH_PROG_REG_PING, &tmp);
	printk("read_ping (%d)\n", rc);

	if (rc == 0) {
		LOG_DBG("Test I2C read: 0x%04x", tmp);
	}

	return rc == 0;
}

void chdrv_measure_rtc(const struct device *dev, enum ch_part_number part_number)
{
	const uint32_t pulselength = 100; // TODO Make this a Kconfig (units are ms)
	struct ch_common_data *data = NULL;

	switch (part_number) {
	case CH101_PART_NUMBER:
		data = ch101_get_common_data(dev);
		break;
	default:
		return;
	}

	LOG_INF("Measuring RTC");
	/* Set  interrupt line low */
	gpio_pin_set_dt(data->gpio_int, 0);

	/* Set up RTC calibration */
	ch_common_prepare_pulse_timer(dev, part_number);

	/* Trigger the pulse on the IO pin */
	gpio_pin_set_dt(data->gpio_int, 1);
	k_busy_wait(pulselength * 1000);
	gpio_pin_set_dt(data->gpio_int, 0);

	/* Kep the IO low for at least 50us to allow the ASIC FW to deactivate the PT logic. */
	k_busy_wait(100);

	LOG_INF("Gathering RTC results");
	ch_common_store_pt_result(dev, part_number);
	ch_common_store_op_freq(dev, part_number);
	ch_common_store_scale_factor(dev, part_number);

	LOG_DBG("scale_factor=%u", data->scale_factor);
	LOG_DBG("rtc_cal_result=%u", data->rtc_cal_result);
	LOG_DBG("op_frequency=%u", data->op_frequency);
}
