/*
 * Copyright (c) 2023 Google LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT invensense_ch101

#include <stdint.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/ch101.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "ch_common.h"
#include "ch_driver.h"

LOG_MODULE_REGISTER(CH101, CONFIG_SENSOR_LOG_LEVEL);

enum default_firmware {
	DEFAULT_FIRMWARE_NONE,
	DEFAULT_FIRMWARE_GPR,
	DEFAULT_FIRMWARE_GPR_SR,
};

struct ch101_data {
	const struct ch_firmware *firmware;
	uint32_t range;
	struct gpio_callback gpio_cb;
	struct ch_common_data common_data;
};

struct ch101_config {
	struct gpio_dt_spec gpio_reset;
	struct gpio_dt_spec gpio_interrupt;
	struct gpio_dt_spec gpio_program;
	enum default_firmware default_firmware;
};

struct ch_common_data *ch101_get_common_data(const struct device *dev)
{
	struct ch101_data *data = dev->data;

	return &data->common_data;
}

static void ch101_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	LOG_DBG("\n\n**** DATA IS READY! ****\n\n");
}

static void ch101_init_trigger(const struct device *dev)
{
	const struct ch101_config *cfg = dev->config;
	struct ch101_data *data = dev->data;
	int rc;

	rc = gpio_pin_configure_dt(&cfg->gpio_interrupt, GPIO_INPUT | GPIO_OUTPUT_HIGH);
	if (rc) {
		LOG_ERR("Failed to configure interrupt GPIO (%d)", rc);
	}
	gpio_init_callback(&data->gpio_cb, ch101_gpio_callback, BIT(cfg->gpio_interrupt.pin));
	rc = gpio_add_callback(cfg->gpio_interrupt.port, &data->gpio_cb);
	if (rc) {
		LOG_ERR("Failed to add interrupt callback (%d)", rc);
	}
}

int ch101_flash_firmware(const struct device *dev, const struct ch_firmware *firmware)
{
	const struct ch101_config *cfg = dev->config;
	struct ch101_data *data = dev->data;
	int rc = 0;

	if (!i2c_is_ready_dt(&data->common_data.i2c)) {
		LOG_ERR("i2c not ready\n");
		return -ENODEV;
	}

	/* Reset the chip */
	LOG_INF("(%s) Resetting", dev->name);
	gpio_pin_set_dt(&cfg->gpio_reset, 0);
	gpio_pin_set_dt(&cfg->gpio_program, 1);
	k_msleep(1);
	gpio_pin_set_dt(&cfg->gpio_reset, 1);

	/* Set the device idle */
	LOG_INF("(%s) Idling", dev->name);
	chdrv_set_idle(data->common_data.i2c.bus);
	gpio_pin_set_dt(&cfg->gpio_program, 0);

	if (firmware == NULL) {
		data->firmware = NULL;
		return 0;
	}

	gpio_pin_set_dt(&cfg->gpio_program, 1);
	LOG_INF("(%s) Initializing RAM", dev->name);
	rc = chdrv_init_ram(data->common_data.i2c.bus, firmware);
	if (rc != 0) {
		LOG_ERR("Failed to init RAM");
		goto done;
	}

	LOG_INF("(%s) Flashing firmware", dev->name);
	rc = chdrv_fw_load(data->common_data.i2c.bus, CH101_PROG_MEM_ADDR, firmware->fw,
			   firmware->fw_size);
	if (rc != 0) {
		LOG_ERR("Failed to load firmware");
		goto done;
	}

	LOG_INF("(%s) Resetting and halting", dev->name);
	rc = chdrv_reset_and_halt(data->common_data.i2c.bus);
	if (rc != 0) {
		LOG_ERR("Failed to reset after loading new firmware");
		goto done;
	}

	LOG_INF("(%s) Setting RW I2C address to 0x%02x", dev->name, data->common_data.i2c.addr);
	uint8_t rw_i2c_addr = data->common_data.i2c.addr & 0xff;
	rc = chdrv_prog_mem_write(data->common_data.i2c.bus, 0x1c5, &rw_i2c_addr, 1);
	if (rc != 0) {
		LOG_ERR("Failed to set RW I2C address");
		goto done;
	}

	/* Run charge pumps */
	LOG_INF("(%s) Running charge pumps", dev->name);
	uint16_t write_val;
	write_val = 0x0200; // XXX need defines
	rc |= chdrv_prog_mem_write(data->common_data.i2c.bus, 0x01A6, (uint8_t *)&write_val,
				   2); // PMUT.CNTRL4 = HVVSS_FON
	k_busy_wait(5000);
	write_val = 0x0600;
	rc |= chdrv_prog_mem_write(data->common_data.i2c.bus, 0x01A6, (uint8_t *)&write_val,
				   2); // PMUT.CNTRL4 = (HVVSS_FON | HVVDD_FON)
	k_busy_wait(5000);
	write_val = 0x0000;
	rc |= chdrv_prog_mem_write(data->common_data.i2c.bus, 0x01A6, (uint8_t *)&write_val,
				   2); // PMUT.CNTRL4 = 0
	if (rc != 0) {
		LOG_ERR("Failed to run charge pumps");
		rc = -EIO;
		goto done;
	}

	LOG_INF("(%s) Exit programming mode", dev->name);
	rc = chdrv_prog_write(data->common_data.i2c.bus, CH_PROG_REG_CPU,
			      2); // Exit programming mode and run the chip
	if (rc != 0) {
		LOG_ERR("Failed to exit programming mode");
		goto done;
	}
done:
	gpio_pin_set_dt(&cfg->gpio_program, 0);

	if (rc == 0) {
		data->firmware = firmware;

		k_msleep(1);

		/* Calibrate the RTC */
		chdrv_measure_rtc(dev, CH101_PART_NUMBER);
	}

	return rc;
}

static int ch101_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct ch101_data *data = dev->data;

	LOG_DBG("%s(chan=%d) SENSOR_CHAN_DISTANCE=%d", __FUNCTION__, chan, SENSOR_CHAN_DISTANCE);
	switch (chan) {
	case SENSOR_CHAN_DISTANCE:
	case SENSOR_CHAN_ALL:
		return ch_common_get_range(dev, CH101_PART_NUMBER, &data->range);
	default:
		return -EINVAL;
	}
}

static int ch101_channel_get(const struct device *dev, enum sensor_channel chan,
			     struct sensor_value *val)
{
	struct ch101_data *data = dev->data;

	LOG_DBG("%s(chan=%d) SENSOR_CHAN_DISTANCE=%d", __FUNCTION__, chan, SENSOR_CHAN_DISTANCE);
	switch (chan) {
	case SENSOR_CHAN_DISTANCE:
	case SENSOR_CHAN_ALL:
		int64_t range_mm = data->range;

		val->val1 = range_mm / 1000;
		val->val2 = (range_mm - (val->val1 * 1000)) * 1000;
		return 0;
	default:
		return -EINVAL;
	}
	return -EINVAL;
}

static int ch101_attr_set(const struct device *dev, enum sensor_channel chan,
			  enum sensor_attribute attr, const struct sensor_value *val)
{
	struct ch101_data *data = dev->data;
	int rc = 0;

	switch (chan) {
	case SENSOR_CHAN_DISTANCE:
		if (attr != SENSOR_ATTR_SAMPLING_FREQUENCY) {
			return -EINVAL;
		}
		int64_t mhz = (val->val1 * INT64_C(1000000) + val->val2) / 1000;
		if (mhz == 0) {
			/* Disable the sampling */
			LOG_DBG("Disabling");
			rc = ch_common_set_mode(dev, CH101_PART_NUMBER, CH_MODE_IDLE);
			if (rc != 0) {
				return rc;
			}
			data->common_data.mode = CH_MODE_IDLE;
			return 0;
		} else {
			uint16_t interval_ms = (uint16_t)MAX(10, INT64_C(1000000) / mhz);

			LOG_DBG("Setting sample interval to %u ms", interval_ms);
//			rc = ch_common_set_sample_interval(dev, CH101_PART_NUMBER, interval_ms);
//			if (rc != 0) {
//				LOG_ERR("Failed to set sample interval");
//				return rc;
//			}
			data->common_data.interval_ms = interval_ms;

			rc = ch_common_set_mode(dev, CH101_PART_NUMBER, CH_MODE_FREERUN);
			if (rc != 0) {
				LOG_ERR("Failed to set mode to 'free-running'");
				return rc;
			}
			data->common_data.mode = CH_MODE_FREERUN;
			ch101_init_trigger(dev);
			return 0;
		}
		break;
	default:
		return -ENOTSUP;
	}
	return -EINVAL;
}

static int ch101_attr_get(const struct device *dev, enum sensor_channel chan,
			  enum sensor_attribute attr, struct sensor_value *val)
{
	struct ch101_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_DISTANCE:
		if (attr != SENSOR_ATTR_SAMPLING_FREQUENCY) {
			return -EINVAL;
		}
		if (data->common_data.interval_ms == 0) {
			val->val1 = 0;
			val->val2 = 0;
		} else {
			uint64_t ufrequency = INT64_C(1000000000) / data->common_data.interval_ms;

			val->val1 = ufrequency / 1000000;
			val->val2 = ufrequency - (val->val1 * 1000000);
		}
		return 0;
	default:
		return -ENOTSUP;
	}
}

static const struct sensor_driver_api ch101_driver_api = {
	.sample_fetch = ch101_sample_fetch,
	.channel_get = ch101_channel_get,
	.attr_set = ch101_attr_set,
	.attr_get = ch101_attr_get,
};

static inline int ch101_check_connected(const struct device *dev)
{
	const struct ch101_config *cfg = dev->config;
	struct ch101_data *data = dev->data;
	uint8_t sig_bytes[2];
	uint8_t sig_addr = 0;
	int rc = 0;

	/* Set the RESET and PROGRAM gpios high */
	gpio_pin_set_dt(&cfg->gpio_reset, 1);
	gpio_pin_set_dt(&cfg->gpio_program, 1);

	/* Read the SIG values */
	rc = i2c_write_read(data->common_data.i2c.bus, CH_I2C_ADDR_PROG, &sig_addr, 1, sig_bytes,
			    2);
	if (rc != 0) {
		LOG_ERR("Failed to read SIG bytes");
		rc = -ENODEV;
	}
	if (rc == 0 && (sig_bytes[0] != CH_SIG_BYTE_0 || sig_bytes[1] != CH_SIG_BYTE_1)) {
		LOG_ERR("Incorrect SIG bytes 0x%02x%02x", sig_bytes[0], sig_bytes[1]);
		rc = -ENODEV;
	}

	/* Reset the RESET and PROGRAM gpios low */
	gpio_pin_set_dt(&cfg->gpio_reset, 0);
	gpio_pin_set_dt(&cfg->gpio_program, 0);

	return rc;
}

static int ch101_init(const struct device *dev)
{
	const struct ch101_config *cfg = dev->config;
	struct ch101_data *data = dev->data;
	const struct ch_firmware *firmware = NULL;
	int rc;

	data->common_data.gpio_int = &cfg->gpio_interrupt;
	if (!gpio_is_ready_dt(&cfg->gpio_program)) {
		LOG_ERR("PROGRAM GPIO not ready");
		return -ENODEV;
	}

	rc = gpio_pin_configure_dt(&cfg->gpio_program, GPIO_OUTPUT);
	if (rc != 0) {
		LOG_ERR("Failed to configure PROGRAM gpio");
		return -ENODEV;
	}

	rc = gpio_pin_configure_dt(&cfg->gpio_reset, GPIO_OUTPUT);
	if (rc != 0) {
		LOG_ERR("Failed to configure RESET gpio");
		return -ENODEV;
	}

	ch101_init_trigger(dev);

	/* Check that the device is connected */
	rc = ch101_check_connected(dev);
	if (rc != 0) {
		return rc;
	}
	LOG_DBG("CH101 found at %s/0x%02x",
		data->common_data.i2c.bus->name ? data->common_data.i2c.bus->name : "NULL",
		data->common_data.i2c.addr);

	switch (cfg->default_firmware) {
#ifdef CONFIG_CH101_GPR_FW
	case DEFAULT_FIRMWARE_GPR:
		LOG_DBG("Loading GPR firmware");
		firmware = &ch101_gpr_fw;
		break;
#endif
#ifdef CONFIG_CH101_GPR_SR_FW
	case DEFAULT_FIRMWARE_GPR_SR:
		LOG_DBG("Loading GPR-SR firmware");
		firmware = &ch101_gpr_sr_fw;
		break;
#endif
	case DEFAULT_FIRMWARE_NONE:
	default:
		firmware = NULL;
		break;
	}

	if (firmware != NULL) {
		rc = ch101_flash_firmware(dev, firmware);
		if (rc != 0) {
			LOG_ERR("Failed to flash firmware (%d)", rc);
			return rc;
		}
		LOG_DBG("Flashed firmware");
	}
	return 0;
}

#define CH101_DEFINE(inst)                                                                         \
	static struct ch101_data ch101_data_##inst = {                                             \
		.common_data =                                                                     \
			{                                                                          \
				.i2c = I2C_DT_SPEC_INST_GET(inst),                                 \
				.scale_factor = 0,                                                 \
			},                                                                         \
	};                                                                                         \
	static struct ch101_config ch101_config_##inst = {                                         \
		.gpio_reset = GPIO_DT_SPEC_INST_GET(inst, reset_gpios),                            \
		.gpio_interrupt = GPIO_DT_SPEC_INST_GET(inst, int_gpios),                          \
		.gpio_program = GPIO_DT_SPEC_INST_GET(inst, program_gpios),                        \
		.default_firmware = UTIL_CAT(DEFAULT_FIRMWARE_,                                    \
					     DT_INST_STRING_UPPER_TOKEN_OR(inst, firmware, NONE)), \
	};                                                                                         \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, ch101_init, NULL, &ch101_data_##inst,                   \
				     &ch101_config_##inst, POST_KERNEL,                            \
				     CONFIG_SENSOR_INIT_PRIORITY, &ch101_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CH101_DEFINE)
