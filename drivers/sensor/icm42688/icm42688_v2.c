/*
 * Copyright (c) 2022 Intel Corporation
 * Copyright (c) 2022 Esco Medical ApS
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/__assert.h>
#define DT_DRV_COMPAT invensense_icm42688

#include "icm42688.h"
#include "icm42688_reg.h"
#include "icm42688_spi.h"

#include <stdint.h>

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_types.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/dsp/dsp.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
LOG_MODULE_REGISTER(ICM42688, CONFIG_SENSOR_LOG_LEVEL);

struct icm42688_sensor_data {
	struct icm42688_dev_data dev_data;

	struct k_mutex data_buffer_lock;
	union {
		uint8_t raw_data_bytes[sizeof(struct sensor_raw_data) + 13];
		struct sensor_raw_data data;
	};
	struct sensor_raw_data *data_buffer;
	sensor_data_callback_t data_callback;

	int16_t readings[7];
};

struct icm42688_sensor_config {
	struct icm42688_dev_cfg dev_cfg;
};

int16_t raw_to_cpu(uint8_t byte_h, uint8_t byte_l)
{
	return sys_le16_to_cpu((byte_h << 8) | byte_l);
}

/**
 * @NOTE Unclear what void *data actually is... is it a senseor_raw_data, sensor three axis?
 * The sensor type shouldn't probably define the data type.
 *
 * @NOTE The fp_t math means I can't reuse the existing, working math, from the v1 API
 * without float unfortunately.
 *
 * @NOTE No guarantee the samples are of the same sampling clock instant either here
 */
static int icm42688_read_data(const struct device *dev, uint32_t *sensor_types,
			      size_t type_list_count, const struct sensor_raw_data **data)
{
	int res;
	struct icm42688_sensor_data *sens_data = dev->data;
	int data_buffer_offset = 0;

	uint8_t raw_data[14];

	k_mutex_lock(&sens_data->data_buffer_lock, K_FOREVER);

	sens_data->data.header.base_timestamp = k_uptime_get() * USEC_PER_MSEC;
	sens_data->data.header.reading_count = 0;
	res = icm42688_read_all(dev, raw_data);

	if (res != 0) {
		LOG_ERR("Error reading data from sensor");
		goto out;
	}

	for (size_t i = 0; i < type_list_count; ++i) {
		uint32_t sensor_type = sensor_types[i];
		if (sensor_type == SENSOR_TYPE_ACCELEROMETER) {
			if (data_buffer_offset + 6 > sens_data->data.header.reading_size) {
				res = -ENOSR;
				break;
			}

			memcpy(sens_data->data.readings + data_buffer_offset, raw_data + 2, 6);
		} else if (sensor_type == SENSOR_TYPE_GYROSCOPE) {
			if (data_buffer_offset + 6 > sens_data->data.header.reading_size) {
				res = -ENOSR;
				break;
			}

			memcpy(sens_data->data.readings + data_buffer_offset, raw_data + 8, 6);
		} else if (sensor_type == SENSOR_TYPE_ACCELEROMETER_TEMPERATURE ||
			   sensor_type == SENSOR_TYPE_GYROSCOPE_TEMPERATURE) {
			if (data_buffer_offset + 2 > sens_data->data.header.reading_size) {
				res = -ENOSR;
				break;
			}

			memcpy(sens_data->data.readings + data_buffer_offset, raw_data, 2);
		} else {
			res = -ENOTSUP;
			break;
		}
	}

out:
	k_mutex_unlock(&sens_data->data_buffer_lock);
	if (res == 0) {
		*data = &sens_data->data;
	}
	return res;
}

static inline int icm42688_accel_range(enum icm42688_accel_fs fs)
{
	int ret = 0;

	switch (fs) {
	case ICM42688_ACCEL_FS_16G:
		ret = 16;
		break;
	case ICM42688_ACCEL_FS_8G:
		ret = 8;
		break;
	case ICM42688_ACCEL_FS_4G:
		ret = 4;
		break;
	case ICM42688_ACCEL_FS_2G:
		ret = 2;
		break;
	}

	return ret;
}

static inline int icm42688_gyro_range_x1000(enum icm42688_gyro_fs fs)
{
	int ret = 0;

	switch (fs) {
	case ICM42688_GYRO_FS_2000:
		ret = 2000000;
		break;
	case ICM42688_GYRO_FS_1000:
		ret = 1000000;
		break;
	case ICM42688_GYRO_FS_500:
		ret = 500000;
		break;
	case ICM42688_GYRO_FS_250:
		ret = 250000;
		break;
	case ICM42688_GYRO_FS_125:
		ret = 125000;
		break;
	case ICM42688_GYRO_FS_62_5:
		ret = 62500;
		break;
	case ICM42688_GYRO_FS_31_25:
		ret = 31250;
		break;
	case ICM42688_GYRO_FS_15_625:
		ret = 15625;
		break;
	}

	return ret;
}

static int icm42688_get_configuration(const struct device *dev, uint32_t sensor_type,
				      enum sensor_configure_mode mode, uint32_t *resolution,
				      uint32_t *range)
{
	int res = 0;
	struct icm42688_dev_data *ddata = dev->data;

	__ASSERT_NO_MSG(mode == SENSOR_CONFIGURE_MODE_ONE_SHOT);

	if (sensor_type == SENSOR_TYPE_ACCELEROMETER) {
		*range = icm42688_accel_range(ddata->cfg.accel_fs);
		// TODO this doesn't work, just because the FIFO is enabled doesn't mean someone
		// can't read_all and the resolution is possibly different
		*resolution = 16;
	} else if (sensor_type == SENSOR_TYPE_GYROSCOPE) {
		// TODO deg/s and rad/s not just degrees or radians
		*range = icm42688_gyro_range_x1000(ddata->cfg.gyro_fs) / 1000;
		// TODO this doesn't work, just because the FIFO is enabled doesn't mean someone
		//   can't read_all and the resolution is possibly different
		*resolution = 16;
	} else if (sensor_type == SENSOR_TYPE_ACCELEROMETER_TEMPERATURE ||
		   sensor_type == SENSOR_TYPE_GYROSCOPE_TEMPERATURE) {
		*range = 100; /* datasheet gives sensitivity (change per bit) and
						  offset, just assume 100C */
		// TODO if temp data is from the fifo the resolution is 8 bits, from the data
		//   registers its 16 bits
		*resolution = 16;
	} else {
		res = -ENOTSUP;
	}

	return res;
}

static inline int icm42688_accel_range_to_fs(uint32_t range,
					     enum sensor_configure_rounding_mode rounding_mode,
					     enum icm42688_accel_fs *fs)
{
	int ret = 0;

	if (range == 16 || (rounding_mode == SENSOR_CONFIGURE_ROUNDING_MODE_UP && range > 8) ||
	    (rounding_mode == SENSOR_CONFIGURE_ROUNDING_MODE_DOWN && range > 16)) {
		*fs = ICM42688_ACCEL_FS_16G;
	} else if (range == 8 ||
		   (rounding_mode == SENSOR_CONFIGURE_ROUNDING_MODE_UP && range > 4) ||
		   (rounding_mode == SENSOR_CONFIGURE_ROUNDING_MODE_DOWN && range < 16 &&
		    range > 8)) {
		*fs = ICM42688_ACCEL_FS_8G;
	} else if (range == 4 ||
		   (rounding_mode == SENSOR_CONFIGURE_ROUNDING_MODE_UP && range > 2) ||
		   (rounding_mode == SENSOR_CONFIGURE_ROUNDING_MODE_DOWN && range < 8 &&
		    range > 4)) {
		*fs = ICM42688_ACCEL_FS_4G;
	} else if (range == 2 ||
		   (rounding_mode == SENSOR_CONFIGURE_ROUNDING_MODE_UP && range > 0) ||
		   (rounding_mode == SENSOR_CONFIGURE_ROUNDING_MODE_DOWN && range < 4 &&
		    range > 2)) {
		*fs = ICM42688_ACCEL_FS_2G;
	} else {
		ret = -1;
	}

	return ret;
}

static inline int icm42688_gyro_range_to_fs(uint32_t range,
					    enum sensor_configure_rounding_mode rounding_mode,
					    enum icm42688_gyro_fs *fs)
{
	int ret = 0;

	if (rounding_mode == SENSOR_CONFIGURE_ROUNDING_MODE_EXACT) {
		if (range == 2000) {
			*fs = ICM42688_GYRO_FS_2000;
		} else if (range == 1000) {
			*fs = ICM42688_GYRO_FS_1000;
		} else if (range == 500) {
			*fs = ICM42688_GYRO_FS_500;
		} else if (range == 250) {
			*fs = ICM42688_GYRO_FS_250;
		} else if (range == 125) {
			*fs = ICM42688_GYRO_FS_125;
		} else if (range == 62 || range == 63) {
			*fs = ICM42688_GYRO_FS_62_5;
		} else if (range == 31 || range == 32) {
			*fs = ICM42688_GYRO_FS_31_25;
		} else if (range == 15 || range == 16) {
			*fs = ICM42688_GYRO_FS_15_625;
		} else {
			ret = -1;
		}
	} else if (rounding_mode == SENSOR_CONFIGURE_ROUNDING_MODE_UP) {
		if (range > 1000) {
			*fs = ICM42688_GYRO_FS_2000;
		} else if (range > 500) {
			*fs = ICM42688_GYRO_FS_1000;
		} else if (range > 250) {
			*fs = ICM42688_GYRO_FS_500;
		} else if (range > 125) {
			*fs = ICM42688_GYRO_FS_250;
		} else if (range > 63) {
			*fs = ICM42688_GYRO_FS_125;
		} else if (range > 32) {
			*fs = ICM42688_GYRO_FS_62_5;
		} else if (range > 16) {
			*fs = ICM42688_GYRO_FS_31_25;
		} else {
			*fs = ICM42688_GYRO_FS_15_625;
		}
	} else {
		if (range >= 2000) {
			*fs = ICM42688_GYRO_FS_2000;
		} else if (range >= 1000) {
			*fs = ICM42688_GYRO_FS_1000;
		} else if (range >= 500) {
			*fs = ICM42688_GYRO_FS_500;
		} else if (range >= 250) {
			*fs = ICM42688_GYRO_FS_250;
		} else if (range >= 125) {
			*fs = ICM42688_GYRO_FS_125;
		} else if (range >= 62) {
			*fs = ICM42688_GYRO_FS_62_5;
		} else if (range >= 31) {
			*fs = ICM42688_GYRO_FS_31_25;
		} else if (range >= 15) {
			*fs = ICM42688_GYRO_FS_15_625;
		} else {
			ret = -1;
		}
	}

	return ret;
}

static int icm42688_get_available_resolutions(const struct device *dev, uint32_t sensor_type,
					      enum sensor_configure_mode mode,
					      const uint16_t **values, size_t *count)
{
	static const uint16_t resolutions[] = {16, 20};
	ARG_UNUSED(dev);
	ARG_UNUSED(mode);

	switch (sensor_type) {
	case SENSOR_TYPE_ACCELEROMETER:
	case SENSOR_TYPE_GYROSCOPE:
		*values = resolutions;
		*count = ARRAY_SIZE(resolutions);
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int icm42688_get_available_ranges(const struct device *dev, uint32_t sensor_type,
					 enum sensor_configure_mode mode, const uint16_t **values,
					 size_t *count)
{
	static const uint16_t gyro_ranges[] = {15, 31, 62, 125, 250, 500, 1000, 2000};
	static const uint16_t accel_ranges[] = {2, 4, 8, 16};
	ARG_UNUSED(dev);
	ARG_UNUSED(mode);

	switch (sensor_type) {
	case SENSOR_TYPE_ACCELEROMETER:
		*values = accel_ranges;
		*count = ARRAY_SIZE(accel_ranges);
		return 0;
	case SENSOR_TYPE_GYROSCOPE:
		*values = gyro_ranges;
		*count = ARRAY_SIZE(gyro_ranges);
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int icm42688_api_configure(const struct device *dev, uint32_t sensor_type,
				  enum sensor_configure_mode mode,
				  enum sensor_configure_rounding_mode rounding_mode,
				  uint32_t resolution, uint32_t range)
{
	int res;
	struct icm42688_sensor_data *data = dev->data;
	struct icm42688_cfg mcfg = data->dev_data.cfg;

	if (sensor_type == SENSOR_TYPE_ACCELEROMETER) {
		enum icm42688_accel_fs fs;

		res = icm42688_accel_range_to_fs(range, rounding_mode, &fs);
		if (res != 0) {
			goto out;
		}
		mcfg.accel_fs = fs;
	} else if (sensor_type == SENSOR_TYPE_GYROSCOPE) {
		enum icm42688_gyro_fs fs;

		res = icm42688_gyro_range_to_fs(range, rounding_mode, &fs);
		if (res != 0) {
			goto out;
		}
		mcfg.gyro_fs = fs;
	} else {
		res = -ENOTSUP;
		goto out;
	}

	/* reconfigure sensor, making the modified config the new sensor config if valid */
	res = icm42688_configure(dev, &mcfg);

out:
	return res;
}

int icm42688_set_resolution(const struct device *dev, uint32_t sesnor_type, uint8_t resolution,
			    bool round_up)
{
	/* TODO streaming mode allows up to 20bits, without streaming mode 16 bits, not sure how to
	   implement this as streaming mode is mutable elsewhere... very confused
	 */

	return -ENOTSUP;
}

int icm42688_get_bias(const struct device *dev, uint32_t sensor_type, int8_t *temperature,
		      int32_t *bias_x, int32_t *bias_y, int32_t *bias_z)
{

	/* TODO get bias values */
	return -ENOTSUP;
}

int icm42688_set_bias(const struct device *dev, uint32_t sensor_type, int8_t temperature,
		      int32_t bias_x, int32_t bias_y, int32_t bias_z)
{

	/* TODO set bias values */
	return -ENOTSUP;
}

static int icm42688_get_sample_rate_available(const struct device *dev, uint32_t sensor_type,
					      const uint32_t **sample_rates, size_t *count)
{
	static const uint32_t accel_rates_mhz[] = {
		32000000, 16000000, 8000000, 4000000, 2000000, 1000000, 500000, 200000,
		100000,	  50000,    25000,   12500,   6250,    3125,	1562,
	};
	static const uint32_t gyro_rates_mhz[] = {
		32000000, 16000000, 8000000, 4000000, 2000000, 1000000,
		500000,	  200000,   100000,  50000,   25000,   12500,
	};

	switch (sensor_type) {
	case SENSOR_TYPE_ACCELEROMETER:
		*sample_rates = accel_rates_mhz;
		*count = ARRAY_SIZE(accel_rates_mhz);
		return 0;
	case SENSOR_TYPE_GYROSCOPE:
		*sample_rates = gyro_rates_mhz;
		*count = ARRAY_SIZE(gyro_rates_mhz);
		return 0;
	default:
		return -ENOTSUP;
	}
}

int icm42688_get_sample_rate(const struct device *dev, uint32_t sensor_type, uint32_t *sample_rate)
{
	int res = 0;

	/* TODO get sample rate */

	return res;
}

static inline int icm42688_accel_rate_to_odr(uint32_t sample_rate, bool round_up,
					     enum icm42688_accel_odr *odr)
{
	int ret;

	if (sample_rate == 32000 || (round_up && sample_rate > 16000)) {
		*odr = ICM42688_ACCEL_ODR_32000;
	} else if (sample_rate == 16000 || (round_up && sample_rate > 8000)) {
		*odr = ICM42688_ACCEL_ODR_16000;
	} else if (sample_rate == 8000 || (round_up && sample_rate > 4000)) {
		*odr = ICM42688_ACCEL_ODR_8000;
	} else if (sample_rate == 4000 || (round_up && sample_rate > 2000)) {
		*odr = ICM42688_ACCEL_ODR_4000;
	} else if (sample_rate == 2000 || (round_up && sample_rate > 1000)) {
		*odr = ICM42688_ACCEL_ODR_2000;
	} else if (sample_rate == 1000 || (round_up && sample_rate > 500)) {
		*odr = ICM42688_ACCEL_ODR_1000;
	} else if (sample_rate == 500 || (round_up && sample_rate > 200)) {
		*odr = ICM42688_ACCEL_ODR_500;
	} else if (sample_rate == 200 || (round_up && sample_rate > 100)) {
		*odr = ICM42688_ACCEL_ODR_200;
	} else if (sample_rate == 100 || (round_up && sample_rate > 50)) {
		*odr = ICM42688_ACCEL_ODR_100;
	} else if (sample_rate == 50 || (round_up && sample_rate > 25)) {
		*odr = ICM42688_ACCEL_ODR_50;
	} else if (sample_rate == 25 || (round_up && sample_rate > 13)) {
		*odr = ICM42688_ACCEL_ODR_25;
	} else if (sample_rate == 12 || sample_rate == 13 || (round_up && sample_rate > 7)) {
		*odr = ICM42688_ACCEL_ODR_12_5;
	} else if (sample_rate == 6 || sample_rate == 7 || (round_up && sample_rate > 4)) {
		*odr = ICM42688_ACCEL_ODR_6_25;
	} else if (sample_rate == 3 || sample_rate == 4 || (round_up && sample_rate > 2)) {
		*odr = ICM42688_ACCEL_ODR_3_125;
	} else if (sample_rate == 1 || sample_rate == 2 || (round_up && sample_rate > 0)) {
		*odr = ICM42688_ACCEL_ODR_1_5625;
	} else {
		ret = -1;
	}

	return ret;
}

static inline int icm42688_gyro_rate_to_odr(uint32_t sample_rate, bool round_up,
					    enum icm42688_gyro_odr *odr)
{
	int ret = 0;

	if (sample_rate == 32000 || (round_up && sample_rate > 16000)) {
		*odr = ICM42688_GYRO_ODR_32000;
	} else if (sample_rate == 16000 || (round_up && sample_rate > 8000)) {
		*odr = ICM42688_GYRO_ODR_16000;
	} else if (sample_rate == 8000 || (round_up && sample_rate > 4000)) {
		*odr = ICM42688_GYRO_ODR_8000;
	} else if (sample_rate == 4000 || (round_up && sample_rate > 2000)) {
		*odr = ICM42688_GYRO_ODR_4000;
	} else if (sample_rate == 2000 || (round_up && sample_rate > 1000)) {
		*odr = ICM42688_GYRO_ODR_2000;
	} else if (sample_rate == 1000 || (round_up && sample_rate > 500)) {
		*odr = ICM42688_GYRO_ODR_1000;
	} else if (sample_rate == 500 || (round_up && sample_rate > 200)) {
		*odr = ICM42688_GYRO_ODR_500;
	} else if (sample_rate == 200 || (round_up && sample_rate > 100)) {
		*odr = ICM42688_GYRO_ODR_200;
	} else if (sample_rate == 100 || (round_up && sample_rate > 50)) {
		*odr = ICM42688_GYRO_ODR_100;
	} else if (sample_rate == 50 || (round_up && sample_rate > 25)) {
		*odr = ICM42688_GYRO_ODR_50;
	} else if (sample_rate == 25 || (round_up && sample_rate > 13)) {
		*odr = ICM42688_GYRO_ODR_25;
	} else if (sample_rate == 12 || sample_rate == 13 || (round_up && sample_rate > 0)) {
		*odr = ICM42688_GYRO_ODR_12_5;
	} else {
		ret = -1;
	}

	return ret;
}

int icm42688_set_sample_rate(const struct device *dev, uint32_t sensor_type, uint32_t sample_rate,
			     bool round_up)
{
	struct icm42688_sensor_data *data = dev->data;

	int res;

	/* copy configuration */
	struct icm42688_cfg mcfg = data->dev_data.cfg;

	if (sensor_type == SENSOR_TYPE_ACCELEROMETER) {
		enum icm42688_accel_odr odr;

		res = icm42688_accel_rate_to_odr(sample_rate, round_up, &odr);
		if (res != 0) {
			goto out;
		}
		mcfg.accel_odr = odr;
	} else if (sensor_type == SENSOR_TYPE_GYROSCOPE) {
		enum icm42688_gyro_odr odr;

		res = icm42688_gyro_rate_to_odr(sample_rate, round_up, &odr);
		if (res != 0) {
			goto out;
		}
		mcfg.gyro_odr = odr;
	} else {
		res = -ENOTSUP;
		goto out;
	}

	/* reconfigure sensor, making the modified config the new sensor config if valid */
	res = icm42688_configure(dev, &mcfg);

out:
	return res;
}

int icm42688_set_data_buffer(const struct device *dev, struct sensor_raw_data *buffer)
{
	struct icm42688_sensor_data *data = dev->data;

	k_mutex_lock(&data->data_buffer_lock, K_FOREVER);
	data->data_buffer = buffer;
	k_mutex_unlock(&data->data_buffer_lock);

	return 0;
}

int icm42688_set_data_callback(const struct device *dev, sensor_data_callback_t callback)
{
	struct icm42688_sensor_data *data = dev->data;

	k_mutex_lock(&data->data_buffer_lock, K_FOREVER);
	data->data_callback = callback;
	k_mutex_unlock(&data->data_buffer_lock);

	return 0;
}

static int icm42688_flush_fifo(const struct device *dev)
{
	return -ENOTSUP;
}

int icm42688_get_watermark(const struct device *dev, uint8_t *wm_pct)

{
	return -ENOTSUP;
}

int icm42688_set_watermark(const struct device *dev, uint8_t wm_pct, bool round_up)

{
	struct icm42688_sensor_data *data = dev->data;
	int res;

	/* copy configuration */
	struct icm42688_cfg mcfg = data->dev_data.cfg;

	const int FIFO_SIZE = 2048;
	const int HIRES_PKT = 20;
	const int PKT = 16;

	__ASSERT(wm_pct <= 100 && wm_pct >= 0, "watermark percentage should be in range 0 to 100");

	int pkt_sz = mcfg.fifo_hires ? HIRES_PKT : PKT;
	int n_pkts = FIFO_SIZE / pkt_sz;
	int wm = (n_pkts * wm_pct) / (100);

	mcfg.fifo_wm = wm;

	/* reconfigure sensor, making the modified config the new sensor config if valid */
	res = icm42688_configure(dev, &mcfg);

	return res;
}

int icm42688_get_streaming_mode(const struct device *dev, bool *enabled)
{

	struct icm42688_sensor_data *data = dev->data;

	*enabled = data->dev_data.cfg.fifo_en;

	return 0;
}

int icm42688_set_streaming_mode(const struct device *dev, bool enabled)
{
	struct icm42688_sensor_data *data = dev->data;
	int res;

	/* copy configuration */
	struct icm42688_cfg mcfg = data->dev_data.cfg;

	/* TODO Might be cool to enable this */
	mcfg.fifo_hires = false;
	mcfg.fifo_en = true;

	/* reconfigure sensor, making the modified config the new sensor config if valid */
	res = icm42688_configure(dev, &mcfg);

	return res;
}

int icm42688_perform_calibration(const struct device *dev, bool enabled)
{
	return -ENOTSUP;
}

static int icm42688_init(const struct device *dev)
{
	struct icm42688_sensor_data *data = dev->data;
	const struct icm42688_sensor_config *cfg = dev->config;
	int res;

	if (!spi_is_ready(&cfg->dev_cfg.spi)) {
		LOG_ERR("SPI bus is not ready");
		return -ENODEV;
	}

	res = icm42688_reset(dev);
	if (res != 0) {
		LOG_ERR("could not initialize sensor");
		return -EIO;
	}
	k_mutex_init(&data->data_buffer_lock);

	// TODO interpret the config params from DT here using the X to Y conversions.
	data->dev_data.cfg.accel_mode = ICM42688_ACCEL_LN;
	data->dev_data.cfg.gyro_mode = ICM42688_GYRO_LN;
	data->dev_data.cfg.accel_fs = ICM42688_ACCEL_FS_16G;
	data->dev_data.cfg.gyro_fs = ICM42688_GYRO_FS_2000;
	data->dev_data.cfg.accel_odr = ICM42688_ACCEL_ODR_32000;
	data->dev_data.cfg.gyro_odr = ICM42688_GYRO_ODR_32000;

	res = icm42688_configure(dev, &data->dev_data.cfg);

	if (res != 0) {
		LOG_ERR("could not configure sensor");
	}

	return res;
}

static const struct sensor_driver_api_v2 icm42688_driver_api = {
	.set_data_buffer = icm42688_set_data_buffer,
	.set_data_callback = icm42688_set_data_callback,
	.read_data = icm42688_read_data,
	.get_raw_data_converter = NULL,
	.get_available_resolutions = icm42688_get_available_resolutions,
	.get_available_ranges = icm42688_get_available_ranges,
	.configure = icm42688_api_configure,
	.get_configuration = icm42688_get_configuration,
	.get_bias = icm42688_get_bias,
	.set_bias = icm42688_set_bias,
#ifdef CONFIG_SENSOR_STREAMING_MODE

	.flush_fifo = icm42688_flush_fifo,
	.get_available_sample_rates = icm42688_get_sample_rate_available,
	.get_sample_rate = icm42688_get_sample_rate,
	.set_sample_rate = icm42688_set_sample_rate,
	.get_watermark = icm42688_get_watermark,
	.set_watermark = icm42688_set_watermark,
	.get_streaming_mode = icm42688_get_streaming_mode,
	.set_streaming_mode = icm42688_set_streaming_mode,
#endif /* CONFIG_SENSOR_STREAMING_MODE */
#ifdef CONFIG_SENSOR_HW_CALIBRATION
	.perform_calibration = icm42688_perform_calibration,
#endif
};

/* device defaults to spi mode 0/3 support */
#define ICM42688_SPI_CFG                                                                           \
	SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_TRANSFER_MSB

#define ICM42688_INIT(inst)                                                                        \
	static struct icm42688_sensor_data icm42688_driver_##inst = {};                            \
                                                                                                   \
	static const struct icm42688_sensor_config icm42688_cfg_##inst = {                         \
		.dev_cfg =                                                                         \
			{                                                                          \
				.spi = SPI_DT_SPEC_INST_GET(inst, ICM42688_SPI_CFG, 0U),           \
			},                                                                         \
	};                                                                                         \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, icm42688_init, NULL, &icm42688_driver_##inst,           \
				     &icm42688_cfg_##inst, POST_KERNEL,                            \
				     CONFIG_SENSOR_INIT_PRIORITY, &icm42688_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ICM42688_INIT)
