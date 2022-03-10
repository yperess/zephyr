/*
 * Copyright (c) 2022 Google Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file drivers/sensor_v2.h
 *
 * @brief Public APIs for the sensor driver.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_V2_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_V2_H_

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_H_
#error "Please include drivers/sensor.h directly"
#endif

/**
 * @brief Sensor Interface V2
 * @defgroup sensor_interface Sensor Interface
 * @ingroup io_interfaces
 * @{
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor_types.h>
//#include <zephyr/drivers/sensor_utils.h>
#include <zephyr/dsp/dsp.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

enum sensor_configure_mode {
	SENSOR_CONFIGURE_MODE_ONE_SHOT,
	SENSOR_CONFIGURE_MODE_CONTINUOUS,
};

enum sensor_configure_rounding_mode {
	SENSOR_CONFIGURE_ROUNDING_MODE_EXACT,
	SENSOR_CONFIGURE_ROUNDING_MODE_DOWN,
	SENSOR_CONFIGURE_ROUNDING_MODE_UP,
};

/**
 * @typedef sensor_set_data_buffer_t
 * @brief Sensor API function for setting the data buffer for reads
 *
 * @see sensor_set_data_buffer() for argument description
 */
typedef int (*sensor_set_data_buffer_t)(const struct device *sensor,
					struct sensor_raw_data *buffer);

/**
 * @typedef sensor_data_callback_t
 * @brief Callback type for handling asynchronous data
 *
 * This callback will be called once per data generated by the sensor in an asynchronous manner.
 * Usually this means that the sensor is using a hardware FIFO and data was being read. There will
 * be one call per data event. The data pointer (set by sensor_set_data_buffer()) will contain
 * the readings. The number of readings can be obtained via data->reading_count, while the
 * individual readings can be attained by accessing the data->readings[] array. Note that each
 * reading has the sensor type associated with it to allow mixed buffers.
 *
 * @param sensor Pointer to the sensor device
 * @param data The data generated
 * @return 0 on success
 * @return < 0 on failure
 *
 * @see sensor_set_data_callback() for use.
 */
typedef int (*sensor_data_callback_t)(const struct device *sensor, struct sensor_raw_data *data);

/**
 * @typedef sensor_set_data_callback_t
 * @brief Sensor API function for setting the asynchronous callback handler function
 *
 * @see sensor_set_data_callback() for argument description
 */
typedef int (*sensor_set_data_callback_t)(const struct device *sensor,
					  sensor_data_callback_t callback);

/**
 * @typedef sensor_read_data_t
 * @brief Sensor API function for reading a single data point
 *
 * @see sensor_read_data() for argument description
 */
typedef int (*sensor_read_data_t)(const struct device *sensor, uint32_t *sensor_types,
				  size_t type_list_count, const struct sensor_raw_data **data);

/**
 * A separate API used to convert raw data stored in the struct sensor_raw_data into more friendly
 * and less compact formats (uses SI units).
 */
struct sensor_raw_data_converter_api {
	/**
	 * @brief Get the number of samples stored in a given buffer
	 *
	 * @param buffer The buffer that was returned in the data callback
	 * @param count The number of valid entries in the buffer.
	 * @return 0 on success
	 * @return < 0 on failure
	 */
	int (*get_sample_count)(const struct sensor_raw_data *buffer, size_t *count);

	/**
	 * @brief Get the sensor type represented by the sample at a given offset.
	 *
	 * This call should be combined with get_sensor_data_at() by providing the correct data
	 * struct type. See sensor_types.h for more information about SENSOR_TYPE_* mapping to the
	 * data struct types.
	 *
	 * @param buffer The buffer that was returned in the data callback
	 * @param offset The offset into the buffer (reading number)
	 * @param sensor_type The sensor type of the given reading.
	 * @return 0 on success
	 * @return < 0 on failure
	 */
	int (*get_sensor_type_at)(const struct sensor_raw_data *buffer, size_t offset,
				  uint32_t *sensor_type);
	/**
	 * @brief Get the data represented by the sample at a given offset
	 *
	 * @param buffer The buffer that was returned in the data callback
	 * @param offset The offset into the buffer (reading number)
	 * @param data Pointer to the correct data structure to hold the result
	 * @return 0 on success
	 * @return < 0 on failure
	 */
	int (*get_sensor_data_at)(const struct sensor_raw_data *buffer, size_t offset, void *data);
};

/**
 * @typedef sensor_get_raw_data_converter_t
 * @brief Sensor API function for getting the converter API
 *
 * @see sensor_raw_data_converter_api
 * @see sensor_get_raw_data_converter()
 */
typedef const struct sensor_raw_data_converter_api *(*sensor_get_raw_data_converter_t)(
	const struct device *sensor);

/**
 * @typedef sensor_get_available_values_t
 * @brief Sensor API function for getting a list of available values
 *
 * This is a generic API function type that is used for various call points
 * @see sensor_get_available_resolutions()
 * @see sensor_get_available_ranges()
 * @see sensor_get_available_sample_rates()
 */
typedef int (*sensor_get_available_values_t)(const struct device *sensor, uint32_t sensor_type,
					     enum sensor_configure_mode mode,
					     const uint16_t **values, size_t *count);

/**
 * @typedef sensor_get_available_values_t
 * @brief Sensor API function for getting a list of available sample rate values
 *
 * @see sensor_get_available_sample_rates()
 */
typedef int (*sensor_get_available_sample_rates_t)(const struct device *sensor,
						   uint32_t sensor_type, const uint32_t **values,
						   size_t *count);

/**
 * @typedef sensor_configure_t
 * @brief Sensor API function for setting basic configuration
 *
 * @see sensor_configure() for argument description
 */
typedef int (*sensor_configure_t)(const struct device *sensor, uint32_t sensor_type,
				  enum sensor_configure_mode mode,
				  enum sensor_configure_rounding_mode rounding_mode,
				  uint32_t resolution, uint32_t range);

/**
 * @typedef sensor_get_configuration_t
 * @brief Sensor API function for getting the current basic configuration
 *
 * @see sensor_get_configuration() for argument description
 */
typedef int (*sensor_get_configuration_t)(const struct device *sensor, uint32_t sensor_type,
					  enum sensor_configure_mode mode, uint32_t *resolution,
					  uint32_t *range);

/**
 * @typedef sensor_get_bias_t
 * @brief Sensor API function for getting the current bias and the temperature at which it was set
 *
 * @see sensor_get_bias() for argument description
 */
typedef int (*sensor_get_bias_t)(const struct device *sensor, uint32_t sensor_type,
				 int8_t *temperature, int32_t *bias_x, int32_t *bias_y,
				 int32_t *bias_z);

/**
 * @typedef sensor_set_bias_t
 * @brief Sensor API function for setting the bias and the temperature at which it was attained
 *
 * @see sensor_set_bias() for argument description
 */
typedef int (*sensor_set_bias_t)(const struct device *sensor, uint32_t sensor_type,
				 int8_t temperature, int32_t bias_x, int32_t bias_y,
				 int32_t bias_z);

/**
 * @typedef sensor_set_resolution_t
 * @brief Sensor API function for setting the current resolution in bits/sample
 *
 * @see sensor_set_resolution() for argument description
 */
typedef int (*sensor_set_resolution_t)(const struct device *sensor, uint32_t sensor_type,
				       uint8_t resolution, bool round_up);

/**
 * @typedef sensor_flush_fifo_t
 * @brief Sensor API function for flushing the hardware FIFO
 *
 * @see sensor_flush_fifo() for argument description
 */
typedef int (*sensor_flush_fifo_t)(const struct device *sensor);

/**
 * @typedef sensor_get_sample_rate_t
 * @brief Sensor API function for getting the current sample rate
 *
 * @see sensor_get_sample_rate() for argument description
 */
typedef int (*sensor_get_sample_rate_t)(const struct device *sensor, uint32_t sensor_type,
					uint32_t *sample_rate);

/**
 * @typedef sensor_set_sample_rate_t
 * @brief Sensor API function for setting the current sample rate
 *
 * @see sensor_set_sample_rate() for argument description
 */
typedef int (*sensor_set_sample_rate_t)(const struct device *sensor, uint32_t sensor_type,
					uint32_t sample_rate, bool round_up);

/**
 * @typedef sensor_fifo_get_watermark_t
 * @brief Sensor API function for getting the current FIFO watermark
 *
 * @see sensor_fifo_get_watermark() for argument description
 */
typedef int (*sensor_fifo_get_watermark_t)(const struct device *sensor, uint8_t *watermark_percent);

/**
 * @typedef sensor_fifo_set_watermark_t
 * @brief Sensor API function for setting the current FIFO watermark
 *
 * @see sensor_fifo_set_watermark() for argument description
 */
typedef int (*sensor_fifo_set_watermark_t)(const struct device *sensor, uint8_t watermark_percent,
					   bool round_up);

/**
 * @typedef sensor_fifo_get_streaming_mode_t
 * @brief Sensor API function for checking if streaming mode is enabled in the FIFO
 *
 * @see sensor_fifo_get_streaming_mode() for argument description
 */
typedef int (*sensor_fifo_get_streaming_mode_t)(const struct device *sensor, bool *enabled);

/**
 * @typedef sensor_fifo_set_streaming_mode_t
 * @brief Sensor API function for enabling/disabling the FIFO streaming mode
 *
 * @see sensor_fifo_set_streaming_mode() for argument description
 */
typedef int (*sensor_fifo_set_streaming_mode_t)(const struct device *sensor, bool enabled);

/**
 * @typedef sensor_perform_calibration_t
 * @brief Sensor API function for enabling/disabling live calibration
 *
 * @see sensor_perform_calibration() for argument description
 */
typedef int (*sensor_perform_calibration_t)(const struct device *sensor, bool enable);

__subsystem struct sensor_driver_api_v2 {
	sensor_set_data_buffer_t set_data_buffer;
	sensor_set_data_callback_t set_data_callback;
	sensor_read_data_t read_data;
	sensor_get_raw_data_converter_t get_raw_data_converter;
	sensor_get_available_values_t get_available_resolutions;
	sensor_get_available_values_t get_available_ranges;
	sensor_configure_t configure;
	sensor_get_configuration_t get_configuration;
	sensor_get_bias_t get_bias;
	sensor_set_bias_t set_bias;
#ifdef CONFIG_SENSOR_STREAMING_MODE
	sensor_flush_fifo_t flush_fifo;
	sensor_get_available_sample_rates_t get_available_sample_rates;
	sensor_get_sample_rate_t get_sample_rate;
	sensor_set_sample_rate_t set_sample_rate;
	sensor_fifo_get_watermark_t get_watermark;
	sensor_fifo_set_watermark_t set_watermark;
	sensor_fifo_get_streaming_mode_t get_streaming_mode;
	sensor_fifo_set_streaming_mode_t set_streaming_mode;
#endif /* CONFIG_SENSOR_STREAMING_MODE */
#ifdef CONFIG_SENSOR_HW_CALIBRATION
	sensor_perform_calibration_t perform_calibration;
#endif /* CONFIG_SENSOR_HW_CALIBRATION */
};

/**
 * @brief Set the buffer to use for the next read
 *
 * @param sensor Pointer to the sensor device
 * @param buffer The buffer to use
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_set_data_buffer(const struct device *sensor, struct sensor_raw_data *buffer);

static inline int z_impl_sensor_set_data_buffer(const struct device *sensor,
						struct sensor_raw_data *buffer)
{
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->set_data_buffer == NULL) {
		return -ENOSYS;
	}

	return api->set_data_buffer(sensor, buffer);
}

/**
 * @brief Set the processing function for new data.
 *
 * Set the sensor's function for processing data. This function provides the callback for the
 * sensor under both one shot and continuous modes. Once the callback is triggered, a new call to
 * sensor_set_data_buffer() is expected. This can be done after the callback, but it is highly
 * recommended that it is done during the callback. Otherwise, some data in continuous mode may
 * be lost.
 *
 * @param sensor Pointer to the sensor device
 * @param callback A callback function to use when new data is available
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_set_data_callback(const struct device *sensor,
				       sensor_data_callback_t callback);

static inline int z_impl_sensor_set_data_callback(const struct device *sensor,
						  sensor_data_callback_t callback)
{
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->set_data_callback == NULL) {
		return -ENOSYS;
	}

	return api->set_data_callback(sensor, callback);
}

/**
 * @brief Read sensor data.
 *
 * Read a single sample from the sensor. The sample may comprise of multiple types by supplying an
 * array of the sensor types along with a type_list_count that is > 1. If successful, the 'data'
 * pointer will contain valid data. This data will only be valid until the next call to
 * sensor_read_data(), so applications wanting to perform additional processing should copy the data
 * to a local buffer.
 *
 * @param sensor Pointer to the sensor device
 * @param sensor_types The type of sensors to read (must be one of SENSOR_TYPE_* or a custom vendor
 *        specific value).
 * @param type_list_count The number of items in the sensor_types array.
 * @param data Pointer to sensor data if return was 0.
 * @return 0 on success
 * @return -ENOMEM if buffer provided in sensor_set_data_buffer() isn't big enough.
 * @return < 0 on failure
 */
__syscall int sensor_read_data(const struct device *sensor, uint32_t *sensor_types,
			       size_t type_list_count, const struct sensor_raw_data **data);

static inline int z_impl_sensor_read_data(const struct device *sensor, uint32_t *sensor_types,
					  size_t type_list_count, const struct sensor_raw_data **data)
{
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->read_data == NULL || sensor_types == NULL || type_list_count == 0 || data == NULL) {
		return -ENOSYS;
	}

	return api->read_data(sensor, sensor_types, type_list_count, data);
}

/**
 * @brief Get the raw data converter API for the given sensor
 *
 * The API is used to convert samples from struct sensor_raw_data into type specific sensor data.
 * For example, given a sample with accelerometer data:
 *   size_t count;
 *   uint32_t sensor_type;
 *   struct sensor_three_axis_data data;
 *   const struct sensor_raw_data_converter_api *api = sensor_get_raw_data_converter(sensor);
 *   ASSERT(api->get_sample_count(raw_data, &count) == 0);
 *   ASSERT(count == 1);
 *   ASSERT(api->get_sensor_type_at(raw_data, 0, &sensor_type) == 0);
 *   ASSERT(sensor_type == SENSOR_TYPE_ACCELEROMETER);
 *   ASSERT(api->get_sensor_data_at(raw_data, 0, &data) == 0);
 *
 * This is a separate API in order to avoid relying on current sensor state and making each
 * conversion a system call.
 *
 * @param sensor Pointer to the sensor device
 * @return Pointer to the data converter API or NULL if not provided
 */
__syscall const struct sensor_raw_data_converter_api *sensor_get_raw_data_converter(
	const struct device *sensor);

static inline const struct sensor_raw_data_converter_api *z_impl_sensor_get_raw_data_converter(
	const struct device *sensor)
{
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->get_raw_data_converter == NULL) {
		return NULL;
	}

	return api->get_raw_data_converter(sensor);
}

/**
 * @brief Get the sensor's available resolutions (in bits)
 *
 * @param sensor Pointer to the sensor device
 * @param sensor_type The type of sensor to query
 * @param mode The mode for which to get the available resolutions (one shot / streaming)
 * @param resolutions Array pointer that will be set with the available resolutions
 * @param count The number of elements in the resolutions array
 * @return 0 success
 * @return < 0 on failure
 */
__syscall int sensor_get_available_resolutions(const struct device *sensor, uint32_t sensor_type,
					       enum sensor_configure_mode mode,
					       const uint16_t **resolutions, size_t *count);

static inline int z_impl_sensor_get_available_resolutions(const struct device *sensor,
							  uint32_t sensor_type,
							  enum sensor_configure_mode mode,
							  const uint16_t **resolutions,
							  size_t *count)
{
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->get_available_resolutions == NULL) {
		return -ENOSYS;
	}

	return api->get_available_resolutions(sensor, sensor_type, mode, resolutions, count);
}

/**
 * @brief Get the sensor's available ranges (in the sensor_type's units)
 *
 * @param sensor Pointer to the sensor device
 * @param sensor_type The type of sensor to query
 * @param mode The mode for which to get the available ranges (one shot / streaming)
 * @param ranges Array pointer that will be set with the available ranges
 * @param count The number of elements in the ranges array
 * @return 0 success
 * @return < 0 on failure
 */
__syscall int sensor_get_available_ranges(const struct device *sensor, uint32_t sensor_type,
					  enum sensor_configure_mode mode, const uint16_t **ranges,
					  size_t *count);

static inline int z_impl_sensor_get_available_ranges(const struct device *sensor,
						     uint32_t sensor_type,
						     enum sensor_configure_mode mode,
						     const uint16_t **ranges, size_t *count)
{
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->get_available_ranges == NULL) {
		return -ENOSYS;
	}

	return api->get_available_ranges(sensor, sensor_type, mode, ranges, count);
}

/**
 * @brief Configure the core properties of the sensor.
 *
 * Use this function to configure the sensor's given mode (one shot / continuous). If exact values
 * are expected, they can be attained from sensor_get_available_resolution() and
 * sensor_get_available_ranges(). The rounding_mode parameter can then be set to
 * SENSOR_CONFIGURE_ROUNDING_MODE_EXACT, which will make this function fail if the sensor could not
 * be configured to the exact range/resolution combination. Alternatively,
 * SENSOR_CONFIGURE_ROUNDING_MODE_DOWN and SENSOR_CONFIGURE_ROUNDING_MODE_UP can be used if loose
 * values are permitted.
 *
 * @param sensor Pointer to the sensor device
 * @param sensor_type The sensor type to configure
 * @param mode Which mode of the sensor should be configured
 * @param rounding_mode How to treat inexact resolution/range values
 * @param resolution The desired resolution (in bits)
 * @param range The desired range (in the type's units)
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_configure(const struct device *sensor, uint32_t sensor_type,
			       enum sensor_configure_mode mode,
			       enum sensor_configure_rounding_mode rounding_mode,
			       uint32_t resolution, uint32_t range);

static inline int z_impl_sensor_configure(const struct device *sensor, uint32_t sensor_type,
					  enum sensor_configure_mode mode,
					  enum sensor_configure_rounding_mode rounding_mode,
					  uint32_t resolution, uint32_t range)
{
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->configure == NULL) {
		return -ENOSYS;
	}

	return api->configure(sensor, sensor_type, mode, rounding_mode, resolution, range);
}

/**
 * @brief Get the sensor's current configuration
 *
 * @param sensor Pointer to the sensor device
 * @param sensor_type The type of sensor to query
 * @param mode The sensor's mode for which the configuration should be queried
 * @param resolution Pointer to store the current resolution (in bits)
 * @param range Pointer to store the current range (in the sensor type's units).
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_get_configuration(const struct device *sensor, uint32_t sensor_type,
				       enum sensor_configure_mode mode, uint32_t *resolution,
				       uint32_t *range);

static inline int z_impl_sensor_get_configuration(const struct device *sensor, uint32_t sensor_type,
						  enum sensor_configure_mode mode,
						  uint32_t *resolution, uint32_t *range)
{
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->get_configuration == NULL) {
		return -ENOSYS;
	}

	return api->get_configuration(sensor, sensor_type, mode, resolution, range);
}

/**
 * @brief Get the latest bias for a three axis sensor
 *
 * Get the latest bias set for this sensor. It is important to take note of the temperature at which
 * the bias was set, this will help determine how accurate the bias is.
 *
 * @param sensor Pointer to the sensor device
 * @param sensor_type The type of sensor to read
 * @param temperature Pointer to the temperature variable that will be set on success
 * @param bias_x Pointer to the X component of the bias that will be set on success
 * @param bias_y Pointer to the Y component of the bias that will be set on success
 * @param bias_z Pointer to the Z component of the bias that will be set on success
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_get_bias(const struct device *sensor, uint32_t sensor_type,
			      int8_t *temperature, int32_t *bias_x, int32_t *bias_y,
			      int32_t *bias_z);

static inline int z_impl_sensor_get_bias(const struct device *sensor, uint32_t sensor_type,
					 int8_t *temperature, int32_t *bias_x, int32_t *bias_y,
					 int32_t *bias_z)
{
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->get_bias == NULL) {
		return -ENOSYS;
	}

	return api->get_bias(sensor, sensor_type, temperature, bias_x, bias_y, bias_z);
}

/**
 * @brief Manually set the bias for a three axis sensor
 *
 * Update the sensor's bias. This bias can be used to account for slight changes in the sensor's
 * accuracy (especially when the sensor's internal temperature varies).
 *
 * @param sensor Pointer to the sensor device
 * @param sensor_type The type of sensor to read
 * @param temperature The temperature at which the bias was calculated
 * @param bias_x The X component of the bias
 * @param bias_y The Y component of the bias
 * @param bias_z The Z component of the bias
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_set_bias(const struct device *sensor, uint32_t sensor_type, int8_t temperature,
			      int32_t bias_x, int32_t bias_y, int32_t bias_z);

static inline int z_impl_sensor_set_bias(const struct device *sensor, uint32_t sensor_type,
					 int8_t temperature, int32_t bias_x, int32_t bias_y,
					 int32_t bias_z)
{
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->set_bias == NULL) {
		return -ENOSYS;
	}

	return api->set_bias(sensor, sensor_type, temperature, bias_x, bias_y, bias_z);
}

/**
 * @brief Flush the sensor's hardware FIFO
 *
 * Attempt to flush the FIFO. Calling this function will asynchronously trigger the callback
 * function set in sensor_set_data_callback(). It will write data to the buffer set in
 * sensor_set_data_buffer()
 *
 * @param sensor Pointer to the sensor device
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_flush_fifo(const struct device *sensor);

static inline int z_impl_sensor_flush_fifo(const struct device *sensor)
{
#ifdef CONFIG_SENSOR_STREAMING_MODE
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->flush_fifo == NULL) {
		return -ENOSYS;
	}

	return api->flush_fifo(sensor);
#else
	return -ENOSYS
#endif
}

/**
 * @brief Get the available sample rates for the sensor
 *
 * Get the various available sample rates for the sensor (in mHz). This API
 * supports pagination by making use of the max_count argument. If the sensor, for
 * example, has 10 different supported sample rates and max_count is 5; the first
 * call can fetch the first 5 possible values, and the second call can fetch the
 * last 5 via the offset argument. The function should be re-attempted if the
 * return value is the same as max_count.
 *
 * @param sensor Pointer to the sensor device
 * @param sensor_type The type of sensor to read
 * @param sample_rates Array pointer that will be set with the available sample rates.
 * @param count The number of elements in the sample_rates array
 * @return 0 on success
 * @return < 0 on error
 */
__syscall int sensor_get_available_sample_rates(const struct device *sensor, uint32_t sensor_type,
						const uint32_t **sample_rates, size_t *count);

static inline int z_impl_sensor_get_available_sample_rates(const struct device *sensor,
							   uint32_t sensor_type,
							   const uint32_t **sample_rates,
							   size_t *count)
{
#ifdef CONFIG_SENSOR_STREAMING_MODE
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->get_available_sample_rates == NULL) {
		return -ENOSYS;
	}

	return api->get_available_sample_rates(sensor, sensor_type, sample_rates, count);
#else
	return -ENOSYS;
#endif
}

/**
 * @brief Get the sensor's sample rate
 *
 * Get the sample rate of the sensor in milli-Hz.
 *
 * @param sensor Pointer to the sensor device
 * @param sensor_type The type of sensor to read
 * @param sample_rate Pointer to the sample rate variable that will be set on success
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_get_sample_rate(const struct device *sensor, uint32_t sensor_type,
				     uint32_t *sample_rate);

static inline int z_impl_sensor_get_sample_rate(const struct device *sensor, uint32_t sensor_type,
						uint32_t *sample_rate)
{
#ifdef CONFIG_SENSOR_STREAMING_MODE
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->get_sample_rate == NULL) {
		return -ENOSYS;
	}

	return api->get_sample_rate(sensor, sensor_type, sample_rate);
#else
	return -ENOSYS;
#endif
}

/**
 * @brief Update the sensor's sample rate
 *
 * Set a new sample rate for the sensor. Sample rates should be in milli-Hz.
 *
 * @param sensor Pointer to the sensor device
 * @param sensor_type The type of sensor to read
 * @param sample_rate The new sample rate in milli-Hz
 * @param round_up Whether or not to round up to the nearest available sample rate
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_set_sample_rate(const struct device *sensor, uint32_t sensor_type,
				     uint32_t sample_rate, bool round_up);

static inline int z_impl_sensor_set_sample_rate(const struct device *sensor, uint32_t sensor_type,
						uint32_t sample_rate, bool round_up)
{
#ifdef CONFIG_SENSOR_STREAMING_MODE
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->set_sample_rate == NULL) {
		return -ENOSYS;
	}

	return api->set_sample_rate(sensor, sensor_type, sample_rate, round_up);
#else
	return -ENOSYS;
#endif
}
/**
 * @brief Get the sensor FIFO watermark value as a percent
 *
 * Get the watermark value of the sensor's FIFO. Once the FIFO is filled to the watermark percent,
 * the sensor will trigger the GPIO interrupt.
 *
 * @param sensor Pointer to the sensor device
 * @param watermark_percent Pointer to the watermark variable that will be set on success
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_fifo_get_watermark(const struct device *sensor, uint8_t *watermark_percent);

static inline int z_impl_sensor_fifo_get_watermark(const struct device *sensor,
						   uint8_t *watermark_percent)
{
#ifdef CONFIG_SENSOR_STREAMING_MODE
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->get_watermark == NULL) {
		return -ENOSYS;
	}

	return api->get_watermark(sensor, watermark_percent);
#else
	return -ENOSYS;
#endif
}

/**
 * @brief Set the sensor FIFO watermark value as a percent
 *
 * Set the watermark value of the sensor's FIFO. Once the FIFO is filled to the watermark percent,
 * the sensor will trigger the GPIO interrupt.
 *
 * @param sensor Pointer to the sensor device
 * @param watermark_percent The new watermark value to set as a percent.
 * @param round_up True to round up the watermark to the nearest supported percent, false to round
 *        down.
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_fifo_set_watermark(const struct device *sensor, uint8_t watermark_percent,
					bool round_up);

static inline int z_impl_sensor_fifo_set_watermark(const struct device *sensor,
						   uint8_t watermark_percent, bool round_up)
{
#ifdef CONFIG_SENSOR_STREAMING_MODE
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->set_watermark == NULL) {
		return -ENOSYS;
	}

	return api->set_watermark(sensor, watermark_percent, round_up);
#else
	return -ENOSYS;
#endif
}

/**
 * @brief Get the streaming/normal mode of the FIFO
 *
 * Get the current FIFO mode of operation. See sensor_fifo_set_streaming_mode() for more details.
 *
 * @param sensor Pointer to the sensor device
 * @param enabled Pointer to the value which will be written on success
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_fifo_get_streaming_mode(const struct device *sensor, bool *enabled);

static inline int z_impl_sensor_fifo_get_streaming_mode(const struct device *sensor, bool *enabled)
{
#ifdef CONFIG_SENSOR_STREAMING_MODE
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->get_streaming_mode == NULL) {
		return -ENOSYS;
	}

	return api->get_streaming_mode(sensor, enabled);
#else
	return -ENOSYS;
#endif
}

/**
 * @brief Set the streaming/normal mode of the FIFO
 *
 * When streaming mode is enabled, then the oldest entries in the FIFO are removed to make room for
 * new entries. Otherwise, new samples are discarded.
 *
 * @param sensor Pointer to the sensor device
 * @param enabled Set to true to enable streaming mode, false otherwise
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_fifo_set_streaming_mode(const struct device *sensor, bool enabled);

static inline int z_impl_sensor_fifo_set_streaming_mode(const struct device *sensor, bool enabled)
{
#ifdef CONFIG_SENSOR_STREAMING_MODE
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->set_streaming_mode == NULL) {
		return -ENOSYS;
	}

	return api->set_streaming_mode(sensor, enabled);
#else
	return -ENOSYS;
#endif
}

/**
 * @brief Enable or disable calibration for the sensor
 *
 * For sensors that support an internal calibration mode, this function will start or stop that
 * mode.
 *
 * @param sensor Pointer to the sensor device
 * @param enable Whether or not to enable the calibration mode
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_perform_calibration(const struct device *sensor, bool enable);

static inline int z_impl_sensor_perform_calibration(const struct device *sensor, bool enable)
{
#ifdef CONFIG_SENSOR_HW_CALIBRATION
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->perform_calibration == NULL) {
		return -ENOSYS;
	}

	return api->perform_calibration(sensor, enabled);
#else
	return -ENOSYS;
#endif
}

/**
 * @}
 */

#include <syscalls/sensor_v2.h>

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_V2_H_ */
