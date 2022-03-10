//
// Created by peress on 31.10.2022.
//

#ifndef ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_SENSOR_INFO_H_
#define ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_SENSOR_INFO_H_

/**
 * @addtogroup sensor_interface
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_SENSOR_INFO

struct sensor_info {
	const struct device *dev;
	const char *vendor;
	const char *model;
	const char *friendly_name;
};

#define SENSOR_INFO_INITIALIZER(_dev, _vendor, _model, _friendly_name)	\
	{								\
		.dev = _dev,						\
		.vendor = _vendor,					\
		.model = _model,					\
		.friendly_name = _friendly_name,			\
	}

#define SENSOR_INFO_DEFINE(name, ...)					\
	static const STRUCT_SECTION_ITERABLE(sensor_info, name) =	\
		SENSOR_INFO_INITIALIZER(__VA_ARGS__)

#define SENSOR_INFO_DT_NAME(node_id)					\
	_CONCAT(__sensor_info, DEVICE_DT_NAME_GET(node_id))

#define SENSOR_INFO_DT_DEFINE(node_id)					\
	SENSOR_INFO_DEFINE(SENSOR_INFO_DT_NAME(node_id),		\
			   DEVICE_DT_GET(node_id),			\
			   DT_NODE_VENDOR_OR(node_id, NULL),		\
			   DT_NODE_MODEL_OR(node_id, NULL),		\
			   DT_PROP_OR(node_id, friendly_name, NULL))	\

#else

#define SENSOR_INFO_DEFINE(name, ...)
#define SENSOR_INFO_DT_DEFINE(node_id)

#endif /* CONFIG_SENSOR_INFO */

/**
 * @brief Like DEVICE_DT_DEFINE() with sensor specifics.
 *
 * @details Defines a device which implements the sensor API. May define an
 * element in the sensor info iterable section used to enumerate all sensor
 * devices.
 *
 * @param node_id The devicetree node identifier.
 *
 * @param init_fn Name of the init function of the driver.
 *
 * @param pm_device PM device resources reference (NULL if device does not use
 * PM).
 *
 * @param data_ptr Pointer to the device's private data.
 *
 * @param cfg_ptr The address to the structure containing the configuration
 * information for this instance of the driver.
 *
 * @param level The initialization level. See SYS_INIT() for details.
 *
 * @param prio Priority within the selected initialization level. See
 * SYS_INIT() for details.
 *
 * @param api_ptr Provides an initial pointer to the API function struct used
 * by the driver. Can be NULL.
 */
#define SENSOR_DEVICE_DT_DEFINE(node_id, init_fn, pm_device,		\
				data_ptr, cfg_ptr, level, prio,		\
				api_ptr, ...)				\
	DEVICE_DT_DEFINE(node_id, init_fn, pm_device,			\
			 data_ptr, cfg_ptr, level, prio,		\
			 api_ptr, __VA_ARGS__);				\
									\
	SENSOR_INFO_DT_DEFINE(node_id);

/**
 * @brief Like SENSOR_DEVICE_DT_DEFINE() for an instance of a DT_DRV_COMPAT
 * compatible
 *
 * @param inst instance number. This is replaced by
 * <tt>DT_DRV_COMPAT(inst)</tt> in the call to SENSOR_DEVICE_DT_DEFINE().
 *
 * @param ... other parameters as expected by SENSOR_DEVICE_DT_DEFINE().
 */
#define SENSOR_DEVICE_DT_INST_DEFINE(inst, ...)				\
	SENSOR_DEVICE_DT_DEFINE(DT_DRV_INST(inst), __VA_ARGS__)

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_SENSOR_INFO_H_ */
