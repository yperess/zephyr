/*
 * Copyright (c) 2021 Leica Geosystems AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __TEST_SBS_GAUGE_H_
#define __TEST_SBS_GAUGE_H_

const struct device *get_fuel_gauge_device(void);

void test_get_sensor_value(int16_t channel);
void test_get_sensor_value_not_supp(int16_t channel);

#endif /* __TEST_SBS_GAUGE_H_ */
