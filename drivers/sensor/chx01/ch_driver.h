/*
 * Copyright (c) 2023 Google LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_CHX01_chdrv_H
#define ZEPHYR_DRIVERS_SENSOR_CHX01_chdrv_H

#include <zephyr/device.h>
#include <zephyr/drivers/sensor/ch101.h>

#include "ch_common.h"

/** I2C address of sensor programming interface. */
#define CH_I2C_ADDR_PROG 0x45
/** Signature byte in sensor (1 of 2). */
#define CH_SIG_BYTE_0	 (0x0a)
/** Signature byte in sensor (2 of 2). */
#define CH_SIG_BYTE_1	 (0x02)

/** maximum number of bytes in a single I2C write */
#define CH_I2C_MAX_WRITE_BYTES 256

/** Read-only register used during device discovery. */
#define CH_PROG_REG_PING 0x00
/** Data transfer starting address register address. */
#define CH_PROG_REG_ADDR 0x05
/** Data transfer value register address. */
#define CH_PROG_REG_DATA 0x06
/** Data transfer size register address. */
#define CH_PROG_REG_CNT	 0x07
/** Processor control register address. */
#define CH_PROG_REG_CPU	 0x42
/** Processor status register address. */
#define CH_PROG_REG_STAT 0x43
/** Data transfer control register address. */
#define CH_PROG_REG_CTL	 0x44

/** Macro to determine programming register size. */
#define CH_PROG_SIZEOF(R) ((R)&0x40 ? 1 : 2)

int chdrv_write_byte(const struct device *i2c, uint16_t i2c_addr, uint16_t mem_addr,
		     uint8_t data_value);

int chdrv_write_word(const struct device *i2c, uint16_t i2c_addr, uint16_t mem_addr,
		     uint16_t data_value);

int chdrv_read_word(const struct device *i2c, uint16_t i2c_addr, uint16_t mem_addr,
		    uint16_t *data_value);

int chdrv_set_idle(const struct device *i2c);

int chdrv_init_ram(const struct device *i2c, const struct ch_firmware *fw);

int chdrv_fw_load(const struct device *i2c, uint16_t prog_mem_addr, const uint8_t *fw,
		  uint16_t fw_size);
int chdrv_reset_and_halt(const struct device *i2c);

int chdrv_prog_mem_write(const struct device *i2c, uint16_t addr, const uint8_t *message,
			 uint16_t nbytes);

int chdrv_prog_write(const struct device *i2c, uint8_t reg_addr, uint16_t data);

int chdrv_burst_write(const struct device *i2c, uint16_t mem_addr, const uint8_t *data,
		      uint8_t len);

int chdrv_prog_ping(const struct device *i2c);

void chdrv_measure_rtc(const struct device *dev, enum ch_part_number part_number);

#endif /* ZEPHYR_DRIVERS_SENSOR_CHX01_chdrv_H */
