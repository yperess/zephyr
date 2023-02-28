// Chirp Microsystems Firmware Header Generator
// File generated from presence_v22.hex at 2020-04-27 13:52:22.081000 by rprzybyla

#include <stdint.h>
#include "ch201.h"
#include "ch201_presence.h"

const char *ch201_presence_version = "presence_v22.hex";

#define RAM_INIT_ADDRESS 2030

#define RAM_INIT_WRITE_SIZE 14

uint16_t get_ch201_presence_fw_ram_init_addr(void)
{
	return (uint16_t)RAM_INIT_ADDRESS;
}
uint16_t get_ch201_presence_fw_ram_init_size(void)
{
	return (uint16_t)RAM_INIT_WRITE_SIZE;
}

const unsigned char ram_ch201_presence_init[RAM_INIT_WRITE_SIZE] = {
	0x00, 0x00, 0x64, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x01, 0x00,
};

const unsigned char *get_ram_ch201_presence_init_ptr(void)
{
	return &ram_ch201_presence_init[0];
}

const unsigned char ch201_presence_fw[CH101_FW_SIZE] = {
	0x0a, 0x12, 0x09, 0x12, 0x08, 0x12, 0x07, 0x12, 0x06, 0x12, 0x05, 0x12, 0x04, 0x12, 0x31,
	0x80, 0x10, 0x00, 0x81, 0x4f, 0x0a, 0x00, 0x81, 0x4e, 0x0c, 0x00, 0x05, 0x4d, 0x81, 0x4c,
	0x0e, 0x00, 0xc2, 0x93, 0x08, 0x02, 0x05, 0x20, 0x34, 0x40, 0x88, 0x13, 0x36, 0x40, 0x2c,
	0x01, 0x04, 0x3c, 0x56, 0x42, 0x08, 0x02, 0x14, 0x42, 0x16, 0x02, 0x09, 0x43, 0x0f, 0x43,
	0x81, 0x4f, 0x06, 0x00, 0x81, 0x4f, 0x04, 0x00, 0x0f, 0x4c, 0x0f, 0x93, 0x7e, 0x24, 0x81,
	0x4f, 0x02, 0x00, 0x08, 0x43, 0x47, 0x43, 0x0a, 0x43, 0x0c, 0x43, 0x1f, 0x41, 0x0c, 0x00,
	0x4f, 0x93, 0x11, 0x20, 0x08, 0x95, 0x17, 0x2c, 0x0f, 0x48, 0x0f, 0x5c, 0x0f, 0x5f, 0x3e,
	0x40, 0x28, 0x02, 0x0e, 0x5f, 0x2b, 0x4e, 0x3d, 0x40, 0xd8, 0x06, 0x0d, 0x5f, 0xae, 0x8d,
	0x00, 0x00, 0x8d, 0x4b, 0x00, 0x00, 0x08, 0x3c, 0x08, 0x95, 0x06, 0x2c, 0x0f, 0x48, 0x0f,
	0x5c, 0x0f, 0x5f, 0x9f, 0x4f, 0x28, 0x02, 0xd8, 0x06, 0x1c, 0x53, 0x2c, 0x93, 0xe0, 0x2b,
	0x1f, 0x41, 0x0c, 0x00, 0x4f, 0x93, 0x4f, 0x20, 0x0e, 0x48, 0x0e, 0x5e, 0x0a, 0x96, 0x20,
	0x20, 0x57, 0x53, 0x57, 0x93, 0x16, 0x24, 0x67, 0x93, 0x11, 0x24, 0x77, 0x90, 0x03, 0x00,
	0x0b, 0x24, 0x67, 0x92, 0x06, 0x24, 0x77, 0x90, 0x05, 0x00, 0x0f, 0x20, 0x36, 0x40, 0x2c,
	0x01, 0x0c, 0x3c, 0x5f, 0x42, 0x15, 0x02, 0x08, 0x3c, 0x5f, 0x42, 0x0d, 0x02, 0x05, 0x3c,
	0x5f, 0x42, 0x0c, 0x02, 0x02, 0x3c, 0x5f, 0x42, 0x09, 0x02, 0x06, 0x5f, 0x4f, 0x47, 0x0f,
	0x5f, 0x14, 0x4f, 0x16, 0x02, 0x3d, 0x40, 0x28, 0x02, 0x0f, 0x48, 0x1f, 0x53, 0x0f, 0x5f,
	0x0f, 0x5d, 0x81, 0x4f, 0x00, 0x00, 0x0d, 0x5e, 0x0b, 0x4d, 0x2c, 0x4d, 0x2d, 0x4f, 0xb0,
	0x12, 0x20, 0xff, 0x81, 0x93, 0x06, 0x00, 0x1a, 0x20, 0x5f, 0x42, 0x11, 0x02, 0x0f, 0x5f,
	0x0f, 0x9a, 0x15, 0x2c, 0x09, 0x93, 0x05, 0x20, 0x04, 0x9c, 0x11, 0x2c, 0x81, 0x4a, 0x08,
	0x00, 0x19, 0x43, 0x2c, 0x4b, 0x2f, 0x41, 0x2d, 0x4f, 0xb0, 0x12, 0x10, 0xfe, 0x1c, 0x91,
	0x04, 0x00, 0x03, 0x28, 0x81, 0x4c, 0x04, 0x00, 0x03, 0x3c, 0x09, 0x43, 0x91, 0x43, 0x06,
	0x00, 0x28, 0x53, 0x1a, 0x53, 0x91, 0x83, 0x02, 0x00, 0x87, 0x23, 0x1f, 0x41, 0x06, 0x00,
	0x0f, 0xd9, 0x0f, 0x93, 0x03, 0x20, 0xb2, 0x43, 0x24, 0x02, 0x09, 0x3c, 0x1c, 0x41, 0x08,
	0x00, 0xb0, 0x12, 0x10, 0xff, 0x82, 0x4c, 0x24, 0x02, 0x92, 0x41, 0x04, 0x00, 0x26, 0x02,
	0x0a, 0x43, 0x1f, 0x41, 0x0a, 0x00, 0x7f, 0xb0, 0x80, 0xff, 0x01, 0x24, 0x1a, 0x43, 0x3f,
	0x40, 0x03, 0x00, 0x1e, 0x41, 0x0a, 0x00, 0x4f, 0xfe, 0x4e, 0x4f, 0x0e, 0xda, 0x0e, 0x93,
	0x26, 0x24, 0x19, 0x43, 0x49, 0x5f, 0x81, 0x93, 0x0e, 0x00, 0x21, 0x24, 0x38, 0x40, 0x28,
	0x02, 0x07, 0x43, 0x3e, 0x40, 0x28, 0x02, 0x0f, 0x47, 0x0f, 0x5f, 0x0f, 0x5f, 0x0f, 0x5e,
	0x2c, 0x4f, 0x0f, 0x47, 0x0f, 0x5f, 0x1f, 0x53, 0x0f, 0x5f, 0x0e, 0x5f, 0x2d, 0x4e, 0x0a,
	0x93, 0x05, 0x20, 0x28, 0x53, 0x88, 0x4c, 0xfe, 0xff, 0x0c, 0x4d, 0x02, 0x3c, 0xb0, 0x12,
	0x10, 0xfe, 0x28, 0x53, 0x88, 0x4c, 0xfe, 0xff, 0x4f, 0x49, 0x07, 0x5f, 0x17, 0x91, 0x0e,
	0x00, 0xe2, 0x2b, 0x31, 0x50, 0x10, 0x00, 0x30, 0x40, 0x94, 0xff, 0x0f, 0x12, 0x0e, 0x12,
	0x0d, 0x12, 0x0c, 0x12, 0x0b, 0x12, 0x0a, 0x12, 0x3a, 0x40, 0x77, 0x01, 0x82, 0x4a, 0xa6,
	0x01, 0xd2, 0xc3, 0xee, 0x07, 0xc2, 0x93, 0x14, 0x02, 0x39, 0x20, 0x1b, 0x43, 0x1c, 0x42,
	0x3a, 0x02, 0x1d, 0x42, 0x38, 0x02, 0xb0, 0x12, 0x20, 0xff, 0x1c, 0x92, 0xf6, 0x07, 0x19,
	0x28, 0x1f, 0x42, 0x3a, 0x02, 0x0f, 0x11, 0x1f, 0x82, 0x38, 0x02, 0x1f, 0x93, 0x02, 0x38,
	0x3f, 0x43, 0x01, 0x3c, 0x1f, 0x43, 0xc2, 0x93, 0xf8, 0x07, 0x07, 0x24, 0x5e, 0x42, 0xf8,
	0x07, 0x8e, 0x11, 0x0f, 0x9e, 0x02, 0x24, 0x0b, 0x43, 0x02, 0x3c, 0x82, 0x5f, 0xf0, 0x07,
	0xc2, 0x4f, 0xf8, 0x07, 0x0f, 0x3c, 0xb2, 0x50, 0x14, 0x00, 0xf0, 0x07, 0xb2, 0x90, 0x2d,
	0x01, 0xf0, 0x07, 0x06, 0x28, 0xb2, 0x80, 0xc8, 0x00, 0xf0, 0x07, 0x12, 0xc3, 0x12, 0x10,
	0xf6, 0x07, 0xc2, 0x43, 0xf8, 0x07, 0x0b, 0x93, 0x17, 0x20, 0xd2, 0x43, 0x14, 0x02, 0xc2,
	0x43, 0xec, 0x07, 0x12, 0x3c, 0xd2, 0x93, 0x14, 0x02, 0x0d, 0x20, 0xc2, 0x93, 0xec, 0x07,
	0x0c, 0x20, 0xd2, 0x43, 0x01, 0x02, 0xe2, 0x43, 0x14, 0x02, 0xe2, 0xd3, 0xee, 0x07, 0xb2,
	0x40, 0x7e, 0x14, 0xd0, 0x01, 0x02, 0x3c, 0x82, 0x43, 0xf0, 0x01, 0xf2, 0x90, 0x03, 0x00,
	0xec, 0x07, 0x14, 0x2c, 0xf2, 0x90, 0x21, 0x00, 0x12, 0x02, 0x03, 0x28, 0xf2, 0x40, 0x20,
	0x00, 0x12, 0x02, 0x5c, 0x42, 0x07, 0x02, 0x0c, 0x5c, 0x5d, 0x42, 0x12, 0x02, 0x0d, 0x5d,
	0x0d, 0x5d, 0x5e, 0x42, 0xec, 0x07, 0x5f, 0x42, 0x13, 0x02, 0xb0, 0x12, 0x00, 0xf8, 0xe2,
	0x93, 0x14, 0x02, 0x0d, 0x28, 0xd2, 0xd3, 0xe0, 0x01, 0xd2, 0xc3, 0xe0, 0x01, 0xb2, 0x40,
	0x77, 0x06, 0xa6, 0x01, 0x3c, 0x42, 0xb0, 0x12, 0xb2, 0xff, 0x82, 0x4a, 0xa6, 0x01, 0x05,
	0x3c, 0x5c, 0x43, 0xb0, 0x12, 0x08, 0xfb, 0xa2, 0xc2, 0x92, 0x01, 0xa2, 0xd2, 0x92, 0x01,
	0xd2, 0x42, 0xea, 0x07, 0xe0, 0x01, 0xb1, 0xc0, 0xf0, 0x00, 0x0c, 0x00, 0x3a, 0x41, 0x3b,
	0x41, 0x3c, 0x41, 0x3d, 0x41, 0x3e, 0x41, 0x3f, 0x41, 0x00, 0x13, 0x0a, 0x12, 0xd2, 0xd3,
	0xee, 0x07, 0x1f, 0x42, 0xf0, 0x07, 0x3f, 0x50, 0x00, 0x10, 0x82, 0x4f, 0xf0, 0x01, 0xf2,
	0x90, 0x40, 0x00, 0x01, 0x02, 0x61, 0x24, 0xd2, 0x92, 0x07, 0x02, 0xf4, 0x07, 0x04, 0x20,
	0xd2, 0x92, 0x04, 0x02, 0xf5, 0x07, 0x3b, 0x24, 0xd2, 0x42, 0x07, 0x02, 0xf4, 0x07, 0xd2,
	0x42, 0x04, 0x02, 0xf5, 0x07, 0x5f, 0x42, 0x04, 0x02, 0x0f, 0x5f, 0x3f, 0x80, 0x0b, 0x00,
	0x5e, 0x42, 0x07, 0x02, 0x0e, 0x5e, 0x0e, 0x8f, 0x3e, 0x80, 0x0b, 0x00, 0xc2, 0x93, 0xec,
	0x07, 0x04, 0x20, 0xb2, 0x40, 0x58, 0x18, 0xd8, 0x07, 0x03, 0x3c, 0xb2, 0x40, 0x58, 0x24,
	0xd8, 0x07, 0x3a, 0x40, 0xf8, 0x2f, 0x3d, 0x40, 0xda, 0x07, 0x5b, 0x43, 0x3f, 0xb0, 0x80,
	0xff, 0x2f, 0x20, 0x2d, 0x53, 0x0f, 0x5f, 0x0f, 0x5f, 0x0f, 0x5f, 0x3f, 0x50, 0x00, 0x2c,
	0x8d, 0x4f, 0xfe, 0xff, 0x5b, 0x53, 0x3f, 0x40, 0xf8, 0x4f, 0x3e, 0xb0, 0x80, 0xff, 0x1a,
	0x20, 0x0e, 0x5e, 0x0e, 0x5e, 0x0e, 0x5e, 0x3e, 0x50, 0x00, 0x4c, 0x8d, 0x4e, 0x00, 0x00,
	0x5b, 0x53, 0xc2, 0x4b, 0xed, 0x07, 0x4c, 0x93, 0x04, 0x20, 0xb2, 0x40, 0x8a, 0x10, 0xa2,
	0x01, 0x23, 0x3c, 0xb2, 0x40, 0x8e, 0x10, 0xa2, 0x01, 0x5f, 0x42, 0x10, 0x02, 0x3f, 0x50,
	0x00, 0x38, 0x82, 0x4f, 0xa0, 0x01, 0x19, 0x3c, 0x2d, 0x53, 0x8d, 0x4f, 0xfe, 0xff, 0x5b,
	0x53, 0x3e, 0x80, 0x7f, 0x00, 0xdc, 0x3f, 0x2d, 0x53, 0x8d, 0x4a, 0xfe, 0xff, 0x5b, 0x53,
	0x3f, 0x80, 0x7f, 0x00, 0xc7, 0x3f, 0xb2, 0x40, 0x40, 0x20, 0xd8, 0x07, 0xd2, 0x43, 0xed,
	0x07, 0xb2, 0x40, 0x1e, 0x18, 0xa0, 0x01, 0xb2, 0x40, 0x86, 0x10, 0xa2, 0x01, 0x5f, 0x42,
	0xed, 0x07, 0x0f, 0x93, 0x06, 0x24, 0x3e, 0x40, 0xd8, 0x07, 0xb2, 0x4e, 0xa4, 0x01, 0x1f,
	0x83, 0xfc, 0x23, 0x92, 0x43, 0xae, 0x01, 0xa2, 0x43, 0xae, 0x01, 0x3a, 0x41, 0x30, 0x41,
	0x0a, 0x12, 0xb2, 0x40, 0x80, 0x5a, 0x20, 0x01, 0xe2, 0x42, 0xe0, 0x01, 0xd2, 0x43, 0xe2,
	0x01, 0xf2, 0x40, 0x40, 0x00, 0x01, 0x02, 0xf2, 0x40, 0x3c, 0x00, 0x07, 0x02, 0xf2, 0x40,
	0x10, 0x00, 0x04, 0x02, 0xf2, 0x40, 0x09, 0x00, 0x00, 0x02, 0xc2, 0x43, 0x13, 0x02, 0xc2,
	0x43, 0x12, 0x02, 0xd2, 0x43, 0x05, 0x02, 0xc2, 0x43, 0x11, 0x02, 0xf2, 0x40, 0x1e, 0x00,
	0x10, 0x02, 0xb2, 0x40, 0x80, 0x00, 0x02, 0x02, 0xf2, 0x40, 0x03, 0x00, 0xc2, 0x01, 0xb2,
	0x40, 0x00, 0x02, 0xa6, 0x01, 0xb2, 0x40, 0x00, 0x06, 0xa6, 0x01, 0xb2, 0x40, 0x28, 0x02,
	0xb0, 0x01, 0xb2, 0x40, 0x12, 0x00, 0xb2, 0x01, 0xb2, 0x40, 0x77, 0x01, 0xa6, 0x01, 0xb2,
	0x40, 0x80, 0x00, 0x90, 0x01, 0xb2, 0x40, 0x07, 0x00, 0x92, 0x01, 0x0a, 0x43, 0x05, 0x3c,
	0xc2, 0x93, 0xee, 0x07, 0x02, 0x24, 0x32, 0xd0, 0x18, 0x00, 0x5f, 0x42, 0x01, 0x02, 0x0a,
	0x9f, 0x20, 0x24, 0x5a, 0x42, 0x01, 0x02, 0x0f, 0x4a, 0x3f, 0x80, 0x10, 0x00, 0x18, 0x24,
	0x3f, 0x80, 0x10, 0x00, 0x15, 0x24, 0x3f, 0x80, 0x20, 0x00, 0x0d, 0x20, 0xc2, 0x43, 0x14,
	0x02, 0xe2, 0x42, 0xec, 0x07, 0x1f, 0x42, 0xf0, 0x07, 0x3f, 0x50, 0x00, 0x10, 0x82, 0x4f,
	0xf0, 0x01, 0x5c, 0x43, 0xb0, 0x12, 0x08, 0xfb, 0xe2, 0x42, 0xea, 0x07, 0xe2, 0xc3, 0xe0,
	0x01, 0x02, 0x3c, 0xe2, 0xd3, 0xe0, 0x01, 0xc2, 0x93, 0xee, 0x07, 0xd4, 0x23, 0x32, 0xd0,
	0x58, 0x00, 0xd6, 0x3f, 0x0f, 0x12, 0x5f, 0x42, 0xfb, 0x07, 0x0f, 0x93, 0x2a, 0x24, 0x1f,
	0x83, 0x3b, 0x24, 0x1f, 0x83, 0x3e, 0x20, 0xb2, 0x90, 0x22, 0x00, 0xe6, 0x07, 0x1c, 0x2c,
	0x1f, 0x42, 0xe6, 0x07, 0xdf, 0x42, 0xc1, 0x01, 0x00, 0x02, 0xb2, 0x90, 0x0f, 0x00, 0xe6,
	0x07, 0x11, 0x20, 0x1f, 0x42, 0x0e, 0x02, 0xf2, 0x40, 0x03, 0x00, 0x14, 0x02, 0x92, 0x42,
	0xf0, 0x01, 0xe8, 0x07, 0x3f, 0x50, 0x00, 0x10, 0x82, 0x4f, 0xf0, 0x01, 0xe2, 0xd3, 0xee,
	0x07, 0xb2, 0x40, 0x7e, 0x14, 0xd0, 0x01, 0x92, 0x53, 0xe6, 0x07, 0xd2, 0x83, 0xeb, 0x07,
	0x1b, 0x20, 0xc2, 0x43, 0xfb, 0x07, 0x18, 0x3c, 0x5f, 0x42, 0xc1, 0x01, 0x82, 0x4f, 0xe6,
	0x07, 0xd2, 0x43, 0xfb, 0x07, 0xd2, 0x4f, 0x00, 0x02, 0xc0, 0x01, 0x3f, 0x90, 0x06, 0x00,
	0x0c, 0x20, 0xf2, 0x40, 0x24, 0x00, 0xe0, 0x01, 0xb2, 0x40, 0x03, 0x00, 0xd8, 0x01, 0x05,
	0x3c, 0xd2, 0x42, 0xc1, 0x01, 0xeb, 0x07, 0xe2, 0x43, 0xfb, 0x07, 0xf2, 0xd0, 0x10, 0x00,
	0xc2, 0x01, 0xf2, 0xd0, 0x20, 0x00, 0xc2, 0x01, 0xb1, 0xc0, 0xf0, 0x00, 0x02, 0x00, 0x3f,
	0x41, 0x00, 0x13, 0x0f, 0x12, 0x0e, 0x12, 0x0d, 0x12, 0x0c, 0x12, 0x0b, 0x12, 0x0a, 0x12,
	0x1a, 0x42, 0x02, 0x02, 0xe2, 0x93, 0x01, 0x02, 0x1e, 0x20, 0xd2, 0x83, 0xfa, 0x07, 0x1b,
	0x20, 0x5e, 0x42, 0x05, 0x02, 0xc2, 0x4e, 0xfa, 0x07, 0x5d, 0x42, 0x07, 0x02, 0x5f, 0x42,
	0x07, 0x02, 0x0f, 0x5f, 0x0f, 0x5f, 0x0f, 0x5f, 0x0f, 0x8d, 0x6e, 0x93, 0x09, 0x28, 0x0a,
	0x9f, 0x07, 0x2c, 0x1a, 0x52, 0x02, 0x02, 0xd2, 0x83, 0xfa, 0x07, 0xe2, 0x93, 0xfa, 0x07,
	0xf7, 0x2f, 0x5c, 0x43, 0xb0, 0x12, 0x08, 0xfb, 0x09, 0x3c, 0xb2, 0x40, 0x77, 0x06, 0xa6,
	0x01, 0x3c, 0x42, 0xb0, 0x12, 0xb2, 0xff, 0xb2, 0x40, 0x77, 0x01, 0xa6, 0x01, 0x82, 0x4a,
	0x90, 0x01, 0xb1, 0xc0, 0xf0, 0x00, 0x0c, 0x00, 0x3a, 0x41, 0x3b, 0x41, 0x3c, 0x41, 0x3d,
	0x41, 0x3e, 0x41, 0x3f, 0x41, 0x00, 0x13, 0x0a, 0x12, 0x1d, 0x93, 0x03, 0x34, 0x3d, 0xe3,
	0x1d, 0x53, 0x02, 0x3c, 0x3c, 0xe3, 0x1c, 0x53, 0x0e, 0x4d, 0x0f, 0x4c, 0x0e, 0x11, 0x0f,
	0x11, 0x0b, 0x43, 0x0c, 0x4e, 0x0d, 0x4b, 0xb0, 0x12, 0xc8, 0xfe, 0x0a, 0x4c, 0x0c, 0x4f,
	0x0d, 0x4b, 0xb0, 0x12, 0xc8, 0xfe, 0x1f, 0x93, 0x03, 0x34, 0x0e, 0x8c, 0x0f, 0x5a, 0x02,
	0x3c, 0x0e, 0x5c, 0x0f, 0x8a, 0x1b, 0x53, 0x2b, 0x92, 0xed, 0x3b, 0x0c, 0x4e, 0x3a, 0x41,
	0x30, 0x41, 0x0f, 0x12, 0x0e, 0x12, 0x0d, 0x12, 0x0c, 0x12, 0x0b, 0x12, 0xe2, 0xb3, 0xe0,
	0x01, 0x12, 0x24, 0xd2, 0x42, 0xe0, 0x01, 0xea, 0x07, 0xe2, 0xc3, 0xe0, 0x01, 0xa2, 0xc2,
	0x92, 0x01, 0x4c, 0x43, 0xf2, 0x90, 0x20, 0x00, 0x01, 0x02, 0x01, 0x24, 0x5c, 0x43, 0xb0,
	0x12, 0x08, 0xfb, 0xb1, 0xc0, 0xf0, 0x00, 0x0a, 0x00, 0x3b, 0x41, 0x3c, 0x41, 0x3d, 0x41,
	0x3e, 0x41, 0x3f, 0x41, 0x00, 0x13, 0x0f, 0x12, 0xc2, 0x43, 0xfb, 0x07, 0x92, 0x53, 0xe6,
	0x07, 0xb2, 0x90, 0xd8, 0x04, 0xe6, 0x07, 0x03, 0x28, 0x82, 0x43, 0xe6, 0x07, 0x05, 0x3c,
	0x1f, 0x42, 0xe6, 0x07, 0xd2, 0x4f, 0x00, 0x02, 0xc0, 0x01, 0xf2, 0xd0, 0x20, 0x00, 0xc2,
	0x01, 0xb1, 0xc0, 0xf0, 0x00, 0x02, 0x00, 0x3f, 0x41, 0x00, 0x13, 0x3d, 0xf0, 0x0f, 0x00,
	0x3d, 0xe0, 0x0f, 0x00, 0x0d, 0x5d, 0x00, 0x5d, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c,
	0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11,
	0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x30, 0x41, 0x3d, 0xf0, 0x0f, 0x00, 0x3d,
	0xe0, 0x0f, 0x00, 0x0d, 0x5d, 0x00, 0x5d, 0x0c, 0x5c, 0x0c, 0x5c, 0x0c, 0x5c, 0x0c, 0x5c,
	0x0c, 0x5c, 0x0c, 0x5c, 0x0c, 0x5c, 0x0c, 0x5c, 0x0c, 0x5c, 0x0c, 0x5c, 0x0c, 0x5c, 0x0c,
	0x5c, 0x0c, 0x5c, 0x0c, 0x5c, 0x0c, 0x5c, 0x30, 0x41, 0x1c, 0x93, 0x02, 0x34, 0x3c, 0xe3,
	0x1c, 0x53, 0x0f, 0x4c, 0x1d, 0x93, 0x02, 0x34, 0x3d, 0xe3, 0x1d, 0x53, 0x0c, 0x4d, 0x0c,
	0x9f, 0x03, 0x2c, 0x0e, 0x4c, 0x0c, 0x4f, 0x0f, 0x4e, 0x12, 0xc3, 0x0f, 0x10, 0x0f, 0x11,
	0x0c, 0x5f, 0x30, 0x41, 0xe2, 0xc3, 0xee, 0x07, 0x92, 0x42, 0xd2, 0x01, 0x22, 0x02, 0xf2,
	0x90, 0x03, 0x00, 0x14, 0x02, 0x03, 0x20, 0x92, 0x42, 0x0e, 0x02, 0xf0, 0x07, 0xe2, 0x42,
	0x14, 0x02, 0xb1, 0xc0, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x13, 0x92, 0x42, 0xda, 0x01, 0x0a,
	0x02, 0x82, 0x43, 0xd8, 0x01, 0xe2, 0x42, 0xe0, 0x01, 0xb1, 0xc0, 0xf0, 0x00, 0x00, 0x00,
	0x00, 0x13, 0x31, 0x40, 0x00, 0x0a, 0xb0, 0x12, 0xc8, 0xff, 0x0c, 0x43, 0xb0, 0x12, 0x1a,
	0xfc, 0xb0, 0x12, 0xcc, 0xff, 0x34, 0x41, 0x35, 0x41, 0x36, 0x41, 0x37, 0x41, 0x38, 0x41,
	0x39, 0x41, 0x3a, 0x41, 0x30, 0x41, 0xb2, 0x40, 0x77, 0x13, 0xa6, 0x01, 0xb1, 0xc0, 0xf0,
	0x00, 0x00, 0x00, 0x00, 0x13, 0x1c, 0x83, 0x03, 0x43, 0xfd, 0x23, 0x30, 0x41, 0xb1, 0xc0,
	0xf0, 0x00, 0x00, 0x00, 0x00, 0x13, 0x32, 0xd0, 0x10, 0x00, 0xfd, 0x3f, 0x1c, 0x43, 0x30,
	0x41, 0x03, 0x43, 0xff, 0x3f, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x96, 0xfe, 0xf0, 0xfc, 0xd0, 0xff, 0x6c, 0xff, 0x56,
	0xfe, 0x00, 0x00, 0xc2, 0xff, 0xdc, 0xf9, 0xba, 0xff, 0xa4, 0xff, 0xc2, 0xff, 0x00, 0x00,
	0x48, 0xff, 0x94, 0xfd, 0xc2, 0xff, 0x82, 0xff,
};