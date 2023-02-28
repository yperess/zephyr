//
// Chirp Microsystems Firmware Header Generator v2.0 (Python 3.7.0)
// File generated from ccsout.hex at 2020-10-28 10:25:41.165552 by cryang
//
// Copyright @ 2020, Chirp Microsystems. All rights reserved.
//

#include <stdint.h>
#include "ch101.h"
#include "ch101_gppc.h"

const char *ch101_gppc_version = "gppc_17test1";
const char *ch101_gppc_gitsha1 = "37dead873d3d05d76c6324a52c6676e6e8418cad";

#define RAM_INIT_ADDRESS    1982
#define RAM_INIT_WRITE_SIZE 12

uint16_t get_ch101_gppc_fw_ram_init_addr(void)
{
	return (uint16_t)RAM_INIT_ADDRESS;
}
uint16_t get_ch101_gppc_fw_ram_init_size(void)
{
	return (uint16_t)RAM_INIT_WRITE_SIZE;
}

const unsigned char ram_ch101_gppc_init[RAM_INIT_WRITE_SIZE] = {
	0x00, 0x00, 0x64, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00,
};

const unsigned char *get_ram_ch101_gppc_init_ptr(void)
{
	return &ram_ch101_gppc_init[0];
}

const unsigned char ch101_gppc_fw[CH101_FW_SIZE] = {
	0x0a, 0x12, 0x09, 0x12, 0x08, 0x12, 0x07, 0x12, 0x06, 0x12, 0x05, 0x12, 0x04, 0x12, 0x31,
	0x80, 0x0c, 0x00, 0x81, 0x4c, 0x0a, 0x00, 0xc2, 0x43, 0xb0, 0x07, 0x2e, 0x42, 0x0f, 0x41,
	0x1f, 0x53, 0xcf, 0x43, 0xff, 0xff, 0x1e, 0x83, 0xfb, 0x23, 0x55, 0x42, 0x13, 0x02, 0x25,
	0x92, 0x02, 0x38, 0x35, 0x40, 0x03, 0x00, 0x81, 0x43, 0x04, 0x00, 0x04, 0x43, 0x0c, 0x93,
	0x77, 0x24, 0x36, 0x40, 0x88, 0x13, 0x07, 0x43, 0x0a, 0x43, 0x0e, 0x4a, 0x0e, 0x5e, 0x2c,
	0x41, 0x0d, 0x45, 0xb0, 0x12, 0xf4, 0xfe, 0x2b, 0x41, 0x0b, 0x8c, 0x0f, 0x4a, 0x0f, 0x5f,
	0x0f, 0x5f, 0x3d, 0x40, 0x1e, 0x02, 0x0d, 0x5f, 0x2c, 0x4d, 0x0d, 0x45, 0xb0, 0x12, 0xf4,
	0xfe, 0x0b, 0x5c, 0x81, 0x4b, 0x00, 0x00, 0x1c, 0x41, 0x02, 0x00, 0x0d, 0x45, 0xb0, 0x12,
	0xf4, 0xfe, 0x1f, 0x41, 0x02, 0x00, 0x0f, 0x8c, 0x0d, 0x4e, 0x1d, 0x53, 0x0d, 0x5d, 0x3c,
	0x40, 0x1e, 0x02, 0x0c, 0x5d, 0x2c, 0x4c, 0x0d, 0x45, 0xb0, 0x12, 0xf4, 0xfe, 0x0f, 0x5c,
	0x81, 0x4f, 0x02, 0x00, 0x0c, 0x4b, 0x0d, 0x4f, 0xb0, 0x12, 0x4c, 0xff, 0x3a, 0x90, 0x14,
	0x00, 0x02, 0x20, 0x36, 0x40, 0xc4, 0x09, 0x3a, 0x90, 0x19, 0x00, 0x02, 0x20, 0x36, 0x40,
	0x20, 0x03, 0x3a, 0x90, 0x37, 0x00, 0x02, 0x20, 0x16, 0x42, 0x0e, 0x02, 0x06, 0x9c, 0x30,
	0x2c, 0x07, 0x93, 0x07, 0x20, 0x5f, 0x42, 0x11, 0x02, 0x0f, 0x9a, 0x2a, 0x2c, 0x08, 0x4a,
	0x17, 0x43, 0x06, 0x43, 0x3f, 0x40, 0x1e, 0x02, 0x0d, 0x4e, 0x0d, 0x5d, 0x0d, 0x5f, 0x2c,
	0x4d, 0x1e, 0x53, 0x0e, 0x5e, 0x0e, 0x5f, 0x2d, 0x4e, 0xb0, 0x12, 0xa8, 0xfd, 0x17, 0x93,
	0x0c, 0x24, 0x27, 0x93, 0x17, 0x20, 0x0c, 0x99, 0x15, 0x2c, 0x81, 0x4a, 0x06, 0x00, 0x1a,
	0x41, 0x0a, 0x00, 0x07, 0x43, 0x91, 0x43, 0x04, 0x00, 0x0d, 0x3c, 0x0c, 0x94, 0x02, 0x28,
	0x04, 0x4c, 0x09, 0x3c, 0x0f, 0x4a, 0x0f, 0x88, 0x1f, 0x83, 0x81, 0x4f, 0x08, 0x00, 0x09,
	0x44, 0x12, 0xc3, 0x09, 0x10, 0x27, 0x43, 0x1a, 0x53, 0x1a, 0x91, 0x0a, 0x00, 0x8d, 0x2b,
	0x81, 0x93, 0x04, 0x00, 0x64, 0x24, 0x38, 0x90, 0x2b, 0x00, 0x09, 0x2c, 0x3f, 0x40, 0x07,
	0x00, 0x0f, 0x88, 0x3f, 0x90, 0xfd, 0xff, 0x05, 0x38, 0x3f, 0x40, 0xfc, 0xff, 0x02, 0x3c,
	0x3f, 0x40, 0xdd, 0xff, 0x81, 0x48, 0x04, 0x00, 0x15, 0x41, 0x08, 0x00, 0x0f, 0x95, 0x50,
	0x34, 0x06, 0x45, 0x06, 0x58, 0x06, 0x56, 0x3e, 0x40, 0x1e, 0x02, 0x07, 0x46, 0x17, 0x53,
	0x07, 0x57, 0x07, 0x5e, 0x0a, 0x46, 0x0a, 0x5a, 0x0a, 0x5e, 0x08, 0x45, 0x08, 0x8f, 0x2c,
	0x4a, 0x2d, 0x47, 0xb0, 0x12, 0xa8, 0xfd, 0x0c, 0x99, 0x07, 0x28, 0x26, 0x83, 0x27, 0x82,
	0x2a, 0x82, 0x15, 0x83, 0x18, 0x83, 0xf4, 0x23, 0x35, 0x3c, 0x15, 0x51, 0x04, 0x00, 0x35,
	0x93, 0x31, 0x24, 0x2c, 0x4a, 0x2d, 0x47, 0xb0, 0x12, 0xa8, 0xfd, 0x0a, 0x4c, 0x3e, 0x40,
	0x1e, 0x02, 0x0f, 0x46, 0x2f, 0x53, 0x0f, 0x5f, 0x0f, 0x5e, 0x2c, 0x4f, 0x36, 0x50, 0x03,
	0x00, 0x06, 0x56, 0x06, 0x5e, 0x2d, 0x46, 0xb0, 0x12, 0xa8, 0xfd, 0x09, 0x8a, 0x09, 0x59,
	0x0c, 0x8a, 0x3f, 0x42, 0x4e, 0x43, 0x4e, 0x5e, 0x0c, 0x99, 0x02, 0x2c, 0x09, 0x8c, 0x5e,
	0x53, 0x09, 0x59, 0x1f, 0x83, 0xf8, 0x23, 0x0c, 0x45, 0xb0, 0x12, 0x3c, 0xff, 0x0f, 0x4c,
	0x4c, 0x4e, 0x0c, 0x11, 0x0c, 0xdf, 0x82, 0x4c, 0x18, 0x02, 0x82, 0x44, 0x1a, 0x02, 0x1f,
	0x41, 0x06, 0x00, 0x4f, 0x85, 0xc2, 0x4f, 0x1c, 0x02, 0x02, 0x3c, 0xb2, 0x43, 0x18, 0x02,
	0x31, 0x50, 0x0c, 0x00, 0x34, 0x41, 0x35, 0x41, 0x36, 0x41, 0x37, 0x41, 0x38, 0x41, 0x39,
	0x41, 0x3a, 0x41, 0x30, 0x41, 0x0f, 0x12, 0x0e, 0x12, 0x0d, 0x12, 0x0c, 0x12, 0x0b, 0x12,
	0xc2, 0x93, 0x14, 0x02, 0x3a, 0x20, 0x1c, 0x42, 0x30, 0x02, 0x1d, 0x42, 0x2e, 0x02, 0xb0,
	0x12, 0x4c, 0xff, 0x1c, 0x92, 0xc6, 0x07, 0x21, 0x28, 0x1f, 0x42, 0x30, 0x02, 0x0f, 0x11,
	0x0f, 0x11, 0x1f, 0x82, 0x2e, 0x02, 0x1f, 0x93, 0x02, 0x38, 0x3f, 0x43, 0x01, 0x3c, 0x1f,
	0x43, 0x1d, 0x43, 0xc2, 0x93, 0xc8, 0x07, 0x07, 0x24, 0x5e, 0x42, 0xc8, 0x07, 0x8e, 0x11,
	0x0f, 0x9e, 0x02, 0x24, 0x0d, 0x43, 0x02, 0x3c, 0x82, 0x5f, 0xc0, 0x07, 0xc2, 0x4f, 0xc8,
	0x07, 0x0d, 0x93, 0x27, 0x20, 0xd2, 0x43, 0x14, 0x02, 0xc2, 0x43, 0xba, 0x07, 0x22, 0x3c,
	0xb2, 0x50, 0x14, 0x00, 0xc0, 0x07, 0xb2, 0x90, 0x2d, 0x01, 0xc0, 0x07, 0x06, 0x28, 0xb2,
	0x80, 0xc8, 0x00, 0xc0, 0x07, 0x12, 0xc3, 0x12, 0x10, 0xc6, 0x07, 0xc2, 0x43, 0xc8, 0x07,
	0x12, 0x3c, 0xd2, 0x93, 0x14, 0x02, 0x0d, 0x20, 0xc2, 0x93, 0xba, 0x07, 0x0c, 0x20, 0xd2,
	0x43, 0x01, 0x02, 0xe2, 0x43, 0x14, 0x02, 0xe2, 0xd3, 0xbe, 0x07, 0xb2, 0x40, 0x80, 0x10,
	0xd0, 0x01, 0x02, 0x3c, 0x82, 0x43, 0xf0, 0x01, 0xf2, 0x90, 0x03, 0x00, 0xba, 0x07, 0x07,
	0x2c, 0x5c, 0x42, 0x07, 0x02, 0x0c, 0x5c, 0x5d, 0x42, 0xba, 0x07, 0xb0, 0x12, 0x00, 0xf8,
	0xe2, 0x93, 0x14, 0x02, 0x03, 0x28, 0xb0, 0x12, 0xc8, 0xfe, 0x05, 0x3c, 0x5c, 0x43, 0xb0,
	0x12, 0xf8, 0xfa, 0xa2, 0xc2, 0x92, 0x01, 0xb1, 0xc0, 0xf0, 0x00, 0x0a, 0x00, 0x3b, 0x41,
	0x3c, 0x41, 0x3d, 0x41, 0x3e, 0x41, 0x3f, 0x41, 0x00, 0x13, 0x92, 0x42, 0xc0, 0x07, 0xf0,
	0x01, 0xd2, 0xd3, 0xbe, 0x07, 0xf2, 0x90, 0x40, 0x00, 0x01, 0x02, 0x28, 0x24, 0xd2, 0x92,
	0x07, 0x02, 0xc5, 0x07, 0x29, 0x24, 0xd2, 0x42, 0x07, 0x02, 0xc5, 0x07, 0x5f, 0x42, 0x07,
	0x02, 0x0f, 0x5f, 0x4e, 0x43, 0x3f, 0xb0, 0x80, 0xff, 0x0d, 0x24, 0x3b, 0x40, 0xf8, 0x4f,
	0x3d, 0x40, 0x96, 0x07, 0x2d, 0x53, 0x8d, 0x4b, 0xfe, 0xff, 0x5e, 0x53, 0x3f, 0x80, 0x7f,
	0x00, 0x3f, 0xb0, 0x80, 0xff, 0xf7, 0x23, 0x0f, 0x5f, 0x0f, 0x5f, 0x0f, 0x5f, 0x3f, 0x50,
	0x00, 0x4c, 0x4d, 0x4e, 0x0d, 0x5d, 0x8d, 0x4f, 0x96, 0x07, 0x5e, 0x53, 0xc2, 0x4e, 0xbb,
	0x07, 0x05, 0x3c, 0xb2, 0x40, 0x40, 0x20, 0x96, 0x07, 0xd2, 0x43, 0xbb, 0x07, 0x4c, 0x93,
	0x18, 0x24, 0xf2, 0x90, 0x40, 0x00, 0x01, 0x02, 0x0d, 0x24, 0xb2, 0x40, 0x16, 0x00, 0xa2,
	0x01, 0x5f, 0x42, 0x08, 0x02, 0x3f, 0x50, 0x00, 0x3f, 0x82, 0x4f, 0xb2, 0x07, 0xd2, 0x42,
	0x10, 0x02, 0xbc, 0x07, 0x10, 0x3c, 0xb2, 0x40, 0x1e, 0x18, 0xb2, 0x07, 0xb2, 0x40, 0x86,
	0x10, 0xa2, 0x01, 0x09, 0x3c, 0xb2, 0x40, 0x20, 0x00, 0xb2, 0x07, 0xb2, 0x40, 0x8e, 0x10,
	0xa2, 0x01, 0xb2, 0x40, 0x77, 0x06, 0xa6, 0x01, 0xf2, 0x90, 0x10, 0x00, 0x01, 0x02, 0x0a,
	0x24, 0x5f, 0x42, 0xbb, 0x07, 0x0f, 0x93, 0x06, 0x24, 0x3e, 0x40, 0x96, 0x07, 0xb2, 0x4e,
	0xa4, 0x01, 0x1f, 0x83, 0xfc, 0x23, 0x92, 0x42, 0xb2, 0x07, 0xa0, 0x01, 0x92, 0x43, 0xae,
	0x01, 0xa2, 0x43, 0xae, 0x01, 0x30, 0x41, 0x0a, 0x12, 0xb2, 0x40, 0x80, 0x5a, 0x20, 0x01,
	0xe2, 0x42, 0xe0, 0x01, 0xd2, 0x43, 0xe2, 0x01, 0xf2, 0x40, 0x40, 0x00, 0x01, 0x02, 0xf2,
	0x40, 0x3c, 0x00, 0x07, 0x02, 0xf2, 0x40, 0xfa, 0x00, 0x08, 0x02, 0xf2, 0x40, 0x03, 0x00,
	0x13, 0x02, 0xb2, 0x40, 0x46, 0x00, 0x0e, 0x02, 0xc2, 0x43, 0x00, 0x02, 0x82, 0x43, 0x04,
	0x02, 0xc2, 0x43, 0x10, 0x02, 0xf2, 0x40, 0x20, 0x00, 0x15, 0x02, 0xc2, 0x43, 0x11, 0x02,
	0xb2, 0x40, 0x00, 0x01, 0x02, 0x02, 0xf2, 0x40, 0x03, 0x00, 0xc2, 0x01, 0xb2, 0x40, 0x00,
	0x02, 0xa6, 0x01, 0xb2, 0x40, 0x00, 0x06, 0xa6, 0x01, 0xb2, 0x40, 0x1e, 0x02, 0xb0, 0x01,
	0xb2, 0x40, 0x09, 0x00, 0xb2, 0x01, 0xb2, 0x40, 0x77, 0x01, 0xa6, 0x01, 0xb2, 0x40, 0x00,
	0x01, 0x90, 0x01, 0xb2, 0x40, 0x07, 0x00, 0x92, 0x01, 0x0a, 0x43, 0x05, 0x3c, 0xc2, 0x93,
	0xbe, 0x07, 0x02, 0x24, 0x32, 0xd0, 0x18, 0x00, 0x5f, 0x42, 0x01, 0x02, 0x0a, 0x9f, 0x1d,
	0x24, 0x5a, 0x42, 0x01, 0x02, 0x0f, 0x4a, 0x3f, 0x80, 0x10, 0x00, 0x15, 0x24, 0x3f, 0x80,
	0x10, 0x00, 0x12, 0x24, 0x3f, 0x80, 0x20, 0x00, 0x0a, 0x20, 0xc2, 0x43, 0x14, 0x02, 0xe2,
	0x42, 0xba, 0x07, 0x92, 0x42, 0xc0, 0x07, 0xf0, 0x01, 0x5c, 0x43, 0xb0, 0x12, 0xf8, 0xfa,
	0xe2, 0x42, 0xb8, 0x07, 0xe2, 0xc3, 0xe0, 0x01, 0x02, 0x3c, 0xe2, 0xd3, 0xe0, 0x01, 0x32,
	0xc2, 0x03, 0x43, 0xc2, 0x93, 0xbe, 0x07, 0xd5, 0x23, 0x32, 0xd0, 0x58, 0x00, 0xd7, 0x3f,
	0x0f, 0x12, 0x0e, 0x12, 0x5f, 0x42, 0xc9, 0x07, 0x0f, 0x93, 0x28, 0x24, 0x1f, 0x83, 0x3c,
	0x24, 0x1f, 0x83, 0x3f, 0x20, 0xb2, 0x90, 0x16, 0x00, 0xb4, 0x07, 0x1a, 0x2c, 0x1f, 0x42,
	0xb4, 0x07, 0xdf, 0x42, 0xc1, 0x01, 0x00, 0x02, 0xb2, 0x90, 0x0d, 0x00, 0xb4, 0x07, 0x0f,
	0x20, 0x1f, 0x42, 0x0c, 0x02, 0xf2, 0x40, 0x03, 0x00, 0x14, 0x02, 0x92, 0x42, 0xf0, 0x01,
	0xb6, 0x07, 0x82, 0x4f, 0xf0, 0x01, 0xe2, 0xd3, 0xbe, 0x07, 0xb2, 0x40, 0x80, 0x10, 0xd0,
	0x01, 0x92, 0x53, 0xb4, 0x07, 0xd2, 0x83, 0xb9, 0x07, 0x1e, 0x20, 0xc2, 0x43, 0xc9, 0x07,
	0x1b, 0x3c, 0x5e, 0x42, 0xc1, 0x01, 0x82, 0x4e, 0xb4, 0x07, 0xd2, 0x43, 0xc9, 0x07, 0x0f,
	0x4e, 0x1f, 0x52, 0x04, 0x02, 0xd2, 0x4f, 0x00, 0x02, 0xc0, 0x01, 0x3e, 0x90, 0x06, 0x00,
	0x0c, 0x20, 0xf2, 0x40, 0x24, 0x00, 0xe0, 0x01, 0xb2, 0x40, 0x03, 0x00, 0xd8, 0x01, 0x05,
	0x3c, 0xd2, 0x42, 0xc1, 0x01, 0xb9, 0x07, 0xe2, 0x43, 0xc9, 0x07, 0xf2, 0xd0, 0x10, 0x00,
	0xc2, 0x01, 0xf2, 0xd0, 0x20, 0x00, 0xc2, 0x01, 0xb1, 0xc0, 0xf0, 0x00, 0x04, 0x00, 0x3e,
	0x41, 0x3f, 0x41, 0x00, 0x13, 0x0f, 0x12, 0x0e, 0x12, 0xc2, 0x93, 0xbc, 0x07, 0x06, 0x20,
	0xb2, 0x40, 0x0e, 0x00, 0xa2, 0x01, 0xd2, 0x43, 0xc4, 0x07, 0x03, 0x3c, 0xb2, 0x40, 0x16,
	0x00, 0xa2, 0x01, 0xd2, 0xb3, 0xbc, 0x07, 0x02, 0x20, 0x0e, 0x43, 0x02, 0x3c, 0x3e, 0x40,
	0x00, 0x30, 0x5f, 0x42, 0x15, 0x02, 0x3f, 0x50, 0x00, 0x0f, 0x0e, 0xdf, 0x82, 0x4e, 0xb2,
	0x07, 0x12, 0xc3, 0x52, 0x10, 0xbc, 0x07, 0x82, 0x4e, 0xa0, 0x01, 0xb1, 0xc0, 0xf0, 0x00,
	0x04, 0x00, 0x3e, 0x41, 0x3f, 0x41, 0x00, 0x13, 0x0a, 0x12, 0x1d, 0x93, 0x05, 0x34, 0x0e,
	0x4d, 0x3e, 0xe3, 0x1e, 0x53, 0x0f, 0x4c, 0x04, 0x3c, 0x0e, 0x4d, 0x0f, 0x4c, 0x3f, 0xe3,
	0x1f, 0x53, 0x0e, 0x11, 0x0f, 0x11, 0x0b, 0x43, 0x0c, 0x4e, 0x0d, 0x4b, 0xb0, 0x12, 0xf4,
	0xfe, 0x0a, 0x4c, 0x0c, 0x4f, 0x0d, 0x4b, 0xb0, 0x12, 0xf4, 0xfe, 0x1f, 0x93, 0x03, 0x34,
	0x0e, 0x8c, 0x0f, 0x5a, 0x02, 0x3c, 0x0e, 0x5c, 0x0f, 0x8a, 0x1b, 0x53, 0x2b, 0x92, 0xed,
	0x3b, 0x0c, 0x4e, 0x3a, 0x41, 0x30, 0x41, 0x0f, 0x12, 0x0e, 0x12, 0x0d, 0x12, 0x0c, 0x12,
	0x0b, 0x12, 0xe2, 0xb3, 0xe0, 0x01, 0x12, 0x24, 0xd2, 0x42, 0xe0, 0x01, 0xb8, 0x07, 0xe2,
	0xc3, 0xe0, 0x01, 0xa2, 0xc2, 0x92, 0x01, 0x4c, 0x43, 0xf2, 0x90, 0x20, 0x00, 0x01, 0x02,
	0x01, 0x24, 0x5c, 0x43, 0xb0, 0x12, 0xf8, 0xfa, 0xb1, 0xc0, 0xf0, 0x00, 0x0a, 0x00, 0x3b,
	0x41, 0x3c, 0x41, 0x3d, 0x41, 0x3e, 0x41, 0x3f, 0x41, 0x00, 0x13, 0x0f, 0x12, 0xc2, 0x43,
	0xc9, 0x07, 0x92, 0x53, 0xb4, 0x07, 0xb2, 0x90, 0x96, 0x05, 0xb4, 0x07, 0x03, 0x28, 0x82,
	0x43, 0xb4, 0x07, 0x07, 0x3c, 0x1f, 0x42, 0xb4, 0x07, 0x1f, 0x52, 0x04, 0x02, 0xd2, 0x4f,
	0x00, 0x02, 0xc0, 0x01, 0xf2, 0xd0, 0x20, 0x00, 0xc2, 0x01, 0xb1, 0xc0, 0xf0, 0x00, 0x02,
	0x00, 0x3f, 0x41, 0x00, 0x13, 0x0f, 0x12, 0x0e, 0x12, 0x0d, 0x12, 0x0c, 0x12, 0x0b, 0x12,
	0xb2, 0x40, 0x77, 0x06, 0xa6, 0x01, 0x3c, 0x40, 0x12, 0x00, 0xb0, 0x12, 0xc0, 0xff, 0xb2,
	0x40, 0x77, 0x01, 0xa6, 0x01, 0xb1, 0xc0, 0xf0, 0x00, 0x0a, 0x00, 0x3b, 0x41, 0x3c, 0x41,
	0x3d, 0x41, 0x3e, 0x41, 0x3f, 0x41, 0x00, 0x13, 0x0f, 0x12, 0x0e, 0x12, 0x0d, 0x12, 0x0c,
	0x12, 0x0b, 0x12, 0xb2, 0x40, 0x77, 0x01, 0xa6, 0x01, 0xc2, 0x93, 0xc4, 0x07, 0x02, 0x24,
	0xb0, 0x12, 0xc8, 0xfe, 0xc2, 0x43, 0xc4, 0x07, 0xb1, 0xc0, 0xf0, 0x00, 0x0a, 0x00, 0x3b,
	0x41, 0x3c, 0x41, 0x3d, 0x41, 0x3e, 0x41, 0x3f, 0x41, 0x00, 0x13, 0xd2, 0xc3, 0xbe, 0x07,
	0xd2, 0xd3, 0xe0, 0x01, 0xd2, 0xc3, 0xe0, 0x01, 0xb2, 0x40, 0x77, 0x06, 0xa6, 0x01, 0x3c,
	0x40, 0x12, 0x00, 0xb0, 0x12, 0xc0, 0xff, 0xb2, 0x40, 0x77, 0x01, 0xa6, 0x01, 0xa2, 0xd2,
	0x92, 0x01, 0xd2, 0x42, 0xb8, 0x07, 0xe0, 0x01, 0x30, 0x41, 0x3d, 0xf0, 0x0f, 0x00, 0x3d,
	0xe0, 0x0f, 0x00, 0x0d, 0x5d, 0x00, 0x5d, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11,
	0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c,
	0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x30, 0x41, 0x3d, 0xf0, 0x0f, 0x00, 0x3d, 0xe0,
	0x0f, 0x00, 0x0d, 0x5d, 0x00, 0x5d, 0x0c, 0x5c, 0x0c, 0x5c, 0x0c, 0x5c, 0x0c, 0x5c, 0x0c,
	0x5c, 0x0c, 0x5c, 0x0c, 0x5c, 0x0c, 0x5c, 0x0c, 0x5c, 0x0c, 0x5c, 0x0c, 0x5c, 0x0c, 0x5c,
	0x0c, 0x5c, 0x0c, 0x5c, 0x0c, 0x5c, 0x30, 0x41, 0x1c, 0x93, 0x02, 0x34, 0x3c, 0xe3, 0x1c,
	0x53, 0x1d, 0x93, 0x02, 0x34, 0x3d, 0xe3, 0x1d, 0x53, 0x0d, 0x9c, 0x05, 0x2c, 0x12, 0xc3,
	0x0d, 0x10, 0x0d, 0x11, 0x0c, 0x5d, 0x30, 0x41, 0x12, 0xc3, 0x0c, 0x10, 0x0c, 0x11, 0x0c,
	0x5d, 0x30, 0x41, 0xe2, 0xc3, 0xbe, 0x07, 0x92, 0x42, 0xd2, 0x01, 0x16, 0x02, 0xf2, 0x90,
	0x03, 0x00, 0x14, 0x02, 0x03, 0x20, 0x92, 0x42, 0x0c, 0x02, 0xc0, 0x07, 0xe2, 0x42, 0x14,
	0x02, 0xb1, 0xc0, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x13, 0x92, 0x42, 0xda, 0x01, 0x0a, 0x02,
	0x82, 0x43, 0xd8, 0x01, 0xe2, 0x42, 0xe0, 0x01, 0xb1, 0xc0, 0xf0, 0x00, 0x00, 0x00, 0x00,
	0x13, 0x31, 0x40, 0x00, 0x0a, 0xb0, 0x12, 0xce, 0xff, 0x0c, 0x43, 0xb0, 0x12, 0xd6, 0xfb,
	0xb0, 0x12, 0xd2, 0xff, 0x1c, 0x83, 0x03, 0x43, 0xfd, 0x23, 0x30, 0x41, 0x32, 0xd0, 0x10,
	0x00, 0xfd, 0x3f, 0x1c, 0x43, 0x30, 0x41, 0x03, 0x43, 0xff, 0x3f, 0x00, 0x13, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0xfe, 0xb0, 0xfc, 0xd6, 0xff, 0x98, 0xff, 0xf2,
	0xfd, 0x00, 0x00, 0xc8, 0xff, 0x12, 0xfa, 0x5a, 0xfd, 0x98, 0xfe, 0xc8, 0xff, 0x00, 0x00,
	0x74, 0xff, 0x68, 0xfe, 0xc8, 0xff, 0xae, 0xff,
};
