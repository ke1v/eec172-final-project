/*
 * vl53l0x_i2c_cc3200_comms.c
 *
 *  Created on: Mar 9, 2026
 *      Author: PC
 */

#include <stdint.h>
#include <assert.h>
#include "i2c_if.h"
#include <vl53lo/core/inc/vl53l0x_def.h>
#include <vl53lo/platform/inc/vl53l0x_platform.h>

// inspiration:
// https://github.com/adafruit/Adafruit_VL53L0X/blob/master/src/platform/src/vl53l0x_i2c_comms.cpp

int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t  *pdata, int32_t count) {
    assert(count < 64);

    uint8_t buffer[64];

    buffer[0] = index;
    memcpy(&buffer[1], pdata, count);

    I2C_IF_Write(address, buffer, count + 1, 1);
    return VL53L0X_ERROR_NONE;
}

int32_t VL53L0X_read_multi(uint8_t address,  uint8_t index, uint8_t  *pdata, int32_t count)  {
    I2C_IF_Write(address, &index, 1, 0);
    I2C_IF_Read(address, pdata, count);
    return VL53L0X_ERROR_NONE;
}

int VL53L0X_write_byte(uint8_t address, uint8_t index, uint8_t data) {
    return VL53L0X_write_multi(address, index, &data, 1);
}

int VL53L0X_write_word(uint8_t address, uint8_t index, uint16_t data) {
    uint8_t buff[2];
    buff[1] = data & 0xFF;
    buff[0] = data >> 8;
    return VL53L0X_write_multi(address, index, buff, 2);
}

int VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t data) {
    uint8_t buff[4];

    buff[3] = data & 0xFF;
    buff[2] = data >> 8;
    buff[1] = data >> 16;
    buff[0] = data >> 24;

    return VL53L0X_write_multi(address, index, buff, 4);
}

int VL53L0X_read_byte(uint8_t address, uint8_t index, uint8_t *data) {
    return VL53L0X_read_multi(address, index, data, 1);
}
int VL53L0X_read_word(uint8_t address, uint8_t index, uint16_t *data) {
    uint8_t buff[2];
    int r = VL53L0X_read_multi(address, index, buff, 2);

    uint16_t tmp;
    tmp = buff[0];
    tmp <<= 8;
    tmp |= buff[1];
    *data = tmp;

    return r;
}
int VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *data) {
    uint8_t buff[4];
    int r = VL53L0X_read_multi(address, index, buff, 4);

    uint32_t tmp;
    tmp = buff[0];
    tmp <<= 8;
    tmp |= buff[1];
    tmp <<= 8;
    tmp |= buff[2];
    tmp <<= 8;
    tmp |= buff[3];

    *data = tmp;

    return r;
}
