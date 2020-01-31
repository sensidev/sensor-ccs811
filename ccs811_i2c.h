#ifndef SENSOR_CCS811_I2C_H
#define SENSOR_CCS811_I2C_H

#include "stdint.h"

#define CCS811_I2C_MAX_BUFF_SIZE 10

void ccs811_i2c_init(void);

int8_t ccs811_i2c_read(uint8_t address, uint8_t reg, uint8_t *data, uint16_t count);

int8_t ccs811_i2c_write(uint8_t address, uint8_t reg, const uint8_t *data, uint16_t count);

void ccs811_i2c_wakeup();

void ccs811_i2c_sleep();

void ccs811_i2c_delay_ms(uint32_t delay);

#endif