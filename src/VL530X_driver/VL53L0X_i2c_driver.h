/*
 * VL53L0X_cb.h
 *
 *  Created on: Mar 12, 2025
 *      Author: Kirill
 */

#ifndef SRC_V53L0X_VL53L0X_I2C_DRIVER_H_
#define SRC_V53L0X_VL53L0X_I2C_DRIVER_H_

#include <stdint.h>

void i2c_receive (uint8_t DevAddr, uint8_t* pData, uint16_t Size);
void i2c_transmit (uint8_t DevAddr, uint8_t* pData, uint16_t Size);
void sleep_us(uint32_t us);

#endif /* SRC_V53L0X_VL53L0X_I2C_DRIVER_H_ */
