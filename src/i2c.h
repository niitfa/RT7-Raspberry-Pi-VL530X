#ifndef RGATE_I2C_H
#define RGATE_I2C_H

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

void I2C_Init(uint8_t addr);
void I2C_Read(uint8_t* buff, uint8_t len);
void I2C_Write(const uint8_t* buff, uint8_t len);

#endif