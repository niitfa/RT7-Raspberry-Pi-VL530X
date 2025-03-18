/*
 * VL53L0X_i2c_driver.c
 *
 *  Created on: Mar 12, 2025
 *      Author: Kirill
 */

#include "VL53L0X_i2c_driver.h"

//#define STM32F4xx_I2C_DRIVER
#define RPI_I2C_DRIVER

#ifdef STM32F4xx_I2C_DRIVER
#include "stm32f4xx_hal.h"
extern I2C_HandleTypeDef hi2c3;
static const int timeout = 50;
void i2c_receive (uint8_t DevAddr, uint8_t* pData, uint16_t Size)
{
	HAL_I2C_Master_Receive(&hi2c3, DevAddr, pData, Size, timeout);
}

void i2c_transmit (uint8_t DevAddr, uint8_t* pData, uint16_t Size)
{
	HAL_I2C_Master_Transmit(&hi2c3, DevAddr, pData, Size, timeout);
}

void sleep_us(uint32_t us)
{
	int SYSTICK_LOAD = SystemCoreClock / 1000000U;
	int SYSTICK_DELAY_CALIB = SYSTICK_LOAD >> 1;
    do {
         uint32_t start = SysTick->VAL;
         uint32_t ticks = (us * SYSTICK_LOAD)-SYSTICK_DELAY_CALIB;
         while((start - SysTick->VAL) < ticks);
    } while (0);
}
#endif

#ifdef RPI_I2C_DRIVER
void i2c_receive (uint8_t DevAddr, uint8_t* pData, uint16_t Size)
{
}

void i2c_transmit (uint8_t DevAddr, uint8_t* pData, uint16_t Size)
{
}

void sleep_us(uint32_t us)
{

}
#endif





