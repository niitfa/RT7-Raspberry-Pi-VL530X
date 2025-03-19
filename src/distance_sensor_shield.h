#ifndef DISTANCE_SENSOR_SHIELD_H
#define DISTANCE_SENSOR_SHIELD_H

#include <stdio.h>
#include "bcm2835.h"
#include "VL53L0X_user.h"
#include "moving_average.h"

typedef struct 
{
	VL53L0X_t vl53l0x;
	RPiGPIOPin gpioPowerPin;
	int raw_dist_mm;
	moving_average_t aver;
} DistSensor_t;

/**
 * @brief DistSensor_t constructor.
 * @details Call this function after DistSensor_t instance creation.
 * @param[in] DistSensor_t instance pointer.
 * @param[in] Number of previouis points for simple moving average. Default is 4.
 * @returns Returns 0 if init was successful, -1 otherwise
*/
int DistSensor_init(DistSensor_t* self, int averaging);

/**
 * @brief Enables board power, setups VL53L0X sensor
 * @param[in] DistSensor_t instance pointer.
*/
void DistSensor_enable(DistSensor_t* self);

/**
 * @brief Disables board power.
 * @param[in] DistSensor_t instance pointer.
*/
void DistSensor_disable(DistSensor_t* self);

/**
 * @brief Sends single measurement command on I2C bus, reads measured distance
 * and stores it in DistSensor_t instance
 * @details Call this function cyclically without external delays.
 * Maximum update rate is about 30 samples/sec
 * @param[in] DistSensor_t instance pointer.
*/
void DistSensor_update(DistSensor_t* self);

/**
 * @brief Gets distance measured with last call of DistSensor_update() function
 * @param[in] DistSensor_t instance pointer.
 * @returns Last received distance in mm
*/
int DistSensor_get_raw_distance_mm(DistSensor_t* self);

/**
 * @brief Gets smoothed (with simple moving average method) distance based
 * on values received with last calls of DistSensor_update() function
 * @details Smoothing level can be set in DistSensor_init() function
 * @param[in] DistSensor_t instance pointer.
 * @returns Smoothed distance in mm
*/
float DistSensor_get_smoothed_distance_mm(DistSensor_t* self);

#endif