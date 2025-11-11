#ifndef DISTANCE_SENSOR_SHIELD_H
#define DISTANCE_SENSOR_SHIELD_H

#include <stdio.h>
#include "VL53L0X_user.h"
#include "moving_average.h"

//#define RGATE_RASPBERRY_PI_4
#ifdef RGATE_RASPBERRY_PI_4
#include "bcm2835.h"
#endif

#define RGATE_ORANGE_PI_5_PLUS
#ifdef RGATE_ORANGE_PI_5_PLUS
#include <wiringPi.h>
#include "i2c.h"
#endif


typedef enum
{
	DIST_SENSOR_OK,
	DIST_SENSOR_DISABLED,
	DIST_SENSOR_STANDBY,
	DIST_SENSOR_OUT_OF_RANGE,
	DIST_SENSOR_RECEIVE_ERR,
} DistSensorStatus_t;

/**
 * @brief DistSensor_t constructor.
 * @details Call this function after DistSensor_t instance creation.
 * @returns Returns 0 if init was successful, -1 otherwise
*/
int DistSensor_init(int averaging);

/**
 * @brief Enables board power, setups VL53L0X sensor
*/
void DistSensor_enable();

/**
 * @brief Disables board power.
*/
void DistSensor_disable();

/**
 * @brief Enables / disables emulation
 * @param[in] emul Emulaton flag.
*/
void DistSensor_set_emulated(uint8_t emul);

/**
 * @brief Sends single measurement command on I2C bus, reads measured distance
 * and stores it in DistSensor_t instance
 * @details Call this function cyclically without external delays.
 * Maximum update rate is about 30 samples/sec
*/
void DistSensor_update();

/**
 * @brief Gets distance measured with last call of DistSensor_update() function
 * @returns Last received distance in mm
*/
int DistSensor_get_raw_distance_mm();

/**
 * @brief Gets smoothed (with simple moving average method) distance based
 * on values received with last calls of DistSensor_update() function
 * @details Smoothing level can be set in DistSensor_init() function
 * @returns Smoothed distance in mm
*/
float DistSensor_get_smoothed_distance_mm();

/**
 * @brief Gets sensor status received with last call of DistSensor_update() function
 * @returns Sensor status
 * @see DistSensorStatus_t enumeration
*/
DistSensorStatus_t DistSensor_get_status();

#endif
