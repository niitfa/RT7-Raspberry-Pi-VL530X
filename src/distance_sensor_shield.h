#ifndef DISTANCE_SENSOR_SHIELD_H
#define DISTANCE_SENSOR_SHIELD_H

#include <stdio.h>
#include "bcm2835.h"
#include "VL53L0X_user.h"

typedef struct 
{
	VL53L0X_t vl53l0x;
	RPiGPIOPin gpioPowerPin;
	int dist_mm;
} DistSensor_t;

int DistSensor_init(DistSensor_t* self, RPiGPIOPin gpioPowerPin);
void DistSensor_enable(DistSensor_t* self);
void DistSensor_disable(DistSensor_t* self);
void DistSensor_update(DistSensor_t* self);
int DistSensor_get_distance_mm(DistSensor_t* self);

#endif