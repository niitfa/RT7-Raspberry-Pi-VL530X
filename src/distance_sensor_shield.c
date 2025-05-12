#include "distance_sensor_shield.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <time.h>

static const uint8_t ADDRESS = (0x52 >> 1);

static void i2c_receive (uint8_t DevAddr, uint8_t* pData, uint16_t len);
static void i2c_transmit (uint8_t DevAddr, uint8_t* pData, uint16_t len);
static void sleep_us (uint32_t us);

int DistSensor_init(DistSensor_t* self, int averaging)
{
	memset(self, 0, sizeof(*self));
	self->gpioPowerPin = RPI_GPIO_P1_07;

	// Not emulated by default
	DistSensor_set_emulated(self, 0);
	
	if(!bcm2835_init())
	{
		return -1;
	}

	// setup gpio pin
	bcm2835_gpio_fsel(self->gpioPowerPin, BCM2835_GPIO_FSEL_OUTP);

	// init moving anerage
	moving_average_init(&self->aver, averaging, 0);

	// power off
	DistSensor_disable(self);

	return 0;
}

void DistSensor_enable(DistSensor_t* self)
{
	if (!self->emulated)
	{
		bcm2835_gpio_set(self->gpioPowerPin);
		bcm2835_i2c_begin();
		bcm2835_i2c_setSlaveAddress(ADDRESS);
		bcm2835_i2c_setClockDivider(2500 * 1); // 2500 => 100 kHz, 5000 => 50 kHz, ...
		sleep_us(200 * 1000);
		VL53L0X_init(&self->vl53l0x, ADDRESS);
		VL53L0X_reg_callbacks(&self->vl53l0x, i2c_receive, i2c_transmit, sleep_us);
		VL53L0X_setup(&self->vl53l0x, 0.25);
		sleep_us(200 * 1000);
	}
	else
	{
		srand(time(NULL)); 
	}

	self->status = DIST_SENSOR_STANDBY;
}

void DistSensor_disable(DistSensor_t* self)
{
	if (!self->emulated)
	{
		bcm2835_i2c_end();
		bcm2835_gpio_clr(self->gpioPowerPin);
	}

	self->status = DIST_SENSOR_DISABLED;
}

void DistSensor_set_emulated(DistSensor_t* self, uint8_t emul)
{
	self->emulated = emul;
}

void DistSensor_update(DistSensor_t* self)
{
	if(!self->emulated) 
	{
		// Not emulated sensor section

		// without handling bad values
		// self->raw_dist_mm = VL53L0X_read_single_mm(&self->vl53l0x);
		// with handling bad values

		int dist = VL53L0X_read_single_mm(&self->vl53l0x);
		if (dist == 8190 || dist == 8191)
		{
			self->status = DIST_SENSOR_OUT_OF_RANGE;
		}
		else if (dist == 65535)
		{
			self->status = DIST_SENSOR_RECEIVE_ERR;
		}
		else
		{
			self->status = DIST_SENSOR_OK;
			self->raw_dist_mm = dist;
		}
		moving_average_add(&self->aver, self->raw_dist_mm);
	}
	else
	{
		// Emulated sensor section

		// sine with 6 sec period, 100 mm offset, 20 mm magnitude, +-5 mm error, 30 ms sleep
		static const int SINE_PERIOD_S = 5;
		static const int SINE_MAGNITIDE_MM = 20;
		static const int SINE_OFFSET_MM = 100;
		static const int SINE_SPREAD_MM = 5;
		static const double PI = 3.1415;

		// get current time
		struct timespec ts;
		clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
		uint64_t t_ms = ( ts.tv_sec % 1000000 ) * 1000 + ts.tv_nsec / 1000000;

		// set ok status
		self->status = DIST_SENSOR_OK;

		// calc sine
		self->raw_dist_mm = (int) round (
			SINE_OFFSET_MM + 
			rand() % (2 * SINE_SPREAD_MM) -  SINE_SPREAD_MM +
			SINE_MAGNITIDE_MM * sin( 2 * PI * (double)t_ms / 1000 / SINE_PERIOD_S )
			);

		// ~30 ms sleep is near to i2c bus exchanche time
		usleep(30 * 1000);
		moving_average_add(&self->aver, self->raw_dist_mm);
	}

	
}

int DistSensor_get_raw_distance_mm(DistSensor_t* self)
{
	return self->raw_dist_mm;
}

float DistSensor_get_smoothed_distance_mm(DistSensor_t* self)
{
	return moving_average_get(&self->aver);	
}

DistSensorStatus_t DistSensor_get_status(DistSensor_t* self)
{
	return self->status;
}

static void i2c_receive (uint8_t DevAddr, uint8_t* pData, uint16_t len)
{
	bcm2835_i2c_read(pData, len);
}

static void i2c_transmit (uint8_t DevAddr, uint8_t* pData, uint16_t len)
{
	bcm2835_i2c_write(pData, len);
}

static void sleep_us (uint32_t us)
{
	usleep(us);
}