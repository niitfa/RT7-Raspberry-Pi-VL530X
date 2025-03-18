#include "distance_sensor_shield.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "rpiGpio.h"

static const uint8_t ADDRESS = (0x52 >> 1);

static void i2c_receive (uint8_t DevAddr, uint8_t* pData, uint16_t len);
static void i2c_transmit (uint8_t DevAddr, uint8_t* pData, uint16_t len);
static void sleep_us (uint32_t us);

int DistSensor_init(DistSensor_t* self, RPiGPIOPin gpioPowerPin)
{
	memset(self, 0, sizeof(*self));
	self->gpioPowerPin = gpioPowerPin;
	// init periphery library
	if(!bcm2835_init())
	{
		//printf("Error init bcm2835.\n");
		return -1;
	}

	// setup gpio pin
	bcm2835_gpio_fsel(gpioPowerPin, BCM2835_GPIO_FSEL_OUTP);

	// power off
	DistSensor_disable(self);
}

void DistSensor_enable(DistSensor_t* self)
{
	bcm2835_gpio_set(self->gpioPowerPin);

	usleep(200 * 1000);

	bcm2835_i2c_begin();
	bcm2835_i2c_setSlaveAddress(ADDRESS);
	bcm2835_i2c_setClockDivider(2500 * 1); // 2500 => 100 kHz, 5000 => 50 kHz

	usleep(200 * 1000);

	VL53L0X_init(&self->vl53l0x, ADDRESS);
	VL53L0X_reg_callbacks(&self->vl53l0x, i2c_receive, i2c_transmit, sleep_us);
	VL53L0X_setup(&self->vl53l0x, 0.25);//0.25);
}

void DistSensor_disable(DistSensor_t* self)
{
	bcm2835_i2c_end();
	bcm2835_gpio_clr(self->gpioPowerPin);
	//bcm2835_close();
}

void DistSensor_update(DistSensor_t* self)
{
	self->dist_mm = VL53L0X_read_single_mm(&self->vl53l0x);	
}

int DistSensor_get_distance_mm(DistSensor_t* self)
{
	return self->dist_mm;
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