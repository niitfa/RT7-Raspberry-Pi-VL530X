#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include "bcm2835.h"
#include "VL53L0X_user.h"
#include "distance_sensor_shield.h"

#include <stdio.h>
#include <sys/time.h>
#include <time.h>


int main(int argc, char* argv[])
{
	DistSensor_t sensor;

	DistSensor_init(&sensor, RPI_GPIO_P1_07);
	DistSensor_enable(&sensor);
	usleep(200 * 1000);

	char * filename = "dist.txt";
	FILE *fp = fopen(filename, "w+");

	struct timespec stop, start;
	clock_gettime(CLOCK_REALTIME, &start);
	int time_start_ms = ( (start.tv_sec) * 1000000000 + (start.tv_nsec) ) / 1000000;

	int frame_no = 0;
	while(1)
	{
		DistSensor_update(&sensor);
		printf("Frame: %i\tDistanse: %i mm\n", frame_no++, DistSensor_get_distance_mm(&sensor) );

		clock_gettime(CLOCK_REALTIME, &stop);
		int time_ms = ( (stop.tv_sec) * 1000000000 + (stop.tv_nsec) ) / 1000000;

		fprintf(fp, "%i\t%i\n", DistSensor_get_distance_mm(&sensor), time_ms - time_start_ms);
		fflush(fp);
	}


	/*while(1)
	{
		DistSensor_enable(&sensor);

		usleep(200 * 1000);
		DistSensor_disable(&sensor);
		usleep(200 * 1000);
	} */


	/*if(!bcm2835_init())
	{
		printf("Error init bcm2835.\n");
		exit(EXIT_FAILURE);
	}
	
	bcm2835_gpio_fsel(PIN, BCM2835_GPIO_FSEL_OUTP);
	
	while(1)
	{
		//bcm2835_gpio_write(PIN, HIGH);
		bcm2835_gpio_set(PIN);
		printf("GPIO pin set HIGH.\n");
		usleep(200 * 1000);
		//bcm2835_gpio_write(PIN, LOW);
		bcm2835_gpio_clr(PIN);
		printf("GPIO pin set LOW.\n");
		usleep(200 * 1000);
	}
	
	//VL53L0X_t sensor;
	//VL53L0X_init(&sensor, 0x52); */

	return 0;
}
