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
	DistSensor_init(4);
	DistSensor_set_emulated(1);
	DistSensor_enable();


	char * filename = "dist.txt";
	FILE *fp = fopen(filename, "w+");

	struct timespec stop, start;
	clock_gettime(CLOCK_REALTIME, &start);
	int time_start_ms = ( (start.tv_sec) * 1000000000 + (start.tv_nsec) ) / 1000000;

	int frame_no = 0;
	while(1)
	{
		DistSensor_update();
		printf("Frame: %i\tDistance: %i mm\tSmoothed: %f\tst: %i\n", frame_no++,
		 DistSensor_get_raw_distance_mm(),
			DistSensor_get_smoothed_distance_mm(),
			DistSensor_get_status() );
		clock_gettime(CLOCK_REALTIME, &stop);
		int time_ms = ( (stop.tv_sec) * 1000000000 + (stop.tv_nsec) ) / 1000000;
		fprintf(fp, "%i\t%f\t%i\n",
			DistSensor_get_raw_distance_mm(),
			DistSensor_get_smoothed_distance_mm(),
			time_ms - time_start_ms);
		fflush(fp);
	}

	return 0;
}
