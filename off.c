#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include "distance_sensor_shield.h"

#include <stdio.h>
#include <sys/time.h>
#include <time.h>

int main(int argc, char* argv[])
{
	DistSensor_init(4);
	DistSensor_disable();
	printf("VL53L0X board is disabled.\n");

	return 0;
}
