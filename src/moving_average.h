/*
 * moving_average.h
 *
 *  Created on: Feb 3, 2025
 *      Author: Kirill
 */

#ifndef SRC_UTILITY_MOVING_AVERAGE_H_
#define SRC_UTILITY_MOVING_AVERAGE_H_

#define MOVING_AVERAGE_MAX_SIZE 100

typedef struct
{
	float data[MOVING_AVERAGE_MAX_SIZE + 1];
	float average;
	int size;
	int currentIndex, prevIndex;
} moving_average_t;

int moving_average_init(moving_average_t* self, int size, float initValue);
void moving_average_add(moving_average_t* self, float val);
float moving_average_get(moving_average_t* self);

#endif /* SRC_UTILITY_MOVING_AVERAGE_H_ */
