/*
 * moving_average.c
 *
 *  Created on: Feb 3, 2025
 *      Author: Kirill
 */

#include "moving_average.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

static int min(int x, int y)
{
	return (((x) < (y)) ? (x) : (y));
}

static int max(int x, int y)
{
	return (((x) > (y)) ? (x) : (y));
}

int moving_average_init(moving_average_t* self, int size, float initValue)
{
	memset(self, 0, sizeof(*self));
	self->size = min(size + 1, MOVING_AVERAGE_MAX_SIZE + 1);
	self->size = max(self->size, 2);
	self->currentIndex = 0;
	self->prevIndex = self->currentIndex + 1;

	int i;
	for(i = 0; i < self->size; ++i)
	{
		moving_average_add(self, initValue);
	}
	return 0;
}
void moving_average_add(moving_average_t* self, float val)
{
	self->data[self->currentIndex] = val;
	self->average = self->average + (self->data[self->currentIndex] - self->data[self->prevIndex]) / (self->size);
	self->currentIndex  = (self->currentIndex + 1) % (self->size + 1);
	self->prevIndex  = (self->prevIndex + 1) % (self->size + 1);
}

float moving_average_get(moving_average_t* self)
{
	return self->average;
}
