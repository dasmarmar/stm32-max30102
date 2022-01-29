/*
 * filter.h
 *
 *  Created on: Jan 29, 2022
 *      Author: Mario Regus
 *
 *  Note: Contains code adapted from Raivis Strogonivs
 *  https://morf.lv/implementing-pulse-oximeter-using-max30100
 *  https://github.com/xcoder123/MAX30100
 */

#ifndef __FILTER_H__
#define __FILTER_H__

#include "stm32f4xx_hal.h"

/* Filter parameters */
#define ALPHA 0.95  //dc filter alpha value
#define MEAN_FILTER_SIZE        15


typedef struct{
	float w;
	float result;
}DC_FILTER_T;

typedef struct
{
  float v[2];
  float result;
}BUTTERWORTH_FILTER_T;

typedef struct
{
  float values[MEAN_FILTER_SIZE];
  uint8_t index;
  float sum;
  uint8_t count;
}MEAN_DIFF_FILTER_T;

DC_FILTER_T dcRemoval(float x, float prev_w, float alpha);
void lowPassButterworthFilter(float x, BUTTERWORTH_FILTER_T * filterResult);
float meanDiff(float M, MEAN_DIFF_FILTER_T* filterValues);


#endif /* __FILTER_H__ */
