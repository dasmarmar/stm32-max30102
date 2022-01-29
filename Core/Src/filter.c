/*
 * filter.c
 *
 *  Created on: Jan 29, 2022
 *      Author: Mario Regus
 *
 *  Note: Contains code adapted from Raivis Strogonivs
 *  https://morf.lv/implementing-pulse-oximeter-using-max30100
 *  https://github.com/xcoder123/MAX30100
 */

#include "filter.h"


DC_FILTER_T dcRemoval(float x, float prev_w, float alpha)
{
  DC_FILTER_T filtered;
  filtered.w = x + alpha * prev_w;
  filtered.result = filtered.w - prev_w;

  return filtered;
}

//Low pass butterworth filter order=1 alpha1=0.1
void lowPassButterworthFilter(float x, BUTTERWORTH_FILTER_T * filterResult)
{
	filterResult->v[0] = filterResult->v[1];

    //Fs = 100Hz and Fc = 10Hz
    filterResult->v[1] = (2.452372752527856026e-1 * x) + (0.50952544949442879485 * filterResult->v[0]);

    //Fs = 100Hz and Fc = 4Hz
    //filterResult->v[1] = (1.367287359973195227e-1 * x) + (0.72654252800536101020 * filterResult->v[0]); //Very precise butterworth filter

    filterResult->result = filterResult->v[0] + filterResult->v[1];
}

float meanDiff(float M, MEAN_DIFF_FILTER_T* filterValues)
{
  float avg = 0;

  filterValues->sum -= filterValues->values[filterValues->index];
  filterValues->values[filterValues->index] = M;
  filterValues->sum += filterValues->values[filterValues->index];

  filterValues->index++;
  filterValues->index = filterValues->index % MEAN_FILTER_SIZE;

  if(filterValues->count < MEAN_FILTER_SIZE)
    filterValues->count++;

  avg = filterValues->sum / filterValues->count;
  return avg - M;
}
