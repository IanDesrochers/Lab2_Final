/**
  ******************************************************************************
  * @file    lab2_filter.c
  * @author  Group 6
  * @version V1.0.0
  * @date    18-October-2013
  * @brief   This defines public functions for a simple Linear Moving Average Filter:
	*           - Insert value into buffer
	*           - Calculate average of all elements in buffer
  */

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx.h"
#include "lab2_filter.h"

/* Moving Average Filter Public Functions ---------------------------------------------------------*/

/** @defgroup Moving_Average_Filter_Public_Functions
  * @{
  */

/**
  * @brief  Initializes Moving Average Filter
  * @param  *moving_average: Pointer to a Moving_Average struct
	* @param  size: Length of moving average array
  * @retval None
  */
void init_moving_average(struct Moving_Average *moving_average, uint32_t size) {
	moving_average->index = 0;
	moving_average->average = 0;
	uint32_t i;
	for (i=0; i<sizeof(moving_average->moving_values)/sizeof(moving_average->average); i++) {
		moving_average->moving_values[i] = 0;
	}
}

/**
  * @brief  Inserts a value into the Filter's array.
  * @param  *moving_average: Pointer to a Moving_Average struct
	* @param  new_value: New value to insert into filter's buffer
  * @retval None
  */
void insert_value(struct Moving_Average *moving_average, float new_value) {
	if (moving_average->index == sizeof(moving_average->moving_values)/sizeof(moving_average->average)-1) {
		moving_average->index = 0;
	} else {
		moving_average->index++;
	}
	moving_average->moving_values[moving_average->index] = new_value;
}

/**
  * @brief  Calculate's the current average value of the Filter's buffer and
	*         stores it in the struct
  * @param  *moving_average: Pointer to a Moving_Average struct
  * @retval None
  */
void calculate_average(struct Moving_Average *moving_average) {
	uint32_t i = 0;
	double sum = 0;
	for (i=0; i<sizeof(moving_average->moving_values)/sizeof(moving_average->average); i++) {
		sum += moving_average->moving_values[i];
	}
	moving_average->average = sum/(sizeof(moving_average->moving_values)/sizeof(moving_average->average));
}

/**
  * @}
  */
