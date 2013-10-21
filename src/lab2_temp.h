/**
  ******************************************************************************
  * @file    lab2_temp.h
  * @author  Group 6
  * @version V1.0.0
  * @date    18-October-2013
  * @brief   This defines a temperature reading function using the built-in
	           temperature sensor and produces a display using the four
						 on-board LEDs
  */

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx.h"
#include "lab2_filter.h"

/* Exported Types ---------------------------------------------------------*/

/** @defgroup Structs
  * @{
  */ 

#ifndef TEMPERATURE_READER_STRUCT
#define TEMPERATURE_READER_STRUCT

struct Temperature_Reader {
	struct Moving_Average moving_average;
};

#endif

/**
  * @}
  */

/* Temperature Reader Public Functions ---------------------------------------------------------*/

/** @defgroup Temperature_Reader_Public_Functions
  * @{
  */ 

void init_temp_reader(struct Temperature_Reader *temperature_reader, uint32_t size);
void read_temp(struct Temperature_Reader *temperature_reader);

/**
  * @}
  */
