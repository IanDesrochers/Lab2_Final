/**
  ******************************************************************************
  * @file    gpio_init.h
  * @author  Group 6
  * @version V1.0.0
  * @date    18-October-2013
  * @brief   This defines public initialization functions for the following:
	*           - USER Pushbutton
	*           - On-board LEDs
	*           - ADC1 (for temperature sensor)
	*           - Built-in temperature sensor
  */

/* Includes ------------------------------------------------------------------*/
/* GPIO Init Public Functions ---------------------------------------------------------*/

/** @defgroup GPIO_Init_Public_Functions
  * @{
  */

void init_pushbutton(void);
void init_leds(void);
void init_adc(void);
void init_temp_sensor(void);

/**
  * @}
  */
