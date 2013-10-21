/**
  ******************************************************************************
  * @file    lab2_pwm.h
  * @author  Group 6
  * @version V1.0.0
  * @date    18-October-2013
  * @brief   This defines a PWM function for the second operation mode for
	*          all four onboard LEDs. It should be called at a given frequency
	*          defined by PWM_FREQUENCY.
  */

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx.h"

/* Private Types ---------------------------------------------------------*/

/** @defgroup Structs
  * @{
  */ 

#ifndef PWM_STRUCT
#define PWM_STRUCT

struct PWM {
	uint32_t pwm_count;
	uint32_t intensity;
	uint32_t up_down;
	uint32_t change_intensity_count;
	uint32_t pulse_speed;
	uint32_t pwm_frequency;
	uint32_t max_intensity;
	float real_intensity;
};

#endif

/**
  * @}
  */

/* PWM Public Functions ---------------------------------------------------------*/

/** @defgroup PWM_Public_Functions
  * @{
  */

void init_pwm(struct PWM *pwm, uint32_t pulse_speed, uint32_t pwm_frequency, uint32_t max_intensity);
void pwm_isr(struct PWM *pwm);

/**
  * @}
  */
