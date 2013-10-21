/**
  ******************************************************************************
  * @file    lab2_pwm.c
  * @author  Group 6
  * @version V1.0.0
  * @date    18-October-2013
  * @brief   This defines a PWM function for the second operation mode for
	*          all four onboard LEDs. It should be called at a given frequency
	*          defined by PWM_FREQUENCY.
  */

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx.h"
#include "lab2_pwm.h"
#include <math.h>

/* Defines ------------------------------------------------------------------*/

#define PWM_LED_MAPPING_POWER 2

/* PWM Private Functions ---------------------------------------------------------*/

/** @defgroup PWM_Private_Functions
  * @{
  */

static void do_pwm(struct PWM *pwm) {
	
	//Set LED states
	if (pwm->pwm_count >= (uint32_t)(pwm->max_intensity*pwm->real_intensity)) {
		GPIO_Write(GPIOD, 0x0); 																											//Turn LEDs off if they have been on for specified intensity duration
	} else if (pwm->pwm_count == 0) {
		GPIO_Write(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15); 		//Turn LEDs on at the beginning of each new cycle
	}
	
	//Setting brighness direction mode (increaseing/decreasing)
	if (pwm->pwm_count >= pwm->max_intensity) {
		pwm->pwm_count = 0; 																													//At the end of each cycle, reset cycle counter
		if (pwm->intensity >= pwm->max_intensity) {
			pwm->up_down = 0; 																													//If we are at the brightest, start dimming
		} else if (pwm->intensity <= 0) {
			pwm->up_down = 1; 																													//If we are at the dimmest, start brightening
		}
	} else {
		pwm->pwm_count++; //Increment cycle counter
	}
	
	//Checking change-brighness flag
	if (!pwm->change_intensity_count) { 																						//Check if we need to change the brightness level this iteraiton
		if (pwm->up_down) {
			pwm->intensity++; 																													//Increment brightness if we're on an upwards metaphorical trajectory
		} else {
			pwm->intensity--; 																													//Otherwise, decrement
		}
		pwm->real_intensity = (pow(pwm->intensity, PWM_LED_MAPPING_POWER)/pow(pwm->max_intensity, PWM_LED_MAPPING_POWER));
	}
	
	//Setting change-brighness flag
	if (pwm->change_intensity_count == (uint32_t)(pwm->pwm_frequency * ((float)pwm->pulse_speed / 1000) / pwm->max_intensity)) {
		pwm->change_intensity_count = 0; 																							//Check if we've stayed at the curent brightness for long enough
	} else {
		pwm->change_intensity_count++; 																								//Otherwise, increment counter and check next iteration
	}
}

/**
  * @}
  */

/* PWM Public Functions ---------------------------------------------------------*/

/** @defgroup PWM_Public_Functions
  * @{
  */

/**
  * @brief  Initializes PWM struct
  * @param  *pwm: Pointer to a PWM struct
	* @param  pulse_speed: Frequency at which PWM should rise/fall (in ms)
	* @param  pwm_frequency: PWM Frequency the that pwm_isr is being called at
	* @param  max_intensity: How many intensity levels the PWM should assume (higher=smoother)
  * @retval None
  */
void init_pwm(struct PWM *pwm, uint32_t pulse_speed, uint32_t pwm_frequency, uint32_t max_intensity) {
	pwm->pulse_speed = pulse_speed;
	pwm->pwm_frequency = pwm_frequency;
	pwm->max_intensity = max_intensity;
	pwm->up_down = 1;
}

/**
  * @brief  PWM ISR
	* @note   This function needs to be called at the same frequency as pwm_frequency
  * @param  *pwm: Pointer to a PWM struct
  * @retval None
  */
void pwm_isr(struct PWM *pwm) {
	do_pwm(pwm);
}

/**
  * @}
  */
