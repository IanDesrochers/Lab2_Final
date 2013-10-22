/**
  ******************************************************************************
  * @file    main.c
  * @author  Group 6
  * @version V1.0.0
  * @date    18-October-2013
  * @brief   Main entry point for embedded temperature sensor/LED PWM controller
  */

/* Includes ------------------------------------------------------------------*/

#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "gpio_init.h"
#include "lab2_pwm.h"
#include "lab2_temp.h"

/* Defines ------------------------------------------------------------------*/

#define DEBOUNCE_TIME 50																												//Length of time to check switch during debounce (in ms)
#define PWM_SPEED 1000 																													//Time taken to go from off->fully on (and vice versa) (in ms)
#define PWM_FREQUENCY 100000	 																									//Frequency to run PWM driver at
#define PWM_MAX_INTENSITY 200 																									//Number of PWM intensity steps (higher=smoother)
#define TEMP_SYSTICK_FREQ 20 																										//Sample frequency of internal temperature sensor
#define PWM_SYSTICK_FREQ PWM_FREQUENCY

/* Private Variables ------------------------------------------------------------------*/

uint32_t ticks = 0;
uint32_t mode = 0;
uint32_t button_count = 0;
uint32_t desired_button_count = 0;

/* Private Functions ---------------------------------------------------------*/

/** @defgroup Private_Functions
  * @{
  */

/**
  * @brief  Read USER pushbutton and set operation mode.
	* @note   Includes switch debouncing. Mode 0 = Default (temperature).
	*         Mode 1 = PWM LED control.
	* @param  delay: Length of time to require pushbutton to be pressed for before activation
	* @param  temp_systick_freq: Frequency of SysTick Interrupt for Temperature Reader.
	* @param  pwm_systick_freq: Frequency of SysTick Interrupt for PWM.
  * @retval None
  */
static void set_mode(uint32_t delay, uint32_t temp_systick_freq, uint32_t pwm_systick_freq) {
	uint8_t button_state = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
	if (button_state) {
		button_count++;
	} else {
		if (button_count >= desired_button_count) {
			if (mode) {
				mode = 0;
				SysTick_Config(SystemCoreClock / temp_systick_freq);
				desired_button_count = delay * temp_systick_freq / 1000;
			} else {
				mode = 1;
				SysTick_Config(SystemCoreClock / pwm_systick_freq);
				desired_button_count = delay * pwm_systick_freq / 1000;
				GPIO_Write(GPIOD, 0x0);
			}
		}
		button_count = 0;
	}
}

/**
  * @brief  SysTick ISR.
	* @note   Sets flag indicating interrupt has occurred.
	* @param  None
  * @retval None
  */
void SysTick_Handler(void) {
	ticks++;																																			//Increment flag to signal interrupt
}

/**
  * @}
  */

/* Public Functions ---------------------------------------------------------*/

/** @defgroup Public_Functions
  * @{
  */

/**
  * @brief  Main entry point
	* @param  None
  * @retval int: Error code
  */
int main()
{
	struct Temperature_Reader temperature_reader; 																//Define moving average filter struct
	struct PWM pwm; 																															//Define PWM struct

	SysTick_Config(SystemCoreClock / TEMP_SYSTICK_FREQ); 													//Set default systick interrupt frequency (mode 0 - temperature)
	
	init_pushbutton(); 																														//Initialize pushbutton
	init_leds(); 																																	//Initialize LEDs
	init_adc(); 																																	//Initialize ADC
	init_temp_sensor(); 																													//Initialize temperature sensor
	
	init_temp_reader(&temperature_reader, MOVING_AVERAGE_FILTER_SIZE); 						//initialize temp sensor
	init_pwm(&pwm, PWM_SPEED, PWM_FREQUENCY, PWM_MAX_INTENSITY); 									//Initialize PWM struct
	
	desired_button_count = DEBOUNCE_TIME * TEMP_SYSTICK_FREQ / 1000;

	while(1) {
		if (ticks) {																																//If the interrupt flag has been set
			ticks = 0;																																//Reset interrupt flag
			set_mode(DEBOUNCE_TIME, TEMP_SYSTICK_FREQ, PWM_SYSTICK_FREQ); 						//Read pushbutton and set operation mode accordingly
			if (!mode) { 																															//If we're in the default (temperature mode)
				read_temp(&temperature_reader);
				printf("Average: %f\n", temperature_reader.moving_average.average); 		//Print current and averaged temperature values
			} else { 																																	//Otherwise, we're in the PWM mode
				pwm_isr(&pwm);																													//Set PWM service flag
			}
		}
	}
}

/**
  * @}
  */
