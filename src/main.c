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

#define PWM_SPEED 1000 																												//Time taken to go from off->fully on (and vice versa) (in ms)
#define PWM_FREQUENCY 1000000 																								//Frequency to run PWM driver at
#define PWM_MAX_INTENSITY 1000 																								//Number of PWM intensity steps (higher=smoother)
#define TEMP_SYSTICK_FREQ 20 																									//Sample frequency of internal temperature sensor
#define PWM_SYSTICK_FREQ PWM_FREQUENCY

/* Private Variables ------------------------------------------------------------------*/

uint32_t ticks = 0;
uint32_t mode = 0;

/* Private Function Definitions ------------------------------------------------------------------*/

static void set_mode(uint32_t temp_systick_freq, uint32_t pwm_systick_freq);
static void sleep(uint32_t n_sleep);

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

	while(1) {
		set_mode(TEMP_SYSTICK_FREQ, PWM_SYSTICK_FREQ); 															//Read pushbutton and set operation mode accordingly
		if (!mode) { 																																//If we're in the default (temperature mode)
			if (ticks) {
				ticks = 0; 																															//Reset interrupt flag
				read_temp(&temperature_reader);
				printf("Average: %f\n", temperature_reader.moving_average.average); 		//Print current and averaged temperature values
			}
		} else { 																																		//Otherwise, we're in the PWM mode
			if (ticks) {
				ticks = 0; 																															//Reset interrupt flag
				pwm_isr(&pwm);
			}
		}
	}
}

/**
  * @}
  */

/* Private Functions ---------------------------------------------------------*/

/** @defgroup Private_Functions
  * @{
  */

/**
  * @brief  Read USER pushbutton and set operation mode.
	* @note   Includes switch debouncing. Mode 0 = Default (temperature).
	*         Mode 1 = PWM LED control.
	* @param  temp_systick_freq: Frequency of SysTick Interrupt for Temperature Reader.
	* @param  pwm_systick_freq: Frequency of SysTick Interrupt for PWM.
  * @retval None
  */
static void set_mode(uint32_t temp_systick_freq, uint32_t pwm_systick_freq) {
	uint8_t button_state = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0); 							//Read pushbutton state
	if (button_state) { 																													//If the button is pushed
		if (mode) { 																																//If we're in mode 1 (PWM)
			mode = 0; 																																//Go to mode 0 (temperature)
			SysTick_Config(SystemCoreClock / temp_systick_freq); 											//Configure systick timer for temperature mode
		} else { 																																		//If we're in mode 0 (temperature)
			mode = 1; 																																//Go to mode 1 (PWM)
			SysTick_Config(SystemCoreClock / pwm_systick_freq); 											//Configure systick timer for PWM mode
			GPIO_Write(GPIOD, 0x0); 																									//Turn off all LEDs
		}
		sleep(250); 																																//Wait for a bit (debounce)
		while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)) {
			sleep(250); 																															//Wait a bit more, until the button is released (debounce pt 2)
		}
	}
}

/**
  * @brief  Sleeps for given time.
	* @param  n_sleep: Time to sleep for (in ms)
  * @retval None
  */
static void sleep(uint32_t n_sleep) {
	n_sleep = SystemCoreClock / 1000;																						//Set intermediate number representing clock ticks to sleep for
  while(n_sleep--) {																													//Do nothing for this many ticks
  }
}

/**
  * @brief  SysTick ISR.
	* @note   Sets flag indicating interrupt has occurred.
	* @param  None
  * @retval None
  */
void SysTick_Handler(void) {
	ticks++;																																		//Increment flag to signal interrupt
}

/**
  * @}
  */
