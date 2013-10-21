#include "stm32f4xx_gpio.h"

typedef struct{uint32_t GPIO_Pin; // Pin mask --these are the pins to configure
	GPIOMode_TypeDef GPIO_Mode; // The GPIO mode (in, out, alternate, analog)
	GPIOSpeed_TypeDef GPIO_Speed; // The maximum slew rate of the pin
	GPIOOType_TypeDef GPIO_OType; // Push-pull, open-drain
	GPIOPuPd_TypeDef GPIO_PuPd;
	}GPIO_InitTypeDef;
// Ex:
GPIO_InitTypeDef gpio_init_s;
GPIO_StructInit(&gpio_init_s);
gpio_init_s.GPIO_Pin = GPIO_Pin_4;// Select pin 4
gpio_init_s.GPIO_Mode = GPIO_Mode_OUT; // Set as output
gpio_init_s.GPIO_Speed = GPIO_Speed_100MHz; // Don't limit slew rate
gpio_init_s.GPIO_OType = GPIO_OType_PP; // Push-pull
gpio_init_s.GPIO_PuPd = GPIO_PuPd_NOPULL // Not input, don't pull
// Actually configure that pin
GPIO_Init(GPIOA, &gpio_init_s);
GPIO_SetBits(GPIOA, GPIO_Pin_4);
GPIO_ResetBits(GPIOA, GPIO_Pin_4);
GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET); (or Bit_RESET)
GPIO_Write(GPIOA, 0x16);