#ifndef GPIO_DIGITAL_FILTERED_INPUT_H
#define GPIO_DIGITAL_FILTERED_INPUT_H

#include "main.h"

#ifdef USE_STM32F4_SERIES
#include "stm32f4xx_hal.h"
#endif
#ifdef USE_STM32F1_SERIES
#include "stm32f1xx_hal.h"
#endif

typedef struct
{
	GPIO_TypeDef* 	gpioPort;
	uint16_t		    gpioPin;
	uint8_t			    curRead;
	uint8_t			    preRead;
	uint8_t   		  state;
	uint32_t	    	lastDebounceTime;
}GPIOStruct;

uint8_t GPIO_Digital_Filtered_Input(GPIOStruct* hgpio, uint32_t debounce_time);









#endif
