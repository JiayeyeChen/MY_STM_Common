#ifndef COMMON_H
#define COMMON_H

#include "main.h"

#ifdef USE_STM32F4_SERIES
#include "stm32f4xx_hal.h"
#endif
#ifdef USE_STM32F1_SERIES
#include "stm32f1xx_hal.h"
#endif

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

union UInt32UInt8
{
	uint32_t				b32;
	uint8_t					b8[4];
};

union UInt16UInt8
{
	uint16_t        b16;
	uint8_t         b8[2];
};

union Int16UInt8
{
	int16_t					b16;
	uint8_t					b8[2];
};

union FloatUInt8
{
	float	          f;
	uint8_t	        b8[4];
};









#endif
