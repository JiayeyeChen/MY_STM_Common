#ifndef __COMMON_H
#define __COMMON_H

//#define USE_STM32F1_SERIES
//#define USE_STM32F3_SERIES
//#define USE_STM32F4_SERIES
#define USE_STM32H7_SERIES

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include "my_math.h"

#ifdef USE_STM32F4_SERIES
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#endif
#ifdef USE_STM32F1_SERIES
#include "stm32f1xx_hal.h"
#endif
#ifdef USE_STM32F3_SERIES
#include "stm32f3xx_hal.h"
#endif
#ifdef USE_STM32H7_SERIES
#include "stm32h7xx_hal.h"
#endif

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))
#define MAX(x,y) ((x) > (y) ? (x) : (y))
#define MIN(x,y) ((x) > (y) ? (y) : (x))

typedef void (*FuncTypeVoidVoid)(void);

union UInt32UInt8
{
	uint32_t				b32;
	uint8_t					b8[4];
};

union Int32UInt8
{
	int32_t				  b32;
	uint8_t					b8[4];
};

union Int64UInt8
{
  int64_t         b64;
  uint8_t         b8[8];
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
	uint8_t	        b8[4];};

typedef struct
{
  uint8_t           ifStarted;
  union FloatUInt8  avg;
  float             count;
}AveragerHandle;

typedef struct
{
  float loop_duration;
  float integrateError;
  float preError;
  float curError;
  float kp, ki, kd;
}PIDHandle;
typedef struct
{
  float alpha;
  float preValue;
  union FloatUInt8  output;
}LowPassFilterHandle;
void MicroSecDelay(TIM_HandleTypeDef* htim, uint16_t us);
void PID(float* output, float mesVal, float desVal,  PIDHandle* hpid);
void Averager_Init(AveragerHandle* havg);
void Averager_Update(AveragerHandle* havg, float new_data);
void Averager_Start(AveragerHandle* havg, float initVal);
void LowPassFilter_Init(LowPassFilterHandle* hfilter, float cut_off_frequency, float duration_in_second);
void LowPassFilter_Update(LowPassFilterHandle* hfilter, float new_data);
void InverseMatrix3D(float (*m)[3], float(*output)[3]);
float DetMatrix3D(float (*m)[3]);
float DetMatrix2D(float (*m)[2]);
uint16_t CRC16_Modbus(uint8_t *buf,unsigned char Len);
#endif
