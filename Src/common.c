#include "common.h"

void Averager_Init(AveragerHandle* havg)
{
  havg->count = 1.0f;
  havg->avg.f = 0.0f;
  havg->ifStarted = 0;
}

void Averager_Update(AveragerHandle* havg, float new_data)
{
  if (havg->ifStarted)
  {
    havg->avg.f = ((havg->count - 1.0f)/havg->count) * havg->avg.f + (1.0f/havg->count) * new_data;
    havg->count += 1.0f;
  }
}

void Averager_Start(AveragerHandle* havg, float initVal)
{
  Averager_Init(havg);
  havg->avg.f = initVal;
  havg->ifStarted = 1;
}

void MicroSecDelay(TIM_HandleTypeDef* htim, uint16_t us)
{
	__HAL_TIM_SET_COUNTER(htim,0);
	while (__HAL_TIM_GET_COUNTER(htim) != us - 1);
}

void InverseMatrix3D(float (*m)[3], float(*output)[3])
{
  float det = DetMatrix3D(m);
  float sub[2][2];
  sub[0][0] = m[1][1];
  sub[0][1] = m[1][2];
  sub[1][0] = m[2][1];
  sub[1][1] = m[2][2];
  output[0][0] = DetMatrix2D(sub) / det;
  sub[0][0] = m[1][0];
  sub[0][1] = m[1][2];
  sub[1][0] = m[2][0];
  sub[1][1] = m[2][2];
  output[1][0] = -DetMatrix2D(sub) / det;
  sub[0][0] = m[1][0];
  sub[0][1] = m[1][1];
  sub[1][0] = m[2][0];
  sub[1][1] = m[2][1];
  output[2][0] = DetMatrix2D(sub) / det;
  sub[0][0] = m[0][1];
  sub[0][1] = m[0][2];
  sub[1][0] = m[2][1];
  sub[1][1] = m[2][2];
  output[0][1] = -DetMatrix2D(sub) / det;
  sub[0][0] = m[0][0];
  sub[0][1] = m[0][2];
  sub[1][0] = m[2][0];
  sub[1][1] = m[2][2];
  output[1][1] = DetMatrix2D(sub) / det;
  sub[0][0] = m[0][0];
  sub[0][1] = m[0][1];
  sub[1][0] = m[2][0];
  sub[1][1] = m[2][1];
  output[2][1] = -DetMatrix2D(sub) / det;
  sub[0][0] = m[0][1];
  sub[0][1] = m[0][2];
  sub[1][0] = m[1][1];
  sub[1][1] = m[1][2];
  output[0][2] = DetMatrix2D(sub) / det;
  sub[0][0] = m[0][0];
  sub[0][1] = m[0][2];
  sub[1][0] = m[1][0];
  sub[1][1] = m[1][2];
  output[1][2] = -DetMatrix2D(sub) / det;
  sub[0][0] = m[0][0];
  sub[0][1] = m[0][1];
  sub[1][0] = m[1][0];
  sub[1][1] = m[1][1];
  output[2][2] = DetMatrix2D(sub) / det;
}

float DetMatrix3D(float (*m)[3])
{
  float det;
  det = m[0][0] * m[1][1] * m[2][2] + m[0][1] * m[1][2] * m[2][0] + m[0][2] * m[1][0] * m[2][1] \
        -(m[0][2] * m[1][1] * m[2][0] + m[0][1] * m[1][0] * m[2][2] + m[0][0] * m[1][2] * m[2][1]);
  return det;
}

float DetMatrix2D(float (*m)[2])
{
  float det;
  det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
  return det;
}

void PID(float* output, float mesVal, float desVal,  PIDHandle* hpid)
{
  hpid->curError = desVal - mesVal;
  hpid->integrateError += hpid->curError * hpid->loop_duration;
  *output = hpid->kp * hpid->curError + hpid->ki * hpid->integrateError + hpid->kd * (hpid->preError - hpid->curError) / hpid->loop_duration;
  hpid->preError = hpid->curError;
}

uint16_t CRC16_Modbus(uint8_t *buf,unsigned char Len)
{
  unsigned int temp = 0xffff;
  unsigned char n,i;
  
 for( n = 0; n < Len; n++)          
 {       
     temp = buf[n] ^ temp;
     for( i = 0;i < 8;i++)            
   { 
        if(temp & 0x01)
    {
             temp = temp >> 1;
             temp = temp ^ 0xa001;
        }   
        else
    {
             temp = temp >> 1;
        }   
     }   
  }   
 return temp;                          
}

void LowPassFilter_Init(LowPassFilterHandle* hfilter, float cut_off_frequency, float duration_in_second)
{
  hfilter->alpha = cut_off_frequency * duration_in_second / (1.0f + cut_off_frequency * duration_in_second);
  hfilter->preValue = 0.0f;
  hfilter->output.f = 0.0f;
}
void LowPassFilter_Update(LowPassFilterHandle* hfilter, float new_data)
{
  hfilter->output.f = (1.0f - hfilter->alpha) * hfilter->preValue + hfilter->alpha * new_data;
  hfilter->preValue = new_data;
}
