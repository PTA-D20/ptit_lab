#ifndef __ADC_H__
#define __CORE_H__

#include "basic.h"
C_DECLS_BEGIN

/**********************************************************************/
void ADC1_begin();
void ADC1_temp_sensor_begin();

uint32_t ADC1_read12(int channel, int sample_time);
uint32_t ADC1_read16(int channel, int sample_time);

int32_t ADC1_read_TempSensor(int sample_time);
int32_t ADC1_read_Vrefint(int sample_time);

int32_t ADC1_convert_temperature(int32_t temp, int32_t vref);
int32_t ADC1_convert_vcc(int32_t vref);

/**********************************************************************/
#ifdef STM32F10X_MD
  #define ADC_SampleTime_1_5Cycles       ADC_SampleTime_1Cycles5
  #define ADC_SampleTime_7_5Cycles       ADC_SampleTime_7Cycles5
  #define ADC_SampleTime_13_5Cycles      ADC_SampleTime_13Cycles5
  #define ADC_SampleTime_28_5Cycles      ADC_SampleTime_28Cycles5
  #define ADC_SampleTime_41_5Cycles      ADC_SampleTime_41Cycles5
  #define ADC_SampleTime_55_5Cycles      ADC_SampleTime_55Cycles5
  #define ADC_SampleTime_71_5Cycles      ADC_SampleTime_71Cycles5
  #define ADC_SampleTime_239_5Cycles     ADC_SampleTime_239Cycles5
#endif

/**********************************************************************/
C_DECLS_END
#endif // __ADC_H__

