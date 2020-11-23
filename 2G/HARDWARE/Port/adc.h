#ifndef __ADC_H
#define __ADC_H	
#include "stm32f10x_adc.h"
void  ADC_INIT(void);
uint16_t Get_Adc(uint8_t ch);
uint16_t Get_Adc_Average(uint8_t ch,uint8_t count); 

#endif




