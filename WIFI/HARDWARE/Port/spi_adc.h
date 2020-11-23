#ifndef __SPI_adc_H
#define __SPI_adc_H

#include "stm32f10x.h"


#define SPI_ADC_CH0_LOW()       GPIO_ResetBits(GPIOC, GPIO_Pin_0)
#define SPI_ADC_CH0_HIGH()      GPIO_SetBits(GPIOC, GPIO_Pin_0)
#define SPI_ADC_CH1_LOW()       GPIO_ResetBits(GPIOC, GPIO_Pin_1)
#define SPI_ADC_CH1_HIGH()      GPIO_SetBits(GPIOC, GPIO_Pin_1)


void SPI_ADC_Init(void);
u16 SPI_ADC1_ReadByte(void);
u16 SPI_ADC2_ReadByte(void);
u16 SPI_ADC3_ReadByte(void);
u16 SPI_ADC4_ReadByte(void);


#endif /* __SPI_adc_H */



  
