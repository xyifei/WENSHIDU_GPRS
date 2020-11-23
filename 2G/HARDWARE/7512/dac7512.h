#ifndef __dac7512_H
#define __dac7512_H
#include "sys.h"

#ifdef __cplusplus
 extern "C" {
#endif
   
#include "stm32f10x.h"   
#include "stm32f10x_gpio.h"

#define Dac_sy_L    GPIO_ResetBits(GPIOC,GPIO_Pin_5)
#define Dac_sy_H    GPIO_SetBits(GPIOC,GPIO_Pin_5)
   
#define Dac_clk_L   GPIO_ResetBits(GPIOC,GPIO_Pin_4)
#define Dac_clk_H   GPIO_SetBits(GPIOC,GPIO_Pin_4)

#define Dac_di_L    GPIO_ResetBits(GPIOA,GPIO_Pin_7)
#define Dac_di_H    GPIO_SetBits(GPIOA,GPIO_Pin_7) 
  
   
#define Dac_sy_L_TEMP    GPIO_ResetBits(GPIOB,GPIO_Pin_12)
#define Dac_sy_H_TEMP    GPIO_SetBits(GPIOB,GPIO_Pin_12)
   
#define Dac_clk_L_TEMP   GPIO_ResetBits(GPIOB,GPIO_Pin_13)
#define Dac_clk_H_TEMP   GPIO_SetBits(GPIOB,GPIO_Pin_13)

#define Dac_di_L_TEMP    GPIO_ResetBits(GPIOB,GPIO_Pin_15)
#define Dac_di_H_TEMP    GPIO_SetBits(GPIOB,GPIO_Pin_15)    
 
void Dac_init(void);
void Dac_delay(void);
void Dac_send_1(void);
void Dac_send_0(void);
void Dac_send_2byte(float Dac_val);

void Dac_send_1_TEMP(void);
void Dac_send_0_TEMP(void);
void Dac_send_2byte_TEMP(float Dac_val);

   
   
   
#ifdef __cplusplus
}
#endif

#endif
