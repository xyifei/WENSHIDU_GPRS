#include "relay.h"
void RelayInit(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB| RCC_APB2Periph_AFIO,ENABLE);//AFIO时钟使能
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//失能JTAG 
	DBGMCU->CR &= 0xFFFFFFDF;//不加PB3不能用
	
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_3;				 //PB4-继电器1，PB3-继电器2
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
 GPIO_Init(GPIOB, &GPIO_InitStructure);	
GPIO_ResetBits(GPIOB,GPIO_Pin_3);
GPIO_ResetBits(GPIOB,GPIO_Pin_4);	

 
}
