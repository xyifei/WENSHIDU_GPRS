#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK miniSTM32������
//������������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/3
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////   	 


//#define KEY0 PCin(5)   	
//#define KEY1 PAin(15)	 
//#define WK_UP  PAin(0)	 
 

#define KEY4  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9)//��ȡ����0
#define KEY3  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8)//��ȡ����1
#define KEY2  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7)//��ȡ����2 
#define KEY1  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6)
 

#define KEY4_PRES	4		//KEY4  
#define KEY3_PRES	3		//KEY3 
#define KEY2_PRES	2		//KEY2 
#define KEY1_PRES	1   //KEY1

void KEY_Init(void);//IO��ʼ��
void KEY_Scan();  	//����ɨ�躯��			
unsigned char key_driver1(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
#endif
