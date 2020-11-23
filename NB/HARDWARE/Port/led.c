/******************** (C) COPYRIGHT 2012 WildFire Team ***************************
 * �ļ���  ��led.c
 * ����    ��led Ӧ�ú�����         
 * ʵ��ƽ̨��Ұ��STM32������
 * Ӳ�����ӣ�-----------------
 *          |   PD2 - LED1     |
 *          |   PD3 - LED2     |
 *          |   PD4 - LED3 	   |
            |	PD7 - LED4     |
 *           ----------------- 
 * ��汾  ��ST3.5.0
 * ����    ��wildfire team 
 * ��̳    ��http://www.amobbs.com/forum-1008-1.html
 * �Ա�    ��http://firestm32.taobao.com
**********************************************************************************/
#include "led.h"

/*
 * ��������LED_GPIO_Config
 * ����  ������LED�õ���I/O��
 * ����  ����
 * ���  ����
 */
void LED_GPIO_Config(void)
{		
	/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*����GPIOC������ʱ��*/
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD, ENABLE); 

	/*ѡ��Ҫ���Ƶ�GPIOC����*/															   
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7;	

	/*��������ģʽΪͨ���������*/
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*������������Ϊ50MHz */   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	/*���ÿ⺯������ʼ��GPIOC*/
  	GPIO_Init(GPIOD, &GPIO_InitStructure);		  

	/* �ر�����led��	*/
	GPIO_SetBits(GPIOC, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7);	 
}


/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
