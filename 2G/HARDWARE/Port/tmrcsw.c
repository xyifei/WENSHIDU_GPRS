/********************di8do8l ***************************
 * �ļ���  ��di8do8l.c
 * ����    ��di8do8l ��ʼ��������         
*********************************************************/
#include "tmrcsw.h"
#include <stdarg.h>
extern unsigned char ucRegDiscBuf[];      //����������Ĵ���ucRegDiscBuf
extern unsigned char ucRegCoilsBuf[];	 //����������Ĵ���ucRegCoilsBuf
    
/*******************************************************************************
* Function Name  : GPIO_Di8do8l_Configuration
* Description    : Configures the DI-DO GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Tmrcsw_Configuration(void)
{

  	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE);

	/*DI  PA3----->SW1	   PA4----->SW2   PA5----->SW3  
		  		  	��������ģʽ 1-�������ź� 0-�������ź�*/	
		  	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

}

/*******************************************************************************
* Function Name  : SW1_Readbit_Add
* Description    : ��ȡ�豸��ַ
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SW1_Readbit_Add(void)
{
    	unsigned char temp;
		temp|=(SW2_4<<7);//��λ
		temp|=(SW2_3<<6);
		temp|=(SW2_2<<5);
		temp|=(SW2_1<<4);
		temp|=(SW1_4<<3);
		temp|=(SW1_3<<2);
		temp|=(SW1_2<<1);
		temp|=(SW1_1<<0); //��λ
		temp=~temp;				 //����������״̬ ������ȡ��
    ucRegDiscBuf[0]=temp;	//GPIO״̬��ֵ��Modbus����������Ĵ�����
} 
/*******************************************************************************
* Function Name  : SW3_Readbit_Add
* Description    : ��ȡ�豸������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SW3_Readbit_Add(void)
{
     	unsigned char temp;
	//	temp|=(SW3_4<<3);
		temp|=(SW3_3<<2);
		temp|=(SW3_2<<1);
		temp|=(SW3_1<<0); //��λ
		temp=~temp;				 //����������״̬ ������ȡ��
    ucRegDiscBuf[1]=temp;	//GPIO״̬��ֵ��Modbus����������Ĵ�����
} 
 
