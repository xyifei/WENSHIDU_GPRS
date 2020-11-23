/********************di8do8l ***************************
 * �ļ���  ��di8do8l.c
 * ����    ��di8do8l ��ʼ��������         
*********************************************************/
#include "di8do8l.h"
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
void GPIO_Di8do8l_Configuration(void)
{

  	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB, ENABLE);

	/*D0  PA0----->DO0	   PA1----->DO1   PA2----->DO2 
		  		  		��ͨ������� 1-������ź� 0-������ź�*/	
		  	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*DI  PC8------>SW2	   PC6------>SW1   PB14------>SW3   
		  		  	��ͨ�������� 1-������ź� 0-������ź�*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_8 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	  //��������
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2);	//����С��

}

/*******************************************************************************
* Function Name  : DI8_Readbit_Modbus
* Description    : ��ȡGPIO��ֵ��ֵ��Modbus�Ĵ���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DI8_Readbit_Modbus(void)
{
     	unsigned char temp;
		temp|=(DI_DI7<<7);//��λ
		temp|=(DI_DI6<<6);
		temp|=(DI_DI5<<5);
		temp|=(DI_DI4<<4);
		temp|=(DI_DI3<<3);
		temp|=(DI_DI2<<2);
		temp|=(DI_DI1<<1);
		temp|=(DI_DI0<<0); //��λ
		temp=~temp;				 //����������״̬ ������ȡ��
    ucRegDiscBuf[0]=temp;	//GPIO״̬��ֵ��Modbus����������Ĵ�����
} 
 
/*******************************************************************************
* Function Name  : DO8_Writebit_Modbus
* Description    : ��ȡModbus�Ĵ�����ֵ��ֵ��GPIO
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DO8_Writebit_Modbus(void)
{
    unsigned char temp;
	temp=ucRegCoilsBuf[0];
	temp=~temp;	   //�����ǵ͵�ƽ��� ������ȡ��
	DO_DO0 = ((temp & (0x01<<0))>>0);
	DO_DO1 = ((temp & (0x01<<1))>>1);
	DO_DO2 = ((temp & (0x01<<2))>>2);
	DO_DO3 = ((temp & (0x01<<3))>>3);
	DO_DO4 = ((temp & (0x01<<4))>>4);
	DO_DO5 = ((temp & (0x01<<5))>>5);
	DO_DO6 = ((temp & (0x01<<6))>>6);
	DO_DO7 = ((temp & (0x01<<7))>>7);
	
} 

