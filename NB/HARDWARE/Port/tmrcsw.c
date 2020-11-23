/********************di8do8l ***************************
 * 文件名  ：di8do8l.c
 * 描述    ：di8do8l 初始化及控制         
*********************************************************/
#include "tmrcsw.h"
#include <stdarg.h>
extern unsigned char ucRegDiscBuf[];      //开关量输入寄存器ucRegDiscBuf
extern unsigned char ucRegCoilsBuf[];	 //开关量输出寄存器ucRegCoilsBuf
    
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
		  		  	上拉输入模式 1-无输入信号 0-有输入信号*/	
		  	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

}

/*******************************************************************************
* Function Name  : SW1_Readbit_Add
* Description    : 读取设备地址
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SW1_Readbit_Add(void)
{
    	unsigned char temp;
		temp|=(SW2_4<<7);//高位
		temp|=(SW2_3<<6);
		temp|=(SW2_2<<5);
		temp|=(SW2_1<<4);
		temp|=(SW1_4<<3);
		temp|=(SW1_3<<2);
		temp|=(SW1_2<<1);
		temp|=(SW1_1<<0); //低位
		temp=~temp;				 //由于是上拉状态 开关量取反
    ucRegDiscBuf[0]=temp;	//GPIO状态赋值给Modbus开关量输入寄存器。
} 
/*******************************************************************************
* Function Name  : SW3_Readbit_Add
* Description    : 读取设备波特率
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
		temp|=(SW3_1<<0); //低位
		temp=~temp;				 //由于是上拉状态 开关量取反
    ucRegDiscBuf[1]=temp;	//GPIO状态赋值给Modbus开关量输入寄存器。
} 
 
