/********************di8do8l ***************************
 * 文件名  ：di8do8l.c
 * 描述    ：di8do8l 初始化及控制         
*********************************************************/
#include "di8do8l.h"
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
void GPIO_Di8do8l_Configuration(void)
{

  	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB, ENABLE);

	/*D0  PA0----->DO0	   PA1----->DO1   PA2----->DO2 
		  		  		普通推挽输出 1-无输出信号 0-有输出信号*/	
		  	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*DI  PC8------>SW2	   PC6------>SW1   PB14------>SW3   
		  		  	普通推挽输入 1-无输出信号 0-有输出信号*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_8 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	  //上拉输入
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2);	//点亮小灯

}

/*******************************************************************************
* Function Name  : DI8_Readbit_Modbus
* Description    : 读取GPIO的值赋值给Modbus寄存器
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DI8_Readbit_Modbus(void)
{
     	unsigned char temp;
		temp|=(DI_DI7<<7);//高位
		temp|=(DI_DI6<<6);
		temp|=(DI_DI5<<5);
		temp|=(DI_DI4<<4);
		temp|=(DI_DI3<<3);
		temp|=(DI_DI2<<2);
		temp|=(DI_DI1<<1);
		temp|=(DI_DI0<<0); //低位
		temp=~temp;				 //由于是上拉状态 开关量取反
    ucRegDiscBuf[0]=temp;	//GPIO状态赋值给Modbus开关量输入寄存器。
} 
 
/*******************************************************************************
* Function Name  : DO8_Writebit_Modbus
* Description    : 读取Modbus寄存器的值赋值给GPIO
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DO8_Writebit_Modbus(void)
{
    unsigned char temp;
	temp=ucRegCoilsBuf[0];
	temp=~temp;	   //由于是低电平输出 开关量取反
	DO_DO0 = ((temp & (0x01<<0))>>0);
	DO_DO1 = ((temp & (0x01<<1))>>1);
	DO_DO2 = ((temp & (0x01<<2))>>2);
	DO_DO3 = ((temp & (0x01<<3))>>3);
	DO_DO4 = ((temp & (0x01<<4))>>4);
	DO_DO5 = ((temp & (0x01<<5))>>5);
	DO_DO6 = ((temp & (0x01<<6))>>6);
	DO_DO7 = ((temp & (0x01<<7))>>7);
	
} 

