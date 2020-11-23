#include "dac7512.h"
float Dac_val;
u16  Dac_temp;
/***********************************************************
函数名称：Change_flo2uint()
功    能：将float转换成Dac可处理的uint Dac_temp
入口参数：(float Dac_val)
出口参数：uint Dac_temp
说    明：
***********************************************************/
void Change_flo2uint(float Dac_val)
{
  Dac_temp = (u16)(Dac_val*4095/3.3);
}

/***********************************************************
函数名称：Dac_init()
功    能：初始化
入口参数：void
出口参数：void
说    明：通过普通口模拟串行口工作
**********************************************************/
void Dac_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);	 
  //温度DAC  PB15-DIN_TEMP,PB13-SCLK_TEMP,PB12-SYNC_TEMP
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15  | GPIO_Pin_13 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_Out_OD;//
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	//湿度DAC  PA7-DIN_HUMI,PC4-SCLK_HUMI,PC5-SYNC_HUMI
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5  | GPIO_Pin_4 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_Out_OD;//
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_PP;// GPIO_Mode_Out_OD;//
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  
  
}

/***********************************************************
函数名称：Dac_delay()
功    能：实现指定的延时
入口参数：void
出口参数：void
说    明：延时8051的3个循环语句的时间
***********************************************************/
void Dac_delay(void)
{
  u8 i,j;//n方便调试
 
    for(j=0;j<10;j++)
    for(i=0;i<72;i++);

}

/***********************************************************
函数名称：Dac_send_1()
功    能：CPU向DAC发送1
入口参数：void
出口参数：void
说    明：
***********************************************************/
void Dac_send_1(void)
{
  Dac_di_H;
  Dac_clk_L;
  Dac_delay();
  Dac_clk_H;
  Dac_delay();
  Dac_clk_L;
}

/***********************************************************
函数名称：Dac_send_0()
功    能：CPU向DAC发送0
入口参数：void
出口参数：void
说    明：
***********************************************************/
void Dac_send_0(void)
{
  Dac_di_L;
  Dac_clk_L;
  Dac_delay();
  Dac_clk_H;
  Dac_delay();
  Dac_clk_L;
}


/***********************************************************
函数名称：Dac_send_2byte()
功    能：CPU向DAC发送2byte
入口参数：float Dac_val
出口参数：void
说    明：
***********************************************************/
void Dac_send_2byte(float Dac_val)
{
  u8 i;
  Change_flo2uint(Dac_val);
 // Dac_sy_H;//起始信号
 // Dac_delay();
  Dac_sy_L;
  Dac_delay();
  
  
  for(i=0;i<16;i++)
  {
    if(Dac_temp>=0x8000)  
      Dac_send_1();
    else  
      Dac_send_0(); 
    Dac_temp = Dac_temp<<1;
  }
  Dac_sy_H;//起始信号
  Dac_delay();
}

/***********************************************************
函数名称：Dac_send_1()
功    能：CPU向DAC发送1
入口参数：void
出口参数：void
说    明：
***********************************************************/
void Dac_send_1_TEMP(void)
{
  Dac_di_H_TEMP;
  Dac_clk_L_TEMP;
  Dac_delay();
  Dac_clk_H_TEMP;
  Dac_delay();
  Dac_clk_L_TEMP;
}

/***********************************************************
函数名称：Dac_send_0()
功    能：CPU向DAC发送0
入口参数：void
出口参数：void
说    明：
***********************************************************/
void Dac_send_0_TEMP(void)
{
  Dac_di_L_TEMP;
  Dac_clk_L_TEMP;
  Dac_delay();
  Dac_clk_H_TEMP;
  Dac_delay();
  Dac_clk_L_TEMP;
}


/***********************************************************
函数名称：Dac_send_2byte()
功    能：CPU向DAC发送2byte
入口参数：float Dac_val
出口参数：void
说    明：
***********************************************************/
void Dac_send_2byte_TEMP(float Dac_val)
{
  u8 i;
  Change_flo2uint(Dac_val);
 // Dac_sy_H;//起始信号
 // Dac_delay();
  Dac_sy_L_TEMP;
  Dac_delay();
  
  
  for(i=0;i<16;i++)
  {
    if(Dac_temp>=0x8000)  
      Dac_send_1_TEMP();
    else  
      Dac_send_0_TEMP(); 
    Dac_temp = Dac_temp<<1;
  }
  Dac_sy_H_TEMP;//起始信号
  Dac_delay();
}
