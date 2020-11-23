#include "dac7512.h"
float Dac_val;
u16  Dac_temp;
/***********************************************************
�������ƣ�Change_flo2uint()
��    �ܣ���floatת����Dac�ɴ����uint Dac_temp
��ڲ�����(float Dac_val)
���ڲ�����uint Dac_temp
˵    ����
***********************************************************/
void Change_flo2uint(float Dac_val)
{
  Dac_temp = (u16)(Dac_val*4095/3.3);
}

/***********************************************************
�������ƣ�Dac_init()
��    �ܣ���ʼ��
��ڲ�����void
���ڲ�����void
˵    ����ͨ����ͨ��ģ�⴮�пڹ���
**********************************************************/
void Dac_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);	 
  //�¶�DAC  PB15-DIN_TEMP,PB13-SCLK_TEMP,PB12-SYNC_TEMP
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15  | GPIO_Pin_13 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_Out_OD;//
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	//ʪ��DAC  PA7-DIN_HUMI,PC4-SCLK_HUMI,PC5-SYNC_HUMI
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
�������ƣ�Dac_delay()
��    �ܣ�ʵ��ָ������ʱ
��ڲ�����void
���ڲ�����void
˵    ������ʱ8051��3��ѭ������ʱ��
***********************************************************/
void Dac_delay(void)
{
  u8 i,j;//n�������
 
    for(j=0;j<10;j++)
    for(i=0;i<72;i++);

}

/***********************************************************
�������ƣ�Dac_send_1()
��    �ܣ�CPU��DAC����1
��ڲ�����void
���ڲ�����void
˵    ����
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
�������ƣ�Dac_send_0()
��    �ܣ�CPU��DAC����0
��ڲ�����void
���ڲ�����void
˵    ����
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
�������ƣ�Dac_send_2byte()
��    �ܣ�CPU��DAC����2byte
��ڲ�����float Dac_val
���ڲ�����void
˵    ����
***********************************************************/
void Dac_send_2byte(float Dac_val)
{
  u8 i;
  Change_flo2uint(Dac_val);
 // Dac_sy_H;//��ʼ�ź�
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
  Dac_sy_H;//��ʼ�ź�
  Dac_delay();
}

/***********************************************************
�������ƣ�Dac_send_1()
��    �ܣ�CPU��DAC����1
��ڲ�����void
���ڲ�����void
˵    ����
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
�������ƣ�Dac_send_0()
��    �ܣ�CPU��DAC����0
��ڲ�����void
���ڲ�����void
˵    ����
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
�������ƣ�Dac_send_2byte()
��    �ܣ�CPU��DAC����2byte
��ڲ�����float Dac_val
���ڲ�����void
˵    ����
***********************************************************/
void Dac_send_2byte_TEMP(float Dac_val)
{
  u8 i;
  Change_flo2uint(Dac_val);
 // Dac_sy_H;//��ʼ�ź�
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
  Dac_sy_H_TEMP;//��ʼ�ź�
  Dac_delay();
}
