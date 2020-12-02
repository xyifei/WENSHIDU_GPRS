#include "timer.h"
#include "led.h"
#include "lcd.h"
#include "rtc.h"
#include "sht2x.h"
#include "rtc.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//Mini STM32������
//ͨ�ö�ʱ�� ��������			   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2010/12/03
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  
 

//unsigned char count=0;
u16  flag_1s_nnn;
u16 timetick = 0;
extern u8 flag_fasong;
extern u8  flag_1s;
extern u32 flag_ls_set;
extern u32 flag_ls_set_dangqian;


extern u16 LCD_dizhi;
extern u16 LCD_xindao;
extern u16 LCD_botelv;
extern u16 LCD_yicunchu;
extern u16 LCD_ID;
extern u16 LCD_moshi;

u16 count=0;
u16 count_readsht=0;
u16 TurnOffCount;
extern int Timing;
extern int Timing_Enable;
extern int OnlyDispalyTiming;
extern int TimingReadSensor;
extern int StorgeTiming;
int i_count=0;
extern float V_5V,V_BAT,V_24V;
extern u16 ADC_5V,ADC_BAT,ADC_24V;
extern u16 year;
extern unsigned char month;
extern unsigned char date;
extern unsigned char hour;
extern unsigned char min;
extern unsigned char sec;
extern int RecordTiming;
//ͨ�ö�ʱ���жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ��3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��

	TIM_TimeBaseStructure.TIM_Period = arr; //arr//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //psc//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(  //ʹ�ܻ���ʧ��ָ����TIM�ж�
		TIM3, //TIM2
		TIM_IT_Update ,
		ENABLE  //ʹ��
		);
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ���ȼ�0��
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //�����ȼ�1��
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //�����ȼ�1��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx����
							 
}

void TIM2_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //ʱ��ʹ��

	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(  //ʹ�ܻ���ʧ��ָ����TIM�ж�
		TIM2, //TIM2
		TIM_IT_Update ,
		ENABLE  //ʹ��
		);
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2�ж�
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�1��
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //�����ȼ�1��
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�1��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�1��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	//TIM_Cmd(TIM2, ENABLE);  //ʹ��TIMx����
	TIM_Cmd(TIM2, DISABLE);
							 
}

void TIM5_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); //ʱ��ʹ��

	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(  //ʹ�ܻ���ʧ��ָ����TIM�ж�
		TIM5, //TIM2
		TIM_IT_Update ,
		ENABLE  //ʹ��
		);
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;  //TIM2�ж�
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�1��
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //�����ȼ�1��
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�1��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�1��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	//TIM_Cmd(TIM2, ENABLE);  //ʹ��TIMx����
	TIM_Cmd(TIM5, DISABLE);
							 
}

void TIM3_IRQHandler(void)   //TIM3�ж�
{
	
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
		{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ 
		if(Timing_Enable==1)//��Ļ�޶������غͼ�����ʱ���������
		{
				Timing++;
				OnlyDispalyTiming++;
		}
		TimingReadSensor++;
		if(TimingReadSensor>5)//��ֹTimingReadSensorһֱ����
		{
		TimingReadSensor=0;
		}
		i_count++;
		if(i_count>60)
		{
				StorgeTiming++;
				i_count=0;
				if(StorgeTiming>600)
				{
					StorgeTiming=0;
				}
		}
		//LED0=!LED0;
		}
	
}


void TIM2_IRQHandler(void) //����Ϊ1s��һ���ж�
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		count++;
		LED1=!LED1;
		
		timetick++;
		if(timetick >= flag_ls_set_dangqian*60)
		{
			flag_fasong = 1;
			flag_1s_nnn  = 0;
			timetick = 0;
		}
		
		if(flag_fasong == 1)
		{
			flag_1s_nnn++;
			if(flag_1s_nnn>=5)
			{
				flag_1s_nnn=0;
				flag_1s=1;
			}
		}
	}		
}


void TIM5_IRQHandler(void) //����Ϊ0.1s��һ���ж�
{

	
	

	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
		{
			TIM_ClearITPendingBit(TIM5, TIM_IT_Update);

			TurnOffCount++;
			//GPIO_SetBits(GPIOC,GPIO_Pin_10);
			if(TurnOffCount>=30)
				GPIO_ResetBits(GPIOC,GPIO_Pin_10);
		
			
			//DisplayTime();
			


		}		
}

void TIM4_init(void)
{
  
	
	NVIC_InitTypeDef NVIC_InitStructure;
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	
	
 //Time Base configuration 
  TIM_TimeBaseStructure.TIM_Prescaler = 7200-1;//????,0?????????1  1/(72M/7200)*100
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//
  TIM_TimeBaseStructure.TIM_Period = 30;//????0?????3??0.3ms
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  //TIM_TimeBaseStructure.TIM_RepetitionCounter = 4;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
  TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
  
  TIM_Cmd(TIM4, ENABLE);

}
		












