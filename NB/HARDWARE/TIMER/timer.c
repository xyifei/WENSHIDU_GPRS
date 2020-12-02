#include "timer.h"
#include "led.h"
#include "lcd.h"
#include "rtc.h"
#include "sht2x.h"
#include "rtc.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//Mini STM32开发板
//通用定时器 驱动代码			   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2010/12/03
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
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
//通用定时器中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能

	TIM_TimeBaseStructure.TIM_Period = arr; //arr//设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //psc//设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(  //使能或者失能指定的TIM中断
		TIM3, //TIM2
		TIM_IT_Update ,
		ENABLE  //使能
		);
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级0级
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //从优先级1级
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //从优先级1级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	TIM_Cmd(TIM3, ENABLE);  //使能TIMx外设
							 
}

void TIM2_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //时钟使能

	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(  //使能或者失能指定的TIM中断
		TIM2, //TIM2
		TIM_IT_Update ,
		ENABLE  //使能
		);
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2中断
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级1级
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //从优先级1级
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级1级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级1级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	//TIM_Cmd(TIM2, ENABLE);  //使能TIMx外设
	TIM_Cmd(TIM2, DISABLE);
							 
}

void TIM5_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); //时钟使能

	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(  //使能或者失能指定的TIM中断
		TIM5, //TIM2
		TIM_IT_Update ,
		ENABLE  //使能
		);
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;  //TIM2中断
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级1级
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //从优先级1级
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级1级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级1级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	//TIM_Cmd(TIM2, ENABLE);  //使能TIMx外设
	TIM_Cmd(TIM5, DISABLE);
							 
}

void TIM3_IRQHandler(void)   //TIM3中断
{
	
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
		{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源 
		if(Timing_Enable==1)//屏幕无动作返回和加锁的时间变量计数
		{
				Timing++;
				OnlyDispalyTiming++;
		}
		TimingReadSensor++;
		if(TimingReadSensor>5)//防止TimingReadSensor一直增大
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


void TIM2_IRQHandler(void) //设置为1s进一次中断
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


void TIM5_IRQHandler(void) //设置为0.1s进一次中断
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
		












