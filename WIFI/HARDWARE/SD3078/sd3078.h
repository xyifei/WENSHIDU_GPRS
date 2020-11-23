#ifndef __SD3078_H
#define __SD3078_H	    
//Mini STM32开发板
//RTC实时时钟 驱动代码			 
//正点原子@ALIENTEK
//2010/6/6
#include "stdbool.h"
#include "stm32f10x.h"

////时间结构体
//typedef struct 
//{
//	vu8 hour;
//	vu8 min;
//	vu8 sec;			
//	//公历日月年周
//	vu16 w_year;
//	vu8  w_month;
//	vu8  w_date;
//	vu8  week;		 
//}_calendar_obj;					 
//extern _calendar_obj calendar;	//日历结构体

#define SDA_IN  {GPIOB->CRL&=0XFFFF0FFF;GPIOB->CRL|=(u32)8<<12;}
#define SDA_OUT {GPIOB->CRL&=0XFFFF0FFF;GPIOB->CRL|=(u32)3<<12;}

#define  SDA_H      GPIO_SetBits(GPIOB,GPIO_Pin_3)
#define  SDA_L      GPIO_ResetBits(GPIOB,GPIO_Pin_3)
#define  SCL_H      GPIO_SetBits(GPIOA,GPIO_Pin_15)
#define  SCL_L      GPIO_ResetBits(GPIOA,GPIO_Pin_15)
#define  sdaRead    GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3)
#define  DBGMCU_CR  (*((volatile unsigned long *)0xE0042004))

void i2cInit();
void i2cStart(void);
void i2cSendByte(u8 txd);
u8 i2cWaitAck(void);
bool i2cReadBytes(uint8_t dev, uint8_t reg, uint8_t len, uint8_t *data);
void read_time_from_sd30xx(u8 s_data[7]);
void write_time_to_sd30xx(u8 s_data[7]);

#endif


