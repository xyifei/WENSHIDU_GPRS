/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "stm32f10x.h"


#define RS485E_PORT                         GPIOB  			  //配置485发送接收引脚 
#define RS485E_PIN                          GPIO_Pin_12
#define RS485E_PORTRCC                      RCC_APB2Periph_GPIOB
#define RS485E_TXEN                         GPIO_SetBits(RS485E_PORT,RS485E_PIN)	//高电平为发送模式
#define RS485E_RXEN													GPIO_ResetBits(RS485E_PORT,RS485E_PIN)

#define RS485COM_NAME                       USART3 	              //改串口配置  引脚 时钟        
#define RS485COM_PORT                       GPIOB  
#define RS485COMTX_PIN                      GPIO_Pin_10
#define RS485COMRX_PIN                      GPIO_Pin_11
#define RS485COM_PORTRCC                    RCC_APB2Periph_GPIOB
#define RS485COM_RCC                        RCC_APB1Periph_USART3
#define RS485COM_IRQHandler                 USART3_IRQHandler 

void Delay_ms(uint32_t time);
void Delay_us(uint32_t time);
/* ----------------------- Start implementation -----------------------------*/
/**
  * @brief  控制接收和发送状态
  * @param  xRxEnable 接收使能、
  *         xTxEnable 发送使能
  * @retval None
  */
void vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable ) //控制串口的收发中断
{
	if(TRUE==xRxEnable)
	{
	  RS485E_RXEN;//RS485驱动芯片，收发模式控制。为低电平为接收模式
		Delay_us(10);
		USART_ITConfig(RS485COM_NAME, USART_IT_RXNE, ENABLE);//串口1使能接收中断，准备接收数据。
		
	}
	else
	{	
		USART_ITConfig(RS485COM_NAME, USART_IT_RXNE, DISABLE);	//串口1关闭接收中断
	}

	if(TRUE==xTxEnable)
	{
		RS485E_TXEN;	//高电平为发送模式
		Delay_us(10);
		USART_ITConfig(RS485COM_NAME, USART_IT_TC, ENABLE);//发送寄存器空就中断，使能
	}
	else
	{
		USART_ITConfig(RS485COM_NAME, USART_IT_TC, DISABLE);
	}
}
/*****************************************
  * @brief  串口初始化
  * @param  ucPORT      串口号
  *         ulBaudRate  波特率
  *         ucDataBits  数据位
  *         eParity     校验位 
  * @retval None
*****************************************/

BOOL xMBPortSerialInit( UCHAR ucPort, ULONG ulBaudRate,UCHAR ucDataBits, eMBParity eParity )
{
	GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
	RCC_APB2PeriphClockCmd(RS485COM_PORTRCC |RS485E_PORTRCC | RCC_APB2Periph_AFIO, ENABLE); 
	RCC_APB1PeriphClockCmd(RS485COM_RCC, ENABLE);
	(void)ucPort;     //用串口1,不修改串口
	(void)ucDataBits; //不修改数据位长度
	
    /*  USART1_Tx */
    GPIO_InitStructure.GPIO_Pin = RS485COMTX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode =GPIO_Mode_AF_PP;		//推挽输出-TX
    GPIO_Init(RS485COM_PORT, &GPIO_InitStructure);

    /*  USART1_Rx  */
    GPIO_InitStructure.GPIO_Pin = RS485COMRX_PIN;
    GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;// GPIO_Mode_IN_FLOATING;//浮空输入-RX
    GPIO_Init(RS485COM_PORT, &GPIO_InitStructure);
	
		USART_InitStructure.USART_BaudRate = ulBaudRate; //目前只修改波特率
		USART_InitStructure.USART_StopBits = USART_StopBits_1;

	switch(eParity)
	{
		case MB_PAR_NONE:
		{
		 USART_InitStructure.USART_Parity =USART_Parity_No;
		 USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		 USART_ITConfig(RS485COM_NAME, USART_IT_PE, DISABLE);//关闭奇偶校验错中断
		 break;
		}
		case MB_PAR_ODD: 
		{
		 USART_InitStructure.USART_Parity =USART_Parity_Odd;
		 USART_InitStructure.USART_WordLength = USART_WordLength_9b;
		 USART_ITConfig(RS485COM_NAME, USART_IT_PE, ENABLE);//使能奇偶校验错中断
		 break;
		 }
		case MB_PAR_EVEN:
		{
		 USART_InitStructure.USART_Parity =USART_Parity_Even;
		 USART_InitStructure.USART_WordLength = USART_WordLength_9b;
		 USART_ITConfig(RS485COM_NAME, USART_IT_PE, ENABLE);//使能奇偶校验错中断
		 break;
		 }
		default:break;
	}
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(RS485COM_NAME, &USART_InitStructure);
	USART_ITConfig(RS485COM_NAME, USART_IT_RXNE, ENABLE);
  USART_Cmd(RS485COM_NAME, ENABLE);     // ENABLE the USARTx 

	/* Disable receive and transmit interrupts from the beginning */

	//配置RS485芯片发送和接收模式的控制引脚
	GPIO_InitStructure.GPIO_Pin = RS485E_PIN; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(RS485E_PORT, &GPIO_InitStructure); 

  return TRUE;	
}

// 串口发
BOOL xMBPortSerialPutByte( CHAR ucByte )
{
    USART_SendData(RS485COM_NAME, ucByte);   
    return TRUE;
}

// 串口收
BOOL xMBPortSerialGetByte( CHAR * pucByte )
{
    *pucByte = USART_ReceiveData(RS485COM_NAME);
    return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
 void prvvUARTTxReadyISR( void )
{
    pxMBFrameCBTransmitterEmpty(  );
}

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
void prvvUARTRxISR( void )
{
     pxMBFrameCBByteReceived(  );
}

 /***************************************************************************************
** 函数名称: Delay_ms
** 功能描述: 主频在72MHz时的，粗略延时函数，经示波器检测，相对较准确。
** 参    数: time
** 返 回 值: None       
****************************************************************************************/
void Delay_ms(uint32_t time)
{
   uint32_t i=0;
   while(time--)
   {
      i=10500; 
      while(i--);    
   }
}

/***************************************************************************************
** 函数名称: Delay_us
** 功能描述: 粗略延时函数
** 参    数: 延时微秒
** 返 回 值: None       
****************************************************************************************/
void Delay_us(uint32_t time)
{    
   uint32_t i=0; 
   while(time--)
   {
      i=9;   
      while(i--);    
   }
}

