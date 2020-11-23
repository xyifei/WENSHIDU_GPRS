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


#define RS485E_PORT                         GPIOB  			  //����485���ͽ������� 
#define RS485E_PIN                          GPIO_Pin_12
#define RS485E_PORTRCC                      RCC_APB2Periph_GPIOB
#define RS485E_TXEN                         GPIO_SetBits(RS485E_PORT,RS485E_PIN)	//�ߵ�ƽΪ����ģʽ
#define RS485E_RXEN													GPIO_ResetBits(RS485E_PORT,RS485E_PIN)

#define RS485COM_NAME                       USART3 	              //�Ĵ�������  ���� ʱ��        
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
  * @brief  ���ƽ��պͷ���״̬
  * @param  xRxEnable ����ʹ�ܡ�
  *         xTxEnable ����ʹ��
  * @retval None
  */
void vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable ) //���ƴ��ڵ��շ��ж�
{
	if(TRUE==xRxEnable)
	{
	  RS485E_RXEN;//RS485����оƬ���շ�ģʽ���ơ�Ϊ�͵�ƽΪ����ģʽ
		Delay_us(10);
		USART_ITConfig(RS485COM_NAME, USART_IT_RXNE, ENABLE);//����1ʹ�ܽ����жϣ�׼���������ݡ�
		
	}
	else
	{	
		USART_ITConfig(RS485COM_NAME, USART_IT_RXNE, DISABLE);	//����1�رս����ж�
	}

	if(TRUE==xTxEnable)
	{
		RS485E_TXEN;	//�ߵ�ƽΪ����ģʽ
		Delay_us(10);
		USART_ITConfig(RS485COM_NAME, USART_IT_TC, ENABLE);//���ͼĴ����վ��жϣ�ʹ��
	}
	else
	{
		USART_ITConfig(RS485COM_NAME, USART_IT_TC, DISABLE);
	}
}
/*****************************************
  * @brief  ���ڳ�ʼ��
  * @param  ucPORT      ���ں�
  *         ulBaudRate  ������
  *         ucDataBits  ����λ
  *         eParity     У��λ 
  * @retval None
*****************************************/

BOOL xMBPortSerialInit( UCHAR ucPort, ULONG ulBaudRate,UCHAR ucDataBits, eMBParity eParity )
{
	GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
	RCC_APB2PeriphClockCmd(RS485COM_PORTRCC |RS485E_PORTRCC | RCC_APB2Periph_AFIO, ENABLE); 
	RCC_APB1PeriphClockCmd(RS485COM_RCC, ENABLE);
	(void)ucPort;     //�ô���1,���޸Ĵ���
	(void)ucDataBits; //���޸�����λ����
	
    /*  USART1_Tx */
    GPIO_InitStructure.GPIO_Pin = RS485COMTX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode =GPIO_Mode_AF_PP;		//�������-TX
    GPIO_Init(RS485COM_PORT, &GPIO_InitStructure);

    /*  USART1_Rx  */
    GPIO_InitStructure.GPIO_Pin = RS485COMRX_PIN;
    GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;// GPIO_Mode_IN_FLOATING;//��������-RX
    GPIO_Init(RS485COM_PORT, &GPIO_InitStructure);
	
		USART_InitStructure.USART_BaudRate = ulBaudRate; //Ŀǰֻ�޸Ĳ�����
		USART_InitStructure.USART_StopBits = USART_StopBits_1;

	switch(eParity)
	{
		case MB_PAR_NONE:
		{
		 USART_InitStructure.USART_Parity =USART_Parity_No;
		 USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		 USART_ITConfig(RS485COM_NAME, USART_IT_PE, DISABLE);//�ر���żУ����ж�
		 break;
		}
		case MB_PAR_ODD: 
		{
		 USART_InitStructure.USART_Parity =USART_Parity_Odd;
		 USART_InitStructure.USART_WordLength = USART_WordLength_9b;
		 USART_ITConfig(RS485COM_NAME, USART_IT_PE, ENABLE);//ʹ����żУ����ж�
		 break;
		 }
		case MB_PAR_EVEN:
		{
		 USART_InitStructure.USART_Parity =USART_Parity_Even;
		 USART_InitStructure.USART_WordLength = USART_WordLength_9b;
		 USART_ITConfig(RS485COM_NAME, USART_IT_PE, ENABLE);//ʹ����żУ����ж�
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

	//����RS485оƬ���ͺͽ���ģʽ�Ŀ�������
	GPIO_InitStructure.GPIO_Pin = RS485E_PIN; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(RS485E_PORT, &GPIO_InitStructure); 

  return TRUE;	
}

// ���ڷ�
BOOL xMBPortSerialPutByte( CHAR ucByte )
{
    USART_SendData(RS485COM_NAME, ucByte);   
    return TRUE;
}

// ������
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
** ��������: Delay_ms
** ��������: ��Ƶ��72MHzʱ�ģ�������ʱ��������ʾ������⣬��Խ�׼ȷ��
** ��    ��: time
** �� �� ֵ: None       
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
** ��������: Delay_us
** ��������: ������ʱ����
** ��    ��: ��ʱ΢��
** �� �� ֵ: None       
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

