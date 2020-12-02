/**
  ******************************************************************************
  * @file    GPIO/IOToggle/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h" 
#include "sht2x.h"
#include "stm32_eval_i2c_ee.h"
#include "delay.h"
#include "24cxx.h"
#include "usart.h"
#include "gprs.h"
#include "stdlib.h"

extern unsigned char RXBUF[255];
extern u8 RIDLE;
extern u8 RXNum;
extern u16  flag_state;
extern u16  flag_1s_nnn;
extern u16 timetick;
extern u8 flag_fasong;
extern u16 flag_ls_set_dangqian;

extern u8 ip1;
extern u8 ip2;
extern u8 ip3;
extern u8 ip4;
extern u16 port;

u8 temp;

//����1���յ���IP��ص���Ϣ
extern char temp_nb[];
extern char temp_lala[];
extern char routerSet[];
u32 flag_ls_set = 0;
u8 wifiName[20] = {0};
u8 wifiPassword[20] = {0};
u8 flag_cunchu = 0;



//����
extern u8 *usart_send_pointer;//���ڷ���ָ��
extern u8 usart_rx_buff[50];
extern u8 usart_tx_buff[50];
extern vu8 usart_tx_counter;//���͵ľ�����Ŀ
extern vu8 usart_rev_comflag;
extern u8 usart_rev_counter;//���յľ�����Ŀ
extern u8 usart_tx_counter1;//��ǰ���͵ľ�����Ŀ
extern u8 THtoStorage_10byte[10];
extern u16 LCD_yicunchu;
extern u8 THtoStorage_Read[10];
extern u8 PartofEEPROM;
extern u8 sEEAddress;
extern u8 EEPROMFull;
extern uint16_t StorgeNum;
extern u8 EEPROMDownloading;
//����
extern u8 address;
extern u8 crc_num;
u16 TransmitDataByte=0;//���ص��ֽ���
u16 StorageRead=20;
//modbusͨѶЭ��
u8 Tim_Out=1;//����3.5���ַ�ʱ�䣬����ȡ3ms (������9600��ʱ���Լ2�㼸����)��3.5��8/9600b��s=0.00291666s��
u8 Rcv_Complete=1;//һ֡�Ƿ��Ѿ��������//����Ӧ�ôӳ�ʼ����ʼ
u8 Send_Complete;//һ֡�Ƿ��Ѿ��������
u8 Com_busy;//ͨѶ��æ����ʾ��һ֡��δ�������
u8 Rcv_Buffer[210];//������Ž��յ���������һ֡����	����һ���ֽ�������Ž��յ�����Ч�ֽ�����Ҳ���������е���Ч�ֽ�����						  
u8 Send_Buffer[210];//������Ŵ����͵�������һ֡���ݣ���һ���ֽ�������Ŵ����͵���Ч�ֽ�����Ҳ���������е���Ч�ֽ�����
u8 Rcv_Data;//������Ž��յ�һ���ֽ�
u8 Send_Data;//�������Ҫ���͵�һ�ֽ�
u8 Mod_Id;//������־��Ϊ��վ��վ��
u8 Rcv_Num;//������ʾ���յ�һ֡����Ч�ֽ������ӹ����뵽CRCУ�飩
u8 Send_Num=1;//������ʾ�����͵�һ֡���ֽ���																			
u8 *PointToRcvBuf=Rcv_Buffer;//����ָ����յ����ݻ���
u8 *PointToSendBuf=Send_Buffer;//����ָ������͵����ݻ���
u8 Comu_Busy=0;//������ʾ�ܷ������һ֡����
u8 HaveMes;//���ܵ�����
u8 Online=0;//���߻����ߣ�1���ߣ�0����
u8 SendEERPOM=0;
u8 ReadPartofEEPROM=0;
//u8 BUFFER_SIZE3=10;
u8 EEPROM_Transmit[10];
u8 EEPROM_Write[10]={0,0,0,0,0,0,0,0,0,0};
u16 NumberReadEEPROM=0;
volatile uint16_t BUFFER_SIZE3=10;
void ModRcv(void);//����modbus��Ϣ����
void ModSend(void);//����modbus��Ϣ����
void MessageHandle(u8 *pointer_in,u8 *pointer_out);//�����յ�����Ϣ֡ 


extern float Temperature_Load;//����������ʪ��ͳһ�ŵ������������
extern float Humidity_Load;   //����������ʪ��ͳһ�ŵ������������
//4��1��
//CRCУ�����ò���
/* CRC ��λ�ֽ�ֵ��*/
static u8 auchCRCHi[] = {
0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,
0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,
0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,
0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,
0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,
0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,
0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,
0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,
0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,
0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,
0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,
0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,
0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
0x80,0x41,0x00,0xC1,0x81,0x40
} ;
/* CRC��λ�ֽ�ֵ��*/
static u8 auchCRCLo[] = {
0x00,0xC0,0xC1,0x01,0xC3,0x03,0x02,0xC2,0xC6,0x06,
0x07,0xC7,0x05,0xC5,0xC4,0x04,0xCC,0x0C,0x0D,0xCD,
0x0F,0xCF,0xCE,0x0E,0x0A,0xCA,0xCB,0x0B,0xC9,0x09,
0x08,0xC8,0xD8,0x18,0x19,0xD9,0x1B,0xDB,0xDA,0x1A,
0x1E,0xDE,0xDF,0x1F,0xDD,0x1D,0x1C,0xDC,0x14,0xD4,
0xD5,0x15,0xD7,0x17,0x16,0xD6,0xD2,0x12,0x13,0xD3,
0x11,0xD1,0xD0,0x10,0xF0,0x30,0x31,0xF1,0x33,0xF3,
0xF2,0x32,0x36,0xF6,0xF7,0x37,0xF5,0x35,0x34,0xF4,
0x3C,0xFC,0xFD,0x3D,0xFF,0x3F,0x3E,0xFE,0xFA,0x3A,
0x3B,0xFB,0x39,0xF9,0xF8,0x38,0x28,0xE8,0xE9,0x29,
0xEB,0x2B,0x2A,0xEA,0xEE,0x2E,0x2F,0xEF,0x2D,0xED,
0xEC,0x2C,0xE4,0x24,0x25,0xE5,0x27,0xE7,0xE6,0x26,
0x22,0xE2,0xE3,0x23,0xE1,0x21,0x20,0xE0,0xA0,0x60,
0x61,0xA1,0x63,0xA3,0xA2,0x62,0x66,0xA6,0xA7,0x67,
0xA5,0x65,0x64,0xA4,0x6C,0xAC,0xAD,0x6D,0xAF,0x6F,
0x6E,0xAE,0xAA,0x6A,0x6B,0xAB,0x69,0xA9,0xA8,0x68,
0x78,0xB8,0xB9,0x79,0xBB,0x7B,0x7A,0xBA,0xBE,0x7E,
0x7F,0xBF,0x7D,0xBD,0xBC,0x7C,0xB4,0x74,0x75,0xB5,
0x77,0xB7,0xB6,0x76,0x72,0xB2,0xB3,0x73,0xB1,0x71,
0x70,0xB0,0x50,0x90,0x91,0x51,0x93,0x53,0x52,0x92,
0x96,0x56,0x57,0x97,0x55,0x95,0x94,0x54,0x9C,0x5C,
0x5D,0x9D,0x5F,0x9F,0x9E,0x5E,0x5A,0x9A,0x9B,0x5B,
0x99,0x59,0x58,0x98,0x88,0x48,0x49,0x89,0x4B,0x8B,
0x8A,0x4A,0x4E,0x8E,0x8F,0x4F,0x8D,0x4D,0x4C,0x8C,
0x44,0x84,0x85,0x45,0x87,0x47,0x46,0x86,0x82,0x42,
0x43,0x83,0x41,0x81,0x80,0x40
} ;


u16 CRC16(u8 *puchMsgg,u8 usDataLen)//puchMsgg��Ҫ����CRCУ�����Ϣ��usDataLen����Ϣ���ֽ���  
{
    u8 uchCRCHi = 0xFF ; /* ��CRC�ֽڳ�ʼ��*/
    u8 uchCRCLo = 0xFF ; /* ��CRC �ֽڳ�ʼ��*/
    u8 uIndex ; /* CRCѭ���е�����*/
    while (usDataLen--) /* ������Ϣ������*/
    {
    uIndex = uchCRCHi ^ *puchMsgg++ ; /* ����CRC */
    uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex] ;
    uchCRCLo = auchCRCLo[uIndex] ;
    }
    return ((uchCRCHi << 8) | uchCRCLo) ;
}

void ModRcv(void)
{
              HaveMes=1;//��ʾ���յ�����Ϣ
              Rcv_Data=USART_ReceiveData(USART2);
              if(Comu_Busy!=1)//�����æ�����Խ�����һ֡��Ϣ
              {
                      TIM_Cmd(TIM4, DISABLE);
                      TIM_SetCounter(TIM4,0);
                      if((Tim_Out!=0)&&(Rcv_Data==address))//������ʱ�䳬����3.5���ַ���ͬʱ���ܵ��ֽں��Լ���վ��һ�£�����Ϊ���տ�ʼ
                      {
                              Rcv_Complete=0;//��ʾ����֡���տ�ʼ
                              Rcv_Num=0;//�������ݸ�����ʼ��
                              Rcv_Num++;//ͬʱ������һ
                              Rcv_Buffer[Rcv_Num]=Rcv_Data;
                      }
                      if((0==Tim_Out)&&(0==Rcv_Complete))//������ڽ���һ֡������������
                      {
                              if(Rcv_Num<100)
                              {
                                      Rcv_Buffer[Rcv_Num+1]=Rcv_Data;//�����ݷ������������
                                      Rcv_Num++;//ͬʱ������һ	
                                      
                                      if(Rcv_Num>=8)
                                      Rcv_Num=Rcv_Num;
                                     /* */
                              }
                              else
                              {
                                      
                                      Rcv_Complete=1;
                                      Comu_Busy=1;
                                      Rcv_Buffer[0]=Rcv_Num;
                                      *(PointToSendBuf+2)=*(PointToRcvBuf+2);//��ȡ������
                                      //ErrorHandle(6,PointToSendBuf);//��ʾ�������ֽ���(�ӻ��豸æµ)
                                      Rcv_Num=0;
                                    /*  */
                              }
                      }
                      Tim_Out=0;
                      TIM_Cmd(TIM4, ENABLE);//����4.5ms��ʱ���������ַ��ı��ع��ƣ�
              }
      
}


void ModRcvEEPROM(void)
{
              HaveMes=1;//��ʾ���յ�����Ϣ
							SendEERPOM=1;
              Rcv_Data=USART_ReceiveData(USART1);
              if(Comu_Busy!=1)//�����æ�����Խ�����һ֡��Ϣ
              {
                      TIM_Cmd(TIM4, DISABLE);
                      TIM_SetCounter(TIM4,0);
                      if((Tim_Out!=0)&&(Rcv_Data==address))//������ʱ�䳬����3.5���ַ���ͬʱ���ܵ��ֽں��Լ���վ��һ�£�����Ϊ���տ�ʼ
                      {
                              Rcv_Complete=0;//��ʾ����֡���տ�ʼ
                              Rcv_Num=0;//�������ݸ�����ʼ��
                              Rcv_Num++;//ͬʱ������һ
                              Rcv_Buffer[Rcv_Num]=Rcv_Data;
                      }
                      if((0==Tim_Out)&&(0==Rcv_Complete))//������ڽ���һ֡������������
                      {
                              if(Rcv_Num<100)
                              {
                                      Rcv_Buffer[Rcv_Num+1]=Rcv_Data;//�����ݷ������������
                                      Rcv_Num++;//ͬʱ������һ	
                                      
                                      if(Rcv_Num>=8)
                                      Rcv_Num=Rcv_Num;
                                     /* */
                              }
                              else
                              {
                                      
                                      Rcv_Complete=1;
                                      Comu_Busy=1;
                                      Rcv_Buffer[0]=Rcv_Num;
                                      *(PointToSendBuf+2)=*(PointToRcvBuf+2);//��ȡ������
                                      //ErrorHandle(6,PointToSendBuf);//��ʾ�������ֽ���(�ӻ��豸æµ)
                                      Rcv_Num=0;
                                    /*  */
                              }
                      }
                      Tim_Out=0;
                      TIM_Cmd(TIM4, ENABLE);//����4.5ms��ʱ���������ַ��ı��ع��ƣ�
              }
      
}

void ModSend(void)	
 {	
 	Send_Data=*(PointToSendBuf+Send_Num);
  	USART_SendData(USART2,Send_Data);
	Send_Num++;
	if(Send_Num>(*PointToSendBuf))//�����Ѿ����
	{
		Comu_Busy=0;
		*PointToSendBuf=0;
		Rcv_Num=0;
		Send_Num=1;
		//�������ݷ���
	  USART_ITConfig(USART2, USART_IT_TC, DISABLE);//�ر����ݷ����ж�
	}
 }


//*/
extern SHT2x_PARAM g_sht2x_param;//��ʪ����Ϣ
 void Modbus_Function_0(void)                      //���͵��ֽ���Ϣ
{
  u16 SendKey;
	int i=0;
  usart_tx_buff[0] = address;//�ӻ���ַ
  usart_tx_buff[1] = 0x03;//������
  
	usart_tx_buff[2] = TransmitDataByte;//�����ֽ�������Ҫ��̬�ĵ���
 
	usart_tx_buff[3] = (int)(Humidity_Load*10)>>8;//ʪ�ȸ��ֽ�0x30+1234/1000
	usart_tx_buff[4] = (int)(Humidity_Load*10)&0x00FF;//ʪ�ȵ��ֽ�
	if((int)(Temperature_Load)>>15==1)
	{
		Temperature_Load=65536-Temperature_Load;//���¶��Բ�����ʽ�ϴ�
	}
	usart_tx_buff[5] = (int)(Temperature_Load*10)>>8;//�¶ȸ��ֽ�
	usart_tx_buff[6] = (int)(Temperature_Load*10)&0x00FF;//�¶ȵ��ֽ�
	
	//���0x00
	for(i=TransmitDataByte-4;i>0;i--)
	{
		usart_tx_buff[6+i]=0x00;
	}

	SendKey=CRC16(usart_tx_buff,TransmitDataByte+3);//���س�CRC��֮����ֽ���Ŀ
	
	usart_tx_buff[TransmitDataByte+3] = (u8)(SendKey>>8);
	usart_tx_buff[TransmitDataByte+4] = (u8)(SendKey&0x00FF); 
	usart_tx_counter =TransmitDataByte+5;
  
  
  
  usart_send_pointer=usart_tx_buff;
  
  USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
}

 void Modbus_Function_3(void)                      //ֻ�����¶�
{
  u16 SendKey;
	int i=0;
  usart_tx_buff[0] = address;//�ӻ���ַ
  usart_tx_buff[1] = 0x03;//������
  
	usart_tx_buff[2] = TransmitDataByte;//�����ֽ�������Ҫ��̬�ĵ���
 
	//usart_tx_buff[3] = (int)(Humidity_Load*10)>>8;//ʪ�ȸ��ֽ�0x30+1234/1000
	//usart_tx_buff[4] = (int)(Humidity_Load*10)&0x00FF;//ʪ�ȵ��ֽ�
	if((int)(Temperature_Load)>>15==1)
	{
		Temperature_Load=65536-Temperature_Load;//���¶��Բ�����ʽ�ϴ�
	}
	usart_tx_buff[3] = (int)(Temperature_Load*10)>>8;//�¶ȸ��ֽ�
	usart_tx_buff[4] = (int)(Temperature_Load*10)&0x00FF;//�¶ȵ��ֽ�
	
	//���0x00
	for(i=TransmitDataByte-2;i>0;i--)
	{
		usart_tx_buff[4+i]=0x00;
	}

	SendKey=CRC16(usart_tx_buff,TransmitDataByte+3);//���س�CRC��֮����ֽ���Ŀ
	
	usart_tx_buff[TransmitDataByte+3] = (u8)(SendKey>>8);
	usart_tx_buff[TransmitDataByte+4] = (u8)(SendKey&0x00FF); 
	usart_tx_counter =TransmitDataByte+5;
  
  
  
  usart_send_pointer=usart_tx_buff;
  
  USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
}

 void Modbus_Function_4(void)                      //ֻ����ʪ��
{
  u16 SendKey;
	int i=0;
  usart_tx_buff[0] = address;//�ӻ���ַ
  usart_tx_buff[1] = 0x03;//������
  
	usart_tx_buff[2] = TransmitDataByte;//�����ֽ�������Ҫ��̬�ĵ���
 
	usart_tx_buff[3] = (int)(Humidity_Load*10)>>8;//ʪ�ȸ��ֽ�0x30+1234/1000
	usart_tx_buff[4] = (int)(Humidity_Load*10)&0x00FF;//ʪ�ȵ��ֽ�
//	if((int)(Temperature_Load)>>15==1)
//	{
//		Temperature_Load=65536-Temperature_Load;//���¶��Բ�����ʽ�ϴ�
//	}
//	usart_tx_buff[5] = (int)(Temperature_Load*10)>>8;//�¶ȸ��ֽ�
//	usart_tx_buff[6] = (int)(Temperature_Load*10)&0x00FF;//�¶ȵ��ֽ�
	
	//���0x00
	for(i=TransmitDataByte-2;i>0;i--)
	{
		usart_tx_buff[4+i]=0x00;
	}

	SendKey=CRC16(usart_tx_buff,TransmitDataByte+3);//���س�CRC��֮����ֽ���Ŀ
	
	usart_tx_buff[TransmitDataByte+3] = (u8)(SendKey>>8);
	usart_tx_buff[TransmitDataByte+4] = (u8)(SendKey&0x00FF); 
	usart_tx_counter =TransmitDataByte+5;
  
  
  
  usart_send_pointer=usart_tx_buff;
  
  USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
}


 void Modbus_Function_1(void)                      //����11�ֽ���ʷ����
{
	//const u8 BUFFER_SIZE3=10;//���ֵ���붨���ں����ڲ�����ÿ�ε��ö�дEEPROM֮ǰ�������һ����ֵ
	EEPROMDownloading=2;
	if(StorageRead<65530&&ReadPartofEEPROM==0)
		//sEEAddress=Part1_Write;
		EEPROM1024Part(0);
	if(StorageRead>=65530||ReadPartofEEPROM==1)
	{
		//sEEAddress=Part2_Write;
		EEPROM1024Part(1);
		ReadPartofEEPROM=1;
		if(StorageRead>=65530)
			StorageRead=50;
	}
	BUFFER_SIZE3=10;
	//sEE_ReadBuffer(EEPROM_Transmit,StorageRead,(uint16_t *)(&BUFFER_SIZE3));
	AT24CXX_Read(StorageRead,EEPROM_Transmit,BUFFER_SIZE3);
	//delay_ms(100);
  usart_tx_buff[0] = address;//�ӻ���ַ
  usart_tx_buff[1] = EEPROM_Transmit[0];//�¶ȸ߰�λ
	usart_tx_buff[2] = EEPROM_Transmit[1];//�¶ȵͰ�λ
	usart_tx_buff[3] = EEPROM_Transmit[2];//ʪ�ȸ߰�λ
	usart_tx_buff[4] = EEPROM_Transmit[3];//ʪ�ȵͰ�λ
	usart_tx_buff[5] = EEPROM_Transmit[4];//��
	usart_tx_buff[6] = EEPROM_Transmit[5];//��
	usart_tx_buff[7] = EEPROM_Transmit[6];//��
	usart_tx_buff[8] = EEPROM_Transmit[7];//ʱ
	usart_tx_buff[9] = EEPROM_Transmit[8];//��
	usart_tx_buff[10] = EEPROM_Transmit[9];//��
	

	usart_tx_counter =11;
	
	//sEE_WriteBuffer((u8*) EEPROM_Write,StorageRead,BUFFER_SIZE3);
	
	StorageRead=StorageRead+10;
	if((StorageRead/10)>NumberReadEEPROM)//ȫ�����ݶ�ȡ���֮���´��ٶ�ȡ���´�ͷ��ʼ����EEPROM��ʼ��ͷд
	{
		StorageRead=50;
		EEPROMFull=0;
		StorgeNum=50;
		PartofEEPROM=0;
//		
	}
  usart_send_pointer=usart_tx_buff;

  
  //USART_ITConfig(USART1, USART_IT_TXE, ENABLE); 
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);//��λ�����õ���485����
} 

void Modbus_Function_2(void)                      //���͵�ַ�� ������ 02 00 ������������ CRC�� CRC��
{
  u16 SendKey;
	int i=0;
	NumberReadEEPROM=LCD_yicunchu;
	
//			EEPROMFull=0;
//		StorgeNum=20;
//		PartofEEPROM=0;
	
	
  usart_tx_buff[0] = address;//�ӻ���ַ
  usart_tx_buff[1] = 0x03;//������
  
	usart_tx_buff[2] = 0x02;//
 
	usart_tx_buff[3] = (NumberReadEEPROM&0xFF00)>>8;//�Ѵ洢�������ֽ�
	usart_tx_buff[4] = NumberReadEEPROM&0x00FF;//�Ѵ洢�����ֽ�


	SendKey=CRC16(usart_tx_buff,5);//���س�CRC��֮����ֽ���Ŀ
	
	usart_tx_buff[5] = (u8)(SendKey>>8);
	usart_tx_buff[6] = (u8)(SendKey&0x00FF); 
	usart_tx_counter =7;
  
  
  
  usart_send_pointer=usart_tx_buff;
  
  //USART_ITConfig(USART1, USART_IT_TXE, ENABLE); 
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);//��λ�����õ���485����
}

void MessageHandle(u8 *pointer_in,u8 *pointer_out)		  
 {
 	u16 CalKey;//���������У��ֵ
	u16 RcvKey;//���յ���У��ֵ
	
	HaveMes=0;//�����Ϣλ
	//��ȡ���յ���У��ֵ
	if(SendEERPOM==0)
	{		
			RcvKey=(u16)*(pointer_in+(*pointer_in-1));//pointer_inΪ���յ������ݵ���ʼ��ַ
			RcvKey=RcvKey<<8;
			RcvKey=RcvKey|(u16)*(pointer_in+(*pointer_in));
			TransmitDataByte=(u16)*(pointer_in+(*pointer_in-3));
			TransmitDataByte=TransmitDataByte<<8;
			TransmitDataByte=(TransmitDataByte|(u16)*(pointer_in+(*pointer_in-2)))*2;//���Ӧ�÷��ص��ֽ����Ķ���֮һ��
			if(TransmitDataByte>0x20)
			{
				TransmitDataByte=0x20;
			}
			CalKey=CRC16(pointer_in+1,*pointer_in-2);//
			CalKey=CRC16(Rcv_Buffer+1,6);
			CalKey=CRC16(pointer_in+1,*pointer_in-2);//
	}
	else 
	{
		CalKey=RcvKey;
	}
        if(CalKey==RcvKey)
        {
					if(SendEERPOM==0)
					{
						switch(*(pointer_in+2))//*(pointer_in+2)������
						{
									case 0x03: 
									GPIO_SetBits(GPIOA,GPIO_Pin_1);//����
									if(TransmitDataByte==0x04)//��ѯ�¶Ⱥ�ʪ��
										Modbus_Function_0(); //����λ��������Ϣ
									else if(TransmitDataByte==0x02)//
									{
										if(*(pointer_in+4)==0x00)//ֻ��ѯʪ��
										{
											Modbus_Function_4();
										}
										else if(*(pointer_in+4)==0x01)//ֻ��ѯ�¶�
										{
											Modbus_Function_3();
										}
									}
									break;
									case 0x01:           
									break;
									default:break; 
						 }
				  }
					else 
					{
						switch(*(pointer_in+3))//*(pointer_in+2)������
						{
									case 0x10: 
									GPIO_SetBits(GPIOA,GPIO_Pin_1);//����
									Modbus_Function_1(); //����λ��������Ϣ
									break;
									case 0x00:  
									GPIO_SetBits(GPIOA,GPIO_Pin_1);//����
									Modbus_Function_2(); //����λ��������Ϣ										
									break;
									default:break; 
						 }
					}
					
          
        }
        else
        { 
           Comu_Busy=0;
        }

 } 
void USART2_IRQHandler(void)
{
	if(SET==USART_GetITStatus (USART2,USART_IT_IDLE))
	{
		USART_ReceiveData (USART2);
		USART_ClearITPendingBit (USART2,USART_IT_IDLE);//���жϱ�־
		RIDLE=1;
		RXNum=0;
	}
 if(SET==USART_GetITStatus (USART2,USART_IT_RXNE))
	{
		temp=USART_ReceiveData (USART2);
		USART_ClearITPendingBit (USART2,USART_IT_RXNE);//���жϱ�־
		RXBUF[RXNum]=temp;
		RXNum++;
	}
}

//����1���ϴ���ʷ����
void USART1_IRQHandler(void)                	//����1�жϷ������
{
	u8 i=0;
	u8 n=2;
	u16 m=0;
	uint8_t TxData[10];
	static uint8_t tmp;
	static uint8_t Rec_flag = 0;
	static uint8_t Rec_counter = 0;
	static uint8_t RxData[20];
	
	for(i=0;i<10;i++)
	{
		TxData[i] = 0;
	}
	
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET)
	{
		tmp = USART_ReceiveData(USART1);
		
		if(Rec_flag == 0)
		{
			if(tmp == '#')
			{
				Rec_counter = 0;
			}
			
			RxData[Rec_counter++] = tmp;
		
			if(tmp == 0x0a)
				if(RxData[Rec_counter-2] == 0x0d)
					Rec_flag = 1;
		}
	}

	if(Rec_flag == 1)
	{
		Rec_flag = 0;
		
		//������λ����������IP
		if(RxData[1] == 0x01)
		{
			if(flag_state >= 2)
			{
				Wifi_DisConnect();
//				DisConnect();
//			Nb_DisConnect();
				GPIO_ResetBits(GPIOA,GPIO_Pin_1);
			}
				
			ip1 = RxData[2];
			ip2 = RxData[3];
			ip3 = RxData[4];
			ip4 = RxData[5];
			port = RxData[6]*256+RxData[7];
			
			sprintf(temp_lala,"AT+CIPSTART=\"TCP\",\"%d.%d.%d.%d\",%d\r\n",RxData[2],RxData[3],RxData[4],RxData[5],(RxData[6]*256+RxData[7]));
			//sprintf(temp_lala,"AT+CIPSTART=\"TCP\",\"%d.%d.%d.%d\",%d\r",RxData[2],RxData[3],RxData[4],RxData[5],(RxData[6]*256+RxData[7]));
			//sprintf(temp_nb,"AT+SKTCONNECT=1,%d.%d.%d.%d,%d\r\n",RxData[2],RxData[3],RxData[4],RxData[5],(RxData[6]*256+RxData[7]));
			flag_cunchu = 1;
		}
		
		//��IP��ַ���͸���λ��
		if(RxData[1] == 0x02)
		{
			TxData[0] = 0x23;
			TxData[1] = 0x02;
			TxData[8] = 0x0d;
			TxData[9] = 0x0a;
			
			//2Gģ��
			for(i=19;i<=42;i++)
			{
				if((temp_lala[i] != '.') && (temp_lala[i] != '"'))
				{
					TxData[n] = TxData[n]*10 + temp_lala[i]-48;
				}
				else
				{
					n++;
				}
					
				if(temp_lala[i] == ',')
				{
					n = i;
					break;
				}					
			}
			
			for(i=n+1;i<=42;i++)
			{
				if(temp_lala[i] != 0x0d)
				{
					m = m*10 + temp_lala[i]-48;
				}
				else
				{
					break;
				}
			}
			
			TxData[6] = m/256;
			TxData[7] = m%256;
			
//			//NBģ��
//			for(i=16;i<41;i++)
//			{
//				if(temp_nb[i] == '\r')
//				{
//					break;
//				}
//				
//				if((temp_nb[i]!='.') && (temp_nb[i]!=','))
//				{
//					if(n<=5)
//					{
//						TxData[n] = TxData[n]*10 + temp_nb[i]-48;
//					}
//					else
//					{
//						m = m*10 + temp_nb[i]-48;
//					}
//				}
//				else
//				{
//					n++;
//				}
//				
//			}
//			
//			TxData[6] = m/256;
//			TxData[7] = m%256;
			
			for(i=0;i<10;i++)
			{
				USART_SendData(USART1, TxData[i]); 
				//while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
				while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
			}
		}
		
		//���������ϴ�ʱ����
		if(RxData[1] == 0x03)
		{
			if(flag_state >= 2)
			{
				Wifi_DisConnect();
//				DisConnect();
//				Nb_DisConnect();
				GPIO_ResetBits(GPIOA,GPIO_Pin_1);
			}
				
			flag_ls_set = RxData[2]*256 + RxData[3];
			flag_ls_set_dangqian = flag_ls_set;
			flag_state = 0;
			flag_1s_nnn = 0;
			timetick = 0;
			flag_fasong = 0;
			flag_cunchu = 1;
		}
		
		//��ȡ�����ϴ�ʱ����
		if(RxData[1] == 0x04)
		{
			TxData[0] = 0x23;
			TxData[1] = 0x04;
			TxData[2] = flag_ls_set/256;
			TxData[3] = flag_ls_set%256;
			TxData[4] = 0x0d;
			TxData[5] = 0x0a;
			
			for(i=0;i<6;i++)
			{
				USART_SendData(USART1, TxData[i]); 
				//while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
				while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
			}
		}
		
		//��ȡ·��������
		if(RxData[1] == 0x05)
		{
//			if(flag_state >= 2)
//				Wifi_DisConnect();
//			else if(flag_state == 0)
				GPIO_SetBits(GPIOA,GPIO_Pin_1);
			delay_ms(1000);
			
			for(i=0;i<Rec_counter-4;i++)
			{
				wifiName[i] = RxData[i+2];
			}
			wifiName[i] = '\0';
			
			sprintf(routerSet,"AT+CWJAP=\"%s\",\"%s\"\r\n",wifiName,wifiPassword);
			
			Wifi_Setrouter();
			GPIO_ResetBits(GPIOA,GPIO_Pin_1);
		}
		
		//��ȡ·��������
		if(RxData[1] == 0x06)
		{
//			if(flag_state >= 2)
//				Wifi_DisConnect();
//			else if(flag_state == 0)
				GPIO_SetBits(GPIOA,GPIO_Pin_1);
			delay_ms(1000);
			
			for(i=0;i<Rec_counter-4;i++)
			{
				wifiPassword[i] = RxData[i+2];
			}
			wifiPassword[i] = '\0';
			
			sprintf(routerSet,"AT+CWJAP=\"%s\",\"%s\"\r\n",wifiName,wifiPassword);
			
			Wifi_Setrouter();
			GPIO_ResetBits(GPIOA,GPIO_Pin_1);
		}
	}

} 
	
void TIM4_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
  {
      
        
      	Tim_Out=1;
				TIM_Cmd(TIM4,DISABLE);
        TIM_SetCounter(TIM4,0);
        Rcv_Complete=1;
        Rcv_Buffer[0]=Rcv_Num;
        if(HaveMes!=0&&Rcv_Num>4)
        {
            Comu_Busy=1;
            
						if(Rcv_Buffer[4]==0x22||Rcv_Buffer[3]==0x10)//��λ�����棬������ʷ���ݺ��������䶼���õ�485��
																										//Ҳ������λ���Ĵ���2������1ʵ��δʹ�ã�����Ҫ����һ����ʲôָ��
							SendEERPOM=1;
						else 
							SendEERPOM=0;
            MessageHandle(PointToRcvBuf,PointToSendBuf);
						
        }    
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
  }
}



void NMI_Handler(void)
{
}
 
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}
 
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

 
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}
 
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}
 
void SVC_Handler(void)
{
}
 
void DebugMon_Handler(void)
{
}
 
void PendSV_Handler(void)
{
}
 
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
