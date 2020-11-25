#ifndef __GPRS_H
#define __GPRS_H

#include "sys.h"

#define AT1 0x01
#define TIMEOUT 1
#define SUCESS  2
#define ERROR   3


u8 gprs_init(void);
u8 gprs_parameter_init(void); //参数设置初始化
u8 gprs_connect(void); //TCP连接初始化
void usart_report(void);
u8 gprs_senddata(void);

void Send_Str( char *dat);
void Send_Str1( char *dat);
void Connect(void);
void DisConnect(void);
void CleanRXBUF(void);
void Connect1(void);
u8 Send_AT( char *Str, char *Str2);

void DisConnect1(void);

//NBIOT
void Nb_Connect(void);
void Nb_DisConnect(void);
void Nb_SendData(char *data);

//WIFI
void Wifi_Setrouter(void);
void Wifi_Connect(void);
//void Wifi_Startsend(void);
void Wifi_DisConnect(void);
void GetRSSI_Wifi(void);


void GetRSSI(void);
#endif


