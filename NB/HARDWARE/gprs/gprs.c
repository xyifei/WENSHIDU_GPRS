#include "gprs.h"
#include "usart.h"
#include "timer.h"
#include "delay.h"
#include "led.h"

extern u8 timeout_flag; //超时标志位，为1表示超时
u8 status_flag=1; //状态标志位1  //接收到Ok返回1
u8 status_flag3=1; //状态标志位2
u8 rssi = 0;
u8 RssiGrade = 3;

char temp_lala[43] = {0};
//char temp_lala[]={"AT+CIPSTART=\"TCP\",\"114.55.24.28\",50001\r"};
//char temp_lala[43]={"AT+CIPSTART=\"TCP\",\"103.46.128.21\",17354\r"};
//char temp_lala[]={"AT+CIPSTART=\"TCP\",\"049.233.217.195\",50001\r"};
//char temp_lala[]={"AT+CIPSTART=\"TCP\",\"103.046.128.045\",18631\r"};
//char temp_nb[]={"AT+SKTCONNECT=1,122.51.118.117,50001\r\n"};
char temp_nb[41]={0};
char routerSet[40]={"AT+CWJAP=\"Tcp\",\"12345678\"\r\n"};

extern unsigned char RXBUF[255];//接收GPRS模组返回的数据
extern u8 RIDLE;  //接收完毕的标志
extern u8 RXNum;  //接收数目

/*******************************************************************************
****入口参数：需要发送的字符串
****出口参数：无
****函数备注：USART发送字符串
****版权信息：
*******************************************************************************/
void Send_Str( char *dat)
{
    while(*dat!='\0')
    {      
      // USART_SendData8(USART3,*dat);	
			USART_SendData(USART2, *dat);
			//while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET); 
      while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);
      dat++;	
    }
}
/*******************************************************************************
****入口参数：字符串1，字符串2
****出口参数：位置
****函数备注：判断字符串1里面是否含有字符串2，返回位置，字符串的长度不应该超过255
****版权信息：
*******************************************************************************/
//判断str1里面是否包含str2，返回位置(注意，长度不超过255)
unsigned char Compare_str( char *str1, char *str2)
{
	char *str2_copy;
	char str1_long=0,str2_long=0;
	str2_copy = str2;//保留str2的字符串地址
    
	do
  {
		if(str1[str1_long] == str2_copy[0])
		{
			str2_long = str1_long;
			do
			{
				if(str1[str2_long++] != *str2_copy++)//如果有一个字符串不匹配，就跳出这个循环
				{
												str2_long = 0;
												break;
				}
			}while(*str2_copy);
				
			if(str2_long != 0)
				return str1_long;//如果可以执行到这里，表示有匹配的字符串
		}
		
		str2_copy = str2;
  }while(str1[str1_long++]);
	
  return 255;//如果上述循环自动退出了，那么匹配失败
}


/*******************************************************************************
****入口参数：无
****出口参数：无
****函数备注：清零接收缓冲区，避免误判断
****版权信息：
*******************************************************************************/
void CleanRXBUF(void)
{
  unsigned char num;
  for(num=0;num<255;num++)
  {
    RXBUF[num]=0;
  }
}

/*******************************************************************************
****入口参数：Str字符串-要发送的指令;Str2字符串-需要判断返回的数据有没有它
****出口参数：无
****函数备注：发送AT指令（仅限返回值为OK的指令）
****版权信息：
*******************************************************************************/
//发送AT指令.这里会等待回复的OK,否则会重发
u8 Send_AT( char *Str, char *Str2)
{
	unsigned char Loop_Count=0;
	unsigned char ReSend_Count=0;

	CleanRXBUF();    //清零USART接收缓冲区
	Send_Str(Str);//先通过串口发送出去
	
	if(Str2==0)
		 return 1;
      // return 0;
  while(1)//等待回复
  {
		delay_ms(1000);//延时等待回复
		//    Delay(1000);//延时等待回复
		Loop_Count++;
		if(Loop_Count >= 5)
		{
			ReSend_Count++;
			if(ReSend_Count < 3)
			{
				Loop_Count = 0;
				Send_Str(Str);//重发一遍
			}
			else
			{
				return 0;//重发失败，退出
			}
		}
        
		if(RIDLE)
		{
			if(Compare_str(RXBUF,Str2)!=255)
			{
				RIDLE=0;
				return 1;
			}
			else
			{
				RIDLE=0;
				return 0;
			}
		}
	}	
}

/*******************************************************************************
****入口参数：无
****出口参数：无
****函数备注：建立与服务器的连接
****版权信息：
*******************************************************************************/
void Connect(void)
{
  Send_AT("AT\r","OK");//测试是否支持AT
  Send_AT("ATE0\r","OK");//关闭回显
  Send_AT("AT+CIPMODE=1\r","OK");//进入透传模式
  //Send_AT("AT+CIPSTART=\"TCP\",\"49.233.217.195\",50001\r","OK");//IP 端口号，这个需要修改成自己的，测试时可使用http://tcplab.openluat.com/生成
  Send_AT(temp_lala,"OK");
	delay_ms(2000);
  //CleanRXBUF();
  //Send_Str("654321");
  //GetRSSI();
}

/*******************************************************************************
****入口参数：无
****出口参数：无
****函数备注：断开与服务器的连接
****版权信息：
*******************************************************************************/
void DisConnect(void)
{
  delay_ms(1500);//4000
  CleanRXBUF();   //清零USART接收缓冲区
  Send_AT("+++",0); //+++为返回AT指令模式，注意不要加\r，ATO为返回透传模式，发送前需要与前一个数据间隔1000ms，发送后需要间隔500ms
  delay_ms(1500);//5000--1000
 
  CleanRXBUF();
  
//	Send_AT("AT+CIPCLOSE\r","OK");//关闭与服务器的连接
	Send_AT("AT+CIPSHUT\r","OK");//关闭与服务器的连接
}


//串口1发送1个字符 
//c:要发送的字符
void usart_send_char(u8 c)
{   	
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕   
	USART_SendData(USART2,c);  
} 


void usart_report(void)
{
	u8 send_buf[5];
	u8 i; 
	send_buf[0]=0X88;	
	send_buf[1]=0x02;	//功能字
	send_buf[2]=0x1A;
	for(i=0;i<3;i++)usart_send_char(send_buf[i]);	//发送数据到串口1 
}

/*******************************************************************************
****入口参数：无
****出口参数：无
****函数备注：获取信号强度RSSI值
****版权信息：
*******************************************************************************/
void GetRSSI(void)
{
  unsigned char num;
  
  Send_AT("AT+CSQ\r","OK");
  num=Compare_str(RXBUF,"+CSQ:");
  if(num!=255)     //
  {
     rssi=(RXBUF[num+6]-0x30)*10+(RXBUF[num+7]-0x30);
  }
	
  if(rssi>28)
    RssiGrade=4;
  if(rssi>15&rssi<=28)
    RssiGrade=3;
  if(rssi>2&rssi<=15)
    RssiGrade=2;
  if(rssi<=2)
    RssiGrade=1;
}

void Nb_Connect(void)
{
//  Send_AT("AT\r\n","OK");                                               //测试是否支持AT
//  Send_AT("ATE0\r\n","OK");                                             //关闭回显
  Send_AT("AT+SKTCREATE=1,1,6\r\n","OK");                               //创建一个scoket
  //Send_AT("AT+SKTCONNECT=1,103.46.128.45,18631\r\n","OK");              //建立连接
	//Send_AT("AT+SKTCONNECT=1,049.233.217.195,50001\r\n","OK");
	//Send_AT("AT+SKTCONNECT=1,122.051.118.117,80\r\n","OK");
	delay_ms(3000);
	Send_AT(temp_nb,"OK");
	delay_ms(2000);
  //CleanRXBUF();
  //Send_Str("654321");
  //GetRSSI();
}

void Nb_DisConnect(void)
{
	Send_AT("AT+SKTDELETE=1\r\n","OK");//关闭与服务器的连接
}

void Nb_SendData(char *data)
{
	char str[80];
	char TxData[80];
	u8 i = 0;
	u8 j = 0;
	
	for(j=0;j<80;j++)
	{
		str[j] = 0;
		TxData[j] = 0;
	}
	
	while(*data != '\0')
	{
		if((*data/16) < 10)
		{
			str[i++] = *data/16 + 48;
		}
		else
		{
			str[i++] = *data/16 + 55;
		}
		
		if((*data%16) < 10)
		{
			str[i++] = *data%16 + 48;
		}
		else
		{
			str[i++] = *data%16 + 55;
		}

		data++;
	}
	
	sprintf(TxData, "AT+SKTSEND=1,%d,%s\r\n",(i/2),str);
	Send_AT(TxData,"OK");
}

/**************************           WIFI           ***********************************/
void Wifi_Setrouter()
{
	Send_AT("AT+CWMODE=3\r\n","OK"); 
	Send_AT(routerSet,"OK");
}


void Wifi_Connect(void)
{
//	delay_ms(1000);
//	Send_Str("AT\r\n");
//	delay_ms(1000);
//	Send_Str("ATE0\r\n");
//	delay_ms(1000);
//	Send_Str("AT+CIPSTART=\"TCP\",\"49.233.217.195\",50001\r\n");
	
	Send_AT("AT\r\n","OK");                                               //测试是否支持AT
  Send_AT("ATE0\r\n","OK");                                             //关闭回显
	Send_AT("AT+CIPMODE=1\r\n","OK"); 
	//Send_AT("AT+CIPSTART=\"TCP\",\"122.51.118.117\",50001\r\n","OK");
	Send_AT("AT+CIPSTART=\"TCP\",\"103.46.128.21\",17354\r\n","OK");
	delay_ms(2000);
	Send_AT("AT+CIPSEND\r\n","OK"); 
	
}

void Wifi_DisConnect(void)
{
	delay_ms(1500);
	CleanRXBUF();
	Send_AT("+++",0);
	delay_ms(1500);
	CleanRXBUF();
	Send_AT("AT+CIPCLOSE\r\n","OK");
}
