#include "gprs.h"
#include "usart.h"
#include "timer.h"
#include "delay.h"
#include "led.h"

extern u8 timeout_flag; //��ʱ��־λ��Ϊ1��ʾ��ʱ
u8 status_flag=1; //״̬��־λ1  //���յ�Ok����1
u8 status_flag3=1; //״̬��־λ2
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

extern unsigned char RXBUF[255];//����GPRSģ�鷵�ص�����
extern u8 RIDLE;  //������ϵı�־
extern u8 RXNum;  //������Ŀ

/*******************************************************************************
****��ڲ�������Ҫ���͵��ַ���
****���ڲ�������
****������ע��USART�����ַ���
****��Ȩ��Ϣ��
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
****��ڲ������ַ���1���ַ���2
****���ڲ�����λ��
****������ע���ж��ַ���1�����Ƿ����ַ���2������λ�ã��ַ����ĳ��Ȳ�Ӧ�ó���255
****��Ȩ��Ϣ��
*******************************************************************************/
//�ж�str1�����Ƿ����str2������λ��(ע�⣬���Ȳ�����255)
unsigned char Compare_str( char *str1, char *str2)
{
	char *str2_copy;
	char str1_long=0,str2_long=0;
	str2_copy = str2;//����str2���ַ�����ַ
    
	do
  {
		if(str1[str1_long] == str2_copy[0])
		{
			str2_long = str1_long;
			do
			{
				if(str1[str2_long++] != *str2_copy++)//�����һ���ַ�����ƥ�䣬���������ѭ��
				{
												str2_long = 0;
												break;
				}
			}while(*str2_copy);
				
			if(str2_long != 0)
				return str1_long;//�������ִ�е������ʾ��ƥ����ַ���
		}
		
		str2_copy = str2;
  }while(str1[str1_long++]);
	
  return 255;//�������ѭ���Զ��˳��ˣ���ôƥ��ʧ��
}


/*******************************************************************************
****��ڲ�������
****���ڲ�������
****������ע��������ջ��������������ж�
****��Ȩ��Ϣ��
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
****��ڲ�����Str�ַ���-Ҫ���͵�ָ��;Str2�ַ���-��Ҫ�жϷ��ص�������û����
****���ڲ�������
****������ע������ATָ����޷���ֵΪOK��ָ�
****��Ȩ��Ϣ��
*******************************************************************************/
//����ATָ��.�����ȴ��ظ���OK,������ط�
u8 Send_AT( char *Str, char *Str2)
{
	unsigned char Loop_Count=0;
	unsigned char ReSend_Count=0;

	CleanRXBUF();    //����USART���ջ�����
	Send_Str(Str);//��ͨ�����ڷ��ͳ�ȥ
	
	if(Str2==0)
		 return 1;
      // return 0;
  while(1)//�ȴ��ظ�
  {
		delay_ms(1000);//��ʱ�ȴ��ظ�
		//    Delay(1000);//��ʱ�ȴ��ظ�
		Loop_Count++;
		if(Loop_Count >= 5)
		{
			ReSend_Count++;
			if(ReSend_Count < 3)
			{
				Loop_Count = 0;
				Send_Str(Str);//�ط�һ��
			}
			else
			{
				return 0;//�ط�ʧ�ܣ��˳�
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
****��ڲ�������
****���ڲ�������
****������ע�������������������
****��Ȩ��Ϣ��
*******************************************************************************/
void Connect(void)
{
  Send_AT("AT\r","OK");//�����Ƿ�֧��AT
  Send_AT("ATE0\r","OK");//�رջ���
  Send_AT("AT+CIPMODE=1\r","OK");//����͸��ģʽ
  //Send_AT("AT+CIPSTART=\"TCP\",\"49.233.217.195\",50001\r","OK");//IP �˿ںţ������Ҫ�޸ĳ��Լ��ģ�����ʱ��ʹ��http://tcplab.openluat.com/����
  Send_AT(temp_lala,"OK");
	delay_ms(2000);
  //CleanRXBUF();
  //Send_Str("654321");
  //GetRSSI();
}

/*******************************************************************************
****��ڲ�������
****���ڲ�������
****������ע���Ͽ��������������
****��Ȩ��Ϣ��
*******************************************************************************/
void DisConnect(void)
{
  delay_ms(1500);//4000
  CleanRXBUF();   //����USART���ջ�����
  Send_AT("+++",0); //+++Ϊ����ATָ��ģʽ��ע�ⲻҪ��\r��ATOΪ����͸��ģʽ������ǰ��Ҫ��ǰһ�����ݼ��1000ms�����ͺ���Ҫ���500ms
  delay_ms(1500);//5000--1000
 
  CleanRXBUF();
  
//	Send_AT("AT+CIPCLOSE\r","OK");//�ر��������������
	Send_AT("AT+CIPSHUT\r","OK");//�ر��������������
}


//����1����1���ַ� 
//c:Ҫ���͵��ַ�
void usart_send_char(u8 c)
{   	
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET); //ѭ������,ֱ���������   
	USART_SendData(USART2,c);  
} 


void usart_report(void)
{
	u8 send_buf[5];
	u8 i; 
	send_buf[0]=0X88;	
	send_buf[1]=0x02;	//������
	send_buf[2]=0x1A;
	for(i=0;i<3;i++)usart_send_char(send_buf[i]);	//�������ݵ�����1 
}

/*******************************************************************************
****��ڲ�������
****���ڲ�������
****������ע����ȡ�ź�ǿ��RSSIֵ
****��Ȩ��Ϣ��
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
//  Send_AT("AT\r\n","OK");                                               //�����Ƿ�֧��AT
//  Send_AT("ATE0\r\n","OK");                                             //�رջ���
  Send_AT("AT+SKTCREATE=1,1,6\r\n","OK");                               //����һ��scoket
  //Send_AT("AT+SKTCONNECT=1,103.46.128.45,18631\r\n","OK");              //��������
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
	Send_AT("AT+SKTDELETE=1\r\n","OK");//�ر��������������
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
	
	Send_AT("AT\r\n","OK");                                               //�����Ƿ�֧��AT
  Send_AT("ATE0\r\n","OK");                                             //�رջ���
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
