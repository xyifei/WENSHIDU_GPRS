#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "key.h"  
#include "dac7512.h"
//#include "stm32_eval_i2c_ee.h"
#include "sht2x.h"
#include "lcd.h"
#include "RTC.h"
#include "timer.h"
#include "RELAY.h"
#include "adc.h"
#include "dac.h"
#include "24cxx.h"
#include "myiic.h"
#include "exti.h"
#include "sd3078.h"
#include "gprs.h"

//Lemon
//sht10
//#include "SHT10.h"
//dht10 dht20
#include "sys2.h"
#include "DHxx.h"
//Lemon
const u8 TEXT_Buffer[]={"IICTEST1234567890987654321"};
#define SIZE sizeof(TEXT_Buffer)

extern u8 RssiGrade;
unsigned char RXBUF[255];//����GPRSģ�鷵�ص�����
u8 RIDLE=0;  //������ϵı�־
u8 RXNum=0;  //������Ŀ
u8 DeviceID = 0;
u8 ip1 = 0;
u8 ip2 = 0;
u8 ip3 = 0;
u8 ip4 = 0;
u16 port = 0;

//��ʱ������
u8  flag_1s=0;
u16  flag_state=0;
u8 flag_fasong;
u32 flag_ls_set_dangqian;
extern u32 flag_ls_set;
extern char temp_lala[43];
extern u8 flag_cunchu;
extern u16 timetick;
extern u16 flag_1s_nnn;

//�ײ���ʾ�ı���
u16 LCD_dizhi;
u16 LCD_xindao;
u16 LCD_yicunchu;
u16 LCD_ID;
u16 LCD_botelv;
u16 LCD_moshi;

extern u16 count_readsht;
//robin  RS485����
//u8 address=0x01;
u8 crc_num;
u8 *usart_send_pointer;//���ڷ���ָ��
u8 usart_rx_buff[50];
u8 usart_tx_buff[50];
vu8 usart_tx_counter=0;////���͵ľ�����Ŀ
u8 usart_tx_counter1=0;//��ǰ���͵ľ�����Ŀ
vu8 usart_rev_comflag=0;
u8 usart_rev_counter;//���յľ�����Ŀ
//robin
//lemon  SHT10�ı������ӳٺ���
u16 humi_val, temp_val;
u8 err = 0, checksum = 0;
float humi_val_real = 0.0;
float temp_val_real = 0.0;
float dew_point = 0.0;
void Delay_ms_shtxx(unsigned int n)
{
	unsigned int i;
	for ( i=0;i<n;++i);
}
//lemon
//Lemon   LCD�˵���ʾ�ı�������
int Timing=0;            //
int Timing_Enable=0;
int TimingReadSensor=0;  //������ʪ�ȶ�ȡʱ�������2S��ȡһ��
int Menu_mode=0;         //Ĭ��Ϊ0��������������ʪ�ȵ�ģʽ��1-15�ֱ����ͬ�Ĺ�������
//���õĲ���
u32 address=0x01;          //485�豸��ַ
int BaudRate=9600;       //ͨ�Ų����ʣ�Ĭ��Ϊ9600
int BaudRateNum=3;       
int BaudRateArray[5]={1200,2400,4800,9600,19200};
int Warning=1;           //����ʹ�ܱ�־λ��0��OFF,1:ON
int Temperatuer_Up=80;    //�¶���ֵ����1:�����һ����
int Temperatuer_Down=-40;  //1:�����һ����
int Temperature_O=0; 
int Temperature_I=0;  
int Humtidy_Up=99;        //ʪ����ֵ����
int Humtidy_Down=1;      //ʪ����ֵ����
int C_F=0;               //�¶ȵ�λ�л���־λ��0�����϶�C��1�����϶�F
int RemoteControl=1;     //Զ�̿��Ʊ�־λ��Ĭ�ϴ򿪣���λ�������޸Ĳ���
int RecordMode=0;        //��¼ģʽ��־λ��1��OFF,0:ON
int StorageMode=1;       //�洢ģʽ��0��˳��洢��1:ѭ���洢
int RecordTiming=30;     //������¼���ڣ���������1-600min
int WarningTiming=1;     //�������ڣ���������1-600min
int BatteryMonitor=0;    //��ص������
int SensorSelected=1;    //��ʪ�ȴ�����ѡ��1-4��1��SHT20,2:SHT10,3:DHT10,4:DHT20
int TimingSetting=0;     //ʱ������
int DeleteData=0;				 //0ɾ����ʷ���ݣ�1��ɾ����ʷ����
//int TemperatureHys=120;		 //�¶ȵĳ�����
int TemperatureHys=0;		 //�¶ȵĳ�����
int TemperatureHysDisplay=0;
//int HumtidyHys=100;				//ʪ�ȳ���������
int HumtidyHys=0;				//ʪ�ȳ���������
int HumtidyHysDisplay=0;			
int Temperature_4ma=-40;		 //�¶�4ma
int TemperatureDisplay_4ma=0;		 
int Temperature_20ma=80;		 //�¶�20ma
int TemperatureDisplay_20ma=0;	
int Humtidy_4ma=0;		 //ʪ��4ma
int HumtidyDisplay_4ma=0;		 
int Humtidy_20ma=99;		 //�¶�20ma
int HumtidyDisplay_20ma=0;
int TemperatureCalibration=0;		 //�¶�У׼������ҪУ׼��С�����һλ�����Դ洢����10��
int TemperatureCalibrationDisplay=0;		
int HumtidyCalibration=0;		 //ʪ��У׼������ҪУ׼��С�����һλ�����Դ洢����10��
int HumtidyCalibrationDisplay=0;
//u8 ParameterRead[16];    //��Ÿ����������б���Ͷ�ȡ
//u8 ParameterStorage[16];    //��Ÿ����������б���Ͷ�ȡ
//u8 ParameterStorage[20];
//u8 ParameterRead[20];
u8 ParameterStorage[50];
u8 ParameterRead[50];
volatile uint16_t NumDataRead = 0;
//volatile uint16_t BUFFER_SIZE1=0;
//volatile uint16_t BUFFER_SIZE2=0;
u8 DisplayTime_On=1; //�Ƿ���ʾ�ײ���Ϣ����1����0 
u8 PartofEEPROM=0;//EEPROM�Ĵ洢λ�ã�0ǰ64K��1��64K
u8 EEPROMFull=0;//EEPROM�Ƿ�����
u8 EEPROMDownloading=0;//������������
u8 TemperatureHighWarning=0;//�¶ȸ߱�����־λ
u8 TemperatureLowWarning=0;
u8 HumidityHighWarning=0;
u8 HumidityLowWarning=0;

u8 key=0;
int OnlyDispalyTime=0;
int OnlyDispalyTiming=0;

int NormalState=1;       //����״̬����ʾ
//unsigned char i=0;
u16 year_Set=2018;//2018
char unsigned month_Set=1;
char unsigned date_Set=1;
char unsigned hour_Set=1;
char unsigned min_Set=1;
char unsigned sec_Set=1;
int RTC_Control=0;      //0:�����Ľ������ó���1������RTC���ý������
int RTC_Selected=0;     //ѡ���޸����ڵ���һλ��0Ĭ��Ϊyear
int RTCTimeParametersInit=0;//��֤ÿ�ν���ʱ�����ý������һ��SetTimeParameters����
extern SHT2x_PARAM g_sht2x_param;//SHT20����ʪ������
int WarningOrNot=0;    //�Ƿ����ݱ�����־λ
uint16_t StorgeNum=50;      //��¼��ǰ�����Ѿ��洢�����ֽ�
int StorgeTiming=0;    //��¼����ʱ��
float THtoStorage[]={0,0};
u8 THtoStorage_4byte[]={0,0,0,0};
u8 THtoStorage_10byte[]={0,0,0,0,0,0,0,0,0,0,};
u8 THtoStorage_Read[10];
float Temperature_Load=0;//����������ʪ��ͳһ�ŵ������������
float Humidity_Load=0;   //����������ʪ��ͳһ�ŵ������������
float Temperature_Load_F=0; //�¶Ȼ��϶�


u8    flag_chuanganqi=1;//Ĭ�����д�����
extern u8 tdata[4];
extern u8 Online;

extern u16 sEEAddress;
int Adjust_Tem_4ma;
int Adjust_Tem_20ma;
int Adjust_Hum_4ma;
int Adjust_Hum_20ma;
int Adjust_Tem_5v;
int Adjust_Hum_10v;

u16 ADC_5V=0;
u16 ADC_BAT=0;
u16 ADC_24V=0;
float V_5V=0;
float	V_BAT=0;
float V_24V=0;
u16 Dac_Temp=0;
u16 Dac_Humi=0;
float Battery[10]={3.68,3.74,3.77,3.79,3.82,3.87,3.92,3.98,4.06,4.20};
u32 adc100[100];

/*! 
*  \brief  ������ת�����ַ������������չʾ����Ļ��
*  \param  Arr  ��ŵ�����
*  \param  data �������� 
*/
void NumToArr(uint8_t *Arr, uint32_t data)
{	
	if(data<100)
	{
		Arr[0] = '0';
		Arr[1] = data/10+48;
		Arr[1] = '.';
		Arr[2] = data%10+48;
	}
	else if(data>=100 && data<1000)
	{
		Arr[0] = data/100+48;
		Arr[1] = data%100/10+48;
		Arr[2] = '.';
		Arr[3] = data%100%10+48;
	}
}

void ReadParameters_wuxian(void)
{
	uint16_t BUFFER_SIZE1;
	BUFFER_SIZE1=9;
	NumDataRead=BUFFER_SIZE1;
	EEPROM1024Part(1);
	//sEE_ReadBuffer(ParameterRead,0,(uint16_t *)(&BUFFER_SIZE1));
	AT24CXX_Read(0,ParameterRead,BUFFER_SIZE1);
	delay_ms(100);
	
	ip1 = ParameterRead[0];
	ip2 = ParameterRead[1];
	ip3 = ParameterRead[2];
	ip4 = ParameterRead[3];
	port = ParameterRead[4]*256+ParameterRead[5];
	sprintf(temp_lala,"AT+CIPSTART=\"TCP\",\"%d.%d.%d.%d\",%d\r",ip1,ip2,ip3,ip4,port);
	
//	DeviceID = ParameterRead[6]*65536 + ParameterRead[7]*4096 + ParameterRead[8]*256 + ParameterRead[9];
//	flag_ls_set = ParameterRead[10]*256 + ParameterRead[11];
	
	DeviceID = ParameterRead[6];
	flag_ls_set = ParameterRead[7]*256 + ParameterRead[8];
	
	flag_ls_set_dangqian=flag_ls_set;
}

void StorageParameters_wuxian(void)
{
	uint16_t BUFFER_SIZE1;
	NumDataRead=BUFFER_SIZE1;
	ParameterStorage[0]=ip1;
	ParameterStorage[1]=ip2;
	ParameterStorage[2]=ip3;
	ParameterStorage[3]=ip4;
	ParameterStorage[4]=port/256;
	ParameterStorage[5]=port%256;
	ParameterStorage[6]=DeviceID;
	ParameterStorage[7]=flag_ls_set/256;
	ParameterStorage[8]=flag_ls_set%256;
//	ParameterStorage[6]=DeviceID/65536;
//	ParameterStorage[7]=DeviceID%65536/4096;
//	ParameterStorage[8]=DeviceID%4096/256;
//	ParameterStorage[9]=DeviceID%256;
//	ParameterStorage[10]=flag_ls_set/256;
//	ParameterStorage[11]=flag_ls_set%256;
	BUFFER_SIZE1=9;
	EEPROM1024Part(1);
	AT24CXX_Write(0,(u8*)ParameterStorage,BUFFER_SIZE1);
	delay_ms(100);
}


//�ϵ�����ʱ����ִ��һ��,���EEPROMǰ20���ֽ�
void ReadParameters(void)
{
		uint16_t BUFFER_SIZE1;
		BUFFER_SIZE1=50;
		NumDataRead=BUFFER_SIZE1;
		EEPROM1024Part(0);
		//sEE_ReadBuffer(ParameterRead,0,(uint16_t *)(&BUFFER_SIZE1));
		AT24CXX_Read(0,ParameterRead,BUFFER_SIZE1);
		delay_ms(100);
		address=ParameterRead[0];
		BaudRateNum=	ParameterRead[1];
		BaudRate=BaudRateArray[BaudRateNum];
		Warning=ParameterRead[2];
		Temperatuer_Up=ParameterRead[3];
		if(Temperatuer_Up>80)//Ϊ����
		{
				Temperatuer_Up=Temperatuer_Up-256;
		}
		Temperatuer_Down=ParameterRead[4];
		if(Temperatuer_Down>80)//Ϊ����
		{
				Temperatuer_Down=Temperatuer_Down-256;
		}
		Humtidy_Up=ParameterRead[5];
		Humtidy_Down=ParameterRead[6];
		C_F=ParameterRead[7];
		RemoteControl=ParameterRead[8];
		RecordMode=ParameterRead[9];
		StorageMode=ParameterRead[10];
		RecordTiming=ParameterRead[11];
		WarningTiming=ParameterRead[12];
		BatteryMonitor=ParameterRead[13];
		SensorSelected=ParameterRead[14];
		StorgeNum=ParameterRead[15];
		StorgeNum=(StorgeNum<<8)+ParameterRead[16];
		PartofEEPROM=ParameterRead[17];
		TemperatureHys=ParameterRead[18];//�¶����г�����
		HumtidyHys=ParameterRead[19];//ʪ�����г�����
		Temperature_4ma=ParameterRead[20];
		if(Temperature_4ma>80)//Ϊ����
		{
				Temperature_4ma=Temperature_4ma-256;
		}
		Temperature_20ma=ParameterRead[21];
		if(Temperature_20ma>80)//Ϊ����
		{
				Temperature_20ma=Temperature_20ma-256;
		}
		Humtidy_4ma=ParameterRead[22];
		Humtidy_20ma=ParameterRead[23];
		TemperatureCalibration=ParameterRead[24];
		if(TemperatureCalibration>100)//Ϊ����
		{
				TemperatureCalibration=TemperatureCalibration-256;
		}
		HumtidyCalibration=ParameterRead[25];
		if(HumtidyCalibration>100)//Ϊ����
		{
				HumtidyCalibration=HumtidyCalibration-256;
		}
		if((ParameterRead[26]==0||ParameterRead[26]==0xff)&&(ParameterRead[27]==0||ParameterRead[27]==0xff)&&(ParameterRead[28]==0||ParameterRead[28]==0xff)){
			Adjust_Tem_4ma=400;
		}
		/********************�Ӹ�λ����λ����Adjust_Tem_4ma*********************/
		else {Adjust_Tem_4ma=ParameterRead[26]+ParameterRead[27]*10+ParameterRead[28]*100;}
/********************�Ӹ�λ��ǧλ����Adjust_Tem_20ma*******************/
		if((ParameterRead[29]==0||ParameterRead[29]==0xff)&&(ParameterRead[30]==0||ParameterRead[30]==0xff)&&(ParameterRead[31]==0||ParameterRead[31]==0xff)&&(ParameterRead[32]==0||ParameterRead[32]==0xff)){
			Adjust_Tem_20ma=2000;
		}
		else{Adjust_Tem_20ma=ParameterRead[29]+10*ParameterRead[30]+100*ParameterRead[31]+1000*ParameterRead[32];}
/********************�Ӹ�λ����λ����Adjust_Hum_4ma*********************/
		if((ParameterRead[33]==0||ParameterRead[33]==0xff)&&(ParameterRead[34]==0||ParameterRead[34]==0xff)&&(ParameterRead[35]==0||ParameterRead[35]==0xff)){
			Adjust_Hum_4ma=400;
		}
		else{Adjust_Hum_4ma=ParameterRead[33]+10*ParameterRead[34]+100*ParameterRead[35];}
/********************�Ӹ�λ��ǧλ����Adjust_Hum_20ma******************/
		if((ParameterRead[36]==0||ParameterRead[36]==0xff)&&(ParameterRead[37]==0||ParameterRead[37]==0xff)&&(ParameterRead[38]==0||ParameterRead[38]==0xff)&&(ParameterRead[39]==0||ParameterRead[39]==0xff)){
			Adjust_Hum_20ma=2000;
		}
		else{Adjust_Hum_20ma=ParameterRead[36]+10*ParameterRead[37]+100*ParameterRead[38]+1000*ParameterRead[39];}
/********************�Ӹ�λ��ǧλ����Adjust_Tem_5v********************/
		if((ParameterRead[40]==0||ParameterRead[40]==0xff)&&(ParameterRead[41]==0||ParameterRead[41]==0xff)&&(ParameterRead[42]==0||ParameterRead[42]==0xff)&&(ParameterRead[43]==0||ParameterRead[43]==0xff)){
			Adjust_Tem_5v=2000;
		}
		else{Adjust_Tem_5v=ParameterRead[40]+10*ParameterRead[41]+100*ParameterRead[42]+1000*ParameterRead[43];}
/*******************�Ӹ�λ��ǧλ����Adjust_Hum_10v********************/
		if((ParameterRead[44]==0||ParameterRead[44]==0xff)&&(ParameterRead[45]==0||ParameterRead[45]==0xff)&&(ParameterRead[46]==0||ParameterRead[46]==0xff)&&(ParameterRead[47]==0||ParameterRead[47]==0xff)){
			Adjust_Hum_10v=2000;
		}
		else{Adjust_Hum_10v=ParameterRead[44]+10*ParameterRead[45]+100*ParameterRead[46]+1000*ParameterRead[47];}
		
		ReadParameters_wuxian();
}
//����ȷ�ϼ���ʱ����ʱ
void StorageParameters(void)
{
		uint16_t BUFFER_SIZE1;
		//BUFFER_SIZE1=20;
		NumDataRead=BUFFER_SIZE1;

		//BUFFER_SIZE1=20;

		ParameterStorage[0]=address;
		ParameterStorage[1]=BaudRateNum;
		ParameterStorage[2]=Warning;
		ParameterStorage[3]=Temperatuer_Up;
		ParameterStorage[4]=Temperatuer_Down;
		ParameterStorage[5]=Humtidy_Up;
		ParameterStorage[6]=Humtidy_Down;
		ParameterStorage[7]=C_F;
		ParameterStorage[8]=RemoteControl;
		ParameterStorage[9]=RecordMode;
		ParameterStorage[10]=StorageMode;
		ParameterStorage[11]=RecordTiming;
		ParameterStorage[12]=WarningTiming;
		ParameterStorage[13]=BatteryMonitor;
		ParameterStorage[14]=SensorSelected;
		ParameterStorage[15]=(StorgeNum&0xff00)>>8;//�Ѵ洢���ֽ����߰�λ
		ParameterStorage[16]=StorgeNum&0x00ff;//�Ѵ洢���ֽ����Ͱ�λ
		ParameterStorage[17]=PartofEEPROM;
		ParameterStorage[18]=TemperatureHys;//�¶ȳ�����
		ParameterStorage[19]=HumtidyHys;//ʪ�ȳ�����
		ParameterStorage[20]=Temperature_4ma;
		ParameterStorage[21]=Temperature_20ma;
		ParameterStorage[22]=Humtidy_4ma;
		ParameterStorage[23]=Humtidy_20ma;
		ParameterStorage[24]=TemperatureCalibration;
		ParameterStorage[25]=HumtidyCalibration;
/********************�Ӹ�λ����λ����Adjust_Tem_4ma*********************/
			ParameterStorage[26]=Adjust_Tem_4ma%100%10;
      ParameterStorage[27]=Adjust_Tem_4ma%100/10;
			ParameterStorage[28]=Adjust_Tem_4ma/100;
/********************�Ӹ�λ��ǧλ����Adjust_Tem_20ma*******************/
			ParameterStorage[29]=Adjust_Tem_20ma%1000%100%10;
			ParameterStorage[30]=((Adjust_Tem_20ma%1000)%100)/10;
			ParameterStorage[31]=(Adjust_Tem_20ma%1000)/100;
			ParameterStorage[32]=Adjust_Tem_20ma/1000;
/********************�Ӹ�λ����λ����Adjust_Hum_4ma*********************/
			ParameterStorage[33]=Adjust_Hum_4ma%100%10;
			ParameterStorage[34]=Adjust_Hum_4ma%100/10;
			ParameterStorage[35]=Adjust_Hum_4ma/100;
/********************�Ӹ�λ��ǧλ����Adjust_Hum_20ma******************/
			ParameterStorage[36]=Adjust_Hum_20ma%1000%100%10;
			ParameterStorage[37]=((Adjust_Hum_20ma%1000)%100)/10;
			ParameterStorage[38]=(Adjust_Hum_20ma%1000)/100;
			ParameterStorage[39]=Adjust_Hum_20ma/1000;
/********************�Ӹ�λ��ǧλ����Adjust_Tem_5v********************/
			ParameterStorage[40]=Adjust_Tem_5v%1000%100%10;
			ParameterStorage[41]=((Adjust_Tem_5v%1000)%100)/10;
			ParameterStorage[42]=(Adjust_Tem_5v%1000)/100;
			ParameterStorage[43]=Adjust_Tem_5v/1000;
/*******************�Ӹ�λ��ǧλ����Adjust_Tem_20v********************/
			ParameterStorage[44]=Adjust_Hum_10v%1000%100%10;
			ParameterStorage[45]=((Adjust_Hum_10v%1000)%100)/10;
			ParameterStorage[46]=(Adjust_Hum_10v%1000)/100;
			ParameterStorage[47]=Adjust_Hum_10v/1000;		
		BUFFER_SIZE1=50;
		EEPROM1024Part(0);
		//sEE_WriteBuffer((u8*)ParameterStorage,0 , BUFFER_SIZE1);
		AT24CXX_Write(0,(u8*)ParameterStorage,BUFFER_SIZE1);
		delay_ms(100);
		
		StorageParameters_wuxian();
}

void gpInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;//PC10���������﮵�ؿ���
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC,GPIO_Pin_10);//��ʼ����ʱ��ʹ�﮵�ؿ�������
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_1);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_5);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
	
}

void BeepInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
void Beep(int ms)
{
		GPIO_SetBits(GPIOA,GPIO_Pin_6);
		delay_ms(ms);
		GPIO_ResetBits(GPIOA,GPIO_Pin_6);
}

void ModeSwitch(int mode,int UP_Dowm)//UP_Dowm��0����С��1������
{
		TIM_Cmd(TIM2, DISABLE);
		//DisplayTime();
    switch(mode)
		{
			case 0:
			{
					break;
			}
			case 1://��ַ
			{
				//��ʾ��һ��
				Clr_Wendu();
				Display_Wendu_2(0, 0);//��ʾС��
				write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
				//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
				Display_Wendu_1(2, 1);//��ʾ��λ
				Display_Wendu_1(4, 1);//��ʾʮλ
				//��ʾ�ڶ���
				Clr_Shidu();
				Display_Shidu_3(26,DeviceID%100%10);//��ʾ��λ
				//write_addr_dat_n_1bit(28,0x00, 1);//����ʾС����
				Display_Shidu_3(28,  DeviceID%100/10);//��ʾʮλ
				Display_Shidu_3(30,  DeviceID/100);//��ʾ��λ	
				switch(UP_Dowm)
					{
						case 0:
									{
											
										   DeviceID--;
											if(DeviceID<1) 
											DeviceID=255;
											//��ʾ�ڶ���
											Clr_Shidu();
											Display_Shidu_3(26,DeviceID%100%10);//��ʾ��λ
											//write_addr_dat_n_1bit(28,0x00, 1);//����ʾС����
											Display_Shidu_3(28,  DeviceID%100/10);//��ʾʮλ
											Display_Shidu_3(30,  DeviceID/100);//��ʾ��λ	
											break;
									}
						case 1:
									{
											DeviceID++;
											if(DeviceID>255) 
											DeviceID=1;
											//��ʾ�ڶ���
											Display_Shidu_3(26,DeviceID%100%10);//��ʾ��λ
											//write_addr_dat_n_1bit(28,0x00, 1);//����ʾС����
											Display_Shidu_3(28,  DeviceID%100/10);//��ʾʮλ
											Display_Shidu_3(30,  DeviceID/100);//��ʾ��λ	
											break;
									}
					
					}
					break;
			}
			case 2://������
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 2);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 1);//��ʾ��λ
					Display_Wendu_1(4, 2);//��ʾʮλ
					//��ʾ�ڶ���
					Clr_Shidu();
					Display_Shidu_3(26,flag_ls_set%100%10);//��ʾ��λ
					//write_addr_dat_n_1bit(28,0x00, 1);//����ʾС����
					Display_Shidu_3(28,  flag_ls_set%100/10);//��ʾʮλ
					Display_Shidu_3(30,  flag_ls_set/100);//��ʾ��λ	
					switch(UP_Dowm)
					{
							case 0:
						{
//								BaudRateNum--;
//								if(BaudRateNum<0)
//								{BaudRateNum=4;}
//								BaudRate=BaudRateArray[BaudRateNum];
//								uart2_init(BaudRate);
								flag_ls_set--;
								if(flag_ls_set<1)
								{flag_ls_set=999;}
								//��ʾ�ڶ���
								Clr_Shidu();
								Display_Shidu_3(26,flag_ls_set%100%10);//��ʾ��λ
								//write_addr_dat_n_1bit(28,0x00, 1);//����ʾС����
								Display_Shidu_3(28,  flag_ls_set%100/10);//��ʾʮλ
								Display_Shidu_3(30,  flag_ls_set/100);//��ʾ��λ	
								break;
						}
						case 1:
						{
//								BaudRateNum++;
//								if(BaudRateNum>4)
//								{BaudRateNum=0;}
//								BaudRate=BaudRateArray[BaudRateNum];
//								uart2_init(BaudRate);
//								//��ʾ�ڶ���
//								Clr_Shidu();
//								Display_Shidu_3(26, BaudRate/100%10);//��ʾ��λ
//								//write_addr_dat_n_1bit(28,0x00, 1);//��ʾС����write_addr_dat_n_char
//								Display_Shidu_3(28,  BaudRate/100/10%10);//��ʾʮλ
//								Display_Shidu_3(30, BaudRate/10000);//��ʾ��λ	
								flag_ls_set++;
								if(flag_ls_set>999)
								{flag_ls_set=1;}
								//��ʾ�ڶ���
								Clr_Shidu();
								Display_Shidu_3(26,flag_ls_set%100%10);//��ʾ��λ
								//write_addr_dat_n_1bit(28,0x00, 1);//����ʾС����
								Display_Shidu_3(28,  flag_ls_set%100/10);//��ʾʮλ
								Display_Shidu_3(30,  flag_ls_set/100);//��ʾ��λ	
								break;
						}	
					}
					break;
			}
			case 3://��������/�ر�
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 8);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 7);//��ʾ��λ
					Display_Wendu_1(4, 3);//��ʾʮλ
					//��ʾ�ڶ���
					Clr_Others();
				if(Warning==1) 
				{
					//��ʾ�ڶ���
					Clr_Shidu();
					Display_Shidu_2(26,10);//��ʾ��λ
					Display_Shidu_3(28,0);//��ʾʮλ							
				}
				else 
				{
					//��ʾ�ڶ���
					Clr_Shidu();
					Display_Shidu_2(26,5);//��ʾ��λ
					Display_Shidu_2(28,5);//��ʾʮλ
					Display_Shidu_3(30, 0);//��ʾ��λ	
				}
					switch(UP_Dowm)
					{
						case 0:
									{
									if(Warning==0) 
									{
										Warning=1;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(26,10);//��ʾ��λ
										Display_Shidu_3(28,0);//��ʾʮλ							
									}
									else 
									{
										Warning=0;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(26,5);//��ʾ��λ
										Display_Shidu_2(28,5);//��ʾʮλ
										Display_Shidu_3(30, 0);//��ʾ��λ	
									}
									
									break;
									}//��ת
						case 1:
									{
									if(Warning==0) 
									{
										Warning=1;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(26,10);//��ʾ��λ
										Display_Shidu_3(28,0);//��ʾʮλ
									}
									else 
									{
										Warning=0;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(26,5);//��ʾ��λ
										Display_Shidu_2(28,5);//��ʾʮλ
										Display_Shidu_3(30, 0);//��ʾ��λ	
									}
									break;
									}
					}
					break;
			}
			case 4://�¶�����
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 11);//��ʾС��  H
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 3);//��ʾ��λ
					Display_Wendu_1(4, 4);//��ʾʮλ
					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
					if(Temperatuer_Up<0)
					{
					  write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
						Temperature_I=0-Temperatuer_Up;
					}
					else
					{
						Temperature_I=Temperatuer_Up;
						write_addr_dat_n_char(7,0x08, 0);//����ʾ����
					}
					Display_Shidu_3(26,0);//��ʾ��λ
					write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,Temperature_I%10);//��ʾ��λ
					Display_Shidu_3(30, Temperature_I/10);//��ʾʮλ	
					switch(UP_Dowm)
					{
						case 0:
									{
									if(Temperatuer_Down>=Temperatuer_Up)//���޴������޵����
									{
										//��ʾ�ڶ���Err
										Clr_Shidu();
										if(Temperatuer_Up<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											Temperature_I=0-Temperatuer_Up;
										}
										else
										{
											Temperature_I=Temperatuer_Up;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										
										//test ���
										write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										
										//test ���
										
										Display_Shidu_2(26,8);//��ʾ��λ
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_2(28,8);//��ʾ��λ
										Display_Shidu_2(30,6);//��ʾʮλ		
									}
									else
									{
										Temperatuer_Up--; 
										if(Temperatuer_Up<-40) 
										Temperatuer_Up=-40;
										//��ʾ�ڶ���
										Clr_Shidu();
										if(Temperatuer_Up<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											Temperature_I=0-Temperatuer_Up;
										}
										else
										{
											Temperature_I=Temperatuer_Up;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										Display_Shidu_3(26,0);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,Temperature_I%10);//��ʾ��λ
										Display_Shidu_3(30, Temperature_I/10);//��ʾʮλ	
									}
									break;
									}
						case 1:
									{
									Temperatuer_Up++;
									if(Temperatuer_Up>80)
									Temperatuer_Up=80;
									//��ʾ�ڶ���
									Clr_Shidu();
									//test  ���
									if(Temperatuer_Up<0)
									{
										write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
										Temperature_I=0-Temperatuer_Up;
									}
									else
									{
										Temperature_I=Temperatuer_Up;
										write_addr_dat_n_char(7,0x08, 0);//����ʾ����
									}
									//test  ���
									
									Display_Shidu_3(26,0);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,Temperature_I%10);//��ʾ��λ
									Display_Shidu_3(30, Temperature_I/10);//��ʾʮλ	
									break;
									}
					}
					break;
			}
			case 5://�¶�����  5C.L ��Ϊ4C.L
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 4);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 3);//��ʾ��λ
					//Display_Wendu_1(4, 5);//��ʾʮλ
				  Display_Wendu_1(4, 4);//��ʾʮλ
					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
					if(Temperatuer_Down<0)
					{
					  write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
						Temperature_O=0-Temperatuer_Down;
					}
					else
					{
						Temperature_O=Temperatuer_Down;
						write_addr_dat_n_char(7,0x08, 0);//����ʾ����
					}
					Display_Shidu_3(26,0);//��ʾ��λ
					write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,Temperature_O%10);//��ʾ��λ
					Display_Shidu_3(30,Temperature_O/10);//��ʾʮλ
					switch(UP_Dowm)
					{
						case 0:
									{
									Temperatuer_Down--; 
									if(Temperatuer_Down<-40)
									Temperatuer_Down=-40;
									//��ʾ�ڶ���
									Clr_Shidu();
									if(Temperatuer_Down<0)
									{
										write_addr_dat_n_char(7,0x08, 1);//��ʾ����
										Temperature_O=0-Temperatuer_Down;
									}
									else
									{
										Temperature_O=Temperatuer_Down;
										write_addr_dat_n_char(7,0x08, 0);//����ʾ����
									}
									Display_Shidu_3(26,0);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,Temperature_O%10);//��ʾ��λ
									Display_Shidu_3(30,Temperature_O/10);//��ʾʮλ	
									break;}
						case 1:
									{
									if(Temperatuer_Down>=Temperatuer_Up)//���޴������޵����
									{
									  //��ʾ�ڶ���
										Clr_Shidu();
										
										//test ���
										write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										
										//test ���
										
										Display_Shidu_2(26,8);//��ʾ��λ
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_2(28,8);//��ʾ��λ
										Display_Shidu_2(30,6);//��ʾʮλ					
									}
									else
									{
										Temperatuer_Down++; 
										if(Temperatuer_Down>80) 
										Temperatuer_Down=80;
										//��ʾ�ڶ���
										Clr_Shidu();
										if(Temperatuer_Down<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����
											Temperature_O=0-Temperatuer_Down;
										}
										else
										{
											Temperature_O=Temperatuer_Down;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										Display_Shidu_3(26,0);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,Temperature_O%10);//��ʾ��λ
										Display_Shidu_3(30,Temperature_O/10);//��ʾʮλ	
									}										
									break;
									}
					}
					break;
			}
			case 7://ʪ������  6--7  6H.H--4H.H
			{
					write_addr_dat_n_char(7,0x08, 0);//����ʾ����
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 11);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 11);//��ʾ��λ
					//Display_Wendu_1(4, 6);//��ʾʮλ
				  Display_Wendu_1(4, 4);//��ʾʮλ  �޸�
					//Display_CHARS(huashidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
				
				  //test ���
					write_addr_dat_n_char(7,0x08, 0);//����ʾ����
					
					//test ���
				
					Display_Shidu_1(26,0);//��ʾ��λ
				  write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,Humtidy_Up%10);//��ʾ��λ
					Display_Shidu_3(30,Humtidy_Up/10);//��ʾʮλ
					switch(UP_Dowm)
					{
						case 0:
									{
									if(Humtidy_Up<=Humtidy_Down)//����С�����޵Ĵ������
									{
										//��ʾ�ڶ���
										Clr_Shidu();
										
										//test ���
										write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										
										//test ���
										
										
										
										Display_Shidu_2(26,8);//��ʾ��λ
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_2(28,8);//��ʾ��λ
										Display_Shidu_2(30,6);//��ʾʮλ									
									}
									else
									{
										Humtidy_Up--; 
										if(Humtidy_Up<0) 
										Humtidy_Up=1;
										//��ʾ�ڶ���
										Clr_Shidu();
										
										//test ���
										write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										
										//test ���
										
										
										Display_Shidu_1(26,0);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,Humtidy_Up%10);//��ʾ��λ
										Display_Shidu_3(30,Humtidy_Up/10);//��ʾʮλ
									}
									break;
									}
						case 1:
									{
									Humtidy_Up++; 
									if(Humtidy_Up>99) 
									Humtidy_Up=99;
									//��ʾ�ڶ���
									Clr_Shidu();
									//test ���
									write_addr_dat_n_char(7,0x08, 0);//����ʾ����
									
									//test ���
									
									
									Display_Shidu_1(26,0);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,Humtidy_Up%10);//��ʾ��λ
									Display_Shidu_3(30,Humtidy_Up/10);//��ʾʮλ
									break;
									}
					}
					break;
			}
			case 8://ʪ������  7--8  7H.L--4H.L
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 4);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 11);//��ʾ��λ
					//Display_Wendu_1(4, 7);//��ʾʮλ
				  Display_Wendu_1(4, 4);//��ʾʮλ  �޸�
					//Display_CHARS(huashidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
				
				  //test ���
					write_addr_dat_n_char(7,0x08, 0);//����ʾ����
					
					//test ���
				
				
				
				
					Display_Shidu_1(26,0);//��ʾ��λ
				  write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,Humtidy_Down%10);//��ʾ��λ
					Display_Shidu_3(30,Humtidy_Down/10);//��ʾʮλ
					switch(UP_Dowm)
					{
						case 0:
									{
									Humtidy_Down--; 
									if(Humtidy_Down<0)
									Humtidy_Down=0;
									//��ʾ�ڶ���
									Clr_Shidu();
									
									//test ���
									write_addr_dat_n_char(7,0x08, 0);//����ʾ����
									
									//test ���
									
									Display_Shidu_1(26,0);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,Humtidy_Down%10);//��ʾ��λ
									Display_Shidu_3(30,Humtidy_Down/10);//��ʾʮλ
									break;
									}
						case 1:
									{
									if(Humtidy_Up<=Humtidy_Down)//����С�����޵Ĵ������
									{
										//��ʾ�ڶ���
										Clr_Shidu();
										
										//test ���
										write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										
										//test ���
										
										
										
										Display_Shidu_2(26,8);//��ʾ��λ
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_2(28,8);//��ʾ��λ
										Display_Shidu_2(30,6);//��ʾʮλ									
									}
									else
									{
										Humtidy_Down++; 
										if(Humtidy_Down>99) 
										Humtidy_Down=99;
										//��ʾ�ڶ���
										Clr_Shidu();
										//test ���
										write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										
										//test ���
										
										
										Display_Shidu_1(26,0);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,Humtidy_Down%10);//��ʾ��λ
										Display_Shidu_3(30,Humtidy_Down/10);//��ʾʮλ									
									}	
									break;
									}
					}
					break;
			}
			case 12://���϶Ȼ��϶��л�  8--12   8C.F--6C.F
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 5);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 3);//��ʾ��λ
					//Display_Wendu_1(4, 8);//��ʾʮλ
				  Display_Wendu_1(4, 6);//��ʾʮλ
					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
					if(C_F==1)	
					{
						//��ʾ�ڶ���
						Display_Shidu_2(28,5);//��ʾF
					}
					else 
					{
						//��ʾ�ڶ���
						Display_Shidu_2(28,3);//��ʾC
					}	
					switch(UP_Dowm)
					{
						case 0:
									{
									if(C_F==0)	
										{
										C_F=1;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(28,5);//��ʾC
										}
									else 
										{
										C_F=0;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(28,3);//��ʾF
										}	
									break;
									}
						case 1:
									{
									if(C_F==0)	
										{
										C_F=1;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(28,5);//��ʾC
										}
									else 
										{
										C_F=0;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(28,3);//��ʾF
										}	
									break;
									}
					}
					break;
			}
			case 13://Զ�̿���ѡ��      9--13   9F.r--7F.r
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 8);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 5);//��ʾ��λ
					//Display_Wendu_1(4, 9);//��ʾʮλ
				  Display_Wendu_1(4, 7);//��ʾʮλ
					//��ʾ�ڶ���ON
					Clr_Shidu();
					if(RemoteControl==1) 
					{											
					//��ʾ�ڶ���ON
					Display_Shidu_2(26,10);
					//write_addr_dat_n_1bit(28,0x01, 1);
					Display_Shidu_3(28,  0);//
					}
					else 
					{
					//��ʾ�ڶ���OFF
					Display_Shidu_2(26,5);
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_2(28,  5);
					Display_Shidu_3(30,0);
					}
					switch(UP_Dowm)
					{
						case 0:
									{
											if(RemoteControl==0) 
											{											
											RemoteControl=1;
											//��ʾ�ڶ���ON
											Clr_Shidu();
											Display_Shidu_2(26,10);
											//write_addr_dat_n_1bit(28,0x01, 1);
											Display_Shidu_3(28,  0);//
											}

											else 
											{
											RemoteControl=0;
											//��ʾ�ڶ���OFF
											Clr_Shidu();
											Display_Shidu_2(26,5);
											//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
											Display_Shidu_2(28,  5);
											Display_Shidu_3(30,0);
											}

											break;
									}
						case 1:
									{
											if(RemoteControl==0) 
											{											
											RemoteControl=1;
											//��ʾ�ڶ���ON
											Clr_Shidu();
											Display_Shidu_2(26,10);
											//write_addr_dat_n_1bit(28,0x01, 1);
											Display_Shidu_3(28,  0);//
											//Display_Shidu_1(30, BaudRate/10000);//��ʾ��λ	
											}

											else 
											{
											RemoteControl=0;
											//��ʾ�ڶ���OFF
											Clr_Shidu();
											Display_Shidu_2(26,5);
											//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
											Display_Shidu_2(28,  5);
											Display_Shidu_3(30,0);
											}

											break;
									}
					}
					break;
			}
			case 14://��¼�Ƿ���      10--14  CO.1--8C.1
			{
					/*
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 1);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 0);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
				  */
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 1);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 3);//��ʾ��λ
					Display_Wendu_1(4, 8);//��ʾʮλ
					//��ʾ�ڶ���ON
					Clr_Shidu();
					if(RecordMode==0)
					{
					//��ʾ�ڶ���ON
							Display_Shidu_2(26,10);
							//write_addr_dat_n_1bit(28,0x01, 1);
							Display_Shidu_3(28,  0);//
					}
					if(RecordMode==1)
					{
						//��ʾ�ڶ���OFF
							Display_Shidu_2(26,5);
							Display_Shidu_2(28,  5);
							Display_Shidu_3(30,0);
					}
					switch(UP_Dowm)
					{
						case 0:
									{
									RecordMode--;
									if(RecordMode<0) 
									RecordMode=1;
									if(RecordMode==0)
									{
									//��ʾ�ڶ���ON
											Clr_Shidu();
											Display_Shidu_2(26,10);
											//write_addr_dat_n_1bit(28,0x01, 1);
											Display_Shidu_3(28,  0);//
									}
									if(RecordMode==1)
									{
										//��ʾ�ڶ���OFF
											Clr_Shidu();
											Display_Shidu_2(26,5);
											Display_Shidu_2(28,  5);
											Display_Shidu_3(30,0);
									}
									break;
									}
						case 1:
									{
									RecordMode++;
									if(RecordMode>1) 
									RecordMode=0;
									if(RecordMode==0)
									{
											//��ʾ�ڶ���ON
											Clr_Shidu();
											Display_Shidu_2(26,10);
											//write_addr_dat_n_1bit(28,0x01, 1);
											Display_Shidu_3(28,  0);//
									}
									if(RecordMode==1)
									{
										//��ʾ�ڶ���OFF
											Clr_Shidu();
											Display_Shidu_2(26,5);
											Display_Shidu_2(28,  5);
											Display_Shidu_3(30,0);
									}
									break;}
					}
					break;
			}
			case 15://��¼ģʽ          11--15  CO.2--8C.2
			{
					/*
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 2);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 0);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
				  */
				   //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 2);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 3);//��ʾ��λ
					Display_Wendu_1(4, 8);//��ʾʮλ
				
				
				
					//��ʾ�ڶ���
					Clr_Shidu();
						if(StorageMode==1)
						{
							//��ʾ�ڶ���
							Display_Shidu_2(26,3);//��ʾ��λ
							Display_Shidu_2(28,9);//��ʾ��λ
							Display_Shidu_2(30,3);//��ʾʮλ
						}
						else 
					  {
							//��ʾ�ڶ���
							Display_Shidu_2(26,0);//��ʾ��λ
							Display_Shidu_2(28,8);//��ʾ��λ
							Display_Shidu_3(30,0);//��ʾʮλ
						}
					switch(UP_Dowm)
					{
						case 0:
							{
						if(StorageMode==0)
						{
							StorageMode=1;
							//��ʾ�ڶ���
							Clr_Shidu();
							Display_Shidu_2(26,3);//��ʾ��λ
							Display_Shidu_2(28,9);//��ʾ��λ
							Display_Shidu_2(30,3);//��ʾʮλ
						}
						else 
					  {
							StorageMode=0;//Ĭ��
							//��ʾ�ڶ���
							Clr_Shidu();
							Display_Shidu_2(26,0);//��ʾ��λ
							Display_Shidu_2(28,8);//��ʾ��λ
							Display_Shidu_3(30,0);//��ʾʮλ
						}
						break;
						}
						case 1:
						{
						if(StorageMode==0)
						{
							StorageMode=1;
							//��ʾ�ڶ���
							Clr_Shidu();
							Display_Shidu_2(26,3);//��ʾ��λ
							//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
							Display_Shidu_2(28,9);//��ʾ��λ
							Display_Shidu_2(30,3);//��ʾʮλ
						}
						else 
					  {
							StorageMode=0;
							//��ʾ�ڶ���
							Clr_Shidu();
							Display_Shidu_2(26,0);//��ʾ��λ
							//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
							Display_Shidu_2(28,8);//��ʾ��λ
							Display_Shidu_3(30,0);//��ʾʮλ
						}
						break;
						}
					}
					break;
			}
			case 16://�����洢ʱ��      12--16  CO.3--8C.3
			{
					/*
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 3);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 0);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
				  */
				   //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 3);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 3);//��ʾ��λ
					Display_Wendu_1(4, 8);//��ʾʮλ
				
				
					//��ʾ�ڶ���
					Clr_Shidu();
					Display_Shidu_3(26,RecordTiming%10);//��ʾ��λ
					Display_Shidu_3(28,RecordTiming/10%10);//��ʾʮλ
					Display_Shidu_3(30,RecordTiming/100);//��ʾ��λ
					switch(UP_Dowm)
					{
						case 0:
									{
									RecordTiming--;
									if(RecordTiming<1) 
									RecordTiming=600;
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_3(26,RecordTiming%10);//��ʾ��λ
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_3(28,RecordTiming/10%10);//��ʾʮλ
									Display_Shidu_3(30,RecordTiming/100);//��ʾ��λ
									break;
									}
						case 1:
									{RecordTiming++;
									if(RecordTiming>600) 
									RecordTiming=1;
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_3(26,RecordTiming%10);//��ʾ��λ
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_3(28,RecordTiming/10%10);//��ʾʮλ
									Display_Shidu_3(30,RecordTiming/100);//��ʾ��λ
									break;
									}
					}
					break;
			}
			case 17://�����洢ʱ��      13--17  CO.4--8C.4
			{
					/*
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 4);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 0);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
				  */
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 4);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 3);//��ʾ��λ
					Display_Wendu_1(4, 8);//��ʾʮλ
				
				
				
					//��ʾ�ڶ���
					Clr_Shidu();
					Display_Shidu_3(26,WarningTiming%10);//��ʾ��λ
					Display_Shidu_3(28,WarningTiming/10%10);//��ʾʮλ
					Display_Shidu_3(30,WarningTiming/100);//��ʾ��λ
					switch(UP_Dowm)
					{
						case 0:{
						WarningTiming--;
						if(WarningTiming<1) 
						WarningTiming=600;
						//��ʾ�ڶ���
						Clr_Shidu();
						Display_Shidu_3(26,WarningTiming%10);//��ʾ��λ
						Display_Shidu_3(28,WarningTiming/10%10);//��ʾʮλ
						Display_Shidu_3(30,WarningTiming/100);//��ʾ��λ
						break;}
						case 1:{
						WarningTiming++;
						if(WarningTiming>600) 
						WarningTiming=1;
						//��ʾ�ڶ���
						Clr_Shidu();
						Display_Shidu_3(26,WarningTiming%10);//��ʾ��λ
						Display_Shidu_3(28,WarningTiming/10%10);//��ʾʮλ
						Display_Shidu_3(30,WarningTiming/100);//��ʾ��λ
						break;}
					}
					break;
			}
			case 18://��ʷ�������C05   14--18  CO.5--8C.5
			{
					/*
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 5);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 0);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
				  */
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 5);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 3);//��ʾ��λ
					Display_Wendu_1(4, 8);//��ʾʮλ
				
					//��ʾ�ڶ���
					Clr_Shidu();
					
				
					Display_Shidu_3(26,DeleteData%10);//��ʾ��λ
					//Display_Shidu_3(28,DeleteData/10%10);//��ʾʮλ
					switch(UP_Dowm)
					{
						case 0:{
						DeleteData--;
						if(DeleteData<0)
							DeleteData=1;
						//��ʾ�ڶ���
						Clr_Shidu();
						Display_Shidu_3(26,DeleteData%10);//��ʾ��λ
						//Display_Shidu_3(28,DeleteData/10%10);//��ʾʮλ
						break;}
						case 1:{
						DeleteData++;
						if(DeleteData>1)
							DeleteData=0;
						//��ʾ�ڶ���
						Clr_Shidu();
						Display_Shidu_3(26,DeleteData%10);//��ʾ��λ
						//Display_Shidu_3(28,DeleteData/10%10);//��ʾʮλ
						break;}
					}
					break;
			}
			case 6://�¶ȳ���������C06  15--6  CO.6--4C.d
			{
					//��ʾ��һ��
					/*
				  Clr_Wendu();
					Display_Wendu_1(0, 6);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 0);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
				  */
				  //��ʾ��һ��
					Clr_Wendu();
				  
					 
				  //�������и���Ҳȥ������
					write_addr_dat_n_char(7,0x08, 0);//5G
					write_addr_dat_n_char(10,0x08, 0);//5BC
				  //�������и���Ҳȥ������
					

					Display_Wendu_2(0, 0);//��ʾС��  d
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 3);//��ʾ��λ
					Display_Wendu_1(4, 4);//��ʾʮλ
					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
					Display_Shidu_3(26,TemperatureHys/1%10);//��ʾ��λ
					write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,TemperatureHys/10%10);//��ʾ��λ
					Display_Shidu_3(30,TemperatureHys/100);//��ʾʮλ
					switch(UP_Dowm)
					{
						case 0:
									{
									TemperatureHys--; 
									if(TemperatureHys<0)
									TemperatureHys=0;
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_3(26,TemperatureHys/1%10);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,TemperatureHys/10%10);//��ʾ��λ
									Display_Shidu_3(30,TemperatureHys/100);//��ʾʮλ	
									break;}
						case 1:
									{
										TemperatureHys++; 
										//if(TemperatureHys>120) //Temperatuer_Down>Temperatuer_Up
										//TemperatureHys=120;
										if(TemperatureHys>((Temperatuer_Up-Temperatuer_Down)/3*10))
										TemperatureHys=((Temperatuer_Up-Temperatuer_Down)/3*10);
										
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_3(26,TemperatureHys/1%10);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,TemperatureHys/10%10);//��ʾ��λ
										Display_Shidu_3(30,TemperatureHys/100);//��ʾʮλ	
									break;
									}
					}
					break;
			}

			case 9://ʪ�ȳ���������C07 16--9   CO.7--4H.d
			{
					/*
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 7);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 0);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
					 write_addr_dat_n_char(28,0x01, HumtidyHys%10);//P2
				*/
				//��ʾ��һ��
				 //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 0);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 11);//��ʾ��λ
					//Display_Wendu_1(4, 7);//��ʾʮλ
				  Display_Wendu_1(4, 4);//��ʾʮλ  �޸�
          write_addr_dat_n_char(28,0x01, HumtidyHys%10);//P2
				

					//��ʾ�ڶ���
					Clr_Shidu();
					Display_Shidu_1(26,0);//��ʾ��λ
				//	write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,HumtidyHys/10%10);//��ʾ��λ
					Display_Shidu_3(30,HumtidyHys/100);//��ʾʮλ
					switch(UP_Dowm)
					{
						case 0:
									{
									HumtidyHys--; 
									if(HumtidyHys<0)
									HumtidyHys=0;
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_1(26,HumtidyHys%10);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,HumtidyHys/10%10);//��ʾ��λ
									Display_Shidu_3(30,HumtidyHys/100);//��ʾʮλ	
									break;}
						case 1:
									{

										HumtidyHys++; 
										//if(HumtidyHys>100) //Humtidy_Up<Humtidy_Down
										//HumtidyHys=100;
										
										if(HumtidyHys>((Humtidy_Up-Humtidy_Down)/3*10))
										HumtidyHys=((Humtidy_Up-Humtidy_Down)/3*10);
										
										
										
										
										
										
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_1(26,HumtidyHys%10);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,HumtidyHys/10%10);//��ʾ��λ
										Display_Shidu_3(30,HumtidyHys/100);//��ʾʮλ									
									break;
									}
					}
					break;
			}

			case 19://IP1
			{
					/*
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 8);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 0);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
				  */
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 0);//��ʾС��
					write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					Display_Wendu_2(2, 3);//��ʾ��λ
					Display_Wendu_1(4, 9);//��ʾʮλ
				
					Clr_Shidu();
					Display_Shidu_3(26,ip1%100%10);//��ʾ��λ
					Display_Shidu_3(28,ip1%100/10);//��ʾʮλ
					Display_Shidu_3(30,ip1/100);//��ʾ��λ	
					
					switch(UP_Dowm)
					{
						case 0:
									{
											ip1--;
											if(ip1<1) 
												ip1=255;
											//��ʾ�ڶ���
											Clr_Shidu();
											Display_Shidu_3(26,ip1%100%10);//��ʾ��λ
											Display_Shidu_3(28,ip1%100/10);//��ʾʮλ
											Display_Shidu_3(30,ip1/100);//��ʾ��λ	
											break;
									}
						case 1:
									{
											ip1++;
											if(ip1>255) 
												ip1=1;
											//��ʾ�ڶ���
											Clr_Shidu();
											Display_Shidu_3(26,ip1%100%10);//��ʾ��λ
											Display_Shidu_3(28,ip1%100/10);//��ʾʮλ
											Display_Shidu_3(30,ip1/100);//��ʾ��λ	
											break;
									}
					
					}
					break;
//					//��ʾ�ڶ���
//					Clr_Shidu();
//					if(Temperature_4ma<0)
//					{
//					  write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//						TemperatureDisplay_4ma=0-Temperature_4ma;
//					}
//					else
//					{
//						TemperatureDisplay_4ma=Temperature_4ma;
//						write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//					}
//					Display_Shidu_3(26,0);//��ʾ��λ
//					write_addr_dat_n_char(28,0x01, 1);//P2
//					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//					Display_Shidu_1(28,TemperatureDisplay_4ma%10);//��ʾ��λ
//					Display_Shidu_3(30, TemperatureDisplay_4ma/10);//��ʾʮλ	
//			/*		*/
//					
//					/*
//					switch(UP_Dowm)
//					{
//						case 0:
//									{
//									
//									if(Temperature_4ma>Temperature_20ma)//���޴������޵����
//									{
//										//��ʾ�ڶ���Err
//										Clr_Shidu();
//										if(Temperature_4ma<0)
//										{
//											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//											TemperatureDisplay_4ma=0-Temperature_4ma;
//										}
//										else
//										{
//											TemperatureDisplay_4ma=Temperature_4ma;
//											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//										}
//										Display_Shidu_2(26,8);//��ʾ��λ
//										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//										Display_Shidu_2(28,8);//��ʾ��λ
//										Display_Shidu_2(30,6);//��ʾʮλ		
//									}
//										
//									else
//									{
//										Temperature_4ma--; 
//										if(Temperature_4ma<-40) 
//										Temperature_4ma=-40;
//										//��ʾ�ڶ���
//										Clr_Shidu();
//										if(Temperature_4ma<0)
//										{
//											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//											TemperatureDisplay_4ma=0-Temperature_4ma;
//										}
//										else
//										{
//											TemperatureDisplay_4ma=Temperature_4ma;
//											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//										}
//										Display_Shidu_3(26,0);//��ʾ��λ
//										write_addr_dat_n_char(28,0x01, 1);//P2
//										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//										Display_Shidu_1(28,TemperatureDisplay_4ma%10);//��ʾ��λ
//										Display_Shidu_3(30, TemperatureDisplay_4ma/10);//��ʾʮλ	
//									}
//									break;
//									}
//						case 1:
//									{
//									Temperature_4ma++;
//									if(Temperature_4ma>80)
//									Temperature_4ma=80;
//									//��ʾ�ڶ���
//									Clr_Shidu();
//									Display_Shidu_3(26,0);//��ʾ��λ
//									write_addr_dat_n_char(28,0x01, 1);//P2
//									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//									Display_Shidu_1(28,TemperatureDisplay_4ma%10);//��ʾ��λ
//									Display_Shidu_3(30, TemperatureDisplay_4ma/10);//��ʾʮλ	
//									break;
//									}
//					}
//					
//					*/
//					switch(UP_Dowm)
//					{
//						case 0:
//									{
//									/*
//									if(Temperature_4ma>Temperature_20ma)//���޴������޵����
//									{
//										//��ʾ�ڶ���Err
//										Clr_Shidu();
//										if(Temperature_4ma<0)
//										{
//											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//											TemperatureDisplay_4ma=0-Temperature_4ma;
//										}
//										else
//										{
//											TemperatureDisplay_4ma=Temperature_4ma;
//											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//										}
//										Display_Shidu_2(26,8);//��ʾ��λ
//										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//										Display_Shidu_2(28,8);//��ʾ��λ
//										Display_Shidu_2(30,6);//��ʾʮλ		
//									}
//									*/	
//									//else
//											{
//												Temperature_4ma--; 
//												if(Temperature_4ma<-40) 
//												Temperature_4ma=-40;
//												
//											
//												//��ʾ�ڶ���
//												Clr_Shidu();
//												if(Temperature_4ma<0)
//												{
//													write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//													TemperatureDisplay_4ma=0-Temperature_4ma;
//												}
//												else
//												{
//													TemperatureDisplay_4ma=Temperature_4ma;
//													write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//												}
//												Display_Shidu_3(26,0);//��ʾ��λ
//												write_addr_dat_n_char(28,0x01, 1);//P2
//												//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//												Display_Shidu_1(28,TemperatureDisplay_4ma%10);//��ʾ��λ
//												Display_Shidu_3(30, TemperatureDisplay_4ma/10);//��ʾʮλ	
//											/*		*/
//											}
//											break;
//									}
//						case 1:
//									{
//											if(Temperature_4ma>=Temperature_20ma)//���޴������޵����
//											{	
//												Clr_Shidu();
//												if(Temperature_4ma<0)
//												{
//													write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//													TemperatureDisplay_4ma=0-Temperature_4ma;
//												}
//												else
//												{
//													TemperatureDisplay_4ma=Temperature_4ma;
//													write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//												}
//												//test ���
//												write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//												
//												//test ���
//												
//												
//												
//												Display_Shidu_2(26,8);//��ʾ��λ
//												//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//												Display_Shidu_2(28,8);//��ʾ��λ
//												Display_Shidu_2(30,6);//��ʾʮλ			
//											}
//											else
//											{
//											Temperature_4ma++;
//											if(Temperature_4ma>80)
//											Temperature_4ma=80;
//											/*
//											//��ʾ�ڶ���
//											Clr_Shidu();
//											Display_Shidu_3(26,0);//��ʾ��λ
//											write_addr_dat_n_char(28,0x01, 1);//P2
//											//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//											Display_Shidu_1(28,TemperatureDisplay_4ma%10);//��ʾ��λ
//											Display_Shidu_3(30, TemperatureDisplay_4ma/10);//��ʾʮλ
//											*/	
//											//��ʾ�ڶ���
//											Clr_Shidu();
//											if(Temperature_4ma<0)
//											{
//												write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//												TemperatureDisplay_4ma=0-Temperature_4ma;
//											}
//											else
//											{
//												TemperatureDisplay_4ma=Temperature_4ma;
//												write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//											}
//											Display_Shidu_3(26,0);//��ʾ��λ
//											write_addr_dat_n_char(28,0x01, 1);//P2
//											//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//											Display_Shidu_1(28,TemperatureDisplay_4ma%10);//��ʾ��λ
//											Display_Shidu_3(30, TemperatureDisplay_4ma/10);//��ʾʮλ	
//											
//											}
//											break;
//									}
//					}		
			}
			case 20://IP2
			{
					Clr_Wendu();
					Display_Wendu_1(0, 1);//��ʾС��
					write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					Display_Wendu_2(2, 3);//��ʾ��λ
					Display_Wendu_1(4, 9);//��ʾʮλ
				
					Clr_Shidu();
					Display_Shidu_3(26,ip2%100%10);//��ʾ��λ
					Display_Shidu_3(28,ip2%100/10);//��ʾʮλ
					Display_Shidu_3(30,ip2/100);//��ʾ��λ	
					
					switch(UP_Dowm)
					{
						case 0:
									{
											ip2--;
											if(ip2<1) 
												ip2=255;
											//��ʾ�ڶ���
											Clr_Shidu();
											Display_Shidu_3(26,ip2%100%10);//��ʾ��λ
											Display_Shidu_3(28,ip2%100/10);//��ʾʮλ
											Display_Shidu_3(30,ip2/100);//��ʾ��λ	
											break;
									}
						case 1:
									{
											ip2++;
											if(ip2>255) 
												ip2=1;
											//��ʾ�ڶ���
											Clr_Shidu();
											Display_Shidu_3(26,ip2%100%10);//��ʾ��λ
											Display_Shidu_3(28,ip2%100/10);//��ʾʮλ
											Display_Shidu_3(30,ip2/100);//��ʾ��λ	
											break;
									}
					
					}
					break;
				
				
					/*
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 9);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 0);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
				  */
				   //��ʾ��һ��
//					Clr_Wendu();
//					Display_Wendu_1(0, 1);//��ʾС��
//					write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//					Display_Wendu_2(2, 3);//��ʾ��λ
//					Display_Wendu_1(4, 9);//��ʾʮλ
//				
//				
//					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
//				
//				 
//				  
//					//��ʾ�ڶ���
//					Clr_Shidu();
//					if(Temperature_20ma<0)
//					{
//					  write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//						TemperatureDisplay_20ma=0-Temperature_20ma;
//					}
//					else
//					{
//						TemperatureDisplay_20ma=Temperature_20ma;
//						write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//					}
//					Display_Shidu_3(26,0);//��ʾ��λ
//					write_addr_dat_n_char(28,0x01, 1);//P2
//					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//					Display_Shidu_1(28,TemperatureDisplay_20ma%10);//��ʾ��λ
//					Display_Shidu_3(30, TemperatureDisplay_20ma/10);//��ʾʮλ	
//		 /*			switch(UP_Dowm)
//					{
//						case 0:
//									{
//									if(Temperature_4ma>Temperature_20ma)//���޴������޵����
//									{
//										//��ʾ�ڶ���Err
//										Clr_Shidu();
//										if(Temperature_20ma<0)
//										{
//											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//											TemperatureDisplay_20ma=0-Temperature_20ma;
//										}
//										else
//										{
//											TemperatureDisplay_20ma=Temperature_20ma;
//											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//										}
//										Display_Shidu_2(26,8);//��ʾ��λ
//										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//										Display_Shidu_2(28,8);//��ʾ��λ
//										Display_Shidu_2(30,6);//��ʾʮλ		
//									}
//									else
//									{
//										Temperature_20ma--; 
//										if(Temperature_20ma<-40) 
//										Temperature_20ma=-40;
//										//��ʾ�ڶ���
//										Clr_Shidu();
//										if(Temperature_20ma<0)
//										{
//											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//											TemperatureDisplay_20ma=0-Temperature_20ma;
//										}
//										else
//										{
//											TemperatureDisplay_20ma=Temperature_20ma;
//											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//										}
//										Display_Shidu_3(26,0);//��ʾ��λ
//										write_addr_dat_n_char(28,0x01, 1);//P2
//										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//										Display_Shidu_1(28,TemperatureDisplay_20ma%10);//��ʾ��λ
//										Display_Shidu_3(30, TemperatureDisplay_20ma/10);//��ʾʮλ	
//									}
//									break;
//									}
//						case 1:
//									{
//									Temperature_20ma++;
//									if(Temperature_20ma>80)
//									Temperature_20ma=80;
//									//��ʾ�ڶ���
//									Clr_Shidu();
//									Display_Shidu_3(26,0);//��ʾ��λ
//									write_addr_dat_n_char(28,0x01, 1);//P2
//									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//									Display_Shidu_1(28,TemperatureDisplay_20ma%10);//��ʾ��λ
//									Display_Shidu_3(30, TemperatureDisplay_20ma/10);//��ʾʮλ	
//									break;
//									}
//					}
//					
//					*/
//					switch(UP_Dowm)
//					{
//						case 0:
//									{
//									if(Temperature_4ma>=Temperature_20ma)//���޴������޵����
//									{
//										//��ʾ�ڶ���Err
//										Clr_Shidu();
//										if(Temperature_20ma<0)
//										{
//											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//											TemperatureDisplay_20ma=0-Temperature_20ma;
//										}
//										else
//										{
//											TemperatureDisplay_20ma=Temperature_20ma;
//											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//										}
//										//test ���
//										write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//										
//										//test ���
//										
//										Display_Shidu_2(26,8);//��ʾ��λ
//										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//										Display_Shidu_2(28,8);//��ʾ��λ
//										Display_Shidu_2(30,6);//��ʾʮλ		
//									}
//									else
//									{
//										Temperature_20ma--; 
//										if(Temperature_20ma<-40) 
//										Temperature_20ma=-40;
//										//��ʾ�ڶ���
//										Clr_Shidu();
//										if(Temperature_20ma<0)
//										{
//											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//											TemperatureDisplay_20ma=0-Temperature_20ma;
//										}
//										else
//										{
//											TemperatureDisplay_20ma=Temperature_20ma;
//											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//										}
//										Display_Shidu_3(26,0);//��ʾ��λ
//										write_addr_dat_n_char(28,0x01, 1);//P2
//										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//										Display_Shidu_1(28,TemperatureDisplay_20ma%10);//��ʾ��λ
//										Display_Shidu_3(30, TemperatureDisplay_20ma/10);//��ʾʮλ	
//									}
//									break;
//									}
//						case 1:
//									{
//									Temperature_20ma++;
//									if(Temperature_20ma>80)
//									Temperature_20ma=80;
//									//��ʾ�ڶ���
//									Clr_Shidu();
//									if(Temperature_20ma<0)
//									{
//										write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//										TemperatureDisplay_20ma=0-Temperature_20ma;
//									}
//									else
//									{
//										TemperatureDisplay_20ma=Temperature_20ma;
//										write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//									}
//									
//									
//									
//									
//									Display_Shidu_3(26,0);//��ʾ��λ
//									write_addr_dat_n_char(28,0x01, 1);//P2
//									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//									Display_Shidu_1(28,TemperatureDisplay_20ma%10);//��ʾ��λ
//									Display_Shidu_3(30, TemperatureDisplay_20ma/10);//��ʾʮλ	
//									break;
//									}
//					}
//					
//					break;
			}
			case 21://IP3
			{
					Clr_Wendu();
					Display_Wendu_1(0, 2);//��ʾС��
					write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					Display_Wendu_2(2, 3);//��ʾ��λ
					Display_Wendu_1(4, 9);//��ʾʮλ
				
					Clr_Shidu();
					Display_Shidu_3(26,ip3%100%10);//��ʾ��λ
					Display_Shidu_3(28,ip3%100/10);//��ʾʮλ
					Display_Shidu_3(30,ip3/100);//��ʾ��λ	
					
					switch(UP_Dowm)
					{
						case 0:
									{
											ip3--;
											if(ip3<1) 
												ip3=255;
											//��ʾ�ڶ���
											Clr_Shidu();
											Display_Shidu_3(26,ip3%100%10);//��ʾ��λ
											Display_Shidu_3(28,ip3%100/10);//��ʾʮλ
											Display_Shidu_3(30,ip3/100);//��ʾ��λ	
											break;
									}
						case 1:
									{
											ip3++;
											if(ip3>255) 
												ip3=1;
											//��ʾ�ڶ���
											Clr_Shidu();
											Display_Shidu_3(26,ip3%100%10);//��ʾ��λ
											Display_Shidu_3(28,ip3%100/10);//��ʾʮλ
											Display_Shidu_3(30,ip3/100);//��ʾ��λ	
											break;
									}
					
					}
					break;
//					/*
//				  //��ʾ��һ��
//					Clr_Wendu();
//					Display_Wendu_1(0, 0);//��ʾС��
//				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
//					Display_Wendu_1(2, 1);//��ʾ��λ
//					Display_Wendu_2(4, 3);//��ʾʮλ
//					//write_addr_dat_n_char(28,0x01, 1);//P2
//					//Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
//				  */
//				  //��ʾ��һ��
//					Clr_Wendu();
//					Display_Wendu_1(0, 2);//��ʾС��
//					write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//					Display_Wendu_2(2, 3);//��ʾ��λ
//					Display_Wendu_1(4, 9);//��ʾʮλ

//					//write_addr_dat_n_char(28,0x01, 1);//P2
//					//Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
//				
//					//��ʾ�ڶ���
//					Clr_Shidu();
//					if(Humtidy_4ma<0)
//					{
//					  write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//						HumtidyDisplay_4ma=0-Humtidy_4ma;
//					}
//					else
//					{
//						HumtidyDisplay_4ma=Humtidy_4ma;
//						write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//					}
//					Display_Shidu_1(26,0);//��ʾ��λ
//					write_addr_dat_n_char(28,0x01, 1);//P2
//					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//					Display_Shidu_1(28,HumtidyDisplay_4ma%10);//��ʾ��λ
//					Display_Shidu_3(30, HumtidyDisplay_4ma/10);//��ʾʮλ	
//					
//					/*
//					switch(UP_Dowm)
//					{
//						case 0:
//									{
//									if(Humtidy_4ma>Humtidy_20ma)//���޴������޵����
//									{
//										//��ʾ�ڶ���Err
//										Clr_Shidu();
//										if(Humtidy_4ma<0)
//										{
//											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//											HumtidyDisplay_4ma=0-Humtidy_4ma;
//										}
//										else
//										{
//											HumtidyDisplay_4ma=Humtidy_4ma;
//											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//										}
//										Display_Shidu_2(26,8);//��ʾ��λ
//										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//										Display_Shidu_2(28,8);//��ʾ��λ
//										Display_Shidu_2(30,6);//��ʾʮλ		
//									}
//									else
//									{
//										Humtidy_4ma--; 
//										if(Humtidy_4ma<0) 
//										Humtidy_4ma=0;
//										//��ʾ�ڶ���
//										Clr_Shidu();
//										if(Humtidy_4ma<0)
//										{
//											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//											HumtidyDisplay_4ma=0-Humtidy_4ma;
//										}
//										else
//										{
//											HumtidyDisplay_4ma=Humtidy_4ma;
//											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//										}
//										Display_Shidu_1(26,0);//��ʾ��λ
//										write_addr_dat_n_char(28,0x01, 1);//P2
//										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//										Display_Shidu_1(28,HumtidyDisplay_4ma%10);//��ʾ��λ
//										Display_Shidu_3(30, HumtidyDisplay_4ma/10);//��ʾʮλ	
//									}
//									break;
//									}
//						case 1:
//									{
//									Humtidy_4ma++;
//									if(Humtidy_4ma>99)
//									Humtidy_4ma=9;
//									//��ʾ�ڶ���
//									Clr_Shidu();
//									Display_Shidu_1(26,0);//��ʾ��λ
//									write_addr_dat_n_char(28,0x01, 1);//P2
//									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//									Display_Shidu_1(28,HumtidyDisplay_4ma%10);//��ʾ��λ
//									Display_Shidu_3(30, HumtidyDisplay_4ma/10);//��ʾʮλ	
//									break;
//									}
//					}
//					*/
//					switch(UP_Dowm)
//					{
//						case 0:
//									{
//								  /*
//									if(Humtidy_4ma>Humtidy_20ma)//���޴������޵����
//									{
//										//��ʾ�ڶ���Err
//										Clr_Shidu();
//										if(Humtidy_4ma<0)
//										{
//											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//											HumtidyDisplay_4ma=0-Humtidy_4ma;
//										}
//										else
//										{
//											HumtidyDisplay_4ma=Humtidy_4ma;
//											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//										}
//										Display_Shidu_2(26,8);//��ʾ��λ
//										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//										Display_Shidu_2(28,8);//��ʾ��λ
//										Display_Shidu_2(30,6);//��ʾʮλ		
//									}
//									else
//									*/
//									{
//										Humtidy_4ma--; 
//										if(Humtidy_4ma<0) 
//										Humtidy_4ma=0;
//										//��ʾ�ڶ���
//										Clr_Shidu();
//										if(Humtidy_4ma<0)
//										{
//											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//											HumtidyDisplay_4ma=0-Humtidy_4ma;
//										}
//										else
//										{
//											HumtidyDisplay_4ma=Humtidy_4ma;
//											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//										}
//										Display_Shidu_1(26,0);//��ʾ��λ
//										write_addr_dat_n_char(28,0x01, 1);//P2
//										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//										Display_Shidu_1(28,HumtidyDisplay_4ma%10);//��ʾ��λ
//										Display_Shidu_3(30, HumtidyDisplay_4ma/10);//��ʾʮλ	
//									}
//									break;
//									}
//						case 1:
//							    /*
//									{
//									Humtidy_4ma++;
//									if(Humtidy_4ma>99)
//									Humtidy_4ma=9;
//									//��ʾ�ڶ���
//									Clr_Shidu();
//									Display_Shidu_1(26,0);//��ʾ��λ
//									write_addr_dat_n_char(28,0x01, 1);//P2
//									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//									Display_Shidu_1(28,HumtidyDisplay_4ma%10);//��ʾ��λ
//									Display_Shidu_3(30, HumtidyDisplay_4ma/10);//��ʾʮλ	
//									break;
//									}
//						     */
//						      if(Humtidy_4ma>=Humtidy_20ma)//���޴������޵����
//									{
//										//��ʾ�ڶ���Err
//										Clr_Shidu();
//										if(Humtidy_4ma<0)
//										{
//											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//											HumtidyDisplay_4ma=0-Humtidy_4ma;
//										}
//										else
//										{
//											HumtidyDisplay_4ma=Humtidy_4ma;
//											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//										}
//										Display_Shidu_2(26,8);//��ʾ��λ
//										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//										Display_Shidu_2(28,8);//��ʾ��λ
//										Display_Shidu_2(30,6);//��ʾʮλ		
//									}
//									else
//						      {
//											
//										  Humtidy_4ma++;
//											if(Humtidy_4ma>99)
//											Humtidy_4ma=9;
//											//��ʾ�ڶ���
//											Clr_Shidu();
//											
//											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//											
//											Display_Shidu_1(26,0);//��ʾ��λ
//											write_addr_dat_n_char(28,0x01, 1);//P2
//											//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//											Display_Shidu_1(28,HumtidyDisplay_4ma%10);//��ʾ��λ
//											Display_Shidu_3(30, HumtidyDisplay_4ma/10);//��ʾʮλ	
//											break;
//									}
//					}
//					
//					
//					break;
			}
			case 22://IP4
			{
					Clr_Wendu();
					Display_Wendu_1(0, 3);//��ʾС��
					write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					Display_Wendu_2(2, 3);//��ʾ��λ
					Display_Wendu_1(4, 9);//��ʾʮλ
				
					Clr_Shidu();
					Display_Shidu_3(26,ip4%100%10);//��ʾ��λ
					Display_Shidu_3(28,ip4%100/10);//��ʾʮλ
					Display_Shidu_3(30,ip4/100);//��ʾ��λ	
					
					switch(UP_Dowm)
					{
						case 0:
									{
											ip4--;
											if(ip4<1) 
												ip4=255;
											//��ʾ�ڶ���
											Clr_Shidu();
											Display_Shidu_3(26,ip4%100%10);//��ʾ��λ
											Display_Shidu_3(28,ip4%100/10);//��ʾʮλ
											Display_Shidu_3(30,ip4/100);//��ʾ��λ	
											break;
									}
						case 1:
									{
											ip4++;
											if(ip4>255) 
												ip4=1;
											//��ʾ�ڶ���
											Clr_Shidu();
											Display_Shidu_3(26,ip4%100%10);//��ʾ��λ
											Display_Shidu_3(28,ip4%100/10);//��ʾʮλ
											Display_Shidu_3(30,ip4/100);//��ʾ��λ	
											break;
									}
					
					}
					break;
//					/*
//				  //��ʾ��һ��
//					Clr_Wendu();
//					Display_Wendu_1(0, 1);//��ʾС��
//				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
//					Display_Wendu_1(2, 1);//��ʾ��λ
//					Display_Wendu_2(4, 3);//��ʾʮλ
//					//write_addr_dat_n_char(28,0x01, 1);//P2
//					//Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
//				  */
//				  //��ʾ��һ��
//					Clr_Wendu();
//					Display_Wendu_1(0, 3);//��ʾС��
//					write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//					Display_Wendu_2(2, 3);//��ʾ��λ
//					Display_Wendu_1(4, 9);//��ʾʮλ
//				
//				
//					//write_addr_dat_n_char(28,0x01, 1);//P2
//					//Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
//				
//				
//					//��ʾ�ڶ���
//					Clr_Shidu();
//					if(Humtidy_20ma<0)
//					{
//					  write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//						HumtidyDisplay_20ma=0-Humtidy_20ma;
//					}
//					else
//					{
//						HumtidyDisplay_20ma=Humtidy_20ma;
//						write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//					}
//					Display_Shidu_1(26,0);//��ʾ��λ
//					write_addr_dat_n_char(28,0x01, 1);//P2
//					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//					Display_Shidu_1(28,HumtidyDisplay_20ma%10);//��ʾ��λ
//					Display_Shidu_3(30, HumtidyDisplay_20ma/10);//��ʾʮλ	
//					
//					
//					/*
//					switch(UP_Dowm)
//					{
//						case 0:
//									{
//									if(Humtidy_4ma>Humtidy_20ma)//���޴������޵����
//									{
//										//��ʾ�ڶ���Err
//										Clr_Shidu();
//										if(Humtidy_20ma<0)
//										{
//											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//											HumtidyDisplay_20ma=0-Humtidy_20ma;
//										}
//										else
//										{
//											HumtidyDisplay_20ma=Humtidy_20ma;
//											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//										}
//										Display_Shidu_2(26,8);//��ʾ��λ
//										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//										Display_Shidu_2(28,8);//��ʾ��λ
//										Display_Shidu_2(30,6);//��ʾʮλ		
//									}
//									else
//									{
//										Humtidy_20ma--; 
//										if(Humtidy_20ma<0) 
//										Humtidy_20ma=0;
//										//��ʾ�ڶ���
//										Clr_Shidu();
//										if(Humtidy_20ma<0)
//										{
//											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//											HumtidyDisplay_20ma=0-Humtidy_20ma;
//										}
//										else
//										{
//											HumtidyDisplay_20ma=Humtidy_20ma;
//											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//										}
//										Display_Shidu_1(26,0);//��ʾ��λ
//										write_addr_dat_n_char(28,0x01, 1);//P2
//										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//										Display_Shidu_1(28,HumtidyDisplay_20ma%10);//��ʾ��λ
//										Display_Shidu_3(30, HumtidyDisplay_20ma/10);//��ʾʮλ	
//									}
//									break;
//									}
//						case 1:
//									{
//									Humtidy_20ma++;
//									if(Humtidy_20ma>99)
//									Humtidy_20ma=99;
//									//��ʾ�ڶ���
//									Clr_Shidu();
//									Display_Shidu_1(26,0);//��ʾ��λ
//									write_addr_dat_n_char(28,0x01, 1);//P2
//									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//									Display_Shidu_1(28,HumtidyDisplay_20ma%10);//��ʾ��λ
//									Display_Shidu_3(30, HumtidyDisplay_20ma/10);//��ʾʮλ	
//									break;
//									}
//					}
//					*/
//					switch(UP_Dowm)
//					{
//						case 0:
//									{
//									if(Humtidy_4ma>=Humtidy_20ma)//���޴������޵����
//									{
//										//��ʾ�ڶ���Err
//										Clr_Shidu();
//										
//										/*
//										if(Humtidy_20ma<0)
//										{
//											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//											HumtidyDisplay_20ma=0-Humtidy_20ma;
//										}
//										else
//										{
//											HumtidyDisplay_20ma=Humtidy_20ma;
//											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//										}
//										*/
//										write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//										
//										
//										
//										Display_Shidu_2(26,8);//��ʾ��λ
//										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//										Display_Shidu_2(28,8);//��ʾ��λ
//										Display_Shidu_2(30,6);//��ʾʮλ		
//									}
//									else
//									{
//										Humtidy_20ma--; 
//										if(Humtidy_20ma<0) 
//										Humtidy_20ma=0;
//										//��ʾ�ڶ���
//										Clr_Shidu();
//										if(Humtidy_20ma<0)
//										{
//											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//											HumtidyDisplay_20ma=0-Humtidy_20ma;
//										}
//										else
//										{
//											HumtidyDisplay_20ma=Humtidy_20ma;
//											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//										}
//										Display_Shidu_1(26,0);//��ʾ��λ
//										write_addr_dat_n_char(28,0x01, 1);//P2
//										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//										Display_Shidu_1(28,HumtidyDisplay_20ma%10);//��ʾ��λ
//										Display_Shidu_3(30, HumtidyDisplay_20ma/10);//��ʾʮλ	
//									}
//									break;
//									}
//						case 1:
//									{
//									Humtidy_20ma++;
//									if(Humtidy_20ma>99)
//									Humtidy_20ma=99;
//									//��ʾ�ڶ���
//									Clr_Shidu();
//									
//									if(Humtidy_20ma<0)
//									{
//										write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
//										HumtidyDisplay_20ma=0-Humtidy_20ma;
//									}
//									else
//									{
//										HumtidyDisplay_20ma=Humtidy_20ma;
//										write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//									}
//									
//									
//									
//									
//									
//									Display_Shidu_1(26,0);//��ʾ��λ
//									write_addr_dat_n_char(28,0x01, 1);//P2
//									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
//									Display_Shidu_1(28,HumtidyDisplay_20ma%10);//��ʾ��λ
//									Display_Shidu_3(30, HumtidyDisplay_20ma/10);//��ʾʮλ	
//									break;
//									}
//					}
//					
//					
//					
//					break;
			}
			case 10://C12�¶�У׼      21--10 C1.2--5C.0
			{
					/*
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 2);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 1);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
				  */
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 0);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 3);//��ʾ��λ
					Display_Wendu_1(4, 5);//��ʾʮλ
					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
				
					//��ʾ�ڶ���
					Clr_Shidu();
					if(TemperatureCalibration<0)
					{
					  write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
						TemperatureCalibrationDisplay=0-TemperatureCalibration;
					}
					else
					{
						TemperatureCalibrationDisplay=TemperatureCalibration;
						write_addr_dat_n_char(7,0x08, 0);//����ʾ����
					}
					Display_Shidu_3(26,TemperatureCalibrationDisplay/1%10);//��ʾ��λ
					write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,TemperatureCalibrationDisplay/10%10);//��ʾ��λ
					Display_Shidu_3(30, TemperatureCalibrationDisplay/100);//��ʾʮλ	
					switch(UP_Dowm)
					{
						case 0:
									{
										TemperatureCalibration--; 
										if(TemperatureCalibration<-100) 
										TemperatureCalibration=-100;
										if(TemperatureCalibration<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											TemperatureCalibrationDisplay=0-TemperatureCalibration;
										}
										else
										{
											TemperatureCalibrationDisplay=TemperatureCalibration;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_3(26,TemperatureCalibrationDisplay%10);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,TemperatureCalibrationDisplay/10%10);//��ʾ��λ
										Display_Shidu_3(30, TemperatureCalibrationDisplay/100);//��ʾʮλ	
									break;
									}
						case 1:
									{
									TemperatureCalibration++;
									if(TemperatureCalibration>100)
									TemperatureCalibration=100;
									if(TemperatureCalibration<0)
									{
										write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
										TemperatureCalibrationDisplay=0-TemperatureCalibration;
									}
									else
									{
										TemperatureCalibrationDisplay=TemperatureCalibration;
										write_addr_dat_n_char(7,0x08, 0);//����ʾ����
									}
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_3(26,TemperatureCalibrationDisplay%10);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,TemperatureCalibrationDisplay/10%10);//��ʾ��λ
									Display_Shidu_3(30, TemperatureCalibrationDisplay/100);//��ʾʮλ	
									break;
									}
					}
					break;
			}
			case 11://C13ʪ��У׼      22--11 C1.3--5H.0
				{
					/*
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 3);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 1);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
					//write_addr_dat_n_char(28,0x01, 1);//P2
					//Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
					*/
					Clr_Wendu();
					Display_Wendu_1(0, 0);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 11);//��ʾ��λ
					Display_Wendu_1(4, 5);//��ʾʮλ
					//write_addr_dat_n_char(28,0x01, 1);//P2
					//Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
					if(HumtidyCalibration<0)
					{
					  write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
						HumtidyCalibrationDisplay=0-HumtidyCalibration;
					}
					else
					{
						HumtidyCalibrationDisplay=HumtidyCalibration;
						write_addr_dat_n_char(7,0x08, 0);//����ʾ����
					}
					Display_Shidu_1(26,HumtidyCalibrationDisplay/1%10);//��ʾ��λ
					write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,HumtidyCalibrationDisplay/10%10);//��ʾ��λ
					Display_Shidu_3(30, HumtidyCalibrationDisplay/100);//��ʾʮλ	
					switch(UP_Dowm)
					{
						case 0:
									{
										HumtidyCalibration--; 
										if(HumtidyCalibration<-100) 
										HumtidyCalibration=-100;
										if(HumtidyCalibration<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											HumtidyCalibrationDisplay=0-HumtidyCalibration;
										}
										else
										{
											HumtidyCalibrationDisplay=HumtidyCalibration;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_1(26,HumtidyCalibrationDisplay%10);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,HumtidyCalibrationDisplay/10%10);//��ʾ��λ
										Display_Shidu_3(30, HumtidyCalibrationDisplay/100);//��ʾʮλ	
									break;
									}
						case 1:
									{
									HumtidyCalibration++;
									if(HumtidyCalibration>100)
									HumtidyCalibration=100;
									if(HumtidyCalibration<0)
									{
										write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
										HumtidyCalibrationDisplay=0-HumtidyCalibration;
									}
									else
									{
										HumtidyCalibrationDisplay=HumtidyCalibration;
										write_addr_dat_n_char(7,0x08, 0);//����ʾ����
									}
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_1(26,HumtidyCalibrationDisplay%10);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,HumtidyCalibrationDisplay/10%10);//��ʾ��λ
									Display_Shidu_3(30, HumtidyCalibrationDisplay/100);//��ʾʮλ	
									break;
									}
					}
					break;
			}
			case 23://�˿ں�
			{
				//��ʾ��һ��
				Clr_Wendu();
				Display_Wendu_1(0, 4);//��ʾС��
				write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
				Display_Wendu_2(2, 3);//��ʾ��λ
				Display_Wendu_1(4, 9);//��ʾʮλ
				

				//��ʾ�ڶ���
				Clr_Shidu();
				Clr_Others();
				Display_Others_1(11, port/10000);
				Display_Others_1(13, port%10000/1000);
				Display_Others_1(15, port%1000/100);
				Display_Others_1(17, port%100/10);
				Display_Others_1(19, port%10);
//				Display_Shidu_3(26,port%100%10);//��ʾ��λ
//				//write_addr_dat_n_1bit(28,0x00, 1);//����ʾС����
//				Display_Shidu_3(28,  address%100/10);//��ʾʮλ
//				Display_Shidu_3(30,  address/100);//��ʾ��λ	
				switch(UP_Dowm)
					{
						case 0:
									{
											
										   port--;
											if(port<1) 
											port=65535;
											//��ʾ�ڶ���
											Clr_Others();
											Display_Others_1(11, port/10000);
											Display_Others_1(13, port%10000/1000);
											Display_Others_1(15, port%1000/100);
											Display_Others_1(17, port%100/10);
											Display_Others_1(19, port%10);
											break;
									}
						case 1:
									{
											port++;
											if(port>65535) 
											port=1;
											//��ʾ�ڶ���
											Clr_Others();
											Display_Others_1(11, port/10000);
											Display_Others_1(13, port%10000/1000);
											Display_Others_1(15, port%1000/100);
											Display_Others_1(17, port%100/10);
											Display_Others_1(19, port%10);
											break;
									}
					
					}
					break;
//				//��ʾ��ǰʣ�����
//					//��ʾ��һ��
//					Clr_Wendu();
//					Display_Wendu_2(0, 8);//��ʾС��
//					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
//					Display_Wendu_2(2, 7);//��ʾ��λ
//					Display_Wendu_2(4, 1);//��ʾʮλ
//					//Display_CHARS(huashidu,1);//��ʾ��ʪ�ȵķ���
//					write_addr_dat_n_char(7,0x08, 0);//����ʾ����
//					//��ʾ�ڶ���OFF
//					Clr_Shidu();
//					if((V_BAT*2)<Battery[0]){	
//						Display_Shidu_3(28,1);
//						Display_Shidu_3(26,0);}
//					else if((V_BAT*2)<Battery[1]&&(V_BAT*2)>=Battery[0]){
//						Display_Shidu_3(28,2);
//						Display_Shidu_3(26,0);}
//					else if((V_BAT*2)<Battery[2]&&(V_BAT*2)>=Battery[1]){
//						Display_Shidu_3(28,3);
//						Display_Shidu_3(26,0);}
//					else if((V_BAT*2)<Battery[3]&&(V_BAT*2)>=Battery[2]){
//						Display_Shidu_3(28,4);
//						Display_Shidu_3(26,0);}
//					else if((V_BAT*2)<Battery[4]&&(V_BAT*2)>=Battery[3]){
//						Display_Shidu_3(28,5);
//						Display_Shidu_3(26,0);}
//					else if((V_BAT*2)<Battery[5]&&(V_BAT*2)>=Battery[4]){
//						Display_Shidu_3(28,6);
//						Display_Shidu_3(26,0);}
//					else if((V_BAT*2)<Battery[6]&&(V_BAT*2)>=Battery[5]){
//						Display_Shidu_3(28,7);
//						Display_Shidu_3(26,0);}
//					else if((V_BAT*2)<Battery[7]&&(V_BAT*2)>=Battery[6]){
//						Display_Shidu_3(28,8);
//						Display_Shidu_3(26,0);}
//					else if((V_BAT*2)<Battery[8]&&(V_BAT*2)>=Battery[7]){
//						Display_Shidu_3(28,9);
//						Display_Shidu_3(26,0);}
//					else if((V_BAT*2)<Battery[9]&&(V_BAT*2)>=Battery[8]||(V_BAT*2>=Battery[9])){
//						Display_Shidu_3(30,1);
//						Display_Shidu_3(28,0);
//						Display_Shidu_3(26,0);}	

//					break;
			}
			case 24://������ѡ��
			{
					//��ʾ��һ��SEL
					Clr_Wendu();
					Display_Wendu_2(0, 4);//��ʾС��
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 6);//��ʾ��λ
					Display_Wendu_1(4, 5);//��ʾʮλ
					//Display_CHARS(huashidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Others();
					Clr_Shidu();
					Display_Shidu_3(26,SensorSelected);//��ʾ��
					switch(UP_Dowm)
					{
						case 0:{
						SensorSelected--;
						if(SensorSelected<1)
						SensorSelected=4;
						//��ʾ�ڶ���
						Clr_Shidu();
						Display_Shidu_3(26,SensorSelected);//��ʾ��
						break;}
						case 1:{
						SensorSelected++;
						if(SensorSelected>4)
						SensorSelected=1;
						//��ʾ�ڶ���
						Clr_Shidu();
						Display_Shidu_3(26,SensorSelected);//��ʾ��
						break;}
					}
					break;
			}
			case 25://��������
			{
					//ʱ������
					//��ʾ��һ��16
					Clr_Wendu();
					//Display_Wendu_2(0, 4);//��ʾС��
					Display_Wendu_1(2, 6);//��ʾ��λ
					Display_Wendu_1(4, 1);//��ʾʮλ
					Clr_Shidu();
					RTC_Control=1;
					OnlyDispalyTime=0;
					RTCTimeParametersInit=1;
					break;
			}
		}
			
}
void ModeSwitch1(int mode,int UP_Dowm)//UP_Dowm��0����С��1������
{
		//DisplayTime();
    switch(mode)
		{
			case 0:
			{
					break;
			}
			case 1://��ַ
			{
				//��ʾ��һ��
				Clr_Wendu();
				Display_Wendu_2(0, 0);//��ʾС��
				write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
				//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
				Display_Wendu_1(2, 1);//��ʾ��λ
				Display_Wendu_1(4, 1);//��ʾʮλ
				//��ʾ�ڶ���
				Clr_Shidu();
				Display_Shidu_3(26,address%100%10);//��ʾ��λ
				//write_addr_dat_n_1bit(28,0x00, 1);//����ʾС����
				Display_Shidu_3(28,  address%100/10);//��ʾʮλ
				Display_Shidu_3(30,  address/100);//��ʾ��λ	
				switch(UP_Dowm)
					{
						case 0:
									{
											
										   address--;
											if(address<1) 
											address=254;
											//��ʾ�ڶ���
											Clr_Shidu();
											Display_Shidu_3(26, address%100%10);//��ʾ��λ
											//write_addr_dat_n_1bit(28,0x00, 1);//����ʾС����
											Display_Shidu_3(28,  address%100/10);//��ʾʮλ
											Display_Shidu_3(30,  address/100);//��ʾ��λ	
											break;
									}
						case 1:
									{
											address++;
											if(address>254) 
											address=1;
											//��ʾ�ڶ���
											Clr_Shidu();
											Display_Shidu_3(26, address%100%10);//��ʾ��λ
											//write_addr_dat_n_1bit(28,0x00, 1);//����ʾС����
											Display_Shidu_3(28,  address%100/10);//��ʾʮλ
											Display_Shidu_3(30,  address/100);//��ʾ��λ	
											break;
									}
					
					}
					break;
			}
			case 2://������
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 2);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 1);//��ʾ��λ
					Display_Wendu_1(4, 2);//��ʾʮλ
					//��ʾ�ڶ���
					Clr_Shidu();
					BaudRate=BaudRateArray[BaudRateNum];
					Display_Shidu_3(26, BaudRate/100%10);//��ʾ��λ
					//write_addr_dat_n_1bit(28,0x00, 1);//��ʾС����
					Display_Shidu_3(28,  BaudRate/100/10%10);//��ʾʮλ
					Display_Shidu_3(30, BaudRate/10000);//��ʾ��λ	
					switch(UP_Dowm)
					{
							case 0:
						{
								BaudRateNum--;
								if(BaudRateNum<0)
								{BaudRateNum=4;}
								BaudRate=BaudRateArray[BaudRateNum];
								uart2_init(BaudRate);
								//��ʾ�ڶ���
								Clr_Shidu();
								Display_Shidu_3(26, BaudRate/100%10);//��ʾ��λ
								//write_addr_dat_n_1bit(28,0x00, 1);//����ʾС����
								Display_Shidu_3(28,  BaudRate/100/10%10);//��ʾʮλ
								Display_Shidu_3(30, BaudRate/10000);//��ʾ��λ	
								break;
						}
						case 1:
						{
								BaudRateNum++;
								if(BaudRateNum>4)
								{BaudRateNum=0;}
								BaudRate=BaudRateArray[BaudRateNum];
								uart2_init(BaudRate);
								//��ʾ�ڶ���
								Clr_Shidu();
								Display_Shidu_3(26, BaudRate/100%10);//��ʾ��λ
								//write_addr_dat_n_1bit(28,0x00, 1);//��ʾС����write_addr_dat_n_char
								Display_Shidu_3(28,  BaudRate/100/10%10);//��ʾʮλ
								Display_Shidu_3(30, BaudRate/10000);//��ʾ��λ	
								break;
						}	
					}
					break;
			}
			case 3://��������/�ر�
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 8);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 7);//��ʾ��λ
					Display_Wendu_1(4, 3);//��ʾʮλ
					//��ʾ�ڶ���
				if(Warning==1) 
				{
					//��ʾ�ڶ���
					Clr_Shidu();
					Display_Shidu_2(26,10);//��ʾ��λ
					Display_Shidu_3(28,0);//��ʾʮλ							
				}
				else 
				{
					//��ʾ�ڶ���
					Clr_Shidu();
					Display_Shidu_2(26,5);//��ʾ��λ
					Display_Shidu_2(28,5);//��ʾʮλ
					Display_Shidu_3(30, 0);//��ʾ��λ	
				}
					switch(UP_Dowm)
					{
						case 0:
									{
									if(Warning==0) 
									{
										Warning=1;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(26,10);//��ʾ��λ
										Display_Shidu_3(28,0);//��ʾʮλ							
									}
									else 
									{
										Warning=0;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(26,5);//��ʾ��λ
										Display_Shidu_2(28,5);//��ʾʮλ
										Display_Shidu_3(30, 0);//��ʾ��λ	
									}
									
									break;
									}//��ת
						case 1:
									{
									if(Warning==0) 
									{
										Warning=1;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(26,10);//��ʾ��λ
										Display_Shidu_3(28,0);//��ʾʮλ
									}
									else 
									{
										Warning=0;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(26,5);//��ʾ��λ
										Display_Shidu_2(28,5);//��ʾʮλ
										Display_Shidu_3(30, 0);//��ʾ��λ	
									}
									break;
									}
					}
					break;
			}
			case 4://�¶�����
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 11);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 3);//��ʾ��λ
					Display_Wendu_1(4, 4);//��ʾʮλ
					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
					if(Temperatuer_Up<0)
					{
					  write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
						Temperature_I=0-Temperatuer_Up;
					}
					else
					{
						Temperature_I=Temperatuer_Up;
						write_addr_dat_n_char(7,0x08, 0);//����ʾ����
					}
					Display_Shidu_3(26,0);//��ʾ��λ
					write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,Temperature_I%10);//��ʾ��λ
					Display_Shidu_3(30, Temperature_I/10);//��ʾʮλ	
					switch(UP_Dowm)
					{
						case 0:
									{
									if(Temperatuer_Down>Temperatuer_Up)//���޴������޵����
									{
										//��ʾ�ڶ���Err
										Clr_Shidu();
										if(Temperatuer_Up<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											Temperature_I=0-Temperatuer_Up;
										}
										else
										{
											Temperature_I=Temperatuer_Up;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										Display_Shidu_2(26,8);//��ʾ��λ
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_2(28,8);//��ʾ��λ
										Display_Shidu_2(30,6);//��ʾʮλ		
									}
									else
									{
										Temperatuer_Up--; 
										if(Temperatuer_Up<-40) 
										Temperatuer_Up=-40;
										//��ʾ�ڶ���
										Clr_Shidu();
										if(Temperatuer_Up<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											Temperature_I=0-Temperatuer_Up;
										}
										else
										{
											Temperature_I=Temperatuer_Up;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										Display_Shidu_3(26,0);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,Temperature_I%10);//��ʾ��λ
										Display_Shidu_3(30, Temperature_I/10);//��ʾʮλ	
									}
									break;
									}
						case 1:
									{
									Temperatuer_Up++;
									if(Temperatuer_Up>80)
									Temperatuer_Up=80;
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_3(26,0);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,Temperature_I%10);//��ʾ��λ
									Display_Shidu_3(30, Temperature_I/10);//��ʾʮλ	
									break;
									}
					}
					break;
			}
			case 5://�¶�����
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 4);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 3);//��ʾ��λ
					Display_Wendu_1(4, 5);//��ʾʮλ
					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
					if(Temperatuer_Down<0)
					{
					  write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
						Temperature_O=0-Temperatuer_Down;
					}
					else
					{
						Temperature_O=Temperatuer_Down;
						write_addr_dat_n_char(7,0x08, 0);//����ʾ����
					}
					Display_Shidu_3(26,0);//��ʾ��λ
					write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,Temperature_O%10);//��ʾ��λ
					Display_Shidu_3(30,Temperature_O/10);//��ʾʮλ
					switch(UP_Dowm)
					{
						case 0:
									{
									Temperatuer_Down--; 
									if(Temperatuer_Down<-40)
									Temperatuer_Down=-40;
									//��ʾ�ڶ���
									Clr_Shidu();
									if(Temperatuer_Down<0)
									{
										write_addr_dat_n_char(7,0x08, 1);//��ʾ����
										Temperature_O=0-Temperatuer_Down;
									}
									else
									{
										Temperature_O=Temperatuer_Down;
										write_addr_dat_n_char(7,0x08, 0);//����ʾ����
									}
									Display_Shidu_3(26,0);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,Temperature_O%10);//��ʾ��λ
									Display_Shidu_3(30,Temperature_O/10);//��ʾʮλ	
									break;}
						case 1:
									{
									if(Temperatuer_Down>Temperatuer_Up)//���޴������޵����
									{
									  //��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(26,8);//��ʾ��λ
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_2(28,8);//��ʾ��λ
										Display_Shidu_2(30,6);//��ʾʮλ					
									}
									else
									{
										Temperatuer_Down++; 
										if(Temperatuer_Down>80) 
										Temperatuer_Down=80;
										//��ʾ�ڶ���
										Clr_Shidu();
										if(Temperatuer_Down<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����
											Temperature_O=0-Temperatuer_Down;
										}
										else
										{
											Temperature_O=Temperatuer_Down;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										Display_Shidu_3(26,0);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,Temperature_O%10);//��ʾ��λ
										Display_Shidu_3(30,Temperature_O/10);//��ʾʮλ	
									}										
									break;
									}
					}
					break;
			}
			case 6://ʪ������
			{
					write_addr_dat_n_char(7,0x08, 0);//����ʾ����
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 11);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 11);//��ʾ��λ
					Display_Wendu_1(4, 6);//��ʾʮλ
					//Display_CHARS(huashidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
					Display_Shidu_1(26,0);//��ʾ��λ
				  write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,Humtidy_Up%10);//��ʾ��λ
					Display_Shidu_3(30,Humtidy_Up/10);//��ʾʮλ
					switch(UP_Dowm)
					{
						case 0:
									{
									if(Humtidy_Up<Humtidy_Down)//����С�����޵Ĵ������
									{
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(26,8);//��ʾ��λ
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_2(28,8);//��ʾ��λ
										Display_Shidu_2(30,6);//��ʾʮλ									
									}
									else
									{
										Humtidy_Up--; 
										if(Humtidy_Up<0) 
										Humtidy_Up=1;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_1(26,0);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,Humtidy_Up%10);//��ʾ��λ
										Display_Shidu_3(30,Humtidy_Up/10);//��ʾʮλ
									}
									break;
									}
						case 1:
									{
									Humtidy_Up++; 
									if(Humtidy_Up>99) 
									Humtidy_Up=99;
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_1(26,0);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,Humtidy_Up%10);//��ʾ��λ
									Display_Shidu_3(30,Humtidy_Up/10);//��ʾʮλ
									break;
									}
					}
					break;
			}
			case 7://ʪ������
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 4);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 11);//��ʾ��λ
					Display_Wendu_1(4, 7);//��ʾʮλ
					//Display_CHARS(huashidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
					Display_Shidu_1(26,0);//��ʾ��λ
				  write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,Humtidy_Down%10);//��ʾ��λ
					Display_Shidu_3(30,Humtidy_Down/10);//��ʾʮλ
					switch(UP_Dowm)
					{
						case 0:
									{
									Humtidy_Down--; 
									if(Humtidy_Down<0)
									Humtidy_Down=0;
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_1(26,0);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,Humtidy_Down%10);//��ʾ��λ
									Display_Shidu_3(30,Humtidy_Down/10);//��ʾʮλ
									break;
									}
						case 1:
									{
									if(Humtidy_Up<Humtidy_Down)//����С�����޵Ĵ������
									{
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(26,8);//��ʾ��λ
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_2(28,8);//��ʾ��λ
										Display_Shidu_2(30,6);//��ʾʮλ									
									}
									else
									{
										Humtidy_Down++; 
										if(Humtidy_Down>99) 
										Humtidy_Down=99;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_1(26,0);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,Humtidy_Down%10);//��ʾ��λ
										Display_Shidu_3(30,Humtidy_Down/10);//��ʾʮλ									
									}	
									break;
									}
					}
					break;
			}
			case 8://���϶Ȼ��϶��л�
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 5);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 3);//��ʾ��λ
					Display_Wendu_1(4, 8);//��ʾʮλ
					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
					if(C_F==1)	
					{
						//��ʾ�ڶ���
						Display_Shidu_2(28,5);//��ʾF
					}
					else 
					{
						//��ʾ�ڶ���
						Display_Shidu_2(28,3);//��ʾC
					}	
					switch(UP_Dowm)
					{
						case 0:
									{
									if(C_F==0)	
										{
										C_F=1;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(28,5);//��ʾC
										}
									else 
										{
										C_F=0;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(28,3);//��ʾF
										}	
									break;
									}
						case 1:
									{
									if(C_F==0)	
										{
										C_F=1;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(28,5);//��ʾC
										}
									else 
										{
										C_F=0;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(28,3);//��ʾF
										}	
									break;
									}
					}
					break;
			}
			case 9://Զ�̿���ѡ��
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 8);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 5);//��ʾ��λ
					Display_Wendu_1(4, 9);//��ʾʮλ
					//��ʾ�ڶ���ON
					Clr_Shidu();
					if(RemoteControl==1) 
					{											
					//��ʾ�ڶ���ON
					Display_Shidu_2(26,10);
					//write_addr_dat_n_1bit(28,0x01, 1);
					Display_Shidu_3(28,  0);//
					}
					else 
					{
					//��ʾ�ڶ���OFF
					Display_Shidu_2(26,5);
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_2(28,  5);
					Display_Shidu_3(30,0);
					}
					switch(UP_Dowm)
					{
						case 0:
									{
											if(RemoteControl==0) 
											{											
											RemoteControl=1;
											//��ʾ�ڶ���ON
											Clr_Shidu();
											Display_Shidu_2(26,10);
											//write_addr_dat_n_1bit(28,0x01, 1);
											Display_Shidu_3(28,  0);//
											}

											else 
											{
											RemoteControl=0;
											//��ʾ�ڶ���OFF
											Clr_Shidu();
											Display_Shidu_2(26,5);
											//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
											Display_Shidu_2(28,  5);
											Display_Shidu_3(30,0);
											}

											break;
									}
						case 1:
									{
											if(RemoteControl==0) 
											{											
											RemoteControl=1;
											//��ʾ�ڶ���ON
											Clr_Shidu();
											Display_Shidu_2(26,10);
											//write_addr_dat_n_1bit(28,0x01, 1);
											Display_Shidu_3(28,  0);//
											//Display_Shidu_1(30, BaudRate/10000);//��ʾ��λ	
											}

											else 
											{
											RemoteControl=0;
											//��ʾ�ڶ���OFF
											Clr_Shidu();
											Display_Shidu_2(26,5);
											//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
											Display_Shidu_2(28,  5);
											Display_Shidu_3(30,0);
											}

											break;
									}
					}
					break;
			}
			case 10://��¼�Ƿ���
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 1);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 0);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
					//��ʾ�ڶ���ON
					Clr_Shidu();
					if(RecordMode==0)
					{
					//��ʾ�ڶ���ON
							Display_Shidu_2(26,10);
							//write_addr_dat_n_1bit(28,0x01, 1);
							Display_Shidu_3(28,  0);//
					}
					if(RecordMode==1)
					{
						//��ʾ�ڶ���OFF
							Display_Shidu_2(26,5);
							Display_Shidu_2(28,  5);
							Display_Shidu_3(30,0);
					}
					switch(UP_Dowm)
					{
						case 0:
									{
									RecordMode--;
									if(RecordMode<0) 
									RecordMode=1;
									if(RecordMode==0)
									{
									//��ʾ�ڶ���ON
											Clr_Shidu();
											Display_Shidu_2(26,10);
											//write_addr_dat_n_1bit(28,0x01, 1);
											Display_Shidu_3(28,  0);//
									}
									if(RecordMode==1)
									{
										//��ʾ�ڶ���OFF
											Clr_Shidu();
											Display_Shidu_2(26,5);
											Display_Shidu_2(28,  5);
											Display_Shidu_3(30,0);
									}
									break;
									}
						case 1:
									{
									RecordMode++;
									if(RecordMode>1) 
									RecordMode=0;
									if(RecordMode==0)
									{
											//��ʾ�ڶ���ON
											Clr_Shidu();
											Display_Shidu_2(26,10);
											//write_addr_dat_n_1bit(28,0x01, 1);
											Display_Shidu_3(28,  0);//
									}
									if(RecordMode==1)
									{
										//��ʾ�ڶ���OFF
											Clr_Shidu();
											Display_Shidu_2(26,5);
											Display_Shidu_2(28,  5);
											Display_Shidu_3(30,0);
									}
									break;}
					}
					break;
			}
			case 11://��¼ģʽ
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 2);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 0);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
					//��ʾ�ڶ���
					Clr_Shidu();
						if(StorageMode==1)
						{
							//��ʾ�ڶ���
							Display_Shidu_2(26,3);//��ʾ��λ
							Display_Shidu_2(28,9);//��ʾ��λ
							Display_Shidu_2(30,3);//��ʾʮλ
						}
						else 
					  {
							//��ʾ�ڶ���
							Display_Shidu_2(26,0);//��ʾ��λ
							Display_Shidu_2(28,8);//��ʾ��λ
							Display_Shidu_3(30,0);//��ʾʮλ
						}
					switch(UP_Dowm)
					{
						case 0:
							{
						if(StorageMode==0)
						{
							StorageMode=1;
							//��ʾ�ڶ���
							Clr_Shidu();
							Display_Shidu_2(26,3);//��ʾ��λ
							Display_Shidu_2(28,9);//��ʾ��λ
							Display_Shidu_2(30,3);//��ʾʮλ
						}
						else 
					  {
							StorageMode=0;//Ĭ��
							//��ʾ�ڶ���
							Clr_Shidu();
							Display_Shidu_2(26,0);//��ʾ��λ
							Display_Shidu_2(28,8);//��ʾ��λ
							Display_Shidu_3(30,0);//��ʾʮλ
						}
						break;
						}
						case 1:
						{
						if(StorageMode==0)
						{
							StorageMode=1;
							//��ʾ�ڶ���
							Clr_Shidu();
							Display_Shidu_2(26,3);//��ʾ��λ
							//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
							Display_Shidu_2(28,9);//��ʾ��λ
							Display_Shidu_2(30,3);//��ʾʮλ
						}
						else 
					  {
							StorageMode=0;
							//��ʾ�ڶ���
							Clr_Shidu();
							Display_Shidu_2(26,0);//��ʾ��λ
							//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
							Display_Shidu_2(28,8);//��ʾ��λ
							Display_Shidu_3(30,0);//��ʾʮλ
						}
						break;
						}
					}
					break;
			}
			case 12://�����洢ʱ��
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 3);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 0);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
					//��ʾ�ڶ���
					Clr_Shidu();
					Display_Shidu_3(26,RecordTiming%10);//��ʾ��λ
					Display_Shidu_3(28,RecordTiming/10%10);//��ʾʮλ
					Display_Shidu_3(30,RecordTiming/100);//��ʾ��λ
					switch(UP_Dowm)
					{
						case 0:
									{
									RecordTiming--;
									if(RecordTiming<1) 
									RecordTiming=600;
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_3(26,RecordTiming%10);//��ʾ��λ
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_3(28,RecordTiming/10%10);//��ʾʮλ
									Display_Shidu_3(30,RecordTiming/100);//��ʾ��λ
									break;
									}
						case 1:
									{RecordTiming++;
									if(RecordTiming>600) 
									RecordTiming=1;
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_3(26,RecordTiming%10);//��ʾ��λ
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_3(28,RecordTiming/10%10);//��ʾʮλ
									Display_Shidu_3(30,RecordTiming/100);//��ʾ��λ
									break;
									}
					}
					break;
			}
			case 13://�����洢ʱ��
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 4);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 0);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
					//��ʾ�ڶ���
					Clr_Shidu();
					Display_Shidu_3(26,WarningTiming%10);//��ʾ��λ
					Display_Shidu_3(28,WarningTiming/10%10);//��ʾʮλ
					Display_Shidu_3(30,WarningTiming/100);//��ʾ��λ
					switch(UP_Dowm)
					{
						case 0:{
						WarningTiming--;
						if(WarningTiming<1) 
						WarningTiming=600;
						//��ʾ�ڶ���
						Clr_Shidu();
						Display_Shidu_3(26,WarningTiming%10);//��ʾ��λ
						Display_Shidu_3(28,WarningTiming/10%10);//��ʾʮλ
						Display_Shidu_3(30,WarningTiming/100);//��ʾ��λ
						break;}
						case 1:{
						WarningTiming++;
						if(WarningTiming>600) 
						WarningTiming=1;
						//��ʾ�ڶ���
						Clr_Shidu();
						Display_Shidu_3(26,WarningTiming%10);//��ʾ��λ
						Display_Shidu_3(28,WarningTiming/10%10);//��ʾʮλ
						Display_Shidu_3(30,WarningTiming/100);//��ʾ��λ
						break;}
					}
					break;
			}
			case 14://��ʷ�������C05
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 5);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 0);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
					//��ʾ�ڶ���
					Clr_Shidu();
					Display_Shidu_3(26,DeleteData%10);//��ʾ��λ
					//Display_Shidu_3(28,DeleteData/10%10);//��ʾʮλ
					switch(UP_Dowm)
					{
						case 0:{
						DeleteData--;
						if(DeleteData<0)
							DeleteData=1;
						//��ʾ�ڶ���
						Clr_Shidu();
						Display_Shidu_3(26,DeleteData%10);//��ʾ��λ
						//Display_Shidu_3(28,DeleteData/10%10);//��ʾʮλ
						break;}
						case 1:{
						DeleteData++;
						if(DeleteData>1)
							DeleteData=0;
						//��ʾ�ڶ���
						Clr_Shidu();
						Display_Shidu_3(26,DeleteData%10);//��ʾ��λ
						//Display_Shidu_3(28,DeleteData/10%10);//��ʾʮλ
						break;}
					}
					break;
			}
			case 15://�¶ȳ���������C06
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 6);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 0);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
					Display_Shidu_3(26,TemperatureHys/1%10);//��ʾ��λ
					write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,TemperatureHys/10%10);//��ʾ��λ
					Display_Shidu_3(30,TemperatureHys/100);//��ʾʮλ
					switch(UP_Dowm)
					{
						case 0:
									{
									TemperatureHys--; 
									if(TemperatureHys<0)
									TemperatureHys=0;
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_3(26,TemperatureHys/1%10);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,TemperatureHys/10%10);//��ʾ��λ
									Display_Shidu_3(30,TemperatureHys/100);//��ʾʮλ	
									break;}
						case 1:
									{
										TemperatureHys++; 
										if(TemperatureHys>120) 
										TemperatureHys=120;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_3(26,TemperatureHys/1%10);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,TemperatureHys/10%10);//��ʾ��λ
										Display_Shidu_3(30,TemperatureHys/100);//��ʾʮλ	
									break;
									}
					}
					break;
			}

			case 16://ʪ�ȳ���������C07
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 7);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 0);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
					 write_addr_dat_n_char(28,0x01, HumtidyHys%10);//P2
					//��ʾ�ڶ���
					Clr_Shidu();
					Display_Shidu_1(26,0);//��ʾ��λ
				//	write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,HumtidyHys/10%10);//��ʾ��λ
					Display_Shidu_3(30,HumtidyHys/100);//��ʾʮλ
					switch(UP_Dowm)
					{
						case 0:
									{
									HumtidyHys--; 
									if(HumtidyHys<0)
									HumtidyHys=0;
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_1(26,HumtidyHys%10);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,HumtidyHys/10%10);//��ʾ��λ
									Display_Shidu_3(30,HumtidyHys/100);//��ʾʮλ	
									break;}
						case 1:
									{

										HumtidyHys++; 
										if(HumtidyHys>100) 
										HumtidyHys=100;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_1(26,HumtidyHys%10);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,HumtidyHys/10%10);//��ʾ��λ
										Display_Shidu_3(30,HumtidyHys/100);//��ʾʮλ									
									break;
									}
					}
					break;
			}

			case 17://C08�¶�4ma
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 8);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 0);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
					if(Temperature_4ma<0)
					{
					  write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
						TemperatureDisplay_4ma=0-Temperature_4ma;
					}
					else
					{
						TemperatureDisplay_4ma=Temperature_4ma;
						write_addr_dat_n_char(7,0x08, 0);//����ʾ����
					}
					Display_Shidu_3(26,0);//��ʾ��λ
					write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,TemperatureDisplay_4ma%10);//��ʾ��λ
					Display_Shidu_3(30, TemperatureDisplay_4ma/10);//��ʾʮλ	
					switch(UP_Dowm)
					{
						case 0:
									{
									if(Temperature_4ma>Temperature_20ma)//���޴������޵����
									{
										//��ʾ�ڶ���Err
										Clr_Shidu();
										if(Temperature_4ma<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											TemperatureDisplay_4ma=0-Temperature_4ma;
										}
										else
										{
											TemperatureDisplay_4ma=Temperature_4ma;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										Display_Shidu_2(26,8);//��ʾ��λ
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_2(28,8);//��ʾ��λ
										Display_Shidu_2(30,6);//��ʾʮλ		
									}
									else
									{
										Temperature_4ma--; 
										if(Temperature_4ma<-40) 
										Temperature_4ma=-40;
										//��ʾ�ڶ���
										Clr_Shidu();
										if(Temperature_4ma<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											TemperatureDisplay_4ma=0-Temperature_4ma;
										}
										else
										{
											TemperatureDisplay_4ma=Temperature_4ma;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										Display_Shidu_3(26,0);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,TemperatureDisplay_4ma%10);//��ʾ��λ
										Display_Shidu_3(30, TemperatureDisplay_4ma/10);//��ʾʮλ	
									}
									break;
									}
						case 1:
									{
									Temperature_4ma++;
									if(Temperature_4ma>80)
									Temperature_4ma=80;
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_3(26,0);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,TemperatureDisplay_4ma%10);//��ʾ��λ
									Display_Shidu_3(30, TemperatureDisplay_4ma/10);//��ʾʮλ	
									break;
									}
					}
					break;
			}
			case 18://C09�¶�20ma
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 9);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 0);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
					if(Temperature_20ma<0)
					{
					  write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
						TemperatureDisplay_20ma=0-Temperature_20ma;
					}
					else
					{
						TemperatureDisplay_20ma=Temperature_20ma;
						write_addr_dat_n_char(7,0x08, 0);//����ʾ����
					}
					Display_Shidu_3(26,0);//��ʾ��λ
					write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,TemperatureDisplay_20ma%10);//��ʾ��λ
					Display_Shidu_3(30, TemperatureDisplay_20ma/10);//��ʾʮλ	
					switch(UP_Dowm)
					{
						case 0:
									{
									if(Temperature_4ma>Temperature_20ma)//���޴������޵����
									{
										//��ʾ�ڶ���Err
										Clr_Shidu();
										if(Temperature_20ma<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											TemperatureDisplay_20ma=0-Temperature_20ma;
										}
										else
										{
											TemperatureDisplay_20ma=Temperature_20ma;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										Display_Shidu_2(26,8);//��ʾ��λ
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_2(28,8);//��ʾ��λ
										Display_Shidu_2(30,6);//��ʾʮλ		
									}
									else
									{
										Temperature_20ma--; 
										if(Temperature_20ma<-40) 
										Temperature_20ma=-40;
										//��ʾ�ڶ���
										Clr_Shidu();
										if(Temperature_20ma<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											TemperatureDisplay_20ma=0-Temperature_20ma;
										}
										else
										{
											TemperatureDisplay_20ma=Temperature_20ma;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										Display_Shidu_3(26,0);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,TemperatureDisplay_20ma%10);//��ʾ��λ
										Display_Shidu_3(30, TemperatureDisplay_20ma/10);//��ʾʮλ	
									}
									break;
									}
						case 1:
									{
									Temperature_20ma++;
									if(Temperature_20ma>80)
									Temperature_20ma=80;
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_3(26,0);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,TemperatureDisplay_20ma%10);//��ʾ��λ
									Display_Shidu_3(30, TemperatureDisplay_20ma/10);//��ʾʮλ	
									break;
									}
					}
					break;
			}
			case 19://C10ʪ��4ma
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 0);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 1);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
					//write_addr_dat_n_char(28,0x01, 1);//P2
					//Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
					if(Humtidy_4ma<0)
					{
					  write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
						HumtidyDisplay_4ma=0-Humtidy_4ma;
					}
					else
					{
						HumtidyDisplay_4ma=Humtidy_4ma;
						write_addr_dat_n_char(7,0x08, 0);//����ʾ����
					}
					Display_Shidu_1(26,0);//��ʾ��λ
					write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,HumtidyDisplay_4ma%10);//��ʾ��λ
					Display_Shidu_3(30, HumtidyDisplay_4ma/10);//��ʾʮλ	
					switch(UP_Dowm)
					{
						case 0:
									{
									if(Humtidy_4ma>Humtidy_20ma)//���޴������޵����
									{
										//��ʾ�ڶ���Err
										Clr_Shidu();
										if(Humtidy_4ma<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											HumtidyDisplay_4ma=0-Humtidy_4ma;
										}
										else
										{
											HumtidyDisplay_4ma=Humtidy_4ma;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										Display_Shidu_2(26,8);//��ʾ��λ
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_2(28,8);//��ʾ��λ
										Display_Shidu_2(30,6);//��ʾʮλ		
									}
									else
									{
										Humtidy_4ma--; 
										if(Humtidy_4ma<0) 
										Humtidy_4ma=0;
										//��ʾ�ڶ���
										Clr_Shidu();
										if(Humtidy_4ma<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											HumtidyDisplay_4ma=0-Humtidy_4ma;
										}
										else
										{
											HumtidyDisplay_4ma=Humtidy_4ma;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										Display_Shidu_1(26,0);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,HumtidyDisplay_4ma%10);//��ʾ��λ
										Display_Shidu_3(30, HumtidyDisplay_4ma/10);//��ʾʮλ	
									}
									break;
									}
						case 1:
									{
									Humtidy_4ma++;
									if(Humtidy_4ma>99)
									Humtidy_4ma=9;
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_1(26,0);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,HumtidyDisplay_4ma%10);//��ʾ��λ
									Display_Shidu_3(30, HumtidyDisplay_4ma/10);//��ʾʮλ	
									break;
									}
					}
					break;
			}
			case 20://C11ʪ��20ma
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 1);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 1);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
					//write_addr_dat_n_char(28,0x01, 1);//P2
					//Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
					if(Humtidy_20ma<0)
					{
					  write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
						HumtidyDisplay_20ma=0-Humtidy_20ma;
					}
					else
					{
						HumtidyDisplay_20ma=Humtidy_20ma;
						write_addr_dat_n_char(7,0x08, 0);//����ʾ����
					}
					Display_Shidu_1(26,0);//��ʾ��λ
					write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,HumtidyDisplay_20ma%10);//��ʾ��λ
					Display_Shidu_3(30, HumtidyDisplay_20ma/10);//��ʾʮλ	
					switch(UP_Dowm)
					{
						case 0:
									{
									if(Humtidy_4ma>Humtidy_20ma)//���޴������޵����
									{
										//��ʾ�ڶ���Err
										Clr_Shidu();
										if(Humtidy_20ma<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											HumtidyDisplay_20ma=0-Humtidy_20ma;
										}
										else
										{
											HumtidyDisplay_20ma=Humtidy_20ma;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										Display_Shidu_2(26,8);//��ʾ��λ
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_2(28,8);//��ʾ��λ
										Display_Shidu_2(30,6);//��ʾʮλ		
									}
									else
									{
										Humtidy_20ma--; 
										if(Humtidy_20ma<0) 
										Humtidy_20ma=0;
										//��ʾ�ڶ���
										Clr_Shidu();
										if(Humtidy_20ma<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											HumtidyDisplay_20ma=0-Humtidy_20ma;
										}
										else
										{
											HumtidyDisplay_20ma=Humtidy_20ma;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										Display_Shidu_1(26,0);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,HumtidyDisplay_20ma%10);//��ʾ��λ
										Display_Shidu_3(30, HumtidyDisplay_20ma/10);//��ʾʮλ	
									}
									break;
									}
						case 1:
									{
									Humtidy_20ma++;
									if(Humtidy_20ma>99)
									Humtidy_20ma=99;
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_1(26,0);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,HumtidyDisplay_20ma%10);//��ʾ��λ
									Display_Shidu_3(30, HumtidyDisplay_20ma/10);//��ʾʮλ	
									break;
									}
					}
					break;
			}
			case 21://C12�¶�У׼
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 2);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 1);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
					if(TemperatureCalibration<0)
					{
					  write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
						TemperatureCalibrationDisplay=0-TemperatureCalibration;
					}
					else
					{
						TemperatureCalibrationDisplay=TemperatureCalibration;
						write_addr_dat_n_char(7,0x08, 0);//����ʾ����
					}
					Display_Shidu_3(26,TemperatureCalibrationDisplay/1%10);//��ʾ��λ
					write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,TemperatureCalibrationDisplay/10%10);//��ʾ��λ
					Display_Shidu_3(30, TemperatureCalibrationDisplay/100);//��ʾʮλ	
					switch(UP_Dowm)
					{
						case 0:
									{
										TemperatureCalibration--; 
										if(TemperatureCalibration<-100) 
										TemperatureCalibration=-100;
										if(TemperatureCalibration<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											TemperatureCalibrationDisplay=0-TemperatureCalibration;
										}
										else
										{
											TemperatureCalibrationDisplay=TemperatureCalibration;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_3(26,TemperatureCalibrationDisplay%10);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,TemperatureCalibrationDisplay/10%10);//��ʾ��λ
										Display_Shidu_3(30, TemperatureCalibrationDisplay/100);//��ʾʮλ	
									break;
									}
						case 1:
									{
									TemperatureCalibration++;
									if(TemperatureCalibration>100)
									TemperatureCalibration=100;
									if(TemperatureCalibration<0)
									{
										write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
										TemperatureCalibrationDisplay=0-TemperatureCalibration;
									}
									else
									{
										TemperatureCalibrationDisplay=TemperatureCalibration;
										write_addr_dat_n_char(7,0x08, 0);//����ʾ����
									}
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_3(26,TemperatureCalibrationDisplay%10);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,TemperatureCalibrationDisplay/10%10);//��ʾ��λ
									Display_Shidu_3(30, TemperatureCalibrationDisplay/100);//��ʾʮλ	
									break;
									}
					}
					break;
			}
			case 22://C13ʪ��У׼
				{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 3);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 1);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
					//write_addr_dat_n_char(28,0x01, 1);//P2
					//Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
					if(HumtidyCalibration<0)
					{
					  write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
						HumtidyCalibrationDisplay=0-HumtidyCalibration;
					}
					else
					{
						HumtidyCalibrationDisplay=HumtidyCalibration;
						write_addr_dat_n_char(7,0x08, 0);//����ʾ����
					}
					Display_Shidu_1(26,HumtidyCalibrationDisplay/1%10);//��ʾ��λ
					write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,HumtidyCalibrationDisplay/10%10);//��ʾ��λ
					Display_Shidu_3(30, HumtidyCalibrationDisplay/100);//��ʾʮλ	
					switch(UP_Dowm)
					{
						case 0:
									{
										HumtidyCalibration--; 
										if(HumtidyCalibration<-100) 
										HumtidyCalibration=-100;
										if(HumtidyCalibration<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											HumtidyCalibrationDisplay=0-HumtidyCalibration;
										}
										else
										{
											HumtidyCalibrationDisplay=HumtidyCalibration;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_1(26,HumtidyCalibrationDisplay%10);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,HumtidyCalibrationDisplay/10%10);//��ʾ��λ
										Display_Shidu_3(30, HumtidyCalibrationDisplay/100);//��ʾʮλ	
									break;
									}
						case 1:
									{
									HumtidyCalibration++;
									if(HumtidyCalibration>100)
									HumtidyCalibration=100;
									if(HumtidyCalibration<0)
									{
										write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
										HumtidyCalibrationDisplay=0-HumtidyCalibration;
									}
									else
									{
										HumtidyCalibrationDisplay=HumtidyCalibration;
										write_addr_dat_n_char(7,0x08, 0);//����ʾ����
									}
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_1(26,HumtidyCalibrationDisplay%10);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,HumtidyCalibrationDisplay/10%10);//��ʾ��λ
									Display_Shidu_3(30, HumtidyCalibrationDisplay/100);//��ʾʮλ	
									break;
									}
					}
					break;
			}
			case 23://ʣ�����
			{
				//��ʾ��ǰʣ�����
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 8);//��ʾС��
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 7);//��ʾ��λ
					Display_Wendu_2(4, 1);//��ʾʮλ
					//Display_CHARS(huashidu,1);//��ʾ��ʪ�ȵķ���
					write_addr_dat_n_char(7,0x08, 0);//����ʾ����
					//��ʾ�ڶ���OFF
					Clr_Shidu();
					if((V_BAT*2)<Battery[0]){	
						Display_Shidu_3(28,1);
						Display_Shidu_3(26,0);}
					else if((V_BAT*2)<Battery[1]&&(V_BAT*2)>=Battery[0]){
						Display_Shidu_3(28,2);
						Display_Shidu_3(26,0);}
					else if((V_BAT*2)<Battery[2]&&(V_BAT*2)>=Battery[1]){
						Display_Shidu_3(28,3);
						Display_Shidu_3(26,0);}
					else if((V_BAT*2)<Battery[3]&&(V_BAT*2)>=Battery[2]){
						Display_Shidu_3(28,4);
						Display_Shidu_3(26,0);}
					else if((V_BAT*2)<Battery[4]&&(V_BAT*2)>=Battery[3]){
						Display_Shidu_3(28,5);
						Display_Shidu_3(26,0);}
					else if((V_BAT*2)<Battery[5]&&(V_BAT*2)>=Battery[4]){
						Display_Shidu_3(28,6);
						Display_Shidu_3(26,0);}
					else if((V_BAT*2)<Battery[6]&&(V_BAT*2)>=Battery[5]){
						Display_Shidu_3(28,7);
						Display_Shidu_3(26,0);}
					else if((V_BAT*2)<Battery[7]&&(V_BAT*2)>=Battery[6]){
						Display_Shidu_3(28,8);
						Display_Shidu_3(26,0);}
					else if((V_BAT*2)<Battery[8]&&(V_BAT*2)>=Battery[7]){
						Display_Shidu_3(28,9);
						Display_Shidu_3(26,0);}
					else if((V_BAT*2)<Battery[9]&&(V_BAT*2)>=Battery[8]||(V_BAT*2>=Battery[9])){
						Display_Shidu_3(30,1);
						Display_Shidu_3(28,0);
						Display_Shidu_3(26,0);}	

					break;
			}
			case 24://������ѡ��
			{
					//��ʾ��һ��SEL
					Clr_Wendu();
					Display_Wendu_2(0, 4);//��ʾС��
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 6);//��ʾ��λ
					Display_Wendu_1(4, 5);//��ʾʮλ
					//Display_CHARS(huashidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
					Display_Shidu_3(26,SensorSelected);//��ʾ��
					switch(UP_Dowm)
					{
						case 0:{
						SensorSelected--;
						if(SensorSelected<1)
						SensorSelected=4;
						//��ʾ�ڶ���
						Clr_Shidu();
						Display_Shidu_3(26,SensorSelected);//��ʾ��
						break;}
						case 1:{
						SensorSelected++;
						if(SensorSelected>4)
						SensorSelected=1;
						//��ʾ�ڶ���
						Clr_Shidu();
						Display_Shidu_3(26,SensorSelected);//��ʾ��
						break;}
					}
					break;
			}
			case 25://��������
			{
					//ʱ������
					//��ʾ��һ��16
					Clr_Wendu();
					//Display_Wendu_2(0, 4);//��ʾС��
					Display_Wendu_1(2, 6);//��ʾ��λ
					Display_Wendu_1(4, 1);//��ʾʮλ
					Clr_Shidu();
					RTC_Control=1;
					OnlyDispalyTime=0;
					RTCTimeParametersInit=1;
					break;
			}
		}
			
}







void ModeSwitch2(int mode,int UP_Dowm)//UP_Dowm��0����С��1������
{
		//DisplayTime();
    switch(mode)
		{
			case 0:
			{
					break;
			}
			case 1://��ַ
			{
				//��ʾ��һ��
				Clr_Wendu();
				Display_Wendu_2(0, 0);//��ʾС��
				write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
				//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
				Display_Wendu_1(2, 1);//��ʾ��λ
				Display_Wendu_1(4, 1);//��ʾʮλ
				//��ʾ�ڶ���
				Clr_Shidu();
				Display_Shidu_3(26,address%100%10);//��ʾ��λ
				//write_addr_dat_n_1bit(28,0x00, 1);//����ʾС����
				Display_Shidu_3(28,  address%100/10);//��ʾʮλ
				Display_Shidu_3(30,  address/100);//��ʾ��λ	
				switch(UP_Dowm)
					{
						case 0:
									{
											
										   address--;
											if(address<1) 
											address=254;
											//��ʾ�ڶ���
											Clr_Shidu();
											Display_Shidu_3(26, address%100%10);//��ʾ��λ
											//write_addr_dat_n_1bit(28,0x00, 1);//����ʾС����
											Display_Shidu_3(28,  address%100/10);//��ʾʮλ
											Display_Shidu_3(30,  address/100);//��ʾ��λ	
											break;
									}
						case 1:
									{
											address++;
											if(address>254) 
											address=1;
											//��ʾ�ڶ���
											Clr_Shidu();
											Display_Shidu_3(26, address%100%10);//��ʾ��λ
											//write_addr_dat_n_1bit(28,0x00, 1);//����ʾС����
											Display_Shidu_3(28,  address%100/10);//��ʾʮλ
											Display_Shidu_3(30,  address/100);//��ʾ��λ	
											break;
									}
					
					}
					break;
			}
			case 2://������
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 2);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 1);//��ʾ��λ
					Display_Wendu_1(4, 2);//��ʾʮλ
					//��ʾ�ڶ���
					Clr_Shidu();
					BaudRate=BaudRateArray[BaudRateNum];
					Display_Shidu_3(26, BaudRate/100%10);//��ʾ��λ
					//write_addr_dat_n_1bit(28,0x00, 1);//��ʾС����
					Display_Shidu_3(28,  BaudRate/100/10%10);//��ʾʮλ
					Display_Shidu_3(30, BaudRate/10000);//��ʾ��λ	
					switch(UP_Dowm)
					{
							case 0:
						{
								BaudRateNum--;
								if(BaudRateNum<0)
								{BaudRateNum=4;}
								BaudRate=BaudRateArray[BaudRateNum];
								uart2_init(BaudRate);
								//��ʾ�ڶ���
								Clr_Shidu();
								Display_Shidu_3(26, BaudRate/100%10);//��ʾ��λ
								//write_addr_dat_n_1bit(28,0x00, 1);//����ʾС����
								Display_Shidu_3(28,  BaudRate/100/10%10);//��ʾʮλ
								Display_Shidu_3(30, BaudRate/10000);//��ʾ��λ	
								break;
						}
						case 1:
						{
								BaudRateNum++;
								if(BaudRateNum>4)
								{BaudRateNum=0;}
								BaudRate=BaudRateArray[BaudRateNum];
								uart2_init(BaudRate);
								//��ʾ�ڶ���
								Clr_Shidu();
								Display_Shidu_3(26, BaudRate/100%10);//��ʾ��λ
								//write_addr_dat_n_1bit(28,0x00, 1);//��ʾС����write_addr_dat_n_char
								Display_Shidu_3(28,  BaudRate/100/10%10);//��ʾʮλ
								Display_Shidu_3(30, BaudRate/10000);//��ʾ��λ	
								break;
						}	
					}
					break;
			}
			case 3://��������/�ر�
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 8);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 7);//��ʾ��λ
					Display_Wendu_1(4, 3);//��ʾʮλ
					//��ʾ�ڶ���
				if(Warning==1) 
				{
					//��ʾ�ڶ���
					Clr_Shidu();
					Display_Shidu_2(26,10);//��ʾ��λ
					Display_Shidu_3(28,0);//��ʾʮλ							
				}
				else 
				{
					//��ʾ�ڶ���
					Clr_Shidu();
					Display_Shidu_2(26,5);//��ʾ��λ
					Display_Shidu_2(28,5);//��ʾʮλ
					Display_Shidu_3(30, 0);//��ʾ��λ	
				}
					switch(UP_Dowm)
					{
						case 0:
									{
									if(Warning==0) 
									{
										Warning=1;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(26,10);//��ʾ��λ
										Display_Shidu_3(28,0);//��ʾʮλ							
									}
									else 
									{
										Warning=0;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(26,5);//��ʾ��λ
										Display_Shidu_2(28,5);//��ʾʮλ
										Display_Shidu_3(30, 0);//��ʾ��λ	
									}
									
									break;
									}//��ת
						case 1:
									{
									if(Warning==0) 
									{
										Warning=1;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(26,10);//��ʾ��λ
										Display_Shidu_3(28,0);//��ʾʮλ
									}
									else 
									{
										Warning=0;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(26,5);//��ʾ��λ
										Display_Shidu_2(28,5);//��ʾʮλ
										Display_Shidu_3(30, 0);//��ʾ��λ	
									}
									break;
									}
					}
					break;
			}
			case 4://�¶�����
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 11);//��ʾС��  H
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 3);//��ʾ��λ
					Display_Wendu_1(4, 4);//��ʾʮλ
					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
					if(Temperatuer_Up<0)
					{
					  write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
						Temperature_I=0-Temperatuer_Up;
					}
					else
					{
						Temperature_I=Temperatuer_Up;
						write_addr_dat_n_char(7,0x08, 0);//����ʾ����
					}
					Display_Shidu_3(26,0);//��ʾ��λ
					write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,Temperature_I%10);//��ʾ��λ
					Display_Shidu_3(30, Temperature_I/10);//��ʾʮλ	
					switch(UP_Dowm)
					{
						case 0:
									{
									if(Temperatuer_Down>Temperatuer_Up)//���޴������޵����
									{
										//��ʾ�ڶ���Err
										Clr_Shidu();
										if(Temperatuer_Up<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											Temperature_I=0-Temperatuer_Up;
										}
										else
										{
											Temperature_I=Temperatuer_Up;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										Display_Shidu_2(26,8);//��ʾ��λ
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_2(28,8);//��ʾ��λ
										Display_Shidu_2(30,6);//��ʾʮλ		
									}
									else
									{
										Temperatuer_Up--; 
										if(Temperatuer_Up<-40) 
										Temperatuer_Up=-40;
										//��ʾ�ڶ���
										Clr_Shidu();
										if(Temperatuer_Up<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											Temperature_I=0-Temperatuer_Up;
										}
										else
										{
											Temperature_I=Temperatuer_Up;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										Display_Shidu_3(26,0);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,Temperature_I%10);//��ʾ��λ
										Display_Shidu_3(30, Temperature_I/10);//��ʾʮλ	
									}
									break;
									}
						case 1:
									{
									Temperatuer_Up++;
									if(Temperatuer_Up>80)
									Temperatuer_Up=80;
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_3(26,0);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,Temperature_I%10);//��ʾ��λ
									Display_Shidu_3(30, Temperature_I/10);//��ʾʮλ	
									break;
									}
					}
					break;
			}
			case 5://�¶�����  5C.L ��Ϊ4C.L
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 4);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 3);//��ʾ��λ
					//Display_Wendu_1(4, 5);//��ʾʮλ
				  Display_Wendu_1(4, 4);//��ʾʮλ
					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
					if(Temperatuer_Down<0)
					{
					  write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
						Temperature_O=0-Temperatuer_Down;
					}
					else
					{
						Temperature_O=Temperatuer_Down;
						write_addr_dat_n_char(7,0x08, 0);//����ʾ����
					}
					Display_Shidu_3(26,0);//��ʾ��λ
					write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,Temperature_O%10);//��ʾ��λ
					Display_Shidu_3(30,Temperature_O/10);//��ʾʮλ
					switch(UP_Dowm)
					{
						case 0:
									{
									Temperatuer_Down--; 
									if(Temperatuer_Down<-40)
									Temperatuer_Down=-40;
									//��ʾ�ڶ���
									Clr_Shidu();
									if(Temperatuer_Down<0)
									{
										write_addr_dat_n_char(7,0x08, 1);//��ʾ����
										Temperature_O=0-Temperatuer_Down;
									}
									else
									{
										Temperature_O=Temperatuer_Down;
										write_addr_dat_n_char(7,0x08, 0);//����ʾ����
									}
									Display_Shidu_3(26,0);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,Temperature_O%10);//��ʾ��λ
									Display_Shidu_3(30,Temperature_O/10);//��ʾʮλ	
									break;}
						case 1:
									{
									if(Temperatuer_Down>Temperatuer_Up)//���޴������޵����
									{
									  //��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(26,8);//��ʾ��λ
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_2(28,8);//��ʾ��λ
										Display_Shidu_2(30,6);//��ʾʮλ					
									}
									else
									{
										Temperatuer_Down++; 
										if(Temperatuer_Down>80) 
										Temperatuer_Down=80;
										//��ʾ�ڶ���
										Clr_Shidu();
										if(Temperatuer_Down<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����
											Temperature_O=0-Temperatuer_Down;
										}
										else
										{
											Temperature_O=Temperatuer_Down;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										Display_Shidu_3(26,0);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,Temperature_O%10);//��ʾ��λ
										Display_Shidu_3(30,Temperature_O/10);//��ʾʮλ	
									}										
									break;
									}
					}
					break;
			}
			case 7://ʪ������  6--7  6H.H--4H.H
			{
					write_addr_dat_n_char(7,0x08, 0);//����ʾ����
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 11);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 11);//��ʾ��λ
					//Display_Wendu_1(4, 6);//��ʾʮλ
				  Display_Wendu_1(4, 4);//��ʾʮλ  �޸�
					//Display_CHARS(huashidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
					Display_Shidu_1(26,0);//��ʾ��λ
				  write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,Humtidy_Up%10);//��ʾ��λ
					Display_Shidu_3(30,Humtidy_Up/10);//��ʾʮλ
					switch(UP_Dowm)
					{
						case 0:
									{
									if(Humtidy_Up<Humtidy_Down)//����С�����޵Ĵ������
									{
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(26,8);//��ʾ��λ
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_2(28,8);//��ʾ��λ
										Display_Shidu_2(30,6);//��ʾʮλ									
									}
									else
									{
										Humtidy_Up--; 
										if(Humtidy_Up<0) 
										Humtidy_Up=1;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_1(26,0);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,Humtidy_Up%10);//��ʾ��λ
										Display_Shidu_3(30,Humtidy_Up/10);//��ʾʮλ
									}
									break;
									}
						case 1:
									{
									Humtidy_Up++; 
									if(Humtidy_Up>99) 
									Humtidy_Up=99;
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_1(26,0);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,Humtidy_Up%10);//��ʾ��λ
									Display_Shidu_3(30,Humtidy_Up/10);//��ʾʮλ
									break;
									}
					}
					break;
			}
			case 8://ʪ������  7--8  7H.L--4H.L
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 4);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 11);//��ʾ��λ
					//Display_Wendu_1(4, 7);//��ʾʮλ
				  Display_Wendu_1(4, 4);//��ʾʮλ  �޸�
					//Display_CHARS(huashidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
					Display_Shidu_1(26,0);//��ʾ��λ
				  write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,Humtidy_Down%10);//��ʾ��λ
					Display_Shidu_3(30,Humtidy_Down/10);//��ʾʮλ
					switch(UP_Dowm)
					{
						case 0:
									{
									Humtidy_Down--; 
									if(Humtidy_Down<0)
									Humtidy_Down=0;
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_1(26,0);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,Humtidy_Down%10);//��ʾ��λ
									Display_Shidu_3(30,Humtidy_Down/10);//��ʾʮλ
									break;
									}
						case 1:
									{
									if(Humtidy_Up<Humtidy_Down)//����С�����޵Ĵ������
									{
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(26,8);//��ʾ��λ
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_2(28,8);//��ʾ��λ
										Display_Shidu_2(30,6);//��ʾʮλ									
									}
									else
									{
										Humtidy_Down++; 
										if(Humtidy_Down>99) 
										Humtidy_Down=99;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_1(26,0);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,Humtidy_Down%10);//��ʾ��λ
										Display_Shidu_3(30,Humtidy_Down/10);//��ʾʮλ									
									}	
									break;
									}
					}
					break;
			}
			case 12://���϶Ȼ��϶��л�  8--12   8C.F--6C.F
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 5);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 3);//��ʾ��λ
					//Display_Wendu_1(4, 8);//��ʾʮλ
				  Display_Wendu_1(4, 6);//��ʾʮλ
					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
					if(C_F==1)	
					{
						//��ʾ�ڶ���
						Display_Shidu_2(28,5);//��ʾF
					}
					else 
					{
						//��ʾ�ڶ���
						Display_Shidu_2(28,3);//��ʾC
					}	
					switch(UP_Dowm)
					{
						case 0:
									{
									if(C_F==0)	
										{
										C_F=1;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(28,5);//��ʾC
										}
									else 
										{
										C_F=0;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(28,3);//��ʾF
										}	
									break;
									}
						case 1:
									{
									if(C_F==0)	
										{
										C_F=1;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(28,5);//��ʾC
										}
									else 
										{
										C_F=0;
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_2(28,3);//��ʾF
										}	
									break;
									}
					}
					break;
			}
			case 13://Զ�̿���ѡ��      9--13   9F.r--7F.r
			{
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 8);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 5);//��ʾ��λ
					//Display_Wendu_1(4, 9);//��ʾʮλ
				  Display_Wendu_1(4, 7);//��ʾʮλ
					//��ʾ�ڶ���ON
					Clr_Shidu();
					if(RemoteControl==1) 
					{											
					//��ʾ�ڶ���ON
					Display_Shidu_2(26,10);
					//write_addr_dat_n_1bit(28,0x01, 1);
					Display_Shidu_3(28,  0);//
					}
					else 
					{
					//��ʾ�ڶ���OFF
					Display_Shidu_2(26,5);
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_2(28,  5);
					Display_Shidu_3(30,0);
					}
					switch(UP_Dowm)
					{
						case 0:
									{
											if(RemoteControl==0) 
											{											
											RemoteControl=1;
											//��ʾ�ڶ���ON
											Clr_Shidu();
											Display_Shidu_2(26,10);
											//write_addr_dat_n_1bit(28,0x01, 1);
											Display_Shidu_3(28,  0);//
											}

											else 
											{
											RemoteControl=0;
											//��ʾ�ڶ���OFF
											Clr_Shidu();
											Display_Shidu_2(26,5);
											//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
											Display_Shidu_2(28,  5);
											Display_Shidu_3(30,0);
											}

											break;
									}
						case 1:
									{
											if(RemoteControl==0) 
											{											
											RemoteControl=1;
											//��ʾ�ڶ���ON
											Clr_Shidu();
											Display_Shidu_2(26,10);
											//write_addr_dat_n_1bit(28,0x01, 1);
											Display_Shidu_3(28,  0);//
											//Display_Shidu_1(30, BaudRate/10000);//��ʾ��λ	
											}

											else 
											{
											RemoteControl=0;
											//��ʾ�ڶ���OFF
											Clr_Shidu();
											Display_Shidu_2(26,5);
											//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
											Display_Shidu_2(28,  5);
											Display_Shidu_3(30,0);
											}

											break;
									}
					}
					break;
			}
			case 14://��¼�Ƿ���      10--14  CO.1--8C.1
			{
					/*
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 1);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 0);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
				  */
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 1);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 3);//��ʾ��λ
					Display_Wendu_1(4, 8);//��ʾʮλ
					//��ʾ�ڶ���ON
					Clr_Shidu();
					if(RecordMode==0)
					{
					//��ʾ�ڶ���ON
							Display_Shidu_2(26,10);
							//write_addr_dat_n_1bit(28,0x01, 1);
							Display_Shidu_3(28,  0);//
					}
					if(RecordMode==1)
					{
						//��ʾ�ڶ���OFF
							Display_Shidu_2(26,5);
							Display_Shidu_2(28,  5);
							Display_Shidu_3(30,0);
					}
					switch(UP_Dowm)
					{
						case 0:
									{
									RecordMode--;
									if(RecordMode<0) 
									RecordMode=1;
									if(RecordMode==0)
									{
									//��ʾ�ڶ���ON
											Clr_Shidu();
											Display_Shidu_2(26,10);
											//write_addr_dat_n_1bit(28,0x01, 1);
											Display_Shidu_3(28,  0);//
									}
									if(RecordMode==1)
									{
										//��ʾ�ڶ���OFF
											Clr_Shidu();
											Display_Shidu_2(26,5);
											Display_Shidu_2(28,  5);
											Display_Shidu_3(30,0);
									}
									break;
									}
						case 1:
									{
									RecordMode++;
									if(RecordMode>1) 
									RecordMode=0;
									if(RecordMode==0)
									{
											//��ʾ�ڶ���ON
											Clr_Shidu();
											Display_Shidu_2(26,10);
											//write_addr_dat_n_1bit(28,0x01, 1);
											Display_Shidu_3(28,  0);//
									}
									if(RecordMode==1)
									{
										//��ʾ�ڶ���OFF
											Clr_Shidu();
											Display_Shidu_2(26,5);
											Display_Shidu_2(28,  5);
											Display_Shidu_3(30,0);
									}
									break;}
					}
					break;
			}
			case 15://��¼ģʽ          11--15  CO.2--8C.2
			{
					/*
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 2);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 0);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
				  */
				   //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 2);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 3);//��ʾ��λ
					Display_Wendu_1(4, 8);//��ʾʮλ
				
				
				
					//��ʾ�ڶ���
					Clr_Shidu();
						if(StorageMode==1)
						{
							//��ʾ�ڶ���
							Display_Shidu_2(26,3);//��ʾ��λ
							Display_Shidu_2(28,9);//��ʾ��λ
							Display_Shidu_2(30,3);//��ʾʮλ
						}
						else 
					  {
							//��ʾ�ڶ���
							Display_Shidu_2(26,0);//��ʾ��λ
							Display_Shidu_2(28,8);//��ʾ��λ
							Display_Shidu_3(30,0);//��ʾʮλ
						}
					switch(UP_Dowm)
					{
						case 0:
							{
						if(StorageMode==0)
						{
							StorageMode=1;
							//��ʾ�ڶ���
							Clr_Shidu();
							Display_Shidu_2(26,3);//��ʾ��λ
							Display_Shidu_2(28,9);//��ʾ��λ
							Display_Shidu_2(30,3);//��ʾʮλ
						}
						else 
					  {
							StorageMode=0;//Ĭ��
							//��ʾ�ڶ���
							Clr_Shidu();
							Display_Shidu_2(26,0);//��ʾ��λ
							Display_Shidu_2(28,8);//��ʾ��λ
							Display_Shidu_3(30,0);//��ʾʮλ
						}
						break;
						}
						case 1:
						{
						if(StorageMode==0)
						{
							StorageMode=1;
							//��ʾ�ڶ���
							Clr_Shidu();
							Display_Shidu_2(26,3);//��ʾ��λ
							//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
							Display_Shidu_2(28,9);//��ʾ��λ
							Display_Shidu_2(30,3);//��ʾʮλ
						}
						else 
					  {
							StorageMode=0;
							//��ʾ�ڶ���
							Clr_Shidu();
							Display_Shidu_2(26,0);//��ʾ��λ
							//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
							Display_Shidu_2(28,8);//��ʾ��λ
							Display_Shidu_3(30,0);//��ʾʮλ
						}
						break;
						}
					}
					break;
			}
			case 16://�����洢ʱ��      12--16  CO.3--8C.3
			{
					/*
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 3);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 0);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
				  */
				   //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 3);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 3);//��ʾ��λ
					Display_Wendu_1(4, 8);//��ʾʮλ
				
				
					//��ʾ�ڶ���
					Clr_Shidu();
					Display_Shidu_3(26,RecordTiming%10);//��ʾ��λ
					Display_Shidu_3(28,RecordTiming/10%10);//��ʾʮλ
					Display_Shidu_3(30,RecordTiming/100);//��ʾ��λ
					switch(UP_Dowm)
					{
						case 0:
									{
									RecordTiming--;
									if(RecordTiming<1) 
									RecordTiming=600;
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_3(26,RecordTiming%10);//��ʾ��λ
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_3(28,RecordTiming/10%10);//��ʾʮλ
									Display_Shidu_3(30,RecordTiming/100);//��ʾ��λ
									break;
									}
						case 1:
									{RecordTiming++;
									if(RecordTiming>600) 
									RecordTiming=1;
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_3(26,RecordTiming%10);//��ʾ��λ
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_3(28,RecordTiming/10%10);//��ʾʮλ
									Display_Shidu_3(30,RecordTiming/100);//��ʾ��λ
									break;
									}
					}
					break;
			}
			case 17://�����洢ʱ��      13--17  CO.4--8C.4
			{
					/*
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 4);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 0);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
				  */
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 4);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 3);//��ʾ��λ
					Display_Wendu_1(4, 8);//��ʾʮλ
				
				
				
					//��ʾ�ڶ���
					Clr_Shidu();
					Display_Shidu_3(26,WarningTiming%10);//��ʾ��λ
					Display_Shidu_3(28,WarningTiming/10%10);//��ʾʮλ
					Display_Shidu_3(30,WarningTiming/100);//��ʾ��λ
					switch(UP_Dowm)
					{
						case 0:{
						WarningTiming--;
						if(WarningTiming<1) 
						WarningTiming=600;
						//��ʾ�ڶ���
						Clr_Shidu();
						Display_Shidu_3(26,WarningTiming%10);//��ʾ��λ
						Display_Shidu_3(28,WarningTiming/10%10);//��ʾʮλ
						Display_Shidu_3(30,WarningTiming/100);//��ʾ��λ
						break;}
						case 1:{
						WarningTiming++;
						if(WarningTiming>600) 
						WarningTiming=1;
						//��ʾ�ڶ���
						Clr_Shidu();
						Display_Shidu_3(26,WarningTiming%10);//��ʾ��λ
						Display_Shidu_3(28,WarningTiming/10%10);//��ʾʮλ
						Display_Shidu_3(30,WarningTiming/100);//��ʾ��λ
						break;}
					}
					break;
			}
			case 18://��ʷ�������C05   14--18  CO.5--8C.5
			{
					/*
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 5);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 0);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
				  */
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 5);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 3);//��ʾ��λ
					Display_Wendu_1(4, 8);//��ʾʮλ
				
					//��ʾ�ڶ���
					Clr_Shidu();
					Display_Shidu_3(26,DeleteData%10);//��ʾ��λ
					//Display_Shidu_3(28,DeleteData/10%10);//��ʾʮλ
					switch(UP_Dowm)
					{
						case 0:{
						DeleteData--;
						if(DeleteData<0)
							DeleteData=1;
						//��ʾ�ڶ���
						Clr_Shidu();
						Display_Shidu_3(26,DeleteData%10);//��ʾ��λ
						//Display_Shidu_3(28,DeleteData/10%10);//��ʾʮλ
						break;}
						case 1:{
						DeleteData++;
						if(DeleteData>1)
							DeleteData=0;
						//��ʾ�ڶ���
						Clr_Shidu();
						Display_Shidu_3(26,DeleteData%10);//��ʾ��λ
						//Display_Shidu_3(28,DeleteData/10%10);//��ʾʮλ
						break;}
					}
					break;
			}
			case 6://�¶ȳ���������C06  15--6  CO.6--4C.d
			{
					//��ʾ��һ��
					/*
				  Clr_Wendu();
					Display_Wendu_1(0, 6);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 0);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
				  */
				  //��ʾ��һ��
					Clr_Wendu();
				   //�������и���Ҳȥ������
					write_addr_dat_n_char(7,0x08, 0);//5G
					write_addr_dat_n_char(10,0x08, 0);//5BC
				  //�������и���Ҳȥ������
				
				
					Display_Wendu_2(0, 0);//��ʾС��  d
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 3);//��ʾ��λ
					Display_Wendu_1(4, 4);//��ʾʮλ
					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
					Display_Shidu_3(26,TemperatureHys/1%10);//��ʾ��λ
					write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,TemperatureHys/10%10);//��ʾ��λ
					Display_Shidu_3(30,TemperatureHys/100);//��ʾʮλ
					switch(UP_Dowm)
					{
						case 0:
									{
									TemperatureHys--; 
									if(TemperatureHys<0)
									TemperatureHys=0;
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_3(26,TemperatureHys/1%10);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,TemperatureHys/10%10);//��ʾ��λ
									Display_Shidu_3(30,TemperatureHys/100);//��ʾʮλ	
									break;}
						case 1:
									{
										TemperatureHys++; 
										//if(TemperatureHys>120) 
										//TemperatureHys=120;
										
										if(TemperatureHys>((Temperatuer_Up-Temperatuer_Down)/3*10))
										TemperatureHys=((Temperatuer_Up-Temperatuer_Down)/3*10);
										
										
										
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_3(26,TemperatureHys/1%10);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,TemperatureHys/10%10);//��ʾ��λ
										Display_Shidu_3(30,TemperatureHys/100);//��ʾʮλ	
									break;
									}
					}
					break;
			}

			case 9://ʪ�ȳ���������C07 16--9   CO.7--4H.d
			{
					/*
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 7);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 0);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
					 write_addr_dat_n_char(28,0x01, HumtidyHys%10);//P2
				*/
				//��ʾ��һ��
				 //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 0);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 11);//��ʾ��λ
					//Display_Wendu_1(4, 7);//��ʾʮλ
				  Display_Wendu_1(4, 4);//��ʾʮλ  �޸�
          write_addr_dat_n_char(28,0x01, HumtidyHys%10);//P2
				

					//��ʾ�ڶ���
					Clr_Shidu();
					Display_Shidu_1(26,0);//��ʾ��λ
				//	write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,HumtidyHys/10%10);//��ʾ��λ
					Display_Shidu_3(30,HumtidyHys/100);//��ʾʮλ
					switch(UP_Dowm)
					{
						case 0:
									{
									HumtidyHys--; 
									if(HumtidyHys<0)
									HumtidyHys=0;
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_1(26,HumtidyHys%10);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,HumtidyHys/10%10);//��ʾ��λ
									Display_Shidu_3(30,HumtidyHys/100);//��ʾʮλ	
									break;}
						case 1:
									{

										HumtidyHys++; 
										//if(HumtidyHys>100) 
										//HumtidyHys=100;
										if(HumtidyHys>((Humtidy_Up-Humtidy_Down)/3*10))
										HumtidyHys=((Humtidy_Up-Humtidy_Down)/3*10);
										
										
										
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_1(26,HumtidyHys%10);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,HumtidyHys/10%10);//��ʾ��λ
										Display_Shidu_3(30,HumtidyHys/100);//��ʾʮλ									
									break;
									}
					}
					break;
			}

			case 19://C08�¶�4ma       17--19  CO.8--9C.0
			{
					/*
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 8);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 0);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
				  */
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 0);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 3);//��ʾ��λ
					Display_Wendu_1(4, 9);//��ʾʮλ
					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
				
				
					//��ʾ�ڶ���
					Clr_Shidu();
					if(Temperature_4ma<0)
					{
					  write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
						TemperatureDisplay_4ma=0-Temperature_4ma;
					}
					else
					{
						TemperatureDisplay_4ma=Temperature_4ma;
						write_addr_dat_n_char(7,0x08, 0);//����ʾ����
					}
					Display_Shidu_3(26,0);//��ʾ��λ
					write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,TemperatureDisplay_4ma%10);//��ʾ��λ
					Display_Shidu_3(30, TemperatureDisplay_4ma/10);//��ʾʮλ	
					switch(UP_Dowm)
					{
						case 0:
									{
									if(Temperature_4ma>Temperature_20ma)//���޴������޵����
									{
										//��ʾ�ڶ���Err
										Clr_Shidu();
										if(Temperature_4ma<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											TemperatureDisplay_4ma=0-Temperature_4ma;
										}
										else
										{
											TemperatureDisplay_4ma=Temperature_4ma;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										Display_Shidu_2(26,8);//��ʾ��λ
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_2(28,8);//��ʾ��λ
										Display_Shidu_2(30,6);//��ʾʮλ		
									}
									else
									{
										Temperature_4ma--; 
										if(Temperature_4ma<-40) 
										Temperature_4ma=-40;
										//��ʾ�ڶ���
										Clr_Shidu();
										if(Temperature_4ma<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											TemperatureDisplay_4ma=0-Temperature_4ma;
										}
										else
										{
											TemperatureDisplay_4ma=Temperature_4ma;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										Display_Shidu_3(26,0);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,TemperatureDisplay_4ma%10);//��ʾ��λ
										Display_Shidu_3(30, TemperatureDisplay_4ma/10);//��ʾʮλ	
									}
									break;
									}
						case 1:
									{
									Temperature_4ma++;
									if(Temperature_4ma>80)
									Temperature_4ma=80;
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_3(26,0);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,TemperatureDisplay_4ma%10);//��ʾ��λ
									Display_Shidu_3(30, TemperatureDisplay_4ma/10);//��ʾʮλ	
									break;
									}
					}
					break;
			}
			case 20://C09�¶�20ma      18--20  CO.9--9C.1
			{
					/*
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 9);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 0);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
				  */
				   //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 1);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 3);//��ʾ��λ
					Display_Wendu_1(4, 9);//��ʾʮλ
					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
				
				
				
					//��ʾ�ڶ���
					Clr_Shidu();
					if(Temperature_20ma<0)
					{
					  write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
						TemperatureDisplay_20ma=0-Temperature_20ma;
					}
					else
					{
						TemperatureDisplay_20ma=Temperature_20ma;
						write_addr_dat_n_char(7,0x08, 0);//����ʾ����
					}
					Display_Shidu_3(26,0);//��ʾ��λ
					write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,TemperatureDisplay_20ma%10);//��ʾ��λ
					Display_Shidu_3(30, TemperatureDisplay_20ma/10);//��ʾʮλ	
					switch(UP_Dowm)
					{
						case 0:
									{
									if(Temperature_4ma>Temperature_20ma)//���޴������޵����
									{
										//��ʾ�ڶ���Err
										Clr_Shidu();
										if(Temperature_20ma<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											TemperatureDisplay_20ma=0-Temperature_20ma;
										}
										else
										{
											TemperatureDisplay_20ma=Temperature_20ma;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										Display_Shidu_2(26,8);//��ʾ��λ
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_2(28,8);//��ʾ��λ
										Display_Shidu_2(30,6);//��ʾʮλ		
									}
									else
									{
										Temperature_20ma--; 
										if(Temperature_20ma<-40) 
										Temperature_20ma=-40;
										//��ʾ�ڶ���
										Clr_Shidu();
										if(Temperature_20ma<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											TemperatureDisplay_20ma=0-Temperature_20ma;
										}
										else
										{
											TemperatureDisplay_20ma=Temperature_20ma;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										Display_Shidu_3(26,0);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,TemperatureDisplay_20ma%10);//��ʾ��λ
										Display_Shidu_3(30, TemperatureDisplay_20ma/10);//��ʾʮλ	
									}
									break;
									}
						case 1:
									{
									Temperature_20ma++;
									if(Temperature_20ma>80)
									Temperature_20ma=80;
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_3(26,0);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,TemperatureDisplay_20ma%10);//��ʾ��λ
									Display_Shidu_3(30, TemperatureDisplay_20ma/10);//��ʾʮλ	
									break;
									}
					}
					break;
			}
			case 21://C10ʪ��4ma       19--21  C1.0--9H.0
			{
					/*
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 0);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 1);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
					//write_addr_dat_n_char(28,0x01, 1);//P2
					//Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
				  */
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 0);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 11);//��ʾ��λ
					Display_Wendu_1(4, 9);//��ʾʮλ
					//write_addr_dat_n_char(28,0x01, 1);//P2
					//Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
				
					//��ʾ�ڶ���
					Clr_Shidu();
					if(Humtidy_4ma<0)
					{
					  write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
						HumtidyDisplay_4ma=0-Humtidy_4ma;
					}
					else
					{
						HumtidyDisplay_4ma=Humtidy_4ma;
						write_addr_dat_n_char(7,0x08, 0);//����ʾ����
					}
					Display_Shidu_1(26,0);//��ʾ��λ
					write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,HumtidyDisplay_4ma%10);//��ʾ��λ
					Display_Shidu_3(30, HumtidyDisplay_4ma/10);//��ʾʮλ	
					switch(UP_Dowm)
					{
						case 0:
									{
									if(Humtidy_4ma>Humtidy_20ma)//���޴������޵����
									{
										//��ʾ�ڶ���Err
										Clr_Shidu();
										if(Humtidy_4ma<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											HumtidyDisplay_4ma=0-Humtidy_4ma;
										}
										else
										{
											HumtidyDisplay_4ma=Humtidy_4ma;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										Display_Shidu_2(26,8);//��ʾ��λ
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_2(28,8);//��ʾ��λ
										Display_Shidu_2(30,6);//��ʾʮλ		
									}
									else
									{
										Humtidy_4ma--; 
										if(Humtidy_4ma<0) 
										Humtidy_4ma=0;
										//��ʾ�ڶ���
										Clr_Shidu();
										if(Humtidy_4ma<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											HumtidyDisplay_4ma=0-Humtidy_4ma;
										}
										else
										{
											HumtidyDisplay_4ma=Humtidy_4ma;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										Display_Shidu_1(26,0);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,HumtidyDisplay_4ma%10);//��ʾ��λ
										Display_Shidu_3(30, HumtidyDisplay_4ma/10);//��ʾʮλ	
									}
									break;
									}
						case 1:
									{
									Humtidy_4ma++;
									if(Humtidy_4ma>99)
									Humtidy_4ma=9;
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_1(26,0);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,HumtidyDisplay_4ma%10);//��ʾ��λ
									Display_Shidu_3(30, HumtidyDisplay_4ma/10);//��ʾʮλ	
									break;
									}
					}
					break;
			}
			case 22://C11ʪ��20ma      20--22  C1.1--9H.1
			{
					/*
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 1);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 1);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
					//write_addr_dat_n_char(28,0x01, 1);//P2
					//Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
				  */
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 1);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 11);//��ʾ��λ
					Display_Wendu_1(4, 9);//��ʾʮλ
					//write_addr_dat_n_char(28,0x01, 1);//P2
					//Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
					if(Humtidy_20ma<0)
					{
					  write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
						HumtidyDisplay_20ma=0-Humtidy_20ma;
					}
					else
					{
						HumtidyDisplay_20ma=Humtidy_20ma;
						write_addr_dat_n_char(7,0x08, 0);//����ʾ����
					}
					Display_Shidu_1(26,0);//��ʾ��λ
					write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,HumtidyDisplay_20ma%10);//��ʾ��λ
					Display_Shidu_3(30, HumtidyDisplay_20ma/10);//��ʾʮλ	
					switch(UP_Dowm)
					{
						case 0:
									{
									if(Humtidy_4ma>Humtidy_20ma)//���޴������޵����
									{
										//��ʾ�ڶ���Err
										Clr_Shidu();
										if(Humtidy_20ma<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											HumtidyDisplay_20ma=0-Humtidy_20ma;
										}
										else
										{
											HumtidyDisplay_20ma=Humtidy_20ma;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										Display_Shidu_2(26,8);//��ʾ��λ
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_2(28,8);//��ʾ��λ
										Display_Shidu_2(30,6);//��ʾʮλ		
									}
									else
									{
										Humtidy_20ma--; 
										if(Humtidy_20ma<0) 
										Humtidy_20ma=0;
										//��ʾ�ڶ���
										Clr_Shidu();
										if(Humtidy_20ma<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											HumtidyDisplay_20ma=0-Humtidy_20ma;
										}
										else
										{
											HumtidyDisplay_20ma=Humtidy_20ma;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										Display_Shidu_1(26,0);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,HumtidyDisplay_20ma%10);//��ʾ��λ
										Display_Shidu_3(30, HumtidyDisplay_20ma/10);//��ʾʮλ	
									}
									break;
									}
						case 1:
									{
									Humtidy_20ma++;
									if(Humtidy_20ma>99)
									Humtidy_20ma=99;
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_1(26,0);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,HumtidyDisplay_20ma%10);//��ʾ��λ
									Display_Shidu_3(30, HumtidyDisplay_20ma/10);//��ʾʮλ	
									break;
									}
					}
					break;
			}
			case 10://C12�¶�У׼      21--10 C1.2--5C.0
			{
					/*
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 2);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 1);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
				  */
				  //��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 0);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 3);//��ʾ��λ
					Display_Wendu_1(4, 5);//��ʾʮλ
					Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
				
					//��ʾ�ڶ���
					Clr_Shidu();
					if(TemperatureCalibration<0)
					{
					  write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
						TemperatureCalibrationDisplay=0-TemperatureCalibration;
					}
					else
					{
						TemperatureCalibrationDisplay=TemperatureCalibration;
						write_addr_dat_n_char(7,0x08, 0);//����ʾ����
					}
					Display_Shidu_3(26,TemperatureCalibrationDisplay/1%10);//��ʾ��λ
					write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,TemperatureCalibrationDisplay/10%10);//��ʾ��λ
					Display_Shidu_3(30, TemperatureCalibrationDisplay/100);//��ʾʮλ	
					switch(UP_Dowm)
					{
						case 0:
									{
										TemperatureCalibration--; 
										if(TemperatureCalibration<-100) 
										TemperatureCalibration=-100;
										if(TemperatureCalibration<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											TemperatureCalibrationDisplay=0-TemperatureCalibration;
										}
										else
										{
											TemperatureCalibrationDisplay=TemperatureCalibration;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_3(26,TemperatureCalibrationDisplay%10);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,TemperatureCalibrationDisplay/10%10);//��ʾ��λ
										Display_Shidu_3(30, TemperatureCalibrationDisplay/100);//��ʾʮλ	
									break;
									}
						case 1:
									{
									TemperatureCalibration++;
									if(TemperatureCalibration>100)
									TemperatureCalibration=100;
									if(TemperatureCalibration<0)
									{
										write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
										TemperatureCalibrationDisplay=0-TemperatureCalibration;
									}
									else
									{
										TemperatureCalibrationDisplay=TemperatureCalibration;
										write_addr_dat_n_char(7,0x08, 0);//����ʾ����
									}
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_3(26,TemperatureCalibrationDisplay%10);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,TemperatureCalibrationDisplay/10%10);//��ʾ��λ
									Display_Shidu_3(30, TemperatureCalibrationDisplay/100);//��ʾʮλ	
									break;
									}
					}
					break;
			}
			case 11://C13ʪ��У׼      22--11 C1.3--5H.0
				{
					/*
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_1(0, 3);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_1(2, 1);//��ʾ��λ
					Display_Wendu_2(4, 3);//��ʾʮλ
					//write_addr_dat_n_char(28,0x01, 1);//P2
					//Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
					*/
					Clr_Wendu();
					Display_Wendu_1(0, 0);//��ʾС��
				  write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 11);//��ʾ��λ
					Display_Wendu_1(4, 5);//��ʾʮλ
					//write_addr_dat_n_char(28,0x01, 1);//P2
					//Display_CHARS(sheshidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
					if(HumtidyCalibration<0)
					{
					  write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
						HumtidyCalibrationDisplay=0-HumtidyCalibration;
					}
					else
					{
						HumtidyCalibrationDisplay=HumtidyCalibration;
						write_addr_dat_n_char(7,0x08, 0);//����ʾ����
					}
					Display_Shidu_1(26,HumtidyCalibrationDisplay/1%10);//��ʾ��λ
					write_addr_dat_n_char(28,0x01, 1);//P2
					//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
					Display_Shidu_1(28,HumtidyCalibrationDisplay/10%10);//��ʾ��λ
					Display_Shidu_3(30, HumtidyCalibrationDisplay/100);//��ʾʮλ	
					switch(UP_Dowm)
					{
						case 0:
									{
										HumtidyCalibration--; 
										if(HumtidyCalibration<-100) 
										HumtidyCalibration=-100;
										if(HumtidyCalibration<0)
										{
											write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
											HumtidyCalibrationDisplay=0-HumtidyCalibration;
										}
										else
										{
											HumtidyCalibrationDisplay=HumtidyCalibration;
											write_addr_dat_n_char(7,0x08, 0);//����ʾ����
										}
										//��ʾ�ڶ���
										Clr_Shidu();
										Display_Shidu_1(26,HumtidyCalibrationDisplay%10);//��ʾ��λ
										write_addr_dat_n_char(28,0x01, 1);//P2
										//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
										Display_Shidu_1(28,HumtidyCalibrationDisplay/10%10);//��ʾ��λ
										Display_Shidu_3(30, HumtidyCalibrationDisplay/100);//��ʾʮλ	
									break;
									}
						case 1:
									{
									HumtidyCalibration++;
									if(HumtidyCalibration>100)
									HumtidyCalibration=100;
									if(HumtidyCalibration<0)
									{
										write_addr_dat_n_char(7,0x08, 1);//��ʾ����//write_addr_dat_n_char(2,0x08, 1);//P1write_addr_dat_n_char(28,0x01, 1);//P2
										HumtidyCalibrationDisplay=0-HumtidyCalibration;
									}
									else
									{
										HumtidyCalibrationDisplay=HumtidyCalibration;
										write_addr_dat_n_char(7,0x08, 0);//����ʾ����
									}
									//��ʾ�ڶ���
									Clr_Shidu();
									Display_Shidu_1(26,HumtidyCalibrationDisplay%10);//��ʾ��λ
									write_addr_dat_n_char(28,0x01, 1);//P2
									//write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
									Display_Shidu_1(28,HumtidyCalibrationDisplay/10%10);//��ʾ��λ
									Display_Shidu_3(30, HumtidyCalibrationDisplay/100);//��ʾʮλ	
									break;
									}
					}
					break;
			}
			case 23://ʣ�����
			{
				//��ʾ��ǰʣ�����
					//��ʾ��һ��
					Clr_Wendu();
					Display_Wendu_2(0, 8);//��ʾС��
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 7);//��ʾ��λ
					Display_Wendu_2(4, 1);//��ʾʮλ
					//Display_CHARS(huashidu,1);//��ʾ��ʪ�ȵķ���
					write_addr_dat_n_char(7,0x08, 0);//����ʾ����
					//��ʾ�ڶ���OFF
					Clr_Shidu();
					if((V_BAT*2)<Battery[0]){	
						Display_Shidu_3(28,1);
						Display_Shidu_3(26,0);}
					else if((V_BAT*2)<Battery[1]&&(V_BAT*2)>=Battery[0]){
						Display_Shidu_3(28,2);
						Display_Shidu_3(26,0);}
					else if((V_BAT*2)<Battery[2]&&(V_BAT*2)>=Battery[1]){
						Display_Shidu_3(28,3);
						Display_Shidu_3(26,0);}
					else if((V_BAT*2)<Battery[3]&&(V_BAT*2)>=Battery[2]){
						Display_Shidu_3(28,4);
						Display_Shidu_3(26,0);}
					else if((V_BAT*2)<Battery[4]&&(V_BAT*2)>=Battery[3]){
						Display_Shidu_3(28,5);
						Display_Shidu_3(26,0);}
					else if((V_BAT*2)<Battery[5]&&(V_BAT*2)>=Battery[4]){
						Display_Shidu_3(28,6);
						Display_Shidu_3(26,0);}
					else if((V_BAT*2)<Battery[6]&&(V_BAT*2)>=Battery[5]){
						Display_Shidu_3(28,7);
						Display_Shidu_3(26,0);}
					else if((V_BAT*2)<Battery[7]&&(V_BAT*2)>=Battery[6]){
						Display_Shidu_3(28,8);
						Display_Shidu_3(26,0);}
					else if((V_BAT*2)<Battery[8]&&(V_BAT*2)>=Battery[7]){
						Display_Shidu_3(28,9);
						Display_Shidu_3(26,0);}
					else if((V_BAT*2)<Battery[9]&&(V_BAT*2)>=Battery[8]||(V_BAT*2>=Battery[9])){
						Display_Shidu_3(30,1);
						Display_Shidu_3(28,0);
						Display_Shidu_3(26,0);}	 

					break;
			}
			case 24://������ѡ��
			{
					//��ʾ��һ��SEL
					Clr_Wendu();
					Display_Wendu_2(0, 4);//��ʾС��
					//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
					Display_Wendu_2(2, 6);//��ʾ��λ
					Display_Wendu_1(4, 5);//��ʾʮλ
					//Display_CHARS(huashidu,1);//��ʾ��ʪ�ȵķ���
					//��ʾ�ڶ���
					Clr_Shidu();
					Display_Shidu_3(26,SensorSelected);//��ʾ��
					switch(UP_Dowm)
					{
						case 0:{
						SensorSelected--;
						if(SensorSelected<1)
						SensorSelected=4;
						//��ʾ�ڶ���
						Clr_Shidu();
						Display_Shidu_3(26,SensorSelected);//��ʾ��
						break;}
						case 1:{
						SensorSelected++;
						if(SensorSelected>4)
						SensorSelected=1;
						//��ʾ�ڶ���
						Clr_Shidu();
						Display_Shidu_3(26,SensorSelected);//��ʾ��
						break;}
					}
					break;
			}
			case 25://��������
			{
					//ʱ������
					//��ʾ��һ��16
					Clr_Wendu();
					//Display_Wendu_2(0, 4);//��ʾС��
					Display_Wendu_1(2, 6);//��ʾ��λ
					Display_Wendu_1(4, 1);//��ʾʮλ
					Clr_Shidu();
					RTC_Control=1;
					OnlyDispalyTime=0;
					RTCTimeParametersInit=1;
					break;
			}
		}
			
}

//Lemon
int Temperature_Warning_Data(float Temperature,int WarningEnable)
{
//�¶�
		if(Temperature>=Temperatuer_Up)//��������,Ӧ�ð���������ͼ̵����Ķ���
		{

					Display_CHARS(gaobaojing1,1);
					Display_CHARS(dibaojing1,0);
					//Display_CHARS(fengming1,1);
					Display_CHARS(zhengchang1,0);
			if(WarningEnable)
			{
					//GPIO_ResetBits(GPIOB,GPIO_Pin_4);  //�̵���1
					GPIO_SetBits(GPIOB,GPIO_Pin_4);  //�̵���1
					GPIO_SetBits(GPIOA,GPIO_Pin_6);    //��������
					TemperatureHighWarning=1;
			}
			return 1;
		}
		else if(Temperature<=Temperatuer_Down)//��������
		{
							Display_CHARS(gaobaojing1,0);
							Display_CHARS(dibaojing1,1);
							//Display_CHARS(fengming1,1);
							Display_CHARS(zhengchang1,0);
					if(WarningEnable)
					{
							//GPIO_ResetBits(GPIOB,GPIO_Pin_4);  //�̵���1
							GPIO_SetBits(GPIOB,GPIO_Pin_4);  //�̵���1
							GPIO_SetBits(GPIOA,GPIO_Pin_6);    //��������
							TemperatureLowWarning=1;
						
					}
					return 1;

				
		}		
		if(TemperatureLowWarning==0&&TemperatureHighWarning==0)//���������Ǵӱ���״̬���ģ�һֱ��������״̬
		{
			Display_CHARS(gaobaojing1,0);
			Display_CHARS(dibaojing1,0);
			//Display_CHARS(fengming1,0);
			Display_CHARS(zhengchang1,1);
			//GPIO_SetBits(GPIOB,GPIO_Pin_4);  //�ؼ̵���1
			GPIO_ResetBits(GPIOB,GPIO_Pin_4);  //�ؼ̵���1
			GPIO_ResetBits(GPIOA,GPIO_Pin_6);//�ط�����
			return 0;
		}
		else if(TemperatureLowWarning&&Temperature>(Temperatuer_Down+((float)TemperatureHys/10)))//���޳�����֮�⣬�̵�������
		{
//			Display_CHARS(gaobaojing1,0);
//			Display_CHARS(dibaojing1,0);
//			//Display_CHARS(fengming1,0);
//			Display_CHARS(zhengchang1,1);
//			//GPIO_SetBits(GPIOB,GPIO_Pin_4);  //�ؼ̵���1
			GPIO_ResetBits(GPIOB,GPIO_Pin_4);  //�ؼ̵���1
			//GPIO_ResetBits(GPIOA,GPIO_Pin_6);//�ط�����
			TemperatureLowWarning=0;
			return 0;
		}
		else if(TemperatureLowWarning&&Temperature<=(Temperatuer_Down+((float)TemperatureHys/10)))//���޳�����֮�ڣ��̵����������������Ļָ�
		{
			Display_CHARS(gaobaojing1,0);
			Display_CHARS(dibaojing1,0);
			//Display_CHARS(fengming1,0);
			Display_CHARS(zhengchang1,1);
			//GPIO_SetBits(GPIOB,GPIO_Pin_4);  //�ؼ̵���1
			//GPIO_ResetBits(GPIOB,GPIO_Pin_4);  //�ؼ̵���1
			GPIO_ResetBits(GPIOA,GPIO_Pin_6);//�ط�����
			return 0;
		}
		else if(TemperatureHighWarning&&Temperature<(Temperatuer_Up-((float)TemperatureHys/10)))//���޳�����֮�⣬�̵�������
		{
			//Display_CHARS(gaobaojing1,0);
			//Display_CHARS(dibaojing1,0);
			//Display_CHARS(fengming1,0);
			//Display_CHARS(zhengchang1,1);
			//GPIO_SetBits(GPIOB,GPIO_Pin_4);  //�ؼ̵���1
			GPIO_ResetBits(GPIOB,GPIO_Pin_4);  //�ؼ̵���1
			//GPIO_ResetBits(GPIOA,GPIO_Pin_6);//�ط�����
			TemperatureHighWarning=0;
			return 0;
		}
		else if(TemperatureHighWarning&&Temperature>=(Temperatuer_Up-((float)TemperatureHys/10)))//���޳�����֮�⣬�̵����������������Ļָ�
		{
			Display_CHARS(gaobaojing1,0);
			Display_CHARS(dibaojing1,0);
			//Display_CHARS(fengming1,0);
			Display_CHARS(zhengchang1,1);
			//GPIO_SetBits(GPIOB,GPIO_Pin_4);  //�ؼ̵���1
			//GPIO_ResetBits(GPIOB,GPIO_Pin_4);  //�ؼ̵���1
			GPIO_ResetBits(GPIOA,GPIO_Pin_6);//�ط�����
			return 0;
		}
}
int Humitidy_Warning_Data(float Humitidy,int WarningEnable,int warning)
{
//ʪ��
	if(Humitidy>=Humtidy_Up)//��������
	{

					Display_CHARS(gaobaojing2,1);
					Display_CHARS(dibaojing2,0);
					//Display_CHARS(fengming2,1);
					Display_CHARS(zhengchang2,0);
			if(WarningEnable)
			{
				//	GPIO_ResetBits(GPIOB,GPIO_Pin_3);  //�̵���2
					GPIO_SetBits(GPIOB,GPIO_Pin_3);  //�̵���2
					GPIO_SetBits(GPIOA,GPIO_Pin_6);    //��������
					HumidityHighWarning=1;
					
			}
				return 1;
	}
	else 	if(Humitidy<=Humtidy_Down)//��������
	{
		
			

						Display_CHARS(gaobaojing2,0);
						Display_CHARS(dibaojing2,1);
						//Display_CHARS(fengming2,1);
						Display_CHARS(zhengchang2,0);
				if(WarningEnable)
				{
						//GPIO_ResetBits(GPIOB,GPIO_Pin_3);  //�̵���2
						GPIO_SetBits(GPIOB,GPIO_Pin_3);  //�̵���2
						GPIO_SetBits(GPIOA,GPIO_Pin_6);    //��������
						HumidityHighWarning=1;
				}
				return 1;
	}
	if(HumidityLowWarning==0&&HumidityHighWarning==0)//���������Ǵӱ���״̬���ģ�һֱ��������״̬
		{
						Display_CHARS(gaobaojing2,0);
						Display_CHARS(dibaojing2,0);
						//Display_CHARS(fengming2,0);
						Display_CHARS(zhengchang2,1);
						//GPIO_SetBits(GPIOB,GPIO_Pin_3);  //�ؼ̵���2
						GPIO_ResetBits(GPIOB,GPIO_Pin_3);  //�ؼ̵���2
						HumidityHighWarning=0;
						if(warning==0)
						{
						GPIO_ResetBits(GPIOA,GPIO_Pin_6);//��Ϊֻ��һ����������ֻ�����¶�û�б�����ǰ���¹ط����������¶��Ѿ������ˣ��Ͳ�����������						
						}

						return 0;
			
		}
		else if(HumidityLowWarning&&Humitidy>(Humtidy_Down+((float)HumtidyHys/10)))//���޳�����֮�⣬�̵�������
		{
//						Display_CHARS(gaobaojing2,0);
//						Display_CHARS(dibaojing2,0);
//						Display_CHARS(fengming2,0);
//						Display_CHARS(zhengchang2,1);
//						//GPIO_SetBits(GPIOB,GPIO_Pin_3);  //�ؼ̵���2
						GPIO_ResetBits(GPIOB,GPIO_Pin_3);  //�ؼ̵���2
//						if(warning==0)
//						{
//						GPIO_ResetBits(GPIOA,GPIO_Pin_6);//��Ϊֻ��һ����������ֻ�����¶�û�б�����ǰ���¹ط����������¶��Ѿ������ˣ��Ͳ�����������						
//						}
							HumidityLowWarning=0;
						return 0;
		}
		else if(HumidityLowWarning&&Humitidy<=(Humtidy_Down+((float)HumtidyHys/10)))//���޳�����֮�ڣ����˼̵���֮�������
		{
						Display_CHARS(gaobaojing2,0);
						Display_CHARS(dibaojing2,0);
						//Display_CHARS(fengming2,0);
						Display_CHARS(zhengchang2,1);
						//GPIO_SetBits(GPIOB,GPIO_Pin_3);  //�ؼ̵���2
						//GPIO_ResetBits(GPIOB,GPIO_Pin_3);  //�ؼ̵���2
						if(warning==0)
						{
						GPIO_ResetBits(GPIOA,GPIO_Pin_6);//��Ϊֻ��һ����������ֻ�����¶�û�б�����ǰ���¹ط����������¶��Ѿ������ˣ��Ͳ�����������						
						}
						
						return 0;
		}
		else if(HumidityHighWarning&&Humitidy<(Humtidy_Up-((float)HumtidyHys/10)))//���޳�����֮�⣬�̵�������
		{
//						Display_CHARS(gaobaojing2,0);
//						Display_CHARS(dibaojing2,0);
//						Display_CHARS(fengming2,0);
//						Display_CHARS(zhengchang2,1);
//						//GPIO_SetBits(GPIOB,GPIO_Pin_3);  //�ؼ̵���2
						GPIO_ResetBits(GPIOB,GPIO_Pin_3);  //�ؼ̵���2
//						if(warning==0)
//						{
//						GPIO_ResetBits(GPIOA,GPIO_Pin_6);//��Ϊֻ��һ����������ֻ�����¶�û�б�����ǰ���¹ط����������¶��Ѿ������ˣ��Ͳ�����������						
//						}
							HumidityHighWarning=0;
						return 0;
		}
		else if(HumidityHighWarning&&Humitidy>=(Humtidy_Up-((float)HumtidyHys/10)))//���޳�����֮�ڣ����˼̵����������
		{
						Display_CHARS(gaobaojing2,0);
						Display_CHARS(dibaojing2,0);
					//	Display_CHARS(fengming2,0);
						Display_CHARS(zhengchang2,1);
						//GPIO_SetBits(GPIOB,GPIO_Pin_3);  //�ؼ̵���2
					//	GPIO_ResetBits(GPIOB,GPIO_Pin_3);  //�ؼ̵���2
						if(warning==0)
						{
						GPIO_ResetBits(GPIOA,GPIO_Pin_6);//��Ϊֻ��һ����������ֻ�����¶�û�б�����ǰ���¹ط����������¶��Ѿ������ˣ��Ͳ�����������						
						}

						return 0;
		}
}

//StorgeNum��Ҫ�޸�
//void Storage(float Temperature,float Humitidy,int StorageMode)
//{
//		BUFFER_SIZE2=4;
//		THtoStorage[0]=Temperature;
//		THtoStorage[1]=Humitidy;
//		THtoStorage_4byte[0]=((unsigned short int)(THtoStorage[0]*10)&0xff00)>>8;
//		THtoStorage_4byte[1]=((unsigned short int)(THtoStorage[0]*10)&0x00ff);
//		THtoStorage_4byte[2]=((unsigned short int)(THtoStorage[1]*10)&0xff00)>>8;
//		THtoStorage_4byte[3]=((unsigned short int)(THtoStorage[1]*10)&0x00ff);
//	
//		if(StorageMode==0)   //˳��洢������Ϊֹ
//		{
//			if(StorgeNum<65532)
//			{
//						sEE_WriteBuffer((u8*)THtoStorage_4byte,StorgeNum,BUFFER_SIZE2);
//						delay_ms(100);
//						sEE_ReadBuffer(THtoStorage_Read,StorgeNum,(uint16_t *)(&BUFFER_SIZE2));
//						//StorageParameters();
//						StorgeNum=StorgeNum+4;
//						
//			}
//			else //�����Ѵ���
//			{
//			}
//		}
//		else                 //ѭ���洢������
//		{
//			if(StorgeNum<65532)
//			{
//						sEE_WriteBuffer((u8*)THtoStorage_4byte,StorgeNum, BUFFER_SIZE2);
//						delay_ms(100);
//						sEE_ReadBuffer(THtoStorage_Read,StorgeNum,(uint16_t *)(&BUFFER_SIZE2));
//						//StorageParameters();
//						StorgeNum=StorgeNum+4;
//			}
//			else 
//			{
//						StorgeNum=16;
//			}
//		}
//}

//����洢ʱ��������
void Storage(float Temperature,float Humitidy,u8 Year,u8 Month,u8 Date,u8 Hour,u8 Min,u8 Sec,int StorageMode1)//��������ǰ��64K---7.5
{
			volatile uint16_t BUFFER_SIZE2;//�����ں����ڲ������ڵ���EEPROM��д����֮ǰ������һ�θ�ֵ
	
//		BUFFER_SIZE2=4;
			BUFFER_SIZE2=10;
			if((int)(Temperature)>>15==1)
			{
				Temperature=65536-Temperature;//���¶��Բ�����ʽ�ϴ�
			}
			THtoStorage[0]=Temperature;
			THtoStorage[1]=Humitidy;
//		THtoStorage_4byte[0]=((unsigned short int)(THtoStorage[0]*10)&0xff00)>>8;
//		THtoStorage_4byte[1]=((unsigned short int)(THtoStorage[0]*10)&0x00ff);
//		THtoStorage_4byte[2]=((unsigned short int)(THtoStorage[1]*10)&0xff00)>>8;
//		THtoStorage_4byte[3]=((unsigned short int)(THtoStorage[1]*10)&0x00ff);
		
		//THtoStorage_10byte[0]=((unsigned short int)(THtoStorage[0]*10)&0xff00)>>8;
		//THtoStorage_10byte[1]=((unsigned short int)(THtoStorage[0]*10)&0x00ff);
		THtoStorage_10byte[0]=(int)(THtoStorage[0]*10)>>8;
		THtoStorage_10byte[1]=((int)(THtoStorage[0]*10)&0x00ff);
		THtoStorage_10byte[2]=((unsigned short int)(THtoStorage[1]*10)&0xff00)>>8;
		THtoStorage_10byte[3]=((unsigned short int)(THtoStorage[1]*10)&0x00ff);
		THtoStorage_10byte[4]=Year;
		THtoStorage_10byte[5]=Month;
		THtoStorage_10byte[6]=Date;
		THtoStorage_10byte[7]=Hour;
		THtoStorage_10byte[8]=Min;
		THtoStorage_10byte[9]=Sec;
	
		if(StorageMode==0)   //˳��洢������Ϊֹ
		{
			if(PartofEEPROM==0)
			{			
//						BUFFER_SIZE2=10;
//						PartofEEPROM=0;
//						sEEAddress=Part2_Write;
//						sEE_ReadBuffer(THtoStorage_Read,StorgeNum,(uint16_t *)(&BUFFER_SIZE2));
//						delay_ms(100);
				
				
						//sEEAddress=Part1_Write;
						EEPROM1024Part(0);//�洢��ǰ64K
						BUFFER_SIZE2=10;
						//sEE_WriteBuffer((u8*)THtoStorage_10byte,StorgeNum,BUFFER_SIZE2);
						AT24CXX_Write(StorgeNum,(u8 *)THtoStorage_10byte,BUFFER_SIZE2);
						delay_ms(100);
						//sEEAddress=Part1_Read;
						BUFFER_SIZE2=10;
						//sEE_ReadBuffer(THtoStorage_Read,StorgeNum,(uint16_t *)(&BUFFER_SIZE2));
						//delay_ms(100);
						//StorageParameters();
						StorgeNum=StorgeNum+10;
						if(StorgeNum>=65530)
						{
							PartofEEPROM=1;
							StorgeNum=50;
						}
						
						
			}
			else if(PartofEEPROM==1)//����128KbEEPROM�ĺ��ζ�д----7.2
			{
						
						PartofEEPROM=1;
				
//						sEEAddress=Part1_Write;
//						BUFFER_SIZE2=10;
//						sEE_ReadBuffer(THtoStorage_Read,StorgeNum,(uint16_t *)(&BUFFER_SIZE2));
//						delay_ms(100);
				
						//sEEAddress=Part2_Write;
						EEPROM1024Part(1);//�洢�ں�64K
						BUFFER_SIZE2=10;
						//sEE_WriteBuffer((u8*)THtoStorage_10byte,StorgeNum,BUFFER_SIZE2);
						AT24CXX_Write(StorgeNum,(u8 *)THtoStorage_10byte,BUFFER_SIZE2);
						delay_ms(100);
						//sEEAddress=Part2_Read;
						BUFFER_SIZE2=10;
						//sEE_ReadBuffer(THtoStorage_Read,StorgeNum,(uint16_t *)(&BUFFER_SIZE2));
						//delay_ms(100);
						StorageParameters();
						StorgeNum=StorgeNum+10;				
						if(StorgeNum>=65530)
						{
							PartofEEPROM=0;
							//StorgeNum=50;
							//EEPROMFull=1;
						}
				
			}
			if(LCD_yicunchu>=13102)//�����Ѵ���
			{
				EEPROMFull=1;
			}
		}
		else if(StorageMode==1)                //ѭ���洢������
		{
			if(PartofEEPROM==0)
			{			
//						BUFFER_SIZE2=10;
//						PartofEEPROM=0;
//						sEEAddress=Part2_Write;
//						sEE_ReadBuffer(THtoStorage_Read,StorgeNum,(uint16_t *)(&BUFFER_SIZE2));
//						delay_ms(100);
				
				
						//sEEAddress=Part1_Write;
						EEPROM1024Part(0);
						BUFFER_SIZE2=10;
						//sEE_WriteBuffer((u8*)THtoStorage_10byte,StorgeNum,BUFFER_SIZE2);
						AT24CXX_Write(StorgeNum,(u8 *)THtoStorage_10byte,BUFFER_SIZE2);
						delay_ms(100);
						//sEEAddress=Part1_Read;
						BUFFER_SIZE2=10;
						//sEE_ReadBuffer(THtoStorage_Read,StorgeNum,(uint16_t *)(&BUFFER_SIZE2));
						//delay_ms(100);
						//StorageParameters();
						StorgeNum=StorgeNum+10;
						if(StorgeNum>=65530)
						{
							PartofEEPROM=1;
							StorgeNum=50;
						}
						
						
			}
			else if(PartofEEPROM==1)//����128KbEEPROM�ĺ��ζ�д----7.2
			{
						
						PartofEEPROM=1;
				
//						sEEAddress=Part1_Write;
//						BUFFER_SIZE2=10;
//						sEE_ReadBuffer(THtoStorage_Read,StorgeNum,(uint16_t *)(&BUFFER_SIZE2));
//						delay_ms(100);
				
						//sEEAddress=Part2_Write;
						EEPROM1024Part(1);
						BUFFER_SIZE2=10;
						//sEE_WriteBuffer((u8*)THtoStorage_10byte,StorgeNum,BUFFER_SIZE2);
						AT24CXX_Write(StorgeNum,(u8 *)THtoStorage_10byte,BUFFER_SIZE2);
						delay_ms(100);
						//sEEAddress=Part2_Read;
						BUFFER_SIZE2=10;
						//sEE_ReadBuffer(THtoStorage_Read,StorgeNum,(uint16_t *)(&BUFFER_SIZE2));
						//delay_ms(100);
						StorageParameters();
						StorgeNum=StorgeNum+10;				
						if(StorgeNum>=65530)
						{
							PartofEEPROM=0;
							//StorgeNum=50;
							
						}
				
			}
			if(LCD_yicunchu>=13102)//�����Ѵ���
			{
				StorgeNum=50;
			}
	}
}
float average(u32 *adcArr , u16 num, u16 subtract)	//�������ȴ�С��������ȥͷȥβȡ�м���ƽ��ֵ
{ 
	u16 j_temp,k_temp;
	float add=0,adc_temp;
	for(j_temp=1; j_temp<num; j_temp++)
	{
		 for(k_temp=0; k_temp<num-j_temp; k_temp++)
		 {
		 	if(adcArr[k_temp] > adcArr[k_temp+1])
			{
				adc_temp = adcArr[k_temp];
				adcArr[k_temp] = adcArr[k_temp+1];
				adcArr[k_temp+1] = 	adc_temp;
			}
		 }
	}
	for(j_temp=subtract; j_temp<num-subtract; j_temp++)
	{
		add += adcArr[j_temp];	
	}
	return 	(float)(add/(num-subtract*2));
}
int main(void)
{ 
	u8 l=0;
	u8 BUFFER_SIZE2;
	
	gpInit();
	while(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_11));

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// �����ж����ȼ�����2
	delay_init();	    	//��ʱ������ʼ��	   
	uart1_init(9600);	 	//ch340ʹ�ô���1����ʼ��������9600 �ж����ȼ�33
	uart2_init(BaudRate);	 	//485ʹ�ô���2����ʼ��������9600��Ĭ�ϴ��ڽ���ģʽ��485_SEL�͵�ƽ //�ж����ȼ�32
	LED_Init();		  		//LED��ʼ��	
	KEY_Init();					//������ʼ��	
	i2cInit();	
	//sEE_Init();	 				//EEPROM��ʼ��
	AT24CXX_Init();
	Adc_Init();
	// Dac_init();  				//7512��ʼ��
	//Dac1_Init();
	EXTIX_Init();
	RelayInit();	 				//�̵�����ʼ��
	SHT2x_Init();				//SHT2x��ʼ��
	ht1621_init();				//ht1621��ʼ��
	lcd_init();					//LCD��ʼ��	
	//RTC_Init();					//RTC��ʼ�����ж����ȼ���00
	TIM3_Int_Init(9999,7199);//1s��ֻ���ƺ����˸�����Բ��� /�ж����ȼ�11
	TIM2_Int_Init(9999,7199);//0.5s��LCD�ײ�ѭ�����̵���˸�Ķ�ʱ�� //�ж����ȼ�01
	TIM4_init();         //RS485�ж϶�ʱ��
	TIM5_Int_Init(999,7199);
	BeepInit();          //��������ʼ��
	ReadParameters();  //��EEPROM�ж�ȡ�ػ�ǰ��l������Ϣ

	if(ParameterRead[1]==ParameterRead[2]||ParameterRead[1]>0x04)
	//if(ParameterRead[1]==ParameterRead[2]&&ParameterRead[1]==0xFF)
	{
	 StorageParameters(); //��Ҫ��ʼ��EEPROM�е���Ϣ�ʹ���һ��
	 ReadParameters();
	}
	else
	ReadParameters();

	//GPIO_SetBits(GPIOC,GPIO_Pin_10);
	//GPIO_ResetBits(GPIOC,GPIO_Pin_10);
	//GPIO_ResetBits(GPIOB,GPIO_Pin_3);
	//GPIO_ResetBits(GPIOB,GPIO_Pin_4);	
	//GPIO_SetBits(GPIOB,GPIO_Pin_3);
	//GPIO_SetBits(GPIOB,GPIO_Pin_4);

	//Lemon  ������sht20���������ͻ �ǵùر�SHT2x_Init�ĳ�ʼ��
	//shtxx
	//SHT10_Config();
	//Lemon 

	//		ADC_5V=Get_Adc_Average(ADC_Channel_13 ,30);
	//		V_5V=(float)ADC_5V*(2.48/4096);
	//		ADC_24V=Get_Adc_Average(ADC_Channel_12 ,30);
	//		V_24V=(float)ADC_24V*(2.48/4096);
	//		if(V_5V>2||V_24V>1)
	//			GPIO_ResetBits(GPIOC,GPIO_Pin_10);
	//		else 
	//			GPIO_SetBits(GPIOC,GPIO_Pin_10);

	//lcd_clr();		

	GetRSSI();
	DisplayRssi(RssiGrade);
	GPIO_ResetBits(GPIOA,GPIO_Pin_1);
	
	//StorgeNum=50;

	while(1)
	{
		u8 TxData[20];
		u8 TmpArr[4];
		u8 HumArr[4];
		//			EEPROM1024Part(0);
		//			for(l=0;l<10;l++)THtoStorage_10byte[l]=l;
		//			AT24CXX_Write(2000,(u8 *)THtoStorage_10byte,10);
		//			//delay_ms(100);
		//			AT24CXX_Read(2000,THtoStorage_Read,10);
		//			//delay_ms(100);
		//			for(l=0;l<10;l++)THtoStorage_10byte[l]=l+10;
		//			AT24CXX_Write(2010,(u8 *)THtoStorage_10byte,10);
		//			//delay_ms(100);
		//			AT24CXX_Read(2010,THtoStorage_Read,10);
		//			//delay_ms(100);
		//			EEPROM1024Part(1);
		//		  for(l=0;l<10;l++)THtoStorage_10byte[l]=l+20;
		//			AT24CXX_Write(2000,(u8 *)THtoStorage_10byte,10);
		//			//delay_ms(100);
		//			AT24CXX_Read(2000,THtoStorage_Read,10);
		//			//delay_ms(100);
		//			for(l=0;l<10;l++)THtoStorage_10byte[l]=l+30;
		//			AT24CXX_Write(2010,(u8 *)THtoStorage_10byte,10);
		//			delay_ms(100);
		//			AT24CXX_Read(2010,THtoStorage_Read,10);
		//			//delay_ms(100);
		//			for(l=0;l<10;l++)THtoStorage_10byte[l]=l+40;
		//			AT24CXX_Write(2020,(u8 *)THtoStorage_10byte,10);
		//			delay_ms(100);
		//			AT24CXX_Read(2020,THtoStorage_Read,10);
		//			//delay_ms(100);
		//			while(1);
		//			
		//			while(1);
		//		LCD_yicunchu=6560;
		//		sEEAddress=Part1_Write;
		//		for(l=0;l<10;l++)THtoStorage_10byte[l]=l;
		//		BUFFER_SIZE2=10;
		//		sEE_ReadBuffer(THtoStorage_Read,65500,(uint16_t *)(&BUFFER_SIZE2));
		//		sEEAddress=Part1_Write;
		//		for(l=0;l<10;l++)THtoStorage_10byte[l]=l+10;
		//		BUFFER_SIZE2=10;
		//		sEE_ReadBuffer(THtoStorage_Read,65510,(uint16_t *)(&BUFFER_SIZE2));
		//		sEEAddress=Part1_Write;
		//		for(l=0;l<10;l++)THtoStorage_10byte[l]=l+20;
		//		BUFFER_SIZE2=10;
		//		sEE_ReadBuffer(THtoStorage_Read,65520,(uint16_t *)(&BUFFER_SIZE2));
		//		sEEAddress=Part2_Write;
		//		for(l=0;l<10;l++)THtoStorage_10byte[l]=l+30;
		//		BUFFER_SIZE2=10;
		//		sEE_ReadBuffer(THtoStorage_Read,20,(uint16_t *)(&BUFFER_SIZE2));
		//		sEEAddress=Part2_Write;
		//		for(l=0;l<10;l++)THtoStorage_10byte[l]=l+40;
		//		BUFFER_SIZE2=10;
		//		sEE_ReadBuffer(THtoStorage_Read,30,(uint16_t *)(&BUFFER_SIZE2));
		//		sEEAddress=Part2_Write;
		//		for(l=0;l<10;l++)THtoStorage_10byte[l]=l+50;
		//		BUFFER_SIZE2=10;
		//		sEE_ReadBuffer(THtoStorage_Read,40,(uint16_t *)(&BUFFER_SIZE2));
		//		
		//		while(1);

		LCD_dizhi=address;
		LCD_xindao=485;
		if(PartofEEPROM==0)
			//LCD_yicunchu=(StorgeNum-20)/10;
				LCD_yicunchu=(StorgeNum-50)/10;
		else if(PartofEEPROM==1)
			//LCD_yicunchu=6551+(StorgeNum-20)/10;
				LCD_yicunchu=6551+(-50)/10;
		LCD_ID=address;
		LCD_botelv=BaudRate;
		LCD_moshi=StorageMode;
			


		//	KEY_Scan(0);
		KEY_Scan();		
		if(RTC_Control==0)
		{
			//�������ý���
			if(key==KEY1_PRES)//KEY1 ȷ�ϼ�����
			{
					DisplayTime_On=0;
					DeleteData=0;//��ֹ��ɾ��
					Menu_mode++;
					if(Menu_mode>25)
					{
						Menu_mode=0;
						NormalState=1;
						OnlyDispalyTime=0;
					}
					else
					{
						if(Menu_mode==1)
						{
							lcd_clr();
							//Clr_Wendu();
							//Clr_Shidu();
							
							OnlyDispalyTime=1;
						}
						NormalState=0;
						ModeSwitch(Menu_mode,3);//3��û������ģ�ֻ��Ϊ����ʾ //Dac_send_2byte_TEMP(0.4);
						Timing=0;
						Timing_Enable=1;//��ʼ��ʱ
					}
					key=0;
					Beep(50);         //��������һ��5ms

			}
			if(key==KEY2_PRES)//KEY2 ��С������,DAC���0.4V
			{
				ModeSwitch(Menu_mode,0);//��С 			//Dac_send_2byte_TEMP(0.4);
				key=0;
				Timing=0;
				Timing_Enable=1;//��ʼ��ʱ
				Beep(50);         //��������һ��5ms
			}
			if(key==KEY3_PRES)//KEY3 ���Ӽ�����,DAC���2.0V
			{
				ModeSwitch(Menu_mode,1);//����			//Dac_send_2byte_TEMP(2.0);
				key=0;
				Timing=0;
				Timing_Enable=1;//��ʼ��ʱ
				Beep(50);         //��������һ��5ms
			}
			if(key==KEY4_PRES)//KEY4 �˵�ѡ�������,DAC���1.65V
			{
				if(DeleteData==1)
				{
					DeleteData=0;
					PartofEEPROM=0;
					StorgeNum=50;			
				}
				DisplayTime_On=1;
				Menu_mode=0;//���ô��¿�ʼ 			//Dac_send_2byte_TEMP(1.65);
				NormalState=1;
				OnlyDispalyTime=0;
				key=0;
				
				sprintf(temp_lala,"AT+CIPSTART=\"TCP\",\"%d.%d.%d.%d\",%d\r\n",ip1,ip2,ip3,ip4,port);
				
				if(flag_ls_set_dangqian != flag_ls_set)
				{
					flag_ls_set_dangqian = flag_ls_set;
					flag_state = 0;
					flag_1s_nnn = 0;
					timetick = 0;
					flag_fasong = 0;
				}
				
				TIM_Cmd(TIM2, ENABLE);
				
				StorageParameters();
				Beep(100);         //��������һ��10ms
			}
		//�������ý���
		}
		else
		{
			//����ʱ�����ý��棬��ʱ����ǰʱ�丳ֵ��xxx_Set��������һ�Σ�
			if(RTCTimeParametersInit==1)
			{
				SetTimeParameters();
				RTCTimeParametersInit=0;
				Clr_Shidu();
				Display_Shidu_3(26,RTC_Selected);
				OnlyDisplayTime(RTC_Selected,year_Set,month_Set,date_Set,hour_Set,min_Set,sec_Set);
			}
			if(key==KEY1_PRES)
			{
				RTC_Selected++;
				if(RTC_Selected>5)
				{
						RTC_Selected=0;
				}
			//��ʾ�ڶ���
			Clr_Shidu();
			Display_Shidu_3(26,RTC_Selected);
			OnlyDisplayTime(RTC_Selected,year_Set,month_Set,date_Set,hour_Set,min_Set,sec_Set);
			if(OnlyDispalyTiming>1)
			{
				OnlyDisplayTime(RTC_Selected,year_Set,month_Set,date_Set,hour_Set,min_Set,sec_Set);
				OnlyDispalyTiming=0;
			}
			Timing=0;
			Timing_Enable=1;//��ʼ��ʱ
			key=0;//�����־λ
			}
			if(key==KEY3_PRES)//����
			{
					if(RTC_Selected==0)//year
					{
						 year_Set++;
					}
					if(RTC_Selected==1)//month
					{
						 month_Set++;
						if(month_Set>12)
						{
								month_Set=1;
						}
					}
					if(RTC_Selected==2)//date
					{
							date_Set++;
						if(month_Set==1||month_Set==3||month_Set==5||month_Set==7||month_Set==8||month_Set==10||month_Set==12)
						{
							if(date_Set>31)
							{
									date_Set=1;
							}
						}
						else if(month_Set==4||month_Set==6||month_Set==9||month_Set==11)
						{
							if(date_Set>30)
							{
									date_Set=1;
							}
						}
						else if(month_Set==2&&year_Set%4==0&&year_Set%100==0)
						{
							if(date_Set>29)
							{
									date_Set=1;
							}
						}
						else if(month_Set==2&&year_Set%4!=0&&year_Set%100!=0)	
						{
							if(date_Set>28)
							{
									date_Set=1;
							}
						}
					}
					if(RTC_Selected==3)//hour
					{
							hour_Set++;
						if(hour_Set>23)
						{
							hour_Set=0;
						}
					}
					if(RTC_Selected==4)//min
					{
							min_Set++;
						if(min_Set>59)
						{
								min_Set=0;
						}
					}
					if(RTC_Selected==5)//sec
					{
							sec_Set++;
						if(sec_Set>59)
						{
							sec_Set=0;
						}
					}
				RTC_Set(year_Set,month_Set,date_Set,hour_Set,min_Set,sec_Set);
				OnlyDisplayTime(RTC_Selected,year_Set,month_Set,date_Set,hour_Set,min_Set,sec_Set);
				if(OnlyDispalyTiming>1)
				{
					OnlyDisplayTime(RTC_Selected,year_Set,month_Set,date_Set,hour_Set,min_Set,sec_Set);
					OnlyDispalyTiming=0;
				}
				Timing=0;
				Timing_Enable=1;//��ʼ��ʱ
				key=0;//�����־λ
			}
					if(key==KEY2_PRES)//��С
			{
					if(RTC_Selected==0)//year
					{
						 year_Set--;
					}
					if(RTC_Selected==1)//month
					{
						 month_Set--;
						if(month_Set==0)
						{
								month_Set=12;
						}
					}
					if(RTC_Selected==2)//date
					{
							date_Set--;
						if(month_Set==1||month_Set==3||month_Set==5||month_Set==7||month_Set==8||month_Set==10||month_Set==12)
						{
							if(date_Set==0)
							{
									date_Set=31;
							}
						}
						else if(month_Set==4||month_Set==6||month_Set==9||month_Set==11)
						{
							if(date_Set==0)
							{
									date_Set=30;
							}
						}
						else if(month_Set==2&&year_Set%4==0&&year_Set%100==0)
						{
							if(date_Set==0)
							{
									date_Set=29;
							}
						}
						else if(month_Set==2&&year_Set%4!=0&&year_Set%100!=0)	
						{
							if(date_Set==0)
							{
									date_Set=28;
							}
						}
					}
					if(RTC_Selected==3)//hour
					{
							hour_Set--;
						if(hour_Set==0||hour_Set>23)
						{
							hour_Set=23;
						}
					}
					if(RTC_Selected==4)//min
					{
							min_Set--;
						if(min_Set==0||min_Set>59)
						{
								min_Set=59;
						}
					}
					if(RTC_Selected==5)//sec
					{
							sec_Set--;
						if(sec_Set==0||sec_Set>59)
						{
							sec_Set=59;
						}
					}
				//RTC_Set(year_Set,month_Set,date_Set,hour_Set,min_Set,sec_Set);
				OnlyDisplayTime(RTC_Selected,year_Set,month_Set,date_Set,hour_Set,min_Set,sec_Set);
				if(OnlyDispalyTiming>1)
				{
					OnlyDisplayTime(RTC_Selected,year_Set,month_Set,date_Set,hour_Set,min_Set,sec_Set);
					OnlyDispalyTiming=0;
				}
				Timing=0;
				Timing_Enable=1;//��ʼ��ʱ
				key=0;//�����־λ
			}
			if(key==KEY4_PRES)//ȷ�ϼ�
			{
				//��ʾ�ڶ���BAC
				RTC_Set(year_Set,month_Set,date_Set,hour_Set,min_Set,sec_Set);
				DisplayTime_On=1;
				Clr_Shidu();
				Display_Shidu_2(26,3);
				Display_Shidu_2(28,7);
				Display_Shidu_3(30,8);	
				RTC_Control=0;//������������ֱ���˳�
				Menu_mode=0;
				NormalState=1;
				OnlyDispalyTime=0;
				key=0;//�����־λ
				
				sprintf(temp_lala,"AT+CIPSTART=\"TCP\",\"%d.%d.%d.%d\",%d\r\n",ip1,ip2,ip3,ip4,port);
				
				if(flag_ls_set_dangqian != flag_ls_set)
				{
					flag_ls_set_dangqian = flag_ls_set;
					flag_state = 0;
					flag_1s_nnn = 0;
					timetick = 0;
					flag_fasong = 0;
				}
				
				TIM_Cmd(TIM2, ENABLE);
				
				//�洢����
				StorageParameters();
				write_addr_dat_n_char(7,0x08, 0);//����ʾ����
				Beep(100);         //��������һ��10ms
			}
		}	



		if(Timing>20)//20s��û�ж�����ص�������ʾ״̬
		{
					Menu_mode=0;//���ô��¿�ʼ 
					RTC_Control=0;			
					NormalState=1;
					OnlyDispalyTime=0;
					key=0;
					Timing=0;
					Timing_Enable=0;//�رն�ʱ
					DisplayTime_On=1;
			
					sprintf(temp_lala,"AT+CIPSTART=\"TCP\",\"%d.%d.%d.%d\",%d\r\n",ip1,ip2,ip3,ip4,port);
				
					if(flag_ls_set_dangqian != flag_ls_set)
					{
						flag_ls_set_dangqian = flag_ls_set;
						flag_state = 0;
						flag_1s_nnn = 0;
						timetick = 0;
						flag_fasong = 0;
					}
					
					TIM_Cmd(TIM2, ENABLE);
			
					//�洢����
					StorageParameters();
					//DisplayTime();
		}
		if(OnlyDispalyTime==1)
		{
			if(OnlyDispalyTiming>1)
			{
				//DisplayTime();   //��ʾ�ײ�����
				OnlyDispalyTiming=0;
			}
		}
		if(NormalState==1)
		{	
							if(DisplayTime_On==1)
							{
								DisplayTime();   //��ʾ�ײ�����
							}
							//������ʾ
							if(TimingReadSensor>=1)
							//if(TimingReadSensor>2)
							{

								Online--;
								if(EEPROMDownloading!=0)EEPROMDownloading--;
								/*
								if(SensorSelected==1)
								{
										SHT2x_Test(); //��ȡSHT2x����
										Temperature_Load=g_sht2x_param.TEMP_HM+(float)(TemperatureCalibration)/10;
										Humidity_Load=g_sht2x_param.HUMI_HM+(float)(HumtidyCalibration)/10;
									
		//									Temperature_Load=-28;
		//									Humidity_Load=95;
								}
						
								if(SensorSelected==3)
								{
										ReadDHT11();
										Humidity_Load=tdata[0]+tdata[1]/10.0;
										Temperature_Load=tdata[2]+tdata[3]/10.0;
								}
								
								if(SensorSelected==4)
								{
										ReadDHT21();
										Humidity_Load=(tdata[0]+tdata[1])/10.0;
										Temperature_Load=(tdata[2]+tdata[3])/10.0;
								}
								
								*/
								if(SensorSelected==1)
								{
										SHT2x_Test(); //��ȡSHT2x����
										Temperature_Load=g_sht2x_param.TEMP_HM+(float)(TemperatureCalibration)/10;
										Humidity_Load=g_sht2x_param.HUMI_HM+(float)(HumtidyCalibration)/10;
									
		//									Temperature_Load=-28;
		//									Humidity_Load=95;
										if((Humidity_Load)>100)
										{
											flag_chuanganqi=0;
											Temperature_Load=0;
											Humidity_Load=0;
											//�¶�������ʾErr
											//��ʾ��һ��SEL
											Clr_Wendu();
											Display_Wendu_2(0, 8);//��ʾС��
											//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
											Display_Wendu_2(2, 8);//��ʾ��λ
											Display_Wendu_2(4, 6);//��ʾʮλ
											//Display_CHARS(huashidu,1);//��ʾ��ʪ�ȵķ���
											
											
											
											//ʪ��������ʾ1
											//��ʾ�ڶ���
											Clr_Shidu();
											Display_Shidu_3(26,1);//��ʾ��	
										}
										else
										{
											flag_chuanganqi=1;	
										}
										
										
								}
								if(SensorSelected==2)
								{
										
									
		//									Temperature_Load=-28;
		//									Humidity_Load=95;
									 
											flag_chuanganqi=0;
											Temperature_Load=0;
											Humidity_Load=0;
											//�¶�������ʾErr
											//��ʾ��һ��SEL
											Clr_Wendu();
											Display_Wendu_2(0, 8);//��ʾС��
											//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
											Display_Wendu_2(2, 8);//��ʾ��λ
											Display_Wendu_2(4, 6);//��ʾʮλ
											//Display_CHARS(huashidu,1);//��ʾ��ʪ�ȵķ���
											
											
											
											//ʪ��������ʾ1
											//��ʾ�ڶ���
											Clr_Shidu();
											Display_Shidu_3(26,2);//��ʾ��	
									
										
										
										
								}
								if(SensorSelected==3)
								{
										ReadDHT11();
										Humidity_Load=tdata[0]+tdata[1]/10.0;
										Temperature_Load=tdata[2]+tdata[3]/10.0;
									
										if((Humidity_Load)==0)
										{
											flag_chuanganqi=0;
											Temperature_Load=0;
											Humidity_Load=0;
											//�¶�������ʾErr
											//��ʾ��һ��Err
											Clr_Wendu();
											Display_Wendu_2(0, 8);//��ʾС��
											//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
											Display_Wendu_2(2, 8);//��ʾ��λ
											Display_Wendu_2(4, 6);//��ʾʮλ
											//Display_CHARS(huashidu,1);//��ʾ��ʪ�ȵķ���
											//ʪ��������ʾ3
											Clr_Shidu();
											Display_Shidu_3(26,3);//��ʾ��	
										}
										else
										{	
											flag_chuanganqi=1;
										}
									
									
									
									
								}
								
								if(SensorSelected==4)
								{
										ReadDHT21();
										Humidity_Load=(tdata[0]+tdata[1])/10.0;
										Temperature_Load=(tdata[2]+tdata[3])/10.0;
										if((Humidity_Load)==0)
										{
											flag_chuanganqi=0;
											Temperature_Load=0;
											Humidity_Load=0;
											//�¶�������ʾErr
											//��ʾ��һ��Err
											Clr_Wendu();
											Display_Wendu_2(0, 8);//��ʾС��
											//write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
											Display_Wendu_2(2, 8);//��ʾ��λ
											Display_Wendu_2(4, 6);//��ʾʮλ
											//Display_CHARS(huashidu,1);//��ʾ��ʪ�ȵķ���
											//ʪ��������ʾ4
											Clr_Shidu();
											Display_Shidu_3(26,4);//��ʾ��	
										}
										else
										{
											flag_chuanganqi=1;
										}
								}
								
								
																											
								
								if(flag_chuanganqi==1)//�д���������������ʾ�������ڴ���������У���ʾ�����������
								{
									if(C_F)//����C_F�жϣ����϶Ⱥͻ��϶�ת��
									{
											Display_CHARS(huashidu,1);
											Display_CHARS(sheshidu,0);
											Temperature_Load_F=Temperature_Load*1.8+32;
											Display_Wendu(Temperature_Load_F*10); //��ʾ�¶�,*10������Ա���С��
									}
									else
									{
											Display_CHARS(sheshidu,1);
											Display_CHARS(huashidu,0);
											Display_Wendu(Temperature_Load*10); //��ʾ�¶�,*10������Ա���С��
									}	
																
		//								aa++;
		//							if(aa>10)aa=0;
								//Display_Wendu(Temperature_Load*10); //��ʾ�¶�,*10������Ա���С��
									write_addr_dat_n_char(7,0x08, 0);//�ر�����һ�еĸ���
									Display_Shidu(Humidity_Load*10); //��ʾʪ��
								}
								//Dac_Temp=(int)(13.333*Temperature_Load+933.333);
								//Dac_Humi=(int)(16*Humidity_Load+400);
		//int Adjust_Tem_4ma;
		//int Adjust_Tem_20ma;
		//int Adjust_Hum_4ma;
		//int Adjust_Hum_20ma;
		//int Adjust_Tem_5v;
		//int Adjust_Hum_10v;
		//							Dac_Temp=(Adjust_Tem_20ma-Adjust_Tem_4ma)/(float)(Temperature_20ma-Temperature_4ma)*Temperature_Load-(Adjust_Tem_20ma-Adjust_Tem_4ma)/(float)(Temperature_20ma-Temperature_4ma)*Temperature_4ma+Adjust_Tem_4ma;
		//							if(Temperature_Load<Temperature_4ma)
		//								Dac_Temp=Adjust_Tem_4ma;
		//							else if(Temperature_Load>Temperature_20ma)
		//								Dac_Temp=Adjust_Tem_20ma;			
		//							Dac_Humi=(Adjust_Hum_20ma-Adjust_Hum_4ma)/(float)(Humtidy_20ma-Humtidy_4ma)*Humidity_Load-(Adjust_Hum_20ma-Adjust_Hum_4ma)/(float)(Humtidy_20ma-Humtidy_4ma)*Humtidy_4ma+Adjust_Hum_4ma;
		//							if(Humidity_Load<Humtidy_4ma)
		//								Dac_Humi=Adjust_Hum_4ma;
		//							else if(Humidity_Load>Humtidy_20ma)
		//								Dac_Humi=Adjust_Hum_20ma;	
								if(flag_chuanganqi==1)//�д�����ʱ����������ѹ
								{
									Dac_Temp=(Adjust_Tem_5v-0)/(float)(Temperature_20ma-Temperature_4ma)*Temperature_Load-(Adjust_Tem_5v-0)/(float)(Temperature_20ma-Temperature_4ma)*Temperature_4ma;
									if(Temperature_Load<Temperature_4ma)
										Dac_Temp=0;
									else if(Temperature_Load>Temperature_20ma)
										Dac_Temp=Adjust_Tem_5v;			
									Dac_Humi=Adjust_Hum_10v/(float)(Humtidy_20ma-Humtidy_4ma)*Humidity_Load-Adjust_Hum_10v/(float)(Humtidy_20ma-Humtidy_4ma)*Humtidy_4ma;
									if(Humidity_Load<Humtidy_4ma)
										Dac_Humi=0;
									else if(Humidity_Load>Humtidy_20ma)
										Dac_Humi=Adjust_Hum_10v;	
								}
								else
								{
									//�޴����������е�ѹ���
									Dac_Temp=0;
									Dac_Humi=0;	
									
								}
								
		//							ADC_5V=Get_Adc_Average(ADC_Channel_13 ,30);
		//							V_5V=(float)ADC_5V*(2.48/4096);
		//							ADC_24V=Get_Adc_Average(ADC_Channel_12 ,30);
		//							V_24V=(float)ADC_24V*(2.48/4096);
								
								
								for(l=0;l<100;l++)
								{
									adc100[l]=Get_Adc(ADC_Channel_11);
								}
								ADC_BAT=average(adc100 , 100, 20);
								V_BAT=(float)ADC_BAT*(3.3/4096)*5.7;	
								
								
								
								//DisplayTime();   //��ʾ�ײ�����
								if(flag_chuanganqi==1)//�д��������бȽ�
								{
										WarningOrNot=Temperature_Warning_Data(Temperature_Load,Warning);
										WarningOrNot=Humitidy_Warning_Data(Humidity_Load,Warning,WarningOrNot);
								}
								//��ʾ����
		//							write_addr_dat_n_char(9,0x01, 1);//S0
		//							write_addr_dat_n_char(8,0x02, 1);//S1
		//							write_addr_dat_n_char(8,0x01, 1);//S2
		//							write_addr_dat_n_char(7,0x01, 1);//S3
								//��ʾ����
		//							Display_CHARS(hengxian,1);
		//							Display_CHARS(lixian,1);

								if(RecordMode==0&&EEPROMFull==0&&EEPROMDownloading==0)//�����¼ģʽ
								{
										if(WarningOrNot==0)//����������������¼����
										{
												if(StorgeTiming>=RecordTiming)
												{
													//Storage(g_sht2x_param.TEMP_HM,g_sht2x_param.HUMI_HM,StorageMode);
													//Storage(g_sht2x_param.TEMP_HM,g_sht2x_param.HUMI_HM,calendar.w_year%100,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec,StorageMode);
													Storage(Temperature_Load,Humidity_Load,calendar.w_year%100,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec,StorageMode);
//													StorageParameters();
//													StorageParameters_wuxian();
													StorgeTiming=0;
												}
										}
										else               //���ݱ�������������
										{
										 if(StorgeTiming>=WarningTiming)   
										 {
												//Storage(g_sht2x_param.TEMP_HM,g_sht2x_param.HUMI_HM,StorageMode);
												//Storage(g_sht2x_param.TEMP_HM,g_sht2x_param.HUMI_HM,calendar.w_year%100,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec,StorageMode);
												Storage(Temperature_Load,Humidity_Load,calendar.w_year%100,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec,StorageMode);
//												StorageParameters();
												StorgeTiming=0;
										 }
										}
								}
								//DisplayOthers();

								TimingReadSensor=0;
							}


		//						//Lemon
							//dhtxx
							//
							//
							//delay_ms(1000);
							//delay_ms(1000);
							//shtxx
							//					err += SHT10_Measure(&temp_val, &checksum, TEMP);                  //��ȡ�¶Ȳ���ֵ
							//					err += SHT10_Measure(&humi_val, &checksum, HUMI);                  //��ȡʪ�Ȳ���ֵ
							//					if(err != 0)
							//					SHT10_ConReset();
							//					else
							//					{
							//						SHT10_Calculate(temp_val, humi_val, &temp_val_real, &humi_val_real); //����ʵ�ʵ���ʪ��ֵ
							//						dew_point = SHT10_CalcuDewPoint(temp_val_real, humi_val_real);                 //����¶���¶�
							//					} 
							//					Delay_ms_shtxx(1000);
							//lemon
							//lcd_all();  //�����ã����ô�

		}

		if(flag_cunchu)//���ڽ��մ洢
		{
			flag_cunchu = 0;
			
			StorageParameters();
		}
		
		//�������ݴ���
		if(flag_fasong == 1)
		{
			if(flag_1s==1)
			{
				flag_1s=0;
				flag_state++;
				
				if(flag_state>5)
					flag_state=1;
				
				switch(flag_state)
				{
					case 1:
						GPIO_SetBits(GPIOA,GPIO_Pin_1);
						break;
					case 2:
						GetRSSI();
						DisplayRssi(RssiGrade);
						Connect();
	//					Wifi_Connect();
	//					Nb_Connect();
						break;
					case 3:
						//Wifi_Startsend();
						//Nb_SendData("12345678");
						TxData[0] = DeviceID/100 + 48;
						TxData[1] = DeviceID/100%10 + 48;
						TxData[2] = DeviceID%100 + 48;
						TxData[3] = NULL;
						
						Send_Str(TxData);
						break;
				case 4:
						NumToArr(TmpArr, (int)(Temperature_Load*10));
						NumToArr(HumArr, (int)(Humidity_Load*10));
						
						TxData[0] = 'S';
						TxData[1] = '1';
						TxData[2] = ':';
						TxData[3] = '1';
						TxData[4] = '*';
						TxData[5] = TmpArr[0];
						TxData[6] = TmpArr[1];
						TxData[7] = TmpArr[2];
						TxData[8] = TmpArr[3];
						TxData[9] = ',';
						TxData[10] = '1';
						TxData[11] = ':';
						TxData[12] = '2';
						TxData[13] = '*';
						TxData[14] = HumArr[0];
						TxData[15] = HumArr[1];
						TxData[16] = HumArr[2];
						TxData[17] = HumArr[3];
						TxData[18] = 'E';
						TxData[19] = NULL;
						
						CleanRXBUF();
						//Nb_SendData(TxData);
						Send_Str(TxData);
						break;
					case 5:
						DisConnect();		
						//Wifi_DisConnect();	
						//Nb_DisConnect();
						flag_fasong = 0;
						flag_state = 0;
						GPIO_ResetBits(GPIOA,GPIO_Pin_1);
						break;								 
					default:
						break;
				}
			}
		}
	}


///EEPROM����
//	while(1)
//	{
//		key=KEY_Scan(0);
//		if(key==KEY4_PRES)//KEY4 д��24M01
//		{
//		
//			sEE_WriteBuffer((u8*)TEXT_Buffer, 8000, SIZE);
//		}
//		if(key==KEY3_PRES)//KEY3 ��ȡ24M01
//		{
// 	

//			sEE_ReadBuffer(datatemp,8000,(uint16_t *)SIZE);
//		 
//	  }
//  }


///�̵�������

//	while(1)
//	{
//		key=KEY_Scan(0);
//		if(key==KEY2_PRES)//KEY2���£��̵�������
//		{
//		
//		
//			GPIO_ResetBits(GPIOB,GPIO_Pin_4);  //�̵���1

//			GPIO_ResetBits(GPIOB,GPIO_Pin_3);  //�̵���2
//		}
//		if(key==KEY3_PRES)//KEY3���£��̵�������
//		{
//			GPIO_SetBits(GPIOB,GPIO_Pin_4);  //�̵���1
//			GPIO_SetBits(GPIOB,GPIO_Pin_3);  //�̵���2
//			
//		}
 
//	}



//LCD����

//	
//	lcd_clr();
//	while(1)
//	{

//			SHT2x_Test(); //��ȡSHT2x����
//		
//			//Lemon
//			//dhtxx
//		  //ReadDHT11();
//			//ReadDHT21();
//			delay_ms(1000);
//			delay_ms(1000);
//			//shtxx
////					err += SHT10_Measure(&temp_val, &checksum, TEMP);                  //��ȡ�¶Ȳ���ֵ
////					err += SHT10_Measure(&humi_val, &checksum, HUMI);                  //��ȡʪ�Ȳ���ֵ
////					if(err != 0)
////					SHT10_ConReset();
////					else
////					{
////						SHT10_Calculate(temp_val, humi_val, &temp_val_real, &humi_val_real); //����ʵ�ʵ���ʪ��ֵ
////						dew_point = SHT10_CalcuDewPoint(temp_val_real, humi_val_real);                 //����¶���¶�
////					} 
////					Delay_ms_shtxx(1000);
//			//lemon
//		
//			//lcd_all();  //�����ã����ô�
//			Display_Wendu(); //��ʾ�¶�
//			Display_Shidu(); //��ʾʪ��
//			DisplayTime();   //��ʾ�ײ�����
//		
//		
//	}
}
