#ifndef __LCD__
#define __LCD__

//HT1621
/*#define HT1621_CS		GPIO_Pin_4
#define HT1621_WR		GPIO_Pin_6
#define HT1621_DATA		GPIO_Pin_7
#define HT1621_IRQ		GPIO_Pin_8
#define HT1621_RD		GPIO_Pin_5*/

#define HT1621_CS		GPIO_Pin_0
#define HT1621_WR		GPIO_Pin_9
#define HT1621_DATA		GPIO_Pin_8
//#define HT1621_IRQ		GPIO_Pin_8
//#define HT1621_RD		GPIO_Pin_5

#define LCD_TIME		1500	//15s����Ϩ��

#define DAT 1	//����
#define COMMAND 0	//����


//�ַ�����
#define gaobaojing1 0x01
#define zhengchang1 0x02
#define dibaojing1  0x03
#define fengming1   0x04
#define gaobaojing2 0x05
#define zhengchang2 0x06
#define dibaojing2  0x07
#define fengming2   0x08
#define hengxian    0x09
#define dizhi       0x0a
#define botelv      0x0b
#define yicunchu    0x0c
#define moshi       0x0d 
#define xindao      0x0e
#define jiluzhong   0x0f
#define zaixian    0x10
#define lixian     0x11
#define youliangdian 0x12
#define youyiheng    0x13
#define ID          0x14
#define zuoliangdian 0x15
#define zuoyiheng    0x16
#define sheshidu     0x17
#define huashidu     0x18

/*
*       �¶�      ˮ��      ʪ��
*       0xAF      0xF5      0x5F    |     0
*       0xA0      0x05      0x50    |     1
*       0xCB      0xB6      0x3D    |     2
*       0xE9      0x97      0x79    |     3
*       0xE4      0x47      0x72    |     4
*       0x6D      0xD3      0x6B    |     5
*       0x6F      0xF3      0x6F    |     6
*       0xA8      0x85      0x51    |     7
*       0xEF      0xF7      0x7F    |     8
*       0xED      0xD7      0x7B    |     9
*
*       +0x10    +0x08      +0x80
*/


typedef enum 
{
	Temperature,	//�¶�
	Water,		//ˮ��
	Humidity,	//ʪ��
}lcd_type;

extern int bLcd_flag;
extern int bLcd_timer;

///////////////////////////////////////////////////��������
void write_mode(unsigned char MODE);
void write_command(unsigned char Cbyte);
void write_address(unsigned char Abyte);
void write_data_8bit(unsigned char Dbyte);
void write_data_4bit(unsigned char Dbyte);

//�ڲ�ʹ��
static void write_addr_dat_n(unsigned char _addr, unsigned char _dat, unsigned char n);
void write_addr_dat_n_1bit(unsigned char _addr, unsigned char _dat, unsigned char n);
void write_addr_dat_n_wendu(unsigned char _addr, unsigned char _dat, unsigned char n);
	void write_addr_dat_n_shidu(unsigned char _addr, unsigned char _dat, unsigned char n);
void write_addr_dat_n_others(unsigned char _addr, unsigned char _dat, unsigned char n);
void write_addr_dat_n_char(unsigned char _addr, unsigned char _dat, unsigned char n);

///////////////////////////////////////////////////
//�ӿں���
void ht1621_init(void); //�����ȳ�ʼ��������������ht1621��GPIO
void lcd_init(void);    //��ʼ��lcd
void lcd_clr(void);     //����
void lcd_all(void);     //ȫ��
void display_off(void);		//��ʾOFF

//�¶�
void temperature_all(void);    //�¶�ȫ��
void temperature_clr(void);    //�¶�����
void temperature_display(const char *_val);    //��ʾ�¶�

//ˮ��
void water_all(void);    //ˮ��ȫ��
void water_clr(void);    //ˮ������
void water_display(const char *_val);    //��ʾˮ��

//ʪ��
void humidity_all(void);    //ʪ��ȫ��
void humidity_clr(void);    //ʪ������
void humidity_display(const char *_val);    //��ʾʪ��

void Display_Wendu_1(unsigned char add, unsigned char num );
void Display_Shidu_1(unsigned char add, unsigned char num );
void Display_Others_1(unsigned char add, unsigned char num );

unsigned char getChr_Wendu(unsigned char c);
unsigned char getChr_Others(unsigned char c);
unsigned char getChr_Shidu(unsigned char c);

void Display_CHARS(unsigned char c ,unsigned char state);
void Display_Wendu(int Temp);
void Display_Shidu(int Humi);
void Clr_Wendu(void);
void Clr_Wendu1(void);
void Clr_Wendu2(void);
void Clr_Shidu(void);
void Clr_Others(void);
void OnlyDisplayTime(int RTC_Selected,unsigned short  year,unsigned char month,unsigned char date,unsigned char hour,unsigned char min,unsigned char sec);
void DisplayTime(void);
void SetTimeParameters(void);
void DisplayOthers(void);
void Display_Wendu_2(unsigned char add, unsigned char num );
void Display_Shidu_2(unsigned char add, unsigned char num );
void Display_Shidu_3(unsigned char add, unsigned char num );
void WriteLcdram(unsigned char add, unsigned char data);

void DisplayRssi(int rssi);

#endif //__LCD__
