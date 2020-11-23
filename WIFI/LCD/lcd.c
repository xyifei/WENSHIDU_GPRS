#include "lcd.h"

#include "stm32f10x.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "delay.h"
//#include "sht2x.h"

int bLcd_flag = 0;
int bLcd_timer = 0;
int ii;
int j;



unsigned char Wendu[] = //温度0-9
	{   
	0X5F, 0X05, 0X3D, 0X79, 0X72, 0X6B, 0X6F, 0X51, 0X7F, 0X7B 
	};
	
unsigned char Shidu[] =
	{
	0XAF, 0X0F, 0XCB, 0XE9, 0XE4, 0X6D, 0X6F, 0XA7, 0XEF, 0XED
	};
	
unsigned char Others[] =
	{
	0XFA, 0X0A, 0XD6, 0X9E, 0X2E, 0XBC, 0XFC, 0X1A, 0XFE, 0XBE
	};
unsigned char CHARS[]=
	{
		0X02,//高报警1  地址9
		0x04,//正常1    地址8
		0x08,//低报警1  地址8
		0x04,//蜂鸣1    地址6
		0x08,//高报警2  地址9
		0x04,//正常2   地址9
		0x02,//低报警2  地址10
		0x04,//蜂鸣2    地址7
		
		0x01,//横线    地址10
		
		0x04,//地址    地址10
		0x01,//波特率  地址12
		0x01,//已存储   地址14
		0x01,//模式     地址18
		0x01,//信道    地址24
		0x02,//记录中    地址25
		0x04,//红线     地址25
		0x08,//白线     地址25
		0x01,//右两点   地址22
		0x01,//右一横   地址20
		0x01,//ID       地址25
		0x01,//左两点   地址16
		0x04,//左一横   地址16
	};
///////////////////////////////////////////////////驱动函数
/*
*	LCD 模式写入
*	入口:MODE :COM(命令模式) DAT(数据模式)
*	出口:void
*/
void write_mode(unsigned char MODE)	//写入模式,数据or命令
{
	GPIO_ResetBits(GPIOB, HT1621_WR);								//	RW = 0;
	delay_us(10);
	GPIO_SetBits(GPIOB, HT1621_DATA);									//	DA = 1;
	GPIO_SetBits(GPIOB, HT1621_WR);									//	RW = 1;
	delay_us(10);

	GPIO_ResetBits(GPIOB, HT1621_WR);								//	RW = 0;
	delay_us(10);
	GPIO_ResetBits(GPIOB, HT1621_DATA);								//	DA = 0;
	GPIO_SetBits(GPIOB, HT1621_WR);									//	RW = 1;
	delay_us(10);

	GPIO_ResetBits(GPIOB, HT1621_WR);								//	RW = 0;
	delay_us(10);

	if (0 == MODE)
	{
		GPIO_ResetBits(GPIOB, HT1621_DATA);							//	DA = 0;
	}
	else
	{
		GPIO_SetBits(GPIOB, HT1621_DATA);								//	DA = 1;
	}
	delay_us(10);
	GPIO_SetBits(GPIOB, HT1621_WR);									//	RW = 1;
	delay_us(10);
}

/*
*	LCD 命令写入函数
*	入口:cbyte ,控制命令字
*	出口:void
*/
void write_command(unsigned char Cbyte)
{
	unsigned char i = 0;

	for (i = 0; i < 8; i++)
	{
		GPIO_ResetBits(GPIOB, HT1621_WR);
		//delay_us(10);


		if ((Cbyte >> (7 - i)) & 0x01)
		{
			GPIO_SetBits(GPIOB, HT1621_DATA);
		}
		else
		{
			GPIO_ResetBits(GPIOB, HT1621_DATA);
		}
		delay_us(10);
		GPIO_SetBits(GPIOB, HT1621_WR);
		delay_us(10);
	}
	GPIO_ResetBits(GPIOB, HT1621_WR);
	delay_us(10);
	GPIO_ResetBits(GPIOB, HT1621_DATA);
	GPIO_SetBits(GPIOB, HT1621_WR);
	delay_us(10);
}

/*
*	LCD 地址写入函数
*	入口:cbyte,地址
*	出口:void
*/
void write_address(unsigned char Abyte)
{
	unsigned char i = 0;
	Abyte = Abyte << 2;

	for (i = 0; i < 6; i++)
	{
		GPIO_ResetBits(GPIOB, HT1621_WR);
		//delay_us(10);
		if ((Abyte >> (7 - i)) & 0x01)
		{
			GPIO_SetBits(GPIOB, HT1621_DATA);
		}
		else
		{
			GPIO_ResetBits(GPIOB, HT1621_DATA);
		}
		delay_us(10);
		GPIO_SetBits(GPIOB, HT1621_WR);
		delay_us(10);
	}
}

/*
*	LCD 数据写入函数
*	入口:Dbyte,数据
*	出口:void
*/
void write_data_8bit(unsigned char Dbyte)
{
	int i = 0;

	for (i = 0; i < 8; i++)
	{
		GPIO_ResetBits(GPIOB, HT1621_WR);
		delay_us(10);
		if ((Dbyte >> (7 - i)) & 0x01)
		{
			GPIO_SetBits(GPIOB, HT1621_DATA);
		}
		else
		{
			GPIO_ResetBits(GPIOB, HT1621_DATA);
		}
		delay_us(10);
		GPIO_SetBits(GPIOB, HT1621_WR);
		delay_us(10);
	}
}

void write_data_4bit(unsigned char Dbyte)
{
	int i = 0;

	for (i = 0; i < 4; i++)
	{
		GPIO_ResetBits(GPIOB, HT1621_WR);
		//delay_us(10);
		if ((Dbyte >> (3 - i)) & 0x01)
		{
			GPIO_SetBits(GPIOB, HT1621_DATA);
		}
		else
		{
			GPIO_ResetBits(GPIOB, HT1621_DATA);
		}
		delay_us(10);
		GPIO_SetBits(GPIOB, HT1621_WR);
		delay_us(10);
	}
}

//void write_data_1bit(unsigned char Dbyte)
//{
//	int i = 0;

//	for (i = 0; i < 4; i++)
//	{
//		GPIO_ResetBits(GPIOB, HT1621_WR);
//		//delay_us(10);
//		if ((Dbyte >> (3 - i)) & 0x01)
//		{
//			GPIO_SetBits(GPIOB, HT1621_DATA);
//		}
//		else
//		{
//		//	GPIO_ResetBits(GPIOB, HT1621_DATA);
//		}
//		delay_us(10);
//		GPIO_SetBits(GPIOB, HT1621_WR);
//		delay_us(10);
//	}
//}



///////////////////////////////////////////////////接口函数
void ht1621_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;// declare the structure 
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);	 //使能PA,PD端口时钟

	//////////////////////////////////////////////////////
	//配置PIN13-PA3,PIN15-PA5,PIN15-PA6,PIN15-PA7,
	memset(&GPIO_InitStructure, 0, sizeof(GPIO_InitTypeDef));
	GPIO_InitStructure.GPIO_Pin =  HT1621_WR | HT1621_DATA ;//| HT1621_IRQ
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
		memset(&GPIO_InitStructure, 0, sizeof(GPIO_InitTypeDef));
	GPIO_InitStructure.GPIO_Pin = HT1621_CS ;//| HT1621_IRQ
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//配置PIN19-PB1
	/*
	memset(&GPIO_InitStructure, 0, sizeof(GPIO_InitTypeDef));
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;//HT1621_RD ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);*/
}

/*
*	LCD 初始化，对lcd自身做初始化设置
*	入口:void
*	出口:void
*/
void lcd_init(void)
{
	//////////////////////////////////////////////////////
	GPIO_SetBits(GPIOA, HT1621_CS);
	GPIO_SetBits(GPIOB, HT1621_WR);
	GPIO_SetBits(GPIOB, HT1621_DATA);
	//delay_us(500);
for (ii=0;ii<10000;ii++)
	{for(j=10;j>0;j--);}
	//////////////////////////////////////////////////////        
	GPIO_ResetBits(GPIOA, HT1621_CS);        //CS = 0;
	//delay_us(10);
	for (ii=0;ii<10000;ii++)
	{for(j=10;j>0;j--);}
	write_mode(COMMAND);    //命令模式
	write_command(0x01);	//Enable System
	write_command(0x03);	//Enable Bias
	write_command(0x04);	//Disable Timer
	write_command(0x05);	//Disable WDT
	write_command(0x08);	//Tone OFF
	write_command(0x18);	//on-chip RC震荡
	write_command(0x29);	//1/4Duty 1/3Bias
	write_command(0x80);	//Disable IRQ
	write_command(0x40);	//Tone Frequency 4kHZ
	write_command(0xE3);	//Normal Mode

	GPIO_SetBits(GPIOA, HT1621_CS);  //CS = 1;
}

/*
*	LCD 清屏函数
*	入口:void
*	出口:void
*/
void lcd_clr(void)
{
	write_addr_dat_n(0x0, 0x00, 32);//15
}

/*
*	LCD 全显示函数
*	入口:void
*	出口:void
*/
void lcd_all(void)

{
   write_addr_dat_n(0, 0x06, 1);//15
	
	//write_addr_dat_n_wendu(0, 0x5f, 2);
	//Display_Wendu_1(2,5);
	//Display_Shidu_1(26,9 );
	//Display_Others_1(19,4);
	//Display_CHARS(botelv);
	
	
}

void write_addr_dat_n(unsigned char _addr, unsigned char _dat, unsigned char n)
{
	unsigned char i = 0;
	

	GPIO_ResetBits(GPIOA, HT1621_CS);								// CS = 0;
	write_mode(DAT);
	write_address(_addr);
	

	for (i = 0; i < n; i++)
	
	{
		//write_data_8bit(_dat);
		write_data_4bit(_dat);
		
	}	
}

//void write_addr_dat_n_1bit(unsigned char _addr, unsigned char _dat, unsigned char n)
//{
//	unsigned char i = 0;
//	

//	GPIO_ResetBits(GPIOA, HT1621_CS);								// CS = 0;
//	write_mode(DAT);
//	write_address(_addr);
//	

//	for (i = 0; i < n; i++)
//	
//	{
//		//write_data_8bit(_dat);
//		write_data_4bit(_dat);
//		
//	}	

//	
//	
//	GPIO_SetBits(GPIOA, HT1621_CS);									//CS = 1;
//}

//显示温度
//入口：pos，显示位置，0、2、4分别为从右到左的三个数字
//			num：要显示的一位数
void Display_Wendu_1(unsigned char pos, unsigned char num )
{
	unsigned char n,i,j;
	n=getChr_Wendu(num);
	i=(n&0xF0)>>4;
	j=n&0x0F;
	

	write_addr_dat_n(pos, i,1);

	write_addr_dat_n(pos+1,j,1);
	
		
	
}
//显示湿度
//入口：pos，显示位置，26、28、30分别为从右到左的三个数字
//			num：要显示的一位数
void Display_Shidu_1(unsigned char pos, unsigned char num )
{
	unsigned char n,i,j;
	n=getChr_Shidu(num);
	i=(n&0xF0)>>4;
	j=n&0x0F;
	

	write_addr_dat_n(pos, i,1);

	write_addr_dat_n(pos+1,j,1);
	
		
	
}
//显示底部数字，
//入口：pos，显示位置，11、13、15、17、19、21、23从左到右
//     num:要显示的一位数字
void Display_Others_1(unsigned char pos, unsigned char num )
{
	unsigned char n,i,j;
	n=getChr_Others(num);
	i=(n&0xF0)>>4;
	j=n&0x0F;
	

	write_addr_dat_n(pos, i,1);

	write_addr_dat_n(pos+1,j,1);
	
	
		
	
}

//void Display_CHARS(unsigned char c )
//{

//	unsigned char n;
//	
//	n=CHARS[c];
//	
//	switch(c)
//	{
//		case gaobaojing1:
//			write_addr_dat_n_1bit(9,n,1);break;
//		case zhengchang1:
//			write_addr_dat_n_1bit(8,n,1);break;
//		case dibaojing1:
//			write_addr_dat_n_1bit(8,n,1);break;
//		case fengming1:
//			write_addr_dat_n_1bit(6,n,1);break;
//		case gaobaojing2:
//			write_addr_dat_n_1bit(9,n,1);break;
//		case zhengchang2:
//			write_addr_dat_n_1bit(9,n,1);break;
//		case dibaojing2:
//			write_addr_dat_n_1bit(10,n,1);break;
//		case fengming2:
//			write_addr_dat_n_1bit(7,n,1);break;
//		case hengxian:
//			write_addr_dat_n_1bit(10,n,1);break;
//		case dizhi:
//			write_addr_dat_n_1bit(10,n,1);break;
//		case botelv:
//			write_addr_dat_n_1bit(12,n,1);break;
//		case yicunchu:
//			write_addr_dat_n_1bit(14,n,1);break;
//		case moshi:
//			write_addr_dat_n_1bit(18,n,1);break;
//		case xindao:
//			write_addr_dat_n_1bit(24,n,1);break;
//		case jiluzhong:
//			write_addr_dat_n_1bit(25,n,1);break;
//		case hongxian:
//			write_addr_dat_n_1bit(25,n,1);break;
//		case baixian:
//			write_addr_dat_n_1bit(25,n,1);break;
//		case youliangdian:
//			write_addr_dat_n_1bit(22,n,1);break;
//		case youyiheng:
//			write_addr_dat_n_1bit(20,n,1);break;
//		case ID:
//			write_addr_dat_n_1bit(25,n,1);break;
//		case zuoliangdian:
//			write_addr_dat_n_1bit(16,n,1);break;
//		case zuoyiheng:
//			write_addr_dat_n_1bit(16,n,1);break;
//	
//	}
//}




 unsigned char getChr_Wendu(unsigned char c)
 {
	 unsigned char i;
		for ( i = 0; i < 10; ++i)
		{
			if (c == i)
			{
				return Wendu[i];
			}
		}
		//return Wendu[i];
	}
 
	unsigned char getChr_Shidu(unsigned char c)
 {
	 unsigned char i;
		for ( i = 0; i < 10; ++i)
		{
			if (c == i)
			{
				return Shidu[i];
			}
		}
		//return Wendu[i];
	}
 
	unsigned char getChr_Others(unsigned char c)
 {
	 unsigned char i;
		for ( i = 0; i < 10; ++i)
		{
			if (c == i)
			{
				return Others[i];
			}
		}
		//return Wendu[i];
	}

//////////////////////////////////////////////////////////////////////////
////温度
//void temperature_all(void)
//{
//	write_addr_dat_n(0x0, 0xFF, 5);
//}

//void temperature_clr(void)
//{
//	write_addr_dat_n(0x0, 0x00, 5);
//}

//unsigned char FontLibrary[] = {
//	0xAF, 0xA0, 0xCB, 0xE9, 0xE4, 0x6D, 0x6F, 0xA8, 0xEF, 0xED,
//	0xF5, 0x05, 0xB6, 0x97, 0x47, 0xD3, 0xF3, 0x85, 0xF7, 0xD7,
//	0x5F, 0x50, 0x3D, 0x79, 0x72, 0x6B, 0x6F, 0x51, 0x7F, 0x7B
//};

//unsigned char getChr(unsigned char c, lcd_type _type)
//{
//	switch (_type)
//	{
//	case Temperature:
//		for (int i = 0; i < 10; ++i)
//		{
//			if ((c - '0') == i)
//			{
//				return FontLibrary[i];
//			}
//		}
//		break;
//	case Water:
//		for (int i = 0; i < 10; ++i)
//		{
//			if ((c - '0') == i)
//			{
//				return FontLibrary[i + 10];
//			}
//		}
//		break;
//	case Humidity:
//		for (int i = 0; i < 10; ++i)
//		{
//			if ((c - '0') == i)
//			{
//				return FontLibrary[i + 20];
//			}
//		}
//		break;
//	}

//	return 0x00;
//}

//void temperature_display(const char *_val)
//{
//	float dVal = (float)atof(_val);
//	int iVal = (int)(dVal * 100);
//	char buffer[6] = "\0";
//	char data[5] = { 0x00 };

//	sprintf(buffer, "%d", iVal);

//	unsigned int len = strlen(buffer);
//	for (unsigned int i = 0; i < len; i++)
//	{
//		data[i] = buffer[strlen(buffer) - i - 1];
//	}

//	GPIO_ResetBits(GPIOA, HT1621_CS);								// CS = 0;
//	write_mode(DAT);
//	write_address(0x0);

//	for (unsigned int i = 0; i < 5; i++)
//	{
//		char t = 0x00;

//		if (i < len)
//		{
//			t = getChr(data[i], Temperature);
//		}
//		else
//		{
//			t = 0x00;
//		}

//		if (len == 1)
//		{
//			if (i == 1
//				| i == 2)
//			{
//				t = 0xAF;
//			}
//		}
//		else if (len == 2)
//		{
//			if (i == 2)
//			{
//				t = 0xAF;
//			}
//		}

//		if (i == 0
//			|| i == 2
//			|| i == 4)
//		{
//			t += 0x10;
//		}
//		write_data_8bit(t);
//	}
//	GPIO_SetBits(GPIOA, HT1621_CS);									//CS = 1;
//}

////////////////////////////////////////////////////////////////////////////
////水分
//void water_all(void)
//{
//	write_addr_dat_n(0xA, 0xFF, 5);
//}

//void water_clr(void)
//{
//	write_addr_dat_n(0xA, 0x00, 5);
//}

//void water_display(const char *_val)
//{
//	float dVal = (float)atof(_val);
//	int iVal = (int)(dVal * 100);
//	char buffer[6] = "\0";
//	char data[5] = { 0x00 };

//	sprintf(buffer, "%d", iVal);

//	unsigned int len = strlen(buffer);
//	for (unsigned int i = 0; i < len; i++)
//	{
//		data[5 - len + i] = buffer[i];
//	}

//	GPIO_ResetBits(GPIOA, HT1621_CS);								// CS = 0;
//	write_mode(DAT);
//	write_address(0xA);

//	for (unsigned int i = 0; i < 5; i++)
//	{
//		char w = 0x00;

//		if (i < (5 - len))
//		{
//			w = 0x00;
//		}
//		else
//		{
//			w = getChr(data[i], Water);
//		}

//		if (len == 1)
//		{
//			if (i == 2
//				| i == 3)
//			{
//				w = 0xF5;
//			}
//		}
//		else if (len == 2)
//		{
//			if (i == 2)
//			{
//				w = 0xF5;
//			}
//		}

//		if (i == 0
//			|| i == 2
//			|| i == 4)
//		{
//			w += 0x08;
//		}
//		write_data_8bit(w);
//	}
//	GPIO_SetBits(GPIOA, HT1621_CS);									//CS = 1;
//}

//////////////////////////////////////////////////////////////////////////////
////湿度
//void humidity_all(void)
//{
//	write_addr_dat_n(0x14, 0xFF, 5);
//}

//void humidity_clr(void)
//{
//	write_addr_dat_n(0x14, 0x00, 5);
//}

//void humidity_display(const char *_val)
//{
//	float dVal = (float)atof(_val);
//	int iVal = (int)(dVal * 100);
//	char buffer[6] = "\0";
//	char data[5] = { 0x00 };

//	sprintf(buffer, "%d", iVal);

//	unsigned int len = strlen(buffer);
//	for (unsigned int i = 0; i < len; i++)
//	{
//		data[i] = buffer[strlen(buffer) - i - 1];
//	}

//	GPIO_ResetBits(GPIOA, HT1621_CS);								// CS = 0;
//	write_mode(DAT);
//	write_address(0x14);

//	for (unsigned int i = 0; i < 3; i++)
//	{
//		char h = 0x00;

//		if (i < len)
//		{
//			h = getChr(data[i], Humidity);
//		}
//		else
//		{
//			h = 0x00;
//		}

//		if (len == 1)
//		{
//			if (i == 1
//				| i == 2)
//			{
//				h = 0x5F;
//			}
//		}
//		else if (len == 2)
//		{
//			if (i == 2)
//			{
//				h = 0x5F;
//			}
//		}

//		if (i == 0
//			|| i == 2)
//		{
//			h += 0x80;
//		}
//		write_data_8bit(h);
//	}
//	//	GPIO_SetBits(GPIOA, HT1621_CS);									//CS = 1;

//	char h1 = 0x00;
//	char h2 = 0x00;

//	h1 = getChr(data[3], Humidity);
//	h2 = getChr(data[4], Humidity);

//	len < 3 ? write_data_4bit(0x0) : write_data_4bit((h1 & 0xF0) >> 4);
//	len < 4 ? write_data_4bit(0x8) : write_data_4bit(((h2 & 0xF0) >> 4) + 0x8);
//	len < 3 ? write_data_4bit(0x0) : write_data_4bit(h1 & 0xF);
//	len < 4 ? write_data_4bit(0x0) : write_data_4bit(h2 & 0xF);
//	GPIO_SetBits(GPIOA, HT1621_CS);

//}

//************************************
// 函数名称: 打印 "OFF"
// 返回类型: 
// 参数信息: 
// 函数说明: 
//************************************
void display_off()
{
	GPIO_ResetBits(GPIOA, HT1621_CS);								// CS = 0;
	write_mode(DAT);
	write_address(0x04);
	write_data_8bit(0x4E);
	write_data_8bit(0x4E);
	write_data_8bit(0xAF);
	GPIO_SetBits(GPIOA, HT1621_CS);

	GPIO_ResetBits(GPIOA, HT1621_CS);								// CS = 0;
	write_mode(DAT);
	write_address(0x0A);
	write_data_8bit(0xF5);
 	write_data_8bit(0xE2);
 	write_data_8bit(0xE2);
	GPIO_SetBits(GPIOA, HT1621_CS);

	GPIO_ResetBits(GPIOA, HT1621_CS);								// CS = 0;
	write_mode(DAT);
	write_address(0x18);
	write_data_8bit(0x27);
	write_data_4bit(0x2);
	write_data_4bit(0x5);
 	write_data_4bit(0x7);
 	write_data_4bit(0xF);
	GPIO_SetBits(GPIOA, HT1621_CS);
}

//void Display_Wendu(float val)
//{
//	
//	
//	int Temp;
//	unsigned char Temp_shi,Temp_ge,Temp_dot;
//	if(g_sht2x_param.TEMP_HM>0)
//	{
//	Temp=(int)(g_sht2x_param.TEMP_HM*10);
//	
//	Temp_shi=Temp/100;
//	Temp_ge=Temp%100/10;
//	Temp_dot=Temp%100%10;
//	
//	Display_Wendu_1(0, Temp_dot=Temp%100%10);//显示小数
//	//write_addr_dat_n(_addr, _dat, 1)
//	Display_Wendu_1(2, Temp_dot=Temp%100%10);//显示个位
//	Display_Wendu_1(4, Temp_dot=Temp%100%10);//显示十位	
//		
//	}
//	else if(g_sht2x_param.TEMP_HM<0)
//	{
//		//显示负数-
//		
//	}
//	
//}