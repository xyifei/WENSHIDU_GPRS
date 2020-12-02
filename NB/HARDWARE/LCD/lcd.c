#include "lcd.h"

#include "stm32f10x.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "delay.h"
#include "sht2x.h"
#include "rtc.h"
#include "timer.h"
#include "adc.h"

int bLcd_flag = 0;
int bLcd_timer = 0;
int ii;
int j;
u8 i;

extern u8 DeviceID;
extern u8 RssiGrade;
extern int RemoteControl;

extern u16 count;  //�ײ�ѭ���ļ�������������timer.c��

extern u16 LCD_dizhi;//�ײ�Ҫ��ʾ�ĵ�ַ��ֵ
extern u16 LCD_xindao;//�ײ�Ҫ��ʾ���ŵ���ֵ
extern u16 LCD_botelv;//�ײ�Ҫ��ʾ�Ĳ�������ֵ
extern u16 LCD_yicunchu;//�ײ�Ҫ��ʾ�Ĵ洢��ֵ
extern u16 LCD_ID;//�ײ�Ҫ��ʾ��ID��ֵ
extern u16 LCD_moshi;//�ײ�Ҫ��ʾ��ģʽ��ֵ
extern float V_5V,V_BAT,V_24V;
extern u16 ADC_5V,ADC_BAT,ADC_24V;
extern int C_F;
extern u8 Online;
extern u8 HaveMes;
//u8 State_485;

unsigned char Lcdram[32]=
	{
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00

	
	};

const unsigned char Wendu[] = //�¶�0-9
	{   
	0X5F, 0X50, 0X3D, 0X79, 0X72, 0X6B, 0X6F, 0X51, 0X7F, 0X7B 
	};
const unsigned char Wendu1[]={   //�¶�0-9����һֱ��ʾ���϶ȷ���
	0X5F, 0X50, 0X3D, 0X79, 0X72, 0X6B, 0X6F, 0X51, 0X7F, 0X7B 
	};
	
//const unsigned char Shidu[] =
//	{
//	0XAF, 0XA0, 0XCB, 0XE9, 0XE4, 0X6D, 0X6F, 0XA8, 0XEF, 0XED
//	};
	
const unsigned char Shidu[] =//ʪ��0-9��һֱ��ʾС���㡢�ٷֺš�RH
	{
	0XBF, 0XB0, 0XDB, 0XF9, 0XF4, 0X7D, 0X7F, 0XB8, 0XFF, 0XFD
	};
const unsigned char Shidu2[] =//ʪ��0-9������ʾС���㡢�ٷֺš�RH
	{
	0XAF, 0XA0, 0XCB, 0XE9, 0XE4, 0X6D, 0X6F, 0XA8, 0XEF, 0XED
	};
const unsigned char Others[] =//�ײ�����0-9
	{
	0XFA, 0X0A, 0XD6, 0X9E, 0X2E, 0XBC, 0XFC, 0X1A, 0XFE, 0XBE
	};
//��ӵ����ý����һЩ�����ַ���d,b,p,c,L,F,E,A��t,u,n,H
const unsigned char WenduSetting[] =
	{
	0X7C, 0X6E, 0X37, 0X0F, 0X0E, 0X27, 0X2F, 0X77, 0X24, 0X4C,0X64,0X76
	};
//��ӵ����ý����һЩ�����ַ���d,b,p,c,L,F,E,A��t,u,n,H������ʾС���㡢�ٷֺš�RH
const unsigned char ShiduSetting[] =
	{
	0XE3, 0X67, 0XCE, 0X0F, 0X07, 0X4E, 0X4F, 0XEE, 0X42, 0X23,0X62,0XE6
	};
const unsigned char CHARS[]=//�������е��ַ�
	{
		0x00,//����
		0X02,//�߱���1  ��ַ9
		0x04,//����1    ��ַ8
		0x08,//�ͱ���1  ��ַ8
		0x04,//����1    ��ַ6
		0x08,//�߱���2  ��ַ9
		0x04,//����2   ��ַ9
		0x02,//�ͱ���2  ��ַ10
		0x04,//����2    ��ַ7
		
		0x01,//����    ��ַ10
		
		0x04,//��ַ    ��ַ10
		0x01,//������  ��ַ12
		0x01,//�Ѵ洢   ��ַ14
		0x01,//ģʽ     ��ַ18
		0x01,//�ŵ�    ��ַ24
		0x02,//��¼��    ��ַ25
		0x04,//����     ��ַ25
		0x08,//����     ��ַ25
		0x01,//������   ��ַ22
		0x01,//��һ��   ��ַ20
		0x01,//ID       ��ַ25
		0x01,//������   ��ַ16
		0x04,//��һ��   ��ַ16
		
		0x08,//���϶�    ��ַ0
		0x08,//���϶�    ��ַ2
		0x04
	};


///////////////////////////////////////////////////��������
/*
*	LCD ģʽд��
*	���:MODE :COM(����ģʽ) DAT(����ģʽ)
*	����:void
*/
void write_mode(unsigned char MODE)	//д��ģʽ,����or����
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
*	LCD ����д�뺯��
*	���:cbyte ,����������
*	����:void
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
*	LCD ��ַд�뺯��
*	���:cbyte,��ַ
*	����:void
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
*	LCD ����д�뺯��
*	���:Dbyte,����
*	����:void
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



//1621��ʼ��
void ht1621_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;// declare the structure 
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);

	//////////////////////////////////////////////////////
	
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

	
//	memset(&GPIO_InitStructure, 0, sizeof(GPIO_InitTypeDef));
//	GPIO_InitStructure.GPIO_Pin = HT1621_RD | GPIO_Pin_7;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*
*	LCD ��ʼ������lcd��������ʼ������
*	���:void
*	����:void
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
	write_mode(COMMAND);    //����ģʽ
	write_command(0x01);	//Enable System
	write_command(0x03);	//Enable Bias
	write_command(0x04);	//Disable Timer
	write_command(0x05);	//Disable WDT
	write_command(0x08);	//Tone OFF
	write_command(0x18);	//on-chip RC��
	write_command(0x29);	//1/4Duty 1/3Bias
	write_command(0x80);	//Disable IRQ
	write_command(0x40);	//Tone Frequency 4kHZ
	write_command(0xE3);	//Normal Mode

	GPIO_SetBits(GPIOA, HT1621_CS);  //CS = 1;
}

/*
*	LCD ��������
*	���:void
*	����:void
*/
void lcd_clr(void)
{
	write_addr_dat_n(0x0, 0x00, 32);//15
}

/*
*	LCD ȫ��ʾ����
*	���:void
*	����:void
*/
void lcd_all(void)
{
//	write_addr_dat_n(0x0, 0xFF, 32);//15

//					Clr_Wendu();
//					Clr_Others();
//	
//					Clr_Shidu();
//	
//	write_addr_dat_n(0x11, 0x06, 1);
//	
	Display_Others_1(11, 1 );
	Display_Others_1(13, 2 );
	Display_Others_1(15, 3 );
	Display_Others_1(17, 4 );
	Display_Others_1(19, 5 );
	Display_Others_1(21, 6 );
	Display_Others_1(23, 7 );


//	//write_addr_dat_n(0x0,0x06,1);
//	
//	Display_Wendu_1(0,5);
//	Display_CHARS(sheshidu,1);
//	Display_CHARS(sheshidu,0);
//	Display_CHARS(huashidu,1);
//	Display_CHARS(huashidu,0);
//	Display_CHARS(gaobaojing1,1);	
//	Display_Shidu_1(26,9 );
//	Display_Others_1(19,4);
//	Display_CHARS(gaobaojing1,1);
//	Display_CHARS(gaobaojing1,0);
//	Display_CHARS(zhengchang1,1);
//	Display_CHARS(zhengchang1,0);
//	Display_CHARS(dibaojing1,1);
//	Display_CHARS(dibaojing1,0);
//	Display_CHARS(fengming1,1);
//	Display_CHARS(fengming1,0);
//	Display_CHARS(gaobaojing2,1);
//	Display_CHARS(gaobaojing2,0);
//	Display_CHARS(zhengchang2,1);
//	Display_CHARS(zhengchang2,0);
//	Display_CHARS(dibaojing2,1);
//	Display_CHARS(dibaojing2,0);
//	Display_CHARS(fengming2,1);
//	Display_CHARS(moshi,1);
//	Display_CHARS(botelv,1);
//	Display_CHARS(yicunchu,1);
//	Display_CHARS(dizhi,1);
//	Display_CHARS(xindao,1);
//	Display_CHARS(ID,1);
//	Display_CHARS(hengxian,1);
//	Display_CHARS(jiluzhong,1);
//	Display_CHARS(zaixian,1);
//	Display_CHARS(lixian,1);
//	Display_CHARS(youliangdian,1);
//	Display_CHARS(fengming2,0);
//	Display_CHARS(moshi,0);
//	Display_CHARS(botelv,0);
//	Display_CHARS(yicunchu,0);
//	Display_CHARS(dizhi,0);
//	Display_CHARS(xindao,0);
//	Display_CHARS(ID,0);
//	Display_CHARS(hengxian,0);
//	Display_CHARS(jiluzhong,0);
//	Display_CHARS(zaixian,0);
//	Display_CHARS(lixian,0);
//	Display_CHARS(youliangdian,0);
//	
//	Display_CHARS(youyiheng,1);
//	Display_CHARS(zuoliangdian,1);
//	Display_Others_1(15,1);
//	Display_CHARS(zuoyiheng,1);
//	Display_Others_1(15,6);
 }

 
 //ע�������ַ��ʮ���Ƶ�0-31 n��1-32  //����дȫ��������
void write_addr_dat_n(unsigned char _addr, unsigned char _dat, unsigned char n)  
{
	
	unsigned char i = 0;
	//unsigned char _dat_temp1,_dat_temp2;
	
	//WriteLcdram(_addr, _dat);
		
	

	GPIO_ResetBits(GPIOA, HT1621_CS);								// CS = 0;
	write_mode(DAT);
	

	write_address(_addr);
	

	for (i = 0; i < n; i++)
	{

		write_data_4bit(_dat);
		WriteLcdram(_addr+i, _dat);
	}
	GPIO_SetBits(GPIOA, HT1621_CS);									//CS = 1;
	
	//WriteLcdram(_addr, _dat);
	

	
 
}

//
void write_addr_dat_n_1bit(unsigned char _addr, unsigned char _dat, unsigned char n) 
{
	
	unsigned char i = 0;
	unsigned char _dat_temp1,_dat_temp2;
	
	//WriteLcdram(_addr, _dat);
		
	

	GPIO_ResetBits(GPIOA, HT1621_CS);								// CS = 0;
	write_mode(DAT);
	
	
		//_dat_temp1=(Lcdram[_addr][0]<<3)+(Lcdram[_addr][1]<<2)+(Lcdram[_addr][2]<<1)+(Lcdram[_addr][3]<<0);
	_dat_temp1=_dat_temp1|_dat;
	WriteLcdram(_addr, _dat_temp1);
	//write_address(_addr);
	
	//_dat_temp2=(Lcdram[_addr][0]<<3)+(Lcdram[_addr][1]<<2)+(Lcdram[_addr][2]<<1)+(Lcdram[_addr][3]<<0);
	
	write_address(_addr);
	

	for (i = 0; i < n; i++)
	{
		//write_data_8bit(_dat);
		//write_data_1bit(_dat);
		write_data_4bit(_dat_temp2);
	}
	GPIO_SetBits(GPIOA, HT1621_CS);									//CS = 1;
	
	//WriteLcdram(_addr, _dat_temp2);
	
 
}

//�����¶�����д����
void write_addr_dat_n_wendu(unsigned char _addr, unsigned char _dat, unsigned char n)
{
	
	unsigned char i = 0;
	unsigned char _dat_temp1,_dat_temp2;
	
	//WriteLcdram(_addr, _dat);
		
	

	GPIO_ResetBits(GPIOA, HT1621_CS);								// CS = 0;
	write_mode(DAT);
	
	if(Lcdram[_addr]==0x00)
	{
		WriteLcdram(_addr, _dat);
	}
		if((_addr%2)==0)
		{
			_dat_temp1=Lcdram[_addr];
			_dat_temp2=(_dat_temp1&0x08)|_dat;
			
			write_address(_addr);
			for (i = 0; i < n; i++)
		{
			write_data_4bit(_dat_temp2);
		}
		GPIO_SetBits(GPIOA, HT1621_CS);									//CS = 1;
			
		}
		else if((_addr%2)!=0)
		{
			write_address(_addr);
			for (i = 0; i < n; i++)
		{
			write_data_4bit(_dat);
		}
		GPIO_SetBits(GPIOA, HT1621_CS);
		}
		
		WriteLcdram(_addr, _dat_temp2);
}

//����ʪ������д����
void write_addr_dat_n_shidu(unsigned char _addr, unsigned char _dat, unsigned char n)
{
	
	unsigned char i = 0;
	unsigned char _dat_temp1,_dat_temp2;
	
	//WriteLcdram(_addr, _dat);
		
	

	GPIO_ResetBits(GPIOA, HT1621_CS);								// CS = 0;
	write_mode(DAT);
	
	if(Lcdram[_addr]==0x00)
	{
		WriteLcdram(_addr, _dat);
	}
		if((_addr%2)==0)
		{
			_dat_temp1=Lcdram[_addr];
			_dat_temp2=(_dat_temp1&0x01)|_dat;
			
			write_address(_addr);
			for (i = 0; i < n; i++)
		{
			write_data_4bit(_dat_temp2);
		}
		GPIO_SetBits(GPIOA, HT1621_CS);									//CS = 1;
			
		}
		else if((_addr%2)!=0)
		{
			write_address(_addr);
			for (i = 0; i < n; i++)
		{
			write_data_4bit(_dat);
		}
		GPIO_SetBits(GPIOA, HT1621_CS);
		}
		
		WriteLcdram(_addr, _dat_temp2);
}


//���ڵײ�����д����
void write_addr_dat_n_others(unsigned char _addr, unsigned char _dat, unsigned char n)
{
	
	unsigned char i = 0;
	unsigned char _dat_temp1,_dat_temp2;
	
	//WriteLcdram(_addr, _dat);
		
	

	GPIO_ResetBits(GPIOA, HT1621_CS);								// CS = 0;
	write_mode(DAT);
	
	if(Lcdram[_addr]==0x00)
	{
		WriteLcdram(_addr, _dat);
		
	}
		if((_addr%2)==0)
		{
			_dat_temp1=Lcdram[_addr];
			_dat_temp2=(_dat_temp1&0x01)|_dat;
			
			write_address(_addr);
			for (i = 0; i < n; i++)
		{
			write_data_4bit(_dat_temp2);
		}
		GPIO_SetBits(GPIOA, HT1621_CS);									//CS = 1;
			
		}
		else if((_addr%2)!=0)
		{
			write_address(_addr);
			for (i = 0; i < n; i++)
		{
			write_data_4bit(_dat);
		}
		GPIO_SetBits(GPIOA, HT1621_CS);
		}
		
		//WriteLcdram(_addr, _dat);
		WriteLcdram(_addr, _dat_temp2);
}

//�����ַ�д����
void write_addr_dat_n_char(unsigned char _addr, unsigned char _dat, unsigned char state)
{
	
	unsigned char i = 0;
	unsigned char _dat_temp1,_dat_temp2;
	

		GPIO_ResetBits(GPIOA, HT1621_CS);								// CS = 0;
		 write_mode(DAT);
			
				_dat_temp1=Lcdram[_addr];
			if(state==1)
			{
				
				_dat_temp2=(_dat_temp1|_dat);
			}
			else if(state==0)
			{
				_dat_temp2=(_dat_temp1&(~_dat));
			}
 				write_address(_addr);
			
				for (i = 0; i < 1; i++)
			{
				write_data_4bit(_dat_temp2);
			}
			GPIO_SetBits(GPIOA, HT1621_CS);									//CS = 1;
			WriteLcdram(_addr, _dat_temp2);
		
		
		
	
}
//��ʾ�¶�
//��ڣ�pos����ʾλ�ã���ַ0��2��4�ֱ�Ϊ���ҵ������������
//			num��Ҫ��ʾ��һλ��
void Display_Wendu_1(unsigned char add, unsigned char num )
{
	unsigned char n,i,j;
	n=getChr_Wendu(num);
	i=(n&0xF0)>>4;
	j=n&0x0F;
	write_addr_dat_n_wendu(add,i,1);
	write_addr_dat_n_wendu(add+1,j,1);
	
}
//��ʾ�¶ȵ����ý��������ַ�
//��ڣ�pos����ʾλ�ã���ַ0��2��4�ֱ�Ϊ���ҵ������������
//			num��Ҫ��ʾ����һλ��
void Display_Wendu_2(unsigned char add, unsigned char num )
{
	unsigned char n,i,j;
	n=WenduSetting[num];
	i=(n&0xF0)>>4;
	j=n&0x0F;
	write_addr_dat_n_wendu(add,i,1);
	write_addr_dat_n_wendu(add+1,j,1);
	
}
//��ʾʪ��
//��ڣ�pos����ʾλ�ã���ַ26��28��30�ֱ�Ϊ���ҵ������������
//			num��Ҫ��ʾ��һλ��,������С���㣬�ٷֺţ���RH
void Display_Shidu_3(unsigned char add, unsigned char num )
{
	unsigned char n,i,j;
	n=Shidu2[num];
	i=(n&0xF0)>>4;
	j=n&0x0F;
	

	write_addr_dat_n_shidu(add, i,1);

	write_addr_dat_n_shidu(add+1,j,1);

}
//��ʾʪ�ȵ����ý��������ַ�
//��ڣ�pos����ʾλ�ã���ַ0��2��4�ֱ�Ϊ���ҵ������������
//			num��Ҫ��ʾ����һλ��
void Display_Shidu_2(unsigned char add, unsigned char num )
{
	unsigned char n,i,j;
	n=ShiduSetting[num];
	i=(n&0xF0)>>4;
	j=n&0x0F;
	write_addr_dat_n_wendu(add,i,1);
	write_addr_dat_n_wendu(add+1,j,1);
	
}
//��ʾʪ��
//��ڣ�pos����ʾλ�ã���ַ26��28��30�ֱ�Ϊ���ҵ������������
//			num��Ҫ��ʾ��һλ��
void Display_Shidu_1(unsigned char add, unsigned char num )
{
	unsigned char n,i,j;
	n=getChr_Shidu(num);
	i=(n&0xF0)>>4;
	j=n&0x0F;
	

	write_addr_dat_n_shidu(add, i,1);

	write_addr_dat_n_shidu(add+1,j,1);

}
//��ʾ�ײ����֣�
//��ڣ�pos����ʾλ�ã�11��13��15��17��19��21��23������
//     num:Ҫ��ʾ��һλ����
void Display_Others_1(unsigned char add, unsigned char num )
{
	unsigned char n,i,j;
	n=getChr_Others(num);
	i=(n&0xF0)>>4;
	j=n&0x0F;
	

	write_addr_dat_n_others(add, i,1);

	write_addr_dat_n_others(add+1,j,1);

}

//��ʾ�ַ�
//��ڣ�c��Ҫ��ʾ���ַ�����lcd.h���ж���
//      state�� 1����ʾ 0������ʾ
void Display_CHARS(unsigned char c,unsigned char state)
{

	unsigned char n;
	
	n=CHARS[c];
	
	switch(c)
	{
		case gaobaojing1:
			write_addr_dat_n_char(9,n,state);break;
		case zhengchang1:
			write_addr_dat_n_char(8,n,state);break;
		case dibaojing1:
			write_addr_dat_n_char(8,n,state);break;
		case fengming1:
			write_addr_dat_n_char(6,n,state);break;
		case gaobaojing2:
			write_addr_dat_n_char(9,n,state);break;
		case zhengchang2:
			write_addr_dat_n_char(9,n,state);break;
		case dibaojing2:
			write_addr_dat_n_char(10,n,state);break;
		case fengming2:
			write_addr_dat_n_char(7,n,state);break;
		case hengxian:
			write_addr_dat_n_char(10,n,state);break;
		case dizhi:
			write_addr_dat_n_char(10,n,state);break;
		case botelv:
			write_addr_dat_n_char(12,n,state);break;
		case yicunchu:
			write_addr_dat_n_char(14,n,state);break;
		case moshi:
			write_addr_dat_n_char(18,n,state);break;
		case xindao:
			write_addr_dat_n_char(24,n,state);break;
		case jiluzhong:
			write_addr_dat_n_char(25,n,state);break;
		case zaixian:
			write_addr_dat_n_char(25,n,state);break;
		case lixian:
			write_addr_dat_n_char(25,n,state);break;
		case youliangdian:
			write_addr_dat_n_char(22,n,state);break;
		case youyiheng:
			write_addr_dat_n_char(20,n,state);break;
		case ID:
			write_addr_dat_n_char(25,n,state);break;
		case zuoliangdian:
			write_addr_dat_n_char(16,n,state);break;
		case zuoyiheng:
			write_addr_dat_n_char(16,n,state);break;
		case sheshidu:
			write_addr_dat_n_char(0,n,state);break;
		case huashidu:
			write_addr_dat_n_char(4,n,state);break;
	
	}
}




//�¶�����ת����lcd.c�ڲ�ʹ��
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
//ʪ������ת����lcd.c�ڲ�ʹ�� 
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
//�ײ���������ת����lcd.c�ڲ�ʹ�� 
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
//�¶�


//************************************
// ��������: ��ӡ "OFF"
// ��������: 
// ������Ϣ: 
// ����˵��: 
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

//���SHT20���¶����ݣ���ʾ�¶�
void Display_Wendu(int Temp)
{
	unsigned char Temp_shi,Temp_ge,Temp_xiaoshu;
	
	if(C_F==0){ //���϶�
			Clr_Wendu1();
			
			if(Temp>=0)
			{
			//Temp=(int)(g_sht2x_param.TEMP_HM*10);
			
			Temp_shi=Temp/100;
			Temp_ge=Temp%100/10;
			Temp_xiaoshu=Temp%100%10;
			
			Display_Wendu_1(0, Temp_xiaoshu);//��ʾС��
			write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
			Display_Wendu_1(2, Temp_ge);//��ʾ��λ
			Display_Wendu_1(4, Temp_shi);//��ʾʮλ	
				
			}
			
			
			else if(Temp<0)
			{
			//��ʾ����-
			//write_addr_dat_n_char(7,0x08, 1);//��ʾ����
			write_addr_dat_n_char(6,0x08, 1);//��ʾ����
			Temp=0-Temp;//ת��Ϊ����
			Temp_shi=Temp/100;
			Temp_ge=Temp%100/10;
			Temp_xiaoshu=Temp%100%10;

			Display_Wendu_1(0, Temp_xiaoshu);//��ʾС��
			write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
			Display_Wendu_1(2, Temp_ge);//��ʾ��λ
			Display_Wendu_1(4, Temp_shi);//��ʾʮλ	
				
			}
	}
	else if(C_F==1) //���϶�
	{
			Clr_Wendu2();
			
			if(Temp>=0)
			{
			//Temp=(int)(g_sht2x_param.TEMP_HM*10);
			Temp_shi=Temp/100;
			Temp_ge=Temp%100/10;
			Temp_xiaoshu=Temp%100%10;
			
			if(Temp>=1000)write_addr_dat_n_char(6,0x04,1);
			Display_Wendu_1(0, Temp_xiaoshu);//��ʾС��
			write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
			Display_Wendu_1(2, Temp_ge);//��ʾ��λ
			Display_Wendu_1(4, Temp_shi);//��ʾʮλ	
				
			}
			
			
			else if(Temp<0)
			{
			//��ʾ����-
			//write_addr_dat_n_char(7,0x08, 1);//��ʾ����
				write_addr_dat_n_char(6,0x08, 1);//��ʾ����
			Temp=0-Temp;//ת��Ϊ����
			Temp_shi=Temp/100;
			Temp_ge=Temp%100/10;
			Temp_xiaoshu=Temp%100%10;

			Display_Wendu_1(0, Temp_xiaoshu);//��ʾС��
			write_addr_dat_n_1bit(2,0x08, 1);//��ʾС����
			Display_Wendu_1(2, Temp_ge);//��ʾ��λ
			Display_Wendu_1(4, Temp_shi);//��ʾʮλ	
				
			}
		
	}
	
}

//���SHT20��ʪ�����ݣ���ʾ�¶�
void Display_Shidu(int Humi)
{
	
	unsigned char Humi_shi,Humi_ge,Humi_xiaoshu;
	
	Clr_Shidu();
	
	if(Humi>0)
	{
	//Humi=(int)(g_sht2x_param.HUMI_HM*10);
	
	Humi_shi=Humi/100;
	Humi_ge=Humi%100/10;
	Humi_xiaoshu=Humi%100%10;
	
	Display_Shidu_1(26, Humi_xiaoshu);//��ʾС��
	write_addr_dat_n_1bit(28,0x01, 1);//��ʾС����
	Display_Shidu_1(28, Humi_ge);//��ʾ��λ
	Display_Shidu_1(30, Humi_shi);//��ʾʮλ	
	}
			else if(g_sht2x_param.HUMI_HM<0)
	{
		//��ʾ����-
		
	}
		
	
}

//�¶Ȳ�����ʾȫ��
void Clr_Wendu(void)
{
	write_addr_dat_n(0, 0x00, 6);
	write_addr_dat_n_char(6,0x0C,0);
	
}

void Clr_Wendu1(void) //��������϶ȷ���
{
	write_addr_dat_n(0, 0x08, 1);
	write_addr_dat_n(1, 0x00, 2);
	
}

void Clr_Wendu2(void) //��������϶ȷ���
{
	write_addr_dat_n(0, 0x00, 2);
	write_addr_dat_n(2, 0x08, 1);
	
}


//ʪ�Ȳ�����ʾȫ��

void Clr_Shidu(void)
{
	write_addr_dat_n(26, 0x00, 6);
	write_addr_dat_n_char(7,0x08,0);
	write_addr_dat_n_char(10,0x08,0);
}


//�ײ����ֲ�����ʾȫ��
void Clr_Others(void)

{
	
	write_addr_dat_n(11, 0x00, 14);
	
}


//����lcdram����
void WriteLcdram(unsigned char add, unsigned char data)
{
	
	Lcdram[add]=data;
	
}
//��ʾ��������ķ����ַ�����Ϣ
void DisplayOthers(void)
{
			Display_CHARS(lixian,1);
			Display_CHARS(hengxian,1);
			Display_CHARS(gaobaojing1,1);
			Display_CHARS(zhengchang1,1);
			Display_CHARS(dibaojing1,1);
			Display_CHARS(gaobaojing2,1);
			Display_CHARS(zhengchang2,1);
			Display_CHARS(dibaojing2,1);
			Display_CHARS(sheshidu,1);
}
//extern unsigned char i;
//extern u16 year;
//extern unsigned char month;
//extern unsigned char date;
//extern unsigned char hour;
//extern unsigned char min;
//extern unsigned char sec;
 u16 year;
 unsigned char month;
 unsigned char date;
 unsigned char hour;
 unsigned char min;
 unsigned char sec;

extern u16 year_Set;//2018
extern unsigned char month_Set;
extern unsigned char date_Set;
extern unsigned char hour_Set;
extern unsigned char min_Set;
extern unsigned char sec_Set;
void SetTimeParameters(void)
{
	if(RTC_Get()==0)
		{
			year_Set=calendar.w_year;
			month_Set=calendar.w_month;
			date_Set=calendar.w_date;
			hour_Set=calendar.hour;
			min_Set=calendar.min;
			sec_Set=calendar.sec;				
		}
}
void OnlyDisplayTime(int RTC_Selected,unsigned short     int year,unsigned char month,unsigned char date,unsigned char hour,unsigned char min,unsigned char sec)
{		
	year=year%100;
	if((RTC_Selected<=2)&&(RTC_Selected>=0))//�����յ�����
	{
		Clr_Others();
		Display_CHARS(zuoliangdian,0);
		Display_CHARS(youliangdian,0);
		Display_Others_1(11, year%100/10);
		Display_Others_1(13, year%10);
		Display_Others_1(17, month/10);
		Display_Others_1(19, month%10);
		Display_Others_1(21, date/10);
		Display_Others_1(23, date%10);
		Display_CHARS(zuoyiheng,1);
		Display_CHARS(youyiheng,1);
	}
	else                                    //ʱ���������
	{
		Clr_Others();
		Display_CHARS(ID,0);
		Display_CHARS(zuoliangdian,1);
		Display_CHARS(youliangdian,1);
		Display_Others_1(11, hour/10);
		Display_Others_1(13, hour%10);
		Display_Others_1(17, min/10);
		Display_Others_1(19, min%10);
		Display_Others_1(21, sec/10);
		Display_Others_1(23, sec%10);
	}
	
}
//��ʾ�ײ�ʱ�����Ϣ������������Ϣ��
void DisplayTime(void)
{
	TIM_Cmd(TIM2, ENABLE);
	if(RTC_Get()==0)
		{
			year=calendar.w_year%100;
			month=calendar.w_month;
			date=calendar.w_date;
			hour=calendar.hour;
			min=calendar.min;
			sec=calendar.sec;				
		}

			Display_CHARS(hengxian,1);
		if(RemoteControl)
		{	
			DisplayRssi(RssiGrade);
//				Display_CHARS(zaixian,1);
//				Display_CHARS(lixian,0);
		}
		else 
		{
			write_addr_dat_n_char(7,2,0);
			write_addr_dat_n_char(7,4,0);
			write_addr_dat_n_char(6,2,0);
			write_addr_dat_n_char(6,1,0);
//				Display_CHARS(zaixian,0);
//				Display_CHARS(lixian,1);
		}

		
		
		if(V_BAT>3.87)
		{		
				write_addr_dat_n_char(9,0x01, 1);//��ؿ�
				write_addr_dat_n_char(7,0x01, 1);//��һ���
				write_addr_dat_n_char(8,0x01, 1);//�ڶ����
				write_addr_dat_n_char(8,0x02, 1);//�������
			
		}
		else if(V_BAT>3.77&&V_BAT<=3.87)
		{
				write_addr_dat_n_char(9,0x01, 1);//��ؿ�
				write_addr_dat_n_char(7,0x01, 1);//��һ���
				write_addr_dat_n_char(8,0x01, 1);//�ڶ����
				write_addr_dat_n_char(8,0x02, 0);//�������
		}
		else if(V_BAT>3.5&&V_BAT<=3.77)
		{
				write_addr_dat_n_char(9,0x01, 1);//��ؿ�
				write_addr_dat_n_char(7,0x01, 1);//��һ���
				write_addr_dat_n_char(8,0x01, 0);//�ڶ����
				write_addr_dat_n_char(8,0x02, 0);//�������
			
		}
		else if(V_BAT<3.5)
		{
			write_addr_dat_n_char(9,0x01, 1);//��ؿ�
			write_addr_dat_n_char(7,0x01, 0);//��һ���
			write_addr_dat_n_char(8,0x01, 0);//�ڶ����
			write_addr_dat_n_char(8,0x02, 0);//�������
		}	 
			if(count>0&&count<5)  //��ʾ��ַ
			{
				
				Clr_Others();
				for(i=0;i<100;i++)
				{
					Display_CHARS(ID,1);
					Display_Others_1(11, DeviceID/100);
					Display_Others_1(13, DeviceID%100/10);
					Display_Others_1(15, DeviceID%10/1);
				}

			}
			else if (count>=5&&count<10)//��ʾ�洢
			{
				
				Clr_Others();
				for(i=0;i<100;i++){
					Display_CHARS(ID,0);
					Display_CHARS(yicunchu,1);
					Display_Others_1(11, LCD_yicunchu/10000);
					Display_Others_1(13, LCD_yicunchu%10000/1000);
					Display_Others_1(15, LCD_yicunchu%1000/100);
					Display_Others_1(17, LCD_yicunchu%100/10);
					Display_Others_1(19, LCD_yicunchu%10);
				}

			}
			else if (count>=10&&count<15)  //��ʾģʽ
			{
				Clr_Others();
				for(i=0;i<100;i++)
				{
					Display_CHARS(yicunchu,0);
					Display_CHARS(moshi,1);
					Display_Others_1(11, RemoteControl%100/10);
					Display_Others_1(13, RemoteControl%10);
				}
			}
			else if(count>=15&&count<20)   //��ʾ������
			{
					Clr_Others();
					for(i=0;i<100;i++)
					{
						Display_CHARS(ID,0);
						Display_CHARS(zuoliangdian,0);
						Display_CHARS(youliangdian,0);
						Display_Others_1(11, year%100/10);
						Display_Others_1(13, year%10);
						Display_Others_1(17, month/10);
						Display_Others_1(19, month%10);
						Display_Others_1(21, date/10);
						Display_Others_1(23, date%10);
						Display_CHARS(zuoyiheng,1);
						Display_CHARS(youyiheng,1);
					}
			}
			else if (count>=20&&count<25)    //��ʾʱ����ʱ��
			{
					Clr_Others();
					for(i=0;i<100;i++)
					{
						Display_CHARS(ID,0);
						Display_CHARS(zuoliangdian,1);
						Display_CHARS(youliangdian,1);
						Display_Others_1(11, hour/10);
						Display_Others_1(13, hour%10);
						Display_Others_1(17, min/10);
						Display_Others_1(19, min%10);
						Display_Others_1(21, sec/10);
						Display_Others_1(23, sec%10);
					}
				
			}
		
			else if (count>=25)
			{
				count=0;
			}

}

void DisplayRssi(int rssi)
{
	if(rssi == 1)
	{
		write_addr_dat_n_char(7,2,1);
		write_addr_dat_n_char(7,4,0);
		write_addr_dat_n_char(6,2,0);
		write_addr_dat_n_char(6,1,0);
	}
	else if(rssi == 2)
	{
		write_addr_dat_n_char(7,2,1);
		write_addr_dat_n_char(7,4,1);
		write_addr_dat_n_char(6,2,0);
		write_addr_dat_n_char(6,1,0);
	}
	else if(rssi == 3)
	{
		write_addr_dat_n_char(7,2,1);
		write_addr_dat_n_char(7,4,1);
		write_addr_dat_n_char(6,2,1);
		write_addr_dat_n_char(6,1,0);
	}
	else
	{
		write_addr_dat_n_char(7,2,1);
		write_addr_dat_n_char(7,4,1);
		write_addr_dat_n_char(6,2,1);
		write_addr_dat_n_char(6,1,1);
	}

}
