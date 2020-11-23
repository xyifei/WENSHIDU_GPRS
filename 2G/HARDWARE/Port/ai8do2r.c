/**********************************************************************************
 * 文件名  ：ai8do2r.c
 * 描述    ：ai8do2r初始化及接口函数控制         
 * 实验平台：
 * 硬件连接 ----------------------------------
 *         | PA1(上拉）     : AD7731_19_*CS   |
           | PE11(上拉）    : LTC2602_1_*CS	  |
 *         | PA5-SPI1-SCK  : AD7731_1_SCLK   |
           | (上拉+5V）       LTC2602_2_SCk   |
		   |				  AD5750_7_CLK    |
 *         | PA6-SPI1-MISO : AD7731_21_DOUT  |
           |（上拉+5V)		  AD5750_1_SDO	  |
 *         | PA7-SPI1-MOSI : AD7731_22_DIN	  |
           | (上拉+5V）	      LTC2602_3_SDI	  |
		   |                  AD5750_8_SDIN	  |
		   | PA0-7731-RDY   : AD7731_20_*RDY  |
		   | （上拉+5V）					  |
		   | PE13-5750-SYNC  : AD5750_6_*SYNC |
		   | （上拉+5V）					  |
		   | PA4-IO-RESET   : AD7731_6_*RESET |
		   | （上拉+5V）      AD5750_30_*RESET|
		   | PC4-508-A0     : ADG508_1_A0	  |
		   | PE9-508-A1     : ADG508_16_A1	  |
		   | PE7-508-A2     : ADG508_15_A0	  |
		   |  
		   |                                  |
 *          ----------------------------------
 * 库版本  ：ST3.5.0
**********************************************************************************/
#include "ai8do2r.h"
#include "mb.h"
#include "delay.h"
#include <stdarg.h>
//#include "stdio.h"
//#include "stdbool.h"
/*----------------------------------------------------------------
名称：         模拟量输入模式
用途：         模拟量输入，范围，及数据模式。
//-----------------------------------------------------------------*/
#define AD7731_MODE_NREGS   16
USHORT  ad7731ModeBuf[AD7731_MODE_NREGS]={0x3074,0x3074,0x3074,0x3074,0x3074,0x3074,0x3074,0x3074};

extern unsigned char ucRegDiscBuf[];      //开关量输入寄存器ucRegDiscBuf
extern unsigned char ucRegCoilsBuf[];	 //开关量输出寄存器ucRegCoilsBuf
extern USHORT  usRegInputBuf[];	    //输入寄存器usRegInputBuf
extern USHORT  usRegHoldingBuf[];   //保持/输出寄存器usRegHoldingBuf

/*******************************************************************************
* Function Name  : GPIO_AD5750_Configuration
* Description    : 初始化ai8do2r模块GPIO
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_AD5750_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD 
                         |RCC_APB2Periph_GPIOE |RCC_APB2Periph_AFIO, ENABLE);
	/*PA5-SCK,PA7-MOSI */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5  | GPIO_Pin_7;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	AD5750_SCK = 1;
   /*PA6-MISO */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5  | GPIO_Pin_7;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/*!< Configure AD5750_SYNC_pin: PE13 */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_Init(GPIOE, &GPIO_InitStructure);
  	DA_5750_SYNC_LOW(); //NO OUTPUT

}
/*******************************************************************************
* Function Name  : GPIO_Di8do8l_Configuration
* Description    : 初始化ai8do2r模块GPIO
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_ai8do2r_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD 
                         |RCC_APB2Periph_GPIOE |RCC_APB2Periph_AFIO, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  
  /*!< Configure SPI1_ADDA_SPI pins: SCK */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /*!< Configure SPI1_ADDA_SPI pins: MISO */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /*!< Configure SPI1_ADDA_SPI pins: MOSI */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
 
  /*!< Configure SPI_AD7731_SPI_CS_PIN pin: SPI_AD7731 CS pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  SPI_AD7731_CS_HIGH(); //Deselect the AD7731: Chip Select high 
  
  /*!< Configure SPI_DALTC2601_SPI_CS_PIN pin: SPI_DSLTC2601 CS pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  SPI_DA2602_CS_HIGH(); // Deselect the DA2601: Chip Select high 
 
  /*!< Configure AD5750_SYNC_pin: PE13 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  DA_5750_SYNC_LOW(); //NO OUTPUT

  /*!< Configure AD_ADG508_pin: PC4_AO*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  AD_ADG508_A0_LOW();

  /*!< Configure AD_ADG508_pin: PE9_A1 PE7_A2*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_9 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  AD_ADG508_A1_LOW();
  AD_ADG508_A2_LOW(); //Initialization AI0 Channel
  
  /*!< Configure Reset_pin: PA4 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  AD_DA_RESET_HIGH(); //Deselect the Reset

  /*!< Configure AD7731_RDY_pin: PA0 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
}

/*******************************************************************************
* Function Name  : SPI2_AD7731_Init
* Description    : Initializes the AD7731 used by the SPI2 driver.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI2_AD7731_Init(void)
{
   /* SPI2 configuration */
  SPI_InitTypeDef  SPI_InitStructure;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;   //SPI设置为双线双向全双工
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						   //设置为主SPI
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					   //SPI发送接收8位帧结构
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;						   //时钟悬空低
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;						   //数据捕获于第二个时钟沿
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							   //内部NSS信号有SSI位控制
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;   //波特率预分频值为16 2M
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;				   //数据传输从MSB位开始
  SPI_InitStructure.SPI_CRCPolynomial = 7;							   //CRC校验值
  SPI_Init(SPI1, &SPI_InitStructure);

  /* Enable SPI1  */
  SPI_Cmd(SPI1, ENABLE);
}

void SPI2_DA2602_Init(void)
{
   /* SPI1 configuration */
  SPI_InitTypeDef  SPI_InitStructure;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;   //SPI设置为双线双向全双工
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						   //设置为主SPI
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					   //SPI发送接收8位帧结构
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;						   //时钟悬空低
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;						   //数据捕获于第二个时钟沿
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							   //内部NSS信号有SSI位控制
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;  //波特率预分频值为16 2M
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;				   //数据传输从MSB位开始
  SPI_InitStructure.SPI_CRCPolynomial = 7;							   //CRC校验值
  SPI_Init(SPI1, &SPI_InitStructure);

  /* Enable SPI1  */
  SPI_Cmd(SPI1, ENABLE); 
}
void SPI2_AD5750_Init(void)
{
   /* SPI2 configuration */
  SPI_InitTypeDef  SPI_InitStructure;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;   //SPI设置为双线双向全双工
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						   //设置为主SPI
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;				   //SPI发送接收16位帧结构
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;						   //时钟悬空低
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;						   //数据捕获于第一个时钟沿
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							   //内部NSS信号有SSI位控制
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;  //波特率预分频值为16 2M
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;				   //数据传输从MSB位开始
  SPI_InitStructure.SPI_CRCPolynomial = 7;							   //CRC校验值
  SPI_Init(SPI1, &SPI_InitStructure);

  /* Enable SPI2  */
  SPI_Cmd(SPI1, ENABLE);
}

/*********************************************************************
 * AD7731 读状态寄存器
 * 发送数据: 
 * 位功能：  | WEN* | ZERO | RW1 | RW0 | ZERO | RS2 | RS1 | RS0 |
 *    bit：  |  7   |   6  |  5  |  4  |   3  |  2  |  1  |  0  |
 *   代码：  |  0   |   0  |  0  |  1  |   0  |  0  |  0  |  0  | 
 * 功能说明：|写使能| 仅为0|  读写功能 |仅为0 | 读写寄存器选择  |  
 *	 发送：0x10
 *  ——————————————————————————————————————————————————————————————————————————
 * 返回数据: 
 * 位功能：  |  RDY*  | STDY*   | STBY   |NOREF   | MS3  | MS2 | MS1 | MS0 |
 *    bit：  |   7    |   6     |  5     |  4     |   3  |  2  |  1  |  0  |
 *   代码：  |   1    |   1     |  0     |  0     |   1  |  1  |  0  |  0  | 
 *功能说明： |RDY就绪 | STDY稳定|待机状态|参考电压|      出厂默认设置      |
 *	 返回：0xCC		 
***********************************************************************/
void SPI_ReadAD7731_StatusReg(void)
{
  /* Select the AD7731: Chip Select low */
   SPI_AD7731_CS = 0;
  /* Send "RDID " instruction */
  SPI_AD7731_SendByte(0x10);
  /* Read a byte from the FLASH */
  usRegInputBuf[0] = SPI_AD7731_SendByte(0xFF);
  /* Deselect the FLASH: Chip Select high */
   SPI_AD7731_CS = 1;
}

/******************************************************************************
 * AD7731 读模式寄存器 发送 0x12 返回 0xCC
 * 发送数据: 
 * 位功能：  | WEN* | ZERO | RW1 | RW0 | ZERO | RS2 | RS1 | RS0 |
 *    bit：  |  7   |   6  |  5  |  4  |   3  |  2  |  1  |  0  |
 *   代码：  |  0   |   0  |  0  |  1  |   0  |  0  |  1  |  0  | 
 * 功能说明：|写使能| 仅为0|  读写功能 |仅为0 | 读写寄存器选择  |  
 *	 发送：0x12
 * ——————————————————————————————————————————————————————————————————————————————————
 * 返回数据: 
 * 位功能：  |         MODE Bits         |   B*U     |DOEN   | DO1  | DO0 |  WL    |
 *    bit：  |   15   |   14    |  13    |    12     | 11    |  10  |  9  |  8     |
 *   代码：  |   0    |   0     |  0     |     0     |  1    |  1   |  0  |  0     | 
 *功能说明： |  数据转换模式及校准模式   |单极性     |DO使能 | 如果DOEN=1 |数据长度|
 *			 |   000 同步模式			 |双极性	 |1=IN3	 | DO1 DO0	  |0=16Bit |
 *			 |	 001 连续转换模式		 |0=双极性 	 |	IN4  | 开关量输出 |1=24Bit |
 *			 |	 010 单步转换模式		 |00 负满量程|DO输出 |			  |		   |
 *			 |	 011 待机模式			 |10~0V		 |0=IN3	 |			  |		   |
 *			 |	 100 内部零点校准		 |11~正满量程|	IN4	 |			  |		   |
 *			 |	 101 内部满量程校准		 |1=单极性	 |AI输入 |			  |		   |
 *			 |	 110 系统零点校准		 |00~ 0V	 |		 |			  |		   |
 *			 |	 111 系统满量程校准		 |11~正满量程|		 |			  |		   |
 *—————————————————————————————————————————————————————————————————————————————————
 * 位功能：  | HIREF   |      Input Range Bits         |  BO    |In Channel Select |
 *    bit：  |   7     |   6    |  5      |    4       |   3    |  2   |  1  | 0   |
 *   代码：  |   0     |   0    |  0      |     0      |   1    |  1   |  0  |  0  | 
 *功能说明： |REF设置  |  双极性  设置B*U  单极性      |烧毁电流|输入通道模式设置  |
 *           |0=2.5VREF|000=-20~+20mV     000=0~+20mV  |1=激活  |000=AI1~AI6 伪差分|
 *			 |1=5V REF |001=-20~+20mV     001=0~+20mV  |0=关闭	|001=AI2~AI6 伪差分|
 *			 |	 	   |010=-40~+40mV     000=0~+40mV  | 	    |010=AI3~AI6 伪差分|
 *			 |	 	   |011=-80~+80mV     000=0~+80mV  |        |011=AI4~AI6 伪差分|
 *			 |	 	   |100=-160~+160mV   100=0~+160mV |		|100=AI1~AI2 全差分|			  		   |
 *			 |		   |101=-320~+320mV   100=0~+320mV |        |101=AI3~AI4 全差分|
 *			 |		   |110=-160~+160mV   100=0~+160mV |	    |110=AI5~AI6 全差分|		  
 *			 |	 	   |111=-1.28~+1.28mV 100=0~+1.28mV|	    |111=AI4~AI6 测试  |
********************************************************************************/
void SPI_ReadAD7731_ModeReg(void)
{
  u8 temp1,temp2 ;
  /* Select the AD7731: Chip Select low */
   SPI_AD7731_CS = 0;
  /* Send "RDID " instruction */
  SPI_AD7731_SendByte(0x12);
   SPI_AD7731_CS = 1;
   delay_us(1);
    SPI_AD7731_CS = 0;
  /* Read a byte from the FLASH */
  temp1 = SPI_AD7731_SendByte(0x20);
  temp2 = SPI_AD7731_SendByte(0x74);
  usRegInputBuf[8] = (temp1<<8) + temp2;
  /* Deselect the FLASH: Chip Select high */
  SPI_AD7731_CS = 1;
}
/***************************************
 * AD7731 读滤波寄存器 发送 0x13 返回 0x2002
 * 发送数据: 
 * 位功能：  | WEN* | ZERO | RW1 | RW0 | ZERO | RS2 | RS1 | RS0 |
 *    bit：  |  7   |   6  |  5  |  4  |   3  |  2  |  1  |  0  |
 *   代码：  |  0   |   0  |  0  |  1  |   0  |  0  |  1  |  1  | 
 * 功能说明：|写使能| 仅为0|  读写功能 |仅为0 | 读写寄存器选择  |  
 *	 发送：0x13
 *  ————————————————————————————————————————————————————————————————————————————————————————————
 * 返回数据: 
 * 位功能：  |                 Sinc3 Filter SC                     |ZERO | CHP  |    SKIP| FR0  |                
 *    bit：  | 15 | 14 | 13 | 12 | 11 | 10 | 9 | 8 | 7 | 6 | 5 | 4 |  3  |  2   |     1  |  0   | 
 *   代码：  |                                                     |  0  |  0   |     1  |  0   | 
 *功能说明： |                                                     |仅为0|Sinc3滤波选择	 | FAST |
 *			 |                                                     |     |00=150~2.048KHz| 模式 |
 *			 |                                                     |     |10=50~1.365KHz |0=关闭|
 *			 |		                                      	       |	 |01=150~7.6KHz	 |1=打开|
 *			 |                                      			   |	 |11=50~5.12KHz	 |		|
**************************************************************************************************/
void SPI_ReadAD7731_FilterReg(void)
{
  u8 temp1,temp2 ;
  /* Select the AD7731: Chip Select low */
   SPI_AD7731_CS = 0;
  /* Send "RDID " instruction */
  SPI_AD7731_SendByte(0x13);
  /* Read a byte from the FLASH */
  temp1 = SPI_AD7731_SendByte(0xFF);
  temp2 = SPI_AD7731_SendByte(0xFF);
  usRegInputBuf[2] = (temp1<<8) + temp2;
  /* Deselect the FLASH: Chip Select high */
   SPI_AD7731_CS = 1;
}
/**************************************************************************
 *AD7731 读偏置校准寄存器（效零点） 发送 0x15 返回 0x80000
 * 发送数据: 
 * 位功能：  | WEN* | ZERO | RW1 | RW0 | ZERO | RS2 | RS1 | RS0 |
 *    bit：  |  7   |   6  |  5  |  4  |   3  |  2  |  1  |  0  |
 *   代码：  |  0   |   0  |  0  |  1  |   0  |  1  |  0  |  1  | 
 * 功能说明：|写使能| 仅为0|  读写功能 |仅为0 | 读写寄存器选择  |  
 *	 发送：0x15
 *  ————————————————————————————————————————————————————————————————————————
 * 返回数据: 
 * 位功能：  |                                                             |                
 *    bit：  |23|22|21|20|19|18|17|16|15|14|13|12|11|10|9|8|7|6|5|4|3|2|1|0| 
 *   代码：  |                                                             | 
 *功能说明： |                                                             |
****************************************************************************/
void SPI_ReadAD7731_OffsetReg(void)
{
  u8 temp1,temp2,temp3 ;
  /* Select the AD7731: Chip Select low */
   SPI_AD7731_CS = 0;
  /* Send "RDID " instruction */
  SPI_AD7731_SendByte(0x15);
  /* Read a byte from the FLASH */
  temp1 = SPI_AD7731_SendByte(0xFF);
  temp2 = SPI_AD7731_SendByte(0xFF);
  temp3 = SPI_AD7731_SendByte(0xFF);
  usRegInputBuf[3] = (temp1<<8) + temp2;
  usRegInputBuf[4] = (temp3<<8);
  /* Deselect the FLASH: Chip Select high */
   SPI_AD7731_CS = 1;
}
/**************************************************************************
 * AD7731 读增益寄存器（校满量程） 发送 0x16 返回 0x593CEA
 * 发送数据: 
 * 位功能：  | WEN* | ZERO | RW1 | RW0 | ZERO | RS2 | RS1 | RS0 |
 *    bit：  |  7   |   6  |  5  |  4  |   3  |  2  |  1  |  0  |
 *   代码：  |  0   |   0  |  0  |  1  |   0  |  1  |  1  |  1  | 
 * 功能说明：|写使能| 仅为0|  读写功能 |仅为0 | 读写寄存器选择  |  
 *	 发送：0x15
 *  ———————————————————————————————————————————————————————————————————————
 * 返回数据: 
 * 位功能：  |                                                             |                
 *    bit：  |23|22|21|20|19|18|17|16|15|14|13|12|11|10|9|8|7|6|5|4|3|2|1|0| 
 *   代码：  |                                                             | 
 *功能说明： |                                                             |
***************************************************************************/
void SPI_ReadAD7731_GainReg(void)
{
  u8 temp1,temp2,temp3;
  /* Select the AD7731: Chip Select low */
   SPI_AD7731_CS = 0;
  /* Send "RDID " instruction */
  SPI_AD7731_SendByte(0x16);
  /* Read a byte from the FLASH */
  temp1 = SPI_AD7731_SendByte(0xFF);
  temp2 = SPI_AD7731_SendByte(0xFF);
  temp3 = SPI_AD7731_SendByte(0xFF);
  usRegInputBuf[5] = (temp1<<8) + temp2;
  usRegInputBuf[6] = (temp3<<8);
  /* Deselect the FLASH: Chip Select high */
   SPI_AD7731_CS = 1;
}
/***************************************
 AD7731 写模式寄存器 发送 0x02 0x1174 返回 0x593CEA
****************************************/
void SPI_WriteAD7731_ModeReg(u16 adMode)
{
  	u8 temp1,temp2 ;
	temp1 = adMode/0x100;
	temp2 = adMode%0x100;
  /* Select the AD7731: Chip Select low */
   SPI_AD7731_CS = 0;
  /* Send "RDID " instruction */
  SPI_AD7731_SendByte(0x02);
   SPI_AD7731_CS = 1;
	delay_us(2);
	SPI_AD7731_CS = 0;
  SPI_AD7731_SendByte(temp1);
  SPI_AD7731_SendByte(temp2);
  SPI_AD7731_CS = 1;
}
/***************************************
 ADG508通道选择
****************************************/
void ADG508_ChannelSet(u8 usChannel)
{
	unsigned char temp;
	temp=usChannel;
	AD_ADG508_A0 = ((temp & (0x01<<0))>>0);
	AD_ADG508_A1 = ((temp & (0x01<<1))>>1);
	AD_ADG508_A2 = ((temp & (0x01<<2))>>2);
  
}

/***************************************
 AD7731 读数据寄存器 
****************************************/
void SPI_ReadAD7731_DataReg(void)
{
  	
    u8 temp1=0,temp2=0;
	u8 i=0;
	for(i = 0;i < 8;i ++)
	{
	 ADG508_ChannelSet(i);
	 delay_us(1000);
//	 temp3=ad7731ModeBuf[i];
//	 SPI_WriteAD7731_ModeReg(temp3);
  	 while (AD7731_RDY == 1);
	 SPI_AD7731_CS = 0;
     SPI_AD7731_SendByte(0x11);
	 SPI_AD7731_CS = 1;
	 delay_us(2);
	 SPI_AD7731_CS = 0;
	 temp1 = SPI_AD7731_SendByte(0xFF);
 	 temp2 = SPI_AD7731_SendByte(0xFF);
	 usRegInputBuf[i] = (temp1<<8) + temp2;
	 SPI_AD7731_CS = 1;
	 temp1=0,temp2=0;
  }
}

/***************************************
LTC2602 写数据寄存器并更新寄存器 发送 0x02 0x1174 返回 0x593CEA
        OUTA 0x30  +DATA
		OUTB 0x31  +DATA
****************************************/
void SPI_WriteDA2602_DataReg(void)
{
	u8 temp1,temp2,temp3,temp4;
	temp1 = usRegHoldingBuf[0]/0x100;
	temp2 = usRegHoldingBuf[0]%0x100;
	temp3 = usRegHoldingBuf[1]/0x100;
	temp4 = usRegHoldingBuf[1]%0x100;
  /*OUT1 Select the AD7731: Chip Select low */
  SPI_DA2602_CS = 0;
  delay_us(2);
  /* Send "RDID " instruction */
  SPI_AD7731_SendByte(0x30);
  SPI_AD7731_SendByte(temp1);
  SPI_AD7731_SendByte(temp2);
//  SPI_AD7731_SendByte(0x7F);
//  SPI_AD7731_SendByte(0xFF);
  /* Deselect the FLASH: Chip Select high */
  SPI_DA2602_CS = 1;
  delay_us(2);
   /* OUT2 Select the AD7731: Chip Select low */
  SPI_DA2602_CS = 0;
  delay_us(2);
  /* Send "RDID " instruction */
  SPI_AD7731_SendByte(0x31);
  SPI_AD7731_SendByte(temp3);
  SPI_AD7731_SendByte(temp4);
//  SPI_AD7731_SendByte(0x7F);
//  SPI_AD7731_SendByte(0xFF);
  delay_us(2);
 
  /* Deselect the FLASH: Chip Select high */
 SPI_DA2602_CS = 1;
}
/*******************************************************************************************************************************
*读取AD5750寄存器数据 u16 SPI_ReadAD5750CH_DataReg(u16 usAddress)
 *发送数据: 
 * 位功能：  |  A2   A1  A0 | R/W*  | D11 | R3 | R2 | R1 | R0 |CLRSEL | OUTEN |RSET   |PECERR|OVERTEMP |IOUTFAULT |VOUTFAULT  |
 *    bit：  | 15 | 14 | 13 |  12   | 11  | 10 | 9  | 8  | 7  |   6   |	5	  |	4	  |	 3	 |	 2	   |  1		  |	   0	  |
 *   代码：  | 0/1| 0/1| 0/1|  0/1  |  0  | 0/1| 0/1|0/1 |0/1 |	0/1   |0/1	  |0/1    |0/1	 |0/1	   | 0/1	  |	0/1		  |
 * 功能说明：|000 器件0地址 |0写操作|仅为0|0000 4-20mA        |0清至0 |0不输出|0正常  |0错误 |0温度正常|0 Iout正常|0 Vout正常 | 
 * 			 |001 器件1地址	|1读操作|	  |0001 0-20mA        |1清自中|1输出  |1清零  |1错误 |1超过150 |1 Iout开路|1 Vout短路 |
 *			 |010 器件2地址	|			  |0011 +/-20mA       |模式	  |
 *			 |...........	|			  |..........         |									 
 *	 地址0器件发送：0x1000; 地址2器件发送 0x3000;
 *******************************************************************************************************************************/
u16 SPI_ReadAD5750CH_DataReg(u16 usAddress)
{
    u16 temp;
	temp = SPI_AD5750_SendByte(usAddress);
	return temp;
 
}
/*******************************************************************************************************************************
 *写入AD5750寄存器数据 u16 SPI_WriteAD5750CH_DataReg(u16 usAdddate)
 *发送数据: 
 * 位功能：  |  A2   A1  A0 | R/W*  | D11 | R3 | R2 | R1 | R0 |CLRSEL | OUTEN |Clear  |RSET      |Reset |D1   | D0  |
 *    bit：  | 15 | 14 | 13 |  12   | 11  | 10 | 9  | 8  | 7  |   6   |	5	  |	4	  |	 3	     |	 2	| 1	  |	 0  |
 *   代码：  | 0/1| 0/1| 0/1|  0/1  |  0  | 0/1| 0/1|0/1 |0/1 |	0/1   |0/1	  |0/1    |0/1	     |0/1	| 0	  |	 0	|	 
 * 功能说明：|000 器件0地址 |0写操作|仅为0|0000 4-20mA        |0清至0 |0不输出|0不清零|0外部电阻 |0正常 |仅为0|仅为0| 
 * 			 |001 器件1地址	|1读操作|	  |0001 0-20mA        |1清自中|1输出  |1清零  |1内部电阻 |1复位 |   
 *			 |010 器件2地址	|			  |0011 +/-20mA       |模式	  |
 *			 |...........	|			  |..........         |									 
 *	 地址0器件发送：0x2A0 外部电阻0-5V输出; 地址2器件发送 0x22A0 外部电阻0-5V输出;
 *******************************************************************************************************************************/
u16 SPI_WriteAD5750CH_DataReg(u16 usAdddate)
{
   	u16 temp;
	SPI_AD5750_SendByte(usAdddate);
	temp = usAdddate;
 	return temp;
}
/********************************************************************
 *   系统函数 													    *
 *	 SPI_I2S_GetFlagStatus	//检查指定的SPI标志位设置与否		    *
 *    	SPI_I2S_FLAG_TXE:       //发送缓存空标志位. 			    *
 *   	SPI_I2S_FLAG_RXNE:      //接受缓存非空标志位. 				*
 *  	SPI_I2S_FLAG_BSY:       //忙标志位. 						*
 *   	SPI_I2S_FLAG_OVR:       //超出标志位. 						*
 *	    SPI_FLAG_MODF:          //模式错位标志位. 					*
 *  	SPI_FLAG_CRCERR:        //CRC错误标志位. 					*
 *  	I2S_FLAG_UDR:           //欠载错误标志. 					*
 *  	I2S_FLAG_CHSIDE:        //通道侧标志.					    *
 *	    Return values:			//SET or RESET  					*
 *	 SPI_I2S_SendData           //通过外设SPIx发送一个数据 u8或u16	*
 *	 SPI_I2S_ReceiveData        //返回通过SPIx最近接收的数据 u8或u16*
 ********************************************************************/	
u8 SPI_AD7731_SendByte(u8 byte)
{								
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);//检查SPI2的发送缓存器是否为空
		
		 SPI_I2S_SendData(SPI1, byte); //如果发送缓存器为空，通过SPI2发送一个字节数据 a byte
          
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); //等待接收缓存器接收到 a byte 
  
  		 return SPI_I2S_ReceiveData(SPI1); //返回接收到的数据 a byge
}
/********************************************************************
 *   系统函数 													    *
 *	 SPI_I2S_GetFlagStatus	//检查指定的SPI标志位设置与否		    *
 *    	SPI_I2S_FLAG_TXE:       //发送缓存空标志位. 			    *
 *   	SPI_I2S_FLAG_RXNE:      //接受缓存非空标志位. 				*
 *  	SPI_I2S_FLAG_BSY:       //忙标志位. 						*
 *   	SPI_I2S_FLAG_OVR:       //超出标志位. 						*
 *	    SPI_FLAG_MODF:          //模式错位标志位. 					*
 *  	SPI_FLAG_CRCERR:        //CRC错误标志位. 					*
 *  	I2S_FLAG_UDR:           //欠载错误标志. 					*
 *  	I2S_FLAG_CHSIDE:        //通道侧标志.					    *
 *	    Return values:			//SET or RESET  					*
 *	 SPI_I2S_SendData           //通过外设SPIx发送一个数据 u8或u16	*
 *	 SPI_I2S_ReceiveData        //返回通过SPIx最近接收的数据 u8或u16*
 ********************************************************************/	
u16 SPI_AD5750_SendByte(u16 byte)
{	
    AD5750_SYNC= 0 ;
	nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;
	AD5750_SYNC= 1 ;
    nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;
	AD5750_SYNC= 0 ;
	nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;							
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);//检查SPI2的发送缓存器是否为空
	SPI_I2S_SendData(SPI1, byte); //如果发送缓存器为空，通过SPI2发送一个字节数据 a byte  
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); //等待接收缓存器接收到 a byte 
	nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;
	AD5750_SYNC= 1 ;
	return SPI_I2S_ReceiveData(SPI1); //返回接收到的数据 a byge
}

/*	#define AD5750_SYNC	              PEout(13)
	#define AD5750_SCK	              PAout(5)
	#define AD5750_MOSI	              PAout(7)
	#define AD5750_MISO	              PAin(6) */

//u16 AD5750_SendByte(u16 usAdddate)
//{
//	u16 temp = usAdddate;
////	u16 temp1;
//	u8 i,y;
//	AD5750_SYNC= 0 ;
//	delay_us(2);
//	AD5750_SYNC= 1 ;
//    delay_us(2);
//	AD5750_SYNC= 0 ;
//	delay_us(2);
//	for(i = 0;i < 16;i ++)
//	{
//	    y = 15 - i;
//	    AD5750_SCK = 1;
//		AD5750_MOSI = ((usAdddate & (0x01<<y))>>y);		
//		delay_us(4);
//		AD5750_SCK = 0;
//		delay_us(1);
//	 }
//	 delay_us(1);
//	 AD5750_SYNC= 1 ;
//	return temp;
// }
//
// u16 AD5750_ReadByte(u16 usAdddate)
//{
//	u16 temp;
////	u16 temp1;
//	u8 i,y;
//	AD5750_SYNC= 1 ;
//    delay_us(2);
//	AD5750_SYNC= 0 ;
//	delay_us(2);
//	for(i = 0;i < 16;i ++)
//	{
//	    y = 15 - i;
//		AD5750_MOSI = ((usAdddate & (0x01<<y))>>y);
//		AD5750_SCK = 0;		
//		delay_us(3);
//		temp|=(AD5750_MISO<<y);
//		AD5750_SCK = 1;
//		nop;nop;nop;nop;
//	 }
//	 delay_us(1);
//	 AD5750_SYNC= 1 ;
//	 delay_us(5);
//	 AD5750_SYNC= 0 ;
//	return temp;
// }
/******************************END OF FILE*****************************/

