/**********************************************************************************
 * �ļ���  ��ai8do2r.c
 * ����    ��ai8do2r��ʼ�����ӿں�������         
 * ʵ��ƽ̨��
 * Ӳ������ ----------------------------------
 *         | PA1(������     : AD7731_19_*CS   |
           | PE11(������    : LTC2602_1_*CS	  |
 *         | PA5-SPI1-SCK  : AD7731_1_SCLK   |
           | (����+5V��       LTC2602_2_SCk   |
		   |				  AD5750_7_CLK    |
 *         | PA6-SPI1-MISO : AD7731_21_DOUT  |
           |������+5V)		  AD5750_1_SDO	  |
 *         | PA7-SPI1-MOSI : AD7731_22_DIN	  |
           | (����+5V��	      LTC2602_3_SDI	  |
		   |                  AD5750_8_SDIN	  |
		   | PA0-7731-RDY   : AD7731_20_*RDY  |
		   | ������+5V��					  |
		   | PE13-5750-SYNC  : AD5750_6_*SYNC |
		   | ������+5V��					  |
		   | PA4-IO-RESET   : AD7731_6_*RESET |
		   | ������+5V��      AD5750_30_*RESET|
		   | PC4-508-A0     : ADG508_1_A0	  |
		   | PE9-508-A1     : ADG508_16_A1	  |
		   | PE7-508-A2     : ADG508_15_A0	  |
		   |  
		   |                                  |
 *          ----------------------------------
 * ��汾  ��ST3.5.0
**********************************************************************************/
#include "ai8do2r.h"
#include "mb.h"
#include "delay.h"
#include <stdarg.h>
//#include "stdio.h"
//#include "stdbool.h"
/*----------------------------------------------------------------
���ƣ�         ģ��������ģʽ
��;��         ģ�������룬��Χ��������ģʽ��
//-----------------------------------------------------------------*/
#define AD7731_MODE_NREGS   16
USHORT  ad7731ModeBuf[AD7731_MODE_NREGS]={0x3074,0x3074,0x3074,0x3074,0x3074,0x3074,0x3074,0x3074};

extern unsigned char ucRegDiscBuf[];      //����������Ĵ���ucRegDiscBuf
extern unsigned char ucRegCoilsBuf[];	 //����������Ĵ���ucRegCoilsBuf
extern USHORT  usRegInputBuf[];	    //����Ĵ���usRegInputBuf
extern USHORT  usRegHoldingBuf[];   //����/����Ĵ���usRegHoldingBuf

/*******************************************************************************
* Function Name  : GPIO_AD5750_Configuration
* Description    : ��ʼ��ai8do2rģ��GPIO
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
* Description    : ��ʼ��ai8do2rģ��GPIO
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
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;   //SPI����Ϊ˫��˫��ȫ˫��
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						   //����Ϊ��SPI
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					   //SPI���ͽ���8λ֡�ṹ
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;						   //ʱ�����յ�
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;						   //���ݲ����ڵڶ���ʱ����
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							   //�ڲ�NSS�ź���SSIλ����
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;   //������Ԥ��ƵֵΪ16 2M
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;				   //���ݴ����MSBλ��ʼ
  SPI_InitStructure.SPI_CRCPolynomial = 7;							   //CRCУ��ֵ
  SPI_Init(SPI1, &SPI_InitStructure);

  /* Enable SPI1  */
  SPI_Cmd(SPI1, ENABLE);
}

void SPI2_DA2602_Init(void)
{
   /* SPI1 configuration */
  SPI_InitTypeDef  SPI_InitStructure;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;   //SPI����Ϊ˫��˫��ȫ˫��
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						   //����Ϊ��SPI
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					   //SPI���ͽ���8λ֡�ṹ
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;						   //ʱ�����յ�
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;						   //���ݲ����ڵڶ���ʱ����
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							   //�ڲ�NSS�ź���SSIλ����
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;  //������Ԥ��ƵֵΪ16 2M
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;				   //���ݴ����MSBλ��ʼ
  SPI_InitStructure.SPI_CRCPolynomial = 7;							   //CRCУ��ֵ
  SPI_Init(SPI1, &SPI_InitStructure);

  /* Enable SPI1  */
  SPI_Cmd(SPI1, ENABLE); 
}
void SPI2_AD5750_Init(void)
{
   /* SPI2 configuration */
  SPI_InitTypeDef  SPI_InitStructure;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;   //SPI����Ϊ˫��˫��ȫ˫��
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						   //����Ϊ��SPI
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;				   //SPI���ͽ���16λ֡�ṹ
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;						   //ʱ�����յ�
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;						   //���ݲ����ڵ�һ��ʱ����
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							   //�ڲ�NSS�ź���SSIλ����
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;  //������Ԥ��ƵֵΪ16 2M
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;				   //���ݴ����MSBλ��ʼ
  SPI_InitStructure.SPI_CRCPolynomial = 7;							   //CRCУ��ֵ
  SPI_Init(SPI1, &SPI_InitStructure);

  /* Enable SPI2  */
  SPI_Cmd(SPI1, ENABLE);
}

/*********************************************************************
 * AD7731 ��״̬�Ĵ���
 * ��������: 
 * λ���ܣ�  | WEN* | ZERO | RW1 | RW0 | ZERO | RS2 | RS1 | RS0 |
 *    bit��  |  7   |   6  |  5  |  4  |   3  |  2  |  1  |  0  |
 *   ���룺  |  0   |   0  |  0  |  1  |   0  |  0  |  0  |  0  | 
 * ����˵����|дʹ��| ��Ϊ0|  ��д���� |��Ϊ0 | ��д�Ĵ���ѡ��  |  
 *	 ���ͣ�0x10
 *  ����������������������������������������������������������������������������������������������������������������������������������������������������
 * ��������: 
 * λ���ܣ�  |  RDY*  | STDY*   | STBY   |NOREF   | MS3  | MS2 | MS1 | MS0 |
 *    bit��  |   7    |   6     |  5     |  4     |   3  |  2  |  1  |  0  |
 *   ���룺  |   1    |   1     |  0     |  0     |   1  |  1  |  0  |  0  | 
 *����˵���� |RDY���� | STDY�ȶ�|����״̬|�ο���ѹ|      ����Ĭ������      |
 *	 ���أ�0xCC		 
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
 * AD7731 ��ģʽ�Ĵ��� ���� 0x12 ���� 0xCC
 * ��������: 
 * λ���ܣ�  | WEN* | ZERO | RW1 | RW0 | ZERO | RS2 | RS1 | RS0 |
 *    bit��  |  7   |   6  |  5  |  4  |   3  |  2  |  1  |  0  |
 *   ���룺  |  0   |   0  |  0  |  1  |   0  |  0  |  1  |  0  | 
 * ����˵����|дʹ��| ��Ϊ0|  ��д���� |��Ϊ0 | ��д�Ĵ���ѡ��  |  
 *	 ���ͣ�0x12
 * ��������������������������������������������������������������������������������������������������������������������������������������������������������������������
 * ��������: 
 * λ���ܣ�  |         MODE Bits         |   B*U     |DOEN   | DO1  | DO0 |  WL    |
 *    bit��  |   15   |   14    |  13    |    12     | 11    |  10  |  9  |  8     |
 *   ���룺  |   0    |   0     |  0     |     0     |  1    |  1   |  0  |  0     | 
 *����˵���� |  ����ת��ģʽ��У׼ģʽ   |������     |DOʹ�� | ���DOEN=1 |���ݳ���|
 *			 |   000 ͬ��ģʽ			 |˫����	 |1=IN3	 | DO1 DO0	  |0=16Bit |
 *			 |	 001 ����ת��ģʽ		 |0=˫���� 	 |	IN4  | ��������� |1=24Bit |
 *			 |	 010 ����ת��ģʽ		 |00 ��������|DO��� |			  |		   |
 *			 |	 011 ����ģʽ			 |10~0V		 |0=IN3	 |			  |		   |
 *			 |	 100 �ڲ����У׼		 |11~��������|	IN4	 |			  |		   |
 *			 |	 101 �ڲ�������У׼		 |1=������	 |AI���� |			  |		   |
 *			 |	 110 ϵͳ���У׼		 |00~ 0V	 |		 |			  |		   |
 *			 |	 111 ϵͳ������У׼		 |11~��������|		 |			  |		   |
 *������������������������������������������������������������������������������������������������������������������������������������������������������������������
 * λ���ܣ�  | HIREF   |      Input Range Bits         |  BO    |In Channel Select |
 *    bit��  |   7     |   6    |  5      |    4       |   3    |  2   |  1  | 0   |
 *   ���룺  |   0     |   0    |  0      |     0      |   1    |  1   |  0  |  0  | 
 *����˵���� |REF����  |  ˫����  ����B*U  ������      |�ջٵ���|����ͨ��ģʽ����  |
 *           |0=2.5VREF|000=-20~+20mV     000=0~+20mV  |1=����  |000=AI1~AI6 α���|
 *			 |1=5V REF |001=-20~+20mV     001=0~+20mV  |0=�ر�	|001=AI2~AI6 α���|
 *			 |	 	   |010=-40~+40mV     000=0~+40mV  | 	    |010=AI3~AI6 α���|
 *			 |	 	   |011=-80~+80mV     000=0~+80mV  |        |011=AI4~AI6 α���|
 *			 |	 	   |100=-160~+160mV   100=0~+160mV |		|100=AI1~AI2 ȫ���|			  		   |
 *			 |		   |101=-320~+320mV   100=0~+320mV |        |101=AI3~AI4 ȫ���|
 *			 |		   |110=-160~+160mV   100=0~+160mV |	    |110=AI5~AI6 ȫ���|		  
 *			 |	 	   |111=-1.28~+1.28mV 100=0~+1.28mV|	    |111=AI4~AI6 ����  |
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
 * AD7731 ���˲��Ĵ��� ���� 0x13 ���� 0x2002
 * ��������: 
 * λ���ܣ�  | WEN* | ZERO | RW1 | RW0 | ZERO | RS2 | RS1 | RS0 |
 *    bit��  |  7   |   6  |  5  |  4  |   3  |  2  |  1  |  0  |
 *   ���룺  |  0   |   0  |  0  |  1  |   0  |  0  |  1  |  1  | 
 * ����˵����|дʹ��| ��Ϊ0|  ��д���� |��Ϊ0 | ��д�Ĵ���ѡ��  |  
 *	 ���ͣ�0x13
 *  ����������������������������������������������������������������������������������������������������������������������������������������������������������������������������������������
 * ��������: 
 * λ���ܣ�  |                 Sinc3 Filter SC                     |ZERO | CHP  |    SKIP| FR0  |                
 *    bit��  | 15 | 14 | 13 | 12 | 11 | 10 | 9 | 8 | 7 | 6 | 5 | 4 |  3  |  2   |     1  |  0   | 
 *   ���룺  |                                                     |  0  |  0   |     1  |  0   | 
 *����˵���� |                                                     |��Ϊ0|Sinc3�˲�ѡ��	 | FAST |
 *			 |                                                     |     |00=150~2.048KHz| ģʽ |
 *			 |                                                     |     |10=50~1.365KHz |0=�ر�|
 *			 |		                                      	       |	 |01=150~7.6KHz	 |1=��|
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
 *AD7731 ��ƫ��У׼�Ĵ�����Ч��㣩 ���� 0x15 ���� 0x80000
 * ��������: 
 * λ���ܣ�  | WEN* | ZERO | RW1 | RW0 | ZERO | RS2 | RS1 | RS0 |
 *    bit��  |  7   |   6  |  5  |  4  |   3  |  2  |  1  |  0  |
 *   ���룺  |  0   |   0  |  0  |  1  |   0  |  1  |  0  |  1  | 
 * ����˵����|дʹ��| ��Ϊ0|  ��д���� |��Ϊ0 | ��д�Ĵ���ѡ��  |  
 *	 ���ͣ�0x15
 *  ������������������������������������������������������������������������������������������������������������������������������������������������
 * ��������: 
 * λ���ܣ�  |                                                             |                
 *    bit��  |23|22|21|20|19|18|17|16|15|14|13|12|11|10|9|8|7|6|5|4|3|2|1|0| 
 *   ���룺  |                                                             | 
 *����˵���� |                                                             |
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
 * AD7731 ������Ĵ�����У�����̣� ���� 0x16 ���� 0x593CEA
 * ��������: 
 * λ���ܣ�  | WEN* | ZERO | RW1 | RW0 | ZERO | RS2 | RS1 | RS0 |
 *    bit��  |  7   |   6  |  5  |  4  |   3  |  2  |  1  |  0  |
 *   ���룺  |  0   |   0  |  0  |  1  |   0  |  1  |  1  |  1  | 
 * ����˵����|дʹ��| ��Ϊ0|  ��д���� |��Ϊ0 | ��д�Ĵ���ѡ��  |  
 *	 ���ͣ�0x15
 *  ����������������������������������������������������������������������������������������������������������������������������������������������
 * ��������: 
 * λ���ܣ�  |                                                             |                
 *    bit��  |23|22|21|20|19|18|17|16|15|14|13|12|11|10|9|8|7|6|5|4|3|2|1|0| 
 *   ���룺  |                                                             | 
 *����˵���� |                                                             |
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
 AD7731 дģʽ�Ĵ��� ���� 0x02 0x1174 ���� 0x593CEA
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
 ADG508ͨ��ѡ��
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
 AD7731 �����ݼĴ��� 
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
LTC2602 д���ݼĴ��������¼Ĵ��� ���� 0x02 0x1174 ���� 0x593CEA
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
*��ȡAD5750�Ĵ������� u16 SPI_ReadAD5750CH_DataReg(u16 usAddress)
 *��������: 
 * λ���ܣ�  |  A2   A1  A0 | R/W*  | D11 | R3 | R2 | R1 | R0 |CLRSEL | OUTEN |RSET   |PECERR|OVERTEMP |IOUTFAULT |VOUTFAULT  |
 *    bit��  | 15 | 14 | 13 |  12   | 11  | 10 | 9  | 8  | 7  |   6   |	5	  |	4	  |	 3	 |	 2	   |  1		  |	   0	  |
 *   ���룺  | 0/1| 0/1| 0/1|  0/1  |  0  | 0/1| 0/1|0/1 |0/1 |	0/1   |0/1	  |0/1    |0/1	 |0/1	   | 0/1	  |	0/1		  |
 * ����˵����|000 ����0��ַ |0д����|��Ϊ0|0000 4-20mA        |0����0 |0�����|0����  |0���� |0�¶�����|0 Iout����|0 Vout���� | 
 * 			 |001 ����1��ַ	|1������|	  |0001 0-20mA        |1������|1���  |1����  |1���� |1����150 |1 Iout��·|1 Vout��· |
 *			 |010 ����2��ַ	|			  |0011 +/-20mA       |ģʽ	  |
 *			 |...........	|			  |..........         |									 
 *	 ��ַ0�������ͣ�0x1000; ��ַ2�������� 0x3000;
 *******************************************************************************************************************************/
u16 SPI_ReadAD5750CH_DataReg(u16 usAddress)
{
    u16 temp;
	temp = SPI_AD5750_SendByte(usAddress);
	return temp;
 
}
/*******************************************************************************************************************************
 *д��AD5750�Ĵ������� u16 SPI_WriteAD5750CH_DataReg(u16 usAdddate)
 *��������: 
 * λ���ܣ�  |  A2   A1  A0 | R/W*  | D11 | R3 | R2 | R1 | R0 |CLRSEL | OUTEN |Clear  |RSET      |Reset |D1   | D0  |
 *    bit��  | 15 | 14 | 13 |  12   | 11  | 10 | 9  | 8  | 7  |   6   |	5	  |	4	  |	 3	     |	 2	| 1	  |	 0  |
 *   ���룺  | 0/1| 0/1| 0/1|  0/1  |  0  | 0/1| 0/1|0/1 |0/1 |	0/1   |0/1	  |0/1    |0/1	     |0/1	| 0	  |	 0	|	 
 * ����˵����|000 ����0��ַ |0д����|��Ϊ0|0000 4-20mA        |0����0 |0�����|0������|0�ⲿ���� |0���� |��Ϊ0|��Ϊ0| 
 * 			 |001 ����1��ַ	|1������|	  |0001 0-20mA        |1������|1���  |1����  |1�ڲ����� |1��λ |   
 *			 |010 ����2��ַ	|			  |0011 +/-20mA       |ģʽ	  |
 *			 |...........	|			  |..........         |									 
 *	 ��ַ0�������ͣ�0x2A0 �ⲿ����0-5V���; ��ַ2�������� 0x22A0 �ⲿ����0-5V���;
 *******************************************************************************************************************************/
u16 SPI_WriteAD5750CH_DataReg(u16 usAdddate)
{
   	u16 temp;
	SPI_AD5750_SendByte(usAdddate);
	temp = usAdddate;
 	return temp;
}
/********************************************************************
 *   ϵͳ���� 													    *
 *	 SPI_I2S_GetFlagStatus	//���ָ����SPI��־λ�������		    *
 *    	SPI_I2S_FLAG_TXE:       //���ͻ���ձ�־λ. 			    *
 *   	SPI_I2S_FLAG_RXNE:      //���ܻ���ǿձ�־λ. 				*
 *  	SPI_I2S_FLAG_BSY:       //æ��־λ. 						*
 *   	SPI_I2S_FLAG_OVR:       //������־λ. 						*
 *	    SPI_FLAG_MODF:          //ģʽ��λ��־λ. 					*
 *  	SPI_FLAG_CRCERR:        //CRC�����־λ. 					*
 *  	I2S_FLAG_UDR:           //Ƿ�ش����־. 					*
 *  	I2S_FLAG_CHSIDE:        //ͨ�����־.					    *
 *	    Return values:			//SET or RESET  					*
 *	 SPI_I2S_SendData           //ͨ������SPIx����һ������ u8��u16	*
 *	 SPI_I2S_ReceiveData        //����ͨ��SPIx������յ����� u8��u16*
 ********************************************************************/	
u8 SPI_AD7731_SendByte(u8 byte)
{								
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);//���SPI2�ķ��ͻ������Ƿ�Ϊ��
		
		 SPI_I2S_SendData(SPI1, byte); //������ͻ�����Ϊ�գ�ͨ��SPI2����һ���ֽ����� a byte
          
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); //�ȴ����ջ��������յ� a byte 
  
  		 return SPI_I2S_ReceiveData(SPI1); //���ؽ��յ������� a byge
}
/********************************************************************
 *   ϵͳ���� 													    *
 *	 SPI_I2S_GetFlagStatus	//���ָ����SPI��־λ�������		    *
 *    	SPI_I2S_FLAG_TXE:       //���ͻ���ձ�־λ. 			    *
 *   	SPI_I2S_FLAG_RXNE:      //���ܻ���ǿձ�־λ. 				*
 *  	SPI_I2S_FLAG_BSY:       //æ��־λ. 						*
 *   	SPI_I2S_FLAG_OVR:       //������־λ. 						*
 *	    SPI_FLAG_MODF:          //ģʽ��λ��־λ. 					*
 *  	SPI_FLAG_CRCERR:        //CRC�����־λ. 					*
 *  	I2S_FLAG_UDR:           //Ƿ�ش����־. 					*
 *  	I2S_FLAG_CHSIDE:        //ͨ�����־.					    *
 *	    Return values:			//SET or RESET  					*
 *	 SPI_I2S_SendData           //ͨ������SPIx����һ������ u8��u16	*
 *	 SPI_I2S_ReceiveData        //����ͨ��SPIx������յ����� u8��u16*
 ********************************************************************/	
u16 SPI_AD5750_SendByte(u16 byte)
{	
    AD5750_SYNC= 0 ;
	nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;
	AD5750_SYNC= 1 ;
    nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;
	AD5750_SYNC= 0 ;
	nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;							
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);//���SPI2�ķ��ͻ������Ƿ�Ϊ��
	SPI_I2S_SendData(SPI1, byte); //������ͻ�����Ϊ�գ�ͨ��SPI2����һ���ֽ����� a byte  
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); //�ȴ����ջ��������յ� a byte 
	nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;
	AD5750_SYNC= 1 ;
	return SPI_I2S_ReceiveData(SPI1); //���ؽ��յ������� a byge
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

