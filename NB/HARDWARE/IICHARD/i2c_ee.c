/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : i2c_ee.c
* Author             : MCD Application Team
* Version            : V1.0
* Date               : 10/08/2007
* Description        : This file provides a set of functions needed to manage the
*                      communication between I2C peripheral and I2C M24C08 EEPROM.
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "i2c_ee.h"
#include "testchip.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define I2C_Speed              200000
#define I2C1_SLAVE_ADDRESS7    0xA0
#define I2C_PageSize           256


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
u16 EEPROM_ADDRESS;
//1.9 add by robin
extern u8 state_first,state_second,state_third;
extern u8 hand_position;
extern u8 BCD2eeprom[30];
extern u8 eeprom2BCD[30];
extern u32 page2_num_BCD[12];
extern u32 page3_num_BCD[12];
extern void correct_page2_num(u8 position,u32 value);
extern void show_num_init_page2(void);
//1.9 add by robin
/* Private function prototypes -----------------------------------------------*/

void I2C_Configuration(void);



/*******************************************************************************
* Function Name  : I2C_Configuration
* Description    : I2C Configuration
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_Configuration(void)
{
  I2C_InitTypeDef  I2C_InitStructure; 

  /* I2C configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = I2C1_SLAVE_ADDRESS7;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2C_Speed;
  
  /* I2C Peripheral Enable */
  I2C_Cmd(I2C1, ENABLE);
  /* Apply I2C configuration after enabling it */
  I2C_Init(I2C1, &I2C_InitStructure);
}

/*******************************************************************************
* Function Name  : I2C_EE_Init
* Description    : Initializes peripherals used by the I2C EEPROM driver.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_EE_Init()
{ 
  /****gpio统一到GPIO函数里面****/
  /* I2C configuration */
  I2C_Configuration(); 
}

/*******************************************************************************
* Function Name  : I2C_EE_BufferWrite
* Description    : Writes buffer of data to the I2C EEPROM.
* Input          : - pBuffer : pointer to the buffer  containing the data to be 
*                    written to the EEPROM.
*                  - WriteAddr : EEPROM's internal address to write to.
*                  - NumByteToWrite : number of bytes to write to the EEPROM.
* Output         : None
* Return         : None(注意这个函数有问题，写入的地址信息不对)
*******************************************************************************/
void I2C_EE_BufferWrite(u8* pBuffer, u16 WriteAddr, u16 NumByteToWrite)
{
  u8 NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0;

  Addr = WriteAddr % I2C_PageSize;
  count = I2C_PageSize - Addr;
  NumOfPage =  NumByteToWrite / I2C_PageSize;
  NumOfSingle = NumByteToWrite % I2C_PageSize;
 
  /* If WriteAddr is I2C_PageSize aligned  */
  if(Addr == 0) 
  {
    /* If NumByteToWrite < I2C_PageSize */
    if(NumOfPage == 0) 
    {
      I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
      I2C_EE_WaitEepromStandbyState();
    }
    /* If NumByteToWrite > I2C_PageSize */
    else  
    {
      while(NumOfPage--)
      {
        I2C_EE_PageWrite(pBuffer, WriteAddr, I2C_PageSize); 
    	I2C_EE_WaitEepromStandbyState();
        WriteAddr +=  I2C_PageSize;
        pBuffer += I2C_PageSize;
      }

      if(NumOfSingle!=0)
      {
        I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
        I2C_EE_WaitEepromStandbyState();
      }
    }
  }
  /* If WriteAddr is not I2C_PageSize aligned  */
  else 
  {
    /* If NumByteToWrite < I2C_PageSize */
    if(NumOfPage== 0) 
    {
      I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
      I2C_EE_WaitEepromStandbyState();
    }
    /* If NumByteToWrite > I2C_PageSize */
    else
    {
      NumByteToWrite -= count;
      NumOfPage =  NumByteToWrite / I2C_PageSize;
      NumOfSingle = NumByteToWrite % I2C_PageSize;	
      
      if(count != 0)
      {  
        I2C_EE_PageWrite(pBuffer, WriteAddr, count);
        I2C_EE_WaitEepromStandbyState();
        WriteAddr += count;
        pBuffer += count;
      } 
      
      while(NumOfPage--)
      {
        I2C_EE_PageWrite(pBuffer, WriteAddr, I2C_PageSize);
        I2C_EE_WaitEepromStandbyState();
        WriteAddr +=  I2C_PageSize;
        pBuffer += I2C_PageSize;  
      }
      if(NumOfSingle != 0)
      {
        I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle); 
        I2C_EE_WaitEepromStandbyState();
      }
    }
  }  
}

/*******************************************************************************
* Function Name  : I2C_EE_ByteWrite
* Description    : Writes one byte to the I2C EEPROM.
* Input          : - pBuffer : pointer to the buffer  containing the data to be 
*                    written to the EEPROM.
*                  - WriteAddr : EEPROM's internal address to write to.
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_EE_ByteWrite(u8* pBuffer, u16 WriteAddr)
{
  I2C_EE_WaitEepromStandbyState(); //D+
  /* Send STRAT condition */
  I2C_GenerateSTART(I2C1, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));  

  /* Send EEPROM address for write */
  I2C_Send7bitAddress(I2C1, EEPROM_ADDRESS, I2C_Direction_Transmitter);
  
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
      
  /* Send the EEPROM's internal address to write to */
  I2C_SendData(I2C1, (WriteAddr&0xff00)>>8); //D+ 
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
  /* Send the EEPROM's internal address to write to */
  I2C_SendData(I2C1, WriteAddr&0xff); //D+ 
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send the byte to be written */
  I2C_SendData(I2C1, *pBuffer); 
   
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
  /* Send STOP condition */
  I2C_GenerateSTOP(I2C1, ENABLE);
}
/*******************************************************************************
* Function Name  : I2C_EE_PageWrite
* Description    : Writes more than one byte to the EEPROM with a single WRITE
*                  cycle. The number of byte can't exceed the EEPROM page size.
* Input          : - pBuffer : pointer to the buffer containing the data to be 
*                    written to the EEPROM.
*                  - WriteAddr : EEPROM's internal address to write to.
*                  - NumByteToWrite : number of bytes to write to the EEPROM.
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_EE_PageWrite(u8* pBuffer, u16 WriteAddr, u16 NumByteToWrite)
{
  I2C_EE_WaitEepromStandbyState(); //D+
  /* Send START condition */
  I2C_GenerateSTART(I2C1, ENABLE);
  
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)); 
  
  /* Send EEPROM address for write */
  I2C_Send7bitAddress(I2C1, EEPROM_ADDRESS, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));  

  /* Send the EEPROM's internal address to write to */    
  I2C_SendData(I2C1, (WriteAddr&0xff00)>>8); //D+ 
  /* Test on EV8 and clear it */
  while(! I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
  /* Send the EEPROM's internal address to write to */
  I2C_SendData(I2C1, WriteAddr&0xff); //D+ 
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  

  /* While there is data to be written */
  while(NumByteToWrite--)  
  {
    /* Send the current byte */
    I2C_SendData(I2C1, *pBuffer); 

    /* Point to the next byte to be written */
    pBuffer++; 
  
    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  }

  /* Send STOP condition */
  I2C_GenerateSTOP(I2C1, ENABLE);
}

/*******************************************************************************
* Function Name  : I2C_EE_BufferRead
* Description    : Reads a block of data from the EEPROM.
* Input          : - pBuffer : pointer to the buffer that receives the data read 
*                    from the EEPROM.
*                  - ReadAddr : EEPROM's internal address to read from.
*                  - NumByteToRead : number of bytes to read from the EEPROM.
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_EE_BufferRead(u8* pBuffer, u16 ReadAddr, u16 NumByteToRead)
{  
  I2C_EE_WaitEepromStandbyState(); //D+
	//while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)); // Added by Najoua 27/08/2008
  /* Send START condition */
  I2C_GenerateSTART(I2C1, ENABLE);
  
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
   
  /* In the case of a single data transfer disable ACK before reading the data D+*/
  if(NumByteToRead==1) 
  {
    I2C_AcknowledgeConfig(I2C1, DISABLE);
  }
 
  /* Send EEPROM address for write */
  I2C_Send7bitAddress(I2C1, EEPROM_ADDRESS, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  
  /* Clear EV6 by setting again the PE bit */
  I2C_Cmd(I2C1, ENABLE);

  /* Send the EEPROM's internal address to write to */
  I2C_SendData(I2C1, (ReadAddr&0xff00)>>8);  
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
  /* Send the EEPROM's internal address to write to */
  I2C_SendData(I2C1, ReadAddr&0xff);  
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
  /* Send STRAT condition a second time */  
  I2C_GenerateSTART(I2C1, ENABLE);
  
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  
  /* Send EEPROM address for read */
  I2C_Send7bitAddress(I2C1, EEPROM_ADDRESS, I2C_Direction_Receiver);
  
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
  
  /* While there is data to be read */
  while(NumByteToRead)  
  {
//    if(NumByteToRead == 1)
//    {
      /* Disable Acknowledgement */
//      I2C_AcknowledgeConfig(I2C1, DISABLE);
      
      /* Send STOP Condition */
//      I2C_GenerateSTOP(I2C1, ENABLE);
//    }

    /* Test on EV7 and clear it */
    if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))  
    {      
//-------------------------------------------------
      if(NumByteToRead == 2)    //D+
      {
        /* Disable Acknowledgement */
        I2C_AcknowledgeConfig(I2C1, DISABLE);
      }     
      if(NumByteToRead == 1)
      {
        /* Send STOP Condition */
        I2C_GenerateSTOP(I2C1, ENABLE);
      }
//-------------------------------------------------
      
      /* Read a byte from the EEPROM */
      *pBuffer = I2C_ReceiveData(I2C1);

      /* Point to the next location where the byte read will be saved */
      pBuffer++; 
      
      /* Decrement the read bytes counter */
      NumByteToRead--;        
    }   
  }

  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(I2C1, ENABLE);
}
/*******************************************************************************
* Function Name  : I2C_EE_WaitEepromStandbyState
* Description    : Wait for EEPROM Standby state
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_EE_WaitEepromStandbyState(void)      
{
  vu16 SR1_Tmp = 0;

  do
  {
    /* Send START condition */
    I2C_GenerateSTART(I2C1, ENABLE);
    /* Read I2C1 SR1 register */
    SR1_Tmp = I2C_ReadRegister(I2C1, I2C_Register_SR1);
    /* Send EEPROM address for write */
    I2C_Send7bitAddress(I2C1, EEPROM_ADDRESS, I2C_Direction_Transmitter);
  }while(!(I2C_ReadRegister(I2C1, I2C_Register_SR1) & 0x0002));
  
  /* Clear AF flag */
  I2C_ClearFlag(I2C1, I2C_FLAG_AF);
	   
 
}
/*******************************************************************************
* Function Name  : pagenum2store
* Description    : store the data to eeprom()
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  pagenum2store(void)
{
  for(u8 i=0;i<12;i++)
  {    
    BCD2eeprom[2*i]=page2_num_BCD[i]%256;
    BCD2eeprom[2*i+1]=page2_num_BCD[i]/256;
  }
  I2C_EE_BufferWrite(BCD2eeprom, 0x00, 24); 
  I2C_EE_WaitEepromStandbyState();
}
/*******************************************************************************
* Function Name  : store2pagenum
* Description    : load the data to pagenum2
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void   store2pagenum(void)
{
  I2C_EE_BufferRead(eeprom2BCD, 0x00, 24);
  
  I2C_EE_WaitEepromStandbyState();
  for(u8 i=0;i<12;i++)
  {
    page2_num_BCD[i]=eeprom2BCD[2*i]+eeprom2BCD[2*i+1]*256;
    correct_page2_num(i,page2_num_BCD[i]);
  }
  show_num_init_page2();
  if(hand_position<6)
    {
      display_hand(1,(3+hand_position*2));          //关于小手
      close_open_cursor(12,(3+hand_position*2),1);  //打开光标，
    }
  else
    {
      display_hand(20,(3+(hand_position-6)*2));
      close_open_cursor(31,(3+(hand_position-6)*2),1);
    } 
  
  
}
/*******************************************************************************
* Function Name  : pagenum2store
* Description    : store the data to eeprom()
* Input          : address_read_write
* Output         : None
* Return         : None
*******************************************************************************/
void  pagenum2eeprom(u32 address_read_write)
{
	if(address_read_write>65536)
	{
  	EEPROM_ADDRESS=0xA2;
  	address_read_write=address_read_write-98838;  
	}
	else
	{
  	EEPROM_ADDRESS=0xA0;
  	address_read_write=address_read_write;
	}
	for(u8 i=0;i<12;i++)
  {    
    BCD2eeprom[2*i]=page2_num_BCD[i]%256;
    BCD2eeprom[2*i+1]=page2_num_BCD[i]/256;
  }
  //添加第三页面数据存储，文件系统后独立子程序
  BCD2eeprom[24]=page3_num_BCD[10]%256;
  BCD2eeprom[25]=page3_num_BCD[10]/256;
  BCD2eeprom[26]=page3_num_BCD[11]%256;
  BCD2eeprom[27]=page3_num_BCD[11]/256;
	BCD2eeprom[28]=1*state_first+2*state_second+4*state_third;
	I2C_EE_PageWrite(BCD2eeprom, address_read_write, 30);
}
/*******************************************************************************
* Function Name  : eeprom2pagenum
* Description    : load the data of eeprom to pagenum2
* Input          : address_read_write
* Output         : None
* Return         : None
*******************************************************************************/
void   eeprom2pagenum(u32 address_read_write)
{
	u32 temp1,temp2;
	if(address_read_write>65536)
	{
  	EEPROM_ADDRESS=0xA2;
  	address_read_write=address_read_write-98838;  
	}
	else
	{
  	EEPROM_ADDRESS=0xA0;
  	address_read_write=address_read_write;
	}
	I2C_EE_BufferRead(eeprom2BCD, address_read_write, 30);
	temp1=page2_num_BCD[0];
  temp2=page2_num_BCD[1];
  if((temp1==0x00)&&(temp2==0x00))
  {
    for(u8 i=0;i<12;i++)
    {
      page2_num_BCD[i]=eeprom2BCD[2*i]+eeprom2BCD[2*i+1]*256;
			/******3.28*防止eeprom在未写入时读出0xff,如果读出纠正成5*****/
			if(page2_num_BCD[0]%5!=0)
				page2_num_BCD[0]=5;
			if(page2_num_BCD[1]%100!=0)
				page2_num_BCD[1]=1000;
			/******3.28******/
      correct_page2_num(i,page2_num_BCD[i]);
    }
  }
  else
  {
    for(u8 i=2;i<12;i++)
    {
      page2_num_BCD[i]=eeprom2BCD[2*i]+eeprom2BCD[2*i+1]*256;
      correct_page2_num(i,page2_num_BCD[i]);
    } 
    page2_num_BCD[0]=temp1;
		if(page2_num_BCD[0]%5!=0)
				page2_num_BCD[0]=5;
		correct_page2_num(0,page2_num_BCD[0]);  
    page2_num_BCD[1]=temp2;
		if(page2_num_BCD[1]%100!=0)
				page2_num_BCD[1]=1000;
		correct_page2_num(1,page2_num_BCD[1]);
  }
	
	
	
	
	
  //添加2.25
  if(page3_num_BCD[10]<=eeprom2BCD[25]*256+eeprom2BCD[24])  //防止将最新的值还原了，其实不加也可以，好像已有新数据就存储了
    page3_num_BCD[10]=eeprom2BCD[25]*256+eeprom2BCD[24];
  if(page3_num_BCD[11]<=eeprom2BCD[27]*256+eeprom2BCD[26])
    page3_num_BCD[11]=eeprom2BCD[27]*256+eeprom2BCD[26];
  //添加2.25
	//5.6
	state_third=eeprom2BCD[28]/4%2;
	state_second=eeprom2BCD[28]%4/2;
	state_first=eeprom2BCD[28]%4%2;
	//5.6
  show_num_init_page2();
  if(hand_position<6)
    {
      display_hand(1,(3+hand_position*2));          //关于小手
      close_open_cursor(12,(3+hand_position*2),1);  //打开光标，
    }
  else
    {
      display_hand(20,(3+(hand_position-6)*2));
      close_open_cursor(31,(3+(hand_position-6)*2),1);
    } 
  
  

}
/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
