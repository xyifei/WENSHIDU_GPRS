#include "EEPROM.h"


volatile u32 TimeOut=EE_TimeOut;
u8 EE_State=0x00;

static   void IIC_ReadBuffer(unsigned char* pBuffer,unsigned int ReadAddress,unsigned char NumbyteToWrite);
static   void IIC_WriteBuffer(unsigned char* pBuffer,unsigned int WriteAddress,unsigned char NumByteToWrite);
static   u8 *IIC_ReadPage(u8 *,u8,u8,u8);
static   u8 *IIC_WritePage(u8 *,u8,u16,u8);


unsigned char EEPROM_Write(unsigned char* pBuffer,unsigned int WriteAddress,unsigned char NumbyteToWrite)
{
  unsigned char TempBuffer,Temp2Buffer;
  if((WriteAddress+NumbyteToWrite) > EE_SIZE) 
    {
      return 0;
    }
  else
    {
      IIC_WriteBuffer(pBuffer,WriteAddress,NumbyteToWrite);
      IIC_WriteBuffer(pBuffer,WriteAddress,NumbyteToWrite);

      IIC_ReadBuffer(&TempBuffer,WriteAddress+NumbyteToWrite-1,1);
      Temp2Buffer=*(pBuffer+NumbyteToWrite-1);
      if(TempBuffer==Temp2Buffer)
         return 1;
      else
	return 0;
    }
}

unsigned char EEPROM_Read(unsigned char* pBuffer,unsigned int ReadAddress,unsigned char NumbyteToWrite) 
{
  if((ReadAddress+NumbyteToWrite) > EE_SIZE) 
    {
      return 0;
    }
  else
    {
      IIC_ReadBuffer(pBuffer,ReadAddress,NumbyteToWrite);
      IIC_ReadBuffer(pBuffer,ReadAddress,NumbyteToWrite);
      return 1;
    }
}


/**
***************************************************
 *@Function     :IIC_ReadBuffer()
 *@Return_Value :Null
 *@Peremater    :pBuffer:
                 ReadAddr:
                 NumByteToWrite:
 *@Brief        :
***************************************************/  
void IIC_ReadBuffer(unsigned char* pBuffer,unsigned int ReadAddress,unsigned char NumbyteToWrite)
{
//  u8 NumOfPage=0,NumOfSingle=0;count=0 Part=0,
  u8 PageAddress=0;
  /******pageAddress is over 8bit***********/
#if EEPROM < 32
  PageAddress=(u8)(ReadAddress>>7)&0x0E|ReadAddress_EEPROM;
#else
  PageAddress=WriteAddress_EEPROM;
#endif 

 IIC_ReadPage(pBuffer,PageAddress,ReadAddress,NumbyteToWrite); 
}



/**
***************************************************
 *@Function     :IIC_WriteBuffer()
 *@Return_Value :Null
 *@Peremater    :pBuffer:
                 WriteAddr:
                 NumByteToWrite:
 *@Brief        :
***************************************************/
void IIC_WriteBuffer(unsigned char* pBuffer,unsigned int WriteAddress,unsigned char NumByteToWrite)
{
  u8 NumOfPage=0,NumOfSingle=0; //,count=0
  u16 Part=0;//
  u8 PageAddress=0;
  /******pageAddress is over 8bit***********/
#if EEPROM < 32
  PageAddress=(u8)(WriteAddress>>7)&0x0E|WriteAddress_EEPROM;
#else
  PageAddress=WriteAddress_EEPROM;
#endif 
  


  /*******判断起始地址是否跨页片区******/
  Part=WriteAddress/PAGE_SIZE;
  if(Part!=0)
    {
      Part=PAGE_SIZE*(Part+1)-WriteAddress;      
    }
  else 
    {
      Part=PAGE_SIZE-WriteAddress;
    }
  /******/
  if(Part >= NumByteToWrite)
    {
      IIC_WritePage(pBuffer,PageAddress,WriteAddress,NumByteToWrite);
    }
  else
    {
      NumOfPage = (NumByteToWrite-Part)/PAGE_SIZE;  
      NumOfSingle = (NumByteToWrite-Part)%PAGE_SIZE;   
      pBuffer = IIC_WritePage(pBuffer,PageAddress,WriteAddress,Part);
    
      NumByteToWrite -= Part;
      WriteAddress += Part;

      while(NumOfPage--)
	{     
	  pBuffer = IIC_WritePage(pBuffer,PageAddress,WriteAddress,PAGE_SIZE);

	  WriteAddress += PAGE_SIZE;
	  //  pBuffer += PAGE_SIZE;
	}   
      if(NumOfSingle!=0)
	{
	  IIC_WritePage(pBuffer,PageAddress,WriteAddress,NumOfSingle);
	 
	}
    }
}



/**
***************************************************
 *@Function     :IIC_WritePage(u8 *pBuffer,u8 PartAddr,u8 WriteAddr,u8 NumByteToWrite)
 *@Return_Value :Null
 *@Peremater    :pBuffer:
                 PartAddr:
                 WriteAddr:
                 NumByteToWrite:
 *@Brief        :
***************************************************/
u8* IIC_WritePage(u8 *pBuffer,u8 PartAddr,u16 WriteAddr,u8 NumByteToWrite)
{
  /*send Start condition,test on EV5 and clear it*/
  TimeOut=EE_TimeOut;
  I2C_GenerateSTART(I2C1,ENABLE);
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT))
    {
      if((TimeOut--)==0){EE_State=EE_ErrorWTimeOut;}    
    }

  /*send Part Address for write,test on EV6 and clear it*/
  TimeOut=EE_TimeOut;
  I2C_Send7bitAddress(I2C1,PartAddr,I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
      if((TimeOut--)==0){EE_State=EE_ErrorWTimeOut;}
    }

  /******writeAddress for 8bit or 16 bit**********/
  TimeOut=EE_TimeOut;
#if (EEPROM<32)
 /**send the EEPROM's internal address,test on EV8 and clear it**/
  I2C_SendData(I2C1,(u8)WriteAddr);
#else
  I2C_SendData(I2C1,(u8)((WriteAddr&0xFF00)>>8));
    while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
      if((TimeOut--)==0){EE_State=EE_ErrorWTimeOut;}    
    }
    TimeOut=EE_TimeOut;
    I2C_SendData(I2C1,(u8)(WriteAddr&0x00FF)); 
#endif   
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
      if((TimeOut--)==0){EE_State=EE_ErrorWTimeOut;}
    }

  /* DMA control */
  /*send the bufferData and buffer size*/
  while(NumByteToWrite--)
    {
      I2C_SendData(I2C1,*pBuffer);
      TimeOut=EE_TimeOut;
      while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
      {
    	if((TimeOut--)==0){EE_State=EE_ErrorRTimeOut;}
      }
      pBuffer++;
    }
  
  /*send STOP condition*/
  I2C_GenerateSTOP(I2C1,ENABLE);
   delay_ms(5);
  return pBuffer;
}


/**
*************************************************
 *@Function     : IIC_ReadPage(u8 *pBuffer,u8 PartAddr,u8 WriteAddr,u8 NumByteToWrite)
 *@Return_Value :Null
 *@Peremater    :
 *@Brief        :
***************************************************/
u8* IIC_ReadPage(u8 *pBuffer,u8 PartAddr,u8 WriteAddr,u8 NumByteToWrite)
{
//  I2C_ClearFlag(I2C1, I2C_FLAG_AF);
  I2C_AcknowledgeConfig(I2C1,ENABLE);
  /*send Start condition,test on EV5 and clear it*/
  TimeOut=EE_TimeOut;
  I2C_GenerateSTART(I2C1,ENABLE);
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT))
    {
      if((TimeOut--)==0){EE_State=EE_ErrorRTimeOut;}    
    }

  /*send Part Address for write,test on EV6 and clear it*/
  TimeOut=EE_TimeOut;
  I2C_Send7bitAddress(I2C1,PartAddr,I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
      if((TimeOut--)==0){EE_State=EE_ErrorRTimeOut;}
    }

  /*send the EEPROM's internal address,test on EV8 and clear it
  I2C_SendData(I2C1,WriteAddr);
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));  */
   /******writeAddress for 8bit or 16 bit**********/
  TimeOut=EE_TimeOut;
#if (EEPROM<32)
 /**send the EEPROM's internal address,test on EV8 and clear it**/
  I2C_SendData(I2C1,(u8)WriteAddr);
#else
  I2C_SendData(I2C1,(u8)((WriteAddr&0xFF00)>>8));
    while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
      if((TimeOut--)==0){EE_State=EE_ErrorRTimeOut;}
    }

    TimeOut=EE_TimeOut;
    I2C_SendData(I2C1,(u8)(WriteAddr&0x00FF)); 
#endif

  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
      if((TimeOut--)==0){EE_State=EE_ErrorRTimeOut;}
    }


  /* DMA control */
  /*send Start condition,test on EV5 and clear it*/
  TimeOut=EE_TimeOut;
  I2C_GenerateSTART(I2C1,ENABLE);
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT))
    {
      if((TimeOut--)==0){EE_State=EE_ErrorRTimeOut;}    
    }

  /*send Part Address for write,test on EV6 and clear it*/
  TimeOut=EE_TimeOut;
  I2C_Send7bitAddress(I2C1,PartAddr,I2C_Direction_Receiver);
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    {
      if((TimeOut--)==0){EE_State=EE_ErrorRTimeOut;}
    }

  while(NumByteToWrite)
    {
      if(NumByteToWrite==1)
    	{
    	  I2C_AcknowledgeConfig(I2C1,DISABLE);
    	  I2C_GenerateSTOP(I2C1,ENABLE);
    	}
      if(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED))
    	{
    	  *pBuffer=I2C_ReceiveData(I2C1);
    	  pBuffer++;
    	  NumByteToWrite--;
	     TimeOut=EE_TimeOut;
    	}
      else
    	{
    	    if((TimeOut--)==0){EE_State=EE_ErrorRTimeOut;}  
    	}
    
     }
 I2C_GenerateSTOP(I2C1,ENABLE);
 return pBuffer;
    }

