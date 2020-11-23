#ifndef __ai8do2r_H
#define __ai8do2r_H

#include "stm32f10x.h"
#include "system.h"
#define nop __nop()  //ø’÷∏¡Ó

#define SPI_AD7731_CS_LOW()       GPIO_ResetBits(GPIOA, GPIO_Pin_1)
#define SPI_AD7731_CS_HIGH()      GPIO_SetBits(GPIOA, GPIO_Pin_1)
#define SPI_DA2602_CS_LOW()       GPIO_ResetBits(GPIOE, GPIO_Pin_11)
#define SPI_DA2602_CS_HIGH()      GPIO_SetBits(GPIOE, GPIO_Pin_11)
#define AD_ADG508_A0_LOW()        GPIO_ResetBits(GPIOC, GPIO_Pin_4)
#define AD_ADG508_A0_HIGH()       GPIO_SetBits(GPIOC, GPIO_Pin_4)
#define AD_ADG508_A1_LOW()        GPIO_ResetBits(GPIOE, GPIO_Pin_9)
#define AD_ADG508_A1_HIGH()       GPIO_SetBits(GPIOE, GPIO_Pin_9)
#define AD_ADG508_A2_LOW()        GPIO_ResetBits(GPIOE, GPIO_Pin_7)
#define AD_ADG508_A2_HIGH()       GPIO_SetBits(GPIOE, GPIO_Pin_7)
#define AD_DA_RESET_LOW()         GPIO_ResetBits(GPIOA, GPIO_Pin_4)
#define AD_DA_RESET_HIGH()        GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define DA_5750_SYNC_LOW()        GPIO_ResetBits(GPIOE, GPIO_Pin_13)
#define DA_5750_SYNC_HIGH()       GPIO_SetBits(GPIOE, GPIO_Pin_13)
#define SPI_AD7731_CS             PAout(1)
#define SPI_DA2602_CS             PEout(11)
#define AD_ADG508_A0              PCout(4)
#define AD_ADG508_A1              PEout(9)
#define AD_ADG508_A2              PEout(7)
#define AD_DA_RESET               PAout(4)
#define AD5750_SYNC	              PEout(13)
#define AD7731_RDY	              PAin(0)
#define AD5750_SCK	              PAout(5)
#define AD5750_MOSI	              PAout(7)
#define AD5750_MISO	              PAin(6)

void GPIO_ai8do2r_Configuration(void);
void SPI2_AD7731_Init(void);
void SPI2_DA2602_Init(void);
void SPI2_AD5750_Init(void);
u8 SPI_AD7731_SendByte(u8 byte);
u16 SPI_AD5750_SendByte(u16 byte);
void SPI_ReadAD7731_StatusReg(void);
void SPI_ReadAD7731_ModeReg(void);
void SPI_ReadAD7731_FilterReg(void);
void SPI_ReadAD7731_OffsetReg(void);
void SPI_ReadAD7731_GainReg(void);
void SPI_WriteAD7731_ModeReg(u16 adMode);
void SPI_WriteDA2602_DataReg(void);
u16 SPI_ReadAD5750CH_DataReg(u16 usAddress);
u16 SPI_WriteAD5750CH_DataReg(u16 usAdddate);
void GPIO_AD5750_Configuration(void);
u16 AD5750_SendByte(u16 usAdddate);
u16 AD5750_ReadByte(u16 usAdddate);
void ADG508_ChannelSet(u8 usChannel);
void SPI_ReadAD7731_DataReg(void);


//void SPI_FLASH_Init(void);
//void SPI_FLASH_SectorErase(u32 SectorAddr);
//void SPI_FLASH_BulkErase(void);
//void SPI_FLASH_PageWrite(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite);
//void SPI_FLASH_BufferWrite(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite);
//void SPI_FLASH_BufferRead(u8* pBuffer, u32 ReadAddr, u16 NumByteToRead);
//u32 SPI_FLASH_ReadID(void);
//u32 SPI_FLASH_ReadDeviceID(void);
//void SPI_FLASH_StartReadSequence(u32 ReadAddr);
//void SPI_Flash_PowerDown(void);
//void SPI_Flash_WAKEUP(void);
//
//
//u8 SPI_FLASH_ReadByte(void);
//u8 SPI_FLASH_SendByte(u8 byte);
//u16 SPI_FLASH_SendHalfWord(u16 HalfWord);
//void SPI_FLASH_WriteEnable(void);
//void SPI_FLASH_WaitForWriteEnd(void);

#endif /* __ai8do2r_H */

