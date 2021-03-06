#ifndef __I2C_EE_H
#define	__I2C_EE_H

#include "stm32f10x.h"

/* EEPROM Addresses defines */
#define EEPROM_Block0_ADDRESS 0x48   /* E2 = 0 */
//#define EEPROM_Block0_ADDRESS 0xA0   /* E2 = 0 */
//#define EEPROM_Block1_ADDRESS 0xA2 /* E2 = 0 */
//#define EEPROM_Block2_ADDRESS 0xA4 /* E2 = 0 */
//#define EEPROM_Block3_ADDRESS 0xA6 /* E2 = 0 */
#define I2C_ADC_A1_LOW()       GPIO_ResetBits(GPIOB, GPIO_Pin_8)
#define I2C_ADC_A1_HIGH()      GPIO_SetBits(GPIOB, GPIO_Pin_8)
#define I2C_ADC_A2_LOW()       GPIO_ResetBits(GPIOB, GPIO_Pin_9)
#define I2C_ADC_A2_HIGH()      GPIO_SetBits(GPIOB, GPIO_Pin_9)
#define I2C_ADC_A3_LOW()       GPIO_ResetBits(GPIOE, GPIO_Pin_0)
#define I2C_ADC_A3_HIGH()      GPIO_SetBits(GPIOE, GPIO_Pin_0)
#define I2C_ADC_A4_LOW()       GPIO_ResetBits(GPIOE, GPIO_Pin_1)
#define I2C_ADC_A4_HIGH()      GPIO_SetBits(GPIOE, GPIO_Pin_1)

void I2C_EE_Init(void);
void I2C_EE_BufferWrite(u8* pBuffer, u8 WriteAddr, u16 NumByteToWrite);
void I2C_EE_ByteWrite(u8* pBuffer, u8 WriteAddr);
void I2C_EE_PageWrite(u8* pBuffer, u8 WriteAddr, u8 NumByteToWrite);
void I2C_EE_BufferRead(u8* pBuffer, u8 ReadAddr, u16 NumByteToRead);
void I2C_EE_WaitEepromStandbyState(void);

#endif /* __I2C_EE_H */
