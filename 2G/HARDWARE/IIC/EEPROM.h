#ifndef __EEPROM_H
#define __EEPROM_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f10x.h"

/*****24Cxx Seriel EEPROM*************************/
#define EEPROM 128
/********
01 -> 24C01;   02 -> 24C02;  04 -> 24C04;   08 -> 24C08;
16 -> 24C16;   32 -> 24C32;  64 -> 24C64;   128 -> 24C128;
256-> 24C256;  512 -> 24C512;  
*/

#if EEPROM==1
   #define PAGE_SIZE           8
   #define EE_SIZE             0x007F
#elif EEPROM==2
   #define PAGE_SIZE           16 
   #define EE_SIZE             0x00FF
#elif EEPROM==4
   #define PAGE_SIZE           16
   #define EE_SIZE             0x01FF
#elif EEPROM==8
   #define PAGE_SIZE           16
   #define EE_SIZE             0x03FF
#elif EEPROM==16
   #define PAGE_SIZE           16
   #define EE_SIZE             0x07FF
#elif EEPROM==32
   #define PAGE_SIZE           32
   #define EE_SIZE             0x0FFF
#elif EEPROM==64
   #define PAGE_SIZE           32
   #define EE_SIZE             0x1FFF
#elif EEPROM==128
   #define PAGE_SIZE           64
   #define EE_SIZE             0x3FFF
#elif EEPROM==256
   #define PAGE_SIZE           64
   #define EE_SIZE             0x7FFF
#elif EEPROM==512
   #define PAGE_SIZE           128
   #define EE_SIZE             0xFFFF
#endif 


#define WriteAddress_EEPROM    0xA0
#define ReadAddress_EEPROM     0xA1

#define EE_TimeOut             10000


#define EE_ErrorWTimeOut       0x01
#define EE_ErrorRTimeOut       0x10

   extern u8 EE_State;

/**********************************************************************
------------------------define the EEPROM Address --------------------
**********************************************************************/





/***********************************************************************
-------------------------function define   ----------------------------
***********************************************************************/


   unsigned char EEPROM_Write(unsigned char* pBuffer,unsigned int WriteAddress,unsigned char NumbyteToWrite);
   unsigned char EEPROM_Read(unsigned char* pBuffer,unsigned int ReadAddress,unsigned char NumbyteToWrite);
//   void IIC_ReadBuffer(u8*,u16,u8);







#ifdef __cplusplus
}
#endif

#endif
