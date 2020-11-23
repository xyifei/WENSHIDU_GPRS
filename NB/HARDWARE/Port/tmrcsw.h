

#include "stm32f10x.h"
#include "system.h"	
/* 开关量输入 SW 宏定义 */
#define SW1_1  PBin(12)
#define SW1_2  PBin(13)
#define SW1_3  PBin(14)
#define SW1_4  PBin(15)
#define SW2_1  PBin(9)
#define SW2_2  PBin(9)
#define SW2_3  PBin(9)
#define SW2_4  PBin(9)
#define SW3_1  PAin(3)
#define SW3_2  PAin(4)
#define SW3_3  PAin(5)
#define SW3_4  PBin(9)
#define SW4_1  PBin(9)

/* 开关量输出 DO 宏定义 */
#define DO_LINK	    PAout(12)
#define DO_FAULT	PAout(12)

void GPIO_Tmrcsw_Configuration(void); //GPIO初始化
void SW1_Readbit_Add(void);  
void SW3_Readbit_Add(void);

