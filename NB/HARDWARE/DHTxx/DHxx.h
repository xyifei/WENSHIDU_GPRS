#ifndef _DHXX__H_
#define _DHXX__H
#include "sys.h"
#define DHT_Out PBout(1)
#define DHT_In  PBin(1)

#define uchar unsigned char 
#define uint  unsigned int

unsigned char StartDHT11(void);
unsigned char StartDHT21(void);
void DHT_PortIN(void);
void DHT_PortOUT(void);
u8 ReadDHT11(void);
u8 ReadDHT21(void);
void com(void);
//**********************************

//**********************************

extern u8 tdata[4];
extern u8 sbuf;
extern u8 check;

#endif
