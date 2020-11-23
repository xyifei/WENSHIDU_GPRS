#ifndef __di8do8l_H
#define __di8do8l_H

#include "stm32f10x.h"
#include "system.h"	
/* ���������� DI �궨�� */
#define DI_DI0  PCin(8)
#define DI_DI1  PCin(6)
#define DI_DI2  PBin(14)
#define DI_DI3  PAin(3)
#define DI_DI4  PAin(11)
#define DI_DI5  PAin(11)
#define DI_DI6  PAin(11)
#define DI_DI7  PAin(11)
/* ��������� DO �궨�� */
#define DO_DO0	PAout(4)
#define DO_DO1	PAout(5)
#define DO_DO2	PAout(6)
#define DO_DO3	PAout(7)
#define DO_DO4	PAout(12)
#define DO_DO5	PAout(12)
#define DO_DO6	PAout(12)
#define DO_DO7	PAout(12)


void GPIO_Di8do8l_Configuration(void); //GPIO��ʼ��
void DI8_Readbit_Modbus(void); //��ȡGPIOֵ��ֵ��modbus�Ĵ���
void DO8_Writebit_Modbus(void);// ��ȡmodbus�Ĵ�����ֵ��ֵ��GPIO
#endif /* __di8do8l_H */
