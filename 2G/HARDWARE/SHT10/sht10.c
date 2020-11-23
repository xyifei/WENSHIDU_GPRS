/*************************************************************
                             \(^o^)/
  Copyright (C), 2013-2020, ZheJiang University of Technology
  File name  : SHT10.c 
  Author     : ziye334    
  Version    : V1.0 
  Data       : 2014/3/10      
  Description: Digital temperature and humidity sensor driver code
  
*************************************************************/
#include "SHT10.h"
#include <math.h>


/*************************************************************
  Function   ��SHT10_Dly  
  Description��SHT10ʱ����Ҫ����ʱ
  Input      : none        
  return     : none    
*************************************************************/
void SHT10_Dly(void)
{
        u16 i;
        for(i = 500; i > 0; i--);
}


/*************************************************************
  Function   ��SHT10_Config  
  Description����ʼ�� SHT10����
  Input      : none        
  return     : none    
*************************************************************/
void SHT10_Config(void)
{
        GPIO_InitTypeDef GPIO_InitStructure;        
        //��ʼ��SHT10����ʱ��
        RCC_APB2PeriphClockCmd(SHT10_AHB2_CLK ,ENABLE);
                
        // DATA �������        
        GPIO_InitStructure.GPIO_Pin = SHT10_DATA_PIN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(SHT10_DATA_PORT, &GPIO_InitStructure);
        // SCK �������
        GPIO_InitStructure.GPIO_Pin = SHT10_SCK_PIN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(SHT10_SCK_PORT, &GPIO_InitStructure);

        SHT10_ConReset();        //��λͨѶ
}


/*************************************************************
  Function   ��SHT10_DATAOut
  Description������DATA����Ϊ���
  Input      : none        
  return     : none    
*************************************************************/
void SHT10_DATAOut(void)
{
        GPIO_InitTypeDef GPIO_InitStructure;
        //PD0 DATA �������        
        GPIO_InitStructure.GPIO_Pin = SHT10_DATA_PIN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;         
        GPIO_Init(SHT10_DATA_PORT, &GPIO_InitStructure);
}


/*************************************************************
  Function   ��SHT10_DATAIn  
  Description������DATA����Ϊ����
  Input      : none        
  return     : none    
*************************************************************/
void SHT10_DATAIn(void)
{
        GPIO_InitTypeDef GPIO_InitStructure;
        //PD0 DATA ��������        
        GPIO_InitStructure.GPIO_Pin = SHT10_DATA_PIN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(SHT10_DATA_PORT, &GPIO_InitStructure);
}


/*************************************************************
  Function   ��SHT10_WriteByte  
  Description��д1�ֽ�
  Input      : value:Ҫд����ֽ�        
  return     : err: 0-��ȷ  1-����    
*************************************************************/
u8 SHT10_WriteByte(u8 value)
{
        u8 i, err = 0;
        
        SHT10_DATAOut();                          //����DATA������Ϊ���

        for(i = 0x80; i > 0; i /= 2)  //д1���ֽ�
        {
                if(i & value)
                        SHT10_DATA_H();
                else
                        SHT10_DATA_L();
                SHT10_Dly();
                SHT10_SCK_H();
                SHT10_Dly();
                SHT10_SCK_L();
                SHT10_Dly();
        }
        SHT10_DATAIn();                                  //����DATA������Ϊ����,�ͷ�DATA��
        SHT10_SCK_H();
        err = SHT10_DATA_R();                  //��ȡSHT10��Ӧ��λ
        SHT10_SCK_L();

        return err;
}

/*************************************************************
  Function   ��SHT10_ReadByte  
  Description����1�ֽ�����
  Input      : Ack: 0-��Ӧ��  1-Ӧ��        
  return     : err: 0-��ȷ 1-����    
*************************************************************/
u8 SHT10_ReadByte(u8 Ack)
{
        u8 i, val = 0;

        SHT10_DATAIn();                                  //����DATA������Ϊ����
        for(i = 0x80; i > 0; i /= 2)  //��ȡ1�ֽڵ�����
        {
                SHT10_Dly();
                SHT10_SCK_H();
                SHT10_Dly();
                if(SHT10_DATA_R())
                        val = (val | i);
                SHT10_SCK_L();
        }
        SHT10_DATAOut();                          //����DATA������Ϊ���
        if(Ack)
                SHT10_DATA_L();                          //Ӧ��������ȥ������ȥ������(У������)
        else
                SHT10_DATA_H();                          //��Ӧ���������˽���
        SHT10_Dly();
        SHT10_SCK_H();
        SHT10_Dly();
        SHT10_SCK_L();
        SHT10_Dly();

        return val;                                          //���ض�����ֵ
}


/*************************************************************
  Function   ��SHT10_TransStart  
  Description����ʼ�����źţ�ʱ�����£�
                     _____         ________
               DATA:      |_______|
                         ___     ___
               SCK : ___|   |___|   |______        
  Input      : none        
  return     : none    
*************************************************************/
void SHT10_TransStart(void)
{
        SHT10_DATAOut();                          //����DATA������Ϊ���

        SHT10_DATA_H();
        SHT10_SCK_L();
        SHT10_Dly();
        SHT10_SCK_H();
        SHT10_Dly();
        SHT10_DATA_L();
        SHT10_Dly();
        SHT10_SCK_L();
        SHT10_Dly();
        SHT10_SCK_H();
        SHT10_Dly();
        SHT10_DATA_H();
        SHT10_Dly();
        SHT10_SCK_L();

}


/*************************************************************
  Function   ��SHT10_ConReset  
  Description��ͨѶ��λ��ʱ�����£�
                     _____________________________________________________         ________
               DATA:                                                      |_______|
                        _    _    _    _    _    _    _    _    _        ___     ___
               SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|   |___|   |______
  Input      : none        
  return     : none    
*************************************************************/
void SHT10_ConReset(void)
{
        u8 i;

        SHT10_DATAOut();

        SHT10_DATA_H();
        SHT10_SCK_L();

        for(i = 0; i < 9; i++)                  //����SCKʱ��9c��
        {
                SHT10_SCK_H();
                SHT10_Dly();
                SHT10_SCK_L();
                SHT10_Dly();
        }
        SHT10_TransStart();                          //��������
}



/*************************************************************
  Function   ��SHT10_SoftReset  
  Description����λ
  Input      : none        
  return     : err: 0-��ȷ  1-����    
*************************************************************/
u8 SHT10_SoftReset(void)
{
        u8 err = 0;

        SHT10_ConReset();                              //ͨѶ��λ
        err += SHT10_WriteByte(SOFTRESET);//дRESET��λ����

        return err;
}


/*************************************************************
  Function   ��SHT10_ReadStatusReg  
  Description����״̬�Ĵ���
  Input      : p_value-���������ݣ�p_checksun-������У������       
  return     : err: 0-��ȷ  0-����    
*************************************************************/
u8 SHT10_ReadStatusReg(u8 *p_value, u8 *p_checksum)
{
        u8 err = 0;

        SHT10_TransStart();                                        //��ʼ����
        err = SHT10_WriteByte(STATUS_REG_R);//дSTATUS_REG_R��ȡ״̬�Ĵ�������
        *p_value = SHT10_ReadByte(ACK);                //��ȡ״̬����
        *p_checksum = SHT10_ReadByte(noACK);//��ȡ���������
        
        return err;
}



/*************************************************************
  Function   ��SHT10_WriteStatusReg  
  Description��д״̬�Ĵ���
  Input      : p_value-Ҫд�������ֵ       
  return     : err: 0-��ȷ  1-����    
*************************************************************/
u8 SHT10_WriteStatusReg(u8 *p_value)
{
        u8 err = 0;

        SHT10_TransStart();                                         //��ʼ����
        err += SHT10_WriteByte(STATUS_REG_W);//дSTATUS_REG_Wд״̬�Ĵ�������
        err += SHT10_WriteByte(*p_value);         //д������ֵ

        return err;
}



/*************************************************************
  Function   ��SHT10_Measure  
  Description������ʪ�ȴ�������ȡ��ʪ��
  Input      : p_value-������ֵ��p_checksum-������У����        
  return     : err: 0-��ȷ 1������    
*************************************************************/
u8 SHT10_Measure(u16 *p_value, u8 *p_checksum, u8 mode)
{
        u8 err = 0;
        u32 i;
        u8 value_H = 0;
        u8 value_L = 0;

        SHT10_TransStart();   //��ʼ����
        switch(mode)                                                         
        {
        case TEMP:            //�����¶�
                err += SHT10_WriteByte(MEASURE_TEMP);//дMEASURE_TEMP�����¶�����
                break;
        case HUMI:
                err += SHT10_WriteByte(MEASURE_HUMI);//дMEASURE_HUMI����ʪ������
                break;
        default:
                break;
        }
        SHT10_DATAIn();
        for(i = 0; i < 72000000; i++)                             //�ȴ�DATA�źű�����
        {
                if(SHT10_DATA_R() == 0) break;             //��⵽DATA�������ˣ�����ѭ��
        }
        if(SHT10_DATA_R() == 1)                                //����ȴ���ʱ��
                err += 1;
        value_H = SHT10_ReadByte(ACK);
        value_L = SHT10_ReadByte(ACK);
        *p_checksum = SHT10_ReadByte(noACK);           //��ȡУ������
        *p_value = (value_H << 8) | value_L;
        return err;
}


/*************************************************************
  Function   ��SHT10_Calculate  
  Description��������ʪ�ȵ�ֵ
  Input      : Temp-�Ӵ������������¶�ֵ��Humi-�Ӵ�����������ʪ��ֵ
               p_humidity-�������ʵ�ʵ�ʪ��ֵ��p_temperature-�������ʵ���¶�ֵ        
  return     : none    
*************************************************************/
void SHT10_Calculate(u16 t, u16 rh, float *p_temperature, float *p_humidity)
{
        const float d1 = -39.7;
        const float d2 = +0.01;
        const float C1 = -2.0468;
        const float        C2 = +0.0367;
        const float C3 = -0.0000015955;        
        const float T1 = +0.01;
        const float T2 = +0.00008;

        float RH_Lin;                                                                                  //RH����ֵ        
        float RH_Ture;                                                                                 //RH��ʵֵ
        float temp_C;

        temp_C = d1 + d2 * t;                                                              //�����¶�ֵ        
        RH_Lin = C1 + C2 * rh + C3 * rh * rh;                            //����ʪ��ֵ
        RH_Ture = (temp_C -25) * (T1 + T2 * rh) + RH_Lin;        //ʪ�ȵ��¶Ȳ���������ʵ�ʵ�ʪ��ֵ

        if(RH_Ture > 100)                                                                        //����ʪ��ֵ����
                RH_Ture        = 100;
        if(RH_Ture < 0.1)
                RH_Ture = 0.1;                                                                        //����ʪ��ֵ����

        *p_humidity = RH_Ture;
        *p_temperature = temp_C;

}


/*************************************************************
  Function   ��SHT10_CalcuDewPoint  
  Description������¶��
  Input      : h-ʵ�ʵ�ʪ�ȣ�t-ʵ�ʵ��¶�        
  return     : dew_point-¶��    
*************************************************************/
float SHT10_CalcuDewPoint(float t, float h)
{
        float logEx, dew_point;

        logEx = 0.66077 + 7.5 * t / (237.3 + t) + (log10(h) - 2);
        dew_point = ((0.66077 - logEx) * 237.3) / (logEx - 8.16077);

        return dew_point; 
}
