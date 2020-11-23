#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "sd3078.h"
#include "delay.h"
//Mini STM32开发板
//RTC实时时钟 驱动代码			 
//正点原子@ALIENTEK
//2010/6/6
	   
//_calendar_obj calendar;//时钟结构体 

void i2cInit()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);//打开AFIO时钟
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
		DBGMCU_CR &= 0xFFFFFFDF;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_SetBits(GPIOA,GPIO_Pin_15);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_SetBits(GPIOB,GPIO_Pin_3);
}

void i2cStart(void)
{
	SDA_OUT;     //sda线输出
	SDA_H;
	SCL_H;
	delay_us(4);
 	SDA_L;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	SCL_L;//钳住I2C总线，准备发送或接收数据 
}	 

void i2cStop(void)
{
	SDA_OUT;
	SCL_L;
	SDA_L;
	delay_us(4);
	SCL_H;
	SDA_H;
	delay_us(4);
}

u8 i2cWaitAck(void)
{
	u8 ucErrTime=0;
	SDA_IN;      //SDA设置为输入  
	SDA_H;delay_us(1);	   
	SCL_H;delay_us(1);	 
	while(sdaRead)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			i2cStop();
			return 1;
		}
	}
	SCL_L;//时钟输出0 	   
	return 0; 
}

void i2cAck(void)
{
	SCL_L;
	SDA_OUT;
	SDA_L;
	delay_us(2);
	SCL_H;
	delay_us(2);
	SCL_L;
}

void i2cNoAck(void)
{
	SCL_L;
	SDA_OUT;
	SDA_H;
	delay_us(2);
	SCL_H;
	delay_us(2);
	SCL_L;
}

void i2cSendByte(u8 txd)
{
	u8 t;   
	SDA_OUT; 	    
	SCL_L;//拉低时钟开始数据传输
	for(t=0;t<8;t++)
	{              
		if((txd&0x80)>>7)
			SDA_H;
		else
			SDA_L;
		txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		SCL_H;
		delay_us(2); 
		SCL_L;	
		delay_us(2);
	}	 
}

u8 i2cReadByte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN;//SDA设置为输入
  for(i=0;i<8;i++ )
	{
		SCL_L;
		delay_us(2);
		SCL_H;
		receive<<=1;
		if(sdaRead)receive++;   
		delay_us(1); 
	}					 
	if (!ack)
			i2cNoAck();//发送nACK
	else
			i2cAck(); //发送ACK   
	return receive;
}

/**
 *  
 * @param[in] dev:设备I2C地址
 * @param[in] reg:寄存器地址
 * @param[in] len:字节数 
 * @param[in] data:待写入的数据 
 */
bool i2cWriteBytes(uint8_t dev, uint8_t reg, uint8_t len, uint8_t *data)
{
    uint8_t i;

    i2cStart();      
    i2cSendByte(dev << 1);          
    if (i2cWaitAck()) {     
        i2cStop();
        return false;
    }
    i2cSendByte(reg);   
    i2cWaitAck();
    for (i = 0; i < len; i++) {
        i2cSendByte(data[i]);
        if (i2cWaitAck()) {
            i2cStop();
            return false;
        }
    }
    i2cStop();
    return true;
}


/**
 * 从I2C设备中读取数据
 * @param[in] dev:设备I2C地址
 * @param[in] reg:寄存器地址
 * @param[in] len:数据字节数
 * @param[out] data:读出的数据
 */
bool i2cReadBytes(uint8_t dev, uint8_t reg, uint8_t len, uint8_t *data)
{
		i2cStart();       
    i2cSendByte(dev << 1);      
    if (i2cWaitAck()) {     
        i2cStop();
        return false;
    }
    i2cSendByte(reg);     
    i2cWaitAck();
		
    i2cStart();           
    i2cSendByte((dev << 1) | 0x01);  // 器件地址+读命令    
    i2cWaitAck();
    while (len) {
        if (len == 1)
            *data = i2cReadByte(0);  // 最后一个字节不应答
        else
            *data = i2cReadByte(1);
        data++;
        len--;
    }
    i2cStop();
    return true;
}
		  
void WriteOn(void)//打开写保护
{
  i2cStart();
  i2cSendByte(0x64);
	i2cWaitAck();
  i2cSendByte(0x10);
	i2cWaitAck();
  i2cSendByte(0x80);
	i2cWaitAck();
  i2cStop();
  
  i2cStart();
  i2cSendByte(0x64);
	i2cWaitAck();
  i2cSendByte(0x0f);
	i2cWaitAck();
  i2cSendByte(0x84);
	i2cWaitAck();
  i2cStop();
}

void WriteOff(void)//关闭写保护
{
  i2cStart();
  i2cSendByte(0x64);
	i2cWaitAck();
  i2cSendByte(0x10);
	i2cWaitAck();
  i2cSendByte(0x00);
	i2cWaitAck();
  i2cStop();
  
  i2cStart();
  i2cSendByte(0x64);
	i2cWaitAck();
  i2cSendByte(0x0f);
	i2cWaitAck();
  i2cSendByte(0x00);
	i2cWaitAck();
	i2cStop();
}

void read_time_from_sd30xx(u8 s_data[7])//从sd30xx读时间
{
  u8 n = 0;

	i2cStart();
	i2cSendByte(0x65);
	if (i2cWaitAck()) {     
		i2cStop();
		return;
	}
	
  for (n = 0; n < 7; n++)
  {
    if (n == 6)
    {
      s_data[n] = i2cReadByte(0);
    }
		else
		{
			s_data[n] = i2cReadByte(1);
		}
  }
  i2cStop();
}

void write_time_to_sd30xx(u8 s_data[7])//写时间到sd30xx
{
  u8 n = 0;

  WriteOn();
  
	i2cStart();
  i2cSendByte(0x64);
	if (i2cWaitAck()) {     
		i2cStop();
		return;
	}
	
  i2cSendByte(0x00);
  for (n = 0; n < 7; n++)
  {
    i2cSendByte(s_data[n]);
		i2cWaitAck();
  }
  i2cStop();
  
  WriteOff();
}













