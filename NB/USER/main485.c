 
#include "stm32f10x.h"  
#include "stm32f10x_rcc.h"  
#include "stm32f10x_gpio.h"  
#include "stm32f10x_usart.h"  
#include "stm32f10x_exti.h"  
#include "system_stm32f10x.h"  
#include "misc.h"  
#include "delay.h"
#include "led.h"
void RCC_Configuration(void);  
void GPIO_Configuration(void);   
void USART_Configuration(void);  
void NVIC_Configuration(void); 
int main()  
{  
	u8 i;
    SystemInit();//72m  
      
    RCC_Configuration();  
    GPIO_Configuration();   
    USART_Configuration();  
		delay_init();	
  	LED_Init();
    NVIC_Configuration();

  
                
                                                   
    while(1){
				i++;
		delay_ms(10);
		if(i==20)
		{
			LED1=!LED1;//提示系统正在运行	
			i=0;
		}
	
}
}  
  
void RCC_Configuration(void)    
{     
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOE|RCC_APB2Periph_AFIO,ENABLE);    
  RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);  
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); 
}   
  
void GPIO_Configuration(void)    
{    
  GPIO_InitTypeDef GPIO_InitStructure;   
    
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;             //CS_485  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //推挽输出  
  GPIO_Init(GPIOA, &GPIO_InitStructure);     
    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;             //管脚2  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;       //复用推挽输出  
  GPIO_Init(GPIOA, &GPIO_InitStructure);                //TX初始化  
    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;             //管脚3  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入          
  GPIO_Init(GPIOA, &GPIO_InitStructure);                //RX初始化  
//	GPIO_SetBits(GPIOA,GPIO_Pin_1); 
	 GPIO_ResetBits(GPIOA,GPIO_Pin_1);
}   
  
void USART_Configuration(void)//串口初始化函数  
{  
    //串口参数初始化    
    USART_InitTypeDef USART_InitStructure;               //串口设置恢复默认参数  
    //初始化参数设置  
    USART_InitStructure.USART_BaudRate = 9600;                  //波特率9600  
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //字长8位  
    USART_InitStructure.USART_StopBits = USART_StopBits_1;      //1位停止字节  
    USART_InitStructure.USART_Parity = USART_Parity_No;         //无奇偶校验  
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无流控制  
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//打开Rx接收和Tx发送功能  
    USART_Init(USART2,&USART_InitStructure);                   //初始化  
      
    USART_ITConfig(USART2,USART_IT_TXE,ENABLE);                //允许串口1发送中断。  
    USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);               //允许串口1接收中断。  
    USART_Cmd(USART2,ENABLE);                                  //启动串口  
    USART_ClearFlag(USART2,USART_FLAG_TC);                     //发送完成标志位  
}   
void NVIC_Configuration(void)//配置中断优先级    
{    
  NVIC_InitTypeDef NVIC_InitStructure;    
      
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);    
      
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;    
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;    
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;    
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    
  NVIC_Init(&NVIC_InitStructure);    
}    