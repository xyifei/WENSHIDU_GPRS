 
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
			LED1=!LED1;//��ʾϵͳ��������	
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
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //�������  
  GPIO_Init(GPIOA, &GPIO_InitStructure);     
    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;             //�ܽ�2  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;       //�����������  
  GPIO_Init(GPIOA, &GPIO_InitStructure);                //TX��ʼ��  
    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;             //�ܽ�3  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������          
  GPIO_Init(GPIOA, &GPIO_InitStructure);                //RX��ʼ��  
//	GPIO_SetBits(GPIOA,GPIO_Pin_1); 
	 GPIO_ResetBits(GPIOA,GPIO_Pin_1);
}   
  
void USART_Configuration(void)//���ڳ�ʼ������  
{  
    //���ڲ�����ʼ��    
    USART_InitTypeDef USART_InitStructure;               //�������ûָ�Ĭ�ϲ���  
    //��ʼ����������  
    USART_InitStructure.USART_BaudRate = 9600;                  //������9600  
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //�ֳ�8λ  
    USART_InitStructure.USART_StopBits = USART_StopBits_1;      //1λֹͣ�ֽ�  
    USART_InitStructure.USART_Parity = USART_Parity_No;         //����żУ��  
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��������  
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//��Rx���պ�Tx���͹���  
    USART_Init(USART2,&USART_InitStructure);                   //��ʼ��  
      
    USART_ITConfig(USART2,USART_IT_TXE,ENABLE);                //������1�����жϡ�  
    USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);               //������1�����жϡ�  
    USART_Cmd(USART2,ENABLE);                                  //��������  
    USART_ClearFlag(USART2,USART_FLAG_TC);                     //������ɱ�־λ  
}   
void NVIC_Configuration(void)//�����ж����ȼ�    
{    
  NVIC_InitTypeDef NVIC_InitStructure;    
      
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);    
      
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;    
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;    
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;    
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    
  NVIC_Init(&NVIC_InitStructure);    
}    