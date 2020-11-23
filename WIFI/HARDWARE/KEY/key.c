#include "key.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK Mini STM32������
//�������� ��������		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/3/06
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									   
//////////////////////////////////////////////////////////////////////////////////	 
 	    
//������ʼ������ 
//PA15��PC5 ���ó�����
//#define N_key    0             //�޼� 
//#define GE_key    1            //��λ
//#define SHI_key    2           //ʮλ 
//#define BAI_key    3           //��λ
//#define QIAN_key    4          //ǧλ
//#define key_state_0        0
//#define key_state_1        1   //�а�������
//#define key_state_2        2
//#define key_state_3        3
//#define key_state_4        4
void KEY_Init(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;

 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);//ʹ��PORTA,PORTCʱ��

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//�ر�jtag��ʹ��SWD��������SWDģʽ����
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_9|GPIO_Pin_8|GPIO_Pin_7|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //���ó���������//GPIO_Mode_IPU
 	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	
} 
//����������
//���ذ���ֵ
//mode:0,��֧��������;1,֧��������;
 //����ֵ��
//0��û���κΰ�������
//KEY4_PRES��KEY4����
//KEY3_PRES��KEY3����
//KEY2_PRES��KEY2����
//KEY1_PRES��KEY1����
//ע��˺�������Ӧ���ȼ�,KEY4>KEY3>KEY2>KEY1!!
u16  Trg=0;//��������
u16 Cont=0;//��������
u16 AddUp=0;
extern u8 key;
//void KEY_Scan(u8 mode)
//{
////if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9))
//	if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9))
//{
//		delay_ms(10);//ȥ���� 
//		//if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9))
//		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9))
//		{
//			key=1;
//			//key=4;
//			//while(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9));
//			while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9));
//		}
//}
//else
//{
////if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8))
//if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8))		
//{
//		delay_ms(10);//ȥ���� 
//		//if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8))	
//		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8))	
//		{
//			key=2;
//			//while(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8))	;
//			while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8))	;
//		}
//}
//else
//{
////if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7))
//	if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7))
//{
//		delay_ms(10);//ȥ���� 
//		//if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7))
//		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7))
//		{
//			key=3;
//			//while(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7));
//			while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7));
//		}
//}
//else
//{
////if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6))
//	if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6))
//{
//		delay_ms(10);//ȥ���� 
//		//if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6))
//		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6))
//		{
//			key=4;
//			//key=1;
//			//while(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6));
//			while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6));
//		}
//		
//}
//else
//	key=0;	
//}
//}
//}




////	if((KEY4==0||KEY3==0||KEY2==0||KEY1==0))
////	{
////		delay_ms(10);//ȥ���� 
////		
////		if(KEY4==0)
////		{
////			return 4;
////			while(!KEY4);
////		}
////    if(KEY3==0)
////		{
////		   return 3;
////			while(!KEY3);
////		}

////		if(KEY1==0)
////		{
////		   return 1; 
////			while(!KEY1);
////		}
////				if(KEY2==0)
////		{
////		   return 2;
////			while(!KEY2);
////		}
////	}
////	else
////	{
////		return 0;// �ް�������
////	}
//// 

//}


void KeyRead( void )

{
	u16 ReadData;  
	
	if(GPIO_ReadInputData(GPIOC)&0x03C0)
		ReadData=(GPIO_ReadInputData(GPIOC))&0x03C0^0x0000;// 1
	Trg = ReadData & (ReadData ^Cont);     // 2
	Cont=ReadData;                              // 3

}

void KEY_Scan()
{

	KeyRead();
	if(Trg&0x200)//PC9�̰��˳�
	{
		delay_ms(100);
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9))
		{

			AddUp=0;
			key=1;
			//key=4;
			//while(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9));
			//while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9));
		}

	}
	else if(Trg&0x100)//PC8�̰���
	{
		delay_ms(100);//ȥ���� 
		//if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8))	
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8))	
		{
	
			AddUp=0;
			key=2;
			//while(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8))	;
			//while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8))	;
		}
		
	}
	else if(Trg&0x80)//PC7�̰���
	{
		delay_ms(100);//ȥ���� 
		//if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7))
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7))
		{

			AddUp=0;
			key=3;
			//while(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7));
			//while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7));
		}
	}
	else if(Trg&0x40)//PC6�̰�ȷ�ϼ�
	{
		delay_ms(100);//ȥ���� 
		//if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6))
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6))
		{
			AddUp=0;
			key=4;
			//key=1;
			//while(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6));
			//while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6));
		}
	}
	else 
	{
		key=0;

	}

	if(Cont&0x100)//PC8����������
	{
		AddUp++;
		if(AddUp>10000)
		{
			AddUp=0;
			key=2;
		}
	}

	else if(Cont&0x80)//PC7����������
	{
		AddUp++;
		if(AddUp>10000)
		{
			AddUp=0;
			key=3;
		}
	}

	
}
//unsigned char key_driver1(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) 
//{ 
//    static u16 key_state = key_state_0, key_time = 0,key_time1=0,key_time2=0,key_time3=0; 
//    unsigned char key_press, key_return = N_key; 

//    key_press = GPIO_ReadInputDataBit(GPIOx, GPIO_Pin);                    // ������I/O��ƽ 

//    switch (key_state) 
//    { 
//      case key_state_0:                              // ������ʼ̬ 
//        if (key_press) key_state = key_state_1;      // �������£�״̬ת��������������ȷ��״̬ 
//        break; 
//       
//      case key_state_1:                      // ����������ȷ��̬ 
//        if (key_press) 
//        { 
//             key_time= 0;                   //  ������ʼ��ʱ
//             key_time1=0;
//             key_time2=0;
//             key_time3=0;
//             key_state = key_state_2;   // ������Ȼ���ڰ��£�������ɣ�״̬ת�������¼�ʱ��ļ�ʱ״̬�������صĻ����޼��¼� 
//        } 
//        else 
//             key_state = key_state_0;   // ������̧��ת����������ʼ̬���˴���ɺ�ʵ�������������ʵ�����İ��º��ͷŶ��ڴ������ġ� 
//        break; 
//       
//      case key_state_2: 
//        if(!key_press) 
//        { 
//             key_return = GE_key;        // ��ʱ�����ͷţ�˵���ǲ���һ�ζ̲���������S_key 
//             key_state = key_state_0;   // ת����������ʼ̬ 
//        } 
//        else if (++key_time >=250)
//        {
//             key_state = key_state_3;
//          
//        } 
//        else if(++key_time1 >=25)     // ÿ��0.5s����10
//        {
//             key_return = SHI_key;  
//             key_time1=0;
//        }
//        break; 
//         case key_state_3:                 // �ȴ������ͷ�״̬����״ֻ̬�����ް����¼� 
//        if (!key_press) 
//          key_state = key_state_0; //�������ͷţ�ת����������ʼ̬ 
//        else if (++key_time >=500)     // �������£���ʱ��10ms��10msΪ������ѭ��ִ�м���� 
//              { 
//                   key_state = key_state_4;
//                   key_time=0;
//              } 
//              else
//              {
//                 if (++key_time2 >=25)     // �������£���ʱ��10ms��10msΪ������ѭ��ִ�м����
//                  { 
//                      key_return = BAI_key;        // ����ʱ��>1000ms���˰���Ϊ�������������س����¼� 
//                      key_time2=0;//ÿ��0.5s����100
//                  }
//                
//                
//              }
//              break; 
//          
//          
//          
//         
//      case key_state_4:                 // �ȴ������ͷ�״̬����״ֻ̬�����ް����¼� 
//        if (!key_press) 
//          key_state = key_state_0; //�������ͷţ�ת����������ʼ̬ 
//        else if (++key_time3 >= 25)     // �������£���ʱ��10ms��10msΪ������ѭ��ִ�м���� 
//        { 
//             key_return = QIAN_key;        // ����ʱ��>1000ms���˰���Ϊ�������������س����¼� 
//             //key_state = key_state_3;   // ת�����ȴ������ͷ�״̬ 
//             key_time3=0;
//        }
//        break;  
//        
//       
//    } 
//    return key_return; 
//} 


//u8 KEY_Scan(u8 mode)
//{	 
//	static u8 key_up=1;//�������ɿ���־
//	if(mode)key_up=1;  //֧������		  
//	if(key_up&&(KEY4==0||KEY3==0||KEY2==0||KEY1==0))
//	{
//		delay_ms(10);//ȥ���� 
//		key_up=0;
//		if(KEY4==0)return KEY4_PRES;
//		else if(KEY3==0)return KEY3_PRES;
//		else if(KEY2==0)return KEY2_PRES; 
//		else if(KEY1==0)return KEY1_PRES; 
//	}else if(KEY4==1&&KEY3==1&&KEY2==1&&KEY1==1)key_up=1; 	     
//	return 0;// �ް�������
//}
