#include "key.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK Mini STM32开发板
//按键输入 驱动代码		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/3/06
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									   
//////////////////////////////////////////////////////////////////////////////////	 
 	    
//按键初始化函数 
//PA15和PC5 设置成输入
//#define N_key    0             //无键 
//#define GE_key    1            //个位
//#define SHI_key    2           //十位 
//#define BAI_key    3           //百位
//#define QIAN_key    4          //千位
//#define key_state_0        0
//#define key_state_1        1   //有按键按下
//#define key_state_2        2
//#define key_state_3        3
//#define key_state_4        4
void KEY_Init(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;

 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);//使能PORTA,PORTC时钟

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//关闭jtag，使能SWD，可以用SWD模式调试
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_9|GPIO_Pin_8|GPIO_Pin_7|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //设置成下拉输入//GPIO_Mode_IPU
 	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	
} 
//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
 //返回值：
//0，没有任何按键按下
//KEY4_PRES，KEY4按下
//KEY3_PRES，KEY3按下
//KEY2_PRES，KEY2按下
//KEY1_PRES，KEY1按下
//注意此函数有响应优先级,KEY4>KEY3>KEY2>KEY1!!
u16  Trg=0;//按键触发
u16 Cont=0;//按键连按
u16 AddUp=0;
extern u8 key;
//void KEY_Scan(u8 mode)
//{
////if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9))
//	if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9))
//{
//		delay_ms(10);//去抖动 
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
//		delay_ms(10);//去抖动 
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
//		delay_ms(10);//去抖动 
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
//		delay_ms(10);//去抖动 
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
////		delay_ms(10);//去抖动 
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
////		return 0;// 无按键按下
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
	if(Trg&0x200)//PC9短按退出
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
	else if(Trg&0x100)//PC8短按加
	{
		delay_ms(100);//去抖动 
		//if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8))	
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8))	
		{
	
			AddUp=0;
			key=2;
			//while(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8))	;
			//while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8))	;
		}
		
	}
	else if(Trg&0x80)//PC7短按减
	{
		delay_ms(100);//去抖动 
		//if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7))
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7))
		{

			AddUp=0;
			key=3;
			//while(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7));
			//while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7));
		}
	}
	else if(Trg&0x40)//PC6短按确认键
	{
		delay_ms(100);//去抖动 
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

	if(Cont&0x100)//PC8长按连续加
	{
		AddUp++;
		if(AddUp>10000)
		{
			AddUp=0;
			key=2;
		}
	}

	else if(Cont&0x80)//PC7长按连续减
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

//    key_press = GPIO_ReadInputDataBit(GPIOx, GPIO_Pin);                    // 读按键I/O电平 

//    switch (key_state) 
//    { 
//      case key_state_0:                              // 按键初始态 
//        if (key_press) key_state = key_state_1;      // 键被按下，状态转换到按键消抖和确认状态 
//        break; 
//       
//      case key_state_1:                      // 按键消抖与确认态 
//        if (key_press) 
//        { 
//             key_time= 0;                   //  按键开始计时
//             key_time1=0;
//             key_time2=0;
//             key_time3=0;
//             key_state = key_state_2;   // 按键仍然处于按下，消抖完成，状态转换到按下键时间的计时状态，但返回的还是无键事件 
//        } 
//        else 
//             key_state = key_state_0;   // 按键已抬起，转换到按键初始态。此处完成和实现软件消抖，其实按键的按下和释放都在此消抖的。 
//        break; 
//       
//      case key_state_2: 
//        if(!key_press) 
//        { 
//             key_return = GE_key;        // 此时按键释放，说明是产生一次短操作，回送S_key 
//             key_state = key_state_0;   // 转换到按键初始态 
//        } 
//        else if (++key_time >=250)
//        {
//             key_state = key_state_3;
//          
//        } 
//        else if(++key_time1 >=25)     // 每隔0.5s增加10
//        {
//             key_return = SHI_key;  
//             key_time1=0;
//        }
//        break; 
//         case key_state_3:                 // 等待按键释放状态，此状态只返回无按键事件 
//        if (!key_press) 
//          key_state = key_state_0; //按键已释放，转换到按键初始态 
//        else if (++key_time >=500)     // 继续按下，计时加10ms（10ms为本函数循环执行间隔） 
//              { 
//                   key_state = key_state_4;
//                   key_time=0;
//              } 
//              else
//              {
//                 if (++key_time2 >=25)     // 继续按下，计时加10ms（10ms为本函数循环执行间隔）
//                  { 
//                      key_return = BAI_key;        // 按下时间>1000ms，此按键为长按操作，返回长键事件 
//                      key_time2=0;//每隔0.5s增加100
//                  }
//                
//                
//              }
//              break; 
//          
//          
//          
//         
//      case key_state_4:                 // 等待按键释放状态，此状态只返回无按键事件 
//        if (!key_press) 
//          key_state = key_state_0; //按键已释放，转换到按键初始态 
//        else if (++key_time3 >= 25)     // 继续按下，计时加10ms（10ms为本函数循环执行间隔） 
//        { 
//             key_return = QIAN_key;        // 按下时间>1000ms，此按键为长按操作，返回长键事件 
//             //key_state = key_state_3;   // 转换到等待按键释放状态 
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
//	static u8 key_up=1;//按键按松开标志
//	if(mode)key_up=1;  //支持连按		  
//	if(key_up&&(KEY4==0||KEY3==0||KEY2==0||KEY1==0))
//	{
//		delay_ms(10);//去抖动 
//		key_up=0;
//		if(KEY4==0)return KEY4_PRES;
//		else if(KEY3==0)return KEY3_PRES;
//		else if(KEY2==0)return KEY2_PRES; 
//		else if(KEY1==0)return KEY1_PRES; 
//	}else if(KEY4==1&&KEY3==1&&KEY2==1&&KEY1==1)key_up=1; 	     
//	return 0;// 无按键按下
//}
