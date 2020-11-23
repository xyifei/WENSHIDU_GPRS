//=================================================================
// 项目名称：MODBUS从机通讯
//==================================================================
#include "stm32f10x.h"
#include "mb.h"
 
/*----------------------------------------------------------------
名称：         线圈
存储区标识：   0XXXX
类型：         位
读写：         读/写
存储单元地址：(00001~0XXXX).XXXX:与设备相关
功能码：       读取线圈状态 (0x01)，写单个线圈 (0x05)，写多个线圈 (0x0F)
用途：         开关量输出，也就是DO继电器功能
//-----------------------------------------------------------------*/
#define REG_COILS_START     0x0001
#define REG_COILS_SIZE      32
unsigned char   ucRegCoilsBuf[REG_COILS_SIZE / 8]={0XFF,0X00};

/*----------------------------------------------------------------
名称：         输入线圈
存储区标识：   1XXXX
类型：         位
读写：         只读
存储单元地址： (10001~1XXXX).XXXX:与设备相关
功能码：       读输入状态 (0x02)
用途：         开关量输入功能、也就是DI开关量输入
//-----------------------------------------------------------------*/
/* ----------------------- Defines ------------------------------------------*/
#define REG_DISC_START     0x0001
#define REG_DISC_SIZE      32
unsigned char ucRegDiscBuf[REG_DISC_SIZE / 8] = { 0x0F, 0XF0 };

/*----------------------------------------------------------------
名称：         输入寄存器
存储区标识：   3XXXX
类型：         字
读写：         只读
存储单元地址： (30001~3XXXX).XXXX:与设备相关
功能码：       读输入寄存器 (0x04)
用途：         模拟量输入，如果采集的电流、电压、温度等等模拟量数据。
//-----------------------------------------------------------------*/
#define REG_INPUT_START   0x0001
#define REG_INPUT_NREGS   32
USHORT   usRegInputStart = REG_INPUT_START;
USHORT   usRegInputBuf[REG_INPUT_NREGS]={0x1122,0x3344,0x5566,0x7788};
//定义4个寄存器，前两个是ADC采样值，后2个是DAC输出值。上位机用04功能码命令来读数据时，应回复这4个寄存器。
/*----------------------------------------------------------------
名称：         保持/输出寄存器
存储区标识：   4XXXX
类型：         字
读写：         读/写
存储单元地址： (40001~4XXXX).XXXX:与设备相关
功能码：       读保持寄存器 (0x03)，写单个寄存器 (0x06)，写多个寄存器 (0x10)
用途：         模拟量输出，也就是DA功能模拟量输出相关的功能。
//-----------------------------------------------------------------*/
#define REG_HOLDING_START 0x0001  //保持寄存器起始地址 
#define REG_HOLDING_NREGS 32       //保持寄存器数量
USHORT   usRegHoldingStart = REG_HOLDING_START;
USHORT   usRegHoldingBuf[REG_HOLDING_NREGS]={0x0001,0x0002,0x0003,0x0004};
//这里是放2个DCA的数据值，放在后2两个寄存器中。上位机发送03命令时，就是这个数据回送。
//同时这里也可以对它写命令，即主站发命令来修改这里的数据，然后输出。 


void ENTER_CRITICAL_SECTION(void)// 关总中断
{
	__set_PRIMASK(1);
}

void EXIT_CRITICAL_SECTION(void)// 开总中断
{
	__set_PRIMASK(0);
}

/*===================================================================
function name: eMBRegCoilsCB()
Description:  0x01 ,0x05,0x0F 处理函数，将数据放入BUF
Attention：   0x01读输出（DO），0x05,0x0f强制单个，多个线圈（DO）
Input Parameters: 
Returncode：
====================================================================*/
eMBErrorCode eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    
    int             iNCoils = ( int )usNCoils;
    unsigned short  usBitOffset;

    /* Check if we have registers mapped at this block. */
    if( ((usAddress>REG_COILS_START)||(usAddress==REG_COILS_START)) &&( usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE ) )
    {
        usBitOffset = ( unsigned short )( usAddress - REG_COILS_START );
        switch ( eMode )
        {
                /* Read current values and pass to protocol stack. */
            case MB_REG_READ:
                while( iNCoils > 0 )
                {
                    *pucRegBuffer++ = xMBUtilGetBits( ucRegCoilsBuf, usBitOffset, ( unsigned char )( iNCoils > 8 ? 8 :iNCoils ) );
                    iNCoils -= 8;
                    usBitOffset += 8;
                }
                break;

                /* Update current register values. */
            case MB_REG_WRITE:
                {
					while( iNCoils > 0 )
	                {
	                    xMBUtilSetBits( ucRegCoilsBuf, usBitOffset,( unsigned char )( iNCoils > 8 ? 8 : iNCoils ), *pucRegBuffer++ );
	                    iNCoils -= 8;
	                    usBitOffset += 8;
	                }
				 }
                break;
        }

    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;  
    
}
/*===================================================================
function name: eMBRegDiscreteCB()
Description:  0x02 读输入状态/读取输入状态处理函数，将数据放入BUF
Attention：
Input Parameters: 
Returncode：
====================================================================*/
eMBErrorCode  eMBRegDiscreteCB( UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    short           iNDiscrete = ( short )usNDiscrete;
    unsigned short  usBitOffset;

    /* Check if we have registers mapped at this block. */
    if( ((usAddress>REG_DISC_START)||(usAddress==REG_DISC_START)) &&( usAddress + usNDiscrete<= REG_DISC_START + REG_DISC_SIZE ))
    {
        usBitOffset = (unsigned short)( usAddress - REG_DISC_START );
        while( iNDiscrete > 0 )
        {
            *pucRegBuffer++ =xMBUtilGetBits(ucRegDiscBuf, usBitOffset, (unsigned char)( iNDiscrete >8?8: iNDiscrete ) );
            iNDiscrete -= 8;
            usBitOffset+=8;
        }
    }
    else
    {
        eStatus = MB_ENOREG;//illegal register address
    }
    return eStatus; 
}
/*===================================================================
function name: eMBRegInputCB()
Description:   0x04 入寄存器处理函数，将数据放入BUF 
Attention：
Input Parameters: 
Returncode：这个功能函数是输入寄存器，04功能命令码。只支持上位机来读。所以只要把采样得到数值放usRegInputBuf[]里就好了。
为了方便上位机发04功能码，就可以把ADC采样值，DAC当前值读到上位机中，这里把DAC数据值也放在usRegInputBuf[]
====================================================================*/
eMBErrorCode  eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs ) 
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( (usAddress>REG_INPUT_START)||(usAddress==REG_INPUT_START) ) && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
     {
        iRegIndex = ( int )( usAddress - usRegInputStart );
        
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ = (s32)( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ = ( s32 )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
     }
    else
     {
        eStatus = MB_ENOREG; //illegal register address
     }

    return eStatus;
}
/*===================================================================
function name: eMBRegHoldingCB()
Description:   0x03，0x06.0x10 保持寄存器处理函数，将数据放入BUF 
Attention：
Input Parameters: 
Returncode：
====================================================================*/
eMBErrorCode eMBRegHoldingCB( UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if(((usAddress>REG_HOLDING_START)||(usAddress==REG_HOLDING_START)) && (usAddress+usNRegs<=REG_HOLDING_START+REG_HOLDING_NREGS) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );
        switch ( eMode )
        {
        case MB_REG_READ:
            while( usNRegs > 0 )
            {
				*pucRegBuffer++ = (s32 )( usRegHoldingBuf[iRegIndex] >> 8 );
				*pucRegBuffer++ = ( s32 )( usRegHoldingBuf[iRegIndex] & 0xFF );
				iRegIndex++;

				usNRegs--;
            }
            break;

        case MB_REG_WRITE:
		    {
            while( usNRegs > 0 )
            {
				usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
			}
			break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}


