//=================================================================
// ��Ŀ���ƣ�MODBUS�ӻ�ͨѶ
//==================================================================
#include "stm32f10x.h"
#include "mb.h"
 
/*----------------------------------------------------------------
���ƣ�         ��Ȧ
�洢����ʶ��   0XXXX
���ͣ�         λ
��д��         ��/д
�洢��Ԫ��ַ��(00001~0XXXX).XXXX:���豸���
�����룺       ��ȡ��Ȧ״̬ (0x01)��д������Ȧ (0x05)��д�����Ȧ (0x0F)
��;��         �����������Ҳ����DO�̵�������
//-----------------------------------------------------------------*/
#define REG_COILS_START     0x0001
#define REG_COILS_SIZE      32
unsigned char   ucRegCoilsBuf[REG_COILS_SIZE / 8]={0XFF,0X00};

/*----------------------------------------------------------------
���ƣ�         ������Ȧ
�洢����ʶ��   1XXXX
���ͣ�         λ
��д��         ֻ��
�洢��Ԫ��ַ�� (10001~1XXXX).XXXX:���豸���
�����룺       ������״̬ (0x02)
��;��         ���������빦�ܡ�Ҳ����DI����������
//-----------------------------------------------------------------*/
/* ----------------------- Defines ------------------------------------------*/
#define REG_DISC_START     0x0001
#define REG_DISC_SIZE      32
unsigned char ucRegDiscBuf[REG_DISC_SIZE / 8] = { 0x0F, 0XF0 };

/*----------------------------------------------------------------
���ƣ�         ����Ĵ���
�洢����ʶ��   3XXXX
���ͣ�         ��
��д��         ֻ��
�洢��Ԫ��ַ�� (30001~3XXXX).XXXX:���豸���
�����룺       ������Ĵ��� (0x04)
��;��         ģ�������룬����ɼ��ĵ�������ѹ���¶ȵȵ�ģ�������ݡ�
//-----------------------------------------------------------------*/
#define REG_INPUT_START   0x0001
#define REG_INPUT_NREGS   32
USHORT   usRegInputStart = REG_INPUT_START;
USHORT   usRegInputBuf[REG_INPUT_NREGS]={0x1122,0x3344,0x5566,0x7788};
//����4���Ĵ�����ǰ������ADC����ֵ����2����DAC���ֵ����λ����04������������������ʱ��Ӧ�ظ���4���Ĵ�����
/*----------------------------------------------------------------
���ƣ�         ����/����Ĵ���
�洢����ʶ��   4XXXX
���ͣ�         ��
��д��         ��/д
�洢��Ԫ��ַ�� (40001~4XXXX).XXXX:���豸���
�����룺       �����ּĴ��� (0x03)��д�����Ĵ��� (0x06)��д����Ĵ��� (0x10)
��;��         ģ���������Ҳ����DA����ģ���������صĹ��ܡ�
//-----------------------------------------------------------------*/
#define REG_HOLDING_START 0x0001  //���ּĴ�����ʼ��ַ 
#define REG_HOLDING_NREGS 32       //���ּĴ�������
USHORT   usRegHoldingStart = REG_HOLDING_START;
USHORT   usRegHoldingBuf[REG_HOLDING_NREGS]={0x0001,0x0002,0x0003,0x0004};
//�����Ƿ�2��DCA������ֵ�����ں�2�����Ĵ����С���λ������03����ʱ������������ݻ��͡�
//ͬʱ����Ҳ���Զ���д�������վ���������޸���������ݣ�Ȼ������� 


void ENTER_CRITICAL_SECTION(void)// �����ж�
{
	__set_PRIMASK(1);
}

void EXIT_CRITICAL_SECTION(void)// �����ж�
{
	__set_PRIMASK(0);
}

/*===================================================================
function name: eMBRegCoilsCB()
Description:  0x01 ,0x05,0x0F �������������ݷ���BUF
Attention��   0x01�������DO����0x05,0x0fǿ�Ƶ����������Ȧ��DO��
Input Parameters: 
Returncode��
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
Description:  0x02 ������״̬/��ȡ����״̬�������������ݷ���BUF
Attention��
Input Parameters: 
Returncode��
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
Description:   0x04 ��Ĵ����������������ݷ���BUF 
Attention��
Input Parameters: 
Returncode��������ܺ���������Ĵ�����04���������롣ֻ֧����λ������������ֻҪ�Ѳ����õ���ֵ��usRegInputBuf[]��ͺ��ˡ�
Ϊ�˷�����λ����04�����룬�Ϳ��԰�ADC����ֵ��DAC��ǰֵ������λ���У������DAC����ֵҲ����usRegInputBuf[]
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
Description:   0x03��0x06.0x10 ���ּĴ����������������ݷ���BUF 
Attention��
Input Parameters: 
Returncode��
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


