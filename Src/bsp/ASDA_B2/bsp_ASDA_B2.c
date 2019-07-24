/**
  ******************************************************************************
  * �ļ�����: bsp_mb_host.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-4-14
  * ��    ��: ���ص���Modbus������������
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F1Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "ASDA_B2/bsp_ASDA_B2.h"
#include "usart/bsp_debug_usart.h"
#include "string.h"
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
__IO  enum usartSTA UsartState = Ready;
static uint16_t TimeOut = 10;            // ͨѶ��ʱ ��λ:ms
/* ��չ���� ------------------------------------------------------------------*/
extern __IO uint8_t Rx_Buf[50];         // �������ݻ���
/* ˽�к���ԭ�� --------------------------------------------------------------*/
static void FunCode_ReadParam(uint8_t SlaveAddr,uint16_t RegAddr,uint16_t DataNum);
static void FunCode_WriteParam(uint8_t SlaveAddr,uint16_t RegAddr,uint16_t Data);
static void FunCode_Write_DWord_Param(uint8_t SlaveAddr,uint16_t RegAddr,uint16_t DataNum,int32_t Data);
static uint8_t prvucMBCHAR2BIN( uint8_t ucCharacter );
static uint8_t prvucMBBIN2CHAR( uint8_t ucByte );
static uint8_t prvucMBLRC( uint8_t* pucFrame, uint8_t usLen );
/* ������ --------------------------------------------------------------------*/

void ASDAB2_Init()
{
  /* �����ŷ���� */
  FunCode_WriteParam(SEARCH_SLAVENUM,REG_ADR,SLAVE_NUM);
  
  /* �����ٶ�ģʽ ��Ҫ�ŷ��ϵ�����������Ч */
  FunCode_WriteParam(SLAVE_NUM,REG_CTL,SLAVE_SPEEDMODE);
    
  /* ���ø�������,���в����ϵ�󲻴洢,����ָ����Ч֮��Ĳ��������ᱣ�� */
  FunCode_WriteParam(SLAVE_NUM,REG_INH,SLAVE_TEMPMODE);
  
  /* DI���Ź滮 */
  CONFIG_DIn_SPD0();    // ���� DinΪ TCM0
  CONFIG_DIn_SPD1();    // ���� DinΪ TCM1
  CONFIG_DIn_SON();     // ���� DInΪ SON �ŷ���������
  /* DI����� */
  FunCode_WriteParam(SLAVE_NUM,REG_SDI,SLAVE_ALL_DI);     // ����DI������P4-07����
  
  /* ѡ���ٶȼĴ��� */
  CONFIG_AS_SP3();// ѡ���ٶ�ֵ��SP3����;SPD1:SPD0=1:1;SON = 0;
}
/**
  * ��������: ����DIn��״̬
  * �������: REG_DIn:DI����;  DIState: �޸ĺ��DI״̬
  * �� �� ֵ: ��
  * ˵    ��: ����DI�ź�Ϊ�����ʱ,���е�DI״̬��P4-07��Ӧλ����,
  *           �ȶ�ȡP4-07ԭ��ֵ,Ȼ���޸Ķ�Ӧλ,�����д��
  */
void ModifyDIn(uint16_t REG_DIn,DI_STA DIState)
{
  uint16_t data = 0;
  uint8_t dataH = 0,dataL = 0;              // ���ݸ߰���,�Ͱ���
  FunCode_ReadParam(SLAVE_NUM,REG_ITST,0x0001); // ��ȡDI�ź�
  dataH = (prvucMBCHAR2BIN(Rx_Buf[7]))<<4|(prvucMBCHAR2BIN(Rx_Buf[8]));
  dataL = (prvucMBCHAR2BIN(Rx_Buf[9]))<<4|(prvucMBCHAR2BIN(Rx_Buf[10]));
  data = (dataH<<8)|dataL;                    // �ϲ�Ϊ1����
  /* �޸�data��Ӧ��λ */
  switch(REG_DIn)
  {
    case REG_DI1:data &= ~(0x0001<<0);data |=  (DIState<<0);break;    
    case REG_DI2:data &= ~(0x0001<<1);data |=  (DIState<<1);break; 
    case REG_DI3:data &= ~(0x0001<<2);data |=  (DIState<<2);break; 
    case REG_DI4:data &= ~(0x0001<<3);data |=  (DIState<<3);break; 
    case REG_DI5:data &= ~(0x0001<<4);data |=  (DIState<<4);break; 
    case REG_DI6:data &= ~(0x0001<<5);data |=  (DIState<<5);break; 
    case REG_DI7:data &= ~(0x0001<<6);data |=  (DIState<<6);break; 
    case REG_DI8:data &= ~(0x0001<<7);data |=  (DIState<<7);break; 
    default :break;
  }
  /* ����д��P4-07 */
  FunCode_WriteParam(SLAVE_NUM,REG_ITST,data);
}
/**
  * ��������: �����ٶ�ֵ
  * �������: REG_SPn:�ٶȼĴ���;Speed:�ٶ�ֵ,��λ:0.1r/min
  * �� �� ֵ: ��
  * ˵    ��: ���õ���ٶ�,�ŷ������ڲ��ٶ�ָ������������Ĵ�������
  */
void SetSpeed(uint16_t REG_SPn,int32_t Speed)
{
  switch(REG_SPn)
  {
    case REG_SP1:FunCode_Write_DWord_Param(SLAVE_NUM,REG_SP1,0x0002,Speed);break;
    case REG_SP2:FunCode_Write_DWord_Param(SLAVE_NUM,REG_SP2,0x0002,Speed);break;
    case REG_SP3:FunCode_Write_DWord_Param(SLAVE_NUM,REG_SP3,0x0002,Speed);break;
    default :break;
  }
}
/**
  * ��������: ����n���ֵ�����
  * �������: SlaveAddr:�ӻ���ַ;RegAddr:�Ĵ����׵�ַ;DataNum:�������������
  * �� �� ֵ: ��
  * ˵    ��: ��modbusЭ���ASCIIģʽ��ӻ���ȡn��(n*16bits),
  */
void FunCode_ReadParam(uint8_t SlaveAddr,uint16_t RegAddr,uint16_t DataNum)
{
  uint8_t tmpBuf[10];
  uint8_t TxBuf[20];
  uint8_t bin_LRC = 0;
  uint8_t i = 0,j = 0;
  uint32_t Tick = 0;

  /* ��䷢�ͻ��� */
  tmpBuf[i++] = ':';
  tmpBuf[i++] = (uint8_t)SlaveAddr;
  tmpBuf[i++] = (uint8_t)FUNCODE_03H;      //������
  tmpBuf[i++] = (uint8_t)(RegAddr>>8);     //�Ĵ�����ַ��8λ
  tmpBuf[i++] = (uint8_t)RegAddr;          //�Ĵ�����ַ��8λ
  tmpBuf[i++] = (uint8_t)(DataNum>>8);     //���ݸ�����8λ
  tmpBuf[i++] = (uint8_t)DataNum;          //���ݸ�����ַ��8λ
  bin_LRC = prvucMBLRC((uint8_t*)&tmpBuf[1],6);//����LRC
  tmpBuf[i++] = bin_LRC;
  tmpBuf[i] = '\0';
  
  /* ����֡ת����ASCIIģʽ */
  i = 0; j = 0;
  TxBuf[j++] = tmpBuf[0];
  while( i++ < 7)
  {
    TxBuf[j++] = prvucMBBIN2CHAR( tmpBuf[i] >> 4);
    TxBuf[j++] = prvucMBBIN2CHAR( tmpBuf[i] & 0x0F);
  }
  TxBuf[j++] = 0x0D;//������
  TxBuf[j++] = 0x0A;
  TxBuf[j] = '\0';
  HAL_UART_Transmit(&husartx_rs485,TxBuf,strlen((const char*)TxBuf),0x0F);
  UsartState = WaitRx;
  Tick = HAL_GetTick ();
  while(UsartState == WaitRx)              // �ȴ���������
  {
    if((HAL_GetTick() - Tick >= TimeOut))
    {
      UsartState = Ready;
      printf("ͨѶ��ʱ ����֡:\n");
      printf("%s\n",TxBuf);
      break;
    }  
  }
}
/**
  * ��������: д��1���ֵ��ӻ�
  * �������: SlaveAddr:�ӻ���ַ;RegAddr:�Ĵ����׵�ַ;Data:д�������
  * �� �� ֵ: ��
  * ˵    ��: ��modbusЭ���ASCIIģʽ��ӻ�д��1��(16bits),
  */
void FunCode_WriteParam(uint8_t SlaveAddr,uint16_t RegAddr,uint16_t Data)
{
  uint8_t tmpBuf[10];
  uint8_t TxBuf[20];
  uint8_t i = 0,j = 0;
  uint32_t Tick = 0;
  /* ��䷢�ͻ��� */
  tmpBuf[i++] = ':';                          // ��ʼ��
  tmpBuf[i++] = (uint8_t)SlaveAddr;           // �豸��ַ
  tmpBuf[i++] = (uint8_t)FUNCODE_06H;         // ������
  tmpBuf[i++] = (uint8_t)(RegAddr>>8);        // �Ĵ�����ַ��8λ
  tmpBuf[i++] = (uint8_t) RegAddr;            // �Ĵ�����ַ��8λ
  tmpBuf[i++] = (uint8_t)(Data>>8);           // ���ݸ�8λ
  tmpBuf[i++] = (uint8_t) Data;               // ���ݵ�8λ
  tmpBuf[i++] = prvucMBLRC((uint8_t*)&tmpBuf[1],6);//����LRC
  tmpBuf[i] = '\0';
  
  /* ����֡ת����ASCIIģʽ */
  i = 0; j = 0;
  TxBuf[j++] = tmpBuf[0];
  while( i++ < 7)
  {
    TxBuf[j++] = prvucMBBIN2CHAR( tmpBuf[i] >> 4);
    TxBuf[j++] = prvucMBBIN2CHAR( tmpBuf[i] & 0x0F);
  }
  TxBuf[j++] = '\r';        // ������,0x0D CR
  TxBuf[j++] = '\n';        // ������,0x0A LF
  TxBuf[j] = '\0';
  HAL_UART_Transmit(&husartx_rs485,TxBuf,strlen((const char*)TxBuf),0x0F);
  UsartState = WaitRx;      // ����Ѿ�����,�ȴ�����״̬
  Tick = HAL_GetTick ();    // ��ȡ��ǰ�������ݵ�ʱ��
  while(UsartState == WaitRx)              // �ȴ���������
  {
    if((HAL_GetTick() - Tick >= TimeOut))
    {
      UsartState = Ready;
      printf("ͨѶ��ʱ ����֡:\n");
      printf("%s\n",TxBuf);
      break;
    }  
  }
}
/**
  * ��������: д˫�ֵ��ӻ�
  * �������: SlaveAddr:�ӻ���ַ;RegAddr:�Ĵ����׵�ַ;DataNum:д�������;Data:д�������
  * �� �� ֵ: ��
  * ˵    ��: ��modbusЭ���ASCIIģʽ��ӻ�д��˫��(32bits)
  */
void FunCode_Write_DWord_Param(uint8_t SlaveAddr,uint16_t RegAddr,uint16_t DataNum,int32_t Data)
{
  uint8_t tmpBuf[20];
  uint8_t TxBuf[50];
  uint8_t i = 0,j = 0;
  uint32_t Tick = 0;
  /* ��䷢�ͻ��� */
  tmpBuf[i++] = ':';                           // ��ʼ��
  tmpBuf[i++] = (uint8_t)SlaveAddr;            // �豸��ַ
  tmpBuf[i++] = (uint8_t)FUNCODE_10H;          // ������
  
  tmpBuf[i++] = (uint8_t)(RegAddr >> 8);       // �Ĵ�����ַ��8λ
  tmpBuf[i++] = (uint8_t) RegAddr;             // �Ĵ�����ַ��8λ
  tmpBuf[i++] = (uint8_t)(DataNum >> 8);       // ������8λ
  tmpBuf[i++] = (uint8_t) DataNum;             // ������8λ
  tmpBuf[i++] = (uint8_t)(DataNum << 1) ;      // �ֽ���
  
  tmpBuf[i++] = (uint8_t)(Data>>8);            // ����,���ֽ���ǰ
  tmpBuf[i++] = (uint8_t) Data;                // 
  tmpBuf[i++] = (uint8_t)(Data>>24);           //
  tmpBuf[i++] = (uint8_t)(Data>>16);           //
  
  tmpBuf[i++] = prvucMBLRC((uint8_t*)&tmpBuf[1],11);//����LRC
  tmpBuf[i] = '\0';
  
  i = 0; j = 0;
  /* ����֡ת����ASCIIģʽ */
  TxBuf[j++] = tmpBuf[0];
  while( i++ < 12)
  {
    TxBuf[j++] = prvucMBBIN2CHAR( tmpBuf[i] >> 4);
    TxBuf[j++] = prvucMBBIN2CHAR( tmpBuf[i] & 0x0F);
  }
  TxBuf[j++] = '\r';                 // ������,0x0D CR
  TxBuf[j++] = '\n';                 // ������,0x0A LF
  TxBuf[j] = '\0';
  /* ʹ�ô��ڷ������� */
  HAL_UART_Transmit(&husartx_rs485,TxBuf,strlen((const char*)TxBuf),0x0F);
  UsartState = WaitRx;              // ����Ѿ�����,�ȴ�����״̬
  Tick = HAL_GetTick ();
  while(UsartState == WaitRx)              // �ȴ���������
  {
    if((HAL_GetTick() - Tick >= TimeOut))
    {
      UsartState = Ready;
      printf("ͨѶ��ʱ ����֡:\n");
      printf("%s\n",TxBuf);
      break;
    }  
  }
}
/**
  * ��������: ASCII�ַ�ת16������
  * �������: ucCharacter��ASCII�ַ�
  * �� �� ֵ: 16������
  * ˵    ��: ��һ��ASCII�ַ�ת����16������,��:'A'-->0AH
  */
uint8_t prvucMBCHAR2BIN( uint8_t ucCharacter )
{
    if( ( ucCharacter >= '0' ) && ( ucCharacter <= '9' ) )
    {
        return ( uint8_t )( ucCharacter - '0' );
    }
    else if( ( ucCharacter >= 'A' ) && ( ucCharacter <= 'F' ) )
    {
        return ( uint8_t )( ucCharacter - 'A' + 0x0A );
    }
    else
    {
        return 0xFF;
    }
}
/**
  * ��������: 16����תASCII�ַ�
  * �������: ucByte��16�������Ͱ��ֽ�
  * �� �� ֵ: ASCII�ַ�
  * ˵    ��: ��һ��16�������ĵͰ��ֽ�ת����ASCII,��:06H->'6'
  */
static uint8_t prvucMBBIN2CHAR( uint8_t ucByte )
{
    if( ucByte <= 0x09 )
    {
        return ( uint8_t )( '0' + ucByte );
    }
    else if( ( ucByte >= 0x0A ) && ( ucByte <= 0x0F ) )
    {
        return ( uint8_t )( ucByte - 0x0A + 'A' );
    }
    return '0';
}

/**
  * ��������: LRC����
  * �������: pucFrame����Ҫ�����Դ����,usLen:���ݳ���
  * �� �� ֵ: LRCУ��ֵ
  * ˵    ��: ����̶����ݳ��ȵ�LRCУ��ֵ
  */
static uint8_t prvucMBLRC( uint8_t* pucFrame, uint8_t usLen )
{
    uint8_t ucLRC = 0;  /* LRC char initialized */

    while( usLen-- )
    {
        ucLRC += *pucFrame++;   /* Add buffer byte without carry */
    }

    /* Return twos complement */
    ucLRC = ( uint8_t) ( -( ( uint8_t) ucLRC ) );
    return ucLRC;
}
