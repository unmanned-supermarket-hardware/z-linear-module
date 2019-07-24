/**
  ******************************************************************************
  * 文件名程: bsp_mb_host.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2017-4-14
  * 功    能: 板载调试Modbus主机驱动程序
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-F1Pro使用。
  * 
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "ASDA_B2/bsp_ASDA_B2.h"
#include "usart/bsp_debug_usart.h"
#include "string.h"
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
__IO  enum usartSTA UsartState = Ready;
static uint16_t TimeOut = 10;            // 通讯超时 单位:ms
/* 扩展变量 ------------------------------------------------------------------*/
extern __IO uint8_t Rx_Buf[50];         // 接收数据缓存
/* 私有函数原形 --------------------------------------------------------------*/
static void FunCode_ReadParam(uint8_t SlaveAddr,uint16_t RegAddr,uint16_t DataNum);
static void FunCode_WriteParam(uint8_t SlaveAddr,uint16_t RegAddr,uint16_t Data);
static void FunCode_Write_DWord_Param(uint8_t SlaveAddr,uint16_t RegAddr,uint16_t DataNum,int32_t Data);
static uint8_t prvucMBCHAR2BIN( uint8_t ucCharacter );
static uint8_t prvucMBBIN2CHAR( uint8_t ucByte );
static uint8_t prvucMBLRC( uint8_t* pucFrame, uint8_t usLen );
/* 函数体 --------------------------------------------------------------------*/

void ASDAB2_Init()
{
  /* 设置伺服编号 */
  FunCode_WriteParam(SEARCH_SLAVENUM,REG_ADR,SLAVE_NUM);
  
  /* 配置速度模式 需要伺服断电重启才能生效 */
  FunCode_WriteParam(SLAVE_NUM,REG_CTL,SLAVE_SPEEDMODE);
    
  /* 配置辅助功能,所有参数断电后不存储,在这指令生效之后的参数都不会保存 */
  FunCode_WriteParam(SLAVE_NUM,REG_INH,SLAVE_TEMPMODE);
  
  /* DI引脚规划 */
  CONFIG_DIn_SPD0();    // 配置 Din为 TCM0
  CONFIG_DIn_SPD1();    // 配置 Din为 TCM1
  CONFIG_DIn_SON();     // 配置 DIn为 SON 伺服启动引脚
  /* DI软控制 */
  FunCode_WriteParam(SLAVE_NUM,REG_SDI,SLAVE_ALL_DI);     // 所有DI引脚有P4-07控制
  
  /* 选择速度寄存器 */
  CONFIG_AS_SP3();// 选择速度值由SP3决定;SPD1:SPD0=1:1;SON = 0;
}
/**
  * 函数功能: 更改DIn的状态
  * 输入参数: REG_DIn:DI功能;  DIState: 修改后的DI状态
  * 返 回 值: 无
  * 说    明: 配置DI信号为软控制时,所有的DI状态有P4-07对应位决定,
  *           先读取P4-07原先值,然后修改对应位,最后再写入
  */
void ModifyDIn(uint16_t REG_DIn,DI_STA DIState)
{
  uint16_t data = 0;
  uint8_t dataH = 0,dataL = 0;              // 数据高半字,低半字
  FunCode_ReadParam(SLAVE_NUM,REG_ITST,0x0001); // 读取DI信号
  dataH = (prvucMBCHAR2BIN(Rx_Buf[7]))<<4|(prvucMBCHAR2BIN(Rx_Buf[8]));
  dataL = (prvucMBCHAR2BIN(Rx_Buf[9]))<<4|(prvucMBCHAR2BIN(Rx_Buf[10]));
  data = (dataH<<8)|dataL;                    // 合并为1个字
  /* 修改data对应的位 */
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
  /* 重新写入P4-07 */
  FunCode_WriteParam(SLAVE_NUM,REG_ITST,data);
}
/**
  * 函数功能: 设置速度值
  * 输入参数: REG_SPn:速度寄存器;Speed:速度值,单位:0.1r/min
  * 返 回 值: 无
  * 说    明: 设置电机速度,伺服驱动内部速度指令可以由三个寄存器控制
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
  * 函数功能: 读入n个字的数据
  * 输入参数: SlaveAddr:从机地址;RegAddr:寄存器首地址;DataNum:读入的数据数量
  * 返 回 值: 无
  * 说    明: 以modbus协议的ASCII模式向从机读取n字(n*16bits),
  */
void FunCode_ReadParam(uint8_t SlaveAddr,uint16_t RegAddr,uint16_t DataNum)
{
  uint8_t tmpBuf[10];
  uint8_t TxBuf[20];
  uint8_t bin_LRC = 0;
  uint8_t i = 0,j = 0;
  uint32_t Tick = 0;

  /* 填充发送缓存 */
  tmpBuf[i++] = ':';
  tmpBuf[i++] = (uint8_t)SlaveAddr;
  tmpBuf[i++] = (uint8_t)FUNCODE_03H;      //功能码
  tmpBuf[i++] = (uint8_t)(RegAddr>>8);     //寄存器地址高8位
  tmpBuf[i++] = (uint8_t)RegAddr;          //寄存器地址低8位
  tmpBuf[i++] = (uint8_t)(DataNum>>8);     //数据个数低8位
  tmpBuf[i++] = (uint8_t)DataNum;          //数据个数地址低8位
  bin_LRC = prvucMBLRC((uint8_t*)&tmpBuf[1],6);//计算LRC
  tmpBuf[i++] = bin_LRC;
  tmpBuf[i] = '\0';
  
  /* 发送帧转换成ASCII模式 */
  i = 0; j = 0;
  TxBuf[j++] = tmpBuf[0];
  while( i++ < 7)
  {
    TxBuf[j++] = prvucMBBIN2CHAR( tmpBuf[i] >> 4);
    TxBuf[j++] = prvucMBBIN2CHAR( tmpBuf[i] & 0x0F);
  }
  TxBuf[j++] = 0x0D;//结束符
  TxBuf[j++] = 0x0A;
  TxBuf[j] = '\0';
  HAL_UART_Transmit(&husartx_rs485,TxBuf,strlen((const char*)TxBuf),0x0F);
  UsartState = WaitRx;
  Tick = HAL_GetTick ();
  while(UsartState == WaitRx)              // 等待反馈数据
  {
    if((HAL_GetTick() - Tick >= TimeOut))
    {
      UsartState = Ready;
      printf("通讯超时 数据帧:\n");
      printf("%s\n",TxBuf);
      break;
    }  
  }
}
/**
  * 函数功能: 写入1个字到从机
  * 输入参数: SlaveAddr:从机地址;RegAddr:寄存器首地址;Data:写入的数据
  * 返 回 值: 无
  * 说    明: 以modbus协议的ASCII模式向从机写入1字(16bits),
  */
void FunCode_WriteParam(uint8_t SlaveAddr,uint16_t RegAddr,uint16_t Data)
{
  uint8_t tmpBuf[10];
  uint8_t TxBuf[20];
  uint8_t i = 0,j = 0;
  uint32_t Tick = 0;
  /* 填充发送缓存 */
  tmpBuf[i++] = ':';                          // 起始符
  tmpBuf[i++] = (uint8_t)SlaveAddr;           // 设备地址
  tmpBuf[i++] = (uint8_t)FUNCODE_06H;         // 功能码
  tmpBuf[i++] = (uint8_t)(RegAddr>>8);        // 寄存器地址高8位
  tmpBuf[i++] = (uint8_t) RegAddr;            // 寄存器地址低8位
  tmpBuf[i++] = (uint8_t)(Data>>8);           // 数据高8位
  tmpBuf[i++] = (uint8_t) Data;               // 数据低8位
  tmpBuf[i++] = prvucMBLRC((uint8_t*)&tmpBuf[1],6);//计算LRC
  tmpBuf[i] = '\0';
  
  /* 发送帧转换成ASCII模式 */
  i = 0; j = 0;
  TxBuf[j++] = tmpBuf[0];
  while( i++ < 7)
  {
    TxBuf[j++] = prvucMBBIN2CHAR( tmpBuf[i] >> 4);
    TxBuf[j++] = prvucMBBIN2CHAR( tmpBuf[i] & 0x0F);
  }
  TxBuf[j++] = '\r';        // 结束符,0x0D CR
  TxBuf[j++] = '\n';        // 结束符,0x0A LF
  TxBuf[j] = '\0';
  HAL_UART_Transmit(&husartx_rs485,TxBuf,strlen((const char*)TxBuf),0x0F);
  UsartState = WaitRx;      // 标记已经发送,等待接收状态
  Tick = HAL_GetTick ();    // 获取当前发送数据的时间
  while(UsartState == WaitRx)              // 等待反馈数据
  {
    if((HAL_GetTick() - Tick >= TimeOut))
    {
      UsartState = Ready;
      printf("通讯超时 数据帧:\n");
      printf("%s\n",TxBuf);
      break;
    }  
  }
}
/**
  * 函数功能: 写双字到从机
  * 输入参数: SlaveAddr:从机地址;RegAddr:寄存器首地址;DataNum:写入的字数;Data:写入的数据
  * 返 回 值: 无
  * 说    明: 以modbus协议的ASCII模式向从机写入双字(32bits)
  */
void FunCode_Write_DWord_Param(uint8_t SlaveAddr,uint16_t RegAddr,uint16_t DataNum,int32_t Data)
{
  uint8_t tmpBuf[20];
  uint8_t TxBuf[50];
  uint8_t i = 0,j = 0;
  uint32_t Tick = 0;
  /* 填充发送缓存 */
  tmpBuf[i++] = ':';                           // 起始符
  tmpBuf[i++] = (uint8_t)SlaveAddr;            // 设备地址
  tmpBuf[i++] = (uint8_t)FUNCODE_10H;          // 功能码
  
  tmpBuf[i++] = (uint8_t)(RegAddr >> 8);       // 寄存器地址高8位
  tmpBuf[i++] = (uint8_t) RegAddr;             // 寄存器地址低8位
  tmpBuf[i++] = (uint8_t)(DataNum >> 8);       // 字数高8位
  tmpBuf[i++] = (uint8_t) DataNum;             // 字数低8位
  tmpBuf[i++] = (uint8_t)(DataNum << 1) ;      // 字节数
  
  tmpBuf[i++] = (uint8_t)(Data>>8);            // 数据,低字节在前
  tmpBuf[i++] = (uint8_t) Data;                // 
  tmpBuf[i++] = (uint8_t)(Data>>24);           //
  tmpBuf[i++] = (uint8_t)(Data>>16);           //
  
  tmpBuf[i++] = prvucMBLRC((uint8_t*)&tmpBuf[1],11);//计算LRC
  tmpBuf[i] = '\0';
  
  i = 0; j = 0;
  /* 发送帧转换成ASCII模式 */
  TxBuf[j++] = tmpBuf[0];
  while( i++ < 12)
  {
    TxBuf[j++] = prvucMBBIN2CHAR( tmpBuf[i] >> 4);
    TxBuf[j++] = prvucMBBIN2CHAR( tmpBuf[i] & 0x0F);
  }
  TxBuf[j++] = '\r';                 // 结束符,0x0D CR
  TxBuf[j++] = '\n';                 // 结束符,0x0A LF
  TxBuf[j] = '\0';
  /* 使用串口发送数据 */
  HAL_UART_Transmit(&husartx_rs485,TxBuf,strlen((const char*)TxBuf),0x0F);
  UsartState = WaitRx;              // 标记已经发送,等待接收状态
  Tick = HAL_GetTick ();
  while(UsartState == WaitRx)              // 等待反馈数据
  {
    if((HAL_GetTick() - Tick >= TimeOut))
    {
      UsartState = Ready;
      printf("通讯超时 数据帧:\n");
      printf("%s\n",TxBuf);
      break;
    }  
  }
}
/**
  * 函数功能: ASCII字符转16进制数
  * 输入参数: ucCharacter：ASCII字符
  * 返 回 值: 16进制数
  * 说    明: 将一个ASCII字符转换成16进制数,例:'A'-->0AH
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
  * 函数功能: 16进制转ASCII字符
  * 输入参数: ucByte：16进制数低半字节
  * 返 回 值: ASCII字符
  * 说    明: 将一个16进制数的低半字节转换成ASCII,例:06H->'6'
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
  * 函数功能: LRC计算
  * 输入参数: pucFrame：需要计算的源数据,usLen:数据长度
  * 返 回 值: LRC校验值
  * 说    明: 计算固定数据长度的LRC校验值
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
