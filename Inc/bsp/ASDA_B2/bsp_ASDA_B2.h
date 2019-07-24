#ifndef _BSP_ASDA_B2_h_
#define _BSP_ASDA_B2_h_

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
enum usartSTA
{
	WaitRx = 0x00 ,Ready = 0x01
};
typedef enum 
{
  DI_RESET = 0,
  DI_SET = 1
}DI_STA;
/* 宏定义 --------------------------------------------------------------------*/
#define SLAVE_NUM           0x0002            // 从机在通讯网络上的绝对地址,绝对地址具有唯一性
                                              // 最多可连接7F个 ASDA-B2型号的驱动器
#define SEARCH_SLAVENUM     0x00FF            // 发送FFH搜索从机,从机会自动响应"FF" 
#define SLAVE_SPEEDMODE     0x0104            // 从机运行模式,速度模式(Sz),扭矩方向:正转时为CW
#define SLAVE_ALL_DI        0x01FF            // 从机所有DI有P4-07控制
#define DICODE_SON          0x0101            // 从机配置DI功能为SON
#define DICODE_SPD0         0x0114            // 从机配置DI功能为SPD0
#define DICODE_SPD1         0x0115            // 从机配置DI功能为SPD1
#define SLAVE_BAUDRATE      0x0055            // 通讯波特率 0x55:115200
#define SLAVE_TEMPMODE      0x0005            // 掉电后不存储参数

/* 伺服驱动器部分常用寄存器地址 */
#define REG_CTL             0x0102            // P1-01 控制模式及控制指令输入源设定
#define REG_SP1             0x0112            // P1-09 内部速度指令 1/内部速度限制 1
#define REG_SP2             0x0114            // P1-10 内部速度指令 2/内部速度限制 2
#define REG_SP3             0x0116            // P1-11 内部速度指令 3/内部速度限制 3

#define REG_DI1             0x0214            // P2-10 数字输入接脚 REG_DI1 功能规划
#define REG_DI2             0x0216            // P2-11 数字输入接脚 REG_DI2 功能规划
#define REG_DI3             0x0218            // P2-12 数字输入接脚 REG_DI3 功能规划
#define REG_DI4             0x021A            // P2-13 数字输入接脚 REG_DI4 功能规划
#define REG_DI5             0x021C            // P2-14 数字输入接脚 REG_DI5 功能规划
#define REG_DI6             0x021E            // P2-15 数字输入接脚 REG_DI6 功能规划
#define REG_DI7             0x0220            // P2-16 数字输入接脚 REG_DI7 功能规划
#define REG_DI8             0x0222            // P2-17 数字输入接脚 REG_DI8 功能规划
#define REG_INH             0x023C            // P2-30 辅助功能

#define REG_ADR             0x0300            // P3-00 局号设定
#define REG_BRT             0x0302            // P3-01 通讯传输率
#define REG_PTL             0x0304            // P3-02 通讯协议
#define REG_SDI             0x030C            // P3-06 输入接点（ DI）来源控制开关
                                                 // 断点后此参数不记忆设定的内容值                                            
#define REG_ITST            0x040E            // P4-07 数字输入接点多重功能 
                                                 // 断电后此参数不记忆设定的内容值
/* Modbus部分功能码 */

#define FUNCODE_03H         0x03              // 功能码,03H,读多个寄存器
#define FUNCODE_06H         0x06              // 功能码,06H,写单个寄存器
#define FUNCODE_10H         0x10              // 功能码,10H,写多个寄存器

/* DI引脚功能定义 */
#define SON                 REG_DI1           // 定义 REG_DI1功能 为SON
#define SPD0                REG_DI3           // 定义 REG_DI3功能 为SPD0
#define SPD1                REG_DI4           // 定义 REG_DI4功能 为SPD1

#define CONFIG_DIn_SON()    FunCode_WriteParam(SLAVE_NUM,SON,DICODE_SON);      // 配置REG_DI1的功能为SON
#define CONFIG_DIn_SPD0()   FunCode_WriteParam(SLAVE_NUM,SPD0,DICODE_SPD0)     // 配置REG_DI3的功能为SPD0
#define CONFIG_DIn_SPD1()   FunCode_WriteParam(SLAVE_NUM,SPD1,DICODE_SPD1);    // 配置REG_DI4的功能为SPD1

#define StartServo()        ModifyDIn(SON,DI_SET)   // 启动伺服电机 SON = 1
#define StopServo()         ModifyDIn(SON,DI_RESET) // 停止伺服电机 SON = 0
#define CONFIG_AS_SP1()     {ModifyDIn(SPD1,DI_RESET);ModifyDIn(SPD0,DI_SET);}     // 选择速度寄存器 REG_SP1
#define CONFIG_AS_SP2()     {ModifyDIn(SPD1,DI_SET);ModifyDIn(SPD0,DI_RESET);}     // 选择速度寄存器 REG_SP1
#define CONFIG_AS_SP3()     {ModifyDIn(SPD1,DI_SET);ModifyDIn(SPD0,DI_SET);}       // 选择速度寄存器 REG_SP3

/* 扩展变量 ------------------------------------------------------------------*/
extern __IO enum usartSTA UsartState;

/* 函数声明 ------------------------------------------------------------------*/
void ASDAB2_Init(void);
void SetSpeed(uint16_t REG_SP,int32_t Speed);     // 设置速度
void ModifyDIn(uint16_t REG_DIn,DI_STA DIState);  // 更改DIn状态 
#endif /* _BSP_ASDA_B2_h_ */
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
