#ifndef _BSP_ASDA_B2_h_
#define _BSP_ASDA_B2_h_

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
enum usartSTA
{
	WaitRx = 0x00 ,Ready = 0x01
};
typedef enum 
{
  DI_RESET = 0,
  DI_SET = 1
}DI_STA;
/* �궨�� --------------------------------------------------------------------*/
#define SLAVE_NUM           0x0002            // �ӻ���ͨѶ�����ϵľ��Ե�ַ,���Ե�ַ����Ψһ��
                                              // ��������7F�� ASDA-B2�ͺŵ�������
#define SEARCH_SLAVENUM     0x00FF            // ����FFH�����ӻ�,�ӻ����Զ���Ӧ"FF" 
#define SLAVE_SPEEDMODE     0x0104            // �ӻ�����ģʽ,�ٶ�ģʽ(Sz),Ť�ط���:��תʱΪCW
#define SLAVE_ALL_DI        0x01FF            // �ӻ�����DI��P4-07����
#define DICODE_SON          0x0101            // �ӻ�����DI����ΪSON
#define DICODE_SPD0         0x0114            // �ӻ�����DI����ΪSPD0
#define DICODE_SPD1         0x0115            // �ӻ�����DI����ΪSPD1
#define SLAVE_BAUDRATE      0x0055            // ͨѶ������ 0x55:115200
#define SLAVE_TEMPMODE      0x0005            // ����󲻴洢����

/* �ŷ����������ֳ��üĴ�����ַ */
#define REG_CTL             0x0102            // P1-01 ����ģʽ������ָ������Դ�趨
#define REG_SP1             0x0112            // P1-09 �ڲ��ٶ�ָ�� 1/�ڲ��ٶ����� 1
#define REG_SP2             0x0114            // P1-10 �ڲ��ٶ�ָ�� 2/�ڲ��ٶ����� 2
#define REG_SP3             0x0116            // P1-11 �ڲ��ٶ�ָ�� 3/�ڲ��ٶ����� 3

#define REG_DI1             0x0214            // P2-10 ��������ӽ� REG_DI1 ���ܹ滮
#define REG_DI2             0x0216            // P2-11 ��������ӽ� REG_DI2 ���ܹ滮
#define REG_DI3             0x0218            // P2-12 ��������ӽ� REG_DI3 ���ܹ滮
#define REG_DI4             0x021A            // P2-13 ��������ӽ� REG_DI4 ���ܹ滮
#define REG_DI5             0x021C            // P2-14 ��������ӽ� REG_DI5 ���ܹ滮
#define REG_DI6             0x021E            // P2-15 ��������ӽ� REG_DI6 ���ܹ滮
#define REG_DI7             0x0220            // P2-16 ��������ӽ� REG_DI7 ���ܹ滮
#define REG_DI8             0x0222            // P2-17 ��������ӽ� REG_DI8 ���ܹ滮
#define REG_INH             0x023C            // P2-30 ��������

#define REG_ADR             0x0300            // P3-00 �ֺ��趨
#define REG_BRT             0x0302            // P3-01 ͨѶ������
#define REG_PTL             0x0304            // P3-02 ͨѶЭ��
#define REG_SDI             0x030C            // P3-06 ����ӵ㣨 DI����Դ���ƿ���
                                                 // �ϵ��˲����������趨������ֵ                                            
#define REG_ITST            0x040E            // P4-07 ��������ӵ���ع��� 
                                                 // �ϵ��˲����������趨������ֵ
/* Modbus���ֹ����� */

#define FUNCODE_03H         0x03              // ������,03H,������Ĵ���
#define FUNCODE_06H         0x06              // ������,06H,д�����Ĵ���
#define FUNCODE_10H         0x10              // ������,10H,д����Ĵ���

/* DI���Ź��ܶ��� */
#define SON                 REG_DI1           // ���� REG_DI1���� ΪSON
#define SPD0                REG_DI3           // ���� REG_DI3���� ΪSPD0
#define SPD1                REG_DI4           // ���� REG_DI4���� ΪSPD1

#define CONFIG_DIn_SON()    FunCode_WriteParam(SLAVE_NUM,SON,DICODE_SON);      // ����REG_DI1�Ĺ���ΪSON
#define CONFIG_DIn_SPD0()   FunCode_WriteParam(SLAVE_NUM,SPD0,DICODE_SPD0)     // ����REG_DI3�Ĺ���ΪSPD0
#define CONFIG_DIn_SPD1()   FunCode_WriteParam(SLAVE_NUM,SPD1,DICODE_SPD1);    // ����REG_DI4�Ĺ���ΪSPD1

#define StartServo()        ModifyDIn(SON,DI_SET)   // �����ŷ���� SON = 1
#define StopServo()         ModifyDIn(SON,DI_RESET) // ֹͣ�ŷ���� SON = 0
#define CONFIG_AS_SP1()     {ModifyDIn(SPD1,DI_RESET);ModifyDIn(SPD0,DI_SET);}     // ѡ���ٶȼĴ��� REG_SP1
#define CONFIG_AS_SP2()     {ModifyDIn(SPD1,DI_SET);ModifyDIn(SPD0,DI_RESET);}     // ѡ���ٶȼĴ��� REG_SP1
#define CONFIG_AS_SP3()     {ModifyDIn(SPD1,DI_SET);ModifyDIn(SPD0,DI_SET);}       // ѡ���ٶȼĴ��� REG_SP3

/* ��չ���� ------------------------------------------------------------------*/
extern __IO enum usartSTA UsartState;

/* �������� ------------------------------------------------------------------*/
void ASDAB2_Init(void);
void SetSpeed(uint16_t REG_SP,int32_t Speed);     // �����ٶ�
void ModifyDIn(uint16_t REG_DIn,DI_STA DIState);  // ����DIn״̬ 
#endif /* _BSP_ASDA_B2_h_ */
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
