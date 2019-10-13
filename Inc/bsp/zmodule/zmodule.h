#ifndef _ZMODULE_H
#define _ZMODULE_H

#include "head.h"
#define MAX_MSG_SIZE  256
#define MAX_JSON_SIZE (MAX_MSG_SIZE-8)

//---------------------- ͨ�����-----------------------------
//ȡ����Ԫ��������ģ�����Ϣ��businessType
#define MSG_WRONG_BUSINESS_TYPE  -1
#define MSG_WRONG_JSON  -2
#define MSG_WRONG_CRC -3
#define MSG_GO_TO_HEIGHT 22


extern double current_height_in_m;


// ----------------go to ��ض���------------------------------
//�ٶȺ;��붨��
#define FAST_VELOCITY 1500  //����
#define SLOW_VELOCITY 600   //����
#define SLOW_RANGE 0.05  //�����С��0.05m��ʱ�����
#define EQUAL_RANGE 0.005  //��������0.005m��ʱ����Ϊ�������

//goto��״̬����
#define STOP -1
#define FAR2HIGH 0    
#define NEAR2HIGH 1
#define TOO_HIGH 2
#define FAR2LOW 3
#define NEAR2LOW 4
#define TOO_LOW 5

//������
#define DIR_HIGH 1  //�ߣ�Զ��������
#define DIR_LOW -1

//����ģ���˶���ĳ���̶�λ��,�����

extern int state ;
extern double current_height_in_m;
extern double destination_height;
extern int global_state ;
extern u8 UART5_JSON_BUF[256]; //���յ�������
extern u8 UART5_JSON_CRC;
extern int is_distance_receiving ;  //��ʾ����һֱ�ڽ�������
extern int is_distance_right ;      //��ʾ�����������������ΪD = ***m������Error

int goTo(double destination_height_in_m);

//����crcУ��ֵ
unsigned	char crc8_calculate(unsigned char * ucPtr, unsigned char ucLen) ;
int generate_send_str(cJSON *root,char * strSend);

//��ȡ����Ԫͨ��
int resolve_msg(void);  //����ģ��ͨ������һ�������������businessType������Ŀ��height��depthȫ�ֱ�����ֵ
void on_go_to_height_msg(void);  //�յ�ȡ����ԪҪ��ȥĳ�߶ȵ�����
void on_check_msg(void);




#endif

