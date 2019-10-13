#ifndef _ZMODULE_H
#define _ZMODULE_H

#include "head.h"
#define MAX_MSG_SIZE  256
#define MAX_JSON_SIZE (MAX_MSG_SIZE-8)

//---------------------- 通信相关-----------------------------
//取货单元发给线性模组的信息的businessType
#define MSG_WRONG_BUSINESS_TYPE  -1
#define MSG_WRONG_JSON  -2
#define MSG_WRONG_CRC -3
#define MSG_GO_TO_HEIGHT 22


extern double current_height_in_m;


// ----------------go to 相关定义------------------------------
//速度和距离定义
#define FAST_VELOCITY 1500  //快速
#define SLOW_VELOCITY 600   //慢速
#define SLOW_RANGE 0.05  //距离差小于0.05m的时候减速
#define EQUAL_RANGE 0.005  //距离差不超过0.005m的时候认为距离相等

//goto的状态定义
#define STOP -1
#define FAR2HIGH 0    
#define NEAR2HIGH 1
#define TOO_HIGH 2
#define FAR2LOW 3
#define NEAR2LOW 4
#define TOO_LOW 5

//方向定义
#define DIR_HIGH 1  //高，远离电机，负
#define DIR_LOW -1

//控制模组运动到某个固定位置,含测距

extern int state ;
extern double current_height_in_m;
extern double destination_height;
extern int global_state ;
extern u8 UART5_JSON_BUF[256]; //接收到的数据
extern u8 UART5_JSON_CRC;
extern int is_distance_receiving ;  //表示红外一直在接收数据
extern int is_distance_right ;      //表示红外接收数据正常，为D = ***m，而非Error

int goTo(double destination_height_in_m);

//计算crc校验值
unsigned	char crc8_calculate(unsigned char * ucPtr, unsigned char ucLen) ;
int generate_send_str(cJSON *root,char * strSend);

//与取货单元通信
int resolve_msg(void);  //解析模组通过串口一传来的命令，返回businessType，并给目标height和depth全局变量赋值
void on_go_to_height_msg(void);  //收到取货单元要求去某高度的命令
void on_check_msg(void);




#endif

