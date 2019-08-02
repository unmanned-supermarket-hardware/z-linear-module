#include "zmodule/zmodule.h"
#include "head.h"
int state = STOP;
double current_height_in_m;
//--------------------------------------CRC校验-----------------------------------------
unsigned char const crc8_tab[256]  = 
{ 
	0x00,0x07,0x0E,0x09,0x1C,0x1B,0x12,0x15,0x38,0x3F,0x36,0x31,0x24,0x23,0x2A,0x2D, 
	0x70,0x77,0x7E,0x79,0x6C,0x6B,0x62,0x65,0x48,0x4F,0x46,0x41,0x54,0x53,0x5A,0x5D, 
	0xE0,0xE7,0xEE,0xE9,0xFC,0xFB,0xF2,0xF5,0xD8,0xDF,0xD6,0xD1,0xC4,0xC3,0xCA,0xCD, 
	0x90,0x97,0x9E,0x99,0x8C,0x8B,0x82,0x85,0xA8,0xAF,0xA6,0xA1,0xB4,0xB3,0xBA,0xBD, 
	0xC7,0xC0,0xC9,0xCE,0xDB,0xDC,0xD5,0xD2,0xFF,0xF8,0xF1,0xF6,0xE3,0xE4,0xED,0xEA, 
	0xB7,0xB0,0xB9,0xBE,0xAB,0xAC,0xA5,0xA2,0x8F,0x88,0x81,0x86,0x93,0x94,0x9D,0x9A, 
	0x27,0x20,0x29,0x2E,0x3B,0x3C,0x35,0x32,0x1F,0x18,0x11,0x16,0x03,0x04,0x0D,0x0A, 
	0x57,0x50,0x59,0x5E,0x4B,0x4C,0x45,0x42,0x6F,0x68,0x61,0x66,0x73,0x74,0x7D,0x7A, 
	0x89,0x8E,0x87,0x80,0x95,0x92,0x9B,0x9C,0xB1,0xB6,0xBF,0xB8,0xAD,0xAA,0xA3,0xA4, 
	0xF9,0xFE,0xF7,0xF0,0xE5,0xE2,0xEB,0xEC,0xC1,0xC6,0xCF,0xC8,0xDD,0xDA,0xD3,0xD4, 
	0x69,0x6E,0x67,0x60,0x75,0x72,0x7B,0x7C,0x51,0x56,0x5F,0x58,0x4D,0x4A,0x43,0x44, 
	0x19,0x1E,0x17,0x10,0x05,0x02,0x0B,0x0C,0x21,0x26,0x2F,0x28,0x3D,0x3A,0x33,0x34, 
	0x4E,0x49,0x40,0x47,0x52,0x55,0x5C,0x5B,0x76,0x71,0x78,0x7F,0x6A,0x6D,0x64,0x63, 
	0x3E,0x39,0x30,0x37,0x22,0x25,0x2C,0x2B,0x06,0x01,0x08,0x0F,0x1A,0x1D,0x14,0x13,
	0xAE,0xA9,0xA0,0xA7,0xB2,0xB5,0xBC,0xBB,0x96,0x91,0x98,0x9F,0x8A,0x8D,0x84,0x83, 
	0xDE,0xD9,0xD0,0xD7,0xC2,0xC5,0xCC,0xCB,0xE6,0xE1,0xE8,0xEF,0xFA,0xFD,0xF4,0xF3 
}; 



//计算crc校验值
unsigned	char crc8_calculate(unsigned char * ucPtr, unsigned char ucLen) 
{ 
	unsigned char ucCRC8 = 0;

	while(ucLen--)
	{ 
		ucCRC8 = crc8_tab[ucCRC8^(*ucPtr++)]; 
	} 
	return (~ucCRC8)&0xFF; 

} 

/*
root：一个json对象
strSend：要发送的内容的字符串
开头加上#! (LEN_HEIGH) ()LEN_LOW)
结尾加上 * CRC & \0
返回整个strSend的长度
*/
int generate_send_str(cJSON *root,char * strSend)
{
	char *strJson;
	u16 jsonSize;
	if(root == NULL) 
	{
		printf("error: generate_send_str: not a json!\n");
		return 0;
	}
		
	strSend[0] = '#';
	strSend[1] = '!';
	
	strJson =cJSON_Print(root);  
	jsonSize = strlen(strJson);
	
	if(jsonSize > MAX_JSON_SIZE)
	{
		printf("error: generate_send_str:json size too big! = %d\n",jsonSize);
		return 0;
	}
	strSend[2] = jsonSize>>8 & 0x00ff;
	strSend[3] = jsonSize & 0x00ff;
	strncpy(strSend+4,strJson,jsonSize);
	strSend[4+jsonSize] = '*';
	strSend[5+jsonSize] = crc8_calculate((u8 *)strJson,jsonSize);
	strSend[6+jsonSize] = '&';
	strSend[7+jsonSize] = '\0';
	return jsonSize+7;
}

//------------------------------------------与线性模组通信-----------------------------------------
int resolve_msg(void)  //解析取货单元通过串口一传来的命令，返回businessType，并给目标height全局变量赋值
{
	cJSON *root1, *businessTypeJSON ;  
	char businessType[5];
	int	res;
	//CRC验证
	if(UART5_JSON_CRC != crc8_calculate(UART5_JSON_BUF,strlen((char *)UART5_JSON_BUF)))
	{
		//此时还没有parse root1，可以直接return了
		printf("error! wrong crc\n");
		return MSG_WRONG_CRC;
	}
	root1 = cJSON_Parse((char *)UART5_JSON_BUF);
	
	//printf("%s\n",UART5_JSON_BUF);
	//JSON 有效性判断
	if(root1 == NULL) return MSG_WRONG_JSON;
	
	businessTypeJSON = cJSON_GetObjectItem(root1, "businessType");
	if (!businessTypeJSON) {
//			printf("get businessType faild !\n");
//			printf("Error before: [%s]\n", cJSON_GetErrorPtr());
			res = -1;
	}
	else
	{
		sprintf(businessType, "%s", businessTypeJSON->valuestring);
	}
	//一般情况下返回值是businessType
	res = atoi(businessType);
	if(res != MSG_GO_TO_HEIGHT)
	{
		res = MSG_WRONG_BUSINESS_TYPE; //business type有误，返回错误码
	}
	else 
	{
			cJSON *heightJSON;
		 heightJSON = cJSON_GetObjectItem(root1, "Height");
		 destination_height = heightJSON->valuedouble;
		 cJSON_Delete(heightJSON);
	}
 
	cJSON_Delete(root1);
	cJSON_Delete(businessTypeJSON);
	return res;
}


void on_go_to_height_msg()
{
	const int SUCCESS = 1;
	const int FAIL = 0;
	
	cJSON *root;
	char strSend[MAX_MSG_SIZE];
	u8 strSendLen;
	
	int result = SUCCESS;
	printf("收到消息了！/n");
	if(goTo(destination_height)!= 1) result = FAIL;
	//告知取货单元已经到达相应位置
	root=cJSON_CreateObject();

	cJSON_AddStringToObject(root,"businessType","0023");
	cJSON_AddNumberToObject(root,"Result",result);
	strSendLen = generate_send_str(root,strSend);
	if(strSendLen >0)
	{
		//发送
		send_msg_to_good_getter((uint8_t *)strSend,strSendLen);
	}

	//清理内存
	cJSON_Delete(root);

}
void on_check_msg()
{
	cJSON *root;
	char strSend[MAX_MSG_SIZE];
	u8 strSendLen;
	int errorCode = 200;
	char errorDesc[64] = "fff";
	
	root=cJSON_CreateObject();

	cJSON_AddStringToObject(root,"businessType","0024");
	
	if(!is_distance_receiving  ) //没收到红外优先级更高
	{
		errorCode = 119;
		sprintf(errorDesc,"%s","一段时间没收到距离数据了");
	}
	else if(!is_distance_right  )
	{
		errorCode = 118;
		sprintf(errorDesc,"%s","测距模块返回值是Error");
	}
	cJSON_AddNumberToObject(root,"errorCode",errorCode);
	cJSON_AddStringToObject(root,"errorDesc",errorDesc);
	
	strSendLen = generate_send_str(root,strSend);
	if(strSendLen >0)
	{
		//发送
		send_msg_to_good_getter((uint8_t *)strSend,strSendLen);
	}
	//清理内存
	cJSON_Delete(root);
}
//------------------------------------------前往某个位置-----------------------------------------
int goTo(double destination_height_in_m)
{
	printf("goto");
	double distance2Go;
	SetSpeed(REG_SP3,0);
	StartServo(); 
	
	while(1)   //
	{
		
		if(current_height_in_m < 0 )  //红外数据错误
		{
			printf("error: %f\n",current_height_in_m);
			SetSpeed(REG_SP3,0);
			StopServo();          
			return 0;
		}
		//
		distance2Go = destination_height_in_m - current_height_in_m;

		//上升
		if(distance2Go > 0)
		{
			if(distance2Go >SLOW_RANGE && state != FAR2HIGH)  //还差的远，快点儿走
			{
				printf("FAR2HIGH");
				SetSpeed(REG_SP3,FAST_VELOCITY * DIR_HIGH);
				
			
				state = FAR2HIGH;
			}
			else if(distance2Go > EQUAL_RANGE && distance2Go < SLOW_RANGE && state!= NEAR2HIGH)   
			{
				printf("NEAR2HIGH");
				SetSpeed(REG_SP3,SLOW_VELOCITY* DIR_HIGH);
			
				state = NEAR2HIGH;
			}
			else if(distance2Go < EQUAL_RANGE && distance2Go > -EQUAL_RANGE )   
			{
				printf("arrive");
				SetSpeed(REG_SP3,0);
				StopServo();  
				state = STOP;
				return 1;
			}
		}
		//下降
		else if(distance2Go < 0) 
		{
			if(-distance2Go >SLOW_RANGE && state != FAR2LOW)  //还差的远，快点儿走
			{
				printf("FAR2LOW");
				SetSpeed(REG_SP3,FAST_VELOCITY * DIR_LOW);
				
				state = FAR2LOW;
			}
			else if(-distance2Go > EQUAL_RANGE && -distance2Go < SLOW_RANGE && state!= NEAR2LOW)   
			{
				printf("NEAR2LOW");
				SetSpeed(REG_SP3,SLOW_VELOCITY* DIR_LOW);
			
				state = NEAR2LOW;
			}
			else if(-distance2Go < EQUAL_RANGE && distance2Go > -EQUAL_RANGE )   
			{
				printf("arrive");
				SetSpeed(REG_SP3,0);
				StopServo();  
				state = STOP;
				return 1;
			}
		}

	}
	
}
