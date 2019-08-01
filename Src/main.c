/**
  ******************************************************************************
  * 文件名程: main.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2017-4-14
  * 功    能: 伺服驱动器速度模式控制电机正转反转
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

#include "head.h"
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
__IO uint8_t Rx_Buf[50];       //接收数据缓存


extern uint8_t UART4_RX_BUF[64]; //接收到的数据

double destination_height;


uint8_t new_msg = 0;
uint8_t USART1_JSON_BUF[256]; //接收到的数据
uint8_t USART1_RX_STA=0; 
uint16_t USART1_JSON_SIZE = 0;
uint16_t USART1_JSON_INDEX = 0;
uint8_t	USART1_JSON_CRC = 0;

int is_distance_receiving = 1;  //表示红外一直在接收数据
int is_distance_right = 1;      //表示红外接收数据正常，为D = ***m，而非Error
uint32_t distanceModuleMonitor = 0;

/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 系统时钟配置
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;  // 外部晶振，8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;  // 9倍频，得到72MHz主时钟
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // 系统时钟：72MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHB时钟：72MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;               // APB1时钟：36MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;               // APB2时钟：72MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

 	// HAL_RCC_GetHCLKFreq()/1000    1ms中断一次
	// HAL_RCC_GetHCLKFreq()/100000	 10us中断一次
	// HAL_RCC_GetHCLKFreq()/1000000 1us中断一次
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);  // 配置并启动系统滴答定时器
  /* 系统滴答定时器时钟源 */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  /* 系统滴答定时器中断优先级配置 */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/**
  * 函数功能: 主函数.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
int main(void)
{  

  /* 复位所有外设，初始化Flash接口和系统滴答定时器 */
  HAL_Init();
  /* 配置系统时钟 */
  SystemClock_Config();

  /* 初始化串口并配置串口中断优先级 */
  MX_DEBUG_USART_Init();
  RS485_USARTx_Init();
	uart4_init(115200);
	/* timer init */
	TIM3_Init(5000-1,7200-1);       	//定时器3初始化，定时器时钟为84M，分频系数为8400-1，
										//所以定时器3的频率为72M/7200=10K，自动重装载为5000-1，那么定时器周期就是500ms
  HAL_UART_Receive_DMA(&husartx_rs485,(uint8_t*)&husart_debug.Instance->DR, 1);// Data Direction: 485 --> USART1
  HAL_UART_Receive_DMA(&husart_debug,(uint8_t*)&husartx_rs485.Instance->DR, 1);// Data Direction: USART1 --> 485
  
  /* Disable the Half transfer complete interrupt */
  __HAL_DMA_DISABLE_IT(&hdma_debug_rx, DMA_IT_HT);
  __HAL_DMA_DISABLE_IT(&hdma_debug_rx, DMA_IT_TE);
  
  __HAL_DMA_DISABLE_IT(&hdma_rs485_rx, DMA_IT_HT);
  __HAL_DMA_DISABLE_IT(&hdma_rs485_rx, DMA_IT_TE);
  
	/* 初始化测距模块 */
	HAL_UART_Transmit(&UART4_Handler,(uint8_t *)"iFACM:0",7,0x0F);
  /* 初始化ASDA-B2参数,配置为速度模式*/
  ASDAB2_Init();          
  /* 设置SP3速度值为600*0.1r/min  60r/min */
//  SetSpeed(REG_SP3,-600);      
//  /* 启动伺服 */
//  StartServo();    


		//goTo(0.8);
  /* 无限循环 */
  while (1)
  {
		 
//		HAL_Delay(2000);
//		SetSpeed(REG_SP3,600);      
//		HAL_Delay(2000);
//		SetSpeed(REG_SP3,-600); 

//		goTo(0.7);
//		HAL_Delay(2000);
//		goTo(0.5);
//		HAL_Delay(2000);
		
		
		
		//测距模块监视
		distanceModuleMonitor++;
		if(distanceModuleMonitor>655344)
		{
			//printf("too long without distance data received!\n");
			is_distance_receiving = 0;
		}
		else
			is_distance_receiving = 1;
		
		//如果测距模块工作状态不正常，
		if(!is_distance_receiving || !is_distance_right)
		{
			//电机运动逻辑得考虑啊
			;
		}
		
		//处理取货单元发来的消息
		if(new_msg)
		{
			//printf("%s\n",USART1_JSON_BUF);
			switch(resolve_msg())
			{
				case(MSG_GO_TO_HEIGHT):{on_go_to_height_msg();break;}
			}
			new_msg = 0;
		}






		
  }
}

//----------------------------Yu Qiao ---------------------------//







/**
  * 函数功能: 串口接收回调函数
  * 输入参数: UARTHandle:串口句柄
  * 返 回 值: 无
  * 说    明: 接收到从机的反馈数据之后,分包并存放到Rx_Buf里面,一次只能接收一帧数据
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  static uint8_t i = 0;
  if(UartHandle->Instance == RS485_USARTx)
  {
    Rx_Buf[i] = husartx_rs485.Instance->DR;
    if(Rx_Buf[0] == ':')
    { 
      i++;
      if(Rx_Buf[i-1] == 0x0A)
      {  
        if(Rx_Buf[i-2] ==0x0D)
        {
          Rx_Buf[i] = '\0';         // 手动添加结束符
          UsartState = Ready;       // 接收完成,通讯待机
          i = 0;  
        }
      }
    }
    else i = 0;
  }
	//与取货单元通信的串口中断处理函数
#define UART_IDLE 0
#define WELL 1 //收到#
#define EXCLAMATION 2 //收到！
#define HIGH_SIZE 3
#define LOW_SIZE 4
#define JSON_END 5
#define STAR 6
#define CRC_CHECK 7
#define AND 8
	if(UartHandle->Instance == DEBUG_USARTx)
  {
    uint8_t res = husart_debug.Instance->DR;
    switch(USART1_RX_STA)
		{
			case(UART_IDLE):	{if(res == '#') 							USART1_RX_STA = WELL;					break;}
			case(WELL):				{if(res == '!') 							USART1_RX_STA = EXCLAMATION;	break;}
			case(EXCLAMATION):{USART1_JSON_SIZE = res<<8; 	USART1_RX_STA = HIGH_SIZE;		break;}
			case(HIGH_SIZE):	{USART1_JSON_SIZE += res; 		USART1_RX_STA = LOW_SIZE;			break;}
			case(LOW_SIZE):		
			{
				if(USART1_JSON_INDEX < USART1_JSON_SIZE -1)
				{
					USART1_JSON_BUF[USART1_JSON_INDEX]=res; 
					USART1_JSON_INDEX++;
				}
				else if(USART1_JSON_INDEX == USART1_JSON_SIZE -1) //JSON的最后一个字节了
				{
					USART1_JSON_BUF[USART1_JSON_INDEX]=res; 
					USART1_JSON_BUF[USART1_JSON_SIZE]= '\0'; 
					USART1_JSON_INDEX = 0;
					USART1_RX_STA = JSON_END;
					
				}					
				break;
			}
			case(JSON_END):		{if(res == '*')								USART1_RX_STA = STAR; 				break;}
			case(STAR):				{USART1_JSON_CRC = res; 			USART1_RX_STA = CRC_CHECK;		break;}
			case(CRC_CHECK):  
			{
				if(res == '&')
				{
					USART1_RX_STA = UART_IDLE;
					new_msg = 1;
				}
				break;
			}
		}	
		
  }
}

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
