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
#include "stm32f1xx_hal.h"
#include "usart/bsp_debug_usart.h"
#include "string.h"
#include "ASDA_B2/bsp_ASDA_B2.h"
#include "timer/timer.h"
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
__IO uint8_t Rx_Buf[50];       //接收数据缓存


extern u8 UART4_RX_BUF[64]; //接收到的数据


double current_height_in_m;

//速度和距离定义
#define FAST_VELOCITY 1500  //快速
#define SLOW_VELOCITY 600   //慢速
#define SLOW_RANGE 0.05  //距离差小于0.05m的时候减速
#define EQUAL_RANGE 0.005  //距离差不超过0.005m的时候认为距离相等

//状态定义
#define GO -2
#define STOP -1
#define FAR2HIGH 0    
#define NEAR2HIGH 1
#define TOO_HIGH 2
#define FAR2LOW 3
#define NEAR2LOW 4
#define TOO_LOW 5

//方向定义
#define DIR_HIGH -1
#define DIR_LOW 1

//控制模组运动到某个固定位置
int last_state = STOP;
int current_state = STOP;
int state = STOP;
void goTo(double destination_height_in_m);



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
  uint8_t i= 0;
  int8_t dir = 1;
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
	HAL_UART_Transmit(&UART4_Handler,(u8 *)"iFACM:0",7,0x0F);
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
		if(state == GO)
		{
			goTo(0.5);
		}
		goTo(0.3);
		HAL_Delay(2000);
		goTo(0.5);
		HAL_Delay(2000);
  }
}

//----------------------------Yu Qiao ---------------------------//






void goTo(double destination_height_in_m)
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
			return;
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
				return;
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
				return;
			}
		}

	}
	
}

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
	if(UartHandle->Instance == DEBUG_USARTx)
  {
    u8 temp = husart_debug.Instance->DR;
    if(temp == 'g')
		{
			state = GO;
		}
  }
}

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
