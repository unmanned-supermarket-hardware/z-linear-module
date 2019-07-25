/**
  ******************************************************************************
  * �ļ�����: main.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-4-14
  * ��    ��: �ŷ��������ٶ�ģʽ���Ƶ����ת��ת
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
#include "stm32f1xx_hal.h"
#include "usart/bsp_debug_usart.h"
#include "string.h"
#include "ASDA_B2/bsp_ASDA_B2.h"
#include "timer/timer.h"
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
__IO uint8_t Rx_Buf[50];       //�������ݻ���


extern u8 UART4_RX_BUF[64]; //���յ�������


double current_height_in_m;

//�ٶȺ;��붨��
#define FAST_VELOCITY 1500  //����
#define SLOW_VELOCITY 600   //����
#define SLOW_RANGE 0.05  //�����С��0.05m��ʱ�����
#define EQUAL_RANGE 0.005  //��������0.005m��ʱ����Ϊ�������

//״̬����
#define GO -2
#define STOP -1
#define FAR2HIGH 0    
#define NEAR2HIGH 1
#define TOO_HIGH 2
#define FAR2LOW 3
#define NEAR2LOW 4
#define TOO_LOW 5

//������
#define DIR_HIGH -1
#define DIR_LOW 1

//����ģ���˶���ĳ���̶�λ��
int last_state = STOP;
int current_state = STOP;
int state = STOP;
void goTo(double destination_height_in_m);



/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ϵͳʱ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;  // �ⲿ����8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;  // 9��Ƶ���õ�72MHz��ʱ��
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // ϵͳʱ�ӣ�72MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHBʱ�ӣ�72MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;               // APB1ʱ�ӣ�36MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;               // APB2ʱ�ӣ�72MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

 	// HAL_RCC_GetHCLKFreq()/1000    1ms�ж�һ��
	// HAL_RCC_GetHCLKFreq()/100000	 10us�ж�һ��
	// HAL_RCC_GetHCLKFreq()/1000000 1us�ж�һ��
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);  // ���ò�����ϵͳ�δ�ʱ��
  /* ϵͳ�δ�ʱ��ʱ��Դ */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  /* ϵͳ�δ�ʱ���ж����ȼ����� */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/**
  * ��������: ������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
int main(void)
{  
  uint8_t i= 0;
  int8_t dir = 1;
  /* ��λ�������裬��ʼ��Flash�ӿں�ϵͳ�δ�ʱ�� */
  HAL_Init();
  /* ����ϵͳʱ�� */
  SystemClock_Config();

  /* ��ʼ�����ڲ����ô����ж����ȼ� */
  MX_DEBUG_USART_Init();
  RS485_USARTx_Init();
	uart4_init(115200);
	/* timer init */
	TIM3_Init(5000-1,7200-1);       	//��ʱ��3��ʼ������ʱ��ʱ��Ϊ84M����Ƶϵ��Ϊ8400-1��
										//���Զ�ʱ��3��Ƶ��Ϊ72M/7200=10K���Զ���װ��Ϊ5000-1����ô��ʱ�����ھ���500ms
  HAL_UART_Receive_DMA(&husartx_rs485,(uint8_t*)&husart_debug.Instance->DR, 1);// Data Direction: 485 --> USART1
  HAL_UART_Receive_DMA(&husart_debug,(uint8_t*)&husartx_rs485.Instance->DR, 1);// Data Direction: USART1 --> 485
  
  /* Disable the Half transfer complete interrupt */
  __HAL_DMA_DISABLE_IT(&hdma_debug_rx, DMA_IT_HT);
  __HAL_DMA_DISABLE_IT(&hdma_debug_rx, DMA_IT_TE);
  
  __HAL_DMA_DISABLE_IT(&hdma_rs485_rx, DMA_IT_HT);
  __HAL_DMA_DISABLE_IT(&hdma_rs485_rx, DMA_IT_TE);
  
	/* ��ʼ�����ģ�� */
	HAL_UART_Transmit(&UART4_Handler,(u8 *)"iFACM:0",7,0x0F);
  /* ��ʼ��ASDA-B2����,����Ϊ�ٶ�ģʽ*/
  ASDAB2_Init();          
  /* ����SP3�ٶ�ֵΪ600*0.1r/min  60r/min */
//  SetSpeed(REG_SP3,-600);      
//  /* �����ŷ� */
//  StartServo();    


		//goTo(0.8);
  /* ����ѭ�� */
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
		
		if(current_height_in_m < 0 )  //�������ݴ���
		{
			printf("error: %f\n",current_height_in_m);
			SetSpeed(REG_SP3,0);
			StopServo();          
			return;
		}
		//
		distance2Go = destination_height_in_m - current_height_in_m;

		//����
		if(distance2Go > 0)
		{
			if(distance2Go >SLOW_RANGE && state != FAR2HIGH)  //�����Զ��������
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
		//�½�
		else if(distance2Go < 0) 
		{
			if(-distance2Go >SLOW_RANGE && state != FAR2LOW)  //�����Զ��������
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
  * ��������: ���ڽ��ջص�����
  * �������: UARTHandle:���ھ��
  * �� �� ֵ: ��
  * ˵    ��: ���յ��ӻ��ķ�������֮��,�ְ�����ŵ�Rx_Buf����,һ��ֻ�ܽ���һ֡����
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
          Rx_Buf[i] = '\0';         // �ֶ���ӽ�����
          UsartState = Ready;       // �������,ͨѶ����
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

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
