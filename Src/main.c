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

#include "head.h"
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
__IO uint8_t Rx_Buf[50];       //�������ݻ���


extern uint8_t UART4_RX_BUF[64]; //���յ�������

extern uint8_t new_msg ;
double destination_height;

int is_distance_receiving = 1;  //��ʾ����һֱ�ڽ�������
int is_distance_right = 1;      //��ʾ�����������������ΪD = ***m������Error
uint32_t distanceModuleMonitor = 0;

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

  /* ��λ�������裬��ʼ��Flash�ӿں�ϵͳ�δ�ʱ�� */
  HAL_Init();
  /* ����ϵͳʱ�� */
  SystemClock_Config();

  /* ��ʼ�����ڲ����ô����ж����ȼ� */
  MX_DEBUG_USART_Init();
  RS485_USARTx_Init();
	uart4_init(115200);
	uart5_init(115200);
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
	HAL_UART_Transmit(&UART4_Handler,(uint8_t *)"iFACM:0",7,0x0F);
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
//		goTo(0.7);
//		HAL_Delay(2000);
//		goTo(0.5);
//		HAL_Delay(2000);
		
		//���ģ�����
		distanceModuleMonitor++;
		if(distanceModuleMonitor>655344)
		{
			//printf("too long without distance data received!\n");
			is_distance_receiving = 0;
		}
		else
			is_distance_receiving = 1;
		
		//������ģ�鹤��״̬��������
		if(!is_distance_receiving || !is_distance_right)
		{
			//����˶��߼��ÿ��ǰ�
			;
		}
		
		//����ȡ����Ԫ��������Ϣ
		if(new_msg)
		{
			printf("new_msg == 1/n");
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
    
		
  }
}

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
