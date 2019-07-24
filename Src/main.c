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
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
__IO uint8_t Rx_Buf[50];       //�������ݻ���

/* ��չ���� ------------------------------------------------------------------*/
		 extern u8 UART4_RX_BUF[64]; //���յ�������

/* ˽�к���ԭ�� --------------------------------------------------------------*/
double current_height_in_m;
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

  HAL_UART_Receive_DMA(&husartx_rs485,(uint8_t*)&husart_debug.Instance->DR, 1);// Data Direction: 485 --> USART1
  HAL_UART_Receive_DMA(&husart_debug,(uint8_t*)&husartx_rs485.Instance->DR, 1);// Data Direction: USART1 --> 485
  
  /* Disable the Half transfer complete interrupt */
  __HAL_DMA_DISABLE_IT(&hdma_debug_rx, DMA_IT_HT);
  __HAL_DMA_DISABLE_IT(&hdma_debug_rx, DMA_IT_TE);
  
  __HAL_DMA_DISABLE_IT(&hdma_rs485_rx, DMA_IT_HT);
  __HAL_DMA_DISABLE_IT(&hdma_rs485_rx, DMA_IT_TE);
  
	/* ��ʼ�����ģ�� */
	HAL_UART_Transmit(&UART4_Handler,"iFACM:0",7,0x0F);
  /* ��ʼ��ASDA-B2����,����Ϊ�ٶ�ģʽ*/
  ASDAB2_Init();          
  /* ����SP3�ٶ�ֵΪ600*0.1r/min  60r/min */
  SetSpeed(REG_SP3,100);      
  /* �����ŷ� */
  StartServo();          
  /* ����ѭ�� */
  while (1)
  {
//    uint16_t j = 100;
//    for(i=0; i<20;i++)
//    {
//      HAL_Delay(500);          
//      SetSpeed(REG_SP3,(j+=500)*dir);   // ����Ϊ��ת
//    }
//    for(i=0; i<20;i++)
//    {
//      HAL_Delay(500);          
//      SetSpeed(REG_SP3,(j-=500)*dir);   // ����Ϊ��ת
//    }
//    StopServo();              // ֹͣ�ŷ�
//    HAL_Delay(2000);      
//    dir = -dir;
//    SetSpeed(REG_SP3,j*dir);
//    StartServo();             // ���������ŷ����
		

		printf("current_height_in_m = %f \ndStr = %s\n\n",current_height_in_m,UART4_RX_BUF);	
		HAL_Delay(1000);     
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
}

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
