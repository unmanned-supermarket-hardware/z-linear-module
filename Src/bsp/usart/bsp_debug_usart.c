/**
  ******************************************************************************
  * �ļ�����: bsp_debug_usart.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2015-10-04
  * ��    ��: ���ص��Դ��ڵײ���������Ĭ��ʹ��USART1
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
#include "usart/bsp_debug_usart.h"
#include <string.h> 		 //strlen						 
#include <stdlib.h>	        //atof()  malloc()
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/

u8 aRxBuffer4[RXBUFFERSIZE];//HAL??????????
UART_HandleTypeDef UART4_Handler; //UART?? 


UART_HandleTypeDef husart_debug;
DMA_HandleTypeDef hdma_debug_rx;

DMA_HandleTypeDef hdma_rs485_rx;
UART_HandleTypeDef husartx_rs485;
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/

/**
  * ��������: ����Ӳ����ʼ������
  * �������: huart�����ھ������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(huart->Instance==DEBUG_USARTx)
  {
    /* ��������ʱ��ʹ�� */
    DEBUG_USART_RCC_CLK_ENABLE();
    USARTx_RCC_DMAx_CLK_ENABLE();

    /* �������蹦��GPIO���� */
    GPIO_InitStruct.Pin = DEBUG_USARTx_Tx_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DEBUG_USARTx_Tx_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DEBUG_USARTx_Rx_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DEBUG_USARTx_Rx_GPIO, &GPIO_InitStruct);
    
    /* ��ʼ��DMA���� */  
    hdma_debug_rx.Instance = USARTx_DMAx_CHANNELn;
    hdma_debug_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_debug_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_debug_rx.Init.MemInc = DMA_MINC_DISABLE;
    hdma_debug_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_debug_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_debug_rx.Init.Mode = DMA_CIRCULAR;
    hdma_debug_rx.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_debug_rx);

    __HAL_LINKDMA(huart,hdmarx,hdma_debug_rx);

  }
  if(huart->Instance==RS485_USARTx)  
  {
    /* ��������ʱ��ʹ�� */
    RS485_USARTx_GPIO_ClK_ENABLE();
    RS485_REDE_GPIO_ClK_ENABLE();
    RS485_RCC_DMAx_CLK_ENABLE();
    
    /* �������蹦��GPIO���� */
    GPIO_InitStruct.Pin = RS485_USARTx_Tx_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(RS485_USARTx_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RS485_USARTx_Rx_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(RS485_USARTx_PORT, &GPIO_InitStruct);
    
    /* SP3485E��������ʹ�ܿ������ų�ʼ�� */
    HAL_GPIO_WritePin(RS485_REDE_PORT,RS485_REDE_PIN,GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = RS485_REDE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(RS485_REDE_PORT, &GPIO_InitStruct);
    
    /* ��ʼ��DMA���� */  
    hdma_rs485_rx.Instance = RS485_DMAx_CHANNELn;
    hdma_rs485_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_rs485_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_rs485_rx.Init.MemInc = DMA_MINC_DISABLE;
    hdma_rs485_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rs485_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_rs485_rx.Init.Mode = DMA_CIRCULAR;
    hdma_rs485_rx.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_rs485_rx);

    __HAL_LINKDMA(huart,hdmarx,hdma_rs485_rx);
  }
	
	if(huart->Instance==UART4)//?????1,????1 MSP???
	{
		//GPIO????
	GPIO_InitTypeDef GPIO_Initure;
		__HAL_RCC_GPIOC_CLK_ENABLE();			//??GPIOA??
		__HAL_RCC_UART4_CLK_ENABLE();			//??USART1??
		__HAL_RCC_AFIO_CLK_ENABLE();
	
		GPIO_Initure.Pin=GPIO_PIN_10;			//PA9
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//??????
		GPIO_Initure.Pull=GPIO_PULLUP;			//??
		GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//??
		HAL_GPIO_Init(GPIOC,&GPIO_Initure);	   	//???PA9

		GPIO_Initure.Pin=GPIO_PIN_11;			//PA10
		GPIO_Initure.Mode=GPIO_MODE_AF_INPUT;	//????????????!	
		HAL_GPIO_Init(GPIOC,&GPIO_Initure);	   	//???PA10
		

		HAL_NVIC_EnableIRQ(UART4_IRQn);				//??USART1????
		HAL_NVIC_SetPriority(UART4_IRQn,1,1);			//?????3,????3

	
	}
}

/**
  * ��������: ����Ӳ������ʼ������
  * �������: huart�����ھ������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

  if(huart->Instance==DEBUG_USARTx)
  {
    /* ��������ʱ�ӽ��� */
    DEBUG_USART_RCC_CLK_DISABLE();
  
    /* �������蹦��GPIO���� */
    HAL_GPIO_DeInit(DEBUG_USARTx_Tx_GPIO, DEBUG_USARTx_Tx_GPIO_PIN);
    HAL_GPIO_DeInit(DEBUG_USARTx_Rx_GPIO, DEBUG_USARTx_Rx_GPIO_PIN);
    
    /* �����жϽ��� */
    HAL_NVIC_DisableIRQ(DEBUG_USART_IRQn);
  }
}


/**
  * ��������: ���ڲ�������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void MX_DEBUG_USART_Init(void)
{
  /* ʹ�ܴ��ڹ�������GPIOʱ�� */
  DEBUG_USARTx_GPIO_ClK_ENABLE();
  
  husart_debug.Instance = DEBUG_USARTx;
  husart_debug.Init.BaudRate = DEBUG_USARTx_BAUDRATE;
  husart_debug.Init.WordLength = UART_WORDLENGTH_8B;
  husart_debug.Init.StopBits = UART_STOPBITS_2;
  husart_debug.Init.Parity = UART_PARITY_NONE;
  husart_debug.Init.Mode = UART_MODE_TX_RX;
  husart_debug.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  husart_debug.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&husart_debug);
    
  HAL_NVIC_SetPriority(DEBUG_USART_IRQn, 1, 2);
  HAL_NVIC_EnableIRQ(DEBUG_USART_IRQn); 
  HAL_NVIC_SetPriority(USARTx_DMAx_CHANNELn_IRQn, 1, 3);
  HAL_NVIC_EnableIRQ(USARTx_DMAx_CHANNELn_IRQn); 
}

/**
  * ��������: �ض���c�⺯��printf��DEBUG_USARTx
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&husart_debug, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

/**
  * ��������: �ض���c�⺯��getchar,scanf��DEBUG_USARTx
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
int fgetc(FILE * f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&husart_debug,&ch, 1, 0xffff);
  return ch;
}

/**
  * ��������: ���ڲ�������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void RS485_USARTx_Init(void)
{ 
  /* RS485ʱ��ʹ�� */
  RS485_USART_RCC_CLK_ENABLE();
  
  husartx_rs485.Instance = RS485_USARTx;
  husartx_rs485.Init.BaudRate = RS485_USARTx_BAUDRATE;
  husartx_rs485.Init.WordLength = UART_WORDLENGTH_8B;
  husartx_rs485.Init.StopBits = UART_STOPBITS_2;
  husartx_rs485.Init.Parity = UART_PARITY_NONE;
  husartx_rs485.Init.Mode = UART_MODE_TX_RX;
  husartx_rs485.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  husartx_rs485.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&husartx_rs485);
  
  HAL_NVIC_SetPriority(RS485_USARTX_IRQn, 1, 2);
  HAL_NVIC_EnableIRQ(RS485_USARTX_IRQn);
  /* DMA interrupt configuration */
  HAL_NVIC_SetPriority(RS485_DMAx_CHANNELn_IRQn, 1, 3);
  HAL_NVIC_EnableIRQ(RS485_DMAx_CHANNELn_IRQn);  
}
//-------------------------------------------------
void uart4_init(u32 bound)
{	
	//UART ?????
	UART4_Handler.Instance=UART4;					    //USART
	UART4_Handler.Init.BaudRate=bound;				    //???
	UART4_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //???8?????
	UART4_Handler.Init.StopBits=UART_STOPBITS_1;	    //?????
	UART4_Handler.Init.Parity=UART_PARITY_NONE;		    //??????
	UART4_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //?????
	UART4_Handler.Init.Mode=UART_MODE_TX_RX;		    //????
	HAL_UART_Init(&UART4_Handler);					    //HAL_UART_Init()???UART1
	
	HAL_UART_Receive_IT(&UART4_Handler, (u8 *)aRxBuffer4, RXBUFFERSIZE);//??????????:???UART_IT_RXNE,?????????????????????
  
}
 
#define USART_REC_LEN  			200  	//�����������ֽ��� 200
u8 UART4_RX_BUF[64]; //���յ�������
u16 UART4_RX_STA=0; 
u8 UART4_COUNT = 0;
double d =0;
char dStr[5];
u8 UART4_RX_FLAG = 0;
extern double current_height_in_m;
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
void UART4_IRQHandler(void)     
{
	if((__HAL_UART_GET_FLAG(&UART4_Handler,UART_FLAG_RXNE)!=RESET))  //????(?????????0x0d 0x0a??)
	{
		u8 res=UART4->DR; 
		char strTemp[64];
		//HAL_UART_Transmit(&UART4_Handler,&res,1,1000);	//????????
		if((UART4_RX_STA&0x8000)==0)//����δ���
			{
				if(UART4_RX_STA&0x4000)//���յ���0x0d
				{
					if(res!=0x0a)UART4_RX_STA=0;//���մ���,���¿�ʼ
					else 
					{
						UART4_RX_STA|=0x8000;	//���������
						UART4_RX_BUF[UART4_RX_STA&0X3FFF]=res;
						//usart1_sendString((char *)UART4_RX_BUF,UART4_COUNT+1);
						//-------------------
						UART4_RX_BUF[(UART4_RX_STA&0X3FFF)-2] = '\0';
						strncpy(dStr,(char *)(UART4_RX_BUF+2),6);
						sprintf(strTemp,"%s\r\n",dStr);
						
						//usart1_sendString(strTemp,strlen(strTemp));
						if(UART4_RX_BUF[0] =='D')
						{
				
							current_height_in_m = atof(dStr);
							//current_height_mm = 5;
						}
							
						else
							current_height_in_m =-1;
						//-------------------

						UART4_RX_STA = 0;
						UART4_COUNT = 0;
					}
					
				}else //��û�յ�0X0D
				{	
					if(res==0x0d)
					{
						UART4_RX_STA|=0x4000;
						UART4_RX_BUF[UART4_RX_STA&0X3FFF]=res;
						UART4_RX_STA++;
						UART4_COUNT ++;
					}
					else
					{
						UART4_RX_BUF[UART4_RX_STA&0X3FFF]=res;
						UART4_RX_STA++;
						UART4_COUNT ++;
						if(UART4_RX_STA>(USART_REC_LEN-1))UART4_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
					}		 
				}
			}
	}
}	

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
