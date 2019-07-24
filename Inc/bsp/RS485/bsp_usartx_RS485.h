#ifndef __BSP_USARTX_RS485_H__
#define __BSP_USARTX_RS485_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/
#define RS485_USARTx                                 USART3
#define RS485_USARTx_BAUDRATE                        38400
#define RS485_USART_RCC_CLK_ENABLE()                 __HAL_RCC_USART3_CLK_ENABLE()
#define RS485_USART_RCC_CLK_DISABLE()                __HAL_RCC_USART3_CLK_DISABLE()

#define RS485_USARTx_GPIO_ClK_ENABLE()               __HAL_RCC_GPIOB_CLK_ENABLE()
#define RS485_USARTx_PORT                            GPIOB
#define RS485_USARTx_Tx_PIN                          GPIO_PIN_10
#define RS485_USARTx_Rx_PIN                          GPIO_PIN_11

#define RS485_USARTX_IRQn                            USART3_IRQn

#define RS485_REDE_GPIO_ClK_ENABLE()                 __HAL_RCC_GPIOB_CLK_ENABLE()
#define RS485_REDE_PORT                              GPIOB
#define RS485_REDE_PIN                               GPIO_PIN_2
#define RS485_RX_MODE()                              HAL_GPIO_WritePin(RS485_REDE_PORT,RS485_REDE_PIN,GPIO_PIN_RESET)
#define RS485_TX_MODE()                              HAL_GPIO_WritePin(RS485_REDE_PORT,RS485_REDE_PIN,GPIO_PIN_SET)

/* ��չ���� ------------------------------------------------------------------*/
extern UART_HandleTypeDef husartx_rs485;

/* �������� ------------------------------------------------------------------*/
void RS485_USARTx_Init(void);


#endif  /* __BSP_USARTX_RS485_H__ */

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
