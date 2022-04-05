/*
 * stm32f407xx_usart_driver.h
 *
 *  Created on: 24-Mar-2022
 *      Author: pro
 */

#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_

#include <stdint.h>
#include "stm32f407xx.h"

/* USART Configuration Structure */
typedef struct{
	uint8_t	USART_Mode ;
	uint32_t	USART_Baud ;
	uint8_t	USART_NoOfStopBits ;
	uint8_t	USART_WordLength ;
	uint8_t	USART_ParityControl ;
	uint8_t	USART_HWFlowControl ;
}USART_Config_t;

/*
 * Handle Structure for USARTx peripheral
 */
typedef struct{
	USART_RegDef_t *pUSARTx;
	USART_Config_t USART_Config ;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxBusyState;
	uint8_t RxBusyState;
}USART_Handle_t;

// Possible USART Mode
#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TXRX  2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200				1200
#define USART_STD_BAUD_2400				2400
#define USART_STD_BAUD_9600				9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 				2000000
#define USART_STD_BAUD_3M 				3000000


/*
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE  0

/*
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/*
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

/*USART possible flag name mcros */

#define USART_FLAG_CTS	(1 << USART_SR_CTS)
#define USART_FLAG_LBD	(1 << USART_SR_LBD)
#define USART_FLAG_TXE	(1 << USART_SR_TXE)
#define USART_FLAG_TC	(1 << USART_SR_TC)
#define USART_FLAG_RXNE	(1 << USART_SR_RXNE)
#define USART_FLAG_IDLE	(1 << USART_SR_IDLE)
#define USART_FLAG_ORE	(1 << USART_SR_ORE)
#define USART_FLAG_NF	(1 << USART_SR_NF)
#define USART_FLAG_FE	(1 << USART_SR_FE)
#define USART_FLAG_PE	(1 << USART_SR_PE)

/*
 * Application states
 */
#define USART_BUSY_IN_RX 1
#define USART_BUSY_IN_TX 2
#define USART_READY 0

uint32_t RCC_GetPCLK1Vlaue(void );
uint32_t RCC_GetPllClk(void ) ;

void USART_PeriClockControl(USART_RegDef_t *pUSARTx , uint8_t EnOrDi) ;
void USART_PeripheralControl(USART_RegDef_t *pUSARTx , uint8_t EnOrDi) ;

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint8_t StatusFlagName) ;
void USART_ClearFlag(USART_RegDef_t *pUSARTx , uint16_t StatusFlagName) ;

void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len);

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle) ;

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi) ;
void USART_IRQPrioriyConfig(uint8_t IRQNumber, uint8_t IRQPriority) ;

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);




#endif /* INC_STM32F407XX_USART_DRIVER_H_ */

