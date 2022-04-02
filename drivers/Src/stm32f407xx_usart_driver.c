/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: 24-Mar-2022
 *      Author: pro
 */


#include "stm32f407xx_usart_driver.h"

uint32_t RCC_GetPllClk(void ){
	return 0 ;
}

uint32_t RCC_GetPCLK1Vlaue(void ){
	uint16_t AHB_PreScaler[9] = {2,4,8,16,32,64,128,256,512} ;
	uint8_t APB1_PreScaler[4] = {2,4,8,16} ;
	uint32_t psclk ;
	uint8_t clksrc ,  temp  , ahbp , apb1p ;
	uint32_t SystemClk = 0 ;
	clksrc = ((RCC->CFGR >> 2) & 0x3 ); 	// check the clock source

	if (clksrc == 0 ) {
		SystemClk = 16000000 ;	// hsi is selected
	}else if (clksrc == 1 ){
		SystemClk = 8000000 ;	// hse is selected
	}else if (clksrc == 2 ){
		SystemClk = RCC_GetPllClk(); // pll is selected
	}

	// get the value of ahb prescalar value
	temp = ((RCC->CFGR >> 4 ) & 0xf) ;
	if (temp <  8 ) {
		ahbp = 1 ;
	}else  {
		ahbp = AHB_PreScaler[temp - 8] ;
	}
	// get the value of apb1 prescalar value
	temp = ((RCC->CFGR >> 10) & 0x7) ;

	if (temp <  4 ) {
		apb1p = 1 ;
	}else  {
		apb1p = APB1_PreScaler[temp - 4] ;
	}
	psclk =	( (SystemClk / ahbp) / apb1p) ;
	return psclk ;

}


void USART_PeriClockControl(USART_RegDef_t *pUSARTx , uint8_t EnOrDi) {
if (EnOrDi == ENABLE) {
	if (pUSARTx == USART1) {
		USART1_CLOCK_ENABLE();
	}else if(pUSARTx == USART2){
		USART2_CLOCK_ENABLE();
	}else if(pUSARTx == USART3){
		USART3_CLOCK_ENABLE();
	}
 }else if(EnOrDi == DISABLE){
   if (pUSARTx == USART1) {
		USART1_CLOCK_DISABLE();
	}else if(pUSARTx == USART2){
		USART2_CLOCK_DISABLE();
	}else if(pUSARTx == USART3){
		USART3_CLOCK_DISABLE();
	}
  }
}

void USART_PeripheralControl(USART_RegDef_t *pUSARTx , uint8_t EnOrDi) {
if (EnOrDi == ENABLE) {
	pUSARTx->CR1 |= (1 << USART_CR1_UE) ;
 }else{
	 pUSARTx->CR1 &= ~(1 << USART_CR1_UE) ;
 }
}

void USART_DeInit(USART_RegDef_t *pUSARTx){
 if (pUSARTx == USART1) {
	USART1_REG_RESET() ;
 }else if(pUSARTx == USART2){
	USART2_REG_RESET() ;
 }else if(pUSARTx == USART3){
	USART3_REG_RESET() ;
 }
}

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint8_t StatusFlagName) {
	if ((pUSARTx->SR & (StatusFlagName))) {
		return FLAG_SET ;
	}
	return FLAG_RESET ;
}

void USART_ClearFlag(USART_RegDef_t *pUSARTx , uint16_t StatusFlagName) {
	pUSARTx->SR &= ~(1 << StatusFlagName) ;
}


void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate){
	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

  uint32_t tempreg=0;

  //Get the value of APB bus clock in to the variable PCLKx
  if(pUSARTx == USART1 || pUSARTx == USART6)
  {
	   //USART1 and USART6 are hanging on APB2 bus
//	   PCLKx = RCC_GetPCLK2Vlaue();
  }else
  {
	   PCLKx = RCC_GetPCLK1Vlaue();
  }

  //Check for OVER8 configuration bit
  if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
  {
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
  }else
  {
	   //over sampling by 16
	  usartdiv = ((25 * PCLKx) / (4 *BaudRate));
  }

  //Calculate the Mantissa part
  M_part = usartdiv/100;

  //Place the Mantissa part in appropriate bit position . refer USART_BRR
  tempreg |= M_part << 4;

  //Extract the fraction part
  F_part = (usartdiv - (M_part * 100));

  //Calculate the final fractional
  if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
   {
	  //OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

   }else
   {
	   //over sampling by 16
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

   }

  //Place the fractional part in appropriate bit position . refer USART_BRR
  tempreg |= F_part;

  //copy the value of tempreg in to BRR register
  pUSARTx->BRR = tempreg;
}

void USART_Init(USART_Handle_t *pUSARTHandle){
	uint32_t tempReg = 0 ;
	// usart tx and rx setting
	if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX) {
		tempReg |= (1 << USART_CR1_RE) ;
		tempReg &= ~(1 << USART_CR1_TE) ;
	}else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX){
		tempReg |= (1 << USART_CR1_TE) ;
		tempReg &= ~(1 << USART_CR1_RE) ;
	}else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX){
		tempReg |= (1 << USART_CR1_RE) ;
		tempReg |= (1 << USART_CR1_TE) ;
	}

	tempReg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;

    //Configuration of parity control bit fields
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		tempReg |= ( 1 << USART_CR1_PCE);
		// selecting even parity (selected by default )
		tempReg |= ( 0 << USART_CR1_PS);
	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		tempReg |= ( 1 << USART_CR1_PCE);
		// selecting ODD parity
		tempReg |= ( 1 << USART_CR1_PS);
	}
	// programming the CR1 register
   pUSARTHandle->pUSARTx->CR1 = tempReg ;

   /* Configuring the CR2 register */

   tempReg = 0;	// resetting the temp varaible
   tempReg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;
	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempReg ;

	/* Configuring the CR3 register */
	tempReg = 0;

	//Configuration of USART hardware flow control
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//enable hardware CTS flow control
		tempReg |= (1 < USART_CR3_CTSE) ;

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		// enable hardware RTS flow control
		tempReg |= (1 < USART_CR3_RSTE) ;

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		// enable both CTS and RTS hardware flow control
			// enable hardware RTS flow control
			tempReg |= (1 < USART_CR3_RSTE) ;
			//enable hardware CTS flow control
			tempReg |= (1 < USART_CR3_CTSE) ;
	}

	pUSARTHandle->pUSARTx->CR3 = tempReg ;

	// baud generation
	USART_SetBaudRate(pUSARTHandle->pUSARTx , pUSARTHandle->USART_Config.USART_Baud) ;
}


void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len){
	uint16_t *pdata;
	for(uint32_t i = 0 ; i < Len; i++)
	{
		while( !USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));

    	if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);
			if(pUSARTHandle->USART_Config.USART_ParityControl== USART_PARITY_DISABLE)
			{
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				pTxBuffer++;
			}
		}
		else
		{
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);
			pTxBuffer++;
		}
	}

	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){

	for(uint32_t i = 0 ; i < Len; i++){
	while(USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE) );

	if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){

	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){

	*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x1FF);
	 pRxBuffer ++ ;
	 pRxBuffer ++ ;
	}
	else{
		 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
		  pRxBuffer ++ ;
		}
	}
	else{
	 if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
		 *pRxBuffer = pUSARTHandle->pUSARTx->DR ;
	}
	 else{
	 *pRxBuffer = ((uint8_t)pUSARTHandle->pUSARTx->DR & (0x7F)) ;
	}
	pRxBuffer++;

  }
 }
}
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len){
return 0 ;
}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){
return 0 ;
}

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi) {

}

void USART_IRQPrioriyConfig(uint8_t IRQNumber, uint8_t IRQPriority) {

}
