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

for(uint32_t i = 0 ; i < Len; i++)
 {
	while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE) );

	if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
	{
		if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
		{

		*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);
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
		 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
	}
	 else{
	 *pRxBuffer = ((uint8_t)pUSARTHandle->pUSARTx->DR & (0x7F)) ;
	}
	pRxBuffer++;

  }
 }
}
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len){

	uint8_t txstate = pUSARTHandle->RxBusyState ;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		// enable TXEIE interrupt
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE) ;

		// enable TCIE interrupt
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE) ;

	}

	return txstate;

}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){

	uint8_t rxstate = pUSARTHandle->RxBusyState;

		if(rxstate != USART_BUSY_IN_RX)
		{
			pUSARTHandle->RxLen = Len;
			pUSARTHandle->pRxBuffer = pRxBuffer;
			pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

			// enable RXNEIE interrupt
			pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE) ;

		}

		return rxstate;
}


void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{

	if(EnOrDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}

}

void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{

	uint32_t temp1 , temp2, temp3;
	uint16_t *pdata;

/*************************Check for TC flag ********************************************/

    //Implement the code to check the state of TC bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TC);

	 //Implement the code to check the state of TCEIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE);

	if(temp1 && temp2 )
	{
		//this interrupt is because of TC

		//close transmission and call application callback if TxLen is zero
		if ( pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if (pUSARTHandle->TxLen <= 0)
			{
				//Implement the code to clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_TC);

				//Implement the code to clear the TCIE control bit
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TCIE);
				//Reset the application state
				pUSARTHandle->TxBusyState = USART_READY;

				//Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL ;

				//Reset the length to zero
				pUSARTHandle->TxLen = 0 ;

				//Call the applicaton call back with event USART_EVENT_TX_CMPLT
//				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
			}
		}
	}

/*************************Check for TXE flag ********************************************/

	//Implement the code to check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TXE);

	//Implement the code to check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TXEIE);

	if(temp1 && temp2 )
	{
		//this interrupt is because of TXE

		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Keep sending data until Txlen reaches to zero
			if(pUSARTHandle->TxLen > 0)
			{
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//if 9BIT , load the DR with 2bytes masking the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer ;

					//loading only first 9 bits , so we have to mask with the value 0x01FF
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used in this transfer , so, 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;

						//Implement the code to decrement the length
						pUSARTHandle->TxLen -- ;
					}
					else
					{
						//Parity bit is used in this transfer . so , 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;

						//Implement the code to decrement the length
						pUSARTHandle->TxLen -- ;
					}
				}
				else
				{
					//This is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer  & (uint8_t)0xFF);

					//Implement the code to increment the buffer address
					pUSARTHandle->pTxBuffer++;

					//Implement the code to decrement the length
					pUSARTHandle->TxLen -- ;
				}

			}
			if (pUSARTHandle->TxLen == 0 )
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE) ;
			}
		}
	}

/*************************Check for RXNE flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);


	if(temp1 && temp2 )
	{
		//this interrupt is because of rxne
		//this interrupt is because of txe
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			//TXE is set so send data
			if(pUSARTHandle->RxLen > 0)
			{
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//We are going to receive 9bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used. so, all 9bits will be of user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;

						//Implement the code to decrement the length
						pUSARTHandle->RxLen -- ;
					}
					else
					{
						//Parity is used. so, 8bits will be of user data and 1 bit is parity
						 *pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

						 //Now increment the pRxBuffer
						 pUSARTHandle->pRxBuffer++;

						 //Implement the code to decrement the length
						 pUSARTHandle->RxLen -- ;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 8bits will be of user data

						//read 8 bits from DR
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
					}

					else
					{
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity

						//read only 7 bits , hence mask the DR with 0X7F
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

					}

					//Now , increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;

					//Implement the code to decrement the length
					pUSARTHandle->RxLen -- ;
				}


			}//if of >0

			if(! pUSARTHandle->RxLen)
			{
				//disable the rxne
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pUSARTHandle->RxBusyState = USART_READY;
//				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}


/*************************Check for CTS flag ********************************************/
//Note : CTS feature is not applicable for UART4 and UART5

	//Implement the code to check the status of CTS bit in the SR
	temp1 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_CTS) ;

	//Implement the code to check the state of CTSE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);

	//Implement the code to check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
	temp3 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE);


	if(temp1  && temp2 )
	{
		//Implement the code to clear the CTS flag in SR
		USART_ClearFlag(pUSARTHandle->pUSARTx, USART_FLAG_CTS) ;

		//this interrupt is because of cts
//		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
	}

/*************************Check for IDLE detection flag ********************************************/

	//Implement the code to check the status of IDLE flag bit in the SR
	temp1 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_IDLE) ;

	//Implement the code to check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);


	if(temp1 && temp2)
	{
		//Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence

		//this interrupt is because of idle
//		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
	}

/*************************Check for Overrun detection flag ********************************************/

	//Implement the code to check the status of ORE flag  in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;

	//Implement the code to check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;


	if(temp1  && temp2 )
	{
		//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .

		//this interrupt is because of Overrun error
//		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_ORE);
	}



/*************************Check for Error Flag ********************************************/

//Noise Flag, Overrun error and Framing Error in multibuffer communication
//We dont discuss multibuffer communication in this course. please refer to the RM
//The blow code will get executed in only if multibuffer mode is used.

	temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;

	if(temp2 )
	{
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & ( 1 << USART_SR_FE))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
//			USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_FE);
		}

		if(temp1 & ( 1 << USART_SR_NF) )
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
//			USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_NE);
		}

		if(temp1 & ( 1 << USART_SR_ORE) )
		{
//			USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_ORE);
		}
	}

}


void USART_IRQPrioriyConfig(uint8_t IRQNumber, uint8_t IRQPriority) {

}
