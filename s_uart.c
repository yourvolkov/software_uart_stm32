
#include "s_uart.h"

/******************************************************************************/
/********************** Private functions prototypes **************************/
/******************************************************************************/
static void GPIO_Set_Bit(GPIO_TypeDef* port, uint16_t pin);
static void GPIO_Clear_Bit(GPIO_TypeDef* port, uint16_t pin);
static uint8_t GPIO_Read_Bit(GPIO_TypeDef* port, uint16_t pin);
static void S_UART_periph_init(S_UART_init_struct* struct_init);
static void send_byte(soft_uart* s_uart, uint8_t byte);
static uint8_t majority_function(uint8_t* middle_sample);

/******************************************************************************/
/************************* Rx/Tx handlers *************************************/
/******************************************************************************/
__weak void S_UART_TC_Handler(){
  /* NOTE : This function Should not be modified, when the handler is needed,
            the S_UART_TC_Handler could be implemented in the user file
   */
}

__weak void S_UART_RXNE_Handler(){
  /* NOTE : This function Should not be modified, when the handler is needed,
            the S_UART_RXNE_Handler could be implemented in the user file
   */
}

/******************************************************************************/
/********************** Platform dependent functions **************************/
/******************************************************************************/
static void GPIO_Set_Bit(GPIO_TypeDef* port, uint16_t pin){
	#ifdef HAL_LIB
		HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
	#else
		/* Standart peripheral lib realization */
	#endif 
}

static void GPIO_Clear_Bit(GPIO_TypeDef* port, uint16_t pin){
	#ifdef HAL_LIB
		HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
	#else
		/* Standart peripheral lib realization */
	#endif 
}

static uint8_t GPIO_Read_Bit(GPIO_TypeDef* port, uint16_t pin){
	#ifdef HAL_LIB
		return HAL_GPIO_ReadPin(port, pin);
	#else
		/* Standart peripheral lib realization */
	#endif 
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
// checks the middle sample, previous and next sample in array!
static uint8_t majority_function(uint8_t* middle_sample){ 
	return 	( (*middle_sample)&(*(middle_sample - 1)) )|
			( (*(middle_sample - 1))&(*(middle_sample + 1)) )|
			( (*middle_sample)&(*(middle_sample + 1)) );	
}
/******************************************************************************/
 static void S_UART_periph_init(S_UART_init_struct* struct_init){
	struct_init->s_uart_port = S_UART_PORT;
	struct_init->s_uart_rx_pin = S_UART_RX_PIN;
	struct_init->s_uart_tx_pin = S_UART_TX_PIN;
	struct_init->s_uart_timer = S_UART_TIMER;
	struct_init->s_uart_timer_period = S_UART_TIMER_PERIOD;
	
	#ifdef HAL_LIB
		TIM_HandleTypeDef s_uart_timer;

		s_uart_timer.Instance = S_UART_TIMER;
		s_uart_timer.Init.Prescaler = S_UART_PRESCALER;
		s_uart_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
		s_uart_timer.Init.Period = S_UART_TIMER_PERIOD;
		s_uart_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		HAL_TIM_Base_Start(&s_uart_timer);
  		if (HAL_TIM_Base_Init(&s_uart_timer) != HAL_OK){
    		Error_Handler();
  		}
		HAL_TIM_Base_Start_IT(&s_uart_timer);
	#else 
		/* Standart peripheral lib realization */
	#endif 
}

/******************************************************************************/
static void send_byte(soft_uart* s_uart, uint8_t byte){
	s_uart->tx_bit_cnt = 0;
	s_uart->tx_sample_cnt = 0;
	s_uart->TDR = byte;
	CLEAR_FLAG(s_uart->SR, TXE_FLAG);
} 

/******************************************************************************/
/*
 *
*/
void S_UART_init(soft_uart* s_uart){
	S_UART_periph_init(&(s_uart->peripheral));
	s_uart->rx_bit_cnt = 0;
	s_uart->rx_sample_cnt = 0;
	s_uart->tx_bit_cnt = 0;
	s_uart->tx_sample_cnt = 0;
	s_uart->TDR = 0;
	s_uart->RDR = 0;
	SET_FLAG(s_uart->SR, TXE_FLAG);
	CLEAR_FLAG(s_uart->SR, TC_FLAG);
	CLEAR_FLAG(s_uart->SR, RXNE_FLAG);
	CLEAR_FLAG(s_uart->SR, RX_START_BIT_FLAG);
	GPIO_Set_Bit(s_uart->peripheral.s_uart_port, s_uart->peripheral.s_uart_tx_pin);
}

/******************************************************************************/
/*
 *
*/
void S_UART_Timer_Handler(soft_uart* s_uart){

	/*** Transmitting part ***/
	if(( (s_uart->SR)&(1 << TXE_FLAG) ) == 0){ // Means tx buffer is not empty
		if(s_uart->tx_sample_cnt == 0){
			if(s_uart->tx_bit_cnt == 0){ // means that we need to send a start bit
				GPIO_Clear_Bit(s_uart->peripheral.s_uart_port, s_uart->peripheral.s_uart_tx_pin);
				s_uart->tx_sample_cnt++;
				s_uart->tx_bit_cnt++;
			}else if( ((s_uart->tx_bit_cnt) > 0) && ((s_uart->tx_bit_cnt) < (DATA_BIT_LEN - 1))){ // sending a data byte
				if( ( (s_uart->TDR) & (1 << (s_uart->tx_bit_cnt - 1)) ) != 0){
					GPIO_Set_Bit(s_uart->peripheral.s_uart_port, s_uart->peripheral.s_uart_tx_pin);
				}else{
					GPIO_Clear_Bit(s_uart->peripheral.s_uart_port, s_uart->peripheral.s_uart_tx_pin);
				}
				s_uart->tx_sample_cnt++;
				s_uart->tx_bit_cnt++;
			}else if(s_uart->tx_bit_cnt == (DATA_BIT_LEN - 1)){ // means that we need to send a stop bit
				GPIO_Set_Bit(s_uart->peripheral.s_uart_port, s_uart->peripheral.s_uart_tx_pin);
				s_uart->tx_sample_cnt++;
				s_uart->tx_bit_cnt++;
			}else if(s_uart->tx_bit_cnt == DATA_BIT_LEN){ // means that transmission of one byte is done
				SET_FLAG(s_uart->SR, TC_FLAG);
				SET_FLAG(s_uart->SR, TXE_FLAG);	
				s_uart->tx_bit_cnt = 0;		
			}else{ // unknown counter value
				s_uart->tx_bit_cnt = 0;
				s_uart->tx_sample_cnt = 0;
				SET_FLAG(s_uart->SR, TXE_FLAG);
			}
		}else{
			if(s_uart->tx_sample_cnt < (S_UART_SAMPLES_PER_BIT - 1)){
				s_uart->tx_sample_cnt++;
			}else{
				s_uart->tx_sample_cnt = 0;
			}
			
		} 
	}
	/*** Transmission complete part ***/
	if(( (s_uart->SR)&(1 << TC_FLAG) ) != 0){ // Means that transmission of one byte has been completed
		S_UART_TC_Handler();
		CLEAR_FLAG(s_uart->SR, TC_FLAG); // Clear pending flag
	}

	/*** Receiving part ***/
	uint8_t received_sample = GPIO_Read_Bit(s_uart->peripheral.s_uart_port, s_uart->peripheral.s_uart_rx_pin);
	if(( (s_uart->SR)&(1 << RX_START_BIT_FLAG) ) == 0){ // wait for start bit falling edge
		if(s_uart->rx_sample_cnt == 0){
			if(received_sample == BIT_LOW){
				s_uart->rx_samples_buff[s_uart->rx_sample_cnt] = received_sample;
				s_uart->rx_sample_cnt++;
			}
		}else if(s_uart->rx_sample_cnt < START_BIT_SAMPLES){
				s_uart->rx_samples_buff[s_uart->rx_sample_cnt] = received_sample;
				s_uart->rx_sample_cnt++;
		}else if(s_uart->rx_sample_cnt == START_BIT_SAMPLES){ // Check with majority function
			if(majority_function(((s_uart->rx_samples_buff) + 1)) == S_UART_START_BIT){ // noise filter
				s_uart->rx_samples_buff[s_uart->rx_sample_cnt] = received_sample;
				s_uart->rx_sample_cnt++;				
			}else{ // noise 
				s_uart->rx_sample_cnt = 0;
			}
		}else if(s_uart->rx_sample_cnt < (S_UART_SAMPLES_PER_BIT - 1)){
			s_uart->rx_samples_buff[s_uart->rx_sample_cnt] = received_sample;
			s_uart->rx_sample_cnt++;		
		}else if(s_uart->rx_sample_cnt == (S_UART_SAMPLES_PER_BIT - 1)){
			if(majority_function(((s_uart->rx_samples_buff) + (S_UART_SAMPLES_PER_BIT - 1)/2)) == S_UART_START_BIT){ // check in the middle of a bit
				SET_FLAG(s_uart->SR, RX_START_BIT_FLAG);
				s_uart->rx_sample_cnt = 0;
				s_uart->rx_bit_cnt = 1;
				s_uart->RDR = 0;
			}else{
				s_uart->rx_sample_cnt = 0;
			}
		}else{// unknown counter value
			s_uart->rx_sample_cnt = 0;
		}
		
	}else{ // receive data
		uint8_t s_uart_rx_bit;
		if(s_uart->rx_sample_cnt < (S_UART_SAMPLES_PER_BIT - 1)){
			s_uart->rx_samples_buff[s_uart->rx_sample_cnt] = received_sample;
			s_uart->rx_sample_cnt++;	
		}else{
			if(majority_function(((s_uart->rx_samples_buff) + (S_UART_SAMPLES_PER_BIT - 1)/2)) == 0){ // check in the middle of a bit
				/* 0 received */	
				s_uart_rx_bit = BIT_LOW;	
			}else{
				/* 1 received */
				s_uart_rx_bit = BIT_HIGH;	
			}
			s_uart->rx_sample_cnt = 0;
			s_uart->rx_bit_cnt++;
			if((s_uart->rx_bit_cnt) < DATA_BIT_LEN){
				if(s_uart_rx_bit == BIT_HIGH){
					SET_FLAG(s_uart->RDR, s_uart->rx_bit_cnt - 2); // shift 1 to data register
				}else{
					CLEAR_FLAG(s_uart->RDR, s_uart->rx_bit_cnt - 2); // shift 0 to data register
				}	
			}else if((s_uart->rx_bit_cnt) == DATA_BIT_LEN){ //stop bit required
				if(s_uart_rx_bit != BIT_HIGH){
					/* Capture error */
				}else{
					SET_FLAG(s_uart->SR, RXNE_FLAG);
				}
				s_uart->rx_bit_cnt = 0;
				s_uart->rx_sample_cnt = 0;
				CLEAR_FLAG(s_uart->SR, RX_START_BIT_FLAG);
			}
		}
	}

	/*** Receive complete part ***/
	 if(( (s_uart->SR)&(1 << RXNE_FLAG) ) != 0){ // Means that transmission of one byte has been completed
		S_UART_RXNE_Handler();
		CLEAR_FLAG(s_uart->SR, RXNE_FLAG); // Clear pending flag
	}
	
}

/******************************************************************************/
void S_UART_SendData(soft_uart* s_uart, uint8_t data){
	if( ( (s_uart->SR)&(1 << TXE_FLAG) ) != 0){
		send_byte(s_uart, data);
	}else{
		/* Attempt to send something when the previous transmission is not completed yet */
	}
	
}
/******************************************************************************/
uint8_t S_UART_ReceiveData(soft_uart* s_uart){
	if( ( (s_uart->SR)&(1 << RXNE_FLAG) ) != 0){
		return s_uart->RDR;
	}else{
		/* Attempt to receive something when the data is not valid  */
	}
}
/******************************************************************************/