static volatile uint8_t adcHalf0Ready = 0; // 1 when the first half of the buffer is ready
static volatile uint8_t adcHalf1Ready = 0; // 1 when the second half o the buffer is ready

// Starts ADC Sampling and timer-triggered DMA streaming
static void App_StartAdcSpiStreaming(void){

	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK){
		Error_Handler();
	}

	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuf, ADC_BUF_SAMPLES) != HAL_OK){
		Error_Handler();
	}

	if (HAL_TIM_Base_Start(&htim6) != HAL_OK){
		Error_Handler();
	}
}


// Is called when the ADC buffer is half full
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc){

	if (hadc->Instance == ADC1){
		// Tell the main loop that adcBuf[0..255] is ready
		adcHalf0Ready = 1;
		// For debugging to see what sample rate its hitting
		HAL_GPIO_TogglePin(RATE_PORT_GPIO_Port, RATE_PORT_Pin);
	}
}

// Is called when the ADC buffer is full
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){

	if (hadc->Instance == ADC1){
		// Tell the main loop that adcBuf[256..511 is ready
		adcHalf1Ready = 1;
		// For debugging to see what sample rate its hitting
		HAL_GPIO_TogglePin(RATE_PORT_GPIO_Port, RATE_PORT_Pin);
	}
}
