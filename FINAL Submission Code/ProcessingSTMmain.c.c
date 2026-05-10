/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PACKET_MAGIC0          		0xA5
#define PACKET_MAGIC1          		0x5A

// 4 Header bytes
#define HEADER_BYTES           		4

// Amount of bytes for 256 ADC samples
#define PAYLOAD_BYTES          		384
#define SPI_PACKET_BYTES       		(HEADER_BYTES + PAYLOAD_BYTES)

//UART transmit buffer is double-buffered
//One half can be transmitting while the other is being filled
#define UART_TX_HALVES         		2
#define UART_TX_BUF_BYTES     		(UART_TX_HALVES * SPI_PACKET_BYTES)

#define ADC_HALF_SAMPLES          	256
#define MOVING_AVG_WINDOW          	4

//Reject samples that jump more than 500 ADC values
#define OUTLIER_REJECT_THRESHOLD    500U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
//Raw SPI packet recieved by DMA
uint8_t spiRxPacket[SPI_PACKET_BYTES];

// Double-buffer UART transmit buffer
uint8_t uartTxBuf[UART_TX_BUF_BYTES];

// Trackets which UART buffer halves are ready to send or currently in use
static volatile uint8_t uartHalfReady[UART_TX_HALVES] = {0, 0};
static volatile uint8_t uartHalfInUse[UART_TX_HALVES] = {0, 0};

// UART DMA State
static volatile uint8_t uartActive = 0;
static volatile uint8_t uartActiveHalf = 0;
static volatile uint8_t uartWriteHalf = 0;

// Debug counters for SPI packet transmission
static volatile uint32_t packetGoodCount = 0;
static volatile uint32_t packetBadCount = 0;
static volatile uint32_t packetDroppedCount = 0;

// Debug counters for UART DMA activity
static volatile uint32_t uartStartedCount = 0;
static volatile uint32_t uartDoneCount = 0;
static volatile uint32_t uartDroppedCount = 0;

//Temporary unpacked ADC sample buffer used while filtering
static uint16_t filterSamples[ADC_HALF_SAMPLES];

// Counts how many ADC samples were rejected as outliers for debugging
static volatile uint32_t rejected = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t PacketIsValid(uint8_t *packet){

	// A packet is considered valid if the first two bytes match
	if (packet[0] != PACKET_MAGIC0 || packet[1] != PACKET_MAGIC1){
		return 0;
	}

	return 1;

}

static void TryStartUartDma(void){

	// If UART DMA is already busy
	if (uartActive){
		return;
	}

	// Look for a buffer half that has a packet ready to transmit
	for (uint8_t half = 0; half < UART_TX_HALVES; half++){

		if (uartHalfReady[half]){

			uartHalfReady[half] = 0;
			uartHalfInUse[half] = 1;
			uartActive = 1;
			uartActiveHalf = half;

			// Start DMA transmit from this half of the UART buffer
			if (HAL_UART_Transmit_DMA(&huart2, &uartTxBuf[half * SPI_PACKET_BYTES], SPI_PACKET_BYTES) != HAL_OK){

				// If DMA failed to start, release the buffer half
				uartHalfInUse[half] = 0;
				uartActive = 0;
				uartDroppedCount++;
			}
			else{
				uartStartedCount++;
			}

			return;
		}
	}
}

static void QueuePacketForUart(uint8_t *packet){

	// Try to write into the preferred buffer half first
	uint8_t half = uartWriteHalf;

	// If that half is unavailable, try the other one
	if (uartHalfReady[half] || uartHalfInUse[half]){
		half ^= 1;
	}

	// If both halves are unavailable, drop the packet
	if (uartHalfReady[half] || uartHalfInUse[half]){
		packetDroppedCount++;
		uartDroppedCount++;
		return;
	}

	// Copy the SPI packet into the selected UART transmit buffer half
	for (uint32_t i = 0; i < SPI_PACKET_BYTES; i++){
		uartTxBuf[(half * SPI_PACKET_BYTES) + i] = packet[i];
	}

	// Mark this half ready and switch the preferred write half
	uartHalfReady[half] = 1;
	uartWriteHalf = half ^ 1;

	// Start UART DMA immediately if it is idle
	TryStartUartDma();
}

static void UnpackAdcPayload(uint8_t *payload, uint16_t *samples){
    // ADC samples are packed as 12-bit values:
    //
    // byte j:     lower 8 bits of sample 0
    // byte j + 1: upper 4 bits of sample 0, lower 4 bits of sample 1
    // byte j + 2: upper 8 bits of sample 1
    //
    // Every 3 bytes becomes 2 uint16_t samples.
	for (uint32_t i = 0, j = 0; i < ADC_HALF_SAMPLES; i += 2, j += 3){

		samples[i] = (uint16_t)(payload[j] | ((payload[j + 1] & 0x0F) << 8));
		samples[i + 1] = (uint16_t)(((payload[j + 1] >> 4) & 0x0F) | (payload[j + 2] << 4));

	}
}

static void PackAdcPayload(uint8_t *payload, uint16_t *samples){
	// Re-pack two 12-bit ADC samples into three bytes.
	for (uint32_t i = 0, j = 0; i < ADC_HALF_SAMPLES; i += 2, j += 3){
		uint16_t s0 = samples[i] & 0x0FFF;
		uint16_t s1 = samples[i + 1] & 0x0FFF;

		payload[j] = (uint8_t)(s0 & 0xFF);
		payload[j + 1] = (uint8_t)(((s0 >> 8) & 0x0F) | ((s1 & 0x0F) << 4));
		payload[j + 2] = (uint8_t)((s1 >> 4) & 0xFF);
	}
}

static void RejectOutliers(uint16_t *samples){

	// Keeps track of the previous accepted sample across packets
	static uint8_t initialized = 0;
	static uint16_t lastAccepted = 2048; //Initialise the last accepted as the mid point


	for (uint32_t i = 0; i < ADC_HALF_SAMPLES; i++){
		uint16_t sample = samples[i];

		// On startup, wait for the first reasonable sample
		if (!initialized){
			if (sample > 100U && sample < 3995U){
				lastAccepted = sample;
				initialized = 1;
			}
			else {
				// If the first value was not reasonable replace it with the midpoint/deafult
				sample = lastAccepted;
				rejected++;
			}
		}
		else {
			uint16_t diff;

			// Calaculate the difference form the last accepted sample
			if (sample > lastAccepted){
				diff = sample - lastAccepted;
			}
			else {
				diff = lastAccepted - sample;
			}

			// If the jump is too large, treat it as a spike and replace it
			if (diff > OUTLIER_REJECT_THRESHOLD){
				sample = lastAccepted;
				rejected++;
			}
			else {
				// Otherwise accept the new sample as the latest good value
				lastAccepted = sample;
			}
		}

		samples[i] = sample;
	}
}

static void ApplyMovingAverage(uint16_t *samples){

	// Small rolling average filter that persits across packets
	static uint16_t window[MOVING_AVG_WINDOW] = {0};
	static uint32_t sum = 0;
	static uint8_t index = 0;
	static uint8_t count = 0;

	for (uint32_t i = 0; i < ADC_HALF_SAMPLES; i++){

		// Remove the oldest sample from the sum
		sum -= window[index];

		// Insert the newest sample
		window[index] = samples[i];
		sum += window[index];

		// Advance circular buffer index
		index++;

		if (index >= MOVING_AVG_WINDOW){
			index = 0;
		}

		// During startup, average only the samples that exist so far
		if (count < MOVING_AVG_WINDOW)
		{
			count++;
		}

		// Replace sample with the moving average result
		samples[i] = (uint16_t)(sum / count);
	}
}

static void FilterPacketInPlace(uint8_t *packet){

	// Convert packed ADC bytes into uint16_t samples
	UnpackAdcPayload(&packet[HEADER_BYTES], filterSamples);

	// Remove large single-sample jumps
	RejectOutliers(filterSamples);

	// Smooth the signal slightly
	ApplyMovingAverage(filterSamples);

	// Re-pack filtered sampels back into the original packet payload
	PackAdcPayload(&packet[HEADER_BYTES], filterSamples);

}

static void ArmSpiReceive(void){

	// Tell the SPI master this slave is not ready while DMA is being armed
	HAL_GPIO_WritePin(SLAVE_READY_GPIO_Port, SLAVE_READY_Pin, GPIO_PIN_RESET);

	// Start the SPI DMA receive for one complete packet
	if (HAL_SPI_Receive_DMA(&hspi1, spiRxPacket, SPI_PACKET_BYTES) != HAL_OK){
		Error_Handler();
	}

	// Signal the SPI master that this slave is ready for a packet
	HAL_GPIO_WritePin(SLAVE_READY_GPIO_Port, SLAVE_READY_Pin, GPIO_PIN_SET);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){

	// Called automatically when SPI DMA receives a full packet
	if (hspi->Instance == SPI1){

		// Mark slave not ready while processing the received packet
		HAL_GPIO_WritePin(SLAVE_READY_GPIO_Port, SLAVE_READY_Pin, GPIO_PIN_RESET);

		if (PacketIsValid(spiRxPacket)){
			packetGoodCount++;

			// Filter ADC samples inside the packet
			FilterPacketInPlace(spiRxPacket);

			// Queue filtered packet for UART DMA transmission
			QueuePacketForUart(spiRxPacket);
		}
		else{

			packetBadCount++;
		}

		//Re-arm SPI DMA for the next packet
		ArmSpiReceive();
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){

	// Called automatically when UART DMA finishes sending one packet
	if (huart->Instance == USART2){
		uartHalfInUse[uartActiveHalf] = 0;
		uartActive = 0;
		uartDoneCount++;

		// If another packet is queued, start sending it immediately
		TryStartUartDma();
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(SLAVE_READY_GPIO_Port, SLAVE_READY_Pin, GPIO_PIN_RESET);

  HAL_Delay(1000);

  ArmSpiReceive();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SLAVE_READY_GPIO_Port, SLAVE_READY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SLAVE_READY_Pin */
  GPIO_InitStruct.Pin = SLAVE_READY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SLAVE_READY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
