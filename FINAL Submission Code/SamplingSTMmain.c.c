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
// Magic bytes used to identify the start of a valid SPI packet.
#define PACKET_MAGIC0          	0xA5
#define PACKET_MAGIC1          	0x5A

// Total ADC DMA buffer size.
#define ADC_BUF_SAMPLES        	512
#define ADC_HALF_SAMPLES       	256

// Every 2 ADC samples are packed into 3 bytes because each ADC sample is 12-bit.
#define FRAME_SIZE_BYTES       	3

// Payload size in bytes.
// 256 samples * 12 bits = 3072 bits = 384 bytes.
#define PAYLOAD_BYTES          	384

// Packet header size.
// byte 0 = magic0
// byte 1 = magic1
// byte 2 = ADC buffer half flag
// byte 3 = ultrasonic near flag
#define HEADER_BYTES           	4
#define SPI_PACKET_BYTES       	(HEADER_BYTES + PAYLOAD_BYTES)

#define ULTRASONIC_POLL_MS					90U // How often to trigger and read the ultrasonic sensor.
#define ULTRASONIC_WAIT_START_TIMEOUT_US   	2300U // Maximum time to wait for the echo pin to go high after triggering.
#define ULTRASONIC_NEAR_THRESHOLD_CM       	10U // Object is considered "near" if it is within 10 cm.
#define ULTRASONIC_NEAR_THRESHOLD_US       	(ULTRASONIC_NEAR_THRESHOLD_CM * 58U) // Approx distance in cm = pulse width us / 58.
#define ULTRASONIC_ECHO_HIGH_TIMEOUT_US  	900U // This limits the measured distance range. 900ms is about 15cm
#define ULTRASONIC_NEAR_HOLD_MS        		700U // Once a near object is detected, keep the near flag high for this long.
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// DMA buffer filled by the ADC.
uint16_t adcBuf[ADC_BUF_SAMPLES];

// Two Spi packet buffers
uint8_t spiPacketA[SPI_PACKET_BYTES];
uint8_t spiPacketB[SPI_PACKET_BYTES];

static volatile uint8_t adcHalf0Ready = 0; // 1 when the first half of the buffer is ready
static volatile uint8_t adcHalf1Ready = 0; // 1 when the second half o the buffer is ready
static volatile uint8_t spiBusy = 0; //1 if the Spi is currently in transmission

static volatile uint32_t spiDroppedPackets = 0; // Counts how many packets couldn't be sent because Spi was busy
static volatile uint32_t spiSentPackets = 0; // Counts how many packets were successfully completed


static volatile uint8_t ultrasonicNear = 0; // 1 if something is near
static uint32_t lastUltrasonicPollMs = 0; // Stores the last time the ultrasonic sensor was polled

static volatile uint32_t ultrasonicDistanceCm = 0; // Last calculated distance in cm
static volatile uint32_t ultrasonicPulseWidthUs = 0; // Last measured echo width in us
static uint8_t ultrasonicNearSeen = 0; // Whether an object has been seen recently
static uint32_t lastUltrasonicNearMs = 0; // stores the time when a near object was last detected
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void BuildPacket(uint8_t *packet, uint16_t *adcSrc, uint8_t flags){

	packet[0] = PACKET_MAGIC0;
	packet[1] = PACKET_MAGIC1;
	packet[2] = flags;
	packet[3] = ultrasonicNear;

	// dst points to the payload area after the 4-byte header.
	uint8_t *dst = &packet[HEADER_BYTES];

    // Pack 256 12-bit ADC samples into 384 bytes.
    // Every loop packs 2 samples into 3 bytes.
	for (uint32_t i = 0, j = 0; i < ADC_HALF_SAMPLES; i += 2, j += 3){

		// Keep only the lower 12 bits of each sample as we are doing 12 bit readings.
		uint16_t s0 = adcSrc[i] & 0x0FFF;
		uint16_t s1 = adcSrc[i + 1] & 0x0FFF;

		dst[j + 0] = (uint8_t)(s0 & 0xFF); // Lower 8 bits of s0

		// Upper 4 bits of s0 in lower 4 bits
		// Lower 4 bits of s1 in upper 4 bits
		dst[j + 1] = (uint8_t)(((s0 >> 8) & 0x0F) | ((s1 & 0x0F) << 4));

		dst[j + 2] = (uint8_t)((s1 >> 4) & 0xFF); //Upper 8 bits of s1
	}
}

// Attempts to send one packet over SPI using DMA.
static void TrySendPacket(uint8_t *packet){

	// If SPI is already sending a packet, drop this packet.
	if (spiBusy){
		spiDroppedPackets++;
		return;
	}

	// Check if the processing STM is ready to recieve bytes
	// If SLAVE_READY is low, drop this packet
	if (HAL_GPIO_ReadPin(SLAVE_READY_GPIO_Port, SLAVE_READY_Pin) == GPIO_PIN_RESET){

		spiDroppedPackets++;
		return;
	}

	// Mark SPI as busy before starting DMA.
	spiBusy = 1;

	// Pull chip select low to being the transmission
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

	// Start SPI DMA transmission
	if (HAL_SPI_Transmit_DMA(&hspi1, packet, SPI_PACKET_BYTES) != HAL_OK){

		// If starting DMA failed
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
		spiBusy = 0;
		spiDroppedPackets++;
	}
}

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

// Called automatically when SPI DMA transmission finishes.
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){

	if (hspi->Instance == SPI1){

		// Release chip select to end the SPI transaction.
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
		// Count the successfully sent packet.
		spiSentPackets++;
		// Spi is no longer busy
		spiBusy = 0;
	}
}

// Enables the CPU cycle counter, used for microsecond timing for Ultrasonic sensor
static void DwtInit(void){

	// Enables access to the DWT cycle counter
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	// Reset cycle count to 0
	DWT->CYCCNT = 0;
	// Start the cycle counter
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// Returns approximate time in microseconds using CPU cycle counter
static uint32_t Micros(void){
	return DWT->CYCCNT / (HAL_RCC_GetHCLKFreq() / 1000000U);
}

// Delay thing in microseconds
static void DwtDelayUs(uint32_t us){

	uint32_t start = DWT->CYCCNT;
	uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000U);

	// Wait until enough CPU cycles have passed
	while ((DWT->CYCCNT - start) < ticks){
	}
}

// Updates ultrasonicNear using the hold timer
static void UpdateUltrasonicNearHold(uint32_t now){

	// If something was recently seen near, keep ultrasonicNear high
	if (ultrasonicNearSeen && ((now - lastUltrasonicNearMs) <= ULTRASONIC_NEAR_HOLD_MS)){
		ultrasonicNear = 1;
	}
	else{
		ultrasonicNear = 0;
	}
}

//Polls the ultrasonic sensor if enough time has passed
static void PollUltrasonicIfDue(void){

	// Current system time in milliseconds
	uint32_t now = HAL_GetTick();

	// Only poll every ULTRASONIC_POLL_MS milliseconds
	if ((now - lastUltrasonicPollMs) < ULTRASONIC_POLL_MS){
		return;
	}

	lastUltrasonicPollMs = now;

	// Make sure trigger starts low
	HAL_GPIO_WritePin(ULTRASONIC_TRIG_GPIO_Port, ULTRASONIC_TRIG_Pin, GPIO_PIN_RESET);
	DwtDelayUs(2);

	// Send 10us trigger pulse
	HAL_GPIO_WritePin(ULTRASONIC_TRIG_GPIO_Port, ULTRASONIC_TRIG_Pin, GPIO_PIN_SET);
	DwtDelayUs(10);
	HAL_GPIO_WritePin(ULTRASONIC_TRIG_GPIO_Port, ULTRASONIC_TRIG_Pin, GPIO_PIN_RESET);

	// Record time when waiting for echo to start
	uint32_t waitStart = Micros();

	// Wait for echo pin to go high
	while (HAL_GPIO_ReadPin(ULTRASONIC_ECHO_GPIO_Port, ULTRASONIC_ECHO_Pin) == GPIO_PIN_RESET){

		// If echo never starts, mark distance return
		if ((Micros() - waitStart) > ULTRASONIC_WAIT_START_TIMEOUT_US){
			UpdateUltrasonicNearHold(now);
			return;
		}

	}

	uint32_t pulseStart = Micros();

	// Wait while echo remains high
	while (HAL_GPIO_ReadPin(ULTRASONIC_ECHO_GPIO_Port, ULTRASONIC_ECHO_Pin) == GPIO_PIN_SET){

		// If echo stays high for more than the ULTRASONIC_ECHO_HIGH_TIMEOUT_US stop it
		if ((Micros() - pulseStart) > ULTRASONIC_ECHO_HIGH_TIMEOUT_US){

			ultrasonicDistanceCm = ULTRASONIC_ECHO_HIGH_TIMEOUT_US / 58U; // About 15cm
			ultrasonicPulseWidthUs = ULTRASONIC_ECHO_HIGH_TIMEOUT_US;
			UpdateUltrasonicNearHold(now);
			return;
		}
	}

	// Calculate actual echo pulse width
	uint32_t pulseWidthUs = Micros() - pulseStart;

	// Save width for debugging
	ultrasonicPulseWidthUs = pulseWidthUs;

	//Convert pulse width to distance in cm
	ultrasonicDistanceCm = pulseWidthUs / 58U;

	// If the pulse is short enough, object is near
	if (pulseWidthUs <= ULTRASONIC_NEAR_THRESHOLD_US){
		ultrasonicNearSeen = 1;
		lastUltrasonicNearMs = now;
	}

	// Apply hold behaviour to ultrasonicNear
	UpdateUltrasonicNearHold(now);
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
  MX_ADC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

  DwtInit();

  HAL_Delay(1000);

  App_StartAdcSpiStreaming();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Poll ultrasonic sensor periodically
	  PollUltrasonicIfDue();

	  // If first half of ADC buffer is ready and SPI is free, build the packet and send it off
	  if (adcHalf0Ready && !spiBusy){

		  adcHalf0Ready = 0;
		  BuildPacket(spiPacketA, &adcBuf[0], 0);
		  TrySendPacket(spiPacketA);

	  }

	  // If second half of ADC buffer is ready and SPI is free, build the packet and send it off
	  if (adcHalf1Ready && !spiBusy){

		  adcHalf1Ready = 0;
		  BuildPacket(spiPacketB, &adcBuf[ADC_HALF_SAMPLES], 1);
		  TrySendPacket(spiPacketB);

	  }
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 721;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RATE_PORT_Pin|ULTRASONIC_TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPI_CS_Pin RATE_PORT_Pin ULTRASONIC_TRIG_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin|RATE_PORT_Pin|ULTRASONIC_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SLAVE_READY_Pin ULTRASONIC_ECHO_Pin */
  GPIO_InitStruct.Pin = SLAVE_READY_Pin|ULTRASONIC_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
