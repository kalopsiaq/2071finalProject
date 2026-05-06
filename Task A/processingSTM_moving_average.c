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
#define PACKET_MAGIC0          0xA5
#define PACKET_MAGIC1          0x5A

#define HEADER_BYTES           8
#define PAYLOAD_BYTES          384
#define SPI_PACKET_BYTES       (HEADER_BYTES + PAYLOAD_BYTES)

#define UART_TX_HALVES         2
#define UART_TX_BUF_BYTES      (UART_TX_HALVES * SPI_PACKET_BYTES)

/* DSP buffer sizing.
 * Sampling STM packs 12-bit samples 3-bytes-per-2-samples in the payload.
 * 384 payload bytes / 3 bytes-per-pair * 2 samples-per-pair = 256 samples. */
#define SAMPLES_PER_PACKET     256
#define SAMPLE_MASK_12BIT      0x0FFF


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
uint8_t spiRxPacket[SPI_PACKET_BYTES];

uint8_t uartTxBuf[UART_TX_BUF_BYTES];

static volatile uint8_t uartHalfReady[UART_TX_HALVES] = {0, 0};
static volatile uint8_t uartHalfInUse[UART_TX_HALVES] = {0, 0};

static volatile uint8_t uartActive = 0;
static volatile uint8_t uartActiveHalf = 0;
static volatile uint8_t uartWriteHalf = 0;

static volatile uint32_t packetGoodCount = 0;
static volatile uint32_t packetBadCount = 0;
static volatile uint32_t packetDroppedCount = 0;

static volatile uint32_t uartStartedCount = 0;
static volatile uint32_t uartDoneCount = 0;
static volatile uint32_t uartDroppedCount = 0;

static volatile uint16_t lastSequence = 0;
static volatile uint8_t lastFlags = 0;

/* Scratch buffer for the DSP stage: unpacked 12-bit samples sit here
 * while we filter them. Reused for every packet. Not volatile because
 * only main-thread / SPI callback touches it, never two contexts at once. */
static uint16_t dspSamples[SAMPLES_PER_PACKET];

/* Length-2 moving average needs the previous input sample to persist
 * across packet boundaries, otherwise the first sample of every packet
 * gets averaged against zero and you hear a click at ~172 Hz. */
static uint16_t prevSampleMA = 0;

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

/* Unpack 384 packed bytes from the payload into 256 12-bit samples.
 * Format (Sampling STM contract):
 *   byte0 = sampleA[7:0]
 *   byte1 = (sampleA[11:8])  |  (sampleB[3:0] << 4)
 *   byte2 = sampleB[11:4]
 * If audio comes out as garbled noise, this byte ordering is the
 * first thing to suspect — confirm with teammate's Sampling STM code. */
static void UnpackPayload(const uint8_t *payload)
{
  uint32_t inIdx = 0;
  for (uint32_t i = 0; i < SAMPLES_PER_PACKET; i += 2)
  {
    uint8_t b0 = payload[inIdx++];
    uint8_t b1 = payload[inIdx++];
    uint8_t b2 = payload[inIdx++];

    uint16_t a = (uint16_t)b0           | ((uint16_t)(b1 & 0x0F) << 8);
    uint16_t b = (uint16_t)(b1 >> 4)    | ((uint16_t)b2          << 4);

    dspSamples[i]     = a & SAMPLE_MASK_12BIT;
    dspSamples[i + 1] = b & SAMPLE_MASK_12BIT;
  }
}

/* Repack 256 12-bit samples back into 384 packed bytes, in place
 * in the original packet buffer. Reverse of UnpackPayload. */
static void RepackPayload(uint8_t *payload)
{
  uint32_t outIdx = 0;
  for (uint32_t i = 0; i < SAMPLES_PER_PACKET; i += 2)
  {
    uint16_t a = dspSamples[i]     & SAMPLE_MASK_12BIT;
    uint16_t b = dspSamples[i + 1] & SAMPLE_MASK_12BIT;

    payload[outIdx++] = (uint8_t)(a & 0xFF);
    payload[outIdx++] = (uint8_t)(((a >> 8) & 0x0F) | ((b & 0x0F) << 4));
    payload[outIdx++] = (uint8_t)((b >> 4) & 0xFF);
  }
}

/* Length-2 moving average filter, in place on dspSamples[].
 * y[n] = (x[n-1] + x[n]) / 2
 * x[n-1] is held in prevSampleMA across calls so the chunk boundary
 * is seamless. Note we store the unfiltered input as next prev,
 * not the output — this is FIR, not IIR. */

/* Outlier rejection — Task 3 requirement.
 *
 * Maintains a running mean of recent samples using an exponential moving
 * average implemented with bit shifts (no floats, no division beyond shifts).
 * Each new sample is compared against the mean: if it differs by more than
 * OUTLIER_THRESHOLD, it's replaced with the mean (assume it was noise).
 *
 * The running mean is held in a static so it tracks across packet boundaries.
 * It's stored shifted left by EMA_SHIFT bits to keep fractional precision
 * without floating point — this is a standard fixed-point trick.
 *
 *   mean += (sample - mean) >> EMA_SHIFT     (in shifted form)
 *
 * EMA_SHIFT = 4 means the mean responds to changes over ~16 samples,
 * which at 44.1 ksps is about 360 µs — slow enough to be a stable
 * reference, fast enough to track drifting DC bias from the audio circuit. */
#define EMA_SHIFT           4
#define OUTLIER_THRESHOLD   1000   /* in 12-bit sample units, range 0-4095 */

static void RejectOutliers(void)
{
  /* runningMeanShifted is the mean << EMA_SHIFT.
   * Initialised to mid-scale (2048 << 4 = 32768) so the first packet
   * doesn't see everything as an outlier compared to zero. */
  static uint32_t runningMeanShifted = (2048u << EMA_SHIFT);

  for (uint32_t i = 0; i < SAMPLES_PER_PACKET; i++)
  {
    uint16_t sample = dspSamples[i];
    uint16_t mean   = (uint16_t)(runningMeanShifted >> EMA_SHIFT);

    /* Compute |sample - mean| without going negative (uint16_t). */
    uint16_t diff = (sample > mean) ? (sample - mean) : (mean - sample);

    if (diff > OUTLIER_THRESHOLD)
    {
      /* Outlier: replace with the running mean.
       * Don't update the mean from this sample — it's noise, ignore it. */
      dspSamples[i] = mean & SAMPLE_MASK_12BIT;
    }
    else
    {
      /* Good sample: update the running mean toward it.
       * In shifted form: mean += (sample - mean) >> EMA_SHIFT
       * Equivalently: meanShifted += sample - mean */
      runningMeanShifted += (uint32_t)sample;
      runningMeanShifted -= mean;
      /* dspSamples[i] is already correct — leave it untouched. */
    }
  }
}

static void ApplyMovingAverage(void)
{
  for (uint32_t i = 0; i < SAMPLES_PER_PACKET; i++)
  {
    uint16_t cur = dspSamples[i];
    uint16_t avg = (uint16_t)((prevSampleMA + cur) >> 1);
    prevSampleMA = cur;
    dspSamples[i] = avg & SAMPLE_MASK_12BIT;
  }
}

static uint8_t PacketIsValid(uint8_t *packet)
{
  if (packet[0] != PACKET_MAGIC0 || packet[1] != PACKET_MAGIC1)
  {
    return 0;
  }

  uint16_t payloadLen = (uint16_t)(packet[4] | (packet[5] << 8));

  if (payloadLen != PAYLOAD_BYTES)
  {
    return 0;
  }

  return 1;
}

static void TryStartUartDma(void)
{
  if (uartActive)
  {
    return;
  }

  for (uint8_t half = 0; half < UART_TX_HALVES; half++)
  {
    if (uartHalfReady[half])
    {
      uartHalfReady[half] = 0;
      uartHalfInUse[half] = 1;
      uartActive = 1;
      uartActiveHalf = half;

      if (HAL_UART_Transmit_DMA(&huart2,
                                &uartTxBuf[half * SPI_PACKET_BYTES],
                                SPI_PACKET_BYTES) != HAL_OK)
      {
        uartHalfInUse[half] = 0;
        uartActive = 0;
        uartDroppedCount++;
      }
      else
      {
        uartStartedCount++;
      }

      return;
    }
  }
}

static void QueuePacketForUart(uint8_t *packet)
{
  uint8_t half = uartWriteHalf;

  if (uartHalfReady[half] || uartHalfInUse[half])
  {
    half ^= 1;
  }

  if (uartHalfReady[half] || uartHalfInUse[half])
  {
    packetDroppedCount++;
    uartDroppedCount++;
    return;
  }

  for (uint32_t i = 0; i < SPI_PACKET_BYTES; i++)
  {
    uartTxBuf[(half * SPI_PACKET_BYTES) + i] = packet[i];
  }

  uartHalfReady[half] = 1;
  uartWriteHalf = half ^ 1;

  TryStartUartDma();
}

static void ArmSpiReceive(void)
{
  HAL_GPIO_WritePin(SLAVE_READY_GPIO_Port, SLAVE_READY_Pin, GPIO_PIN_RESET);

  if (HAL_SPI_Receive_DMA(&hspi1, spiRxPacket, SPI_PACKET_BYTES) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_GPIO_WritePin(SLAVE_READY_GPIO_Port, SLAVE_READY_Pin, GPIO_PIN_SET);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1)
  {
    HAL_GPIO_WritePin(SLAVE_READY_GPIO_Port, SLAVE_READY_Pin, GPIO_PIN_RESET);

    if (PacketIsValid(spiRxPacket))
    {
      lastSequence = (uint16_t)(spiRxPacket[2] | (spiRxPacket[3] << 8));
      lastFlags = spiRxPacket[6];
      packetGoodCount++;

      /* DSP pipeline: payload only. Header (bytes 0-7) is untouched,
       * which keeps magic, sequence, length, and the distance-trigger
       * flag (byte 6) flowing through to the PC unchanged. */
      UnpackPayload(&spiRxPacket[HEADER_BYTES]);
      RejectOutliers();
      ApplyMovingAverage();
      RepackPayload(&spiRxPacket[HEADER_BYTES]);

      QueuePacketForUart(spiRxPacket);
    }
    else
    {
      packetBadCount++;
    }

    ArmSpiReceive();
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uartHalfInUse[uartActiveHalf] = 0;
    uartActive = 0;
    uartDoneCount++;

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
