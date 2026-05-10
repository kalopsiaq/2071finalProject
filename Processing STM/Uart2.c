// Trackets which UART buffer halves are ready to send or currently in use
static volatile uint8_t uartHalfReady[UART_TX_HALVES] = {0, 0};
static volatile uint8_t uartHalfInUse[UART_TX_HALVES] = {0, 0};

// UART DMA State
static volatile uint8_t uartActive = 0;
static volatile uint8_t uartActiveHalf = 0;
static volatile uint8_t uartWriteHalf = 0;

// Debug counters for UART DMA activity
static volatile uint32_t uartStartedCount = 0;
static volatile uint32_t uartDoneCount = 0;
static volatile uint32_t uartDroppedCount = 0;

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