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