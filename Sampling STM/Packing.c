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
