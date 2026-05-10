#define PACKET_MAGIC0          		0xA5
#define PACKET_MAGIC1          		0x5A

// 4 Header bytes
#define HEADER_BYTES           		4

// Amount of bytes for 256 ADC samples
#define PAYLOAD_BYTES          		384
#define SPI_PACKET_BYTES       		(HEADER_BYTES + PAYLOAD_BYTES)


static uint8_t PacketIsValid(uint8_t *packet){

	// A packet is considered valid if the first two bytes match
	if (packet[0] != PACKET_MAGIC0 || packet[1] != PACKET_MAGIC1){
		return 0;
	}

	return 1;

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
