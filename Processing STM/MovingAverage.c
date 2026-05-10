#define ADC_HALF_SAMPLES          	256
#define MOVING_AVG_WINDOW          	4

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
