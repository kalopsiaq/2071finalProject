//Reject samples that jump more than 500 ADC values
#define OUTLIER_REJECT_THRESHOLD    500U

//Temporary unpacked ADC sample buffer used while filtering
static uint16_t filterSamples[ADC_HALF_SAMPLES];

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