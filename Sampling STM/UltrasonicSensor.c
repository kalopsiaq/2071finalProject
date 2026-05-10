#define ULTRASONIC_POLL_MS					90U // How often to trigger and read the ultrasonic sensor.
#define ULTRASONIC_WAIT_START_TIMEOUT_US   	2300U // Maximum time to wait for the echo pin to go high after triggering.
#define ULTRASONIC_NEAR_THRESHOLD_CM       	10U // Object is considered "near" if it is within 10 cm.
#define ULTRASONIC_NEAR_THRESHOLD_US       	(ULTRASONIC_NEAR_THRESHOLD_CM * 58U) // Approx distance in cm = pulse width us / 58.
#define ULTRASONIC_ECHO_HIGH_TIMEOUT_US  	900U // This limits the measured distance range. 900ms is about 15cm
#define ULTRASONIC_NEAR_HOLD_MS        		700U // Once a near object is detected, keep the near flag high for this long.


static volatile uint8_t ultrasonicNear = 0; // 1 if something is near
static uint32_t lastUltrasonicPollMs = 0; // Stores the last time the ultrasonic sensor was polled

static volatile uint32_t ultrasonicDistanceCm = 0; // Last calculated distance in cm
static volatile uint32_t ultrasonicPulseWidthUs = 0; // Last measured echo width in us
static uint8_t ultrasonicNearSeen = 0; // Whether an object has been seen recently
static uint32_t lastUltrasonicNearMs = 0; // stores the time when a near object was last detected

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

//Poll "PollUltrasonicIfDue" in the main while(1) loop