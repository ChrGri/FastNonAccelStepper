#include <Arduino.h>
#include "FastNonAccelStepper.h"


float sineFrequencyInHz = 1;
float sineAmplitudeInSteps = 1000;


FastNonAccelStepper stepper(PWM_GPIO_PIN, PCNT_DIR_GPIO);

void setup() {
    Serial.begin(115200);
    stepper.init();
    stepper.setMaxSpeed(200000);
}


void loop() {
    static long targetPosition = 1000; // Example target position
	
	float tInSeconds = (float)millis() / 1000.0f;


	// choose target pattern
	uint8_t targetPattern_u8 = 1;
	float targetPos_fl32 = 0;
	switch (targetPattern_u8)
	{
		case 0: 
		  targetPosition += 500;  // Increment target position cyclically
		  if (targetPosition > 50000) targetPosition = -50000;  // Reset after reaching max range
		  break;
		case 1:
		  // sine wave
		  targetPos_fl32 = sineAmplitudeInSteps * sin( 2.0f * M_PI * sineFrequencyInHz * tInSeconds);
		  targetPosition = targetPos_fl32;
		  break;
		case 2:
		  // step
		  if ( fmod(tInSeconds, 2) > 1)
		  {targetPosition = -sineAmplitudeInSteps;}
		  else
		  {targetPosition = sineAmplitudeInSteps;}
		  break;
	}

	// Update PCNT limits and target
	stepper.setTargetPosition(targetPosition);

	// Read and print the pulse count
	long currentPosition = stepper.getCurrentPosition();

	// print routine
	Serial.printf("target:%d,currentPos:%d\n", targetPosition, currentPosition);		//the first variable for plotting

	delay(10);  // Adjust delay as needed
	    
}