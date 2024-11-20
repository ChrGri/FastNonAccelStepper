#include <Arduino.h>
#include "FastNonAccelStepper.h"


float sineFrequencyInHz = 1;
float sineAmplitudeInSteps = 1000;


int dirPin = 18;
int pulPin = 19;

FastNonAccelStepper stepper(pulPin, dirPin, false);


void setup() {
    Serial.begin(115200);
    stepper.setMaxSpeed(1000);
}


void loop() {	


  float tInSeconds = (float)millis() / 1000.0f;

  uint32_t speed = 100000;
	// Kepp running in one direction
  // switch every N seconds
  if ( fmod(tInSeconds, 20) <= 10)
  {
    stepper.keepRunningForward(speed);
    delay(1000);
  }
  else
  {
    stepper.keepRunningBackward(speed);
    delay(1000);
  }
	

  //stepper.keepRunningForward();

	// Read and print the pulse count
	long currentPosition = stepper.getCurrentPosition();

	// print routine
	Serial.printf("currentPos:%d\n", currentPosition);		//the first variable for plotting

	delay(100);  // Adjust delay as needed
	    
}