#include <Arduino.h>
#include "FastNonAccelStepper.h"


float sineFrequencyInHz = 1;
float sineAmplitudeInSteps = 1000;


int dirPin = 18;
int pulPin = 19;

FastNonAccelStepper stepper;


void setup() {
    Serial.begin(115200);
    stepper.begin(pulPin, dirPin);
    stepper.setMaxSpeed(1000);
}


void loop() {	



	// Kepp running in one direction
  // switch every N seconds
  /*if (fmod(millis()/1000, 10) == 0)
  {
    stepper.keepRunningForward();
    delay(1000);
  }
  else
  {
    stepper.keepRunningBackward();
    delay(1000);
  }*/
	

  stepper.keepRunningForward();

	// Read and print the pulse count
	long currentPosition = stepper.getCurrentPosition();

	// print routine
	Serial.printf("currentPos:%d\n", currentPosition);		//the first variable for plotting

	delay(100);  // Adjust delay as needed
	    
}