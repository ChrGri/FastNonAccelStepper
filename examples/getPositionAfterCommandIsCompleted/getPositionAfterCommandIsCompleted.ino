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



int32_t targetPos = 10000;

void loop() {	


  float tInSeconds = (float)millis() / 1000.0f;

  uint32_t speed = 3000;
  
  stepper.setMaxSpeed(speed);
  
  // flip poisition
  targetPos *= -1;

  stepper.moveTo(targetPos);
  bool isRunning = true;
  while( isRunning )
  {
    isRunning = stepper.isRunning();
    int32_t returnedTargetPos = stepper.getPositionAfterCommandsCompleted();

    // Read and print the pulse count
    long currentPosition = stepper.getCurrentPosition();

    // print routine
    Serial.printf("targetPos:%d, retTargetPos:%d, currentPos:%d\n", targetPos, returnedTargetPos, currentPosition);		//the first variable for plotting

    delay(100);
  }

	    
}