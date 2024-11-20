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



int32_t targetPos = 10000;

void loop() {	


  float tInSeconds = (float)millis() / 1000.0f;

  uint32_t speed = 3000;
  
  stepper.setMaxSpeed(speed);

  stepper.setCurrentPosition(-1000);

  long currentPosition = stepper.getCurrentPosition();
  //Serial.printf("targetPos:%d, retTargetPos:%d, currentPos:%d\n", targetPos, returnedTargetPos, currentPosition);
  
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

    if (currentPosition > 5000)
    {
      //stepper.forceStop();
      stepper.forceStopAndNewPosition(-20000);
    }

    // print routine
    Serial.printf("targetPos:%d, retTargetPos:%d, currentPos:%d\n", targetPos, returnedTargetPos, currentPosition);		//the first variable for plotting

    delay(100);
  }

	    
}