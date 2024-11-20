#include <Arduino.h>
#include "FastNonAccelStepper.h"



int dirPin = 18;
int pulPin = 19;

FastNonAccelStepper stepper(pulPin, dirPin, false);


void setup() {
    Serial.begin(115200);
    stepper.setMaxSpeed(100);

    delay(2000);
}



int32_t targetPos = 10000;

void loop() {	


  float tInSeconds = (float)millis() / 1000.0f;

  uint32_t speed = 10000;
  
  stepper.setMaxSpeed(speed);

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

    // print routine
    Serial.printf("targetPos:%d, retTargetPos:%d, currentPos:%d\n", targetPos, returnedTargetPos, currentPosition);		//the first variable for plotting

    delay(100);
  }

	    
}