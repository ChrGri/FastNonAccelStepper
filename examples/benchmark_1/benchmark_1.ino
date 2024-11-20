#include <Arduino.h>

//#define NON_ACCEL_STEPPER
#ifdef NON_ACCEL_STEPPER
  #include "FastNonAccelStepper.h"
#else
  #include "FastAccelStepper.h"
#endif

float sineFrequencyInHz = 10;
float sineAmplitudeInSteps = 1000;


int dirPin = 18;
int pulPin = 19;

int dirPin2 = 20;
int pulPin2 = 21;


#ifdef NON_ACCEL_STEPPER
  FastNonAccelStepper stepper(pulPin, dirPin, false);
#else
  FastAccelStepperEngine engine = FastAccelStepperEngine();
  FastAccelStepper *stepper2 = NULL;
#endif

void setup() {
  Serial.begin(921600);
  //Serial.begin(115200);

  #ifdef NON_ACCEL_STEPPER
    stepper.setMaxSpeed(500000);
  #else
    engine.init();
    stepper2 = engine.stepperConnectToPin(pulPin2);
    if (stepper2) {
      stepper2->setDirectionPin(dirPin2);
      //stepper->setEnablePin(26);
      stepper2->setAutoEnable(true);

      
      //uint32_t speed_in_hz = TICKS_PER_S / ticks;
      // TICKS_PER_S = 16000000L
      // ticks = TICKS_PER_S / speed_in_hz
      #define MAXIMUM_STEPPER_SPEED 500000
      #define maxSpeedInTicks  (TICKS_PER_S / MAXIMUM_STEPPER_SPEED)

      stepper2->setAbsoluteSpeedLimit( maxSpeedInTicks ); // ticks
      //stepper->setSpeedInTicks( maxSpeedInTicks ); // ticks
      stepper2->setSpeedInUs( 2 ); // ticks
      stepper2->setAcceleration(INT32_MAX);  // steps/s²
      stepper2->setLinearAcceleration(0);
      stepper2->setForwardPlanningTimeInMs(1);



      /*stepper->setSpeedInUs(1000);  // the parameter is us/step !!!
      stepper->setAcceleration(100);
      stepper->move(1000);*/


    }
  #endif



  


}

static long targetPosition = 1000;  // Example target position
uint64_t cycle = 0;
void loop() {


  float tInSeconds = (float)millis() / 1000.0f;


  // choose target pattern
  uint8_t targetPattern_u8 = 1;
  float targetPos_fl32 = 0;
  switch (targetPattern_u8) {
    case 0:
      targetPosition += 500;                                // Increment target position cyclically
      if (targetPosition > 50000) targetPosition = -50000;  // Reset after reaching max range
      break;
    case 1:
      // sine wave
      targetPos_fl32 = sineAmplitudeInSteps * sin(2.0f * M_PI * sineFrequencyInHz * tInSeconds);
      targetPosition = targetPos_fl32;
      break;
    case 2:
      // step
      if (fmod(tInSeconds, 2) > 1) {
        targetPosition = -sineAmplitudeInSteps;
      } else {
        targetPosition = sineAmplitudeInSteps;
      }
      break;
  }

  // Update PCNT limits and target
  long currentPosition;
  long currentPosition2;
  #ifdef NON_ACCEL_STEPPER
    stepper.moveTo(targetPosition);
    // Read and print the pulse count
    currentPosition = stepper.getCurrentPosition();
  #else
    stepper2->moveTo(targetPosition, false);
    // Read and print the pulse count
    currentPosition2 = stepper2->getCurrentPosition();
  #endif
  

  

  // print routine
  cycle++;
  if (cycle % 5 == 0)
  {
    //Serial.printf("tarPos:%d,curPos:%d\n", targetPosition, currentPosition);  //the first variable for plotting
    //Serial.printf("tarPos:%d\t curPos:%d\n", targetPosition, currentPosition);  //the first variable for plotting

    /*Serial.print(tInSeconds);
    Serial.print(" ");         // add spacing between variables
    
    Serial.print(targetPosition);  // output sin(t) variable
    Serial.print(" ");      // add spacing between variables

    Serial.print(currentPosition);  // output cos(t) variable
    Serial.println();*/


    //Serial.printf("%d %d %d\n", millis(), targetPosition, currentPosition);

    Serial.printf("%d %d %d\n", targetPosition, currentPosition, currentPosition2);
    
    

  }
  

  delayMicroseconds(500);  // Adjust delay as needed
}