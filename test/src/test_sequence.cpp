#include <Arduino.h>
#include "FastNonAccelStepper.h"

#define STEP_PIN 18
#define DIR_PIN 19

// Instance pointer
FastNonAccelStepper* _stepper;

// Constants for the test
#define MAXIMUM_STEPPER_SPEED_U32 (uint32_t)250000
#define TEST_SPEED_U32 (uint32_t)(MAXIMUM_STEPPER_SPEED_U32 / 1) 

void setup() 
{
    Serial.begin(115200);
    // Wait for USB Serial to connect (essential for S3)
    while(!Serial) 
    { 
        delay(10); 
    } 
    Serial.println("System Ready - Starting Sequence Test...");
    
    // Initializing hardware
    _stepper = new FastNonAccelStepper(STEP_PIN, DIR_PIN, false); 
    _stepper->begin();
    
    // Initial speed setup
    _stepper->setMaxSpeed(MAXIMUM_STEPPER_SPEED_U32);
    _stepper->setExpectedCycleTimeUs(1000);

    delay(1000);
}

void loop() 
{
    // 1. Run backward for 2 seconds at 1/16 speed
    Serial.println("Action: Running Backward...");
    _stepper->keepRunningBackward(TEST_SPEED_U32);
    _stepper->setSpeedLive(TEST_SPEED_U32); 
    delay(2000);

    // 2. Force stop and redefine the current position as -2500
    Serial.println("Action: Force Stop & Reset Position to -2500");
    _stepper->forceStopAndNewPosition(-2500);
    delay(50);

    // print current position to verify it was set correctly
    Serial.printf("Current Position after reset: %d\n", _stepper->getCurrentPosition());

    // 3. Move back to 0 (blocking call)
    Serial.println("Action: Moving to 0...");
    _stepper->setMaxSpeed(TEST_SPEED_U32); // Use a safe speed for the return trip
    _stepper->moveTo(0, true); 

    // 4. Wait for 500ms before repeating
    Serial.println("Sequence complete. Waiting 500ms...");
    delay(500);
}