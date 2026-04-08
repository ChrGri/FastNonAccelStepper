#include <Arduino.h>
#include "FastNonAccelStepper.h"

// Pins matching your setup
#define STEP_PIN 18
#define DIR_PIN 19

//FastNonAccelStepper stepper(STEP_PIN, DIR_PIN, false);

FastNonAccelStepper* _stepper;

// Extracted from your provided constants
#define MAX_SPEED_IN_HZ_U32 (uint32_t)250000
#define TEST_SPEED_U32 (MAX_SPEED_IN_HZ_U32 / 16) // 15625 Hz

void setup()
{
    Serial.begin(115200);
    while(!Serial) { delay(10); } // Wait for USB Serial to connect
    Serial.println("System Ready...");
    
    Serial.println("--- MCPWM Logic Analyzer Test Start ---");
    Serial.printf("Target Speed: %u Hz\n", TEST_SPEED_U32);

    // Initializing hardware
    _stepper = new FastNonAccelStepper(STEP_PIN, DIR_PIN, false); 
    _stepper->begin();
    _stepper->setMaxSpeed(TEST_SPEED_U32);
    _stepper->setExpectedCycleTimeUs(1000);

    // Give the hardware and serial time to settle
    delay(2000);
}

void loop()
{
    Serial.println("Executing: keepRunningBackward...");

    // This mimics your StepperWithLimits.cpp line:
    // _stepper->keepRunningBackward(MAXIMUM_STEPPER_SPEED_U32 / 16);
    _stepper->keepRunningBackward(TEST_SPEED_U32);

    // Run for 2 seconds so you can see it on the analyzer
    delay(2000);

    Serial.println("Executing: forceStop...");
    _stepper->forceStop();

    // Wait 3 seconds before next burst
    delay(3000);
}