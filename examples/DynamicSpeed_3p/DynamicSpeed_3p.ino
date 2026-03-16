#include <Arduino.h>
#include "FastNonAccelStepper.h"

// Define your pins
#define STEP_PIN 18
#define DIR_PIN 19

// Create stepper instance
FastNonAccelStepper stepper(STEP_PIN, DIR_PIN, false);

// Timing variables for the high-speed control loop (300us)
unsigned long previousMicros = 0;
const unsigned long loopInterval_us = 300; 

// Timing variables for the serial plotter loop (20ms)
unsigned long previousPlotMillis = 0;
const unsigned long plotInterval_ms = 20; 

// State tracking
bool currentDirection = true;
bool isFirstRun = true;

// Shared variable so the slow plot loop can see what the fast control loop is doing
int32_t currentCommandedVelocity = 0; 

void setup() {
    Serial.begin(115200);
    
    // Initialize the stepper and RMT/PCNT hardware
    stepper.begin();
	stepper.setExpectedCycleTimeUs(300); // Now it will batch 1000us worth of steps!
    
    // Brief pause before starting (helps avoid missing the first serial outputs)
    delay(1000); 
}

void loop() {
    unsigned long currentMicros = micros();
    unsigned long currentMillis = millis();

    // ---------------------------------------------------------
    // TASK 1: High-Speed Control Loop (Updates every 300us)
    // ---------------------------------------------------------
    if (currentMicros - previousMicros >= loopInterval_us) {
        previousMicros = currentMicros;

        // 1. Calculate the time in seconds for the sine function
        float time_sec = currentMicros / 1000000.0;

        // 2. Calculate sine wave: sin(2 * PI * frequency * time)
        float sineValue = sin(2.0 * PI * 0.1 * time_sec); 

        // 3. Map sine value to speed (-250,000 to 250,000)
        int32_t targetVelocity = sineValue * 250000.0;
        
        // Save the signed velocity for the plotter
        currentCommandedVelocity = targetVelocity; 

        // 4. Determine direction and absolute speed
        bool targetDirection = (targetVelocity >= 0);
        uint32_t absoluteSpeed = abs(targetVelocity);

        // Prevent speed from hitting 0 (which would cause a divide-by-zero interval)
        if (absoluteSpeed < 1) {
            absoluteSpeed = 1;
        }

        // 5. Apply the updates to the hardware
        if (targetDirection != currentDirection || isFirstRun) {
            stepper.keepRunningInDir(targetDirection, absoluteSpeed);
            currentDirection = targetDirection;
            isFirstRun = false;
        } else {
            stepper.setSpeedLive(absoluteSpeed);
        }
    }

    // ---------------------------------------------------------
    // TASK 2: Telemetry Loop (Updates Serial Plotter every 20ms)
    // ---------------------------------------------------------
    if (currentMillis - previousPlotMillis >= plotInterval_ms) {
        previousPlotMillis = currentMillis;

        // Grab the physical hardware position from the PCNT counter
        int32_t actualPosition = stepper.getCurrentPosition();

        // Print using the "Label:Value, Label:Value" format 
        // which the Arduino IDE 2.x Serial Plotter automatically parses into legends.
        Serial.print("Speed:");
        Serial.print(currentCommandedVelocity);
        Serial.print(",");
        Serial.print("Position:");
        Serial.println(actualPosition);
    }
}