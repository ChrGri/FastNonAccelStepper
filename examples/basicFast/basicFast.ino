#include "FastNonAccelStepper.h"

// The purpose of this test sketch is to test the correctness of by counting the transmitted pulse captured by a logic analyzer.

// Define Step, Dir und Enable pins
#define STEP_PIN 2
#define DIR_PIN 3
#define ENABLE_PIN 4

// Create the StepperMotor object
StepperMotor motor(STEP_PIN, DIR_PIN, ENABLE_PIN);

void setup() {
    Serial.begin(115200);
    motor.begin();           // Initialises the motor control unit
    motor.setMaxSpeed(300000); // Sets the maximum step frequency to 2000 Hz
}

void loop() {
    motor.moveTo(10);    // Move the motor to position 10 steps
    delay(50);           // Wait 50 milliseconds
    motor.moveTo(-10);   // Move the motor back to position -10 steps
    delay(50);           // Wait 50 milliseconds
}
