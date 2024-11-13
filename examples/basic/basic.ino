#include "FastNonAccelStepper.h"

// Define Step, Dir und Enable pins
#define STEP_PIN 2
#define DIR_PIN 3
#define ENABLE_PIN 4

// Create the StepperMotor object
StepperMotor motor(STEP_PIN, DIR_PIN, ENABLE_PIN);

void setup() {
    Serial.begin(115200);
    motor.begin();           // Initialises the motor control unit
    motor.setMaxSpeed(2000); // Sets the maximum step frequency to 2000 Hz
}

void loop() {
    motor.moveTo(1000);    // Move the motor to position 1000 steps
    delay(5000);           // Wait 5 seconds
    motor.moveTo(-1000);   // Move the motor back to position -1000 steps
    delay(5000);           // Wait 5 seconds
    Serial.println(motor.currentPosition()); // Shows the current position
}
