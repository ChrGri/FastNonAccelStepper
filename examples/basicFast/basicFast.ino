#include "FastNonAccelStepper.h"

// The purpose of this test sketch is to test the correctness of by counting the transmitted pulse captured by a logic analyzer.

// Define Step, Dir und Enable pins
#define STEP_PIN 2
#define DIR_PIN 3
#define ENABLE_PIN 4

// Create the StepperMotor object
FastNonAccelStepper motor(STEP_PIN, DIR_PIN, ENABLE_PIN);

void setup() {
    Serial.begin(115200);
    motor.begin();           // Initialises the motor control unit
    motor.setMaxSpeed(30); // Sets the maximum step frequency to 2000 Hz
}

long targetPos = 100;
void loop() {

    

    targetPos += 100;
    targetPos %= 1000;

    motor.moveTo(targetPos);    // Move the motor to position 10 steps

    Serial.println( motor.currentPosition() );
    Serial.println( motor.targetPosition() );
    Serial.println( motor.currentPosition() );
    /*delay(10);
    Serial.println( motor.currentPosition() );
    delay(10);
    Serial.println( motor.currentPosition() );
    delay(10);
    Serial.println( motor.currentPosition() );
    delay(10);*/



    //delay(50);           // Wait 50 milliseconds
    //motor.moveTo(-10);   // Move the motor back to position -10 steps
    delay(500);           // Wait 50 milliseconds
}
