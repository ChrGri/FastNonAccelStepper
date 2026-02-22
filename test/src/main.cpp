#include <Arduino.h>
#include <FastNonAccelStepper.h>

// Define your stepper motor connections and settings
#define STEP_PIN 2
#define DIR_PIN 3
#define INVERT_DIR false

// Create a new stepper instance
FastNonAccelStepper stepper = FastNonAccelStepper(STEP_PIN, DIR_PIN, INVERT_DIR);

void setup() 
{
  // Set a default speed
  stepper.setMaxSpeed(1000); // in Hz
}

void loop() 
{
  // Example usage:
  // Move 2000 steps forward and wait
  stepper.move(2000, true);
  delay(1000);

  // Move 2000 steps backward and wait
  stepper.move(-2000, true);
  delay(1000);
}
