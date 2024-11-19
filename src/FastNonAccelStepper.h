#ifndef FASTNONACCELSTEPPER_H
#define FASTNONACCELSTEPPER_H

#include <Arduino.h>


/**
 * @class FastNonAccelStepper
 * @brief A class to control a stepper motor using ESP32's MCPWM and PCNT modules.
 */
class FastNonAccelStepper {
public:
    /**
     * @brief Constructor to initialize the FastNonAccelStepper.
     */
    FastNonAccelStepper();

    /**
     * @brief Initialize the stepper motor with specified step and direction pins.
     * @param stepPin The GPIO pin connected to the step input of the stepper motor driver.
     * @param dirPin The GPIO pin connected to the direction input of the stepper motor driver.
     */
    void begin(uint8_t stepPin, uint8_t dirPin);

    /**
     * @brief Set the maximum speed of the stepper motor.
     * @param speed Maximum speed in Hz.
     */
    void setMaxSpeed(uint32_t speed);

    /**
     * @brief Update the target position for the stepper motor.
     * @param targetPos The desired target position in steps.
     */
    void moveTo(long targetPos);

    /**
     * @brief Read the current position of the stepper motor.
     * @return The current position in steps.
     */
    long getCurrentPosition() const;

    /**
     * @brief Run the stepper motor control logic. Call this frequently in the loop.
     */
    void run();

private:
    uint8_t _stepPin;          ///< Step pin number.
    uint8_t _dirPin;           ///< Direction pin number.
    long _targetPosition;      ///< Target position in steps.
    uint32_t _maxSpeed;            ///< Maximum speed in Hz.
    volatile int _overflowCount; ///< Overflow count for multiturn position tracking.

    xQueueHandle _pcntQueue;   ///< Queue to handle PCNT events.

    /**
     * @brief Initialize the MCPWM module for step signal generation.
     */
    void initMCPWM();

    /**
     * @brief Initialize the PCNT module for multiturn position tracking.
     */
    void initPCNTMultiturn();

    /**
     * @brief Initialize the PCNT module for position control.
     */
    void initPCNTControl();

    /**
     * @brief Handle PCNT events for multiturn tracking.
     * @param arg ISR argument.
     */
    static void IRAM_ATTR multiturnPCNTISR(void* arg);

    /**
     * @brief Handle PCNT events for position control.
     * @param arg ISR argument.
     */
    static void IRAM_ATTR controlPCNTISR(void* arg);
};

#endif // FASTNONACCELSTEPPER_H
