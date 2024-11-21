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
    FastNonAccelStepper(uint8_t stepPin, uint8_t dirPin, bool invertMotorDir);

    /**
     * @brief Set the maximum speed of the stepper motor.
     * @param speed Maximum speed in Hz.
     */
    void setMaxSpeed(uint32_t speed);

    /**
     * @brief Update the target position for the stepper motor.
     * @param targetPos The desired target position in steps.
     * @param blocking Wait until the move has finished.
     */
    void moveTo(long targetPos, bool blocking);

    /**
     * @brief Read the current position of the stepper motor.
     * @return The current position in steps.
     */
    long getCurrentPosition() const;

    /**
    * @brief Move the stepper motor by a specified number of steps.
    * @param stepsToMove The number of steps to move (positive for forward, negative for backward).
    * @param blocking Wait until the move has finished.
    */
    void move(long stepsToMove, bool blocking);

    /**
     * @brief Force the stepper motor to stop immediately.
     */
    void forceStop();

    /**
     * @brief Set the current position of the stepper motor.
     * @param newPosition_i32 The new position in steps.
     */
    void setCurrentPosition(int32_t newPosition_i32);

    /**
     * @brief Force the stepper motor to stop and set a new current position.
     * @param newPosition_i32 The new position in steps.
     */
    void forceStopAndNewPosition(int32_t newPosition_i32);

    /**
     * @brief Check if the stepper motor is currently running.
     * @return True if the motor is running, false otherwise.
     */
    bool isRunning();

    /**
     * @brief Keep the stepper motor running in a specified direction.
     * @param forwardDir True to run forward, false to run backward.
     * @param speed The speed at which to run the motor in Hz.
     */
    void keepRunningInDir(bool forwardDir, uint32_t speed);

    /**
     * @brief Keep the stepper motor running forward.
     * @param speed The speed at which to run the motor in Hz.
     */
    void keepRunningForward(uint32_t speed);

    /**
     * @brief Keep the stepper motor running backward.
     * @param speed The speed at which to run the motor in Hz.
     */
    void keepRunningBackward(uint32_t speed);

    /**
     * @brief Get the motor's position after all commanded moves are completed.
     * @return The final position in steps.
     */
    int32_t getPositionAfterCommandsCompleted();


    

private:
    FastNonAccelStepper* _stepper;
    uint8_t _stepPin;          ///< Step pin number.
    uint8_t _dirPin;           ///< Direction pin number.
    long _targetPosition;      ///< Target position in steps.
    uint32_t _maxSpeed;            ///< Maximum speed in Hz.
    volatile int _overflowCount; ///< Overflow count for multiturn position tracking.
    volatile int _overflowCountControl; ///< Overflow count for control position tracking.

    int32_t _zeroPosition_i32 = 0;
    bool _isRunning = false;
    bool _invertMotorDirection = false;

    // Variable to store the DIR pin logic
    bool _dir_level_forward_b = true;
    bool _dir_level_backward_b = false;
    uint8_t _dir_pcnt_lctrl_mode_b = 0;
    uint8_t _dir_pcnt_hctrl_mode_b = 0;


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

    /**
     * @brief Initialize the stepper motor with specified step and direction pins.
     * @param stepPin The GPIO pin connected to the step input of the stepper motor driver.
     * @param dirPin The GPIO pin connected to the direction input of the stepper motor driver.
     * @param invertMotorDir Invert the direction pin logic.
     */
    void begin(uint8_t stepPin, uint8_t dirPin, bool invertMotorDir);


};

#endif // FASTNONACCELSTEPPER_H
