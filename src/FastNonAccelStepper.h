#ifndef FASTNONACCELSTEPPER_H
#define FASTNONACCELSTEPPER_H

#include <Arduino.h>

#define MAX_SPEED_IN_HZ (int32_t)250000

/**
 * @class FastNonAccelStepper
 * @brief A class to control a stepper motor using ESP32's MCPWM and PCNT modules.
 */
class FastNonAccelStepper
{
public:
    /**
     * @brief Constructor to initialize the FastNonAccelStepper.
     */
    FastNonAccelStepper(uint8_t stepPin_u8, uint8_t dirPin_u8, bool invertMotorDir_b);

    /**
     * @brief Set the maximum speed of the stepper motor.
     * @param speed_u32 Maximum speed in Hz.
     */
    void setMaxSpeed(uint32_t speed_u32);
	
	/**
     * @brief Get the maximum speed of the stepper motor.
     * @return Maximum speed in Hz.
     */
    uint32_t getMaxSpeed(void);

    /**
     * @brief Update the target position for the stepper motor.
     * @param targetPos_i32 The desired target position in steps.
     * @param blocking_b Wait until the move has finished.
     */
    void moveTo(int32_t targetPos_i32, bool blocking_b);

    /**
     * @brief Read the current position of the stepper motor.
     * @return The current position in steps.
     */
    int32_t getCurrentPosition() const;

    /**
    * @brief Move the stepper motor by a specified number of steps.
    * @param stepsToMove_i32 The number of steps to move (positive for forward, negative for backward).
    * @param blocking_b Wait until the move has finished.
    */
    void move(int32_t stepsToMove_i32, bool blocking_b);

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
     * @param forwardDir_b True to run forward, false to run backward.
     * @param speed_u32 The speed at which to run the motor in Hz.
     */
    void keepRunningInDir(bool forwardDir_b, uint32_t speed_u32);

    /**
     * @brief Keep the stepper motor running forward.
     * @param speed_u32 The speed at which to run the motor in Hz.
     */
    void keepRunningForward(uint32_t speed_u32);

    /**
     * @brief Keep the stepper motor running backward.
     * @param speed_u32 The speed at which to run the motor in Hz.
     */
    void keepRunningBackward(uint32_t speed_u32);

    /**
     * @brief Get the motor's position after all commanded moves are completed.
     * @return The final position in steps.
     */
    int32_t getPositionAfterCommandsCompleted();


    

private:
    FastNonAccelStepper* stepper_p;
    uint8_t stepPin_u8;          ///< Step pin number.
    uint8_t dirPin_u8;           ///< Direction pin number.
    int32_t targetPosition_i32;      ///< Target position in steps.
    uint32_t maxSpeed_u32;            ///< Maximum speed in Hz.
    volatile int32_t overflowCount_i32; ///< Overflow count for multiturn position tracking.
    volatile int32_t overflowCountControl_i32; ///< Overflow count for control position tracking.

    int32_t zeroPosition_i32 = 0;
    bool isRunning_b = false;
    bool invertMotorDirection_b = false;

    // Variable to store the DIR pin logic
    bool dirLevelForward_b = true;
    bool dirLevelBackward_b = false;
    uint8_t dirPcntLctrlMode_u8 = 0;
    uint8_t dirPcntHctrlMode_u8 = 0;


    QueueHandle_t pcntQueue;   ///< Queue to handle PCNT events.

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
     * @param arg_p ISR argument.
     */
    static void IRAM_ATTR multiturnPCNTISR(void* arg_p);

    /**
     * @brief Handle PCNT events for position control.
     * @param arg_p ISR argument.
     */
    static void IRAM_ATTR controlPCNTISR(void* arg_p); 

    /**
     * @brief Initialize the stepper motor with specified step and direction pins.
     * @param stepPin_u8 The GPIO pin connected to the step input of the stepper motor driver.
     * @param dirPin_u8 The GPIO pin connected to the direction input of the stepper motor driver.
     * @param invertMotorDir_b Invert the direction pin logic.
     */
    void begin(uint8_t stepPin_u8, uint8_t dirPin_u8, bool invertMotorDir_b);


};

#endif // FASTNONACCELSTEPPER_H
