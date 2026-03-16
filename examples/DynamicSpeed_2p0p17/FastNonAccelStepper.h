#ifndef FAST_NON_ACCEL_STEPPER_H
#define FAST_NON_ACCEL_STEPPER_H

#include <Arduino.h>
#include "driver/rmt.h"
#include "driver/pcnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class FastNonAccelStepper {
public:
    // Constructor
    FastNonAccelStepper(uint8_t stepPin_u8, uint8_t dirPin_u8, bool invertMotorDir_b = false);

    // Initialization: Must be called in setup() to spin up the FreeRTOS task
    void begin(rmt_channel_t rmtChannel = RMT_CHANNEL_0);

    // Speed Control
    void IRAM_ATTR setMaxSpeed(uint32_t speed_u32);
    void IRAM_ATTR setSpeedLive(uint32_t speed_u32);
    uint32_t IRAM_ATTR getMaxSpeed(void);

    // Movement Commands (blocking_b = true will pause the calling loop until done)
    void IRAM_ATTR move(int32_t stepsToMove_i32, bool blocking_b = false);
    void IRAM_ATTR moveTo(int32_t targetPos_i32, bool blocking_b = false);
    
    // Infinite Movement Commands (Runs until forceStop() is called)
    void IRAM_ATTR keepRunningInDir(bool forwardDir_b, uint32_t speed_u32);
    void IRAM_ATTR keepRunningForward(uint32_t speed_u32);
    void IRAM_ATTR keepRunningBackward(uint32_t speed_u32);

    // Stop and Position Management
    void IRAM_ATTR forceStop();
    void IRAM_ATTR setCurrentPosition(int32_t newPosition_i32);
    void IRAM_ATTR forceStopAndNewPosition(int32_t newPosition_i32);
    
    // Status
    bool IRAM_ATTR isRunning();
    int32_t IRAM_ATTR getCurrentPosition() const;
    int32_t IRAM_ATTR getPositionAfterCommandsCompleted();
	
	// Set the expected update interval of your control loop (in microseconds)
    void IRAM_ATTR setExpectedCycleTimeUs(uint32_t cycleTimeUs_u32);

private:
    // Pins and Settings
    uint8_t stepPin_u8;
    uint8_t dirPin_u8;
    bool invertMotorDirection_b;
    uint8_t dirLevelForward_b;
    uint8_t dirLevelBackward_b;

    // --- PCNT (Pulse Counter) Hardware Tracking ---
    // PCNT is a hardware peripheral that physically counts the HIGH/LOW pulses on a pin.
    // It runs completely independently of the CPU, meaning 0 overhead for position tracking.
    uint8_t dirPcntLctrlMode_u8;
    uint8_t dirPcntHctrlMode_u8;
    volatile int32_t overflowCount_i32;
    int32_t zeroPosition_i32;
    void initPCNTMultiturn();
    static void IRAM_ATTR multiturnPCNTISR(void* arg_p);

    // --- RMT (Remote Control) Hardware Pulse Generation ---
    // RMT is a hardware peripheral that plays memory arrays as physical GPIO waveforms.
    rmt_channel_t rmtChannel_e;
    TaskHandle_t rmtTaskHandle;
    static void rmtFeedTaskWrapper(void *pvParameters);
    void rmtFeedTask();

    // State Variables
    // Declared as 'volatile' because they are modified by your main loop and 
    // read by the FreeRTOS background task concurrently.
    volatile int32_t targetPosition_i32;
    volatile int32_t stepsRemaining_i32;
    volatile uint32_t maxSpeed_u32;
    volatile uint32_t currentIntervalUs_u32;
    volatile int8_t directionMultiplier_i8;
    volatile bool isRunning_b;
    volatile bool runInfinite_b;
	
	// Add this with the other volatile state variables
    volatile uint32_t expectedCycleTimeUs_u32;
};

#endif