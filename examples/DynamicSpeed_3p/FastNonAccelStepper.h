#ifndef FAST_NON_ACCEL_STEPPER_H
#define FAST_NON_ACCEL_STEPPER_H

#include <Arduino.h>
#include "driver/rmt_tx.h" // ESP-IDF V5 Next Generation RMT API
#include "driver/pcnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class FastNonAccelStepper {
public:
    FastNonAccelStepper(uint8_t stepPin_u8, uint8_t dirPin_u8, bool invertMotorDir_b = false);
    
    // In V5, channels are assigned dynamically, but we keep the parameter for backwards compatibility
    void begin(int rmtChannel = 0);

    void IRAM_ATTR setMaxSpeed(uint32_t speed_u32);
    void IRAM_ATTR setSpeedLive(uint32_t speed_u32);
    uint32_t IRAM_ATTR getMaxSpeed(void);
    void IRAM_ATTR setExpectedCycleTimeUs(uint32_t cycleTimeUs_u32);

    void IRAM_ATTR move(int32_t stepsToMove_i32, bool blocking_b = false);
    void IRAM_ATTR moveTo(int32_t targetPos_i32, bool blocking_b = false);
    
    void IRAM_ATTR keepRunningInDir(bool forwardDir_b, uint32_t speed_u32);
    void IRAM_ATTR keepRunningForward(uint32_t speed_u32);
    void IRAM_ATTR keepRunningBackward(uint32_t speed_u32);

    void IRAM_ATTR forceStop();
    void IRAM_ATTR setCurrentPosition(int32_t newPosition_i32);
    void IRAM_ATTR forceStopAndNewPosition(int32_t newPosition_i32);
    
    bool IRAM_ATTR isRunning();
    int32_t IRAM_ATTR getCurrentPosition() const;
    int32_t IRAM_ATTR getPositionAfterCommandsCompleted();

private:
    uint8_t stepPin_u8;
    uint8_t dirPin_u8;
    bool invertMotorDirection_b;
    uint8_t dirLevelForward_b;
    uint8_t dirLevelBackward_b;

    // PCNT Hardware Tracking
    uint8_t dirPcntLctrlMode_u8;
    uint8_t dirPcntHctrlMode_u8;
    volatile int32_t overflowCount_i32;
    int32_t zeroPosition_i32;
    void initPCNTMultiturn();
    static void IRAM_ATTR multiturnPCNTISR(void* arg_p);

    // RMT V5 Hardware Handles
    rmt_channel_handle_t rmtChannel_e; 
    rmt_encoder_handle_t copy_encoder; 
    TaskHandle_t rmtTaskHandle;
    static void rmtFeedTaskWrapper(void *pvParameters);
    void rmtFeedTask();

    // State Variables
    volatile int32_t targetPosition_i32;
    volatile int32_t stepsRemaining_i32;
    volatile uint32_t maxSpeed_u32;
    volatile uint32_t currentIntervalUs_u32;
    volatile int8_t directionMultiplier_i8;
    volatile bool isRunning_b;
    volatile bool runInfinite_b;
    volatile uint32_t expectedCycleTimeUs_u32;
};

#endif