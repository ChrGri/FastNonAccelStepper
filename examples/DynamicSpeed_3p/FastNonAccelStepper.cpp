#include "FastNonAccelStepper.h"
#include <rom/gpio.h> 
#include <driver/gpio.h> 
#include "soc/gpio_sig_map.h"
#include "esp_rom_gpio.h" // Needed for V5 internal signal routing

// PCNT limits (16-bit integer max)
#define PCNT_MIN_MAX_THRESHOLD (int16_t)32767
#define PCNT_FILTER_VALUE 1

FastNonAccelStepper::FastNonAccelStepper(uint8_t stepPin_u8, uint8_t dirPin_u8, bool invertMotorDir_b)
    : stepPin_u8(stepPin_u8), dirPin_u8(dirPin_u8), invertMotorDirection_b(invertMotorDir_b),
      targetPosition_i32(0), stepsRemaining_i32(0), maxSpeed_u32(1000), 
      currentIntervalUs_u32(1000), directionMultiplier_i8(1),
      isRunning_b(false), runInfinite_b(false), rmtTaskHandle(NULL),
      rmtChannel_e(NULL), copy_encoder(NULL),
      overflowCount_i32(0), zeroPosition_i32(0),
      expectedCycleTimeUs_u32(300) 
{
}

void FastNonAccelStepper::begin(int rmtChannel)
{
    pinMode(stepPin_u8, OUTPUT);
    pinMode(dirPin_u8, OUTPUT);
    digitalWrite(stepPin_u8, LOW);
    digitalWrite(dirPin_u8, LOW);

    // 1. Setup Direction Logic 
    if (false == invertMotorDirection_b) {
        dirLevelForward_b = LOW;
        dirLevelBackward_b = HIGH;
        dirPcntLctrlMode_u8 = PCNT_MODE_KEEP;    
        dirPcntHctrlMode_u8 = PCNT_MODE_REVERSE; 
    } else {
        dirLevelForward_b = HIGH;
        dirLevelBackward_b = LOW;
        dirPcntLctrlMode_u8 = PCNT_MODE_REVERSE;
        dirPcntHctrlMode_u8 = PCNT_MODE_KEEP;
    }

    // 2. Initialize Hardware PCNT Multiturn Tracker
    initPCNTMultiturn();
    
    // 3. Configure Hardware RMT Pulse Generator (V5 Next-Gen API)
    rmt_tx_channel_config_t tx_chan_config = {};
    tx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;
    tx_chan_config.gpio_num = (gpio_num_t)stepPin_u8;
    tx_chan_config.mem_block_symbols = 64;           
    tx_chan_config.resolution_hz = 1000000; 
    tx_chan_config.trans_queue_depth = 4;   
    
    // --- THE V5 MAGIC BULLET ---
    // This official flag tells the RMT driver to keep the input buffer ON 
    // so our PCNT peripheral can listen to the pulses flawlessly.
    tx_chan_config.flags.io_loop_back = true; 
    
    rmt_new_tx_channel(&tx_chan_config, &rmtChannel_e);

    rmt_copy_encoder_config_t copy_encoder_config = {};
    rmt_new_copy_encoder(&copy_encoder_config, &copy_encoder);

    rmt_enable(rmtChannel_e);

    // ----------------------------------------------------------------------
    // THE V5 SIGNAL ROUTING
    // ----------------------------------------------------------------------
    // Force the DIR pin to be bidirectional so PCNT can read it and overcome crosstalk.
    gpio_set_direction((gpio_num_t)dirPin_u8, GPIO_MODE_INPUT_OUTPUT); 
    
    // Route the physical pins to the PCNT input channels
    esp_rom_gpio_connect_in_signal(stepPin_u8, PCNT_SIG_CH0_IN0_IDX, false);
    esp_rom_gpio_connect_in_signal(dirPin_u8, PCNT_SIG_CH0_IN1_IDX, false);
    // ----------------------------------------------------------------------

    // 4. Start the Background FreeRTOS Task
    xTaskCreatePinnedToCore(
        rmtFeedTaskWrapper,
        "RMT_Stepper_Task",
        4096,                    
        this,                    
        configMAX_PRIORITIES - 1,
        &rmtTaskHandle,
        1                        
    );
}

// ------------------------------------------------------------------------
// PCNT HARDWARE POSITION TRACKING 
// ------------------------------------------------------------------------
void FastNonAccelStepper::initPCNTMultiturn()
{
    pcnt_config_t pcntConfig = {}; 
    pcntConfig.pulse_gpio_num = stepPin_u8;
    pcntConfig.ctrl_gpio_num = dirPin_u8;
    pcntConfig.channel = PCNT_CHANNEL_0;
    pcntConfig.unit = PCNT_UNIT_0;
    pcntConfig.pos_mode = PCNT_COUNT_INC; 
    pcntConfig.neg_mode = PCNT_COUNT_DIS; 
    pcntConfig.lctrl_mode = (pcnt_ctrl_mode_t)dirPcntLctrlMode_u8;
    pcntConfig.hctrl_mode = (pcnt_ctrl_mode_t)dirPcntHctrlMode_u8;
    
    pcntConfig.counter_h_lim = PCNT_MIN_MAX_THRESHOLD;
    pcntConfig.counter_l_lim = -PCNT_MIN_MAX_THRESHOLD;

    pcnt_unit_config(&pcntConfig);

    pcnt_set_filter_value(PCNT_UNIT_0, PCNT_FILTER_VALUE);
    pcnt_filter_enable(PCNT_UNIT_0);

    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_L_LIM);

    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(PCNT_UNIT_0, multiturnPCNTISR, this);
    
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
}

void IRAM_ATTR FastNonAccelStepper::multiturnPCNTISR(void* arg_p)
{
    FastNonAccelStepper* instance_p = static_cast<FastNonAccelStepper*>(arg_p);
    uint32_t status_u32;
    pcnt_get_event_status(PCNT_UNIT_0, &status_u32);

    if (status_u32 & PCNT_EVT_H_LIM) {
        instance_p->overflowCount_i32++;
    }
    if (status_u32 & PCNT_EVT_L_LIM) {
        instance_p->overflowCount_i32--;
    }
}

int32_t IRAM_ATTR FastNonAccelStepper::getCurrentPosition() const
{
    int16_t pulseCount_i16 = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &pulseCount_i16);
    return ((int32_t)overflowCount_i32 * (int32_t)PCNT_MIN_MAX_THRESHOLD) + (int32_t)pulseCount_i16 - zeroPosition_i32;
}

void IRAM_ATTR FastNonAccelStepper::setCurrentPosition(int32_t newPosition_i32)
{
    int32_t newZeroPos_i32 = (getCurrentPosition() + zeroPosition_i32) - newPosition_i32;
    zeroPosition_i32 = newZeroPos_i32;
}

// ------------------------------------------------------------------------
// RMT HARDWARE PULSE GENERATION ENGINE
// ------------------------------------------------------------------------
void FastNonAccelStepper::rmtFeedTaskWrapper(void *pvParameters)
{
    FastNonAccelStepper* instance = static_cast<FastNonAccelStepper*>(pvParameters);
    instance->rmtFeedTask();
}

void FastNonAccelStepper::rmtFeedTask()
{
    const int MAX_BATCH = 100;
    rmt_symbol_word_t step_batch[MAX_BATCH];

    while (true) 
    {
        if (isRunning_b) 
        {
            if (runInfinite_b || stepsRemaining_i32 > 0) 
            {
                uint32_t current_interval = currentIntervalUs_u32;
                uint32_t cycle_time = expectedCycleTimeUs_u32;
                
                uint32_t half_period = current_interval / 2;
                if (half_period < 1) half_period = 1;
                if (half_period > 32767) half_period = 32767; 

                int32_t pulses_in_batch = cycle_time / current_interval;
                
                if (pulses_in_batch < 1) pulses_in_batch = 1;
                if (pulses_in_batch > MAX_BATCH) pulses_in_batch = MAX_BATCH;

                if (!runInfinite_b && pulses_in_batch > stepsRemaining_i32) {
                    pulses_in_batch = stepsRemaining_i32;
                }

                for (int i = 0; i < pulses_in_batch; i++) {
                    step_batch[i].duration0 = half_period;
                    step_batch[i].level0    = 1;
                    step_batch[i].duration1 = half_period;
                    step_batch[i].level1    = 0;
                }

                rmt_transmit_config_t tx_config = {};
                tx_config.loop_count = 0; 
                
                esp_err_t transmit_result = rmt_transmit(rmtChannel_e, copy_encoder, step_batch, pulses_in_batch * sizeof(rmt_symbol_word_t), &tx_config);
                
                if (transmit_result == ESP_OK) {
                    if (!runInfinite_b) {
                        stepsRemaining_i32 -= pulses_in_batch;
                    }
                } else {
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
            } 
            else 
            {
                rmt_tx_wait_all_done(rmtChannel_e, -1);
                isRunning_b = false;
            }
        } 
        else 
        {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

// ------------------------------------------------------------------------
// Standard Methods
// ------------------------------------------------------------------------
void IRAM_ATTR FastNonAccelStepper::setExpectedCycleTimeUs(uint32_t cycleTimeUs_u32) {
    if (cycleTimeUs_u32 < 1) {
        expectedCycleTimeUs_u32 = 1;
    } else {
        expectedCycleTimeUs_u32 = cycleTimeUs_u32;
    }
}

void IRAM_ATTR FastNonAccelStepper::setMaxSpeed(uint32_t speed_u32) {
    setSpeedLive(speed_u32);
}

void IRAM_ATTR FastNonAccelStepper::setSpeedLive(uint32_t speed_u32) {
    maxSpeed_u32 = constrain(speed_u32, 20, 250000); 
    currentIntervalUs_u32 = 1000000 / maxSpeed_u32;
}

uint32_t IRAM_ATTR FastNonAccelStepper::getMaxSpeed(void) {
    return maxSpeed_u32;
}

void IRAM_ATTR FastNonAccelStepper::move(int32_t stepsToMove_i32, bool blocking_b) {
    if (stepsToMove_i32 == 0) return;

    forceStop(); 

    if (stepsToMove_i32 > 0) {
        digitalWrite(dirPin_u8, dirLevelForward_b);
        directionMultiplier_i8 = 1;
    } else {
        digitalWrite(dirPin_u8, dirLevelBackward_b);
        directionMultiplier_i8 = -1;
    }

    stepsRemaining_i32 = abs(stepsToMove_i32);
    targetPosition_i32 = getCurrentPosition() + stepsToMove_i32;
    runInfinite_b = false;
    isRunning_b = true; 

    if (blocking_b) {
        while(isRunning_b) delay(1);
    }
}

void IRAM_ATTR FastNonAccelStepper::moveTo(int32_t targetPos_i32, bool blocking_b) {
    move(targetPos_i32 - getCurrentPosition(), blocking_b);
} 

void IRAM_ATTR FastNonAccelStepper::forceStop() {
    isRunning_b = false;
    stepsRemaining_i32 = 0;
    runInfinite_b = false;
    
    // In V5, disabling and re-enabling flushes the active transmission safely.
    // The driver maintains ownership of the pin at all times!
    if (rmtChannel_e != NULL) {
        rmt_disable(rmtChannel_e);
        rmt_enable(rmtChannel_e);
    }
}  

void IRAM_ATTR FastNonAccelStepper::forceStopAndNewPosition(int32_t newPosition_i32) {
    forceStop();
    setCurrentPosition(newPosition_i32);
}   

bool IRAM_ATTR FastNonAccelStepper::isRunning() {
    return isRunning_b;
}

void IRAM_ATTR FastNonAccelStepper::keepRunningInDir(bool forwardDir_b, uint32_t speed_u32) {
    if (forwardDir_b) {
        digitalWrite(dirPin_u8, dirLevelForward_b);
        directionMultiplier_i8 = 1;
    } else {
        digitalWrite(dirPin_u8, dirLevelBackward_b);
        directionMultiplier_i8 = -1;
    }

    setSpeedLive(speed_u32); 
    runInfinite_b = true;
    isRunning_b = true; 
}

void IRAM_ATTR FastNonAccelStepper::keepRunningForward(uint32_t speed_u32) {
    keepRunningInDir(true, speed_u32);
}

void IRAM_ATTR FastNonAccelStepper::keepRunningBackward(uint32_t speed_u32) {
    keepRunningInDir(false, speed_u32);
}

int32_t IRAM_ATTR FastNonAccelStepper::getPositionAfterCommandsCompleted() {
    return targetPosition_i32;
}