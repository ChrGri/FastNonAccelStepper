#include "FastNonAccelStepper.h"
#include <driver/pcnt.h>


/************************************************************************/
/*								Defines 								                              */
/************************************************************************/
#define MAX_ALLOWED_POSITION_CHANGE_PER_CYCLE (int32_t)20000
#define PCNT_MIN_MAX_THRESHOLD (int16_t)32767 // INT16_MAX = (2^15)-1 = 32767
#define POSITION_TRIGGER_THRESHOLD 1
#define PCNT_FILTER_VALUE 1
#define RMT_RESOLUTION_HZ 1000000 // 1 MHz resolution, 1 tick = 1 us
#define PULSE_WIDTH_US 5 // The duration of the high pulse in microseconds


/************************************************************************/
/*								Implementation							                          */
/************************************************************************/
FastNonAccelStepper::FastNonAccelStepper(uint8_t stepPin_u8, uint8_t dirPin_u8, bool invertMotorDir_b)
    : stepPin_u8(stepPin_u8), dirPin_u8(dirPin_u8), targetPosition_i32(0), maxSpeed_u32(MAX_SPEED_IN_HZ), overflowCount_i32(0), pcntQueue(nullptr), invertMotorDirection_b(invertMotorDir_b), zeroPosition_i32(0)
{
    stepper_p = this;  // Assign the current instance to stepper_p
    stepper_p->begin(stepPin_u8, dirPin_u8, invertMotorDirection_b);
}

void FastNonAccelStepper::begin(uint8_t stepPin_u8, uint8_t dirPin_u8, bool invertMotorDir_b)
{
    // set dir logic for counting
    // 1) If invertMotorDir_b == false, the DIR_PIN == HIGH, when motor moving forward and DIR_PIN == LOW, when motor moving backwards
    // 2) If invertMotorDir_b == false, the PCNT counter should go up, when DIR_PIN == HIGH
    if (false == invertMotorDir_b)
    {
        dirLevelForward_b = LOW;
        dirLevelBackward_b = HIGH;
        dirPcntLctrlMode_u8 = PCNT_MODE_KEEP;
        dirPcntHctrlMode_u8 = PCNT_MODE_REVERSE;
    }
    else
    {
        dirLevelForward_b = HIGH;
        dirLevelBackward_b = LOW;
        dirPcntLctrlMode_u8 = PCNT_MODE_REVERSE;
        dirPcntHctrlMode_u8 = PCNT_MODE_KEEP;
    }

        // init RMT and PCNTs
    initRMT();
    initPCNTMultiturn(); // unit to track the overall position

    pinMode(dirPin_u8, OUTPUT);
    digitalWrite(dirPin_u8, LOW);

    // connect PCNT to GPIO pin
    gpio_iomux_in(stepPin_u8, PCNT_SIG_CH0_IN0_IDX); // overall position unit

    // reset pcnt counters
    pcnt_counter_clear(PCNT_UNIT_0); // overall position unit

    // make sure RMT is stopped
    forceStop();
}

void IRAM_ATTR FastNonAccelStepper::setMaxSpeed(uint32_t speed_u32)
{
    // Constrain the speed to valid limits
    maxSpeed_u32 = constrain(speed_u32, 1, MAX_SPEED_IN_HZ);
}

void IRAM_ATTR FastNonAccelStepper::setSpeedLive(uint32_t speed_u32)
{
    // Only meaningful for MCPWM. We leave it as a no-op or alias for the new trajectory method.
    maxSpeed_u32 = constrain(speed_u32, 1, MAX_SPEED_IN_HZ);
}

void IRAM_ATTR FastNonAccelStepper::updateLiveTrajectory(uint32_t speed_u32, uint32_t updateInterval_us)
{
    if (speed_u32 == 0) {
        return;
    }
    
    // We want to generate 'speed_u32' pulses per second.
    // In 'updateInterval_us' microseconds, how many pulses should we send?
    // pulses = speed * (updateInterval_us / 1,000,000)
    
    // Duration of one full pulse cycle (high + low) in microseconds:
    uint32_t cycleDuration_us = 1000000 / speed_u32;
    if (cycleDuration_us <= PULSE_WIDTH_US) {
        cycleDuration_us = PULSE_WIDTH_US + 1; // absolute minimum safeguard
    }
    uint32_t lowDuration_us = cycleDuration_us - PULSE_WIDTH_US;
    
    // Calculate how many pulses fit in the current update interval
    uint32_t numPulses = (updateInterval_us * speed_u32) / 1000000;
    
    if (numPulses == 0) {
        return; // nothing to do in this short window for this speed
    }
    
    // Allocate RMT items (max we can allocate dynamically cleanly or statically)
    // For a 0.3ms window at 250kHz, max pulses = 0.0003 * 250000 = 75 pulses
    // We'll safely allocate an array large enough for the maximum reasonable expected burst.
    // Note: If numPulses is larger than this array, we constrain it to avoid stack overflow.
    const uint32_t MAX_BURST_PULSES = 200; 
    if (numPulses > MAX_BURST_PULSES) {
        numPulses = MAX_BURST_PULSES;
    }
    
    rmt_item32_t items[MAX_BURST_PULSES + 1]; // +1 for end marker
    
    for (uint32_t i = 0; i < numPulses; i++) {
        items[i].level0 = 1;
        items[i].duration0 = PULSE_WIDTH_US * rmtTicksPerUs;
        items[i].level1 = 0;
        items[i].duration1 = lowDuration_us * rmtTicksPerUs;
    }
    
    // RMT End Marker
    items[numPulses].level0 = 0;
    items[numPulses].duration0 = 0;
    items[numPulses].level1 = 0;
    items[numPulses].duration1 = 0;

    // Write sequence to RMT buffer. It will stream automatically.
    rmt_write_items(rmtChannel, items, numPulses + 1, false);
}

uint32_t IRAM_ATTR FastNonAccelStepper::getMaxSpeed(void)
{
    // Constrain the speed to valid limits
    return maxSpeed_u32;
}


void IRAM_ATTR FastNonAccelStepper::move(int32_t stepsToMove_i32, bool blocking_b)
{
    // stop previous move
    forceStop();

    int32_t absStepsToMove_i32 = abs(stepsToMove_i32);

    if (absStepsToMove_i32 > POSITION_TRIGGER_THRESHOLD)
    {
	  // MATLAB code:
	  // PCNT_MIN_MAX_THRESHOLD = 32767; absStepsToMove = 2*PCNT_MIN_MAX_THRESHOLD; nmbWraps = idivide( int32(absStepsToMove) , int32(PCNT_MIN_MAX_THRESHOLD) ); limit = absStepsToMove / (nmbWraps+1)
	  // close all; clear all; clc;
	  // PCNT_MIN_MAX_THRESHOLD = 32767; 
	  // absStepsToMove = int32( (0:1:1000) * PCNT_MIN_MAX_THRESHOLD ); 
	  // nmbWraps = idivide( int32(absStepsToMove), int32(PCNT_MIN_MAX_THRESHOLD) ); 
	  // limit = absStepsToMove ./ (nmbWraps+1);
	  // nexttile()
	  // plot(limit)
	  // nexttile()
  	  // plot(absStepsToMove, (nmbWraps+1) .* limit )
	  // title('Step output')
	  // xlabel('True steps \rightarrow')
	  // ylabel('Actual steps \rightarrow')
	  // nexttile()
	  // plot( (nmbWraps+1) .* limit - absStepsToMove)
	  // title('Step Error')

        // compute the number of wraps
        int32_t numbWraps_i32 = absStepsToMove_i32 / PCNT_MIN_MAX_THRESHOLD;

	  // divide into equally spaced bursts, e.g. when 
	  // if absStepsToMove is < PCNT_MIN_MAX_THRESHOLD --> numbWraps = 0 and limit_i16 = absStepsToMove
	  // if absStepsToMove == PCNT_MIN_MAX_THRESHOLD --> numbWraps = 1 and limit_i16 = absStepsToMove/2
	  // if absStepsToMove == 2*PCNT_MIN_MAX_THRESHOLD --> numbWraps = 2 and limit_i16 = absStepsToMove/3
	  // if absStepsToMove > PCNT_MIN_MAX_THRESHOLD --> numbWraps >= 1
        int32_t limit_i32 = absStepsToMove_i32 / (numbWraps_i32 + 1);
        int16_t limit_i16 = constrain(limit_i32, 0, PCNT_MIN_MAX_THRESHOLD);

        // the highLimit | lowLimit will be hit numbWraps, before stopping the PWM output
        int16_t highLimit_i16;
        int16_t lowLimit_i16;

                // 1) set DIR pin
        // 2) stream pulses
        if (stepsToMove_i32 > 0)
        {
            digitalWrite(dirPin_u8, dirLevelForward_b);
        }
        else
        {
            digitalWrite(dirPin_u8, dirLevelBackward_b);
        }

        uint32_t speed = maxSpeed_u32;
        if (speed == 0) speed = 1000;

        uint32_t cycleDurationUs = 1000000 / speed;
        uint32_t lowDurationUs = cycleDurationUs - PULSE_WIDTH_US;
        if (cycleDurationUs <= PULSE_WIDTH_US) lowDurationUs = 1;

        // Using RMT, we can just send the exact number of pulses
        // But since stepsToMove_i32 can be very large (up to MAX_ALLOWED_POSITION_CHANGE_PER_CYCLE),
        // we should either loop or configure RMT to repeat. We'll build blocks and send them.
        
        isRunning_b = true;
        
        uint32_t pulsesRemaining = absStepsToMove_i32;
        const uint32_t MAX_BATCH = 500;
        
        while (pulsesRemaining > 0) {
            uint32_t batch = pulsesRemaining > MAX_BATCH ? MAX_BATCH : pulsesRemaining;
            
            rmt_item32_t* items = new rmt_item32_t[batch + 1];
            
            for (uint32_t i = 0; i < batch; i++) {
                items[i].level0 = 1;
                items[i].duration0 = PULSE_WIDTH_US * rmtTicksPerUs;
                items[i].level1 = 0;
                items[i].duration1 = lowDurationUs * rmtTicksPerUs;
            }
            
            items[batch].level0 = 0;
            items[batch].duration0 = 0;
            items[batch].level1 = 0;
            items[batch].duration1 = 0;
            
            // Wait for previous transmission to finish
            rmt_wait_tx_done(rmtChannel, portMAX_DELAY);
            rmt_write_items(rmtChannel, items, batch + 1, false);
            
            delete[] items;
            pulsesRemaining -= batch;
        }

        if (blocking_b)
        {
            rmt_wait_tx_done(rmtChannel, portMAX_DELAY);
            isRunning_b = false;
        }
    }
}


void IRAM_ATTR FastNonAccelStepper::moveTo(int32_t targetPos_i32, bool blocking_b)
{
    int32_t currentPos_i32 = getCurrentPosition();
    int32_t positionChange_i32 = targetPos_i32 - currentPos_i32;
    targetPosition_i32 = targetPos_i32;
    move(positionChange_i32, blocking_b);
} 

int32_t IRAM_ATTR FastNonAccelStepper::getCurrentPosition() const
{
    int16_t pulseCount_i16 = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &pulseCount_i16);
    return ((int32_t)overflowCount_i32 * (int32_t)PCNT_MIN_MAX_THRESHOLD) + (int32_t)pulseCount_i16 - zeroPosition_i32;
}


void FastNonAccelStepper::initRMT()
{
    // Configure RMT TX Channel
    rmt_config_t rmt_tx;
    rmt_tx.rmt_mode = RMT_MODE_TX;
    rmt_tx.channel = rmtChannel;
    rmt_tx.gpio_num = (gpio_num_t)stepPin_u8;
    rmt_tx.mem_block_num = 1;
    rmt_tx.clk_div = 80; // 80MHz APB clock / 80 = 1MHz -> 1 tick = 1us
    rmt_tx.tx_config.loop_en = false;
    
    // Carrier modulation is disabled
    rmt_tx.tx_config.carrier_en = false;
    rmt_tx.tx_config.idle_output_en = true;
    rmt_tx.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    
    rmt_config(&rmt_tx);
    rmt_driver_install(rmtChannel, 0, 0);
}

void FastNonAccelStepper::initPCNTMultiturn()
{
    pcnt_config_t pcntConfig;
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

    // Activate pcnt
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);

    // PCNT event
    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_L_LIM);

    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(PCNT_UNIT_0, multiturnPCNTISR, this);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
}

void FastNonAccelStepper::initPCNTControl()
{
    pcnt_config_t pcntConfig;
    pcntConfig.pulse_gpio_num = stepPin_u8;
    pcntConfig.ctrl_gpio_num = dirPin_u8;
    pcntConfig.channel = PCNT_CHANNEL_0;
    pcntConfig.unit = PCNT_UNIT_1;
    pcntConfig.pos_mode = PCNT_COUNT_INC;
    pcntConfig.neg_mode = PCNT_COUNT_DIS;
    pcntConfig.lctrl_mode = (pcnt_ctrl_mode_t)dirPcntLctrlMode_u8;
    pcntConfig.hctrl_mode = (pcnt_ctrl_mode_t)dirPcntHctrlMode_u8;
    pcntConfig.counter_h_lim = PCNT_MIN_MAX_THRESHOLD;
    pcntConfig.counter_l_lim = -PCNT_MIN_MAX_THRESHOLD;

    pcnt_unit_config(&pcntConfig);
    pcnt_set_filter_value(PCNT_UNIT_1, PCNT_FILTER_VALUE);
    pcnt_filter_enable(PCNT_UNIT_1);

    // Activate pcnt
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_1);

    // PCNT event
    pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_L_LIM);

    pcnt_isr_handler_add(PCNT_UNIT_1, controlPCNTISR, this);
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_1);
}

void IRAM_ATTR FastNonAccelStepper::multiturnPCNTISR(void* arg_p)
{
    FastNonAccelStepper* instance_p = static_cast<FastNonAccelStepper*>(arg_p);
    uint32_t status_u32;
    pcnt_get_event_status(PCNT_UNIT_0, &status_u32);

    if (status_u32 & PCNT_EVT_H_LIM)
    {
        instance_p->overflowCount_i32++;
    }
    if (status_u32 & PCNT_EVT_L_LIM)
    {
        instance_p->overflowCount_i32--;
    }
}

void IRAM_ATTR FastNonAccelStepper::controlPCNTISR(void* arg_p)
{
    // Not strictly needed in RMT, unless we want to use it to track bounds
}

void IRAM_ATTR FastNonAccelStepper::forceStop()
{
    // Now, schedule the timer to stop cleanly at the end of its cycle.
    rmt_tx_stop(rmtChannel);
    // clear buffer
    rmt_tx_memory_reset(rmtChannel);
    isRunning_b = false;
}        

void IRAM_ATTR FastNonAccelStepper::setCurrentPosition(int32_t newPosition_i32)
{
    // set new position
    // newPosition_i32 = (getCurrentPosition + oldZeroPos) - (newZeroPos)
    // newZeroPos = (getCurrentPosition + oldZeroPos) - newPosition_i32
    int32_t newZeroPos_i32 = (getCurrentPosition() + zeroPosition_i32) - newPosition_i32;
    zeroPosition_i32 = newZeroPos_i32;
}

void IRAM_ATTR FastNonAccelStepper::forceStopAndNewPosition(int32_t newPosition_i32)
{
    // stop mcpwm
    forceStop();

    // set new position
    setCurrentPosition(newPosition_i32);
}   

bool IRAM_ATTR FastNonAccelStepper::isRunning()
{
    return isRunning_b;
}

void IRAM_ATTR FastNonAccelStepper::keepRunningInDir(bool forwardDir_b, uint32_t speed_u32)
{
    forceStop();
	
	if (forwardDir_b)
    {
        digitalWrite(dirPin_u8, dirLevelForward_b);
    }
    else
    {
        digitalWrite(dirPin_u8, dirLevelBackward_b);
    }

        // Using continuous transmission for simple keep running 
    // In RMT, we can loop the block continuously until stopped
    uint32_t speed = speed_u32;
    if(speed == 0) return;
    
    uint32_t cycleDurationUs = 1000000 / speed;
    uint32_t lowDurationUs = cycleDurationUs - PULSE_WIDTH_US;
    if (cycleDurationUs <= PULSE_WIDTH_US) lowDurationUs = 1;

    rmt_item32_t item;
    item.level0 = 1;
    item.duration0 = PULSE_WIDTH_US * rmtTicksPerUs;
    item.level1 = 0;
    item.duration1 = lowDurationUs * rmtTicksPerUs;

    // enable tx loop in RMT driver directly
    rmt_set_tx_loop_mode(rmtChannel, true);
    
    isRunning_b = true;
    rmt_write_items(rmtChannel, &item, 1, false);
}

void IRAM_ATTR FastNonAccelStepper::keepRunningForward(uint32_t speed_u32)
{
    keepRunningInDir(true, speed_u32);
}

void IRAM_ATTR FastNonAccelStepper::keepRunningBackward(uint32_t speed_u32)
{
    keepRunningInDir(false, speed_u32);
}

int32_t IRAM_ATTR FastNonAccelStepper::getPositionAfterCommandsCompleted()
{
  return targetPosition_i32;
}
