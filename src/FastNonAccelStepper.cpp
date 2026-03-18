#include "FastNonAccelStepper.h"
#include <rom/gpio.h> 
#include <driver/gpio.h> 
#include "soc/gpio_sig_map.h"
#include "esp_rom_gpio.h" 

/************************************************************************/
/* Defines                               */
/************************************************************************/
#define MAX_ALLOWED_POSITION_CHANGE_PER_CYCLE_I32 (int32_t)20000
#define PWM_DUTY_CYCLE_FL32 50.0f
#define PCNT_MIN_MAX_THRESHOLD_I16 (int16_t)32767 // INT16_MAX = (2^15)-1 = 32767
#define POSITION_TRIGGER_THRESHOLD_I32 1
#define PCNT_FILTER_VALUE_I32 1
#define MCPWM_PCNT_MAX_ALLOWED_MOVEMENT_IN_OPPOSITE_DIR_TILL_STOP_I32 50
#define TIMER_RESOLUTION_IN_HZ_U32 10000000
#define MINIMUM_PULSE_FREQUENCY_U32 (uint32_t)20

/************************************************************************/
/* Implementation                            */
/************************************************************************/

FastNonAccelStepper::FastNonAccelStepper(uint8_t stepPin_u8, uint8_t dirPin_u8, bool invertMotorDir_b)
    : stepPin_u8(stepPin_u8), dirPin_u8(dirPin_u8), invertMotorDirection_b(invertMotorDir_b),
      targetPosition_i32(0), stepsRemaining_i32(0), maxSpeed_u32(1000), 
      currentIntervalUs_u32(1000), directionMultiplier_i8(1),
      isRunning_b(false), runInfinite_b(false), monitorTaskHandle_pv(NULL),
      mcpwmTimer_pst(NULL), mcpwmOper_pst(NULL), mcpwmCmpr_pst(NULL), mcpwmGen_pst(NULL),
      overflowCount_i32(0), overflowCountControl_i32(0), zeroPosition_i32(0),
      expectedCycleTimeUs_u32(300) 
{
}

void FastNonAccelStepper::begin(int timerGroup_i32)
{
    // set dir logic for counting
    // 1) If invertMotorDir_b == false, the DIR_PIN == HIGH, when motor moving forward and DIR_PIN == LOW, when motor moving backwards
    // 2) If invertMotorDir_b == false, the PCNT counter should go up, when DIR_PIN == HIGH
    if (false == invertMotorDirection_b) 
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

    // init MCPWM and PCNTs
    initPCNTMultiturn(); // unit to track the overall position
    initPCNTControl();   // unit to control the step bursts

    pinMode(stepPin_u8, OUTPUT);
    pinMode(dirPin_u8, OUTPUT);
    digitalWrite(stepPin_u8, LOW);
    digitalWrite(dirPin_u8, LOW);

    // Configure MCPWM (V5 Next-Gen API)
    mcpwm_timer_config_t timerConfig_st = {};
    timerConfig_st.group_id = timerGroup_i32;
    timerConfig_st.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timerConfig_st.resolution_hz = TIMER_RESOLUTION_IN_HZ_U32; // Increased to 10 MHz
    timerConfig_st.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    timerConfig_st.period_ticks = 10000;      // Adjusted start value
    timerConfig_st.flags.update_period_on_empty = true; 
    mcpwm_new_timer(&timerConfig_st, &mcpwmTimer_pst);

    mcpwm_operator_config_t operConfig_st = {};
    operConfig_st.group_id = timerGroup_i32;
    mcpwm_new_operator(&operConfig_st, &mcpwmOper_pst);
    mcpwm_operator_connect_timer(mcpwmOper_pst, mcpwmTimer_pst);

    mcpwm_comparator_config_t cmprConfig_st = {};
    cmprConfig_st.flags.update_cmp_on_tez = true; 
    mcpwm_new_comparator(mcpwmOper_pst, &cmprConfig_st, &mcpwmCmpr_pst);

    mcpwm_generator_config_t genConfig_st = {};
    genConfig_st.gen_gpio_num = stepPin_u8;
    genConfig_st.flags.io_loop_back = true; 
    mcpwm_new_generator(mcpwmOper_pst, &genConfig_st, &mcpwmGen_pst);

    mcpwm_comparator_set_compare_value(mcpwmCmpr_pst, 500); 

    mcpwm_generator_set_action_on_timer_event(mcpwmGen_pst, 
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    
    mcpwm_generator_set_action_on_compare_event(mcpwmGen_pst, 
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, mcpwmCmpr_pst, MCPWM_GEN_ACTION_LOW));

    mcpwm_timer_enable(mcpwmTimer_pst);

    // SIGNAL ROUTING FOR PCNT
    gpio_set_direction((gpio_num_t)dirPin_u8, GPIO_MODE_INPUT_OUTPUT); 
    
    // Connect STEP and DIR to PCNT_UNIT_0 (Position Tracking)
    esp_rom_gpio_connect_in_signal(stepPin_u8, PCNT_SIG_CH0_IN0_IDX, false); // overall position unit
    esp_rom_gpio_connect_in_signal(dirPin_u8, PCNT_SIG_CH0_IN1_IDX, false);

    // Connect STEP and DIR to PCNT_UNIT_1 (Control Interrupts)
    esp_rom_gpio_connect_in_signal(stepPin_u8, PCNT_SIG_CH1_IN0_IDX, false); // control unit
    esp_rom_gpio_connect_in_signal(dirPin_u8, PCNT_SIG_CH1_IN1_IDX, false);

    // reset pcnt counters
    pcnt_counter_clear(PCNT_UNIT_0); // overall position unit
    pcnt_counter_clear(PCNT_UNIT_1); // control unit

    // Start the Hardware Stop Task
    xTaskCreatePinnedToCore(
        monitorTaskWrapper,
        "MCPWM_Monitor_Task",
        4096,                    
        this,                    
        configMAX_PRIORITIES - 1,
        &monitorTaskHandle_pv,
        1                        
    );

    // make sure mcpwm is stopped
    forceStop();
}

void FastNonAccelStepper::initPCNTMultiturn()
{
    pcnt_config_t pcntConfig_st = {}; 
    pcntConfig_st.pulse_gpio_num = stepPin_u8;
    pcntConfig_st.ctrl_gpio_num = dirPin_u8;
    pcntConfig_st.channel = PCNT_CHANNEL_0;
    pcntConfig_st.unit = PCNT_UNIT_0;
    pcntConfig_st.pos_mode = PCNT_COUNT_INC; 
    pcntConfig_st.neg_mode = PCNT_COUNT_DIS; 
    pcntConfig_st.lctrl_mode = (pcnt_ctrl_mode_t)dirPcntLctrlMode_u8;
    pcntConfig_st.hctrl_mode = (pcnt_ctrl_mode_t)dirPcntHctrlMode_u8;
    pcntConfig_st.counter_h_lim = PCNT_MIN_MAX_THRESHOLD_I16;
    pcntConfig_st.counter_l_lim = -PCNT_MIN_MAX_THRESHOLD_I16;

    pcnt_unit_config(&pcntConfig_st);
    pcnt_set_filter_value(PCNT_UNIT_0, PCNT_FILTER_VALUE_I32);
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
    pcnt_config_t pcntConfig_st = {};
    pcntConfig_st.pulse_gpio_num = stepPin_u8;
    pcntConfig_st.ctrl_gpio_num = dirPin_u8;
    pcntConfig_st.channel = PCNT_CHANNEL_0; 
    pcntConfig_st.unit = PCNT_UNIT_1;
    pcntConfig_st.pos_mode = PCNT_COUNT_INC;
    pcntConfig_st.neg_mode = PCNT_COUNT_DIS;
    pcntConfig_st.lctrl_mode = (pcnt_ctrl_mode_t)dirPcntLctrlMode_u8;
    pcntConfig_st.hctrl_mode = (pcnt_ctrl_mode_t)dirPcntHctrlMode_u8;
    pcntConfig_st.counter_h_lim = PCNT_MIN_MAX_THRESHOLD_I16;
    pcntConfig_st.counter_l_lim = -PCNT_MIN_MAX_THRESHOLD_I16;

    pcnt_unit_config(&pcntConfig_st);
    pcnt_set_filter_value(PCNT_UNIT_1, PCNT_FILTER_VALUE_I32);
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

void IRAM_ATTR FastNonAccelStepper::multiturnPCNTISR(void* arg_pv)
{
    FastNonAccelStepper* instance_pst = static_cast<FastNonAccelStepper*>(arg_pv);
    uint32_t status_u32;
    pcnt_get_event_status(PCNT_UNIT_0, &status_u32);

    if (status_u32 & PCNT_EVT_H_LIM) 
    {
        instance_pst->overflowCount_i32++;
    }
    if (status_u32 & PCNT_EVT_L_LIM) 
    {
        instance_pst->overflowCount_i32--;
    }
}

void IRAM_ATTR FastNonAccelStepper::controlPCNTISR(void* arg_pv)
{
    FastNonAccelStepper* instance_pst = static_cast<FastNonAccelStepper*>(arg_pv);
    uint32_t status_u32;
    pcnt_get_event_status(PCNT_UNIT_1, &status_u32);

    if (status_u32 & PCNT_EVT_H_LIM || status_u32 & PCNT_EVT_L_LIM)
    {
        if (instance_pst->overflowCountControl_i32 < 1)
        {
            // Signal stop task
            BaseType_t highTaskWakeup_st = pdFALSE;
            vTaskNotifyGiveFromISR(instance_pst->monitorTaskHandle_pv, &highTaskWakeup_st);
            
            if (highTaskWakeup_st == pdTRUE) 
            {
                portYIELD_FROM_ISR(); 
            }
        }
        else
        {
            instance_pst->overflowCountControl_i32--;
        }
    }
}

void FastNonAccelStepper::monitorTaskWrapper(void* pvParameters_pv)
{
    FastNonAccelStepper* instance_pst = static_cast<FastNonAccelStepper*>(pvParameters_pv);
    instance_pst->monitorTask();
}

void FastNonAccelStepper::monitorTask()
{
    while (true) 
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        if (isRunning_b && !runInfinite_b) 
        {
            forceStop();
        }
    }
}

int32_t IRAM_ATTR FastNonAccelStepper::getCurrentPosition() const
{
    int16_t pulseCount_i16 = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &pulseCount_i16);
    return ((int32_t)overflowCount_i32 * (int32_t)PCNT_MIN_MAX_THRESHOLD_I16) + (int32_t)pulseCount_i16 - zeroPosition_i32;
}

void IRAM_ATTR FastNonAccelStepper::setCurrentPosition(int32_t newPosition_i32)
{
    // set new position
    // newPosition_i32 = (getCurrentPosition + oldZeroPos) - (newZeroPos)
    // newZeroPos = (getCurrentPosition + oldZeroPos) - newPosition_i32
    int32_t newZeroPos_i32 = (getCurrentPosition() + zeroPosition_i32) - newPosition_i32;
    zeroPosition_i32 = newZeroPos_i32;
}

void IRAM_ATTR FastNonAccelStepper::setExpectedCycleTimeUs(uint32_t cycleTimeUs_u32) 
{
    expectedCycleTimeUs_u32 = cycleTimeUs_u32;
}

void IRAM_ATTR FastNonAccelStepper::setMaxSpeed(uint32_t speed_u32) 
{
    // Constrain the speed to valid limits
    setSpeedLive(speed_u32);
}

void IRAM_ATTR FastNonAccelStepper::setSpeedLive(uint32_t speed_u32) 
{
    // 1. Check if speed is below the minimum threshold
    if (speed_u32 < MINIMUM_PULSE_FREQUENCY_U32)
    {
        // Disable output by stopping the timer
        if (mcpwmTimer_pst != NULL) 
        {
            mcpwm_timer_start_stop(mcpwmTimer_pst, MCPWM_TIMER_STOP_EMPTY);
        }
        isRunning_b = false;
        return;
    }

    // 2. Limit speed to the hardware maximum
    maxSpeed_u32 = constrain(speed_u32, MINIMUM_PULSE_FREQUENCY_U32, MAX_SPEED_IN_HZ);
    
    // 3. Calculation based on 10MHz Resolution
    uint32_t newInterval_u32 = TIMER_RESOLUTION_IN_HZ_U32 / maxSpeed_u32;
    
    if (newInterval_u32 != currentIntervalUs_u32) 
    {
        currentIntervalUs_u32 = newInterval_u32;
        
        if (mcpwmTimer_pst != NULL) 
        {
            // Update period and comparator
            mcpwm_timer_set_period(mcpwmTimer_pst, currentIntervalUs_u32);
            mcpwm_comparator_set_compare_value(mcpwmCmpr_pst, currentIntervalUs_u32 / 2);
            
            // Ensure timer is running if we were previously below min frequency
            if (!isRunning_b)
            {
                mcpwm_timer_start_stop(mcpwmTimer_pst, MCPWM_TIMER_START_NO_STOP);
                isRunning_b = true;
            }
        }
    }
}

uint32_t IRAM_ATTR FastNonAccelStepper::getMaxSpeed(void) 
{
    return maxSpeed_u32;
}

void IRAM_ATTR FastNonAccelStepper::move(int32_t stepsToMove_i32, bool blocking_b) 
{
    // stop previous move
    forceStop(); 

    int32_t absStepsToMove_i32 = abs(stepsToMove_i32);

    if (absStepsToMove_i32 > POSITION_TRIGGER_THRESHOLD_I32)
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
        int32_t numbWraps_i32 = absStepsToMove_i32 / PCNT_MIN_MAX_THRESHOLD_I16;

        // divide into equally spaced bursts, e.g. when
        // if absStepsToMove is < PCNT_MIN_MAX_THRESHOLD --> numbWraps = 0 and limit_i16 = absStepsToMove
        // if absStepsToMove == PCNT_MIN_MAX_THRESHOLD --> numbWraps = 1 and limit_i16 = absStepsToMove/2
        // if absStepsToMove == 2*PCNT_MIN_MAX_THRESHOLD --> numbWraps = 2 and limit_i16 = absStepsToMove/3
        // if absStepsToMove > PCNT_MIN_MAX_THRESHOLD --> numbWraps >= 1
        int32_t limit_i32 = absStepsToMove_i32 / (numbWraps_i32 + 1);
        int16_t limit_i16 = (int16_t)constrain(limit_i32, 0, PCNT_MIN_MAX_THRESHOLD_I16);

        // the highLimit | lowLimit will be hit numbWraps, before stopping the PWM output
        int16_t highLimit_i16;
        int16_t lowLimit_i16;

        // 1) set DIR pin
        // 2) define upper limit for control pcnt
        // 3) define lower limit for control pcnt
        if (stepsToMove_i32 > 0)
        {
            digitalWrite(dirPin_u8, dirLevelForward_b);
            directionMultiplier_i8 = 1;
            highLimit_i16 = limit_i16;
            lowLimit_i16 = (int16_t)(-MCPWM_PCNT_MAX_ALLOWED_MOVEMENT_IN_OPPOSITE_DIR_TILL_STOP_I32);
            overflowCountControl_i32 = numbWraps_i32;
        }
        else
        {
            digitalWrite(dirPin_u8, dirLevelBackward_b);
            directionMultiplier_i8 = -1;
            highLimit_i16 = (int16_t)MCPWM_PCNT_MAX_ALLOWED_MOVEMENT_IN_OPPOSITE_DIR_TILL_STOP_I32;
            lowLimit_i16 = (int16_t)(-limit_i16);
            overflowCountControl_i32 = numbWraps_i32;
        }

        // parameterize control pcnt
        pcnt_counter_pause(PCNT_UNIT_1);
        pcnt_counter_clear(PCNT_UNIT_1);
        pcnt_set_event_value(PCNT_UNIT_1, PCNT_EVT_H_LIM, highLimit_i16);
        pcnt_set_event_value(PCNT_UNIT_1, PCNT_EVT_L_LIM, lowLimit_i16);
        pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_H_LIM);
        pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_L_LIM);
        pcnt_counter_clear(PCNT_UNIT_1);
        pcnt_counter_resume(PCNT_UNIT_1);

        // start mcpwm
        delayMicroseconds(5);
        stepsRemaining_i32 = absStepsToMove_i32;
        targetPosition_i32 = getCurrentPosition() + stepsToMove_i32;
        runInfinite_b = false;
        isRunning_b = true; 

        mcpwm_timer_start_stop(mcpwmTimer_pst, MCPWM_TIMER_START_NO_STOP);

        if (blocking_b) 
        {
            while(isRunning()) 
            {
                delay(1);
            }
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

void IRAM_ATTR FastNonAccelStepper::forceStop() 
{
    // Immediately force the step pin to a known inactive state (LOW).
    // Now, schedule the timer to stop cleanly at the end of its cycle.
    isRunning_b = false;
    stepsRemaining_i32 = 0;
    runInfinite_b = false;
    
    if (mcpwmTimer_pst != NULL) 
    {
        mcpwm_timer_start_stop(mcpwmTimer_pst, MCPWM_TIMER_STOP_EMPTY);
    }
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
        directionMultiplier_i8 = 1;
    } 
    else 
    {
        digitalWrite(dirPin_u8, dirLevelBackward_b);
        directionMultiplier_i8 = -1;
    }

    // Since this is infinite, we just clear out the control PCNT
    pcnt_counter_pause(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_event_disable(PCNT_UNIT_1, PCNT_EVT_H_LIM);
    pcnt_event_disable(PCNT_UNIT_1, PCNT_EVT_L_LIM);
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_1);

    setSpeedLive(speed_u32); 
    
    delayMicroseconds(5);
    runInfinite_b = true;
    isRunning_b = true; 
    
    mcpwm_timer_start_stop(mcpwmTimer_pst, MCPWM_TIMER_START_NO_STOP);
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