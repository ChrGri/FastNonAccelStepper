#include "FastNonAccelStepper.h"
#include <rom/gpio.h> 
#include <driver/gpio.h> 
#include "soc/gpio_sig_map.h"
#include "esp_rom_gpio.h" 

#define PCNT_MIN_MAX_THRESHOLD (int16_t)32767
#define PCNT_FILTER_VALUE 1
#define MCPWM_PCNT_MAX_ALLOWED_MOVEMENT_IN_OPPOSITE_DIR_TILL_STOP 50
#define POSITION_TRIGGER_THRESHOLD 1
#define TIMER_RESOLUTION_IN_HZ 10000000

FastNonAccelStepper::FastNonAccelStepper(uint8_t stepPin_u8, uint8_t dirPin_u8, bool invertMotorDir_b)
    : stepPin_u8(stepPin_u8), dirPin_u8(dirPin_u8), invertMotorDirection_b(invertMotorDir_b),
      targetPosition_i32(0), stepsRemaining_i32(0), maxSpeed_u32(1000), 
      currentIntervalUs_u32(1000), directionMultiplier_i8(1),
      isRunning_b(false), runInfinite_b(false), monitorTaskHandle(NULL),
      mcpwm_timer(NULL), mcpwm_oper(NULL), mcpwm_cmpr(NULL), mcpwm_gen(NULL),
      overflowCount_i32(0), overflowCountControl_i32(0), zeroPosition_i32(0),
      expectedCycleTimeUs_u32(300) 
{
}

void FastNonAccelStepper::begin(int timerGroup)
{
    pinMode(stepPin_u8, OUTPUT);
    pinMode(dirPin_u8, OUTPUT);
    digitalWrite(stepPin_u8, LOW);
    digitalWrite(dirPin_u8, LOW);

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

    // Initialize BOTH PCNT Units (Restored your dual-unit architecture)
    initPCNTMultiturn();
    initPCNTControl(); 
    
    // Configure MCPWM (V5 Next-Gen API)
    /*mcpwm_timer_config_t timer_config = {};
    timer_config.group_id = timerGroup;
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = 1000000; 
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    timer_config.period_ticks = 1000; 
    timer_config.flags.update_period_on_empty = true; 
    mcpwm_new_timer(&timer_config, &mcpwm_timer);*/

    mcpwm_timer_config_t timer_config = {};
    timer_config.group_id = timerGroup;
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = TIMER_RESOLUTION_IN_HZ; // Erhöht auf 10 MHz 
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    timer_config.period_ticks = 10000;      // Startwert angepasst 
    timer_config.flags.update_period_on_empty = true; 
    mcpwm_new_timer(&timer_config, &mcpwm_timer);


    mcpwm_operator_config_t oper_config = {};
    oper_config.group_id = timerGroup;
    mcpwm_new_operator(&oper_config, &mcpwm_oper);
    mcpwm_operator_connect_timer(mcpwm_oper, mcpwm_timer);

    mcpwm_comparator_config_t cmpr_config = {};
    cmpr_config.flags.update_cmp_on_tez = true; 
    mcpwm_new_comparator(mcpwm_oper, &cmpr_config, &mcpwm_cmpr);

    mcpwm_generator_config_t gen_config = {};
    gen_config.gen_gpio_num = stepPin_u8;
    gen_config.flags.io_loop_back = true; 
    mcpwm_new_generator(mcpwm_oper, &gen_config, &mcpwm_gen);

    mcpwm_comparator_set_compare_value(mcpwm_cmpr, 500); 

    mcpwm_generator_set_action_on_timer_event(mcpwm_gen, 
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    
    mcpwm_generator_set_action_on_compare_event(mcpwm_gen, 
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, mcpwm_cmpr, MCPWM_GEN_ACTION_LOW));

    mcpwm_timer_enable(mcpwm_timer);

    // SIGNAL ROUTING FOR PCNT
    gpio_set_direction((gpio_num_t)dirPin_u8, GPIO_MODE_INPUT_OUTPUT); 
    
    // Connect STEP and DIR to PCNT_UNIT_0 (Position Tracking)
    esp_rom_gpio_connect_in_signal(stepPin_u8, PCNT_SIG_CH0_IN0_IDX, false);
    esp_rom_gpio_connect_in_signal(dirPin_u8, PCNT_SIG_CH0_IN1_IDX, false);

    // Connect STEP and DIR to PCNT_UNIT_1 (Control Interrupts)
    esp_rom_gpio_connect_in_signal(stepPin_u8, PCNT_SIG_CH1_IN0_IDX, false);
    esp_rom_gpio_connect_in_signal(dirPin_u8, PCNT_SIG_CH1_IN1_IDX, false);

    // Start the Hardware Stop Task
    xTaskCreatePinnedToCore(
        monitorTaskWrapper,
        "MCPWM_Monitor_Task",
        4096,                    
        this,                    
        configMAX_PRIORITIES - 1,
        &monitorTaskHandle,
        1                        
    );
}

// ------------------------------------------------------------------------
// PCNT HARDWARE POSITION TRACKING (UNIT 0)
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

// ------------------------------------------------------------------------
// PCNT HARDWARE BURST CONTROL (UNIT 1) - RESTORED
// ------------------------------------------------------------------------
void FastNonAccelStepper::initPCNTControl()
{
    pcnt_config_t pcntConfig = {};
    pcntConfig.pulse_gpio_num = stepPin_u8;
    pcntConfig.ctrl_gpio_num = dirPin_u8;
    pcntConfig.channel = PCNT_CHANNEL_0; // Channel 0 of Unit 1
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

    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_1);

    pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_L_LIM);

    pcnt_isr_handler_add(PCNT_UNIT_1, controlPCNTISR, this);
    
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_1);
}

void IRAM_ATTR FastNonAccelStepper::controlPCNTISR(void* arg_p)
{
    FastNonAccelStepper* instance_p = static_cast<FastNonAccelStepper*>(arg_p);
    uint32_t status_u32;
    pcnt_get_event_status(PCNT_UNIT_1, &status_u32);

    if (status_u32 & PCNT_EVT_H_LIM || status_u32 & PCNT_EVT_L_LIM)
    {
        if (instance_p->overflowCountControl_i32 < 1)
        {
            // NEW IN V5: We cannot call forceStop() directly from the ISR anymore.
            // Instead, we fire a high-priority hardware signal directly to the monitor task.
            BaseType_t high_task_wakeup = pdFALSE;
            vTaskNotifyGiveFromISR(instance_p->monitorTaskHandle, &high_task_wakeup);
            
            if (high_task_wakeup == pdTRUE) {
                portYIELD_FROM_ISR(); // Instantly snap the CPU to the stop task!
            }
        }
        else
        {
            instance_p->overflowCountControl_i32--;
        }
    }
}

// ------------------------------------------------------------------------
// HARDWARE STOP TASK
// ------------------------------------------------------------------------
void FastNonAccelStepper::monitorTaskWrapper(void *pvParameters)
{
    FastNonAccelStepper* instance = static_cast<FastNonAccelStepper*>(pvParameters);
    instance->monitorTask();
}

void FastNonAccelStepper::monitorTask()
{
    while (true) 
    {
        // NO MORE POLLING! 
        // This task sleeps with 0% CPU usage until the exact microsecond 
        // PCNT_UNIT_1 fires the hardware interrupt above.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        if (isRunning_b && !runInfinite_b) {
            forceStop();
        }
    }
}

// ------------------------------------------------------------------------
// Standard Methods
// ------------------------------------------------------------------------
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

void IRAM_ATTR FastNonAccelStepper::setExpectedCycleTimeUs(uint32_t cycleTimeUs_u32) {
    expectedCycleTimeUs_u32 = cycleTimeUs_u32;
}

void IRAM_ATTR FastNonAccelStepper::setMaxSpeed(uint32_t speed_u32) {
    setSpeedLive(speed_u32);
}

void IRAM_ATTR FastNonAccelStepper::setSpeedLive(uint32_t speed_u32) {
    // Geschwindigkeit begrenzen (Hardware-Limit des ESP32 MCPWM)
    maxSpeed_u32 = constrain(speed_u32, 20, 250000);
    
    // Berechnung basierend auf 10MHz Resolution
    // Ein Intervall von 40 Ticks entspricht nun 250kHz (10.000.000 / 250.000)
    uint32_t newInterval = TIMER_RESOLUTION_IN_HZ / maxSpeed_u32;
    
    if (newInterval != currentIntervalUs_u32) {
        currentIntervalUs_u32 = newInterval;
        
        if (mcpwm_timer != NULL) {
            // Die Hardware setzt das neue Intervall beim nächsten Zyklus-Ende um
            mcpwm_timer_set_period(mcpwm_timer, currentIntervalUs_u32);
            
            // Duty Cycle auf 50% halten (Pulsbreite)
            mcpwm_comparator_set_compare_value(mcpwm_cmpr, currentIntervalUs_u32 / 2);
        }
    }
}

uint32_t IRAM_ATTR FastNonAccelStepper::getMaxSpeed(void) {
    return maxSpeed_u32;
}

void IRAM_ATTR FastNonAccelStepper::move(int32_t stepsToMove_i32, bool blocking_b) {
    forceStop(); 

    int32_t absStepsToMove_i32 = abs(stepsToMove_i32);

    if (absStepsToMove_i32 > POSITION_TRIGGER_THRESHOLD)
    {
        // Restored your exact wrapper math for the PCNT Unit 1 hardware!
        int32_t numbWraps_i32 = absStepsToMove_i32 / PCNT_MIN_MAX_THRESHOLD;
        int32_t limit_i32 = absStepsToMove_i32 / (numbWraps_i32 + 1);
        int16_t limit_i16 = constrain(limit_i32, 0, PCNT_MIN_MAX_THRESHOLD);

        int16_t highLimit_i16;
        int16_t lowLimit_i16;

        if (stepsToMove_i32 > 0)
        {
            digitalWrite(dirPin_u8, dirLevelForward_b);
            directionMultiplier_i8 = 1;
            highLimit_i16 = limit_i16;
            lowLimit_i16 = -MCPWM_PCNT_MAX_ALLOWED_MOVEMENT_IN_OPPOSITE_DIR_TILL_STOP;
            overflowCountControl_i32 = numbWraps_i32;
        }
        else
        {
            digitalWrite(dirPin_u8, dirLevelBackward_b);
            directionMultiplier_i8 = -1;
            highLimit_i16 = MCPWM_PCNT_MAX_ALLOWED_MOVEMENT_IN_OPPOSITE_DIR_TILL_STOP;
            lowLimit_i16 = -limit_i16;
            overflowCountControl_i32 = numbWraps_i32;
        }

        // Program PCNT_UNIT_1 limits
        pcnt_counter_pause(PCNT_UNIT_1);
        pcnt_counter_clear(PCNT_UNIT_1);
        pcnt_set_event_value(PCNT_UNIT_1, PCNT_EVT_H_LIM, highLimit_i16);
        pcnt_set_event_value(PCNT_UNIT_1, PCNT_EVT_L_LIM, lowLimit_i16);
        pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_H_LIM);
        pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_L_LIM);
        pcnt_counter_clear(PCNT_UNIT_1);
        pcnt_counter_resume(PCNT_UNIT_1);

        stepsRemaining_i32 = absStepsToMove_i32;
        targetPosition_i32 = getCurrentPosition() + stepsToMove_i32;
        runInfinite_b = false;
        isRunning_b = true; 

        mcpwm_timer_start_stop(mcpwm_timer, MCPWM_TIMER_START_NO_STOP);

        if (blocking_b) {
            while(isRunning_b) delay(1);
        }
    }
}

void IRAM_ATTR FastNonAccelStepper::moveTo(int32_t targetPos_i32, bool blocking_b) {
    move(targetPos_i32 - getCurrentPosition(), blocking_b);
} 

void IRAM_ATTR FastNonAccelStepper::forceStop() {
    isRunning_b = false;
    stepsRemaining_i32 = 0;
    runInfinite_b = false;
    
    if (mcpwm_timer != NULL) {
        mcpwm_timer_start_stop(mcpwm_timer, MCPWM_TIMER_STOP_EMPTY);
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
    forceStop();
    
    if (forwardDir_b) {
        digitalWrite(dirPin_u8, dirLevelForward_b);
        directionMultiplier_i8 = 1;
    } else {
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
    runInfinite_b = true;
    isRunning_b = true; 
    
    mcpwm_timer_start_stop(mcpwm_timer, MCPWM_TIMER_START_NO_STOP);
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