#include "FastNonAccelStepper.h"
#include <driver/mcpwm.h>
#include <driver/pcnt.h>
#include <driver/gpio.h>

#include "soc/mcpwm_struct.h"
#include "soc/mcpwm_reg.h"
#include "hal/misc.h" // HAL_FORCE_MODIFY_U32_REG_FIELD

// Direct PCNT register access for the per-cycle hot path (moveToWithSpeed, getCurrentPosition).
// The legacy driver functions do the same register writes, but each call takes a spinlock and
// checks its arguments; the pedal control loop calls this path 4000 times per second.
#include "hal/pcnt_ll.h"
#include "soc/pcnt_struct.h"
static portMUX_TYPE s_pcntRegisterMux = portMUX_INITIALIZER_UNLOCKED;


/************************************************************************/
/*								Defines 								                              */
/************************************************************************/
#define MAX_ALLOWED_POSITION_CHANGE_PER_CYCLE (int32_t)20000
#define PWM_DUTY_CYCLE 50.0f
#define PCNT_MIN_MAX_THRESHOLD (int16_t)32767 // INT16_MAX = (2^15)-1 = 32767
#define POSITION_TRIGGER_THRESHOLD 1
#define MCPWM_PCNT_MAX_ALLOWED_MOVEMENT_IN_OPPOSITE_DIR_TILL_STOP 50

// Fixed MCPWM timer clock: 160 MHz / (15 + 1) = 10 MHz. With the 16 bit period register this
// covers 153 Hz ... 250 kHz without switching the clock (switching used to glitch the pulse
// train around 2.5 kHz). Resolution: 0.5 % at 50 kHz, 2.5 % at 250 kHz.
#define TIMER_RESOLUTION_IN_HZ_U32 10000000
#define MCPWM_GROUP_PRESCALER_U8 15
#define MINIMUM_PULSE_FREQUENCY_U32 (uint32_t)(TIMER_RESOLUTION_IN_HZ_U32 / UINT16_MAX + 1u)

// Margin (timer ticks) between reading the timer counter and a forced register update
#define FORCED_UPDATE_GUARD_TICKS 10

// Direction reversal: minimum high time of a step pulse before it may be cut (> PCNT glitch
// filter of ~2 us), and DIR setup time before the next step edge
#define MIN_STEP_PULSE_HIGH_TIME_US 3
#define DIR_SETUP_TIME_US 5


// Defines the margin (in Hz) that the filter should be set above the maximum speed
#define PCNT_FILTER_MARGIN_HZ 5000

// Dynamic calculation of the filter value (80MHz APB clock / 2 for 50% duty cycle)
// The integer division automatically rounds down, which slightly increases the margin on the hardware side.
#define PCNT_FILTER_VALUE (40000000 / (MAX_SPEED_IN_HZ + PCNT_FILTER_MARGIN_HZ))


/************************************************************************/
/*								Implementation							                          */
/************************************************************************/
FastNonAccelStepper::FastNonAccelStepper(uint8_t stepPin_u8, uint8_t dirPin_u8, bool invertMotorDir_b)
  : stepPin_u8(stepPin_u8), dirPin_u8(dirPin_u8), targetPosition_i32(0), maxSpeed_u32(MAX_SPEED_IN_HZ), overflowCount_i32(0), pcntQueue(nullptr), invertMotorDirection_b(invertMotorDir_b), zeroPosition_i32(0)
{
    //stepper_p = this;  // Assign the current instance to stepper_p
    //stepper_p->begin(stepPin_u8, dirPin_u8, invertMotorDirection_b);
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
    initMCPWM();
    initPCNTMultiturn(); // unit to track the overall position
    initPCNTControl(); // unit to controll the step bursts

    pinMode(stepPin_u8, OUTPUT);
    pinMode(dirPin_u8, OUTPUT);
    digitalWrite(stepPin_u8, LOW);
    digitalWrite(dirPin_u8, LOW);

    // Keep internal state variable in sync with hardware pin
    currentDirState_b = LOW;

    // configure pin modes
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, stepPin_u8);

    // connect PCNT to GPIO pin
    gpio_iomux_in(stepPin_u8, PCNT_SIG_CH0_IN0_IDX); // overall position unit
    gpio_iomux_in(stepPin_u8, PCNT_SIG_CH0_IN1_IDX); // controll unit

    // reset pcnt counters
    pcnt_counter_clear(PCNT_UNIT_0); // overall position unit
    pcnt_counter_clear(PCNT_UNIT_1); // controll unit

    // make sure mcpwm is stopped
    forceStop();
}

void IRAM_ATTR FastNonAccelStepper::setMaxSpeed(uint32_t speed_u32)
{
    // Constrain the speed to valid limits
    maxSpeed_u32 = constrain(speed_u32, 1, MAX_SPEED_IN_HZ);

    // update the speed live if the motor is currently running
    setSpeedLive(speed_u32);
}

int32_t IRAM_ATTR FastNonAccelStepper::getMaxSpeed(void)
{
    // 1. Read direction pin to determine the current direction of movement, which indicates whether the speed is positive (forward) or negative (backward)
    //bool currentDir = digitalRead(dirPin_u8);
    // Using the tracked internal state variable instead
    bool currentDir = currentDirState_b;
    
    // 2. Check if this state corresponds to your defined forward direction
    if (currentDir == dirLevelForward_b)
    {
        return (int32_t)maxSpeed_u32;
    }
    else
    {
        return -(int32_t)maxSpeed_u32;
    }
}


void IRAM_ATTR FastNonAccelStepper::move(int32_t stepsToMove_i32, bool blocking_b)
{
    // stop previous move
    //forceStop();

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
        // 2) define upper limit for control pcnt
        // 3) define lower limit for control pcnt
        if (stepsToMove_i32 > 0)
        {
            digitalWrite(dirPin_u8, dirLevelForward_b);
            currentDirState_b = dirLevelForward_b; // Keep internal state in sync

            highLimit_i16 = limit_i16;
            lowLimit_i16 = -MCPWM_PCNT_MAX_ALLOWED_MOVEMENT_IN_OPPOSITE_DIR_TILL_STOP;
            overflowCountControl_i32 = numbWraps_i32;
        }
        else
        {
            digitalWrite(dirPin_u8, dirLevelBackward_b);
            currentDirState_b = dirLevelBackward_b; // Keep internal state in sync

            highLimit_i16 = MCPWM_PCNT_MAX_ALLOWED_MOVEMENT_IN_OPPOSITE_DIR_TILL_STOP;
            lowLimit_i16 = -limit_i16;
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

      //Serial.printf("Hlim: %d,    LLim: %d,    wraps: %d\n", highLimit, lowLimit, numbWraps);

      // start mcpwm
      delayMicroseconds(5);
      isRunning_b = true;
      mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);

        if (blocking_b)
        {
            uint32_t start = millis();
            while(isRunning_b) { 
                delay(1);

                if(millis() - start > 15000) { forceStop(); break; }

                static uint32_t lastPrint = 0;
                if (millis() - lastPrint >= 500) {
                    lastPrint = millis();
                    
                    // Hardware-Register direkt auslesen
                    int16_t rawUnit0 = 0;
                    int16_t rawUnit1 = 0;
                    pcnt_get_counter_value(PCNT_UNIT_0, &rawUnit0);
                    pcnt_get_counter_value(PCNT_UNIT_1, &rawUnit1);

                    Serial.printf("POS: %d | TRGT: %d | U0_raw: %d (Ovr: %d) | U1_raw: %d (Wraps: %d)\n", 
                                  getCurrentPosition(), 
                                  targetPosition_i32, 
                                  rawUnit0, 
                                  overflowCount_i32, 
                                  rawUnit1, 
                                  overflowCountControl_i32);
                }
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

int32_t IRAM_ATTR FastNonAccelStepper::getCurrentPosition() const
{
    // direct register read (same value as pcnt_get_counter_value, without the driver overhead)
    int16_t pulseCount_i16 = (int16_t)pcnt_ll_get_count(&PCNT, PCNT_UNIT_0);
    return ((int32_t)overflowCount_i32 * (int32_t)PCNT_MIN_MAX_THRESHOLD) + (int32_t)pulseCount_i16 - zeroPosition_i32;
}


void FastNonAccelStepper::initMCPWM()
{
    // Wir setzen den Takt explizit auf 10MHz (160MHz / (15+1))
    //mcpwm_group_set_resolution(MCPWM_UNIT_0, TIMER_RESOLUTION_IN_HZ_U32); 
    
    mcpwm_config_t pwmConfig;
    pwmConfig.frequency = maxSpeed_u32;
    pwmConfig.cmpr_a = PWM_DUTY_CYCLE;
    pwmConfig.counter_mode = MCPWM_UP_COUNTER;
    pwmConfig.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwmConfig);

    // Fixed group clock (see TIMER_RESOLUTION_IN_HZ_U32) and no additional timer prescaler
    MCPWM0.clk_cfg.clk_prescale = MCPWM_GROUP_PRESCALER_U8;
    MCPWM0.timer[0].timer_cfg0.timer_prescale = 0;

    // Load period and compare at the end of a period (TEZ) by default, see setSpeedLive()
    MCPWM0.timer[0].timer_cfg0.timer_period_upmethod = 1;
    MCPWM0.operators[0].gen_stmp_cfg.gen_a_upmethod = 1;

    // Continuous software force (low-speed output disable) takes effect immediately. Its reset
    // default is "disable update", which only worked while every speed change forced a global
    // register update.
    MCPWM0.operators[0].gen_force.gen_cntuforce_upmethod = 0;

    // Software sync restarts the period at a direction change (restartPeriodWhilePaused()).
    // No hardware sync source is selected, so only the software sync reloads the counter.
    HAL_FORCE_MODIFY_U32_REG_FIELD(MCPWM0.timer[0].timer_sync, timer_phase, 0);
    MCPWM0.timer[0].timer_sync.timer_phase_direction = 0;
    MCPWM0.timer[0].timer_sync.timer_synci_en = 1;
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
    FastNonAccelStepper* instance_p = static_cast<FastNonAccelStepper*>(arg_p);
    uint32_t status_u32;
    pcnt_get_event_status(PCNT_UNIT_1, &status_u32);

    if (status_u32 & PCNT_EVT_H_LIM || status_u32 & PCNT_EVT_L_LIM)
    {
        if (instance_p->overflowCountControl_i32 < 1)
        {
            instance_p->forceStop();
			instance_p->maxSpeed_u32 = 0; // set speed to 0 when target position has been reached
        }
        instance_p->overflowCountControl_i32--;
    }
}

void IRAM_ATTR FastNonAccelStepper::forceStop()
{
    // Immediately force the step pin to a known inactive state (LOW).
    // This prevents an extra step pulse from completing after the stop command is issued from the ISR.
    // mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);

    // Now, schedule the timer to stop cleanly at the end of its cycle.
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
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
        currentDirState_b = dirLevelForward_b; // Keep internal state in sync
    }
    else
    {
        digitalWrite(dirPin_u8, dirLevelBackward_b);
        currentDirState_b = dirLevelBackward_b; // Keep internal state in sync
    }

    pcnt_counter_pause(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_1);

    pcnt_event_disable(PCNT_UNIT_1, PCNT_EVT_H_LIM);
    pcnt_event_disable(PCNT_UNIT_1, PCNT_EVT_L_LIM);

    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_1);

    setMaxSpeed(speed_u32); 

    delayMicroseconds(5);	
    isRunning_b = true;
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
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

void IRAM_ATTR FastNonAccelStepper::setExpectedCycleTimeUs(uint32_t cycleTimeUs_u32) 
{
    expectedCycleTimeUs_u32 = cycleTimeUs_u32;
}

void IRAM_ATTR FastNonAccelStepper::pauseOutput() {
    // Force the pin LOW immediately (continuous software force), the timer keeps running.
    // mcpwm_set_signal_low() is not suitable here: it only changes the generator actions for
    // future events, so a pulse that is already high runs on until its compare event. At a
    // direction change the DIR pin then switched in the middle of that pulse: its rising and
    // falling edge saw different DIR levels, and a counter using the other edge than the PCNT
    // (e.g. the servo) counted it in the opposite direction (2 steps offset per event,
    // measured with a logic analyzer at 8 MS/s).
    MCPWM0.operators[0].gen_force.gen_cntuforce_upmethod = 0; // take effect immediately
    MCPWM0.operators[0].gen_force.gen_a_cntuforce_mode = 1;   // 1: continuously LOW
    outputPaused_b = true;
    forceReleasePending_b = false;

    // make sure the pin is really low before the caller switches DIR (bounded, ~1 us)
    uint32_t startUs_u32 = micros();
    while ((gpio_get_level((gpio_num_t)stepPin_u8) != 0) && ((micros() - startUs_u32) < 3)) {
    }
}

void IRAM_ATTR FastNonAccelStepper::restartPeriodWhilePaused() {
    // Only valid while pauseOutput() holds the pin LOW: nothing can be cut or merged then.
    // 1) load the period and compare written by setSpeedLive() immediately
    MCPWM0.update_cfg.global_up_en = 1;
    MCPWM0.update_cfg.op0_up_en = 1;
    MCPWM0.update_cfg.global_force_up = !MCPWM0.update_cfg.global_force_up;
    activeCompareMax_u16 = lastCompare_u16;
    // 2) software sync: the counter restarts at phase 0 (start of the high phase)
    HAL_FORCE_MODIFY_U32_REG_FIELD(MCPWM0.timer[0].timer_sync, timer_phase, 0);
    MCPWM0.timer[0].timer_sync.timer_sync_sw = ~MCPWM0.timer[0].timer_sync.timer_sync_sw;
    periodJustRestarted_b = true;
}

void IRAM_ATTR FastNonAccelStepper::resumeOutput() {
    // Hand the pin back to the timer without cutting a sliver pulse: while the counter is in
    // the high phase of the running period, releasing the force would raise the pin for only
    // the rest of that phase. Release immediately only when the output would be low anyway
    // (timer idle or counter past every active compare), otherwise at the end of the period
    // (TEZ), where the next pulse starts with its full width.
    // Right after restartPeriodWhilePaused() the counter is at the start of the high phase:
    // releasing now gives a full-width pulse.
    outputPaused_b = false;
    uint32_t counter_u32 = MCPWM0.timer[0].timer_status.timer_value;
    bool timerIdle_b = (MCPWM0.timer[0].timer_cfg1.timer_start == 0) && (counter_u32 == 0);
    bool atPeriodStart_b = periodJustRestarted_b && (counter_u32 < FORCED_UPDATE_GUARD_TICKS);
    periodJustRestarted_b = false;
    if (timerIdle_b || atPeriodStart_b || (counter_u32 >= activeCompareMax_u16))
    {
        MCPWM0.operators[0].gen_force.gen_cntuforce_upmethod = 0;
        MCPWM0.operators[0].gen_force.gen_a_cntuforce_mode = desiredForceMode_u8;
        forceReleasePending_b = false;
    }
    else
    {
        MCPWM0.operators[0].gen_force.gen_cntuforce_upmethod = 1; // at TEZ
        MCPWM0.operators[0].gen_force.gen_a_cntuforce_mode = desiredForceMode_u8;
        forceReleasePending_b = true;
        forceReleaseCounter_u16 = (uint16_t)counter_u32;
    }
}

void IRAM_ATTR FastNonAccelStepper::setSpeedLive(uint32_t speed_u32) 
{
    // 1. enable register updates
    MCPWM0.update_cfg.global_up_en = 1;
    MCPWM0.update_cfg.op0_up_en = 1;

	// constrain to allowed intervall
	speed_u32 = constrain(speed_u32, 1, MAX_SPEED_IN_HZ);

    // write to variable that tracks the max speed (for later retrieval and for use in )
    maxSpeed_u32 = speed_u32;

    // forceStop() only commands "stop at the end of the period" (timer_start reads 0 at once),
    // the timer is idle when additionally the counter has reached zero
    bool stopCommanded_b = (MCPWM0.timer[0].timer_cfg1.timer_start == 0);

    // A force release scheduled for the end of the period (resumeOutput) has taken effect once
    // the counter wrapped (or the timer is idle); from then on force changes act immediately.
    if (forceReleasePending_b)
    {
        uint32_t counterNow_u32 = MCPWM0.timer[0].timer_status.timer_value;
        if ((stopCommanded_b && counterNow_u32 == 0) || (counterNow_u32 < forceReleaseCounter_u16))
        {
            forceReleasePending_b = false;
            MCPWM0.operators[0].gen_force.gen_cntuforce_upmethod = 0;
        }
    }

    // 2. timer period and compare (end of the high phase) for the new speed
    uint32_t effectiveSpeed = (speed_u32 < MINIMUM_PULSE_FREQUENCY_U32) ? MINIMUM_PULSE_FREQUENCY_U32 : speed_u32;
    uint32_t period = TIMER_RESOLUTION_IN_HZ_U32 / effectiveSpeed;
    if (period > 0) period--;
    if (period > 0xFFFF) period = 0xFFFF;
    uint32_t compare = (period + 1) / 2;

    // 3. write the shadow registers; by default they are loaded at the end of the running period (TEZ)
    MCPWM0.timer[0].timer_cfg0.timer_period_upmethod = 1;
    MCPWM0.timer[0].timer_cfg0.timer_period = period;
    MCPWM0.operators[0].gen_stmp_cfg.gen_a_upmethod = 1;
    MCPWM0.operators[0].timestamp[0].gen = compare;

    // 4. force stop logic for very low speeds to prevent stalling, with automatic reanimation when speed is increased again
    //    (while the output is paused for a direction change, only remember the mode:
    //    resumeOutput() applies it)
    desiredForceMode_u8 = (speed_u32 < MINIMUM_PULSE_FREQUENCY_U32) ? 1 : 0;
    if (!outputPaused_b)
    {
        MCPWM0.operators[0].gen_force.gen_a_cntuforce_mode = desiredForceMode_u8;
    }

    // 5. Load the new values immediately only when that cannot break the running period:
    //    a) the timer is idle (stopped at the end of a period)
    //    b) the counter is still before the new compare: the high phase ends at the new compare,
    //       the period at the new period
    //    c) the output is already low (counter past every compare value that may be active)
    //       and the counter is still before the new period end
    //    Otherwise they are loaded at the end of the current period. A forced update with the
    //    counter already past the new period end would miss it (the 16 bit counter runs on and
    //    wraps: long pulse gap), past the new compare while still high it would merge two pulses.
    uint32_t counter_u32 = MCPWM0.timer[0].timer_status.timer_value;
    bool timerIdle_b = stopCommanded_b && (counter_u32 == 0);
    //    Not while a force release waits for the end of the period: a forced update would apply
    //    the release at once (sliver pulse).
    bool loadNow_b = timerIdle_b ||
                     (!forceReleasePending_b &&
                      ((counter_u32 + FORCED_UPDATE_GUARD_TICKS < compare) ||
                       ((counter_u32 >= activeCompareMax_u16) && (counter_u32 + FORCED_UPDATE_GUARD_TICKS < period))));
    if (loadNow_b)
    {
        // a toggle triggers one forced update of all active registers
        MCPWM0.update_cfg.global_force_up = !MCPWM0.update_cfg.global_force_up;
        activeCompareMax_u16 = (uint16_t)compare;
    }
    else
    {
        // until the end of the period either the old or the new compare is active
        if (compare > activeCompareMax_u16) activeCompareMax_u16 = (uint16_t)compare;
    }
    lastCompare_u16 = (uint16_t)compare;

    // 6. reanimation if stopped (after the registers hold the new speed)
    if (stopCommanded_b && speed_u32 >= MINIMUM_PULSE_FREQUENCY_U32)
    {
        MCPWM0.timer[0].timer_cfg1.timer_start = 2;
        isRunning_b = true;
    }
}

void IRAM_ATTR FastNonAccelStepper::waitForMinimumPulseWidth()
{
    if ((MCPWM0.timer[0].timer_cfg1.timer_start == 0) && (MCPWM0.timer[0].timer_status.timer_value == 0))
    {
        return; // timer idle, no pulse running
    }

    // The high phase starts at counter 0 (TEZ) and ends at the active compare value. Wait until
    // the running pulse is at least MIN_STEP_PULSE_HIGH_TIME_US long (or already low). Bounded.
    const uint32_t minHighTicks_u32 = MIN_STEP_PULSE_HIGH_TIME_US * (TIMER_RESOLUTION_IN_HZ_U32 / 1000000u);
    uint32_t startUs_u32 = micros();
    while ((micros() - startUs_u32) <= (MIN_STEP_PULSE_HIGH_TIME_US + 1))
    {
        uint32_t counter_u32 = MCPWM0.timer[0].timer_status.timer_value;
        if ((counter_u32 >= minHighTicks_u32) || (counter_u32 >= activeCompareMax_u16))
        {
            return;
        }
    }
}

void IRAM_ATTR FastNonAccelStepper::moveToWithSpeed(int32_t targetPos_i32, uint32_t speed_u32)
{
    int32_t currentPos_i32 = getCurrentPosition();
    int32_t stepsToMove_i32 = targetPos_i32 - currentPos_i32;

    // --- DEADBAND: The most crucial protection for your admittance model ---
    // If we are at the target (or just oscillating by 1 step), we abort immediately.
    // The hardware remains untouched and generates no false pulses!
    if (abs(stepsToMove_i32) <= 1) {
		// maxSpeed_u32 = 0; // Don't reset as this will scew up plots 
        return; 
    }
    
    // Determine direction
    bool forward = (stepsToMove_i32 >= 0);
    uint8_t targetDirLevel = forward ? dirLevelForward_b : dirLevelBackward_b;

    // Check if a direction change is happening
    bool directionChanged = (targetDirLevel != currentDirState_b);

    if (directionChanged) {
        // Force the PWM output LOW to prevent ghost pulses during setup, but do not cut a
        // running step pulse too short: a sliver would be counted by the servo but filtered
        // by the PCNT (or vice versa) and ESP and servo position would drift apart.
        waitForMinimumPulseWidth();
        pauseOutput();
        delayMicroseconds(1); // Give hardware a tiny moment
    }

    int32_t absStepsToMove_i32 = abs(stepsToMove_i32);

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
    // 2) define upper limit for control pcnt
    // 3) define lower limit for control pcnt
    // (the DIR pin is only written when the direction actually changes)
    if (stepsToMove_i32 > 0)
    {
        if (directionChanged) {
            digitalWrite(dirPin_u8, dirLevelForward_b);
        }
        currentDirState_b = dirLevelForward_b; // Keep internal state in sync

        highLimit_i16 = limit_i16;
        lowLimit_i16 = -MCPWM_PCNT_MAX_ALLOWED_MOVEMENT_IN_OPPOSITE_DIR_TILL_STOP;
        overflowCountControl_i32 = numbWraps_i32;
    }
    else if (stepsToMove_i32 < 0)
    {
        if (directionChanged) {
            digitalWrite(dirPin_u8, dirLevelBackward_b);
        }
        currentDirState_b = dirLevelBackward_b; // Keep internal state in sync

        highLimit_i16 = MCPWM_PCNT_MAX_ALLOWED_MOVEMENT_IN_OPPOSITE_DIR_TILL_STOP;
        lowLimit_i16 = -limit_i16;
        overflowCountControl_i32 = numbWraps_i32;
    }
    else
    {
        //digitalWrite(dirPin_u8, dirLevelBackward_b);
        // do not change previous direction
        highLimit_i16 = MCPWM_PCNT_MAX_ALLOWED_MOVEMENT_IN_OPPOSITE_DIR_TILL_STOP;
        lowLimit_i16 = -MCPWM_PCNT_MAX_ALLOWED_MOVEMENT_IN_OPPOSITE_DIR_TILL_STOP;
        overflowCountControl_i32 = 0;
    }

    if (directionChanged) {
        // Setup time for stepper driver direction pin before new pulses arrive
        delayMicroseconds(DIR_SETUP_TIME_US);
    }

    // parameterize control pcnt: same register sequence as the legacy driver calls
    // (pause, clear, set limits, enable limit events, clear, resume), written directly.
    // pause/clear/resume modify the control register shared by all PCNT units, hence the
    // critical section (the driver used a spinlock per call for the same reason).
    portENTER_CRITICAL_SAFE(&s_pcntRegisterMux);
    pcnt_ll_stop_count(&PCNT, PCNT_UNIT_1);
    pcnt_ll_clear_count(&PCNT, PCNT_UNIT_1);
    pcnt_ll_set_high_limit_value(&PCNT, PCNT_UNIT_1, highLimit_i16);
    pcnt_ll_set_low_limit_value(&PCNT, PCNT_UNIT_1, lowLimit_i16);
    pcnt_ll_enable_high_limit_event(&PCNT, PCNT_UNIT_1, true);
    pcnt_ll_enable_low_limit_event(&PCNT, PCNT_UNIT_1, true);
    pcnt_ll_clear_count(&PCNT, PCNT_UNIT_1);
    pcnt_ll_start_count(&PCNT, PCNT_UNIT_1);
    portEXIT_CRITICAL_SAFE(&s_pcntRegisterMux);

    // Setze das Hardware-Limit für UNIT 1 als Sicherheitsfangnetz
    // Wir setzen es immer ein Stück weiter als das aktuelle Ziel
    //int16_t safetyLimit = (int16_t)constrain(abs(stepsToMove_i32) + 100, 0, PCNT_MIN_MAX_THRESHOLD);
    //pcnt_set_event_value(PCNT_UNIT_1, forward ? PCNT_EVT_H_LIM : PCNT_EVT_L_LIM, safetyLimit);

    // Update speed without jitter and hand the pin back to the timer. With the timer idle the
    // output is released first, so the (re)start begins with a full pulse; with the timer
    // running resumeOutput() picks a release point that does not cut a pulse.
    bool timerIdleAtResume_b = (MCPWM0.timer[0].timer_cfg1.timer_start == 0) &&
                               (MCPWM0.timer[0].timer_status.timer_value == 0);
    if (directionChanged && timerIdleAtResume_b) {
        resumeOutput();
    }
    setSpeedLive(speed_u32);

    // Resume PWM output or start if it was completely stopped
    if (directionChanged) {
        if (!timerIdleAtResume_b) {
            // The timer still runs with the period from before the reversal (up to 6.5 ms at
            // the lowest speed). Waiting for its end delayed the first pulse in the new
            // direction by up to that long. Instead, while the output is still forced LOW:
            // load the new speed at once and restart the period, then hand the pin back at
            // the start of the period, so the first pulse starts now with its full width.
            restartPeriodWhilePaused();
            resumeOutput();
        }
    } else if (!isRunning_b && speed_u32 > 10) {
        isRunning_b = true;
        mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
    }
}
