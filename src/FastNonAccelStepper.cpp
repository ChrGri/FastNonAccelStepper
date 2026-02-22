#include "FastNonAccelStepper.h"
#include <driver/mcpwm.h>
#include <driver/pcnt.h>




/************************************************************************/
/*								Defines 								                              */
/************************************************************************/
#define MAX_ALLOWED_POSITION_CHANGE_PER_CYCLE (int32_t)20000
#define PWM_DUTY_CYCLE 50.0f
#define PCNT_MIN_MAX_THRESHOLD (int16_t)32767 // INT16_MAX = (2^15)-1 = 32767
#define POSITION_TRIGGER_THRESHOLD 1
#define PCNT_FILTER_VALUE 1
#define MCPWM_PCNT_MAX_ALLOWED_MOVEMENT_IN_OPPOSITE_DIR_TILL_STOP 50


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

    // init MCPWM and PCNTs
    initMCPWM();
    initPCNTMultiturn(); // unit to track the overall position
    initPCNTControl(); // unit to controll the step bursts

    pinMode(stepPin_u8, OUTPUT);
    pinMode(dirPin_u8, OUTPUT);
    digitalWrite(stepPin_u8, LOW);
    digitalWrite(dirPin_u8, LOW);

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

    // Update the MCPWM timer with the new frequency
    if (maxSpeed_u32 > 0)
    {
        mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, maxSpeed_u32);
        forceStop();
    }
    else
    {
        forceStop();
    }
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
        // 2) define upper limit for control pcnt
        // 3) define lower limit for control pcnt
        if (stepsToMove_i32 > 0)
        {
            digitalWrite(dirPin_u8, dirLevelForward_b);
            highLimit_i16 = limit_i16;
            lowLimit_i16 = -MCPWM_PCNT_MAX_ALLOWED_MOVEMENT_IN_OPPOSITE_DIR_TILL_STOP;
            overflowCountControl_i32 = numbWraps_i32;
        }
        else
        {
            digitalWrite(dirPin_u8, dirLevelBackward_b);
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
            while(isRunning())
            {
                delay(1);
          /*int16_t pulseCountlcl = 0;
          pcnt_get_counter_value(PCNT_UNIT_1, &pulseCountlcl);
          Serial.printf( "CurPos: %d,    CtrlPos:%d,    overfl: %d\n", getCurrentPosition(), pulseCountlcl, _overflowCountControl);
          delay(30);*/
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
    int16_t pulseCount_i16 = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &pulseCount_i16);
    return ((int32_t)overflowCount_i32 * (int32_t)PCNT_MIN_MAX_THRESHOLD) + (int32_t)pulseCount_i16 - zeroPosition_i32;
}


void FastNonAccelStepper::initMCPWM()
{
    mcpwm_config_t pwmConfig;
    pwmConfig.frequency = maxSpeed_u32;
    pwmConfig.cmpr_a = PWM_DUTY_CYCLE;
    pwmConfig.cmpr_b = 0.0f;
    pwmConfig.counter_mode = MCPWM_UP_COUNTER;
    pwmConfig.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwmConfig);
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
    }
    else
    {
        digitalWrite(dirPin_u8, dirLevelBackward_b);
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
