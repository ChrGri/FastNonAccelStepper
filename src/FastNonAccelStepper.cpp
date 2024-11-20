#include "FastNonAccelStepper.h"
#include <driver/mcpwm.h>
#include <driver/pcnt.h>




/************************************************************************/
/*								Defines 								*/
/************************************************************************/
#define MAX_SPEED_IN_HZ (int32_t)400000
#define MAX_ALLOWED_POSITION_CHANGE_PER_CYCLE (int32_t)20000
#define PWM_DUTY_CYCLE 50.0f
#define PCNT_MIN_MAX_THRESHOLD 32767l // INT16_MAX = (2^15)-1 = 32767
#define POSITION_TRIGGER_THRESHOLD 1
#define PCNT_FILTER_VALUE 10


/************************************************************************/
/*								Implementation							*/
/************************************************************************/
FastNonAccelStepper::FastNonAccelStepper()
    : _stepPin(0), _dirPin(0), _targetPosition(0), _maxSpeed(MAX_SPEED_IN_HZ), _overflowCount(0), _pcntQueue(nullptr) {}

void FastNonAccelStepper::begin(uint8_t stepPin, uint8_t dirPin) {
    _stepPin = stepPin;
    _dirPin = dirPin;
    _maxSpeed = MAX_SPEED_IN_HZ;

	// init MCPWM and PCNTs
    initMCPWM();
    initPCNTMultiturn();
    initPCNTControl();

    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    digitalWrite(_stepPin, LOW);
    digitalWrite(_dirPin, LOW);

	// configure pin modes
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, _stepPin);

	// connect PCNT to GPIO pin
	gpio_iomux_in(_stepPin, PCNT_SIG_CH0_IN0_IDX);
	gpio_iomux_in(_stepPin, PCNT_SIG_CH0_IN1_IDX);

	// reset pcnt counters
	pcnt_counter_clear(PCNT_UNIT_0);
	pcnt_counter_clear(PCNT_UNIT_1);

    // make sure mcpwm is stopped
    forceStop();
	  
}

void FastNonAccelStepper::setMaxSpeed(uint32_t speed) {
    // Constrain the speed to valid limits
    _maxSpeed = constrain(speed, 1, MAX_SPEED_IN_HZ);

    // Update the MCPWM timer with the new frequency
    if (_maxSpeed > 0) {
        mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, _maxSpeed);
        forceStop();
    } else {
        //mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        forceStop();
    }
}


void FastNonAccelStepper::move(long stepsToMove) {
    long positionChange = constrain(stepsToMove, -MAX_ALLOWED_POSITION_CHANGE_PER_CYCLE, MAX_ALLOWED_POSITION_CHANGE_PER_CYCLE);

    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);

    int16_t absPositionChange = abs(positionChange);

    if (absPositionChange > POSITION_TRIGGER_THRESHOLD) {
        int16_t highLimit = absPositionChange - 1; // subtract one, since count limit is triggered after value is exceeded
        int16_t lowLimit = -highLimit;

        pcnt_counter_pause(PCNT_UNIT_1);
        pcnt_counter_clear(PCNT_UNIT_1);
        pcnt_set_event_value(PCNT_UNIT_1, PCNT_EVT_H_LIM, highLimit);
        pcnt_set_event_value(PCNT_UNIT_1, PCNT_EVT_L_LIM, lowLimit);
        pcnt_counter_clear(PCNT_UNIT_1);
        pcnt_counter_resume(PCNT_UNIT_1);

        if (positionChange > 0) {
            digitalWrite(_dirPin, HIGH);
        } else {
            digitalWrite(_dirPin, LOW);
        }
        _isRunning = true;
        mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
    }
}


void FastNonAccelStepper::moveTo(long targetPos) {
    long currentPos = getCurrentPosition();
    long positionChange = constrain(targetPos - currentPos, -MAX_ALLOWED_POSITION_CHANGE_PER_CYCLE, MAX_ALLOWED_POSITION_CHANGE_PER_CYCLE);
    _targetPosition = currentPos + positionChange;
    move(positionChange);
} 

long FastNonAccelStepper::getCurrentPosition() const {
    int16_t pulseCount = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &pulseCount);
    return ((long)_overflowCount * (long)PCNT_MIN_MAX_THRESHOLD) + (long)pulseCount - _zeroPosition_i32;
}


void FastNonAccelStepper::initMCPWM() {
    mcpwm_config_t pwmConfig;
    pwmConfig.frequency = _maxSpeed;
    pwmConfig.cmpr_a = PWM_DUTY_CYCLE;
    pwmConfig.cmpr_b = 0.0;
    pwmConfig.counter_mode = MCPWM_UP_COUNTER;
    pwmConfig.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwmConfig);
}

void FastNonAccelStepper::initPCNTMultiturn() {
    pcnt_config_t pcntConfig;
    pcntConfig.pulse_gpio_num = _stepPin;
    pcntConfig.ctrl_gpio_num = _dirPin;
    pcntConfig.channel = PCNT_CHANNEL_0;
    pcntConfig.unit = PCNT_UNIT_0;
    pcntConfig.pos_mode = PCNT_COUNT_INC;
    pcntConfig.neg_mode = PCNT_COUNT_DIS;
    pcntConfig.lctrl_mode = PCNT_MODE_REVERSE;
    pcntConfig.hctrl_mode = PCNT_MODE_KEEP;
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

void FastNonAccelStepper::initPCNTControl() {
    pcnt_config_t pcntConfig;
    pcntConfig.pulse_gpio_num = _stepPin;
    pcntConfig.ctrl_gpio_num = _dirPin;
    pcntConfig.channel = PCNT_CHANNEL_0;
    pcntConfig.unit = PCNT_UNIT_1;
    pcntConfig.pos_mode = PCNT_COUNT_INC;
    pcntConfig.neg_mode = PCNT_COUNT_DIS;
    pcntConfig.lctrl_mode = PCNT_MODE_REVERSE;
    pcntConfig.hctrl_mode = PCNT_MODE_KEEP;
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

void IRAM_ATTR FastNonAccelStepper::multiturnPCNTISR(void* arg) {
    FastNonAccelStepper* instance = static_cast<FastNonAccelStepper*>(arg);
    uint32_t status;
    pcnt_get_event_status(PCNT_UNIT_0, &status);

    if (status & PCNT_EVT_H_LIM) {
        instance->_overflowCount++;
    }
    if (status & PCNT_EVT_L_LIM) {
        instance->_overflowCount--;
    }
}

void IRAM_ATTR FastNonAccelStepper::controlPCNTISR(void* arg) {
    FastNonAccelStepper* instance = static_cast<FastNonAccelStepper*>(arg);
    uint32_t status;
    pcnt_get_event_status(PCNT_UNIT_1, &status);

    if (status & PCNT_EVT_H_LIM || status & PCNT_EVT_L_LIM) {
        //mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        instance->forceStop();
    }
}

void FastNonAccelStepper::forceStop()
{
    // stop mcpwm
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    _isRunning = false;
}  

void FastNonAccelStepper::setCurrentPosition(int32_t newPosition_i32)
{
    // set new position
    // newPosition_i32 = (getCurrentPosition + oldZeroPos) - (newZeroPos)
    // newZeroPos = (getCurrentPosition + oldZeroPos) - newPosition_i32
    int32_t newZeroPos_i32 = (getCurrentPosition() + _zeroPosition_i32) - newPosition_i32;
    _zeroPosition_i32 = newZeroPos_i32;
}

void FastNonAccelStepper::forceStopAndNewPosition(int32_t newPosition_i32)
{
    // stop mcpwm
    forceStop();

    // set new position
    setCurrentPosition(newPosition_i32);
}   

bool FastNonAccelStepper::isRunning()
{
    return _isRunning;
}

void FastNonAccelStepper::keepRunningInDir(bool forwardDir, uint32_t speed)
{
    forceStop();

    pcnt_counter_pause(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_1);

    pcnt_event_disable(PCNT_UNIT_1, PCNT_EVT_H_LIM);
    pcnt_event_disable(PCNT_UNIT_1, PCNT_EVT_L_LIM);

    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_1);

    setMaxSpeed(speed); 
    
    if (forwardDir)
    {
        digitalWrite(_dirPin, HIGH);
    }
    else
    {
        digitalWrite(_dirPin, LOW);
    }

    _isRunning = true;
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
}

void FastNonAccelStepper::keepRunningForward(uint32_t speed)
{
    keepRunningInDir(true, speed);
}

void FastNonAccelStepper::keepRunningBackward(uint32_t speed)
{
    keepRunningInDir(false, speed);
}

int32_t FastNonAccelStepper::getPositionAfterCommandsCompleted()
{
  return _targetPosition;
}



