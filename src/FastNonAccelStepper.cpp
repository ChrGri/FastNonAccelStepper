#include "FastNonAccelStepper.h"
#include <driver/mcpwm.h>
#include <driver/pcnt.h>




/************************************************************************/
/*								Defines 								                              */
/************************************************************************/
#define MAX_SPEED_IN_HZ (int32_t)500000
#define MAX_ALLOWED_POSITION_CHANGE_PER_CYCLE (int32_t)20000
#define PWM_DUTY_CYCLE 50.0f
#define PCNT_MIN_MAX_THRESHOLD 32767l // INT16_MAX = (2^15)-1 = 32767
#define POSITION_TRIGGER_THRESHOLD 1
#define PCNT_FILTER_VALUE 1
#define MCPWM_PCNT_MAX_ALLOWED_MOVEMENT_IN_OPPOSITE_DIR_TILL_STOP 50


/************************************************************************/
/*								Implementation							                          */
/************************************************************************/
FastNonAccelStepper::FastNonAccelStepper(uint8_t stepPin, uint8_t dirPin, bool invertMotorDir)
    : _stepPin(stepPin), _dirPin(dirPin), _targetPosition(0), _maxSpeed(MAX_SPEED_IN_HZ), _overflowCount(0), _pcntQueue(nullptr), _invertMotorDirection(invertMotorDir), _zeroPosition_i32(0)
    {
      _stepper = this;  // Assign the current instance to _stepper
      _stepper->begin(_stepPin, _dirPin, _invertMotorDirection);
    }

void FastNonAccelStepper::begin(uint8_t stepPin, uint8_t dirPin, bool invertMotorDir) {
  
    // set dir logic for counting
    // 1) If invertMotorDir == false, the DIR_PIN == HIGH, when motor moving forward and DIR_PIN == LOW, when motor moving backwards
    // 2) If invertMotorDir == false, the PCNT counter should go up, when DIR_PIN == HIGH
    if (false == invertMotorDir)
    {
      _dir_level_forward_b = LOW;
      _dir_level_backward_b = HIGH;
      _dir_pcnt_lctrl_mode_b = PCNT_MODE_KEEP;
      _dir_pcnt_hctrl_mode_b = PCNT_MODE_REVERSE;
    }
    else
    {
      _dir_level_forward_b = HIGH;
      _dir_level_backward_b = LOW;
      _dir_pcnt_lctrl_mode_b = PCNT_MODE_REVERSE;
      _dir_pcnt_hctrl_mode_b = PCNT_MODE_KEEP;
    }

    // init MCPWM and PCNTs
    initMCPWM();
    initPCNTMultiturn(); // unit to track the overall position
    initPCNTControl(); // unit to controll the step bursts

    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    digitalWrite(_stepPin, LOW);
    digitalWrite(_dirPin, LOW);

    // configure pin modes
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, _stepPin);

    // connect PCNT to GPIO pin
    gpio_iomux_in(_stepPin, PCNT_SIG_CH0_IN0_IDX); // overall position unit
    gpio_iomux_in(_stepPin, PCNT_SIG_CH0_IN1_IDX); // controll unit

    // reset pcnt counters
    pcnt_counter_clear(PCNT_UNIT_0); // overall position unit
    pcnt_counter_clear(PCNT_UNIT_1); // controll unit

    // make sure mcpwm is stopped
    forceStop();
	  
}

void IRAM_ATTR FastNonAccelStepper::setMaxSpeed(uint32_t speed) {
    // Constrain the speed to valid limits
    _maxSpeed = constrain(speed, 1, MAX_SPEED_IN_HZ);

    // Update the MCPWM timer with the new frequency
    if (_maxSpeed > 0) {
        mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, _maxSpeed);
        forceStop();
    } else {
        forceStop();
    }
}

uint32_t IRAM_ATTR FastNonAccelStepper::getMaxSpeed(void) {
    // Constrain the speed to valid limits
    return _maxSpeed;
}


void IRAM_ATTR FastNonAccelStepper::move(long stepsToMove, bool blocking) {
    
    // stop previous move
    forceStop();

    long absStepsToMove = abs(stepsToMove);

    if (absStepsToMove > POSITION_TRIGGER_THRESHOLD) {

	  // MATLAB code:
	  // PCNT_MIN_MAX_THRESHOLD = 32767; absStepsToMove = 2*PCNT_MIN_MAX_THRESHOLD; nmbWraps = idivide( int32(absStepsToMove) , int32(PCNT_MIN_MAX_THRESHOLD) ); limit = absStepsToMove / (nmbWraps+1)

		
	  // compute the number of wraps
	  long numbWraps = absStepsToMove / PCNT_MIN_MAX_THRESHOLD;

	  // divide into equally spaced bursts, e.g. when 
	  // if absStepsToMove is < PCNT_MIN_MAX_THRESHOLD --> numbWraps = 0 and limit_i16 = absStepsToMove
	  // if absStepsToMove == PCNT_MIN_MAX_THRESHOLD --> numbWraps = 1 and limit_i16 = absStepsToMove/2
	  // if absStepsToMove == 2*PCNT_MIN_MAX_THRESHOLD --> numbWraps = 2 and limit_i16 = absStepsToMove/3
	  // if absStepsToMove > PCNT_MIN_MAX_THRESHOLD --> numbWraps >= 1
      long limit;
      limit = absStepsToMove / (numbWraps + 1);
      int16_t limit_i16 = constrain(limit, 0, PCNT_MIN_MAX_THRESHOLD);

	  // the highLimit | lowLimit will be hit numbWraps, before stopping the PWM output

      int16_t highLimit;
      int16_t lowLimit;
      // 1) set DIR pin
      // 2) define upper limit for control pcnt
      // 3) define lower limit for control pcnt
      if (stepsToMove > 0) {
          digitalWrite(_dirPin, _dir_level_forward_b);
          highLimit = limit_i16;//absPositionChange - 1;
          lowLimit = -MCPWM_PCNT_MAX_ALLOWED_MOVEMENT_IN_OPPOSITE_DIR_TILL_STOP;
          _overflowCountControl = numbWraps;
      } else {
          digitalWrite(_dirPin, _dir_level_backward_b);
          highLimit = MCPWM_PCNT_MAX_ALLOWED_MOVEMENT_IN_OPPOSITE_DIR_TILL_STOP;
          lowLimit = -limit_i16;//-(absPositionChange - 1);
          _overflowCountControl = numbWraps;
      }

      // parameterize control pcnt
      pcnt_counter_pause(PCNT_UNIT_1);
      pcnt_counter_clear(PCNT_UNIT_1);
      pcnt_set_event_value(PCNT_UNIT_1, PCNT_EVT_H_LIM, highLimit);
      pcnt_set_event_value(PCNT_UNIT_1, PCNT_EVT_L_LIM, lowLimit);
      pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_H_LIM); 
      pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_L_LIM);
      pcnt_counter_clear(PCNT_UNIT_1);
      pcnt_counter_resume(PCNT_UNIT_1);

      //Serial.printf("Hlim: %d,    LLim: %d,    wraps: %d\n", highLimit, lowLimit, numbWraps);

      // start mcpwm
      delayMicroseconds(5);
      _isRunning = true;
      mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);

      if (blocking)
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


void IRAM_ATTR FastNonAccelStepper::moveTo(long targetPos, bool blocking) {
    long currentPos = getCurrentPosition();
    //long positionChange = constrain(targetPos - currentPos, -MAX_ALLOWED_POSITION_CHANGE_PER_CYCLE, MAX_ALLOWED_POSITION_CHANGE_PER_CYCLE);
    long positionChange = targetPos - currentPos;
    _targetPosition = targetPos;
    //_targetPosition = currentPos + positionChange;
    move(positionChange, blocking);
} 

long IRAM_ATTR FastNonAccelStepper::getCurrentPosition() const {
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
    pcntConfig.lctrl_mode = (pcnt_ctrl_mode_t)_dir_pcnt_lctrl_mode_b;
    pcntConfig.hctrl_mode = (pcnt_ctrl_mode_t)_dir_pcnt_hctrl_mode_b;
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
    pcntConfig.lctrl_mode = (pcnt_ctrl_mode_t)_dir_pcnt_lctrl_mode_b;
    pcntConfig.hctrl_mode = (pcnt_ctrl_mode_t)_dir_pcnt_hctrl_mode_b;
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

    //Serial.println("X\n");
    if (status & PCNT_EVT_H_LIM || status & PCNT_EVT_L_LIM) {

      if (instance->_overflowCountControl < 1)
      {
        instance->forceStop();
      }
      instance->_overflowCountControl--;
    }
}

void IRAM_ATTR FastNonAccelStepper::forceStop()
{
    // Immediately force the step pin to a known inactive state (LOW).
    // This prevents an extra step pulse from completing after the stop command is issued from the ISR.
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);

    // Now, schedule the timer to stop cleanly at the end of its cycle.
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    
    _isRunning = false;
}  

void IRAM_ATTR FastNonAccelStepper::setCurrentPosition(int32_t newPosition_i32)
{
    // set new position
    // newPosition_i32 = (getCurrentPosition + oldZeroPos) - (newZeroPos)
    // newZeroPos = (getCurrentPosition + oldZeroPos) - newPosition_i32
    int32_t newZeroPos_i32 = (getCurrentPosition() + _zeroPosition_i32) - newPosition_i32;
    _zeroPosition_i32 = newZeroPos_i32;
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
    return _isRunning;
}

void IRAM_ATTR FastNonAccelStepper::keepRunningInDir(bool forwardDir, uint32_t speed)
{
    forceStop();
	
	if (forwardDir)
    {
        digitalWrite(_dirPin, _dir_level_forward_b);
    }
    else
    {
        digitalWrite(_dirPin, _dir_level_backward_b);
    }

    pcnt_counter_pause(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_1);

    pcnt_event_disable(PCNT_UNIT_1, PCNT_EVT_H_LIM);
    pcnt_event_disable(PCNT_UNIT_1, PCNT_EVT_L_LIM);

    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_1);

    setMaxSpeed(speed); 
    
    

    delayMicroseconds(5);	
    _isRunning = true;
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
}

void IRAM_ATTR FastNonAccelStepper::keepRunningForward(uint32_t speed)
{
    keepRunningInDir(true, speed);
}

void IRAM_ATTR FastNonAccelStepper::keepRunningBackward(uint32_t speed)
{
    keepRunningInDir(false, speed);
}

int32_t IRAM_ATTR FastNonAccelStepper::getPositionAfterCommandsCompleted()
{
  return _targetPosition;
}



