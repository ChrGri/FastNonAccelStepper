#include "FastNonAccelStepper.h"
#include "driver/pcnt.h"





/**************************************************************************************/
/*                              Task                                                  */
/**************************************************************************************/

void StepperTask(void *parameter) {

  FastNonAccelStepper *instance = (FastNonAccelStepper *)parameter;
  
  while (true) {

    // obtain current states
    long curPos_i32 = instance->currentPosition();
    long targetPos_i32 = instance->targetPosition();
    bool dir_b = instance->getDirection();


    // check if target position was hit or surpassed, in case stop MCPWM output
	  if (HIGH == dir_b) {
      // stop movement when target is reached or exceeded
      if (curPos_i32 >= targetPos_i32)
      {
        //mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, 0); // Stop PWM
        instance->stop();
        Serial.println("Position reached");
      }
    } else {
      
      // stop movement when target is reached or exceeded
      if (curPos_i32 <= targetPos_i32)
      {
        //mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, 0); // Stop PWM
        instance->stop();
        Serial.println("Position reached");
      }
    }


    // wait some microseconds before checking again
    //delayMicroseconds(500);
    vTaskDelay(1); // Short delay to yield control

  }
}


void fas_init_engine(FastNonAccelStepper *engine, uint8_t cpu_core) {
#if ESP_IDF_VERSION_MAJOR == 4
  #define STACK_SIZE 2000
  #define PRIORITY configMAX_PRIORITIES
#else
  #define STACK_SIZE 3000
  #define PRIORITY (configMAX_PRIORITIES - 1)
#endif
  if (cpu_core > 1) {
    xTaskCreate(StepperTask, "StepperTask", STACK_SIZE, engine, PRIORITY, NULL);
  } else {
    xTaskCreatePinnedToCore(StepperTask, "StepperTask", STACK_SIZE, engine,
                            PRIORITY, NULL, cpu_core);
  }
}






// GPIO Interrupt Service Routine (ISR) to track pulses
//void IRAM_ATTR FastNonAccelStepper::gpio_isr_handler(void* arg) {
void IRAM_ATTR gpio_isr_handler(void* arg) {
    FastNonAccelStepper* instance = static_cast<FastNonAccelStepper*>(arg);
	
	// update the current position
	if (instance->getDirection() == HIGH) {
        // Forward direction (increment position)
      instance->addTocCurrentPosition(1);
      
      // stop movement when target is reached or exceeded
      if (instance->currentPosition() >= instance->targetPosition())
      {
        //mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, 0); // Stop PWM
        instance->stop();
      }
	
    } else {
        // Reverse direction (decrement position)
      instance->addTocCurrentPosition(-1);
      
      // stop movement when target is reached or exceeded
      if (instance->currentPosition() <= instance->targetPosition())
      {
        //mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, 0); // Stop PWM
        instance->stop();
      }

    }
}



/**************************************************************************************/
/*                              FastNonAccelStepper                                   */
/**************************************************************************************/
FastNonAccelStepper::FastNonAccelStepper(int stepPin, int dirPin, int enablePin)
    : _stepPin(stepPin), _dirPin(dirPin), _enablePin(enablePin),
      _currentPosition(0), _targetPosition(0), _moving(false), _maxSpeed(1000), _direction(0), _cpuCore(0) {
}

volatile int pulse_count = 0;  // Number of pulses transmitted
volatile bool count_up = true; // Boolean to control counting direction


#define PCNT_UNIT PCNT_UNIT_0
#define PULSE_COUNT_LIMIT   100000    // High limit for pulse count




// Initialize PCNT with up/down control based on GPIO 19 level
void pcnt_init(uint8_t stepPin, uint8_t dirPin) {

    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = stepPin,        // Pin where pulses are received
        .ctrl_gpio_num = dirPin,          // Control pin for up/down counting
        .lctrl_mode = PCNT_MODE_REVERSE,  // Reverse counting when dirPin is low
        .hctrl_mode = PCNT_MODE_KEEP,     // Keep counting when dirPin is high
        .pos_mode = PCNT_COUNT_INC,       // Count on rising edge of pulses
        .neg_mode = PCNT_COUNT_DIS,       // Ignore falling edges
        .counter_h_lim = PULSE_COUNT_LIMIT, // Set high limit
        .counter_l_lim = -PULSE_COUNT_LIMIT, // Set low limit (optional)
        .unit = PCNT_UNIT,
        .channel = PCNT_CHANNEL_0
    };




    // Initialize PCNT unit with the configuration
    pcnt_unit_config(&pcnt_config);

    // Enable counter and clear to start from zero
    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);
    pcnt_counter_resume(PCNT_UNIT);


    

}





// Interrupt Service Routine (ISR) for counting pulses
void IRAM_ATTR pulse_isr_handler(void* arg) {
    // Check the value of the boolean flag to decide counting direction
    if (count_up) {
        pulse_count++;  // Count up
    } else {
        pulse_count--;  // Count down
    }
}


void FastNonAccelStepper::begin() {
    pinMode(_dirPin, OUTPUT);
    if (_enablePin != -1) {
        pinMode(_enablePin, OUTPUT);
        digitalWrite(_enablePin, LOW); // Enable motor
    }
    
	// setup the MCPWM
	// set it up to 
	setupMCPWM();
	
	// setup the pulse counter
	// this will count the pulses on the _stepPin, depending on the state of the _dirPin
	// if _dirPin is LOW, any pulse on _stepPin will decrement the pcnt value
	// if _dirPin is HIGH, any pulse on _stepPin will increment the pcnt value
	//pcnt_init(_stepPin, _dirPin);
	

  // initialize & start stepper task
  fas_init_engine(this, _cpuCore);
	
	// Attach GPIO ISR to count pulses
  gpio_set_intr_type((gpio_num_t)_stepPin, GPIO_INTR_POSEDGE); // Count rising edge of pulses
  gpio_isr_handler_add((gpio_num_t)_stepPin, gpio_isr_handler, this);
  gpio_intr_enable((gpio_num_t)_stepPin);
}

void FastNonAccelStepper::setupMCPWM() {
    // Configure MCPWM unit and timer for generating PWM pulses on step pin
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, _stepPin);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000; // Default 1kHz frequency (adjustable)
    pwm_config.cmpr_a = 50.0;    // 50% duty cycle
    pwm_config.cmpr_b = 0.0;     // Not used
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
	
	
	// Install interrupt for counting pulses (if needed, for external triggers)
  //esp_intr_alloc(ETS_MCPWM_INTR_SOURCE, ESP_INTR_FLAG_LEVEL1, pulse_isr_handler, NULL, NULL);

}



void FastNonAccelStepper::updateMCPWMFrequency(uint32_t frequency) {
    // Update the PWM frequency for MCPWM, which adjusts the step pulse frequency
    mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, frequency);
}

void FastNonAccelStepper::moveTo(long targetPosition) {
    _targetPosition = targetPosition;
    _moving = (_currentPosition != _targetPosition);

    if (_moving) {
      Serial.println("Start move!");
      _direction = (_targetPosition > _currentPosition) ? HIGH : LOW;
      digitalWrite(_dirPin, _direction);
      //updateMCPWMFrequency(_maxSpeed); // Set frequency based on speed
      updateMCPWMFrequency(1000);
      mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0); // Start MCPWM to ensure signal generation

    }
}

void FastNonAccelStepper::setMaxSpeed(uint32_t speed) {
	// limit the speed to a certain value to prevent calling the counter ISR too often
	_maxSpeed = (speed > MAX_SPEED_IN_HZ) ? MAX_SPEED_IN_HZ : speed;
}

long FastNonAccelStepper::currentPosition() {
  return _currentPosition;
	
	/*// Read the current pulse count
  int16_t pulse_count = 0;
  pcnt_get_counter_value(PCNT_UNIT, &pulse_count);
	return pulse_count;*/
}


long FastNonAccelStepper::addTocCurrentPosition(long add_l)
{
  return _currentPosition += add_l;
}

long FastNonAccelStepper::targetPosition()
{
  return _targetPosition;
}

bool FastNonAccelStepper::getDirection()
{
  return _direction;
}



bool FastNonAccelStepper::isMoving() {
    return _moving;
}


void FastNonAccelStepper::stop() {
    _moving = false;
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0); // Stop the PWM signal
}











