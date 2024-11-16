#include "FastNonAccelStepper.h"

FastNonAccelStepper::FastNonAccelStepper(int stepPin, int dirPin, int enablePin)
    : _stepPin(stepPin), _dirPin(dirPin), _enablePin(enablePin),
      _currentPosition(0), _targetPosition(0), _moving(false), _maxSpeed(1000) {
}

void FastNonAccelStepper::begin() {
    pinMode(_dirPin, OUTPUT);
    if (_enablePin != -1) {
        pinMode(_enablePin, OUTPUT);
        digitalWrite(_enablePin, LOW); // Enable motor
    }
    setupMCPWM();
}

void FastNonAccelStepper::setupMCPWM() {
    // Configure MCPWM unit and timer for generating PWM pulses on step pin
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, _stepPin);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000; // Default 1 kHz frequency (adjustable)
    pwm_config.cmpr_a = 50.0;    // 50% duty cycle
    pwm_config.cmpr_b = 0.0;     // Not used
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

void FastNonAccelStepper::updateMCPWMFrequency(uint32_t frequency) {
    // Update the PWM frequency for MCPWM, which adjusts the step pulse frequency
    mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, frequency);
}

void FastNonAccelStepper::moveTo(long targetPosition) {
    _targetPosition = targetPosition;
    _moving = (_currentPosition != _targetPosition);

    if (_moving) {
        digitalWrite(_dirPin, (_targetPosition > _currentPosition) ? HIGH : LOW);
        updateMCPWMFrequency(_maxSpeed); // Set frequency based on speed
    }
}

void FastNonAccelStepper::setMaxSpeed(uint32_t speed) {
    _maxSpeed = speed;
}

long FastNonAccelStepper::currentPosition() {
    return _currentPosition;
}

bool FastNonAccelStepper::isMoving() {
    return _moving;
}

void FastNonAccelStepper::update() {
    if (_moving) {
        if (_currentPosition != _targetPosition) {
            _currentPosition += (_targetPosition > _currentPosition) ? 1 : -1;
        } else {
            _moving = false;
            mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, 0); // Stop PWM
        }
    }
}
