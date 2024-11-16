#ifndef FAST_NONACCEL_STEPPER_H
#define FAST_NONACCEL_STEPPER_H

#include <Arduino.h>
#include "driver/mcpwm.h"

#define MAX_SPEED_IN_HZ 300000

class FastNonAccelStepper {
public:
    FastNonAccelStepper(int stepPin, int dirPin, int enablePin = -1);
    void begin();
    void moveTo(long targetPosition);
    void stop();
    void setMaxSpeed(uint32_t speed);
    long currentPosition();
    long targetPosition();
    bool getDirection();
    bool isMoving();

    long addTocCurrentPosition(long add_l);


private:
    void setupMCPWM();
    void updateMCPWMFrequency(uint32_t frequency);
    //void IRAM_ATTR gpio_isr_handler(void* arg);

    int _stepPin;
    int _dirPin;
    int _enablePin;
    int _cpuCore;
    volatile long _currentPosition;
    volatile long _targetPosition;
    volatile bool _moving;
	volatile bool _direction;
    uint32_t _maxSpeed;
};

#endif // FAST_NONACCEL_STEPPER_H
