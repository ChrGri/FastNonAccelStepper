#include <Arduino.h>
#include "FastNonAccelStepper.h"

#define STEP_PIN 18
#define DIR_PIN 19

FastNonAccelStepper* _stepper;

const float targetHz_fl32 = 0.5f;
const float amplitude_fl32 = 4000.0f;
unsigned long previousMicros_u32 = 0;
const unsigned long loopIntervalUs_u32 = 250;

void setup() 
{
    Serial.begin(115200);
    while(!Serial) { delay(10); } // Wait for USB Serial to connect
    Serial.println("System Ready...");

    // Initializing hardware
    _stepper = new FastNonAccelStepper(STEP_PIN, DIR_PIN, false); 
    _stepper->begin();
    _stepper->setMaxSpeed(250000);
    _stepper->setExpectedCycleTimeUs(1000);

    delay(1000);
}

float speedWas_fl32 = 0.0f;
void loop() 
{
    unsigned long currentMicros_u32 = micros();

//#define VIA_PSOITIONING_COMMAND
#ifdef VIA_PSOITIONING_COMMAND
    if (currentMicros_u32 - previousMicros_u32 >= loopIntervalUs_u32) 
    {
        previousMicros_u32 = currentMicros_u32;
        float t_fl32 = currentMicros_u32 / 1000000.0f;

        float posSoll_fl32 = amplitude_fl32 * sin(2.0f * PI * targetHz_fl32 * t_fl32);
        float vSoll_fl32 = amplitude_fl32 * (2.0f * PI * targetHz_fl32) * cos(2.0f * PI * targetHz_fl32 * t_fl32);

        //_stepper->moveTo((int32_t)posSoll_fl32, false);
        //_stepper->setSpeedLive((uint32_t)abs(vSoll_fl32));

        _stepper->moveToWithSpeed((int32_t)posSoll_fl32, (uint32_t)abs(vSoll_fl32));

        // print when velocity crosses zero for better visibility on the analyzer
        if (vSoll_fl32 * speedWas_fl32 < 0.0f)
        {
            Serial.printf("Time: %.2f s, Pos Soll: %.2f, Vel Soll: %.2f\n", t_fl32, posSoll_fl32, vSoll_fl32);
        }
        speedWas_fl32 = vSoll_fl32;
    
    }

#else

    if (currentMicros_u32 - previousMicros_u32 >= loopIntervalUs_u32) 
    {
        previousMicros_u32 = currentMicros_u32;
        float t_fl32 = currentMicros_u32 / 1000000.0f;

        //float vSoll_fl32 = amplitude_fl32 * (2.0f * PI * targetHz_fl32) * cos(2.0f * PI * targetHz_fl32 * t_fl32);


        float vSoll_fl32 = 250000.0f * cos(2.0f * PI * targetHz_fl32 * t_fl32);

        // 1. Richtung setzen ohne den Timer zu stoppen
        digitalWrite(DIR_PIN, vSoll_fl32 >= 0 ? HIGH : LOW); 

        // 2. Nur Frequenz anpassen (setSpeedLive nutzt kein forceStop!)
        _stepper->setSpeedLive((uint32_t)abs(vSoll_fl32));

        // 3. Nur sicherstellen, dass der PWM-Generator läuft (falls er noch nicht gestartet wurde)
        if (!_stepper->isRunning()) {
            _stepper->keepRunningInDir(vSoll_fl32 >= 0, (uint32_t)abs(vSoll_fl32));
        }
    }

#endif

}