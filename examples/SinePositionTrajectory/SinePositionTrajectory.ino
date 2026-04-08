#include <Arduino.h>
#include "FastNonAccelStepper.h"

#define STEP_PIN 18
#define DIR_PIN 19

FastNonAccelStepper stepper(STEP_PIN, DIR_PIN, false);

// Parameter für 5Hz Sinus
const float targetHz = 0.2; 
// Amplitude für ~200kHz Peak (A * 2 * PI * f = 200.000)
const float amplitude = 4000.0; 

unsigned long previousMicros = 0;
const unsigned long loopInterval_us = 250; 
int32_t debugV = 0;

void setup() {
    Serial.begin(115200);
    stepper.begin(); // Initialisiert MCPWM und PCNT Hardware
    stepper.setExpectedCycleTimeUs(loopInterval_us);
    delay(1000);
}

void loop() {
    unsigned long currentMicros = micros();

    // High-Speed Control Loop (300us - 250us)
    if (currentMicros - previousMicros >= loopInterval_us) {
        previousMicros = currentMicros;
        float t = currentMicros / 1000000.0;

        // 1. Soll-Position (Sinus)
        float pos_soll = amplitude * sin(2.0 * PI * targetHz * t);
        
        // 2. Soll-Geschwindigkeit (Cosinus Ableitung)
        float v_soll = amplitude * (2.0 * PI * targetHz) * cos(2.0 * PI * targetHz * t);
        debugV = (int32_t)v_soll;

        // 3. Hardware-Ansteuerung über die kombinierten Interfaces
        // moveTo setzt das Ziel für den PCNT Hardware-Counter
        stepper.moveTo((int32_t)pos_soll, false); 
        
        // setSpeedLive passt die MCPWM Frequenz "on the fly" an
        stepper.setSpeedLive((uint32_t)abs(debugV));
    }

    // Telemetrie für den Serial Plotter (alle 20ms)
    static uint32_t lastPlot = 0;
    if (millis() - lastPlot > 20) {
        lastPlot = millis();
        
        // Darstellung im Arduino Serial Plotter
        Serial.print("Ist_Pos:"); 
        Serial.print(stepper.getCurrentPosition());
        Serial.print(",");
        
        Serial.print("Soll_Pos:"); 
        Serial.print((int32_t)(amplitude * sin(2.0 * PI * targetHz * (micros()/1000000.0))));
        Serial.print(",");
        
        // Geschwindigkeit skaliert (/10), damit sie im Plot nicht die Position dominiert
        Serial.print("Speed_div10:"); 
        Serial.println(debugV / 10);
    }
}