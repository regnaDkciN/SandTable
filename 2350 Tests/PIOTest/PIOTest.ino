#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "STStepper.pio.h"


#if defined TESTING_WITH_WAVEFORMS
const int EN_ROT_PIN         = 2;           // Z axis (rotation) enable pin.
const int STEP_ROT_PIN       = 3;           // Z axis (rotation) step pin.
const int DIR_ROT_PIN        = 4;           // Z axis (rotation) direction pin.
const int EN_INOUT_PIN       = 6;           // X axis (rotation) enable pin.
const int STEP_INOUT_PIN     = 7;           // X axis (rotation) step pin.
const int DIR_INOUT_PIN      = 8;           // X axis (rotation) direction pin.
#endif

const int EN_INOUT_PIN       = 8;           // Y azis (in/out) enable pin.
const int STEP_INOUT_PIN     = 3;           // Y axis (in/out) step pin.
const int DIR_INOUT_PIN      = 6;           // Y axis (in/out) direction pin.
const int EN_ROT_PIN         = 22;          // Z axis (rotation) enable pin.
const int STEP_ROT_PIN       = 4;           // Z axis (rotation) step pin.
const int DIR_ROT_PIN        = 7;           // Z axis (rotation) direction pin.

const float  FREQ = 1000000.0;

STStepper RotStepper(DIR_ROT_PIN, STEP_ROT_PIN, FREQ);
STStepper InoutStepper(DIR_INOUT_PIN, STEP_INOUT_PIN, FREQ);


void setup()
{
    Serial.begin(115200);
    delay(100);

    pinMode(EN_ROT_PIN, OUTPUT);
    digitalWrite(EN_ROT_PIN, 0);

    pinMode(EN_INOUT_PIN, OUTPUT);
    digitalWrite(EN_INOUT_PIN, 0);
}


void loop()
{
    delay(1000);
Serial.printf("top\n");
delay(1000);
    for (int i = 0; i < 200 * 64; i++)
    {
        RotStepper.Step(1);
        InoutStepper.Step(1);
        delayMicroseconds(50);
    }
Serial.printf("middle\n");
    delayMicroseconds(200);
    for (int i = 0; i < 200 * 64; i++)
    {
        RotStepper.Step(0);
        InoutStepper.Step(0);
        delayMicroseconds(50);
    }
    delayMicroseconds(80);
}
