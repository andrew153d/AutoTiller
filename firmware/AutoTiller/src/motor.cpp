#include "motor.h"

void motor::run(int v)
{
    if (v < 0)
    {
        digitalWrite(int1, HIGH);
        digitalWrite(int2, LOW);
    }
    else if (v > 0)
    {
        digitalWrite(int1, LOW);
        digitalWrite(int2, HIGH);
    }
    else
    {
        digitalWrite(int1, HIGH);
        digitalWrite(int2, HIGH);
    }

    analogWrite(vref, v);
}

void motor::begin()
{
    pinMode(int1, OUTPUT);
    pinMode(int2, OUTPUT);
    pinMode(vref, OUTPUT);
}

