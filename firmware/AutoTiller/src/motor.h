#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class motor
{
    int int1, int2, vref;

public:
    motor(int _int1, int _int2, int _vref) : int1(_int1), int2(_int2), vref(_vref)
    {
     
    }
    void begin();
    void run(int v);
    void task();
};

#endif