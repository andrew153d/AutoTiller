#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <Arduino.h>
#include "../../include/defines.h"
#include "pid.h"

#define PULSES_PER_REV 540;
#define RADIUS 0.5
#define LEDC_CHANNEL 13

class motorDriver
{
public:
    PIDController torquePID = PIDController(1, 0, 0, 100, 20);
    float shaftAngle, position, targetAngle;
    int dir1, dir2, vref, encA, encB;
    long ticks;
    enum direction{
        RIGHT,
        LEFT
    } Direction;

    motorDriver();
    
    void begin();
    void setMotor(int vref);
    void handleInturrupt();
    void setAngle(float angle);
    float getAngle();
    void task();
};

#endif