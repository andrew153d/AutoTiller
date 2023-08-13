#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <Arduino.h>
#include "../../include/defines.h"
#include "pid.h"

#define PULSES_PER_REV 540;
#define RADIUS_CM 0.775 //CM
#define LEDC_CHANNEL 13
#define TICKS_TO_DEGREES 360/540
#define TILLER_LENGTH_CM 60

class motorManager
{
public:
    PIDController torquePID = PIDController(100, 0, 1, 10000, 255);
    float shaftAngle, position, targetAngle;
    int dir1, dir2, vref, encA, encB;
    long ticks;
    float motorTorque;
    enum direction{
        RIGHT,
        LEFT
    } Direction;

    motorManager();
    
    void begin();
    void setMotor(int vref);
    void handleInturrupt();

    double getShaftAngle();
    float getPosition();
    float getTillerAngleToHull();

    void setTargetTillerAngle(float target);
    void task();
};

#endif