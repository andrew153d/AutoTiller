#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>

#define MOTORPIN 10

class motorControl{
    public:
    motorControl();
    void begin();
    void setVoltage(int value);
};



#endif