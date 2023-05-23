#include "motorControl.h"

void motorControl::begin(){
  pinMode(MOTORPIN, OUTPUT);
  analogWrite(MOTORPIN, 0);
}

void motorControl::setVoltage(int value){
    analogWrite(MOTORPIN, value);
}

motorControl::motorControl(){

}