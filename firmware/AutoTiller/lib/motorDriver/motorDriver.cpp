#include "motorDriver.h"

void motorManager::begin()
{
  shaftAngle = 0;
  Direction = RIGHT;
  // channel = 13;
  // Serial.println(ledcSetup(channel, 4000, 8) == ESP_OK);
  // ledcAttachPin(MOTOR_VREF, channel);
  ledcSetup(LEDC_CHANNEL, 4000, 8);
  ledcAttachPin(MOTOR_VREF, LEDC_CHANNEL);
}

motorManager::motorManager()
{
}

void motorManager::setMotor(int vref)
{

  if (vref < 0)
  {
    Direction = RIGHT;
    digitalWrite(MOTOR_DIR_1, HIGH);
    digitalWrite(MOTOR_DIR_2, LOW);
  }
  else if (vref > 0)
  {
    Direction = LEFT;
    digitalWrite(MOTOR_DIR_1, LOW);
    digitalWrite(MOTOR_DIR_2, HIGH);
  }
  else
  {
    digitalWrite(MOTOR_DIR_1, HIGH);
    digitalWrite(MOTOR_DIR_2, HIGH);
  }
  // analogWrite(MOTOR_VREF, abs(vref));
  ledcWrite(LEDC_CHANNEL, abs(vref));
}

void motorManager::handleInturrupt()
{
  if (Direction == RIGHT)
  {
    ticks--;
  }
  else
  {
    ticks++;
  }
}

double motorManager::getShaftAngle(){
  return (double)(ticks)*TICKS_TO_DEGREES;
}

float motorManager::getPosition(){
  return 2*M_PI*RADIUS_CM*(getShaftAngle()/360);
}

float motorManager::getTillerAngleToHull(){
  return (getPosition()*180)/(M_PI*TILLER_LENGTH_CM);
}

void motorManager::setTargetTillerAngle(float target){
  targetAngle = target;   
}

void motorManager::task()
{
  motorTorque = torquePID(targetAngle - getTillerAngleToHull());
  
  setMotor(motorTorque);
}

