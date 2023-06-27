#include "motorDriver.h"

void motorDriver::begin()
{
  shaftAngle = 0;
  Direction = RIGHT;
  // channel = 13;
  // Serial.println(ledcSetup(channel, 4000, 8) == ESP_OK);
  // ledcAttachPin(MOTOR_VREF, channel);
  ledcSetup(LEDC_CHANNEL, 4000, 8);
  ledcAttachPin(MOTOR_VREF, LEDC_CHANNEL);
}

motorDriver::motorDriver()
{
}

void motorDriver::setMotor(int vref)
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

void motorDriver::handleInturrupt()
{
  if (Direction == RIGHT)
  {
    ticks++;
  }
  else
  {
    ticks--;
  }
}

double motorDriver::getShaftAngle(){
  return (double)(ticks)*TICKS_TO_DEGREES;
}

void motorDriver::setAngle(float angle)
{
  targetAngle = angle;
}

void motorDriver::task()
{
  // Serial.println(targetAngle);
}

float motorDriver::getAngle()
{

  return 0;
}