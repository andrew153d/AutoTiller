#include <Arduino.h>
#include "defines.h"
#include "UDPHandler.h"
#include "Wire.h"
#include "BQ25792.h"
#include "LIS2MDL.h"
#include "pid.h"
#include "filter.h"
#include "utils.h"
#include "motorDriver.h"

//running average filter
#define AVG_LENGTH 20
float average_buf[AVG_LENGTH];
int8_t avg_index = 0;
float average(float input);



float deadspace = 10;
PIDController torquePID(1, 0, 0, 10000, 255);
PIDController anglePID(1, 0, 0, 100, 20);

long printTimer = 0;
typedef struct
{
  bool Set;
  bool Function;
  bool m1;
  bool p1;
  bool m15;
  bool p15;
}inputButtons;
inputButtons buttonPressed, lastButtonPressed;

enum Mode
{
  AUTOPILOT_OFF,
  AUTOPILOT_ON
} AutopilotMode;

//global variables
float targetHeading = 0;
int nowTime;
int dummy = 0;

//Connected Devices
BQ25792 charger(0, 0);
//motorDriver motor;
LIS2MDL compass;

void setMotor(int vref);
void testMotor();
float average(float input);
void updateButtons();
void task();
float getCompassError(float targetHeading, float currentHeading);

void IRAM_ATTR ISR()
{
  //motor.handleInturrupt();
}

void setup()
{
  DEBUG.begin(115200);
  
  Wire.begin(SDA_PIN, SCL_PIN);

  //set pinModes
  pinMode(STAT_LED, OUTPUT);
  pinMode(FUNCTION, INPUT);
  pinMode(SET, INPUT);
  pinMode(PIN_m15, INPUT);
  pinMode(PIN_m1, INPUT);
  pinMode(PIN_p15, INPUT);
  pinMode(PIN_p1, INPUT);
  pinMode(MOTOR_DIR_2, OUTPUT);
  pinMode(MOTOR_DIR_1, OUTPUT);
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  pinMode(COMPASSDATAREADY, INPUT);
  pinMode(9, INPUT_PULLUP);

  motor.begin();

  charger.reset();
  delay(500); //give the charger time to reboot

  DEBUG.print("Battery Voltage: ");
  DEBUG.println(charger.getVBAT());
  DEBUG.print("Maximum Voltage: ");
  DEBUG.println(charger.getChargeVoltageLimit());

  if (compass.begin())
  {
    Serial.println("Compass Found");
  }
  else
  {
    Serial.println("Failed to find sensor");
    while(1){};
  }

  attachInterrupt(ENC_A, ISR, FALLING);
  
}

void loop()
{
  nowTime = millis();
  
  updateButtons(); //update the state of the buttons
  
  if (everyXms(&printTimer, 10))
  {
    Serial.printf("Heading: %f  targetAngle: %d\n", compass.getHeading(), motor.shaftAngle);
  }
}

void task(){

  if (buttonPressed.Set && !lastButtonPressed.Set)
  {
    if (!buttonPressed.Function)
    {
      if (AutopilotMode == AUTOPILOT_ON)
      {
        AutopilotMode = AUTOPILOT_OFF;
        DEBUG.println("AutoPilot Off");
      }
      else if (AutopilotMode == AUTOPILOT_OFF)
      {
        AutopilotMode = AUTOPILOT_ON;
        targetHeading = compass.getHeading();
        DEBUG.println("AutoPilot On");
      }
    }
    else
    {
      DEBUG.println("Start Calibrating");
      compass.calibrate();
      DEBUG.println("Done Calibrating");
    }
  }
  else if (buttonPressed.p1 && !lastButtonPressed.p1)
  {
    if (!buttonPressed.Function)
    {
      targetHeading++;
      DEBUG.print("New Heading: ");
      DEBUG.println(targetHeading);
    }
    else
    {
      deadspace++;
      DEBUG.print("New deadspace: ");
      DEBUG.println(deadspace);
    }
  }
  else if (buttonPressed.p15 && !lastButtonPressed.p15)
  {
    if (!buttonPressed.Function)
    {
      targetHeading += 15;
      DEBUG.print("New Heading: ");
      DEBUG.println(targetHeading);
    }
    else
    {
      torquePID.P++;
      DEBUG.print("New P: ");
      DEBUG.println(torquePID.P);
    }
  }
  else if (buttonPressed.m1 && !lastButtonPressed.m1)
  {
    if (!buttonPressed.Function)
    {
      targetHeading--;
      DEBUG.print("New Heading: ");
      DEBUG.println(targetHeading);
    }
    else
    {
      deadspace--;
      if (deadspace < 0)
        deadspace = 0;
      DEBUG.print("New deadspace: ");
      DEBUG.println(deadspace);
    }
  }
  else if (buttonPressed.m15 && !lastButtonPressed.m15)
  {
    if (!buttonPressed.Function)
    {
      targetHeading -= 15;
      DEBUG.print("New Heading: ");
      DEBUG.println(targetHeading);
    }
    else
    {
      torquePID.P--;
      DEBUG.print("New P: ");
      DEBUG.println(torquePID.P);
    }
  }

  if (AutopilotMode == AUTOPILOT_ON)
  { 
    float tillerHeading = average(compass.getHeading());
    float tillerAngleToHull = motor.getAngle();

    float Error = getCompassError(targetHeading, tillerHeading);
    float motorAngle = anglePID(Error);
    motor.setAngle(motorAngle);
    motor.setMotor(50);
  
  }
  else
  {
    motor.setMotor(0);
  }
}

void updateButtons()
{
  lastButtonPressed = buttonPressed;
  buttonPressed.Function = !digitalRead(FUNCTION);
  buttonPressed.Set = !digitalRead(SET);
  buttonPressed.m15 = !digitalRead(PIN_m15);
  buttonPressed.m1 = !digitalRead(PIN_m1);
  buttonPressed.p1 = !digitalRead(PIN_p1);
  buttonPressed.p15 = !digitalRead(PIN_p15);
  digitalWrite(STAT_LED, (buttonPressed.Function || buttonPressed.m15 || buttonPressed.Set || buttonPressed.p15 || buttonPressed.m1 || buttonPressed.p1));
}

float getCompassError(float targetHeading, float currentHeading)
{
  float error = targetHeading - currentHeading;

  if (error > 180.0)
  {
    error -= 360.0;
  }
  else if (error < -180.0)
  {
    error += 360.0;
  }

  return ((abs(error) < deadspace) ? 0 : error);
}

float average(float input){
  
  avg_index++;
  avg_index = avg_index%AVG_LENGTH;
  average_buf[avg_index] = input;

  float min = 360;
  float max = 0;
  float sum = 0;
  for(float val: average_buf){
    if(val<min)
      min = val;
    if(val>max)
      max=val;
    sum+=val;
  }

 //Serial.printf("Min: %f  Max: %f\n", min, max);
  return sum/AVG_LENGTH;
}
