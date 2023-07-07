#include <Arduino.h>
#include "defines.h"
#include "Wire.h"
#include "BQ25792.h"
#include "LIS2MDL.h"
#include "pid.h"
#include "filter.h"
#include "utils.h"
#include "motorDriver.h"

// running average filter
#define AVG_LENGTH 50
float average_buf[AVG_LENGTH];
int8_t avg_index = 0;
float average(float input);

float deadspace = 0;
PIDController anglePID(0.8, 0, 0, 2, 20);

long printTimer = 0;
typedef struct
{
  bool Set;
  bool Function;
  bool m1;
  bool p1;
  bool m15;
  bool p15;
} inputButtons;
inputButtons buttonPressed, lastButtonPressed;

enum Mode
{
  AUTOPILOT_OFF,
  AUTOPILOT_ON
} AutopilotMode;

// global variables
float targetHeading = 0;
int nowTime;
int dummy = 0;
uint8_t channel = 13;

// Connected Devices
BQ25792 charger(0, 0);
LIS2MDL compass;
motorManager motor;

void setMotor(int vref);
void testMotor();
float average(float input);
void updateButtons();
void task();
float getCompassError(float targetHeading, float currentHeading);

void IRAM_ATTR ISR()
{
  motor.handleInturrupt();
}

void setup()
{

  DEBUG_BEGIN(115200);

  Wire.begin(SDA_PIN, SCL_PIN);

  // set pinModes
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

  // start Charger
  charger.reset();
  delay(500); // give the charger time to reboot
  charger.flashChargeLevel(STAT_LED);

  if (compass.begin())
  {
    DEBUG_PRINTLN("Compass Found");
  }
  else
  {
    DEBUG_PRINTLN("Failed to find sensor");
    while (1)
    {
    };
  }

  attachInterrupt(ENC_A, ISR, FALLING);

  motor.begin();
}

void loop()
{
  nowTime = millis();

  updateButtons(); // update the state of the buttons
  task();
}

void task()
{

  if (buttonPressed.Set && !lastButtonPressed.Set)
  {
    if (!buttonPressed.Function)
    {
      if (AutopilotMode == AUTOPILOT_ON)
      {
        AutopilotMode = AUTOPILOT_OFF;
        DEBUG_PRINTLN("AutoPilot Off");
      }
      else if (AutopilotMode == AUTOPILOT_OFF)
      {
        AutopilotMode = AUTOPILOT_ON;
        targetHeading = compass.getHeading();
        DEBUG_PRINTF("Target heading: %.1f\t", targetHeading);
        DEBUG_PRINTLN("AutoPilot On");
      }
    }
    else
    {
      DEBUG_PRINTLN("Start Calibrating");
      DEBUG_PRINTF("Old Values %.1d%5.1d\n", compass.readXOffset(), compass.readYOffset());
      compass.calibrate();
      DEBUG_PRINTF("New Values %.1d%5.1d\n", compass.readXOffset(), compass.readYOffset());
      DEBUG_PRINTLN("Done Calibrating");
    }
  }
  else if (buttonPressed.p1 && !lastButtonPressed.p1)
  {
    if (!buttonPressed.Function)
    {
      targetHeading++;
      DEBUG_PRINT("New Heading: ");
      DEBUG_PRINTLN(targetHeading);
    }
    else
    {
      anglePID.output_ramp++;
      DEBUG_PRINT("New ramp: ");
      DEBUG_PRINTLN(anglePID.output_ramp);
    }
  }
  else if (buttonPressed.p15 && !lastButtonPressed.p15)
  {
    if (!buttonPressed.Function)
    {
      targetHeading += 15;
      DEBUG_PRINT("New Heading: ");
      DEBUG_PRINTLN(targetHeading);
    }
    else
    {
      anglePID.P += .1;
      DEBUG_PRINT("New P: ");
      DEBUG_PRINTLN(anglePID.P);
    }
  }
  else if (buttonPressed.m1 && !lastButtonPressed.m1)
  {
    if (!buttonPressed.Function)
    {
      targetHeading--;
      DEBUG_PRINT("New Heading: ");
      DEBUG_PRINTLN(targetHeading);
    }
    else
    {
      anglePID.output_ramp--;
      if (deadspace < 0)
        deadspace = 0;
      DEBUG_PRINT("New ramp: ");
      DEBUG_PRINTLN(anglePID.output_ramp);
    }
  }
  else if (buttonPressed.m15 && !lastButtonPressed.m15)
  {
    if (!buttonPressed.Function)
    {
      targetHeading -= 15;
      DEBUG_PRINT("New Heading: ");
      DEBUG_PRINTLN(targetHeading);
    }
    else
    {
      anglePID.P -= .1;
      DEBUG_PRINT("New P: ");
      DEBUG_PRINTLN(anglePID.P);
    }
  }

  if (AutopilotMode == AUTOPILOT_ON)
  {
    float tillerHeading = average(compass.getHeading());
    float tillerAngleToHull = motor.getTillerAngleToHull();
    float hullHeading = tillerHeading + tillerAngleToHull;
    float Error = getCompassError(targetHeading, hullHeading);
    float targetAngle = anglePID(Error);
    motor.setTargetTillerAngle(targetAngle);
    motor.task();
    // if (everyXms(&printTimer, 500))
    ///{
    // DEBUG_PRINTF("Tiller Heading %.1f TillerAngleToHull %.1f  Hull Heading %.1f  Error %.1f  TargetTillerAngle %.1f  MotorTorque %.1f\n", tillerHeading, tillerAngleToHull, hullHeading, Error, targetAngle, motor.motorTorque);
    DEBUG_PRINTF("%8.1f%8.1f%8.1f%8.1f%8.1f%8.1f\n", tillerHeading, tillerAngleToHull, hullHeading, Error, targetAngle, motor.motorTorque);
    //}
  }
  else
  {
    motor.setMotor(0);
    if (buttonPressed.Function)
    {
      DEBUG_PRINTLN(average(compass.getHeading()));
    }
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

float average(float input)
{

  avg_index++;
  avg_index = avg_index % AVG_LENGTH;
  average_buf[avg_index] = input;

  float min = 360;
  float max = 0;
  float sum = 0;
  for (float val : average_buf)
  {
    if (val < min)
      min = val;
    if (val > max)
      max = val;
    sum += val;
  }

  // DEBUG_PRINTF("Min: %f  Max: %f\n", min, max);
  return sum / AVG_LENGTH;
}
