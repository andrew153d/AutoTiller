#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "UDPHandler.h"
#include "Wire.h"
#include "BQ25792.h"
#include "LIS2MDL.h"
#include "pid.h"
#include "filter.h"

// PIN definitions
#define IMU_ADDR_BIT7_PIN 22
#define SDA_PIN 32
#define SCL_PIN 33
#define FUNCTION 35
#define PIN_m15 37
#define PIN_m1 38
#define PIN_p1 39
#define PIN_p15 34
#define SET 36
#define STAT_LED 15
#define MOTOR_DIR_1 18
#define MOTOR_DIR_2 23
#define MOTOR_VREF 5
#define COMPASSDATAREADY 25

void setMotor(int vref);
void testMotor();
void updateButtons();
float getCompassError(float targetHeading, float currentHeading);
// #define DO_OTA
// #define CALIBRATE_COMPASS
float deadspace = 10;
PIDController torquePID(0, 0, 0, 10000, 255);
float torquePID_P = 0;
float torquePID_I = 0;
float torquePID_D = 0;

struct buttons
{
  bool Set;
  bool Function;
  bool m1;
  bool p1;
  bool m15;
  bool p15;
} buttonPressed, lastButtonPressed;

enum Mode
{
  AUTOPILOT_OFF,
  AUTOPILOT_ON
} AutopilotMode;

float targetHeading = 0;

#define DEBUG Serial

#ifdef DO_OTA
const char *ssid
const char *password
#endif

BQ25792 charger(0, 0);
filter smooth(20);
int nowTime = 0;

long lastOffTime;

LIS2MDL compass;
float X = -1, Y = -1, Z = -1;
float MagMinX = -41.1;
float MagMaxX = -9.45;
float MagMinY = -60.45;
float MagMaxY = -31.20;
float target = 0;

void setup()
{
  DEBUG.begin(115200);
  ledcAttachPin(MOTOR_VREF, 0);
  ledcSetup(0, 1000, 8);

  Wire.begin(SDA_PIN, SCL_PIN);

  lastOffTime = millis();

  pinMode(STAT_LED, OUTPUT);
  pinMode(FUNCTION, INPUT);
  pinMode(SET, INPUT);
  pinMode(PIN_m15, INPUT);
  pinMode(PIN_m1, INPUT);
  pinMode(PIN_p15, INPUT);
  pinMode(PIN_p1, INPUT);
  pinMode(MOTOR_DIR_2, OUTPUT);
  pinMode(MOTOR_DIR_1, OUTPUT);
  // pinMode(MOTOR_VREF, OUTPUT);
  pinMode(COMPASSDATAREADY, INPUT);

  testMotor();
  digitalWrite(STAT_LED, LOW);

#ifdef DO_OTA
  delay(2000);
  WiFi.mode(WIFI_STA);
  delay(200);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    DEBUG.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  ArduinoOTA.setHostname("AutoTiller");

  ArduinoOTA
      .onStart([]()
               {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      DEBUG.println("Start updating " + type); })
      .onEnd([]()
             { DEBUG.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total) { // DEBUG.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error)
               {
      //DEBUG.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) DEBUG.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) DEBUG.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) DEBUG.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) DEBUG.println("Receive Failed");
      else if (error == OTA_END_ERROR) DEBUG.println("End Failed"); });

  ArduinoOTA.begin();

  DEBUG.println("Ready");
  DEBUG.print("IP address: ");
  // DEBUG.println(WiFi.localIP());

#endif

  charger.reset();
  delay(100);
  DEBUG.print("Battery Voltage: ");
  DEBUG.println(charger.getVBAT());
  delay(100);
  DEBUG.print("Maximum Voltage: ");
  DEBUG.println(charger.getChargeVoltageLimit());

  if (compass.begin())
  {
    Serial.println("Compass Found");
  }
  else
  {
    Serial.println("Failed to find sensor");
    delay(1000000);
  }
  compass.setODR(ODR::ODR_100_Hz);
  AutopilotMode = AUTOPILOT_OFF;
  torquePID.P = 2;
  torquePID.limit = 255;
}

void loop()
{

#ifdef DO_OTA
  ArduinoOTA.handle();
#endif
  nowTime = millis();

  updateButtons();

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
      DEBUG.print("New delay: ");
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
      DEBUG.print("New delay: ");
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
    float heading = compass.getHeading();
    smooth.input(heading);
    float smoothHeading = smooth.getValue();
    float Error = getCompassError(targetHeading, smoothHeading);
    float motorVoltage = torquePID(Error);
    setMotor((int)motorVoltage);
    
    Serial.print("Heading: ");
    Serial.print(heading);
    Serial.print(" Heading: ");
    Serial.print(smoothHeading);
    Serial.print(" error: ");
    Serial.print(Error);
    Serial.print(" Voltage: ");
    Serial.println(motorVoltage);
  }
  else
  {
    setMotor(0);
  }
}

void setMotor(int vref)
{

  if (vref < 0)
  {
    digitalWrite(MOTOR_DIR_1, HIGH);
    digitalWrite(MOTOR_DIR_2, LOW);
  }
  else if (vref > 0)
  {
    digitalWrite(MOTOR_DIR_1, LOW);
    digitalWrite(MOTOR_DIR_2, HIGH);
  }
  else
  {
    digitalWrite(MOTOR_DIR_1, HIGH);
    digitalWrite(MOTOR_DIR_2, HIGH);
  }
  // analogWrite(MOTOR_VREF, abs(vref));
  ledcWrite(0, abs(vref));
}

void testMotor()
{
  for (int i = 0; i < 255; i++)
  {
    setMotor(i);
    delay(1);
  }
  for (int i = 255; i > 0; i--)
  {
    setMotor(i);
    delay(1);
  }
};

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
  
  return ((abs(error)<deadspace)?0:error);
}
