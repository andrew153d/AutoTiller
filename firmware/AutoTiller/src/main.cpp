#include <Arduino.h>
#include <Wire.h>
#define LED 15
#define WIRE Wire

void setup() {
  Wire.setClock(25000);
  Wire.begin(32, 33);  // Initialize Wire library with I2C pins
  Serial.begin(115200);
  while (!Serial); // Wait for serial connection
  pinMode(22, OUTPUT);
  digitalWrite(22, LOW);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  Serial.println("I2C Scanner");
}

void loop() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    WIRE.beginTransmission(address);
    error = WIRE.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  for(int i = 0; i<nDevices; i++){
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
    delay(100);
  }
  delay(5000); // Wait for 5 seconds before scanning again
}
