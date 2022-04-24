#include <Arduino.h>
#include <Wire.h>

#define SDA2      17
#define SCL2      18
uint32_t I2C2_FREQ = 400000;


instantiate second Wire instance
TwoWire Wire1 = TwoWire(1);




void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Wire1.begin(SDA2, SCL2, I2C2_FREQ);
}

void loop() {
  // put your main code here, to run repeatedly:
}