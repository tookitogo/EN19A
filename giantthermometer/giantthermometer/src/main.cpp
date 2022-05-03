#include <Arduino.h>
#include "avr8-stub.h"
#include "app_api.h" // only needed with flash breakpoints

void setup() {
  // put your setup code here, to run once:
    // initialize GDB stub
  debug_init();
}

void loop() {
  // put your main code here, to run repeatedly:
}