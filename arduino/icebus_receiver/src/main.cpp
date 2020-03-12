#include <Arduino.h>
#include "IcebusProtocol.hpp"

IcebusHost icebus;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.println("hello!\n");
}


uint8_t data[256];

void loop() {
  icebus.Listen(128);
}
