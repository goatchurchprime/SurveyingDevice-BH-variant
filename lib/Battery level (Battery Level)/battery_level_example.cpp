// main code: ArduinoCode.ino
#include "Battery_level.h"

Battery_level battery_level; // create a OLED object

void setup() {
}

void loop() {
battery_level.battery_level_percent();
delay(1000);
}