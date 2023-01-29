// main code: ArduinoCode.ino
#include "OLED.h"
#include "BNO085.h"

OLED oled; // create a OLED object
BNO085 bno085;

void setup() {
  Serial.begin(9600);
  oled.Initialise();
  bno085.Initialise();
}

void loop() {

  double distance = 0;
  bool ble_status = random(0,100);
  int batt_percentage = random(0,100);

  float compass = bno085.Compass();
  float clino = bno085.Clino();

  oled.Distance(distance);
  oled.Compass(compass);
  oled.Clino(clino);
  oled.Blutooth(ble_status);
  oled.Battery(batt_percentage);
  delay(50);

}