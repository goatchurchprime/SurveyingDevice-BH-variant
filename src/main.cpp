// main code: ArduinoCode.ino
#include "OLED.h"
#include "BNO085.h"
#include "lidar.h"
#include "Battery_level.h"
#include <BleGamepad.h>


OLED oled; // create a OLED object
BNO085 bno085;
Lidar lidar;
Battery_level battery_level;
BleGamepad bleGamepad("BrendansThingie");

float new_combined_value;
float combined_value;
float threshold = 0.1;
float diff;
float distance = 0;
int sensor_status;

void setup() {
  Serial.begin(9600);
  oled.Initialise();
  bno085.Initialise();
  oled.Distance(distance);
  lidar.init();
  bleGamepad.begin();
}

int count = 0; 
void loop() {
  bool ble_status = random(0,100);
  int batt_percentage = battery_level.battery_level_percent();

  float compass = bno085.Compass();
  float clino = bno085.Clino();

  combined_value = compass + clino;
  diff = combined_value - new_combined_value;
  
  if (diff>threshold||diff<-threshold)  
  {  
  oled.Compass(compass);
  oled.Clino(-clino);
  float compass = bno085.Compass();
  float clino = bno085.Clino();
  new_combined_value = compass + clino;
  oled.Blutooth(ble_status);
  oled.Battery(batt_percentage);
  int sensor_status = bno085.sensor_cal_status();
  oled.Sensor_cal_status(sensor_status);

  if (bleGamepad.isConnected())
  {
      Serial.println("Press buttons 5, 16 and start. Move all enabled axes to max. Set DPAD (hat 1) to down right.");
      bleGamepad.press(BUTTON_5);
      bleGamepad.setX((int)compass*10); 
      bleGamepad.setY((int)(clino+90)*10); 
      bleGamepad.setZ(count++); 
//    void setRZ(int16_t rZ = 0);
//    void setSlider(int16_t slider = 0);
  }
  }
}

