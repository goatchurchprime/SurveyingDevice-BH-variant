// main code: ArduinoCode.ino
#include "OLED.h"
#include "BNO085.h"
#include "lidar.h"

OLED oled; // create a OLED object
BNO085 bno085;
Lidar lidar;

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
}

void loop() {

  bool ble_status = random(0,100);
  int batt_percentage = random(0,100);

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
  }  


  
  int sensor_status = bno085.sensor_cal_status();
  oled.Sensor_cal_status(sensor_status);

}