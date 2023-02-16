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
float quat[4]; 

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
int trec = -1; 
void loop() {
  int batt_percentage = battery_level.battery_level_percent();
  if ((trec != -1) && (millis() - trec > 10)) {
    bleGamepad.release(BUTTON_5);
    trec = -1; 
  }

  float compass = bno085.Compass();
  float clino = bno085.Clino();
  bool bquat = bno085.Quat(quat); 

  combined_value = compass + clino;
  diff = combined_value - new_combined_value;
  
  if (abs(diff)<threshold) 
    return;  

  oled.Compass(compass+1000);
  oled.Clino(-clino);
  new_combined_value = compass + clino;
  oled.Battery(batt_percentage);
  int sensor_status = bno085.sensor_cal_status();
  oled.Sensor_cal_status(sensor_status);

  if (bleGamepad.isConnected())
  {
      oled.Blutooth(true);
      if (bquat) {
        bleGamepad.setAxes((int)((quat[0]+1)*16000), 
                           (int)((quat[1]+1)*16000), 
                           (int)((quat[2]+1)*16000), 
                           (int)((quat[3]+1)*16000), 
                           0, count++, 0, 0);

        trec = millis(); 
        Serial.print(count); 
        Serial.print(" "); 
        Serial.println(trec); 
        bleGamepad.press(BUTTON_5);
      }
//    void setSlider(int16_t slider = 0);
  }
  else 
      oled.Blutooth(false);
}

