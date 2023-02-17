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
BleGamepadConfiguration bleGamepadConfig;
BleGamepad bleGamepad("BrendansThingie");

float quat[4]; 
float prevquat[4]; 

float threshold = 0.1;
float diff;
float distance = 0;
bool lidarenabled = false; 

/*quat operator*(const quat& Q1, const quat& Q2) {
   return quat( 
       Q2.x*Q1.w + Q2.y*Q1.z - Q2.z*Q1.y + Q2.w*Q1.x,
      -Q2.x*Q1.z + Q2.y*Q1.w + Q2.z*Q1.x + Q2.w*Q1.y,
       Q2.x*Q1.y - Q2.y*Q1.x + Q2.z*Q1.w + Q2.w*Q1.z,
      -Q2.x*Q1.x - Q2.y*Q1.y - Q2.z*Q1.z + Q2.w*Q1.w
   );
}*/


void setup() {
  Serial.begin(9600);
  delay(100); 
  oled.Initialise();
  bno085.Initialise();
  oled.Distance(distance);
  lidar.init();
  lidarenabled = lidar.toggle_laser();

  bleGamepadConfig.setAutoReport(false); 
  bleGamepad.begin(&bleGamepadConfig);
  pinMode(27, INPUT_PULLUP);  
  pinMode(32, INPUT_PULLUP);  
}

int scaleaxis(float v) {
  int s = (int)((v + 1.0F)*0.5F*(bleGamepadConfig.getAxesMax() - bleGamepadConfig.getAxesMin())) + bleGamepadConfig.getAxesMin(); 
  return (s < bleGamepadConfig.getAxesMin() ? bleGamepadConfig.getAxesMin() : (s > bleGamepadConfig.getAxesMax() ? bleGamepadConfig.getAxesMax() : s)); 
}


int count = 0; 
int trec = -1; 
float compass = 0.0F;
float clino = 0.0F;
float laserlength = 0.0F; 
int batt_percentage = 0; 
int sensor_status = 0; 
bool bbluetooth = false;
float prevlaserlength = 0.0F; 

int millicountup = 0;
int millicountstate = -1; 
int prevmillis = 0; 
bool prevbutton27 = false; 
bool prevbutton32 = false; 
void loop() {
  int nextmillis = millis(); 
  millicountup += nextmillis - prevmillis;
  prevmillis = nextmillis; 
  int prevmillicountstate = millicountstate; 
  if (((millicountstate == 0) || (millicountstate == 4))&& (millicountup >= 0)) {
      bleGamepad.press(millicountstate == 0 ? BUTTON_5 : BUTTON_6);
      bleGamepad.setAxes(scaleaxis(quat[0]), scaleaxis(quat[1]), scaleaxis(quat[2]), scaleaxis(quat[3]), 
                         scaleaxis(laserlength/20.0F-1.0F)); 
      bleGamepad.sendReport();
      millicountstate += 1; 
      millicountup = 0; 
  } else if (((millicountstate == 1) || (millicountstate == 5)) && (millicountup > 10)) {
      bleGamepad.release(millicountstate == 1 ? BUTTON_5 : BUTTON_6);
      bleGamepad.sendReport();
      millicountstate = -1; 
      millicountup = 0; 
  } else if ((millicountstate == 2) && (millicountup >= 0)) {
      bleGamepad.sendReport();
      millicountstate = -1; 
      millicountup = 0; 
  } else if ((millicountstate == -1) && (millicountup > 1000)) {
      millicountstate = 0; 
      millicountup = 0; 
  }
  if (prevmillicountstate != millicountstate) {
    Serial.print("millicount state: "); 
    Serial.println(millicountstate);
    prevmillicountstate = millicountstate; 
  } 
  bool button27 = digitalRead(GPIO_NUM_27);
  bool button32 = digitalRead(GPIO_NUM_32);
  if ((button27 != prevbutton27) || (button32 != prevbutton32)) {
    Serial.print("button 27 "); 
    Serial.print(button27); 
    Serial.print("  button 32 "); 
    Serial.println(button32); 
    (prevbutton27 ? bleGamepad.press(BUTTON_1) : bleGamepad.release(BUTTON_1));
    (prevbutton32 ? bleGamepad.press(BUTTON_2) : bleGamepad.release(BUTTON_2));
    if (button32 && !prevbutton32) {
      lidarenabled = lidar.toggle_laser();
      Serial.println(lidarenabled ? "lidar enabled" : "lidar disabled"); 
    }
    if (button27 && !prevbutton27) {
      laserlength = (float)lidar.get_measurement();
      Serial.print("laser length ")
      Serial.println(laserlength); 
      millicountstate = 4; 
      millicountup = 0; 
    } 
    prevbutton27 = button27; 
    prevbutton32 = button32; 
    if (millicountstate == -1) {
      millicountstate = 2; 
      millicountup = 0; 
    }
  }

  bool bquat = bno085.Quat(quat); 
  bool borientchange = ((abs(compass - bno085.compass) > 0.5) || (abs(clino - bno085.clino) > 0.5));
  if (borientchange) {
    oled.Compass(compass);
    compass = bno085.compass; 
    oled.Clino(-clino);
    clino = bno085.clino; 
  }
  int lbatt_percentage = battery_level.battery_level_percent();
  int lsensor_status = bno085.sensor_cal_status();
  bool lbbluetooth = bleGamepad.isConnected(); 
  bool botherchange = ((abs(batt_percentage - lbatt_percentage) > 5) || (lsensor_status != sensor_status) || 
                       (lbbluetooth != bbluetooth) || (prevlaserlength != laserlength)); 
  if (botherchange) {
    oled.Battery(batt_percentage);
    batt_percentage = lbatt_percentage; 
    oled.Sensor_cal_status(sensor_status);
    sensor_status = lsensor_status; 
    oled.Blutooth(bbluetooth);
    bbluetooth = lbbluetooth; 
    oled.Distance(laserlength);
    prevlaserlength = laserlength; 
  }
  if (borientchange || botherchange)
    oled.Display(); 

  if (bbluetooth && borientchange && bquat)
  {
      if (millicountstate == -1) {
        millicountstate = 0; 
        millicountup = 0; 
      }
  }
}

