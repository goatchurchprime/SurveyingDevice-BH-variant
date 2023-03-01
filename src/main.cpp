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

int BUTTON_MARKAXESVALUES = BUTTON_14; 
int BUTTON_MARKLAXESVALUES = BUTTON_15; 

int count = 0; 
int trec = -1; 
float compass = 0.0F;
float clino = 0.0F;
float laserlength = 0.0F; 
int batt_percentage = 0; 
int sensor_status = 0; 
bool bbluetooth = false;
float prevlaserlength = 0.0F; 

enum MCS {
   MCS_WAITING = -1,
   MCS_SENDAXESVALUES = 0,
   MCS_MARKAXESVALUES = 1,
   MCS_SENDBEAT = 2,
   MCS_SENDLAXESVALUES = 4,
   MCS_MARKLAXESVALUES = 5,
};

int millicountup = 0;
MCS millicountstate = MCS_WAITING; 
int prevmillis = 0; 
bool prevbutton27 = false; 
bool prevbutton32 = false; 
void loop() {
  int nextmillis = millis(); 
  millicountup += nextmillis - prevmillis;
  prevmillis = nextmillis; 
  MCS prevmillicountstate = millicountstate; 
  if (((millicountstate == MCS_SENDAXESVALUES) || (millicountstate == MCS_SENDLAXESVALUES))&& (millicountup >= 0)) {
      bleGamepad.press(millicountstate == MCS_SENDAXESVALUES ? BUTTON_MARKAXESVALUES : BUTTON_MARKLAXESVALUES);
      bleGamepad.setAxes(scaleaxis(quat[0]), scaleaxis(quat[1]), scaleaxis(quat[2]), scaleaxis(quat[3]), 
                         scaleaxis(laserlength/20.0F-1.0F)); 
      bleGamepad.sendReport();
      millicountstate = (millicountstate == MCS_SENDAXESVALUES ? MCS_MARKAXESVALUES : MCS_MARKLAXESVALUES); 
      millicountup = 0; 
  } else if (((millicountstate == MCS_MARKAXESVALUES) || (millicountstate == MCS_MARKLAXESVALUES)) && (millicountup > 10)) {
      bleGamepad.release(millicountstate == MCS_MARKAXESVALUES ? BUTTON_MARKAXESVALUES : BUTTON_MARKLAXESVALUES);
      bleGamepad.sendReport();
      millicountstate = MCS_WAITING; 
      millicountup = 0; 
  } else if ((millicountstate == MCS_SENDBEAT) && (millicountup >= 0)) {
      bleGamepad.sendReport();
      millicountstate = MCS_WAITING; 
      millicountup = 0; 
  } else if ((millicountstate == MCS_WAITING) && (millicountup > 1000)) {
      millicountstate = MCS_WAITING; 
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
    //(prevbutton27 ? bleGamepad.press(BUTTON_1) : bleGamepad.release(BUTTON_1));
    //(prevbutton32 ? bleGamepad.press(BUTTON_2) : bleGamepad.release(BUTTON_2));
    if (button32 && !prevbutton32) {
      lidarenabled = lidar.toggle_laser();
      Serial.println(lidarenabled ? "lidar enabled" : "lidar disabled"); 
    }
    if (button27 && !prevbutton27) {
      laserlength = (float)lidar.get_measurement();
      Serial.print("laser length ");
      Serial.println(laserlength); 
      millicountstate = MCS_SENDLAXESVALUES; 
      millicountup = 0; 
    } 
    prevbutton27 = button27; 
    prevbutton32 = button32; 
    if (millicountstate == MCS_WAITING) {
      millicountstate = MCS_SENDBEAT; 
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
      if (millicountstate == MCS_WAITING) {
        millicountstate = MCS_SENDAXESVALUES; 
        millicountup = 0; 
      }
  }
}



/*quat operator*(const quat& Q1, const quat& Q2) {
   return quat( 
       Q2.x*Q1.w + Q2.y*Q1.z - Q2.z*Q1.y + Q2.w*Q1.x,
      -Q2.x*Q1.z + Q2.y*Q1.w + Q2.z*Q1.x + Q2.w*Q1.y,
       Q2.x*Q1.y - Q2.y*Q1.x + Q2.z*Q1.w + Q2.w*Q1.z,
      -Q2.x*Q1.x - Q2.y*Q1.y - Q2.z*Q1.z + Q2.w*Q1.w
   );
}*/
/*r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
*/

