#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <M5Unified.h>
#include "gfx.h"  // Sketch tab header for xbm images


/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

int LCDbrightness=1;
uint32_t red_brightness_values[] = {
  0xff0000U,
  0xAA0000U,
  0x550000U,
  0x220000U
};

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) {
  Serial.begin(115200);

  auto cfg = M5.config();

  // If you want to use external IMU, write this
  cfg.external_imu = true;

  M5.begin(cfg);
  M5.Lcd.setBrightness(LCDbrightness);

  M5.Display.setTextColor(red_brightness_values[1],BLACK);



  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Raw Data Test");
  Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    M5.Display.print("Failed to find MPU6050 chip");
    while (1)
      ;
  }
//  M5.Display.println("MPU6050 Found!");


  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  /*
  https://github.com/bcmi-labs/brackets-arduino/issues/10
  ************************************************************
* Remap Axes and Signs to BNO055 User Guide position P0
* This swaps X/Y axes and sets the X axis negative by
* calling the "utility" BNO055 library functions directly.
*
*     value     |   definition      | comments
*   ------------|-------------------|------------
*      0X21     | REMAP_X_Y         | Z=Z;X=Y;Y=X
*      0X18     | REMAP_Y_Z         | X=X;Y=Z;Z=Y
*      0X06     | REMAP_Z_X         | Y=Y;X=Z;Z=X
*      0X12     | REMAP_X_Y_Z_TYPE0 | X=Z;Y=X;Z=Y
*      0X09     | REMAP_X_Y_Z_TYPE1 | X=Y;Y=Z;Z=X
*      0X24     | DEFAULT_AXIS      | X=X;Y=Y;Z=Z
*
/*************************************************************/

  //Axis values:
  //X->00
  //Y->01
  //Z->10

  byte X = 0b00;
  byte Y = 0b01;
  byte Z = 0b10;

  byte DEFAULT_AXES = Z<<4 + Y<<2 + X;

  //  Z  Y  X
  // 10 01 00 -> 0x24 default
//  byte DEFAULT_AXES = 0b100100;

  // X-Y swap:
  //  Z  X  Y
  // 10 00 01
  //  0  1  0
  byte DEFAULT_SIGNS = 0b000;
  

//  byte REMAPPED_AXES = 

  byte REMAPPED_AXES = X<<4 + Y << 2 + Z;
  byte REMAPPED_SIGNS = 0b100;
  
  bno.setMode(OPERATION_MODE_CONFIG);
  delay(25);
  bno.write8(bno.BNO055_AXIS_MAP_CONFIG_ADDR, REMAPPED_AXES );
  delay(10);
  bno.write8(bno.BNO055_AXIS_MAP_SIGN_ADDR, REMAPPED_SIGNS);  // P0-P7, Default is P1
  delay(10);
  bno.setMode(OPERATION_MODE_NDOF);
  delay(20);

//  bno.setAxisRemap(DEFAULT_AXES);
//  bno.setAxisSign(DEFAULT_SIGNS);


  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
auto lastbatt=0;
auto lastW=0;
auto lastH=0;
float EL=0;
float AZ=0;

int state = 0;
bool redraw = true;
unsigned long lastIMUcall = 0;
void loop(void) {


  M5.update();


  //M5.Display.clearDisplay();
  //  M5.Display.println("Calibration status values:");
  //M5.Display.println("0=uncalibrated, 3=fully calibrated");
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  //imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  if(millis()-lastIMUcall>BNO055_SAMPLERATE_DELAY_MS){
  imu::Quaternion q = bno.getQuat();
  q.normalize();
  imu::Vector<3> euler = q.toEuler();
  euler.x() *= -180 / M_PI;
  euler.y() *= -180 / M_PI;
  euler.z() *= -180 / M_PI;
  if (euler.x() < 0)
    euler.x() += 360;
  
  EL = 90-euler.z();
  AZ = euler.x();
  lastIMUcall=millis();
  }

  if(M5.BtnC.isHolding() || M5.BtnC.isPressed() )
  {
     //M5.Lcd.fillRect(250,M5.Display.height()-10,10,10,BLUE);

     M5.Lcd.fillTriangle(255,M5.Display.height(),250,M5.Display.height()-10,260,M5.Display.height()-10,RED);
    if(M5.BtnA.wasClicked()) LCDbrightness--;
    if(M5.BtnB.wasClicked()) LCDbrightness++;
    if(M5.BtnA.isHolding()) LCDbrightness--;
    if(M5.BtnB.isHolding()) LCDbrightness++;
    LCDbrightness=constrain(LCDbrightness,1,64);
    M5.Display.setBrightness(LCDbrightness);

    M5.Display.setTextSize(2);
    M5.Display.setCursor(245, M5.Display.height()-30);
    M5.Display.print(LCDbrightness);

    M5.Display.print("   ");
  }
  
  

  else{

    if(M5.BtnA.wasClicked()) state = 0;
    if(M5.BtnB.wasClicked()) state = 2;
    }

    if(M5.BtnC.wasReleased()){
    M5.Display.setTextSize(2);
    M5.Display.setCursor(245, M5.Display.height()-30);
    M5.Display.print("   ");
          //   M5.Lcd.fillRect(250,M5.Display.height()-10,10,10,BLACK);
          M5.Lcd.fillTriangle(255,M5.Display.height(),250,M5.Display.height()-10,260,M5.Display.height()-10,BLACK);
          redraw=true;

  }
  
  switch(state){
    case 0:
      M5.Lcd.fillScreen(BLACK);
      redraw=true;
      state=1;
      break;
    case 1:
      camTracker();
      redraw=false;
      break;
    case 2:
      drawCompassFace();
      redraw=true;
      state = 3;
      break;
    case 3:
      drawCompassNeedle();
      redraw=false;
      break;
  }
  

  //delay(BNO055_SAMPLERATE_DELAY_MS);
}

void camTracker(){
  
  static float oldAZ;
  static float oldEL;
  static uint8_t oldMag, oldGyro, oldAcc;
  /* Display the floating point data */
  //Serial.print("X: ");
  //Serial.print(euler.x());
  Serial.print(" ELEVATION: ");
  Serial.print(EL);
  Serial.print(" AZIMUTH: ");
  Serial.print(AZ);
  Serial.print("\t\t");

  /*
  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  Serial.print("qW: ");
  Serial.print(quat.w(), 4);
  Serial.print(" qX: ");
  Serial.print(quat.x(), 4);
  Serial.print(" qY: ");
  Serial.print(quat.y(), 4);
  Serial.print(" qZ: ");
  Serial.print(quat.z(), 4);
  Serial.print("\t\t");
  */

  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);



//  M5.Display.print(system, DEC);
//  M5.Display.print(" Gyro=");
//  M5.Display.print(gyro, DEC);
//  M5.Display.print(" Accel=");
//  M5.Display.print(accel, DEC);
//  M5.Display.print(" Mag=");
//  M5.Display.print(mag, DEC);


auto mag_color  = red_brightness_values[mag];
auto acc_color  = red_brightness_values[accel];
auto gyro_color = red_brightness_values[gyro];

/*
auto mag_color  = 0xFF0000U; if(mag==2) mag_color = 0xAA0000U; if(mag==3)   mag_color = 0x000000U;
auto acc_color  = RED; if(accel==2) acc_color = YELLOW; if(accel==3) acc_color = GREEN;
auto gyro_color = RED; if(gyro==2) gyro_color = YELLOW; if(gyro==3) gyro_color = GREEN;
*/

M5.Display.setTextSize(0);
int xpos = 50;
int ypos = 15;
int xSpacing = 10;
auto batt = M5.Power.getBatteryLevel();
if(batt!=lastbatt || redraw) {
  M5.Display.fillRect(xpos , ypos, 41,20,BLACK);
  lastbatt=batt;

if (batt>75) M5.Display.drawXBitmap(xpos , ypos, batt_100_bits, batt_100_width, batt_100_height, red_brightness_values[2]);
else if (batt>50) M5.Display.drawXBitmap(xpos , ypos, batt_75_bits, batt_75_width, batt_75_height, red_brightness_values[2]);
else if (batt>25) M5.Display.drawXBitmap(xpos , ypos, batt_50_bits, batt_50_width, batt_50_height, red_brightness_values[2]);
else M5.Display.drawXBitmap(xpos , ypos, batt_25_bits, batt_25_width, batt_25_height, red_brightness_values[1]);

M5.Display.setCursor(xpos, ypos+batt_75_height+5);
M5.Display.printf(" %i %%    ", batt);
}

xpos += batt_75_width + xSpacing;
if(oldMag!=mag || redraw){
M5.Display.drawXBitmap(xpos, 0, mag_icon_bits, mag_icon_width, mag_icon_height, mag_color);
oldMag=mag;
}
xpos+=mag_icon_width+xSpacing;
if(oldGyro!=gyro || redraw){
M5.Display.drawXBitmap(xpos , 0, gyro_icon_bits, gyro_icon_width, gyro_icon_height, gyro_color);
oldGyro=gyro;
}
xpos+=gyro_icon_width+xSpacing;
if(oldAcc!=accel || redraw){
M5.Display.drawXBitmap(xpos , 0, acc_icon_bits, acc_icon_width, acc_icon_height, acc_color);
oldAcc=accel;
}



if(oldAZ!=AZ || redraw){
  M5.Display.setTextSize(2);
M5.Display.setCursor(200, 100);
M5.Display.print("AZ:");
M5.Display.setTextSize(3);
M5.Display.setCursor(190, 130);
M5.Display.printf("%.1f  ", AZ);

float constrainedAZ = constrain(AZ,0,359);
int W = map(constrainedAZ,0,359,0,M5.Display.width());

M5.Lcd.fillRect(0,M5.Display.height()-3,M5.Display.width(),3,BLACK);
M5.Lcd.fillRect(0,M5.Display.height()-3,W,3,red_brightness_values[2]);
oldAZ=AZ;  
}

if(oldEL!=EL || redraw){ 
M5.Display.setTextSize(2);
  M5.Display.setCursor(75, 100);
  M5.Display.print("EL:");
  M5.Display.setTextSize(3);
  M5.Display.setCursor(65, 130);  
  M5.Display.printf("%.1f  ", EL);
    
  float constrainedEL = constrain(EL,0,90);
  int H = map(constrainedEL,0,90,0,M5.Display.height());

  M5.Lcd.fillRect(M5.Display.width()-3,0,3,M5.Display.height()-3,BLACK);
  M5.Lcd.fillRect(M5.Display.width()-3,M5.Display.height()-H,3,H,red_brightness_values[2]);
  
  oldEL=EL;
}
}

int16_t centerX = 160;
int16_t centerY = 120;
int16_t radius1 = 110;
int16_t radius2 = 92; // seconds hand
int16_t radius3 = 80; // minutes hand
int16_t radius4 = 50; // hours hand

int16_t endX = 0;
int16_t endY = 0;
int16_t endX1 = 0;
int16_t endY1 = 0;
int16_t endX2 = 0;
int16_t endY2 = 0;
int16_t endXold = 0;
int16_t endYold = 0;
int16_t endX1old = 0;
int16_t endY1old = 0;
int16_t radius5 = 100; // compass

void drawCompassFace() {
    M5.Display.setTextSize(2);

    float angle;
    M5.Lcd.fillScreen(BLACK);
   for( int16_t i = 0; i <= 36; i++)
      {   //angle = degtorad(i*10);
          angle = i*10*0.01745329251; // pi/180 = 0.01745329251
         
       
          if(i == 0 || i== 9 || i== 18 || i== 27 || i== 36) 
              { endX = (int16_t)(centerX + (sin(angle)*(radius5+0)));
                endY = (int16_t)(centerY - (cos(angle)*(radius5+0)));
                M5.Lcd.drawLine(centerX, centerY, endX , endY, RED );
              }
          else{ endX = (int16_t)(centerX + (sin(angle)*radius5));
                endY = (int16_t)(centerY - (cos(angle)*radius5));
                M5.Lcd.drawLine(centerX, centerY, endX , endY, red_brightness_values[2] );
              }     
      }
      M5.Lcd.fillCircle(centerX, centerY, radius5-15, BLACK );
      M5.Lcd.setCursor(155, 0);
      M5.Lcd.print("N");
      M5.Lcd.setCursor(275, 110);
      M5.Lcd.print("E");
      M5.Lcd.setCursor(35, 110);
      M5.Lcd.print("W");
      M5.Lcd.setCursor(155, 225);
      M5.Lcd.print("S");
      //compassface = true;     
}
//void drawCompassNeedle(float angle2) 
void drawCompassNeedle() 
{ 
  static float oldAZ;

  if(oldAZ==AZ && !redraw) return; //no changes -> Do nothing

  float angle2;
  angle2 = (360-AZ)*0.01745329251; //deg to rad
  

  M5.Lcd.drawLine(centerX, centerY, endXold , endYold, BLACK );           // erase old needle
  M5.Lcd.drawLine(centerX, centerY, endX1old , endY1old, BLACK );
  

  endX = (int16_t)(centerX + (sin(angle2)*radius3));
  endY = (int16_t)(centerY - (cos(angle2)*radius3));
  endX1 = (int16_t)(centerX - (sin(angle2)*radius4-0));
  endY1 = (int16_t)(centerY + (cos(angle2)*radius4-0));
//  M5.Lcd.fillCircle(centerX, centerY, radius5-15, BLACK );  
  M5.Lcd.drawLine(centerX, centerY, endX , endY, red_brightness_values[1] );
  M5.Lcd.drawLine(centerX, centerY, endX1 , endY1, 0x555555U );

  M5.Lcd.setCursor(centerX-30, centerY-8);
  M5.Display.printf("%.1f  ", AZ);

 endXold = endX;
 endYold = endY;
 endX1old = endX1;
 endY1old = endY1;

 oldAZ=AZ;

 }
 
