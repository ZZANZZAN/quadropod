#include "I2Cdev.h"
//#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "Wire.h"
#include <Servo.h>
#include <math.h>

#define FILTER_STEP 1
#define FILTER_COEF 0.05
#define NUMBER_OF_SERVOMOTORS 12

////////////////////
MPU6050 accelgyro;
//float maxX = 19000;
//float maxY = 27000;
//float maxZ = 24000;
//int ax, ay, az;
//int gx, gy, gz;
int angleXYZ[3]; //[x,y,z]
//int val;
//float ax_f = 0.0, ay_f = 0.0, az_f = 0.0;
//float gx_f = 0.0, gy_f = 0.0, gz_f = 0.0;
unsigned long filter_timer;
const float toDeg = 180.0 / M_PI;
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
//uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 
Quaternion q;           
VectorFloat gravity;    
float ypr[3];           
////////////////////
//bool flagNewData = true;
Servo arrayServo[NUMBER_OF_SERVOMOTORS];
int jointAngleValues[NUMBER_OF_SERVOMOTORS];
////////////////////

void setup(){
  Serial.begin(115200); 
  Wire.begin();
  //Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  initializationDMP();
  //Serial.println("Initializing Servo...");
  initializationServo();
}

void loop(){
  if (millis() - filter_timer > FILTER_STEP) {
    filter_timer = millis();
    readingAngleFromDMP();
    comPortListener();
  }
}

////////////////////
void initializationServo(){
  arrayServo[0].attach(3);arrayServo[1].attach(5);arrayServo[2].attach(7);    //fr
  arrayServo[3].attach(9);arrayServo[4].attach(11);arrayServo[5].attach(13);  //fl
  arrayServo[6].attach(2);arrayServo[7].attach(4);arrayServo[8].attach(6);    //br
  arrayServo[9].attach(8);arrayServo[10].attach(10);arrayServo[11].attach(12);  //bl
}
void initializationDMP(){
  devStatus = accelgyro.dmpInitialize();
  accelgyro.setDMPEnabled(true);
  mpuIntStatus = accelgyro.getIntStatus();
  packetSize = accelgyro.dmpGetFIFOPacketSize();
}
////////////////////

void readingAngleFromDMP(){
  if (accelgyro.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetGravity(&gravity, &q);
    accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);
    angleXYZ[0] = ypr[2] * toDeg;
    angleXYZ[1] = ypr[1] * toDeg;
    angleXYZ[2] = ypr[0] * toDeg;
  }
  Serial.print('D');
  Serial.print(angleXYZ[0]);Serial.print('$');
  Serial.print(angleXYZ[1]);Serial.print('$');
  Serial.print(angleXYZ[2]);
  Serial.println('#');
  /*
  Serial.print(angleXYZ[0]); Serial.print(' ');
  Serial.print(angleXYZ[1]); Serial.print(' ');
  Serial.println(angleXYZ[2]);
  */
}

/*
void readingSensorMPU(){
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  ax_f += (ax - ax_f)*FILTER_COEF;
  ay_f += (ay - ay_f)*FILTER_COEF;
  az_f += (az - az_f)*FILTER_COEF;
  gx_f += (gx - gx_f)*FILTER_COEF;
  gy_f += (gy - gy_f)*FILTER_COEF;
  gz_f += (gz - gz_f)*FILTER_COEF;
  // new_a = (now_value - min_value)*((max_new_value-min_new_value)/(max_old_value-min_old_value))-...
  int new_ax = (int)((ax_f+maxX)*((180-0)/(maxX*2)));
  int new_ay = (int)((ay_f+maxY)*((180-0)/(maxY*2)));
  int new_az = (int)((az_f+maxZ)*((180-0)/(maxZ*2)));
  Serial.print(new_ax); Serial.print(' ');
  Serial.print(new_ay); Serial.print(' ');
  Serial.println(new_az);
  
  Serial.print(String(ax_f,0));
  Serial.print(" ");Serial.print(String(ay_f,0));
  Serial.print(" ");Serial.print(String(az_f,0)); 
  Serial.print(" ");Serial.print(String(gx_f,0)); 
  Serial.print(" ");Serial.print(String(gy_f,0)); 
  Serial.print(" ");Serial.print(String(gz_f,0)); 
  Serial.println(" ");
  
}
*/

////////////////////
void comPortListener(){
  if (Serial.available()>0){
    String data = Serial.readStringUntil('#');
    if (data[0] == 'S'){
      readingServoAngles(data, jointAngleValues);
    }
  }
}
////////////////////
void readingServoAngles(String data, int jointAngleValues[NUMBER_OF_SERVOMOTORS]){
  if (data.length() == (NUMBER_OF_SERVOMOTORS+2)){
    int a = 1, b = a + 3;
    for(int i = 0; i < NUMBER_OF_SERVOMOTORS; i++){
      String timeData = data.substring(a, b);
      jointAngleValues[i] = atoi(timeData.c_str());
      a = b;
      b = a + 3;
    }
    setServoAngles();
  }
}
////////////////////
void setServoAngles(){
  for(int i = 0; i < NUMBER_OF_SERVOMOTORS; i++){
    arrayServo[i].write(jointAngleValues[i]);
  }
}
