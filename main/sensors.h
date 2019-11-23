#include "SR04.h"
#include<Wire.h>

typedef enum {LF_L_I = A0, 
 LF_R_I = A1,
 OB_L_I = 0,
 OB_R_I = 1,
 OB_M_I = 2,
 }pin_ir_t;

typedef enum {EDGE = 3, 
 TRIGGER = 4,
 MAX_DIST = 10
 }pin_ultra_t;

 
int LF_THRESHOLD = 100;
int GYTHRESHOLD = 100;
 
SR04 sr04 = SR04(EDGE,TRIGGER);

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;


bool edgeDetected(){
 int a=sr04.Distance();
 return a > MAX_DIST;
}

bool obstacleDetected(){
 if (digitalRead(OB_L_I) == HIGH || digitalRead(OB_R_I) == HIGH || digitalRead(OB_M_I) == HIGH){
   return true;
 }
 return false;
}

bool spotDetected(){
 if (analogRead(LF_L_I) < LF_THRESHOLD && analogRead(LF_R_I) < LF_THRESHOLD){
   return true;
 }
 return false;
}


bool inclineDetected(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L) 
  if (GyX > GYTHRESHOLD){
    return true;
  }
  
}
