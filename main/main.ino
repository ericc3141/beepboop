#include "SR04.h"
#include<Wire.h>
#include <ros.h>
#include <std_msgs/Empty.h>

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

typedef enum{EN_A = 4,
 EN_B = 5,
 IN_1 = 6,
 IN_2 = 7,
 IN_3 = 8,
 IN_4 = 9,
 }pin_motor_t;

 typedef enum{
  starting_state,
  default_state,
  finished_state,
  forced_stopped_state
 }states_t;


bool lineFollowed = false;
SR04 sr04 = SR04(EDGE,TRIGGER);
int state;
int start_button = 10;
int tic_tac_state = 0;
int LF_THRESHOLD = 100;
int GYTHRESHOLD = 100;
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

void dropTicTac(){

}

void driveForwardandStop(){

}

void driveForward(){

}

void goBackwards(){
  
}

void doTurn(){
  
}

ros::NodeHandle nh;
void messageCb( const std_msgs::Empty& toggle_msg){
  state = forced_stopped_state;
} 
ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );
 


void setup() {
   Serial.begin(9600);
   state = starting_state;
   pinMode(13, OUTPUT);
   nh.initNode();
   nh.subscribe(sub);
}

void loop() {
 nh.spinOnce();
 Serial.print(digitalRead(13));
 if (state == forced_stopped_state){
    digitalWrite(13, HIGH);   // blink the led
 }
 else{
    digitalWrite(13, LOW);   // blink the led
 }
 if (state == starting_state){
    if (digitalRead(start_button) == HIGH){
    delay(3000);
    state = default_state;
    }
    else{
      delay(100);
      return;
    }
  }
  
 else if (state == default_state){
   
    if (edgeDetected()|| obstacleDetected()){
      goBackwards();
      doTurn();
    }
  
    else if (spotDetected() && lineFollowed){
      driveForwardandStop();
      state = finished_state;
    }
  
    else{
      driveForward();
    }

    if (inclineDetected() && tic_tac_state ==0){
      tic_tac_state =1;
    }

    if (!inclineDetected() && tic_tac_state ==1){
      tic_tac_state = 2;
      dropTicTac();
    }
  }

 else{
      return;
    }
 }
  
