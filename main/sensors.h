#include<Wire.h>

const long PULSE_TIMEOUT = 100000;

typedef enum {LF_L_I = A0, 
 LF_R_I = A1,
 OB_L_I = 0,
 OB_R_I = 1,
 OB_M_I = 2,
 }pin_ir_t;


typedef struct {
  int p_echo, p_trig;
  float dist;
} ultra_t;

ultra_t ultra_setup(int p_echo, int p_trig) {
  ultra_t ultra = {p_echo, p_trig};
  pinMode(ultra.p_echo, INPUT);
  pinMode(ultra.p_trig, OUTPUT);
  return ultra;
}
float ultra_read(ultra_t &ultra) {
  digitalWrite(ultra.p_trig, LOW);
  delayMicroseconds(2);
  digitalWrite(ultra.p_trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultra.p_trig, LOW);
  delayMicroseconds(2);
  long d = pulseIn(ultra.p_echo, HIGH, PULSE_TIMEOUT);
  ultra.dist = d * 100. / 5882.;
  return ultra.dist;
}

typedef struct {
  int p_x, p_y, p_button;
  float zero, scale;
  float x, y, button;
} joystick_t;
joystick_t joystick_setup(int p_x, int p_y, int p_button, float zero, float scale) {
  joystick_t joystick = {p_x, p_y, p_button, zero, scale};
  return joystick;
}
void joystick_read(joystick_t &joystick) {
  joystick.x = ((float)analogRead(joystick.p_x) - joystick.zero) / joystick.scale;
  joystick.y = ((float)analogRead(joystick.p_y) - joystick.zero) / joystick.scale;
  joystick.button = analogRead(joystick.p_button);
}

 
int LF_THRESHOLD = 100;
int GYTHRESHOLD = 100;
 
typedef struct {
  int addr;
  union {
    struct {
      int16_t ax,ay,az,tmp,gx,gy,gz;
    } v;
    float reg[14];
  } val;
} imu_t;
void imu_read(imu_t &imu) {
  Wire.beginTransmission(imu.addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(imu.addr,14,true);  // request a total of 14 registers
  int i = 0;
  while (Wire.available()) {
    imu.val.reg[i] = Wire.read();
  }
//  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
//  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
//  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
//  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
//  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
//  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
//  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}
