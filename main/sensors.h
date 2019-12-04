#ifndef SENSORS_H
#define SENSORS_H

#include<Wire.h>

const long PULSE_TIMEOUT = 100000;

typedef enum {LF_L_I = A0, 
 LF_R_I = A1,
 OB_L_I = 0,
 OB_R_I = 1,
 OB_M_I = 2,
} pin_ir_t;

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

typedef struct {
  int p_in;
  float light;
} ir_t;

ir_t ir_setup(int p_in) {
  ir_t ir = {p_in};
  pinMode(p_in, INPUT);
  return ir;
}

float ir_read(ir_t &ir) {
  ir.light = analogRead(ir.p_in);
  return ir.light;
}

typedef struct {
  float x,y,z;
} vec3f_t;
vec3f_t vadd(vec3f_t a, vec3f_t b) {
  return {a.x+b.x, a.y+b.y, a.z+b.z};
}
vec3f_t vmul(vec3f_t v, float c) {
  return {v.x*c, v.y*c, v.z*c};
}
typedef struct {
  int addr;
  vec3f_t a, g, o;
} imu_t;

imu_t imu_setup(int addr) {
  imu_t imu = {addr};
  Wire.beginTransmission(addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  return imu;
}

void imu_read(imu_t &imu, float dt) {
  Wire.beginTransmission(imu.addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(imu.addr,14,true);  // request a total of 14 registers

  union {
    struct {
      int16_t ax,ay,az,tmp,gx,gy,gz;
    } val;
    char reg[14];
  } raw;
  int i = 1;
  while (Wire.available()) {
    raw.reg[i] = Wire.read();
    i += 1;
  }

  float alsb = 16384;
  float glsb = 131;
  imu.a.x = (float)raw.val.ax / alsb;
  imu.a.y = (float)raw.val.ay / alsb;
  imu.a.z = (float)raw.val.az / alsb;

  imu.g.x = (float)raw.val.gx / glsb;
  imu.g.y = (float)raw.val.gy / glsb;
  imu.g.z = (float)raw.val.gz / glsb;

  // complementary filter
  vec3f_t ao = {};
  ao.x = RAD_TO_DEG * (atan2(-imu.a.y, imu.a.z));
  ao.y = RAD_TO_DEG * (atan2(-imu.a.x, imu.a.z));
  ao.z = RAD_TO_DEG * (atan2(-imu.a.y, -imu.a.x)+PI);
   Serial.print("\t");
   Serial.print(ao.x);
  vec3f_t go = vadd(imu.o, vmul(imu.g, dt));
  imu.o = vadd(vmul(ao, 0.02), vmul(go, 0.98));
  imu.o = ao;
}

#endif // SENSORS_H
