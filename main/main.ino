#include <ros.h>
#include <std_msgs/Empty.h>

#include "sensors.h"
#include "actuators.h"

const float ULTRA_THRESH = 15;
const int MPU_addr=0x68;  // I2C address of the MPU-6050

typedef enum{
  S_START,
  S_RUN,
  S_FINISH,
  S_ESTOP
} state_t;

typedef struct {
  int armed, finished, estop;
} leds_t;
leds_t leds = {};

typedef struct {
  int estop, start;
} buttons_t;
buttons_t buttons = {};

typedef struct {
  struct {
    ultra_t left, right;
  } ultra;
  struct {
    ir_t left, right;
  } line;
  struct {
    ir_t left, mid, right;
  } obstacle;
  joystick_t joystick;
} sensors_t;
sensors_t sense = {};

typedef struct {
  struct {
    motor_t left, right;
  } motor;
  tictac_t tictac;
} actuators_t;
actuators_t act = {};

bool lineFollowed = false;
state_t state;
int start_button = 10;
int tic_tac_state = 0;

typedef enum {D_STOP, D_FORWARD, D_REVERSE, D_TURN} drive_t;
drive_t drive_state = D_STOP;
int drive_remaining;

typedef struct {
  unsigned long now, prev, dt;
} clock_t;
clock_t gtime = {0, 0, 1};


ros::NodeHandle nh;
void messageCb( const std_msgs::Empty& toggle_msg){
  state = S_ESTOP;
} 
ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );


void setup() {
  Serial.begin(9600);
//  nh.initNode();
//  nh.subscribe(sub);

  state = S_START;
  leds.estop = 22;
  leds.armed = 9;
  leds.finished = 26;
  buttons.estop = 28;
  buttons.start = 30;
  pinMode(leds.estop, OUTPUT);
  pinMode(leds.armed, OUTPUT);
  pinMode(leds.finished, OUTPUT);
  pinMode(buttons.estop, INPUT_PULLUP);
  pinMode(buttons.start, INPUT_PULLUP);

  sense.ultra.left = ultra_setup(34, 38);
  sense.ultra.right = ultra_setup(32, 36);
  sense.obstacle.left = ir_setup(A0);
  sense.obstacle.right = ir_setup(A1);
  sense.obstacle.mid = ir_setup(A2);
  sense.joystick = joystick_setup(A0, A1, A2, 512, 512);

  act.motor.right = motor_setup(8, 3, 4);
  act.motor.left = motor_setup(7, 6, 5);
  act.tictac = tictac_setup(2);
}

int clamp(int lo, int hi, int val) {
  return max(lo, min(hi, val));
}
void drive(int power, int turn) {
  int left = clamp(-255, 255, power-turn);
  int right = clamp(-255, 255, power+turn);
  Serial.print("\tdrive\t");
  Serial.print(left);
  Serial.print("\t");
  Serial.print(right);
  motor_drive(act.motor.left, left);
  motor_drive(act.motor.right, right);
}

void drivefor() {
  drive_remaining -= gtime.dt;
  if (drive_state == D_FORWARD) {
    drive(50, 0);
  } else if (drive_state == D_REVERSE) {
    drive(-50, 0);
    if (drive_remaining < 0) {
      drive_state = D_TURN;
      drive_remaining = 500;
    }
  } else if (drive_state == D_TURN) {
    drive(0, 92);
    if (drive_remaining < 0) {
      drive_state = D_FORWARD;
    }
  } else {
    drive(0, 0);
  }

  //drive((int)(255.*sense.joystick.y), (int)(255.*sense.joystick.x));
}

void displayState(leds_t leds, state_t state) {
  switch (state) {
    case S_START:
    case S_ESTOP:
      digitalWrite(leds.estop, HIGH);
      analogWrite(leds.armed, LOW);
      digitalWrite(leds.finished, LOW);
      break;
    case S_FINISH:
      digitalWrite(leds.estop, LOW);
      analogWrite(leds.armed, LOW);
      digitalWrite(leds.finished, (millis()%1000 < 500)?HIGH:LOW);
      break;
    default:
      digitalWrite(leds.estop, LOW);
      analogWrite(leds.armed, (millis()%1300 < 100)?255:60);
      digitalWrite(leds.finished, LOW);
  }
}

bool edgeDetected(sensors_t sense) {
  return sense.ultra.left.dist > ULTRA_THRESH || sense.ultra.right.dist > ULTRA_THRESH;
}

bool obstacleDetected(sensors_t sense){
  const float thresh = 300;
  return sense.obstacle.left.light < thresh
      || sense.obstacle.right.light < thresh
      || sense.obstacle.mid.light < thresh;
}

bool spotDetected(sensors_t sense){
  // TODO
  return false;
}
bool inclineDetected(sensors_t sense){
  // TODO
  return false;
}

void loop() {
  delay(10);
  gtime.prev = gtime.now;
  gtime.now = millis();
  gtime.dt = gtime.now - gtime.prev;
  Serial.print("\n");
  //nh.spinOnce();
  ultra_read(sense.ultra.left);
  ultra_read(sense.ultra.right);
  ir_read(sense.line.left);
  ir_read(sense.line.right);
  ir_read(sense.obstacle.left);
  ir_read(sense.obstacle.right);
  ir_read(sense.obstacle.mid);
  joystick_read(sense.joystick);
  if (digitalRead(buttons.estop) == LOW) {
    state = S_ESTOP;
  }

  displayState(leds, state);
  drivefor();
  if (state == S_ESTOP) {
    drive_state = D_STOP;
    return;
  }

//  Serial.print("ultra\t");
//  Serial.print(sense.ultra.left.dist);
//  Serial.print("\t");
//  Serial.print(sense.ultra.right.dist);
  Serial.print(sense.obstacle.left.light);
  tictac_loop(act.tictac);


  if (state == S_ESTOP){ return; }
  if (state == S_START){
    if (digitalRead(buttons.start) == LOW){
      delay(3000);
      state = S_RUN;
      drive_state = D_FORWARD;
    } else{
      delay(100);
      return;
    }
  } else if (state == S_RUN){
    if (edgeDetected(sense)|| obstacleDetected(sense)){
        drive_state = D_REVERSE;
        drive_remaining = 1000;
    } else if (spotDetected(sense) && lineFollowed){
      state = S_FINISH;
      drive_state = D_STOP;
    } else{
      if (sense.line.left.light < 290) {
      } else {
      }
    }

    if (inclineDetected(sense) && tic_tac_state == 0) {
      tic_tac_state =1;
    }
    if (!inclineDetected(sense) && tic_tac_state ==1){
      tic_tac_state = 2;
      tictac_drop(act.tictac);
    }
  }
}
