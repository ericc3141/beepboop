#include <ros.h>
#include <std_msgs/Empty.h>

#include "sensors.h"
#include "actuators.h"

/* State variables */

// Overall robot state
typedef enum {
  S_START,
  S_RUN,
  S_FINISH,
  S_ESTOP
} state_t;

// Drive state
typedef enum {
  D_FORWARD, 
  D_REVERSE, 
  D_TURN
} drive_t;

/* Structs for hardware */

// LEDS
typedef struct {
  int armed, finished, estop;
} leds_t;
leds_t leds = {};

// Buttons
typedef struct {
  int estop, start;
} buttons_t;
buttons_t buttons = {};

// Sensors
typedef struct {
  struct {
    ultra_t left, right;
  } ultra;
  struct {
    ir_t left, right;
  } line;
  struct {
    ultra_t left, mid, right;
  } obstacle;
  struct {
    ir_t left, mid, right;
  } spot;
  imu_t imu;
  joystick_t joystick;
} sensors_t;
sensors_t sense = {};

// Actuators
typedef struct {
  struct {
    motor_t left, right;
  } motor;
  tictac_t tictac;
} actuators_t;
actuators_t act = {};

// Declare state variables
state_t state;
drive_t drive_state;
bool lineFollowed = false;
int tic_tac_state = 0;
int spot_state = 0;

// Initialize drive variables
int drive_power[2] = {0, 0}; // {absolute power, power diff between motors}
int drive_remaining = -1;
int drive_turn = 1; // turn direction

int myTurnTime[] = {200, 200, 200, 200, 200, 200};
int myTurnDir[] = {-1, -1, -1, -1, -1, -1};
int turnVar;

typedef struct {
  unsigned long now, prev, dt;
} clock_t;
clock_t gtime = {0, 0, 1};

/* E-Stop Code */
ros::NodeHandle nh;
void messageCb( const std_msgs::Empty& toggle_msg){
  state = S_ESTOP;
} 
ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb);

void setup() {
  Wire.begin();
  Serial.begin(9600);
  //  nh.initNode();
  //  nh.subscribe(sub);
  turnVar = 0;
  state = S_START;
  drive_state = D_FORWARD;
  
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
  
  sense.line.left = ir_setup(A3);
  sense.line.right = ir_setup(A4);

  sense.spot.left = ir_setup(A5); // TODO: insert actual pins
  sense.spot.right = ir_setup(A6); // TODO: insert actual pins
  sense.spot.mid = ir_setup(A7); // TODO: insert actual pins
  
  sense.obstacle.left = ultra_setup(42, 46);
  sense.obstacle.right = ultra_setup(40, 44);
  //  sense.obstacle.mid = ultra_setup(A2, 1);
  
  sense.joystick = joystick_setup(A0, A1, A2, 512, 512);
  sense.imu = imu_setup(0x68);
  
  act.motor.right = motor_setup(8, 3, 4);
  act.motor.left = motor_setup(7, 6, 5);
  
  act.tictac = tictac_setup(2);
}

int clamp(int lo, int hi, int val) {
  return max(lo, min(hi, val));
}

void drive(int power, int turn) {
  int left = clamp(-255, 255, power - turn);
  int right = clamp(-255, 255, power + turn);
  Serial.print("\tdrive\t");
  Serial.print(left);
  Serial.print("\t");
  Serial.print(right);
  motor_drive(act.motor.left, left);
  motor_drive(act.motor.right, right);
}

  //drive((int)(255.*sense.joystick.y), (int)(255.*sense.joystick.x));

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
      digitalWrite(leds.finished, (millis() % 1000 < 500)?HIGH:LOW);
      break;
    default:
      digitalWrite(leds.estop, LOW);
      analogWrite(leds.armed, (millis() % 1300 < 100)?255:60);
      digitalWrite(leds.finished, LOW);
  }
}

bool edgeDetected(sensors_t sense) {
  return sense.ultra.left.dist > 15 || sense.ultra.right.dist > 15;
}

bool obstacleDetected(sensors_t sense){
  const float dist_thresh = 10;
  return sense.obstacle.left.dist < dist_thresh || sense.obstacle.right.dist < dist_thresh;
}

bool spotDetected(sensors_t sense){
  return sense.spot.left.light > 300 
    && sense.spot.right.light > 300
    && sense.spot.mid.light > 300;
}

bool inclineDetected(sensors_t sense){
  // TODO
  return false;
}

void loop() {
  Serial.print("TurnVar");
  Serial.print(turnVar);
  
  delay(10);
  
  gtime.prev = gtime.now;
  gtime.now = millis();
  gtime.dt = gtime.now - gtime.prev;
  
  Serial.print("\n");
  //nh.spinOnce();

  /* Read all sensors */
  ultra_read(sense.ultra.left);
  ultra_read(sense.ultra.right);
  ir_read(sense.line.left);
  ir_read(sense.line.right);
  Serial.print("IR RIGHT: ");
  Serial.print("IR LEFT: ");
  Serial.print(sense.line.left.light);
  Serial.print(sense.line.right.light);
  
  ultra_read(sense.obstacle.left);
  ultra_read(sense.obstacle.right);
//  ir_read(sense.obstacle.mid);
  joystick_read(sense.joystick);
  imu_read(sense.imu);
  if (digitalRead(buttons.estop) == LOW) {
    state = S_ESTOP;
  }

  Serial.print("\t");
  Serial.print(sense.imu.o.x);
  Serial.print("\t");
  Serial.print(sense.obstacle.left.dist);
  Serial.print("\t");
  Serial.print(sense.obstacle.right.dist);
  Serial.print("\t");
  switch(drive_state) {
    case D_FORWARD: Serial.print("forward"); break;
    case D_REVERSE: Serial.print("reverse"); break;
    case D_TURN: Serial.print("turn"); break;
    default: Serial.print("broken");
  }

  if (state == S_ESTOP) {
    drive_remaining = -1;
  } else if (state == S_START) {
    drive_remaining = -1;
    if (digitalRead(buttons.start) == LOW) {
      delay(3000);
      state = S_RUN;
      drive_state = D_FORWARD;
      drive_power[0] = 100;
      drive_power[1] = 0;
    }
  } else if (state == S_RUN) {
      // begin non-reversing block
      if (drive_state != D_REVERSE) {
        if (sense.ultra.left.dist > 15) {
          drive_state = D_REVERSE;
          drive_remaining = 700;
          turnVar = (turnVar + 1) % 6;
          drive(0,0);
          delay(100);
          drive_turn = myTurnDir[turnVar];
        } if (sense.ultra.right.dist > 15) {
          drive_state = D_REVERSE;
          drive_remaining = 700;
          turnVar = (turnVar + 1) % 6;
          drive_turn = myTurnDir[turnVar];
          drive(0,0);
          delay(100);
        } else if (obstacleDetected(sense)) {
          drive_state = D_REVERSE;
          drive_remaining = 700;
          turnVar = (turnVar + 1) % 6;
          drive_turn = myTurnDir[turnVar];
          drive(0,0);
          delay(100);
        } else if (spotDetected(sense) && lineFollowed) {
          state = S_FINISH;
          drive_remaining = -1;
        }
      }
      // end non-reversing block

      if (drive_state == D_REVERSE && drive_remaining < 0) {
        drive_state = D_TURN;
        drive_remaining = myTurnTime[turnVar];
      } else if (drive_state == D_TURN && drive_remaining < 0) {
        drive_state = D_FORWARD;
        drive_remaining = 1;
      } else if (drive_state == D_FORWARD) {
        drive_remaining = 1;
      }
      
      if (drive_state == D_FORWARD) {
        // IMU logic
        if (300 < sense.imu.o.x && sense.imu.o.x < 350) {
          drive_power[0] = 40;//255;
          drive_power[1] = 0;
        } else if (sense.line.left.light > 300) { // Line following
          drive_power[0] = 48;
          drive_power[1] = 48;
          Serial.println("turn left");
        } else if (sense.line.right.light > 300 ) {
          drive_power[0] = 48  ;
          drive_power[1] = -48;
          Serial.println("turn right");
        } else {
          drive_power[0] = 48;
          drive_power[1] = 0;
        }      
      } else if (drive_state == D_REVERSE) {
        drive_power[0] = -60; 
        drive_power[1] = 0;
      } else if (drive_state = D_TURN) {
        drive_power[0] = 0; 
        drive_power[1] = drive_turn * 180;
      } else {
        drive_power[0] = 0; drive_power[1] = 0;
      }

      /* Tic tac logic */
      if (inclineDetected(sense) && tic_tac_state == 0) {
        tic_tac_state = 1;
      }
      
      if (!inclineDetected(sense) && tic_tac_state == 1) {
        tic_tac_state = 2;
        tictac_drop(act.tictac);
      }
  }


  displayState(leds, state);
  
  if (drive_remaining >= 0) {
    drive(drive_power[0], drive_power[1]);
    drive_remaining -= gtime.dt;
  } else {
    drive(0, 0);
  }
  
  tictac_loop(act.tictac);
}
