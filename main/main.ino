#include <ros.h>
#include <std_msgs/Empty.h>

#include "sensors.h"
#include "actuators.h"

const float ULTRA_THRESH = 10;
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


ros::NodeHandle nh;
void messageCb( const std_msgs::Empty& toggle_msg){
  state = S_ESTOP;
} 
ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );


void setup() {
  Serial.begin(9600);
  //nh.initNode();
  //nh.subscribe(sub);

  state = S_START;
  leds.estop = 22;
  leds.armed = 24;
  leds.finished = 26;
  buttons.estop = 28;
  buttons.start = 30;
  pinMode(leds.estop, OUTPUT);
  pinMode(leds.armed, OUTPUT);
  pinMode(leds.finished, OUTPUT);
  pinMode(buttons.estop, INPUT_PULLUP);
  pinMode(buttons.start, INPUT_PULLUP);

  sense.ultra.left = ultra_setup(13, 12);
  sense.ultra.right = ultra_setup(11, 10);
  sense.joystick = joystick_setup(A1, A2, A0, 330, 400);

  act.motor.left = motor_setup(7, 3, 4);
  act.motor.right = motor_setup(8, 6, 5);
  act.tictac = tictac_setup(2);
}

void drive(int power, int turn) {
  //Serial.print("drive\t");
  Serial.println(power);
  motor_drive(act.motor.left, power-turn);
  motor_drive(act.motor.right, power+turn);
}

void displayState(leds_t leds, state_t state) {
  switch (state) {
    case S_START:
    case S_ESTOP:
      digitalWrite(leds.estop, HIGH);
      digitalWrite(leds.armed, LOW);
      digitalWrite(leds.finished, LOW);
      break;
    case S_FINISH:
      digitalWrite(leds.estop, LOW);
      digitalWrite(leds.armed, LOW);
      digitalWrite(leds.finished, HIGH);
      break;
    default:
      digitalWrite(leds.estop, LOW);
      digitalWrite(leds.armed, (millis()%1300 < 100)?HIGH:LOW);
      digitalWrite(leds.finished, LOW);
  }
}

bool edgeDetected(sensors_t sense) {
  return sense.ultra.left.dist > ULTRA_THRESH || sense.ultra.right.dist > ULTRA_THRESH;
}

bool obstacleDetected(sensors_t sense){
  // TODO
  return false;
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
  //nh.spinOnce();
  ultra_read(sense.ultra.left);
  ultra_read(sense.ultra.right);
  joystick_read(sense.joystick);
  if (digitalRead(buttons.estop) == LOW) {
    state = S_ESTOP;
  }

  displayState(leds, state);
  if (state == S_ESTOP) {
    return;
  }
  //drive((int)(128*sense.joystick.y), (int)(128*sense.joystick.x));
//  if (edgeDetected(sense)) {
//    drive(0, 0);
//    tictac_drop(act.tictac);
//  } else {
//    drive(80, 0);
//  }
  tictac_loop(act.tictac);


  if (state == S_ESTOP){ return; }
  if (state == S_START){
    if (digitalRead(buttons.start) == LOW){
      delay(3000);
      state = S_RUN;
    } else{
      delay(100);
      return;
    }
  } else if (state == S_RUN){
    if (edgeDetected(sense)|| obstacleDetected(sense)){
      tictac_drop(act.tictac);
      drive(-100, 50);
    } else if (spotDetected(sense) && lineFollowed){
      drive(0, 0);
      state = S_FINISH;
    } else{
      drive(100, 0);
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
