#include <ros.h>
#include <std_msgs/Empty.h>

#include "sensors.h"
#include "actuators.h"

const float ULTRA_THRESH = 10;

 typedef enum{
  starting_state,
  default_state,
  finished_state,
  forced_stopped_state
 }states_t;

typedef struct {
  int armed, complete, estop;
} leds_t;
leds_t leds = {};

typedef struct {
  int estop;
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
int state;
int start_button = 10;
int tic_tac_state = 0;




ros::NodeHandle nh;
void messageCb( const std_msgs::Empty& toggle_msg){
  state = forced_stopped_state;
} 
ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );
 


void setup() {
   Serial.begin(9600);
   state = starting_state;
   leds.estop = 22;
   buttons.estop = 28;
   pinMode(leds.estop, OUTPUT);
   pinMode(buttons.estop, INPUT_PULLUP);
   //nh.initNode();
   //nh.subscribe(sub);
   sense.ultra.left = ultra_setup(13, 12);
   sense.ultra.right = ultra_setup(11, 10);
   sense.joystick = joystick_setup(A1, A2, A0, 330, 400);

   act.motor.left = motor_setup(7, 3, 4);
   act.motor.right = motor_setup(8, 6, 5);
   act.tictac = tictac_setup(2);
}

void drive(int power, int turn) {
  //Serial.print("drive\t");
  //Serial.println(power);
  motor_drive(act.motor.left, power-turn);
  motor_drive(act.motor.right, power+turn);
}

void loop() {
 //nh.spinOnce();
 ultra_read(sense.ultra.left);
 ultra_read(sense.ultra.right);
 joystick_read(sense.joystick);

  drive((int)(128*sense.joystick.y), (int)(128*sense.joystick.x));

  Serial.println(sense.ultra.left.dist);
  Serial.println(sense.ultra.right.dist);

 if (sense.ultra.left.dist > ULTRA_THRESH || sense.ultra.right.dist > ULTRA_THRESH) {
  //drive(0);
  tictac_drop(act.tictac);
 } else {
  //drive(80);
 }
 tictac_loop(act.tictac);
 if (digitalRead(buttons.estop) == LOW) {
  state = forced_stopped_state;
 }
 
 if (state == forced_stopped_state){
  Serial.print("hello");
  digitalWrite(leds.estop, HIGH);
 } else {
  digitalWrite(leds.estop, LOW);
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
  
