#include <ros.h>
#include <std_msgs/Empty.h>

#include "sensors.h"
#include "actuators.h"

const float ULTRA_THRESH = 6;

 typedef enum{
  starting_state,
  default_state,
  finished_state,
  forced_stopped_state
 }states_t;

typedef struct {
  struct {
    ultra_t left, right;
  } ultra;
} sensors_t;
sensors_t sense = {};

typedef struct {
  struct {
    motor_t left, right;
  } motor;
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
   //nh.initNode();
   //nh.subscribe(sub);
   sense.ultra.left = ultra_setup(13, 12);
   sense.ultra.right = ultra_setup(11, 10);

   act.motor.left = motor_setup(7, 3, 4);
   act.motor.right = motor_setup(7, 6, 5);
}

void drive(int power) {
  Serial.print("drive\t");
  Serial.println(power);
  motor_drive(act.motor.left, power);
  motor_drive(act.motor.right, power);
}

void loop() {
 //nh.spinOnce();
 ultra_read(sense.ultra.left);
 ultra_read(sense.ultra.right);

 if (sense.ultra.left.dist > ULTRA_THRESH || sense.ultra.right.dist > ULTRA_THRESH) {
  drive(0);
 } else {
  drive(80);
 }
 
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
  
