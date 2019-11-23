#include "SR04.h"
#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
int turned = 0;

long a;
int enable_a = 7;
int in1 = 3;
int in2 = 4;
int in3 = 5;
int in4 = 6;

SR04 sr04_left = SR04(13,12);
SR04 sr04_right = SR04(11,10);

void setup() {
  Serial.begin(9600);
  delay(1000);

  pinMode(enable_a, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  
}

void loop()
{

 long left = sr04_left.Distance();
 long right = sr04_right.Distance();

  if (left > 6 || right > 6) {
    digitalWrite(in4, HIGH);
    digitalWrite(in3, LOW);
    
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enable_a, 0);

    
    
    
    // turn servo
    if (turned == 0) {
      myservo.attach(2);
      myservo.write(180);
      turned = 1;
    } else {
      myservo.detach();
    }
  
  
  } else {
    digitalWrite(in4, HIGH);
    digitalWrite(in3, LOW);
    
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enable_a, 100);
  }

  Serial.print("left");
  Serial.print(left);
  Serial.println("cm");
  
  Serial.print("right");
  Serial.print(right);
  Serial.println("cm");


}

// goforward
// backward
//stop
