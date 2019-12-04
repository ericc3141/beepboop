#include <Servo.h>

typedef struct {
  int p_enable, p_pos, p_neg;
} motor_t;

motor_t motor_setup(int p_enable, int p_pos, int p_neg) {
  motor_t motor = {p_enable, p_pos, p_neg};
  pinMode(p_enable, OUTPUT);
  pinMode(p_pos, OUTPUT);
  pinMode(p_neg, OUTPUT);
  return motor;
}

void motor_drive(motor_t &motor, int power) {
  if (power > 0) {
    digitalWrite(motor.p_pos, HIGH);
    digitalWrite(motor.p_neg, LOW);
    analogWrite(motor.p_enable, power);
  } else if (power < 0) {
    digitalWrite(motor.p_pos, LOW);
    digitalWrite(motor.p_neg, HIGH);
    analogWrite(motor.p_enable, -power);
  } else {
    digitalWrite(motor.p_pos, HIGH);
    digitalWrite(motor.p_neg, HIGH);
    analogWrite(motor.p_enable, 0);
  }
}

typedef struct {
  int p_servo;
  Servo servo;
  unsigned long start;
  bool dropped;
  bool done;
} tictac_t;

tictac_t tictac_setup(int p_servo) {
  tictac_t tictac = {p_servo};
  tictac.servo.attach(p_servo);
  tictac.servo.write(90);
  return tictac;
}

void tictac_drop(tictac_t &tictac) {
  if (tictac.dropped) {
    return;
  }
  tictac.start = millis();
  tictac.servo.write(72);
  tictac.dropped = true;
}

void tictac_loop(tictac_t &tictac) {
  if (tictac.dropped && millis() - tictac.start > 3000) {
    tictac.servo.write(87);
    tictac.done = true;
  }
}
