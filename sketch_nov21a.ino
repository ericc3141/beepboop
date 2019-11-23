  int enable_a = 8;
  int enable_b = 9;
  int in1 = 3;
  int in2 = 4;
  int in3 = 5;
  int in4 = 6;

void setup() {

  pinMode(enable_a, OUTPUT);
//  pinMode(enable_b, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

}

void loop()
{

  // goes forward
  // when I_In 3 = LOW, and I_IN 4 = HIGH, goes backwards
  digitalWrite(in4, LOW);
  digitalWrite(in3, HIGH);
//  analogWrite(enable_b, 80);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enable_a, 80);
}

// goforward
// backward
//stop
