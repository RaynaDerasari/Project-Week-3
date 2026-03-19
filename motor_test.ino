//program to test both motors

#define left_motor D8
#define right_motor D4

void setup() {
  pinMode(left_motor, OUTPUT);
  pinMode(right_motor, OUTPUT); 
}

void loop() {
  analogWrite(left_motor, 255);
  analogWrite(right_motor, 255);
  delay(2000);
  analogWrite(left_motor, 127);
  analogWrite(right_motor, 127);

  delay(2000);
}