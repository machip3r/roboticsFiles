#define PIN_W_LEFT_1 7;
#define PIN_W_LEFT_2 6;
#define PIN_W_RIGHT_1 5;
#define PIN_W_RIGHT_2 4;

#define PIN_V_LEFT 0;
#define PIN_V_RIGHT 1;

#define PWM_VELOCITY 127;

#define PIN_SENSOR_TRIGGER 8;
#define PIN_SENSOR_REPEATER 9;

struct Direction {
  bool left;
  bool right;
  bool up;
  bool down;
  int direction;
};

Direction Map[10][10];

void setup() {
  Serial.begin(9600);

  pinMode(PIN_W_LEFT_1, OUTPUT);
  pinMode(PIN_W_LEFT_2, OUTPUT);
  pinMode(PIN_W_RIGHT_1, OUTPUT);
  pinMode(PIN_W_RIGHT_2, OUTPUT);

  pinMode(PIN_SENSOR_TRIGGER, OUTPUT);
  pinMode(PIN_SENSOR_REPEATER, INPUT);
  digitalWrite(PIN_SENSOR_TRIGGER, LOW);

  Map[0][0].left = true;
  Map[0][0].right = true;
  Map[0][0].up = false;
  Map[0][0].down = true;
  Map[0][0].direction = 1;
}

void loop() {
  /* int pwmOutput = map(potValue, 0, 1023, 0 , 255); */

  moveForward(PWM_VELOCITY);
  Serial.println("Forward");

  if(getUltrasonicDetections() < 20) {
    rotateLeft();
    delay(2500);
  }

  /* delay(5000); */

  /* moveBackward(PWM_VELOCITY);
  Serial.println("Backward");
  delay(5000); */

  /* stopMoving();
  Serial.println("Stop");
  delay(10000); */

  /* getUltrasonicDetections(); */
}

void moveForward(PWM_VELOCITY) {
  analogWrite(PIN_V_LEFT, PWM_VELOCITY);
  analogWrite(PIN_V_RIGHT, PWM_VELOCITY);

  digitalWrite(PIN_V_LEFT, HIGH);
  digitalWrite(PIN_V_RIGHT, HIGH);

  digitalWrite(PIN_W_LEFT_1, LOW);
  digitalWrite(PIN_W_LEFT_2, HIGH);
  digitalWrite(PIN_W_RIGHT_1, LOW);
  digitalWrite(PIN_W_RIGHT_2, HIGH);
}

void moveBackward(PWM_VELOCITY) {
  analogWrite(PIN_V_LEFT, PWM_VELOCITY);
  analogWrite(PIN_V_RIGHT, PWM_VELOCITY);

  digitalWrite(PIN_V_LEFT, HIGH);
  digitalWrite(PIN_V_RIGHT, HIGH);

  digitalWrite(PIN_W_LEFT_1, HIGH);
  digitalWrite(PIN_W_LEFT_2, LOW);
  digitalWrite(PIN_W_RIGHT_1, HIGH);
  digitalWrite(PIN_W_RIGHT_2, LOW);
}

void rotateLeft() {
  digitalWrite(PIN_V_LEFT, HIGH);
  digitalWrite(PIN_V_RIGHT, HIGH);

  digitalWrite(PIN_W_LEFT_1, HIGH);
  digitalWrite(PIN_W_LEFT_2, LOW);
  digitalWrite(PIN_W_RIGHT_1, LOW);
  digitalWrite(PIN_W_RIGHT_2, HIGH);
}

void rotateRight() {
  digitalWrite(PIN_V_LEFT, HIGH);
  digitalWrite(PIN_V_RIGHT, HIGH);

  digitalWrite(PIN_W_LEFT_1, LOW);
  digitalWrite(PIN_W_LEFT_2, HIGH);
  digitalWrite(PIN_W_RIGHT_1, HIGH);
  digitalWrite(PIN_W_RIGHT_2, LOW);
}

void stopMoving() {
  digitalWrite(PIN_V_LEFT, LOW);
  digitalWrite(PIN_V_RIGHT, LOW);

  digitalWrite(PIN_W_LEFT_1, LOW);
  digitalWrite(PIN_W_LEFT_2, LOW);
  digitalWrite(PIN_W_RIGHT_1, LOW);
  digitalWrite(PIN_W_RIGHT_2, LOW);
}

void getUltrasonicDetections(){
  long t, distance;

  digitalWrite(PIN_SENSOR_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_SENSOR_TRIGGER, LOW);

  t = pulseIn(PIN_SENSOR_REPEATER, HIGH);
  distance = (t / 59);

  /* Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print("cm");
  Serial.println();
  delay(100); */

  /* return (distance < 20) ? true : false; */
  return distance;
}
