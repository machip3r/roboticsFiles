#define PIN_W_LEFT_1 6
#define PIN_W_LEFT_2 7
#define PIN_W_RIGHT_1 4
#define PIN_W_RIGHT_2 5

#define PIN_V_LEFT 10
#define PIN_V_RIGHT 11

#define PIN_SENSOR_TRIGGER 8
#define PIN_SENSOR_REPEATER 9

/*
  0 - unknown
  1 - used
  2 - unused
*/

int gridMap[5][5] = {
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 1, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0}
}, trajectBlocks = 8, actualX = 2, actualY = 2;

void moveLeftWheel(unsigned char velocity) {
  analogWrite(PIN_V_LEFT, velocity);

  digitalWrite(PIN_W_LEFT_1, HIGH);
  digitalWrite(PIN_W_LEFT_2, LOW);
}

void moveRightWheel(unsigned char velocity) {
  analogWrite(PIN_V_RIGHT, velocity);

  digitalWrite(PIN_W_RIGHT_1, LOW);
  digitalWrite(PIN_W_RIGHT_2, HIGH);
}

void moveForward(unsigned char velocityL, unsigned char velocityR) {
  moveLeftWheel(velocityL);
  moveRightWheel(velocityR);
}

void stopMoving() {
  moveLeftWheel(0);
  moveRightWheel(0);
}

void rotateToLeft(unsigned char velocity) {
  analogWrite(PIN_V_LEFT, velocity);
  analogWrite(PIN_V_RIGHT, velocity);

  digitalWrite(PIN_W_LEFT_1, LOW);
  digitalWrite(PIN_W_LEFT_2, HIGH);
  digitalWrite(PIN_W_RIGHT_1, LOW);
  digitalWrite(PIN_W_RIGHT_2, HIGH);
}

void rotateToRight(unsigned char velocity) {
  analogWrite(PIN_V_LEFT, velocity);
  analogWrite(PIN_V_RIGHT, velocity);

  digitalWrite(PIN_W_LEFT_1, HIGH);
  digitalWrite(PIN_W_LEFT_2, LOW);
  digitalWrite(PIN_W_RIGHT_1, HIGH);
  digitalWrite(PIN_W_RIGHT_2, LOW);
}

void theresObstacle(int range) {
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

  return (((int) distance) < range);
}

void setup() {
  Serial.begin(9600);

  pinMode(PIN_W_LEFT_1, OUTPUT);
  pinMode(PIN_W_LEFT_2, OUTPUT);
  pinMode(PIN_W_RIGHT_1, OUTPUT);
  pinMode(PIN_W_RIGHT_2, OUTPUT);

  pinMode(PIN_V_LEFT, OUTPUT);
  pinMode(PIN_V_RIGHT, OUTPUT);

  pinMode(PIN_SENSOR_TRIGGER, OUTPUT);
  pinMode(PIN_SENSOR_REPEATER, INPUT);

  digitalWrite(PIN_SENSOR_TRIGGER, LOW);
}

void loop() {
  /* if (counterBlocks < totalBlocks) {
    counterBlocks++;
  } */

  unsigned char vLeft = 82, vRight = 80, vRotation = 50;

  for(int i = 0; i < (trajectBlocks + 2); i++) {
    if (!i) {
      // move to (3, 2)
      /*
        | 0 0 0 0 0 |
        | 0 0 0 0 0 |
        | 0 0 o - 0 |
        | 0 0 0 0 0 |
        | 0 0 0 0 0 |
      */
      moveForward(vLeft, vRight);
      actualX++;
      // actualY = actualY;
      delay(2000);

      // check obstacles and mark in matrix
      // obstacles in (4, 2)
      /*
        | 0 0 0 0 0 |
        | 0 0 0 0 0 |
        | 0 0 o - ? |
        | 0 0 0 0 0 |
        | 0 0 0 0 0 |
      */
      if (theresObstacle(30))
        gridMap[actualX + 1][actualY] = 1;
      delay(3000);

      rotateToLeft(vRotation);
      delay(2000);
    } else if (i && (i % 2) && i != (trajectBlocks + 1)) {
      // move to (3, 1), (1, 1), (1, 3), (3, 3)
      /*
        i = 1         i = 3          i = 5        i = 7
        | 0 0 0 0 0 | | 0 0 0 0 0 | | 0 0 0 0 0 | | 0 0 0 0 0 |
        | 0 0 0 - 0 | | 0 - o 0 0 | | 0 0 0 0 0 | | 0 0 0 0 0 |
        | 0 0 0 o 0 | | 0 0 0 0 0 | | 0 o 0 0 0 | | 0 0 0 0 0 |
        | 0 0 0 0 0 | | 0 0 0 0 0 | | 0 - 0 0 0 | | 0 0 o - 0 |
        | 0 0 0 0 0 | | 0 0 0 0 0 | | 0 0 0 0 0 | | 0 0 0 0 0 |
      */
      moveForward(vLeft, vRight);
      switch (i) {
        case 1:
          // actualX = actualX;
          actualY--;
          break;
        case 3:
          actualX--;
          // actualY = actualY;
          break;
        case 5:
          // actualX = actualX;
          actualY++;
          break;
        case 7:
          actualX++;
          // actualY = actualY;
          break;
      }
      delay(2000);

      rotateToRight(vRotation);
      delay(2000);

      // check obstacles and mark in matrix
      // obstacles in (4, 1), (1, 0), (0, 3), (3, 4)
      /*
        | 0 0 0 0 0 | | 0 ? 0 0 0 | | 0 0 0 0 0 | | 0 0 0 0 0 |
        | 0 0 0 - ? | | 0 - o 0 0 | | 0 0 0 0 0 | | 0 0 0 0 0 |
        | 0 0 0 o 0 | | 0 0 0 0 0 | | 0 o 0 0 0 | | 0 0 0 0 0 |
        | 0 0 0 0 0 | | 0 0 0 0 0 | | ? - 0 0 0 | | 0 0 o - 0 |
        | 0 0 0 0 0 | | 0 0 0 0 0 | | 0 0 0 0 0 | | 0 0 0 ? 0 |
      */
      if (theresObstacle(30))
        switch (i) {
          case 1:
            gridMap[actualX + 1][actualY] = 1;
            break;
          case 3:
            gridMap[actualX][actualY - 1] = 1;
            break;
          case 5:
            gridMap[actualX - 1][actualY] = 1;
            break;
          case 7:
            gridMap[actualX][actualY + 1] = 1;
            break;
        }
      else
        switch (i) {
          case 1:
            gridMap[actualX + 1][actualY] = 2;
            break;
          case 3:
            gridMap[actualX][actualY - 1] = 2;
            break;
          case 5:
            gridMap[actualX - 1][actualY] = 2;
            break;
          case 7:
            gridMap[actualX][actualY + 1] = 2;
            break;
        }
      delay(3000);

      rotateToLeft(vRotation);
      delay(2000);

      // check obstacles and mark in matrix
      // obstacles in (3, 0), (0, 1), (1, 4), (4, 3)
      /*
        | 0 0 0 ? 0 | | 0 0 0 0 0 | | 0 0 0 0 0 | | 0 0 0 0 0 |
        | 0 0 0 - 0 | | ? - o 0 0 | | 0 0 0 0 0 | | 0 0 0 0 0 |
        | 0 0 0 o 0 | | 0 0 0 0 0 | | 0 o 0 0 0 | | 0 0 0 0 0 |
        | 0 0 0 0 0 | | 0 0 0 0 0 | | 0 - 0 0 0 | | 0 0 o - ? |
        | 0 0 0 0 0 | | 0 0 0 0 0 | | 0 ? 0 0 0 | | 0 0 0 0 0 |
      */
      if (theresObstacle(30))
        switch (i) {
          case 1:
            gridMap[actualX][actualY - 1] = 1;
            break;
          case 3:
            gridMap[actualX - 1][actualY] = 1;
            break;
          case 5:
            gridMap[actualX][actualY + 1] = 1;
            break;
          case 7:
            gridMap[actualX + 1][actualY] = 1;
            break;
        }
      else
        switch (i) {
          case 1:
            gridMap[actualX][actualY - 1] = 2;
            break;
          case 3:
            gridMap[actualX - 1][actualY] = 2;
            break;
          case 5:
            gridMap[actualX][actualY + 1] = 2;
            break;
          case 7:
            gridMap[actualX + 1][actualY] = 2;
            break;
        }
      delay(3000);

      rotateToLeft(vRotation);
      delay(2000);
    } else if (i && !(i % 2) && i != trajectBlocks && i != (trajectBlocks + 2)) {
      // move to (2, 1), (1, 2), (2, 3)
      /*
        i = 2         i = 4          i = 6
        | 0 0 0 0 0 | | 0 0 0 0 0 | | 0 0 0 0 0 |
        | 0 0 - o 0 | | 0 o 0 0 0 | | 0 0 0 0 0 |
        | 0 0 0 0 0 | | 0 - 0 0 0 | | 0 0 0 0 0 |
        | 0 0 0 0 0 | | 0 0 0 0 0 | | 0 o - 0 0 |
        | 0 0 0 0 0 | | 0 0 0 0 0 | | 0 0 0 0 0 |
      */
      moveForward(vLeft, vRight);
      switch (i) {
        case 2:
          actualX--;
          // actualY = actualY;
          break;
        case 4:
          // actualX = actualX;
          actualY++;
          break;
        case 6:
          actualX++;
          // actualY = actualY;
          break;
      }
      delay(2000);

      rotateToRight(vRotation);
      delay(2000);

      // check obstacles and mark in matrix
      // obstacles in (2, 0), (0, 2), (2, 4)
      /*
        | 0 0 ? 0 0 | | 0 0 0 0 0 | | 0 0 0 0 0 |
        | 0 0 - o 0 | | 0 o 0 0 0 | | 0 0 0 0 0 |
        | 0 0 0 0 0 | | ? - 0 0 0 | | 0 0 0 0 0 |
        | 0 0 0 0 0 | | 0 0 0 0 0 | | 0 o - 0 0 |
        | 0 0 0 0 0 | | 0 0 0 0 0 | | 0 0 ? 0 0 |
      */
      if (theresObstacle(30))
        switch (i) {
          case 2:
            gridMap[actualX][actualY - 1] = 1;
            break;
          case 4:
            gridMap[actualX - 1][actualY] = 1;
            break;
          case 6:
            gridMap[actualX][actualY + 1] = 1;
            break;
        }
      else
        switch (i) {
          case 2:
            gridMap[actualX][actualY - 1] = 2;
            break;
          case 4:
            gridMap[actualX - 1][actualY] = 2;
            break;
          case 6:
            gridMap[actualX][actualY + 1] = 2;
            break;
        }
      delay(3000);

      rotateToLeft(vRotation);
      delay(2000);
    } else if (i == trajectBlocks) {
      // move to (3, 2)
      /*
        i = 8
        | 0 0 0 0 0 |
        | 0 0 0 0 0 |
        | 0 0 0 - 0 |
        | 0 0 0 o 0 |
        | 0 0 0 0 0 |
      */
      moveForward(vLeft, vRight);
      // actualX = actualX;
      actualY--;
      delay(2000);

      rotateToLeft(vRotation);
      delay(2000);
    } else if (i == (trajectBlocks + 1)) {
      // move to (2, 2)
      /*
        i = 9
        | 0 0 0 0 0 |
        | 0 0 0 0 0 |
        | 0 0 - o 0 |
        | 0 0 0 0 0 |
        | 0 0 0 0 0 |
      */
      moveForward(vLeft, vRight);
      actualX--;
      // actualY = actualY;
      delay(2000);
      break;
    }
  }

  stopMoving();
  delay(20000);
}
