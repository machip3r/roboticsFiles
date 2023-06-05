#define PIN_W_LEFT_1 6
#define PIN_W_LEFT_2 7
#define PIN_W_RIGHT_1 5
#define PIN_W_RIGHT_2 4

#define PIN_V_LEFT 10
#define PIN_V_RIGHT 11

#define PIN_SENSOR_TRIGGER 3
#define PIN_SENSOR_REPEATER 9
#define PIN_SENSOR_LED 13

#define SIZE_MAP 5

/*
  0 - unknown
  1 - used
  2 - unused
*/
int gridMap[SIZE_MAP][SIZE_MAP] = {
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 1, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0}},
    trajectBlocks = 8, actualX = 2, actualY = 2;

// left 96, 103
// right 105, 100
unsigned char vLeft = 100, vRight = 110, vRotationLeftL = 96, vRotationRightL = 103, vRotationLeftR = 105, vRotationRightR = 100;
int tStartDelay = 10000, rangeDetection = 40, tForward = 1000, tRotationL = 410, tRotationR = 410, tCheckObstacles = 2000, tStopDelay = 10000;

bool mapping = false;

void moveLeftWheelForward(unsigned char velocity) {
    analogWrite(PIN_V_LEFT, velocity);

    digitalWrite(PIN_W_LEFT_1, LOW);
    digitalWrite(PIN_W_LEFT_2, HIGH);
}

void moveRightWheelForward(unsigned char velocity) {
    analogWrite(PIN_V_RIGHT, velocity);

    digitalWrite(PIN_W_RIGHT_1, LOW);
    digitalWrite(PIN_W_RIGHT_2, HIGH);
}

void moveForward(unsigned char velocityL, unsigned char velocityR) {
    moveLeftWheelForward(velocityL);
    moveRightWheelForward(velocityR);
}

void stopMoving() {
    moveLeftWheelForward(0);
    moveRightWheelForward(0);
}

void rotateToLeft(unsigned char velocityL, unsigned char velocityR) {
    analogWrite(PIN_V_LEFT, velocityL);
    analogWrite(PIN_V_RIGHT, velocityR);

    digitalWrite(PIN_W_LEFT_1, HIGH);
    digitalWrite(PIN_W_LEFT_2, LOW);
    digitalWrite(PIN_W_RIGHT_1, LOW);
    digitalWrite(PIN_W_RIGHT_2, HIGH);
}

void rotateToRight(unsigned char velocityL, unsigned char velocityR) {
    analogWrite(PIN_V_LEFT, velocityL);
    analogWrite(PIN_V_RIGHT, velocityR);

    digitalWrite(PIN_W_LEFT_1, LOW);
    digitalWrite(PIN_W_LEFT_2, HIGH);
    digitalWrite(PIN_W_RIGHT_1, HIGH);
    digitalWrite(PIN_W_RIGHT_2, LOW);
}

bool theresObstacle(int range) {
    long t, distance;
    int counter = 0;

    while (counter < 2) {
        digitalWrite(PIN_SENSOR_TRIGGER, LOW);
        delayMicroseconds(2);
        digitalWrite(PIN_SENSOR_TRIGGER, HIGH);
        delayMicroseconds(10);
        digitalWrite(PIN_SENSOR_TRIGGER, LOW);

        t = pulseIn(PIN_SENSOR_REPEATER, HIGH);
        /* distance = (t * 0.034 / 2); */
        distance = (t / 59);

        delay(300);
        counter++;
    }

    return ((int)(distance) <= range);
}

void showMap(int gridMap[SIZE_MAP][SIZE_MAP]) {
    /* digitalWrite(PIN_SENSOR_LED, HIGH);
    delay(1000);
    digitalWrite(PIN_SENSOR_LED, LOW);
    delay(1000);
    digitalWrite(PIN_SENSOR_LED, HIGH);
    delay(1000);
    digitalWrite(PIN_SENSOR_LED, LOW);
    delay(1000);
    digitalWrite(PIN_SENSOR_LED, HIGH);
    delay(1000);
    digitalWrite(PIN_SENSOR_LED, LOW);
    delay(1000); */

    for (int i = 0; i < SIZE_MAP; i++) {
        for (int j = 0; j < SIZE_MAP; j++) {
            if ((i < 1 || i > 3) && (j < 1 || j > 3)) {
                /* if (gridMap[i][j] == 1) {
                    digitalWrite(PIN_SENSOR_LED, HIGH);
                    delay(500);
                    digitalWrite(PIN_SENSOR_LED, LOW);
                    delay(500);
                } else if (gridMap[i][j] == 2) {
                    digitalWrite(PIN_SENSOR_LED, HIGH);
                    delay(166);
                    digitalWrite(PIN_SENSOR_LED, LOW);
                    delay(166);
                    digitalWrite(PIN_SENSOR_LED, HIGH);
                    delay(166);
                    digitalWrite(PIN_SENSOR_LED, LOW);
                    delay(166);
                    digitalWrite(PIN_SENSOR_LED, HIGH);
                    delay(166);
                    digitalWrite(PIN_SENSOR_LED, LOW);
                    delay(166);
                } */

                digitalWrite(PIN_SENSOR_LED, HIGH);
                delay(500);
                digitalWrite(PIN_SENSOR_LED, LOW);
                delay(500);

                /* delay(1000); */
            }
        }
    }

    digitalWrite(PIN_SENSOR_LED, LOW);
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
    pinMode(PIN_SENSOR_LED, OUTPUT);

    digitalWrite(PIN_SENSOR_TRIGGER, LOW);
}

void loop() {
    /* int address = 33; */

    delay(tStartDelay);

    if (mapping) {
        for (int i = 0; i < (trajectBlocks + 2); i++) {
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
                delay(tForward);
                actualX++;
                // actualY = actualY;

                // check obstacles and mark in matrix
                // obstacles in (4, 2)
                /*
                  | 0 0 0 0 0 |
                  | 0 0 0 0 0 |
                  | 0 0 o - ? |
                  | 0 0 0 0 0 |
                  | 0 0 0 0 0 |
                */
                stopMoving();
                if (theresObstacle(rangeDetection)) {
                    digitalWrite(PIN_SENSOR_LED, HIGH);
                    gridMap[actualX + 1][actualY] = 1;
                } else
                    gridMap[actualX + 1][actualY] = 2;
                delay(tCheckObstacles);
                digitalWrite(PIN_SENSOR_LED, LOW);

                // left 96, 103
                vRotationLeftL = 96;
                vRotationRightL = 103;
                // left 410
                tRotationL = 260;

                rotateToLeft(vRotationLeftL, vRotationRightL);
                delay(tRotationL);
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
                delay(tForward);
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

                // right 105, 100
                vRotationLeftR = 105;
                vRotationRightR = 100;
                // right 410
                tRotationR = 600;

                rotateToRight(vRotationLeftR, vRotationRightR);
                delay(tRotationR);

                // check obstacles and mark in matrix
                // obstacles in (4, 1), (1, 0), (0, 3), (3, 4)
                /*
                  | 0 0 0 0 0 | | 0 ? 0 0 0 | | 0 0 0 0 0 | | 0 0 0 0 0 |
                  | 0 0 0 - ? | | 0 - o 0 0 | | 0 0 0 0 0 | | 0 0 0 0 0 |
                  | 0 0 0 o 0 | | 0 0 0 0 0 | | 0 o 0 0 0 | | 0 0 0 0 0 |
                  | 0 0 0 0 0 | | 0 0 0 0 0 | | ? - 0 0 0 | | 0 0 o - 0 |
                  | 0 0 0 0 0 | | 0 0 0 0 0 | | 0 0 0 0 0 | | 0 0 0 ? 0 |
                */
                stopMoving();
                if (theresObstacle(rangeDetection)) {
                    digitalWrite(PIN_SENSOR_LED, HIGH);
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
                } else
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
                delay(tCheckObstacles);
                digitalWrite(PIN_SENSOR_LED, LOW);

                // left 96, 103
                vRotationLeftL = 96;
                vRotationRightL = 103;
                // left 410
                tRotationL = 400;

                rotateToLeft(vRotationLeftL, vRotationRightL);
                delay(tRotationL);

                // check obstacles and mark in matrix
                // obstacles in (3, 0), (0, 1), (1, 4), (4, 3)
                /*
                  | 0 0 0 ? 0 | | 0 0 0 0 0 | | 0 0 0 0 0 | | 0 0 0 0 0 |
                  | 0 0 0 - 0 | | ? - o 0 0 | | 0 0 0 0 0 | | 0 0 0 0 0 |
                  | 0 0 0 o 0 | | 0 0 0 0 0 | | 0 o 0 0 0 | | 0 0 0 0 0 |
                  | 0 0 0 0 0 | | 0 0 0 0 0 | | 0 - 0 0 0 | | 0 0 o - ? |
                  | 0 0 0 0 0 | | 0 0 0 0 0 | | 0 ? 0 0 0 | | 0 0 0 0 0 |
                */
                stopMoving();
                if (theresObstacle(rangeDetection)) {
                    digitalWrite(PIN_SENSOR_LED, HIGH);
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
                } else
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
                delay(tCheckObstacles);
                digitalWrite(PIN_SENSOR_LED, LOW);

                // left 96, 103
                vRotationLeftL = 96;
                vRotationRightL = 103;
                // left 410
                tRotationL = 250;

                rotateToLeft(vRotationLeftL, vRotationRightL);
                delay(tRotationL);
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
                delay(tForward);
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

                // right 105, 100
                vRotationLeftR = 105;
                vRotationRightR = 100;
                // right 410
                tRotationR = 600;

                rotateToRight(vRotationLeftR, vRotationRightR);
                delay(tRotationR);

                // check obstacles and mark in matrix
                // obstacles in (2, 0), (0, 2), (2, 4)
                /*
                  | 0 0 ? 0 0 | | 0 0 0 0 0 | | 0 0 0 0 0 |
                  | 0 0 - o 0 | | 0 o 0 0 0 | | 0 0 0 0 0 |
                  | 0 0 0 0 0 | | ? - 0 0 0 | | 0 0 0 0 0 |
                  | 0 0 0 0 0 | | 0 0 0 0 0 | | 0 o - 0 0 |
                  | 0 0 0 0 0 | | 0 0 0 0 0 | | 0 0 ? 0 0 |
                */
                stopMoving();
                if (theresObstacle(rangeDetection)) {
                    digitalWrite(PIN_SENSOR_LED, HIGH);
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
                } else
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
                delay(tCheckObstacles);
                digitalWrite(PIN_SENSOR_LED, LOW);

                // left 96, 103
                vRotationLeftL = 96;
                vRotationRightL = 103;
                // left 410
                tRotationL = 300;

                rotateToLeft(vRotationLeftL, vRotationRightL);
                delay(tRotationL);
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
                delay(tForward);
                // actualX = actualX;
                actualY--;

                // left 96, 103
                vRotationLeftL = 96;
                vRotationRightL = 103;
                // left 410
                tRotationL = 300;

                rotateToLeft(vRotationLeftL, vRotationRightL);
                delay(tRotationL);
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
                delay(tForward);
                actualX--;
                // actualY = actualY;
                break;
            }
        }

        stopMoving();
        /* saveMap(gridMap, address); */
        /* saveMap(gridMap); */
        delay(tStopDelay);

        showMap(gridMap);
    } else {
        Serial.print("Map: ");
        Serial.println();
        /* for (int i = 0; i < 16; i++)
            Serial.print(EEPROM.read(i));
        Serial.println();
        delay(tStopDelay);
        clearEEPROM(); */
        showMap(gridMap);

        delay(tStopDelay);
    }
}

/*
    | 0 0 0 1 0 |
    | 0 - - - 1 |
    | 1 - o - 0 |
    | 0 - - - 0 |
    | 0 1 1 1 0 |

    | 0 0 0 0 0 |
    | 0 - - - 0 |
    | 0 - o - 0 |
    | 0 - - - 0 |
    | 0 0 0 0 0 |

    | 0 0 0 0 0 |
    | 0 - - - 1 |
    | 0 - o - 1 |
    | 0 - - - 1 |
    | 0 0 0 0 0 |
 */
