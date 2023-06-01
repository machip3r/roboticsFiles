#define SONAR_NUM 2
#define TRIGGER_PIN 45
#define ECHO_PIN 44
#define L_TRIG_PIN 36
#define L_ECHO_PIN 37
#define R_TRIG_PIN 30
#define R_ECHO_PIN 31
#define MAX_DISTANCE 200
#define eps 5.0
#define PING_INTERVAL 250

unsigned long pingTimer[SONAR_NUM];
unsigned int cm[SONAR_NUM];
uint8_t currentSensor = 0;

MPU6050 mpu6050(Wire);
AF_DCMotor motorL(3);
AF_DCMotor motorR(4);

NewPing sonar[SONAR_NUM] = {
    NewPing(R_TRIG_PIN, R_ECHO_PIN, MAX_DISTANCE),
    NewPing(L_TRIG_PIN, L_ECHO_PIN, MAX_DISTANCE)
};

NewPing front(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

long timer = 0;
bool WallRight, WallLeft, WallFront, Turned180, TurnedLeft, TurnedRight;
int Distance = 0, modulo;
int Direction = 2;      // Direction 1: front, 2: right, 3: back, 4: left
int x = 0;
int y = 0;
int dx = 1;
int dy = 0;
float Angle, NewAngle;
long TimerFront = 0;


struct Array{
    bool Left;
    bool Right;
    bool Front;
    bool Back;
    int StrucDirection;
};

Array Map[5][5];


void setup() {
    Serial.begin(115200);
    Wire.begin();
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);

    motorL.setSpeed(85);
    motorR.setSpeed(80);

    pingTimer[0] = millis() + 75;

    // Set the starting time for each sensor.
    for (uint8_t i = 1; i < SONAR_NUM; i++)
        pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

    Map[0][0].Left = true;
    Map[0][0].Right = true;
    Map[0][0].Front = false;
    Map[0][0].Back = true;
    Map[0][0].StrucDirection = 2;
}

void loop() {
    if(millis() - TimerFront > 50) {
        Distance = front.ping_cm();
        TimerFront = millis();
    }

    Forward();

    if(Distance > 13) WallFront = false;
    else WallFront = true;
    modulo = Distance%30;


    if ((modulo >= 7)&&(modulo <= 9)) {
        Stop();
        SetDirection();
        Obstacles();
        return;
    }

    DecideWhereNext();
}


void Obstacles() {
    for (uint8_t i = 0; i < SONAR_NUM; i++) {
        if (millis() >= pingTimer[i]) {
            pingTimer[i] += PING_INTERVAL * SONAR_NUM;
            sonar[currentSensor].timer_stop();
            currentSensor = i;
            cm[currentSensor] = 0;
            sonar[currentSensor].ping_timer(echoCheck);
        }
    }
}

void echoCheck() {
    if (sonar[currentSensor].check_timer()) {
        cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
        pingResult(currentSensor);
    }
}

void pingResult(uint8_t sensor) {
    if(cm[0] > 13) {
        WallRight == false;
        Map[x][y].Right = false;
        Serial.println("No obstacle in right");
    } else if (cm[0] <= 13) {
        WallRight == true;
        Map[x][y].Right = true;
        Serial.println("Obstacle on right");
    } else if (cm[1] > 13) {
        WallLeft == false;
        Map[x][y].Left = false;
        Serial.println("No obstacle in left");
    } else if (cm[1] <= 13) {
        WallLeft == true;
        Map[x][y].Left = true;
    } else if(WallFront == true)
        Map[x][y].Front = true;
    else if (!Map[x][y].Front)
        Map[x][y].Back = false;

    Map[x][y].StrucDirection = Direction;

    Serial.print(" x:");
    Serial.print(x);
    Serial.print(" y:");
    Serial.print(y);
    Serial.print(" ");
    Serial.print(Map[x][y].Left);
    Serial.print(" ");
    Serial.print(Map[x][y].Right);
    Serial.print(" ");
    Serial.print(Map[x][y].Front);
    Serial.print(" ");
    Serial.print(Map[x][y].Back);
    Serial.print(" ");
    Serial.print(Map[x][y].StrucDirection);
    Serial.print(" ");
}

void DecideWhereNext() {
    if(!Map[x][y].Front && Map[x][y].Left && Map[x][y].Right ) {
        Forward();
        Serial.println("Forward");
    } else if(!Map[x][y].Front && !Map[x][y].Front && WallRight) {
        Forward();
        Serial.println("Forward");
    } else if(!Map[x][y].Front && !Map[x][y].Right) {
        TurnRight();
        Serial.println("Right");
    } else if(Map[x][y].Front && Map[x][y].Right && !Map[x][y].Left) {
        TurnLeft();
        Serial.println("Left");
    } else if (Map[x][y].Front && Map[x][y].Right && !Map[x][y].Left) {
        Turn180();
        Serial.println("Turning 180");
    }
}


void SetDirection() {
    if(TurnedLeft) {
        if(dy == 1) {
            dx = -1;
            dy = 0;
            Direction = 4;
        } else if(dy == -1) {
            dx = 1;
            dy = 0;
            Direction = 2;
        } else if(dx == 1) {
            dx = 0;
            dy = 1;
            Direction = 1;
        } else {
            dx = 0;
            dy = -1;
            Direction = 3;
        }
    } else if(TurnedRight) {
        if(dy == 1) {
            dx = 1;
            dy = 0;
            Direction = 2;
        } else if(dy == -1) {
            dx = -1;
            dy = 0;
            Direction = 4;
        } else if(dx == -1) {
            dx = 0;
            dy = 1;
            Direction = 1;
        } else {
            dx = 0;
            dy = -1;
            Direction = 3;
        }
    } else if (Turned180) {
        if (dy == 1) {
            dx = 0;
            dy = -1;
            Direction = 3;
        } else if (dy == -1) {
            dx = 0;
            dy = 1;
            Direction = 1;
        } else if (dx == 1) {
            dx = -1;
            dy = 0;
            Direction = 4;
        } else if (dx == -1) {
            dx = 1;
            dy = 0;
            Direction = 2;
        }
    }

    x = x + dx;
    y = y + dy;
}

/* #define SONAR_NUM 3
#define TRIGGER_PIN 45
#define ECHO_PIN 44
#define L_TRIG_PIN 36
#define L_ECHO_PIN 37
#define R_TRIG_PIN 30
#define R_ECHO_PIN 31
#define MAX_DISTANCE 200
#define eps 5.0
#define PING_INTERVAL 250

unsigned long pingTimer[SONAR_NUM];
unsigned int cm[SONAR_NUM];
uint8_t currentSensor = 0;

MPU6050 mpu6050(Wire);
AF_DCMotor motorL(3);
AF_DCMotor motorR(4);
NewPing sonar[SONAR_NUM] = {
  NewPing(R_TRIG_PIN, R_ECHO_PIN, MAX_DISTANCE),
  NewPing(L_TRIG_PIN, L_ECHO_PIN, MAX_DISTANCE),
  NewPing(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE)
  };

long timer = 0;
bool WallRight, WallLeft, WallFront,WallBack, Turned180, TurnedLeft, TurnedRight;
int Distance = 0, modulo;
int Direction = 2;      // Direction 1: front, 2: right, 3: back, 4: left
int x = 0;
int y = 0;
int dx = 1;
int dy = 0;
float Angle, NewAngle;
long TimerFront = 0;
long StopTimer = 0;


struct Array{
  bool Left;
  bool Right;
  bool Front;
  bool Back;
  int StrucDirection;
};

Array Map[5][5];


void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  motorL.setSpeed(85);
  motorR.setSpeed(80);

  pingTimer[0] = millis() + 75;
 for (uint8_t i = 1; i < SONAR_NUM; i++){ // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}
  Map[0][0].Left = true;
  Map[0][0].Right = true;
  Map[0][0].Front = false;
  Map[0][0].Back = true;
  Map[0][0].StrucDirection = 2;


}

void loop() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }

  if ((modulo >= 7)&&(modulo <= 9)|| (cm[2]<7 && cm[2]>0)) {
    if(millis() - StopTimer >= 1000)
    {
     motorL.run(RELEASE);
     motorR.run(RELEASE);
      if(x+dx<5 && x+dx>=0 && y + dy<5 && y + dy>=0)
      {
        x = x + dx;
        y = y + dy;
      }
      Obstacles();
    }
    DecideWhereNext();
    modulo = 0;
  }
}

void Forward(){
  motorL.setSpeed(92);
  motorR.setSpeed(85);

  motorL.run(FORWARD);
  motorR.run(FORWARD);

}


void Obstacles(){
if (WallLeft)
  {
    Map[x][y].Left = true;
  }
  else
  {
    Map[x][y].Left = false;
  }
  if (WallRight)
  {
    Map[x][y].Right = true;
  }
  else
  {
    Map[x][y].Right = false;
  }
  if (WallFront)
  {
    Map[x][y].Front = true;
    Map[x][y].Back = false;
  }
  else
  {
    Map[x][y].Front = false;
    Map[x][y].Back = false;
  }

  Map[x][y].StrucDirection = Direction;
      Serial.print(" x:");
      Serial.print(x);
      Serial.print(" y:");
      Serial.print(y);
      Serial.print(" ");
      Serial.print(Map[x][y].Left);
      Serial.print(" ");
      Serial.print(Map[x][y].Right);
      Serial.print(" ");
      Serial.print(Map[x][y].Front);
      Serial.print(" ");
      Serial.print(Map[x][y].Back);
      Serial.print(" ");
      Serial.print(Map[x][y].StrucDirection);
      Serial.print(" ");
}

void echoCheck() {
  if (sonar[currentSensor].check_timer())
  {
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
    pingResult(currentSensor);
  }
}

void pingResult(uint8_t sensor) {
  modulo = cm[2]%30;
  //Serial.print("Pravy: ");
  //Serial.println(cm[1]);
  //Serial.print("Predni: ");
  //Serial.println(cm[2]);

  if (cm[0] > 13)
  {
    WallLeft = false;
    //Serial.println("Vlevo neni prekazka");
  }
  else if (cm[0] <= 13)
  {
    WallLeft = true;
    //Serial.println("Vlevo je prekazka");
  }

  if(cm[1] > 13)
  {
    WallRight = false;
    //Serial.println(cm[1]);
    //Serial.println("Vpravo neni prekazka");
  }
  else if (cm[1] <= 13){
    WallRight = true;
    //Serial.println(cm[1]);
    //Serial.println("Vpravo je prekazka");
  }

  if(cm[2] > 13)
  {
    WallFront = false;
    WallBack = false;
  }
  else if (cm[2] <= 13)
  {
    WallFront = true;
  }
}

void DecideWhereNext(){

  if(!Map[x][y].Front && Map[x][y].Right )
  {
    Serial.println("Forward");
    Forward();
  }
  else if(!Map[x][y].Front && !Map[x][y].Left && Map[x][y].Right)
  {
    Serial.println("Forward");
    Forward();
  }
  else if(!Map[x][y].Front && !Map[x][y].Right)
  {
    Serial.println("Right");
    TurnRight();
    Forward();
  }
  else if(Map[x][y].Front && Map[x][y].Right && !Map[x][y].Left)
  {
    TurnLeft();
    Serial.println("Left");
    Forward();
  }
  else if (Map[x][y].Front && Map[x][y].Right && !Map[x][y].Left)
  {
    Turn180();
    Serial.println("Turning 180");
    Forward();
  }
} */
