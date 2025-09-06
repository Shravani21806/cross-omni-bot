#include <PS2X_lib.h>
PS2X ps2x;
int M = 0;
byte type = 0;
byte vibrate = 0;
#include <Wire.h>
#include <MPU6050.h>
MPU6050 mpu;
unsigned long timer = 0;
float timeStep = 0.0465;
float yaw = 0;
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

SoftwareSerial SWSerial1(NOT_A_PIN, 10);
SabertoothSimplified ST1(SWSerial1);
SoftwareSerial SWSerial2(NOT_A_PIN, 3);
SabertoothSimplified ST2(SWSerial2);
float kp = 0, ki = 0, kd = 0;
float R, theta, setpoint = 0, pre_error, p, i, d, error, pid, L1, L2, R1, R2, LY, LX, RX, RY, a, b, c, A, B, C, D;
float RED;
float GREEN = 0, PINK = 0;
float BLUE, FORWARD, BACKWARD, RIGHT, LEFT;
int w;
int counter1 = 0;
int y;
float  L3,R3;

void setup() {
  SWSerial1.begin(9600);
  SWSerial2.begin(9600);

  M = ps2x.config_gamepad(9, 7, 6, 8, true, true);
  Serial.begin(57600);
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(1);
}

void loop() {
  // IMU
  timer = millis();
  Vector norm = mpu.readNormalizeGyro();
  yaw = yaw + norm.ZAxis * timeStep;
  delay((timeStep * 1000) - (millis() - timer));

  //  READING GAMEPAD

  ps2x.read_gamepad();
  L1 = ps2x.ButtonPressed(PSB_L1);
  R1 = ps2x.ButtonPressed(PSB_R1);
  L2 = ps2x.ButtonPressed(PSB_L2);
  R2 = ps2x.ButtonPressed(PSB_R2);
  LY = ps2x.Analog(PSS_LY);
  LX = ps2x.Analog(PSS_LX);
  RY = ps2x.Analog(PSS_RY);
  RX = ps2x.Analog(PSS_RX);
  RED = ps2x.ButtonPressed(PSB_RED);
  PINK = ps2x.ButtonPressed(PSB_PINK);
  GREEN = ps2x.Button(PSB_GREEN);
  BLUE = ps2x.ButtonPressed(PSB_BLUE);
  FORWARD = ps2x.Button(PSB_PAD_UP);
  BACKWARD = ps2x.Button(PSB_PAD_DOWN);
  RIGHT = ps2x.Button(PSB_PAD_RIGHT);
  LEFT = ps2x.Button(PSB_PAD_LEFT);
  L3=(ps2x.Button(PSB_L3));
    R3=(ps2x.Button(PSB_R3));
  


  // GAINS
  // kp = 3;
  // kd = 7;

    if(error>=5 && error<=-5 )
  {Serial.print("pid");
    kp=1, kd=7.5;
  }
  else
  {
    kp=2.7,kd=7.2;
  }

  error = setpoint - yaw;
  p = error;
  i = i + error;
  d = error - pre_error;
  pre_error = error;
  pid = kp * p + ki * i + kd * d;
  pid = constrain(pid, -100, 100);

  //  FOR joystick
  RY = map(RY, 0, 255, 127, -127);
  RX = map(RX, 0, 255, -127, 127);
  

  R = sqrt((RX * RX) + (RY * RY));
  theta = atan2(RY, RX) * 180 / 3.142;
  
  // R = sqrt((LX * LX) + (LY * LY));
  // theta = atan2(LY, LX) * 180 / 3.142;
    // LY=constrain(LY, -70, 70);
    //  LX=constrain(LX, -70, 70);

  // if(L3==1){
  //   RY=constrain(RY, -70, 70);
  //    RX=constrain(LX, -70, 70);
  // }
  //   if(R3==1){
  //   LY=constrain(LY, -70, 70);
  //    LX=constrain(LX, -70, 70);
  // }

   //SPEED INCRESE

  if (PINK == 1 && w == 0) {
    w = 1;
    // Serial.print("w1");
  }
   if (PINK == 0 && w == 1) {
    counter1++;
    w = 0;
 
    Serial.print("w2");
  }
    if (RED == 1 && y == 0) {
    y = 1;
    // Serial.print("y1");
  }
  if (RED == 0 && y == 1) {
    counter1--;
    y = 0;
    Serial.print("y2");
  }
 if (counter1 == 1) {
    R = 25;
    Serial.print("w2");
  }
 else  if (counter1 == 2) {
    R = 50;
    Serial.print("w3");
  }
 else  if (counter1 == 3) {
    R = 75;
    Serial.print("w4");
  }
  else if (counter1 == 4) {
    R = 100;
    Serial.print("w5");
    // counter1=1;
  }
    else if (counter1 == 5) {
    R = 127;
    Serial.print("w5");
    // counter1=1;
  }
  else if(counter1==6) {
    counter1=5;
  }
    else if(counter1==-1) {
    counter1=0;
  }

  a = R * sin((theta - 45) * 3.142 / 180);
  d = R * sin((135 - theta) * 3.142 / 180);
  c = R * sin((theta - 45) * 3.142 / 180);
  b = R * sin((135 - theta) * 3.142 / 180);

 if(FORWARD==1)
  {Serial.print("guru");
 a=100*sin((90-45)*3.142/180);
 b=100*sin((135-90)*3.142/180);
 c=100*sin((90-45)*3.142/180);
 d=100*sin((135-90)*3.142/180); 
  }
  if(BACKWARD==1)
  {Serial.print("GURU");
 a=100*sin((270-45)*3.142/180);
 b=100*sin((135-270)*3.142/180);
 c=100*sin((270-45)*3.142/180);
 d=100*sin((135-270)*3.142/180); 
  }
    if(RIGHT==1)
  {Serial.print("GURU");
 a=-(100*sin((0-45)*3.142/180));
 b=-(100*sin((135-0)*3.142/180));
 c=-(100*sin((0-45)*3.142/180));
 d=-(100*sin((135-0)*3.142/180)); 
  }
     if(LEFT==1)
  {Serial.print("GURU");
 a=-(100*sin((180-45)*3.142/180));
 b=-(100*sin((135-180)*3.142/180));
 c=-(100*sin((180-45)*3.142/180));
 d=-(100*sin((135-180)*3.142/180)); 
  }
   if (GREEN == 1) {
    Serial.print("increment");
    // setpoint = setpoint + 20;
    // setpoint=360;
    pid=50;
  }
  else if (GREEN==0){
    setpoint=0;
    // pid=0;
  }

  

  // Switching case of turning
  int gr;
  int counter;
  if (BLUE == 1 && gr == 0) {
    gr = 1;
  }
  if (BLUE == 1  && gr == 1) {
    counter++;
    gr = 0;
  }

  // turning 90 amd 180 left and right
  else if (counter % 2 == 0) {
    if (L1 == 1) {
      setpoint = setpoint + 75;
      Serial.print("hiiiiii");
    }
    else if (R1 == 1) {
      setpoint = setpoint - 75;
      Serial.print("hwwwwwwwwwww");
    }
    else if (L2 == 1) {
      setpoint = setpoint + 150;
      Serial.print("hqqqqqqqqqqqqq");
    }
    else if (R2 == 1) {
      setpoint = setpoint - 150;
      Serial.print("hrrrrrrrrrrrr");
    }
  }

  // turning 270 amd 360 left and right
  else if (counter % 2 != 0) {
    if (L1 == 1) {
      setpoint = setpoint + 225;
      Serial.print("haaaaaaaaaa");
    }
    else if (R1 == 1) {
      setpoint = setpoint - 225;
      Serial.print("hggggggggggg");
    }
    else if (L2 == 1) {
      setpoint = setpoint + 300;
      Serial.print("huuuuuuuuuuu");
    }
    else if (R2 == 1) {
      setpoint = setpoint - 300;
      Serial.print("hpppppppppppp");
    }
  }

  A = a - pid;
  B = b - pid;
  C = c + pid;
  D = d + pid;

  A = constrain(A, -100, 100);
  B = constrain(B, -100, 100);
  C = constrain(C, -100, 100);
  D = constrain(D, -100, 100);

  ST1.motor(1, -A);
  ST1.motor(2, B);
  ST2.motor(1, C);
  ST2.motor(2, D);

  // Serial.print("   COUNTER");
  // Serial.print(counter);
  Serial.print("   yaw");
  Serial.print(yaw);
  Serial.print("   pid");
  Serial.println(pid);
  
  Serial.print("   setpoint");
  Serial.print(setpoint);
  Serial.print("   L2");
  Serial.print(L2);
  // Serial.print("   RX");
  // Serial.print(RX);
  // Serial.print("   RY");
  // Serial.print(RY);
  //  Serial.print("   LX");
  // Serial.print(LX);
  // Serial.print("   LY");
  // Serial.println(LY);
  // Serial.print("  BLUE");
  // Serial.println(BLUE);
}
