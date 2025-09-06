#include <PS2X_lib.h>
#define PS2_DAT 13  //14
#define PS2_CMD 11  //15
#define PS2_SEL 10  //16
#define PS2_CLK 12  //17

PS2X ps2x;
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
float R, theta, M, setpoint = 0, pre_error, p, i, d, error, pid, L1, L2, R1, R2, LY, LX, a, b, c, A, B, C, D;
float kp = 0, ki = 0, kd = 0;
int RED;
int GREEN;
int BLUE,FORWARD,BACKWARD,RIGHT,LEFT;
int count;
int PINK;
int j;
void setup() {
  M = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT);
  SWSerial1.begin(9600);

  SWSerial2.begin(9600);

  Serial.begin(115200);
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(1);
}

void loop() {
  kp = 2.4;
  kd = 6;
  // if(error>=5 && error<=-5 )
  // {Serial.print("pid");
  //   kp=1, kd=12;
  // }
  // else
  // {
  //   kp=2.7,kd=7.5;
  // }
  timer = millis();
  Vector norm = mpu.readNormalizeGyro();
  yaw = yaw + norm.ZAxis * timeStep;
  delay((timeStep * 1000) - (millis() - timer));

  error = setpoint - yaw;
  p = error;
  i = i + error;
  d = error - pre_error;
  pre_error = error;
  pid = kp * p + ki * i + kd * d;
  pid = constrain(pid, -100, 100);
  //  Serial.print("pid ");
  //  Serial.print(pid);
  ps2x.read_gamepad();
  L1 = ps2x.ButtonPressed(PSB_L1);
  R1 = ps2x.ButtonPressed(PSB_R1);
  L2 = ps2x.ButtonPressed(PSB_L2);
  R2 = ps2x.ButtonPressed(PSB_R2);
  LY = ps2x.Analog(PSS_LY);
  LX = ps2x.Analog(PSS_LX);
  RED = ps2x.Button(PSB_RED);
  PINK= ps2x.Button(PSB_PINK);
  GREEN = ps2x.Button(PSB_GREEN);
  BLUE = ps2x.ButtonPressed(PSB_BLUE);
  FORWARD=ps2x.Button(PSB_PAD_UP);
  BACKWARD=ps2x.Button(PSB_PAD_DOWN);
  RIGHT=ps2x.Button(PSB_PAD_RIGHT);
  LEFT=ps2x.Button(PSB_PAD_LEFT);
  //  int RY= ps2x.Analog(PSS_RY);
  //  RY = map(RY,0,255,127,-127);
  // int  RX = ps2x.Analog(PSS_RX);
  //  RX = map(RX,0,255,-127,127);
  //  R=sqrt((RX*RX)+(RY*RY));
  // theta=atan2(RY,RX);

  a = R * sin((theta -135) * 3.142 / 180);
  b = R * sin((45 - theta) * 3.142 / 180);
  c = R * sin((theta - 135) * 3.142 / 180);
  d = R * sin((45 - theta) * 3.142 / 180);
  A = a + pid;
  B = b - pid;
  C = c + pid;
  D = d + pid;
  A = constrain(A, -100, 100);
  B = constrain(B, -100, 100);
  C = constrain(C, -100, 100);
  D = constrain(D, -100, 100);
  ST1.motor(1, A);
  ST1.motor(2, B);
  ST2.motor(1, C);
  ST2.motor(2, D);
  if(FORWARD==1){
 a=100*sin((90-45)*3.142/180);
 b=100*sin((135-90)*3.142/180);
 c=100*sin((90-45)*3.142/180);
 d=100*sin((135-90)*3.142/180); 
  }
    if(BACKWARD==1){
 a=100*sin((270-45)*3.142/180);
 b=100*sin((135-270)*3.142/180);
 c=100*sin((270-45)*3.142/180);
 d=100*sin((135-270)*3.142/180); 
  }
    if(RIGHT==1){
 a=100*sin((0-45)*3.142/180);
 b=100*sin((135-0)*3.142/180);
 c=100*sin((0-45)*3.142/180);
 d=100*sin((135-0)*3.142/180); 
  }
     if(LEFT==1){
 a=100*sin((180-45)*3.142/180);
 b=100*sin((135-180)*3.142/180);
 c=100*sin((180-45)*3.142/180);
 d=100*sin((135-180)*3.142/180); 
  }

   if (PINK==1){
    setpoint=setpoint++;
   }
  if (GREEN == 1) {
    Serial.print("increment");
    // setpoint = setpoint + 20;
    // setpoint=360;
    pid=500;
  }
  else{
    pid=0;
    }

  if (BLUE == 1 && j == 0) {
    j = 1;
  }
  else if (BLUE == 0 && j == 1) {
    count=1;
    j = 0;
  }

  if (count % 2 == 0) {
  if(L1 == 1){
    setpoint=setpoint+90;
  }
  if(R1 == 1){
    setpoint=setpoint-90;
  }
  if(L2 == 1){
    setpoint=setpoint+180;
  }
  if(R2 == 1){
    setpoint=setpoint-180;
  }
}
  if (count % 2 != 0) {
    if(L1 == 1){
    setpoint=setpoint+270;
  }
  if(R1 == 1){
    setpoint=setpoint-270;
  }
  if(L2 == 1){
    setpoint=setpoint+360;
  }
  if(R2 == 1){
    setpoint=setpoint-360;
  }
  }
   
  Serial.print("  BLUE ");
  Serial.print(BLUE);
  Serial.print("  FORWARD ");
  Serial.print(FORWARD);
  Serial.print("  BACKWARD ");
  Serial.print(BACKWARD);
  Serial.print("    pid   ");
  Serial.print(pid);
  Serial.print(" Yaw = ");
  Serial.println(yaw);
}

