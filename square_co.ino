#include <PS2X_lib.h>
PS2X ps2x; 
#include <Wire.h>
#include <MPU6050.h>
MPU6050 mpu;
unsigned long timer = 0;
float timeStep = 0.01;
float yaw = 0;

#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

SoftwareSerial SWSerial1(NOT_A_PIN, 10);
SabertoothSimplified ST1(SWSerial1);
SoftwareSerial SWSerial2(NOT_A_PIN, 3);
SabertoothSimplified ST2(SWSerial2);
float R, theta, M, count1 = 0, count2 = 0, count3 = 0, count4 = 0, h = 0, j = 0, k = 0, l = 0, setpoint = 0, pre_error, p, i, d, error, pid, L1, L2, R1, R2, LY, LX, a, b, c, A, B, C, D;
float kp = 0, ki = 0, kd = 0;
int N,RX,RY;
int RED,counter1;
int GREEN,PINK;
int s1=0;
int BLUE,FORWARD,BACKWARD,RIGHT,LEFT;
 int w;
  int counter;
void setup() {
  M = ps2x.config_gamepad(9,7,6,8, true, true);
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
 if(error>=12 && error<=-12 )
  {
    kp=12, kd=28;
  }
  else
  {
    kp=4,kd=20;
  }
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
  pid = constrain(pid, -120, 120);

    
  //  Serial.print("pid ");
  //  Serial.print(pid);
  ps2x.read_gamepad();
  LX = ps2x.Analog(PSS_LX);
  LY = ps2x.Analog(PSS_LY);
  RX = ps2x.Analog(PSS_RX);
  RY = ps2x.Analog(PSS_RY);
  L1 = ps2x.ButtonPressed(PSB_L1);
  R1 = ps2x.ButtonPressed(PSB_R1);
  L2 = ps2x.ButtonPressed(PSB_L2);
  R2 = ps2x.ButtonPressed(PSB_R2);
  LY = ps2x.Analog(PSS_LY);
  LX = ps2x.Analog(PSS_LX);
  RED = ps2x.ButtonPressed(PSB_RED);
  PINK = ps2x.ButtonPressed(PSB_PINK);
  GREEN = ps2x.ButtonPressed(PSB_GREEN);
  BLUE = ps2x.ButtonPressed(PSB_BLUE);
    FORWARD=ps2x.Button(PSB_PAD_UP);
  BACKWARD=ps2x.Button(PSB_PAD_DOWN);
  RIGHT=ps2x.Button(PSB_PAD_RIGHT);
  LEFT=ps2x.Button(PSB_PAD_LEFT);
  

  RX = map(RX,0,255,127,-127);
  RY = map(RY,0,255,127,-127);
  LX = map(LX,0,255,127,-127);
  LY = map(LY,0,255,127,-127);
  if(RX>-5 && RX<5){
    RX=0;
  }
  if(RY>-5 && RY<5){
    RY=0;
  }
  R=sqrt((RX*RX)+(RY*RY));

  theta=atan2(RY,RX)* 180 / 3.142;

  if(PINK==1 && w==0){
    w=1;
  }
  if(PINK==1 && w==1){
   counter++;
    w=0;
  }
if(counter%2!=0){
  counter1++;
}
if (counter1>0 ){
  R=50;
  theta=90;
  
}
 if (counter1>=200  ){
  R=50;
  theta=0;
  
 
}
 if ( counter1>=400 ){
  R=50;
  theta=-90;
  

}
 if (counter1>=600  ){
  R=50;
  theta=180;

}
if(counter1>=800){
  // counter1=0;
  R=0;
}
if(counter%2==0){
  counter1=0;
}






  a = R * sin((theta - 45) * 3.142 / 180);
  b = R * sin((135 - theta) * 3.142 / 180);
  c = R * sin((theta - 45) * 3.142 / 180);
  d = R * sin((135 - theta) * 3.14 /180);

   A = -a + pid;
  B = b - pid;
  C = c + pid;
  D = -d - pid;
  A = constrain(A, -120, 120);
  B = constrain(B, -120, 120);
  C = constrain(C, -120, 120);
  D = constrain(D, -120, 120);
  ST1.motor(1, A);
  ST1.motor(2, B);
  ST2.motor(1, C);
  ST2.motor(2, D);

  // Serial.print("   counter=");
  // Serial.print(counter);
  //   Serial.print("   counter1=");
  // Serial.print(counter1);
  //    Serial.print("   theta=");
  // // Serial.print(theta);
  //     Serial.print("   R=");
  // Serial.println(R);

  
}

