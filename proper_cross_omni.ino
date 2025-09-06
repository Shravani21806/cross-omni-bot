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

SoftwareSerial SWSerial1(NOT_A_PIN, 38);
SabertoothSimplified ST1(SWSerial1);
SoftwareSerial SWSerial2(NOT_A_PIN, 39);
SabertoothSimplified ST2(SWSerial2);
float R, theta, M, count1 = 0, count2 = 0, count3 = 0, count4 = 0, h = 0, j = 0, k = 0, l = 0, setpoint = 0, pre_error, p, i, d, error, pid, L1, L2, R1, R2, LY, LX, a, b, c, A, B, C, D;
float kp = 0, ki = 0, kd = 0;
int N;
int RED,count,counter;
int GREEN;
int BLUE,FORWARD,BACKWARD,RIGHT,LEFT;
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
  L1 = ps2x.ButtonPressed(PSB_L1);
  R1 = ps2x.ButtonPressed(PSB_R1);
  L2 = ps2x.ButtonPressed(PSB_L2);
  R2 = ps2x.ButtonPressed(PSB_R2);
  LY = ps2x.Analog(PSS_LY);
  LX = ps2x.Analog(PSS_LX);
  RED = ps2x.Button(PSB_RED);
  GREEN = ps2x.Button(PSB_GREEN);
  BLUE = ps2x.ButtonPressed(PSB_BLUE);
    FORWARD=ps2x.Button(PSB_PAD_UP);
  BACKWARD=ps2x.Button(PSB_PAD_DOWN);
  RIGHT=ps2x.Button(PSB_PAD_RIGHT);
  LEFT=ps2x.Button(PSB_PAD_LEFT);
FORWARD=ps2x.Analog(PSAB_PAD_UP);
BACKWARD=ps2x.Analog(PSAB_PAD_DOWN);
RIGHT=ps2x.Analog(PSAB_PAD_RIGHT);
LEFT=ps2x.Analog(PSAB_PAD_LEFT);

 if (GREEN == 1) {
    Serial.print("continuous rotation");
    setpoint = setpoint + 5;
    // setpoint=360;
    // pid = 50;
  } 
  // else if(GREEN==0)
  // {
  //   pid=0;
  // }
  // // Switching case of turning
  int gr;
  int counter;
  if (BLUE== 1 && gr == 0) {
    gr = 1;
  }
  if (BLUE== 1 && gr == 1) {
    counter++;
    gr = 0;
  }

  if (counter % 2 == 0) {
    if (L1 == 1) {
      setpoint = setpoint + 154.5;
      Serial.print("270");
    }
     else if (R1 == 1) {
      setpoint = setpoint - 154.5;
      Serial.print("-270");
    }
     else if (L2 == 1) {
      setpoint = setpoint + 206;
      Serial.print("360");
    } 
    else if (R2 == 1) {
      setpoint = setpoint - 206;
      Serial.print("-360");
    }   
  }

  if (counter % 2 != 0) { 
    if (L1 == 1) {
      setpoint = setpoint + 51.5;
      Serial.print("90");
    } else if (R1 == 1) {
      setpoint = setpoint - 51.5;
      Serial.print("-90");
    } else if (L2 == 1) {
      setpoint = setpoint + 103;
      Serial.print("180");
    } else if (R2 == 1) {
      setpoint = setpoint - 103;
      Serial.print("-180");
    }
  }
 
   int LY= ps2x.Analog(PSS_LY);
   LY = map(LY,0,255,127,-127);
  int  LX = ps2x.Analog(PSS_LX);
   LX = map(LX,0,255,127,-127);
   R=sqrt((LX*LX)+(LY*LY));
  theta=atan2(LY,LX)* 180 / 3.142;

  a = R * sin((theta - 45) * 3.142 / 180);
  b = R * sin((135 - theta) * 3.142 / 180);
  c = R * sin((theta - 45) * 3.142 / 180);
  e = R * sin((135 - theta) * 3.142 / 180);

  A = -a + pid;
  B = b - pid;
  C = c + pid;
  D = -e - pid;
  A = constrain(A, -120, 120);
  B = constrain(B, -120, 120);
  C = constrain(C, -120, 120);
  D = constrain(D, -120, 120);
  ST1.motor(1, A);
  ST1.motor(2, B);
  ST2.motor(1, C);
  ST2.motor(2, D);

  Serial.print("   pid  ");
  Serial.print(pid   );
  Serial.print("     Yaw = ");
  Serial.print(yaw);
     Serial.print("   lx  ");
  Serial.print(LX);
   Serial.print("   LY  ");
  Serial.println(LY);
}

