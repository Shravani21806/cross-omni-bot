#include <PS2X_lib.h>
PS2X ps2x;
int M = 0;
byte type = 0;
byte vibrate = 0;
#include <Wire.h>
#include <MPU6050.h>
MPU6050 mpu;
unsigned long timer = 0;
float timeStep = 0.09999;
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
float L3, R3;


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
  RED = ps2x.Button(PSB_RED);
  PINK = ps2x.ButtonPressed(PSB_PINK);
  GREEN = ps2x.Button(PSB_GREEN);
  BLUE = ps2x.ButtonPressed(PSB_BLUE);
  FORWARD = ps2x.ButtonPressed(PSB_PAD_UP);
  BACKWARD = ps2x.ButtonPressed(PSB_PAD_DOWN);
  RIGHT = ps2x.Button(PSB_PAD_RIGHT);
  LEFT = ps2x.Button(PSB_PAD_LEFT);
  L3 = (ps2x.Button(PSB_L3));
  R3 = (ps2x.Button(PSB_R3));

 kp=3;
 kd=6;
  //   if(error>=5 && error<=-5 )
  // {Serial.print("pid");
  //   kp=6.4, kd=30;
  // }
  // else
  // {
    // kp=3,kd=6;
  // }



  error = setpoint - yaw;
  p = error;
  i = i + error;
  d = error - pre_error;
  pre_error = error;
  pid = kp * p + ki * i + kd * d;
  pid = constrain(pid, -100, 100);

  //  FOR joystick
  RY = map(RY, 0, 255, 127, -127);
  RX = map(RX, 0, 255, 127, -127);


  R = sqrt((RX * RX) + (RY * RY));
  theta = atan2(RY, RX) * 180 / 3.142;

  a = R * sin((theta - 45) * 3.142 / 180);
  d = R * sin((135 - theta) * 3.142 / 180);
  c = R * sin((theta - 45) * 3.142 / 180);
  b = R * sin((135 - theta) * 3.142 / 180);


// TASK NO.5] 45 DEGREE
  int g;
  int counter1;
  if (PINK == 1 && g == 0) {
    g = 1;
  }
  if (PINK == 1 && g == 1) {
    counter1++;
    g = 0;
  }
  if (counter1 % 2 == 0) {
    // if (PINK == 1) {
    a = 70 * sin((45 - 45) * 3.142 / 180);
    d = 70 * sin((135 - 45) * 3.142 / 180);
    c = 70 * sin((45 - 45) * 3.142 / 180);
    b = 70 * sin((135 - 45) * 3.142 / 180);
    // }
  }
  if (counter1 % 2 != 0) {
    R = 0;
  }
   if (GREEN == 1) {
    Serial.print("continuous rotation");
    // setpoint = setpoint + 20;
    // setpoint=360;
    pid = 50;
  } 
  
  if (RED == 1) {
    Serial.print("continuous rotation");
    // setpoint = setpoint + 20;
    // setpoint=360;
    pid = -50;
  } 


  // // Switching case of turning
  int gr;
  int counter;
  if (BLUE == 1 && gr == 0) {
    gr = 1;
  }
  if (BLUE == 1 && gr == 1) {
    counter++;
    gr = 0;
  }

  // turning 90 amd 180 left and right
  else if (counter % 2 == 0) {
    if (L1 == 1) {
      setpoint = setpoint + 90;
      Serial.print("90");
    } else if (R1 == 1) {
      setpoint = setpoint - 90;
      Serial.print("-90");
    } else if (L2 == 1) {
      setpoint = setpoint + 180;
      Serial.print("180");
    } else if (R2 == 1) {
      setpoint = setpoint - 180;
      Serial.print("-180");
    }
  }

  // turning 270 amd 360 left and right
  if (counter % 2 != 0) {
    if (L1 == 1) {
      setpoint = setpoint + 270;
      Serial.print("270");
    } else if (R1 == 1) {
      setpoint = setpoint - 270;
      Serial.print("-270");
    } else if (L2 == 1) {
      setpoint = setpoint + 360;
      Serial.print("360");
    } else if (R2 == 1) {
      setpoint = setpoint - 360;
      Serial.print("-360");
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

  Serial.print("   yaw");
  Serial.print(yaw);
  Serial.print("   pid");
  Serial.println(pid);

}
