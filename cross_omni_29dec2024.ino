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

SoftwareSerial SWSerial1(NOT_A_PIN, 44);
SabertoothSimplified ST1(SWSerial1);
SoftwareSerial SWSerial2(NOT_A_PIN, 45);
SabertoothSimplified ST2(SWSerial2);
float R, theta, M, count1 = 0, count2 = 0, count3 = 0, count4 = 0, h = 0, j = 0, k = 0, l = 0, setpoint = 0, pre_error, proportional, integral, derivative, error, pid, L1, L2, R1, R2, LY, LX, a, b, c,d, A, B, C, D;
float kp = 0, ki = 0, kd = 0;

float motor_A = 0;
float motor_B = 0;
float motor_C = 0;
float motor_D = 0;
int N;
int RED, count, counter;
int GREEN;
int BLUE, FORWARD, BACKWARD, RIGHT, LEFT;
void setup() {
  M = ps2x.config_gamepad(22, 23, 24, 25, true, true);
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



  timer = millis();
  Vector norm = mpu.readNormalizeGyro();
  yaw = yaw + norm.ZAxis * timeStep;
  delay((timeStep * 1000) - (millis() - timer));

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
  FORWARD = ps2x.Button(PSB_PAD_UP);
  BACKWARD = ps2x.Button(PSB_PAD_DOWN);
  RIGHT = ps2x.Button(PSB_PAD_RIGHT);
  LEFT = ps2x.Button(PSB_PAD_LEFT);
  FORWARD = ps2x.Analog(PSAB_PAD_UP);
  BACKWARD = ps2x.Analog(PSAB_PAD_DOWN);
  RIGHT = ps2x.Analog(PSAB_PAD_RIGHT);
  LEFT = ps2x.Analog(PSAB_PAD_LEFT);


  // if (error >= 12 && error <= -12) {
  //   kp = 12, kd = 28;
  // } else {
    kp = 11 , kd = 40;
  // }


  int LY = ps2x.Analog(PSS_LY);
  LY = map(LY, 0, 255, 127, -127);

  int LX = ps2x.Analog(PSS_LX);
  LX = map(LX, 0, 255, -127, 127);

  R = sqrt((LX * LX) + (LY * LY));
  theta = atan2(LY, LX) * 180 / 3.142;

  a = R * sin((theta - 45) * 3.142 / 180);
  b = R * sin((135 - theta) * 3.142 / 180);
  c = R * sin((theta - 45) * 3.142 / 180);
  d = R * sin((135 - theta) * 3.142 / 180);


  error = setpoint - yaw;
  proportional = error;
  integral = integral + error;
  derivative = error - pre_error;
  pre_error = error;
  pid = kp * proportional + ki * integral + kd * derivative;

  A = a + pid;
  B = b + pid;
  C = c + pid;
  D = d + pid;

  motor_A = constrain(A, -120, 120);
  motor_B = constrain(B, -120, 120);
  motor_C = constrain(C, -120, 120);
  motor_D = constrain(D, -120, 120);
  ST1.motor(1, motor_A);
  ST1.motor(2, motor_B);
  ST2.motor(1, motor_C);
  ST2.motor(2, motor_D);

  Serial.print("   pid  ");
  Serial.print(pid);
  Serial.print("     Yaw = ");
  Serial.print(yaw);
  // Serial.print("   motor_D  ");
  // Serial.print(motor_D);
  //  Serial.print("   D  ");
  // Serial.print(D);
  //   Serial.print("   motor_C  ");
  // Serial.print(motor_C);
    Serial.print("   error  ");
  Serial.print(error);
    Serial.print("   LX  ");
  Serial.print(LX);
  Serial.print("   LY  ");
  Serial.println(LY);
}
