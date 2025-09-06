#include <PS2X_lib.h>

PS2X ps2x;

int M = 0;
byte type = 0;
byte vibrate = 0;
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

SoftwareSerial SWSerial1(NOT_A_PIN, 3);
SabertoothSimplified ST1(SWSerial1);
SoftwareSerial SWSerial2(NOT_A_PIN, 10);
SabertoothSimplified ST2(SWSerial2);

#include <Wire.h>
#include <MPU6050.h>
MPU6050 mpu;
unsigned long timer = 0;
float timeStep = 0.0465;
float yaw = 0;


float R, theta, count1 = 0, count2 = 0, count3 = 0, count4 = 0, h = 0, j = 0, k = 0, l = 0, setpoint = 0, pre_error, proportional,integral,derivative,error, pid, L1, L2, R1, R2, LY, LX, RX, RY, a, b, c, d,A, B, C, D;
float kp = 0, ki = 0, kd = 0;
float RED;
float GREEN = 0;
float pink = 0;
float BLUE, FORWARD, BACKWARD, RIGHT, LEFT;
float y;


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
  pink=PINK = ps2x.ButtonPressed(PSB_PINK);
  GREEN = ps2x.Button(PSB_GREEN);
  BLUE = ps2x.ButtonPressed(PSB_BLUE);
  FORWARD = ps2x.Analog(PSAB_PAD_UP);
  BACKWARD = ps2x.Analog(PSAB_PAD_DOWN);
  RIGHT = ps2x.Analog(PSAB_PAD_RIGHT);
  LEFT = ps2x.Analog(PSAB_PAD_LEFT);

  //  FOR joystick
  RY = map(RY, 0, 255, 127, -127);
  RX = map(RX, 0, 255, -127, 127);

  R = sqrt((RX * RX) + (RY * RY));
  theta = atan2(RY, RX)*180/3.142;

  // // FOR button
  // R = sqrt((FORWARD * FORWARD) + (BACKWARD * BACKWARD));
  // theta = atan2(BACKWARD, FORWARD);
  
  // // GAINS
  kp = 2.4;
  kd = 6;

  //SPEED INCRESE && decrease
  int w;
  int counter1;
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
    counter1++;
    y = 0;
    // Serial.print("y2");
  }
  if (counter1 == 1) {
    R = 25;
    Serial.print("w2");
  }
  if (counter1 == 2) {
    R = 50;
    Serial.print("w3");
  }
  if (counter1 == 3) {
    R = 75;
    Serial.print("w4");
  }
  if (counter1 == 4) {
    R = 100;
    Serial.print("w5");
    // counter1=1;
  }
  if (counter1 == 4) {
    R = 100;
    Serial.print("w5");
    // counter1=1;
  }
  // contiunes rotoition
  if (GREEN == 1) {
    pid = 20;
  } else {
    pid = 0;
  }

  // Switching case of turning
  int gr;
  int counter;
  if (BLUE == 1 && gr == 0) {
    gr = 1;
  }
  if (BLUE == 0 && gr == 1) {
    counter++;
    gr = 0;
  }
  // turning 90 amd 180 left and right
  if (counter % 2 == 0) {
    if (L1 == 1) {
      setpoint = setpoint + 75;
      Serial.print("hiiiiii");
    }
    if (R1 == 1) {
      setpoint = setpoint - 75;
      Serial.print("hwwwwww");
    }
    if (L2 == 1) {
      setpoint = setpoint + 150;
      Serial.print("hqqqqqqqqqqqqq");
    }
    if (R2 == 1) {
      setpoint = setpoint - 150;
      Serial.print("hrrrrrrrrrrrr");
    }
  }
  // turning 270 amd 360 left and right
  if (counter % 2 != 0) {
    if (L1 == 1) {
      setpoint = setpoint + 225;
      Serial.print("haaaaaaaaaa");
    }
    if (R1 == 1) {
      setpoint = setpoint - 225;
      Serial.print("hggggggggggg");
    }
    if (L2 == 1) {
      setpoint = setpoint + 300;
      Serial.print("huuuuuuuuuuu");
    }
    if (R2 == 1) {
      setpoint = setpoint - 300;
      Serial.print("hpppppppppppp");
    }
  }

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

  Serial.print("    YAW  ");
  Serial.print(yaw);
  // Serial.print("    L1");
  // Serial.print(L1);
  // Serial.print("  R1 ");
  // Serial.print(R1);
  // Serial.print("   L2 ");
  // Serial.print(L2);
  // Serial.print("   R2 ");
  // Serial.print(R2);
  // Serial.print("  BLUE");
  // Serial.print(BLUE);
  // Serial.print("   UP ");
  // Serial.print(FORWARD);
  // Serial.print("  DOWN");
  // Serial.print(BACKWARD);
  // Serial.print("   RIGHT  ");
  // Serial.print(RIGHT);
  // Serial.print("   LEFT ");
  // Serial.print(LEFT);
  // Serial.print("   PINK");
  // Serial.print(PINK);
  // Serial.print("   RED ");
  // Serial.print(RED);
  // Serial.print("   COUNTER");
  // Serial.println(  counter1);
}