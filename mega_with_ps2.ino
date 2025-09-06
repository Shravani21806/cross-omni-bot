#include <PS2X_lib.h>  //for v1.6

/*
  right now, the library does NOT support hot pluggable controllers, meaning 
  you must always either restart your Arduino after you conect the controller, 
  or call config_gamepad(pins) again after connecting the controller.
*/
float RX,LX,LY,RY;
float R,theta;
float a,b,c,d;
// create PS2 Controller Class
PS2X ps2x;
int error = 0; 
byte type = 0;
byte vibrate = 0;

void setup()
{
  Serial.begin(57600);
  // CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************
  // setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(22,23,24,25, true, true);}
void loop(){
    ps2x.read_gamepad();
    
  LY = ps2x.Analog(PSS_LY);
  LX = ps2x.Analog(PSS_LX);
  RY = ps2x.Analog(PSS_RY);
  RX = ps2x.Analog(PSS_RX);        //read controller 
   R = sqrt((LX * LX) + (LY * LY));
  theta = atan2(LY, LX) * 180 / 3.142;
  
  LY = map(LY, 0, 255, 127, -127);
  LX = map(LX, 0, 255, -127, 127);
     
     
  a = R * sin((theta - 45) * 3.142 / 180);
  b = R * sin((135 - theta) * 3.142 / 180);
  c = R * sin((theta - 45) * 3.142 / 180);
  d = R * sin((135 - theta) * 3.142 / 180);
  
  Serial.print("RX ");
  Serial.print(LX);
  Serial.print("  RY");
  Serial.println(LY);
  delay(50);
}

