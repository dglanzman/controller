//
// University of Pennsylvania
// ESE 505 - 2017A
// Project 2
// OneTwoRedBlue_Automatic.ino
//
// Starter Code for Automatic Mode
// No Joystick -- Desired Ball Position Advanced Automatically
//
// Headers contain device drivers
// angle braces indicate Arduino library driver files
// quotes indicate locally stored driver files
//
#include <SPI.h>  
#include "Pixy.h"
#include <Servo.h>
//
// YOU MAY CHANGE THESE TO MATCH YOUR DEVICE!
//
const double xTargetmm[4] = {-30.0, 34.0, -58.0, 60.0};

//
// define target constants
// (yucky global variables)
//
const byte ONE = 0;
const byte TWO = 1;
const byte RED = 2;
const byte BLUE = 3;
const byte TARGETS = 4;

//
// Pixy is an object type defined in Pixy header file
// Servo is an object type defined in Servo header file
//
Pixy PingPongPixy;
Servo PingPongServo;

//
// here are the pins used
//
const int PingPongServoPin = 3;
const int JoystickButtonPin = A2;
const int REDPin = 4;
const int COMPin = 5;
const int BLUEPin = 6;
const int GREENPin = 7;
//
// time step size
//
double deltaT = 0.021; // time between digital updates (20ms = PIXY & Servo update rates)
//
// servo constants -- DO NOT CHANGE THESE
//
const int ServoCenter_us = 1380;
const double ServoScale_us = 8.0;    // micro-seconds per degree

//////////////////////////////////////////////////////////////////
void setup()
{
//
// Serial baud rate -- fast rate used to allow data logging
//
  Serial.begin(57600);
//
// Attach the Servo to the assigned pin
//
  PingPongServo.attach(PingPongServoPin);
  delay(500);
//
// initialize joystick button pin
//
  pinMode(JoystickButtonPin, INPUT);
//
// initialize the Pixy camera
//
  PingPongPixy.init();
}

void loop()
{ 

//////////////////////////////////////////////////////////////////
// check joystick button
  boolean JoystickButtonPress = false;
  JoystickButtonPress = digitalRead(JoystickButtonPin);

//////////////////////////////////////////////////////////////////
// get the ball position
// negative value returned from Check --> no ball found
// use Last value when no ball found (best guess)
//
  static int xBallRawLast = 2;  // start with max left position as indication of failed PIXY read
  int xBallRaw;
  xBallRaw = PingPongCheck();
  if (xBallRaw < 0)
  {
    xBallRaw = xBallRawLast;
  }
  else
  {
    xBallRawLast = xBallRaw;
  }
//
// convert distance in mm from center of camera field of view
//
  double xBallmm = 0.0;
  xBallmm = -111.0 + (222.0*xBallRaw)/319.0;  // 319 = max pixel value in PIXY camera

//////////////////////////////////////////////////////////////////
// update the desired position
//
  static byte currentTarget = ONE;
  static double cycleTime = 0.0;
  static double xDesiredmm = xTargetmm[ONE];
  static double xCredit = 0.0;
  
  double xDistancemm = abs(xBallmm-xDesiredmm);
  cycleTime += deltaT;
//
// keep track of time spent on target
// joystick button over-rides proximity for debugging
//
  if (xDistancemm < 2.5 || JoystickButtonPress) {
    // very close = accumulate "proximity credit" at fast rate
    xCredit += 1.0*deltaT;
    if (JoystickButtonPress) {cycleTime += 1000.0;}
  }
  else if (xDistancemm < 5.0) {
    // nearby = neither accumulate nor reset credit
  }
  else {
    // far = reset credit
    xCredit = 0.0;
  }
  if (xCredit > 2.0) { // xCredit = total time on target
    xCredit = 0.0;
    currentTarget = (currentTarget+1) % TARGETS;
    if (currentTarget == ONE) {
      Serial.print("Cycle Time = "); Serial.print(cycleTime,1); Serial.println(" sec"); delay(5000);
      cycleTime = 0.0;
    }
    xDesiredmm = xTargetmm[currentTarget];
  }
  Serial.print(cycleTime,2);
  Serial.print(", ");
  Serial.print(xCredit,2);
  Serial.print(", ");
  
//////////////////////////////////////////////////////////////////
// call the control subroutine
//
 double ServoDeg = 0.0;
 ServoDeg = ServoControlLaw(xDesiredmm, xBallmm);

//////////////////////////////////////////////////////////////////
// convert ServoDeg to MicroSeconds
// then send the command to the servo
// smaller values of t_us tilt right (positive sense)
// DON'T CHANGE LIMITS IN CONSTRAIN FUNCTION -- CAN DAMAGE SERVO
//
  double t_us;
  t_us = constrain(ServoCenter_us - ServoScale_us * ServoDeg, 1000, 1800);
  PingPongServo.writeMicroseconds(t_us);

//////////////////////////////////////////////////////////////////
// now send serial information
//
  Serial.print(xDesiredmm,1);
  Serial.print(',');
  Serial.print(xBallmm,1);
  Serial.print(',');
  Serial.println(ServoDeg,1);

//////////////////////////////////////////////////////////////////
// force constant frame rate
//
  static unsigned long microsLast = 0; // elapsed time at end of last call to loop

  while (micros() < microsLast + deltaT*1000000);  // wait for deltaT since last time through
  microsLast = micros(); // save value for next time through
}

//////////////////////////////////////////////////////////////////
// modified from online library
// assumes only 1 block is reported
// that block must be the Ping Pong Ball
//
word PingPongCheck()
{
  int xpos=-1;
  uint16_t nblocks;

  nblocks = PingPongPixy.getBlocks();  
  if (nblocks)
  {
    xpos = PingPongPixy.blocks[0].x;
  }
  
  return xpos;
}

//////////////////////////////////////////////////////////////////
// Control Law Subroutine
//////////////////////////////////////////////////////////////////
//
// Inputs:
//   Desired position [mm]
//   Ball position [mm]
//
// Output:
//   Commanded Servo Position [deg]
//
#define AVG_LEN 3
double filt(double input) {
  static double moving_avg[AVG_LEN] = {0};
  const double factors[AVG_LEN] = {0.8, 0.15, 0.05};
  for (int i = 0; i < AVG_LEN-1; i++) {
    moving_avg[i+1] = moving_avg[i];
  }
  moving_avg[0] = input;
  double fil = 0;
  for (int i = 0; i < AVG_LEN; i++) {
    fil += moving_avg[i] * factors[i];
  }
  return fil;
}

double ServoControlLaw(double xBd, double xBmm) {
  static double integrator = 0;
  static double last_error = 0;

  const double Kp = 0.1; // 0.01
  const double Kd = 1.3;  // 0.1
  const double Ki = 0.1; // 0.01

  double error = filt(xBd - xBmm);
  integrator = integrator + error * deltaT;
  double output = Kp * error + Kd * (error - last_error) + Ki * integrator;
  last_error = error;

  return output;
}
//////////////////////////////////////////////////////////////////
// Control Law Ends Here
//////////////////////////////////////////////////////////////////

