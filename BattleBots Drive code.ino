#include <ESC.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"


#define CHANNEL1 11 // Right stick -Horizontal
#define CHANNEL2 10 // Right stick -Vertical
#define CHANNEL3 9 // WEAPON Left stick -Horizontal
#define CHANNEL4 6 // Right stick -Vertical
#define CHANNEL5 5 // VRA
#define WEAPONOUTPUT 3 // VRB


int CHX;
int CHY;
int WeaponInput;
/*
int channel4Value;
int channel5Value;
int channel6Value;
*/
int speedL;
int speedR;
int directionL;
int directionR;





int readChannel(int CHANNEL, int defaultValue)
  {
    int ch = pulseIn(CHANNEL, HIGH);
    if (ch < 100) return defaultValue;
    return map(ch, 1000, 2000, -245, 250); 
  }

int weaponController (int WeaponInput)
{
  int ch = pulseIn(WeaponInput, HIGH);
  Serial.print("RAW WEAPON DATA:");
  Serial.println(ch);
  ch = map(ch, 1000, 2000, 0, 180);
  if (ch <= 60) {
    return 0;
  } else return ch;
}

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotorL = AFMS.getMotor(1);
Adafruit_DCMotor *myMotorR = AFMS.getMotor(2);
//ESC weapon (3, 1000, 2000, 0);
Servo esc;
void setup() {
  // put your setup code here, to run once:
  AFMS.begin();
  Serial.begin(9600);
  Serial.print("HELLOWORLD");
  esc.attach(3,1000,2000);
  pinMode(CHANNEL1, INPUT);
  pinMode(CHANNEL2, INPUT);
  pinMode(CHANNEL3, INPUT);
  //pinMode(WEAPONOUTPUT, OUTPUT);
  Serial.print("Through pinsetup");
  //weapon.calib();
  //weapon.arm();
  //weapon.stop();
  //weapon->attach(WEAPONOUTPUT);
  Serial.print("Through Weapon Calibration");
  /*
  pinMode(CHANNEL4, INPUT);
  pinMode(CHANNEL5, INPUT);
  pinMode(CHANNEL6, INPUT);
  */
}

void loop() {
  Serial.print("In the Loop");
  CHX = readChannel(CHANNEL1, 0);
  CHY = readChannel(CHANNEL2, 0);
  WeaponInput = weaponController(CHANNEL3);
  esc.write(WeaponInput);
  //if (WeaponInput == 0) weapon.stop();
  //else weapon.speed(WeaponInput);
  
  /*
  channel4Value = readChannel(CHANNEL4, 0);
  channel5Value = readChannel(CHANNEL5, 0);
  channel6Value = readChannel(CHANNEL6, 0);
  */


  //Serial.println("Before Direction assignment");
  if (CHY > CHX)
    {
      directionR = FORWARD;
    }
  if (CHY < CHX)
    {
      directionR = BACKWARD;
    }
  if (CHY > -CHX)
    {
      directionL = FORWARD;
    }
  if (CHY < -CHX)
    {
      directionL = BACKWARD;
    }
 //Serial.println("Before Speed assignment");
  // Speed
  if ((CHY > 0)&&(CHY < CHX)||(CHY < 0)&&(CHY > CHX)) // Green triangles
    {
      speedL = abs(CHX);
      speedR = (abs(CHX)-abs(CHY));
    }
  if ((CHX > 0)&&(CHY > CHX)||(CHX < 0)&&(CHY < CHX)) // Yellow triangles
    {
      speedL = abs(CHY);
      speedR = (abs(CHY)-abs(CHX));
    }
  if ((CHY > 0)&&(CHY < -CHX)||(CHY < 0)&&(CHY > -CHX))// Red triangles
    {
      speedL = (abs(CHX)-abs(CHY));
      speedR = abs(CHX);
    }
  if ((CHX < 0)&&(CHY > -CHX)||(CHX > 0)&&(CHY < -CHX))// Purple triangles
    {
      speedL = (abs(CHY)-abs(CHX));
      speedR = abs(CHY);
    }
  //Serial.println("Before Motors Run");
  // Run motors
  myMotorL->setSpeed(speedL);
  
  myMotorL->run(directionL);
  
  myMotorR->setSpeed(speedR);
  
  myMotorR->run(directionR);

  
  Serial.print("WeaponInput: ");
  Serial.print(WeaponInput);
  Serial.println(",");
  delay(50);
/*
  Serial.print("CHX Value: ");
  Serial.print(CHX);
  Serial.println(",");

  Serial.print("CHY Value: ");
  Serial.print(CHY);
  Serial.println(",");
 
  //Serial.print("Channel4Value:");
  Serial.print(channel4Value);
  Serial.println(",");
  delay(5);
  

  Serial.print("Channel5Value:");
  Serial.print(channel5Value);
  Serial.print(",");

  Serial.print("Channel6Value:");
  Serial.println(channel6Value);
  //Serial.print("\n");
  */
  
  

}
