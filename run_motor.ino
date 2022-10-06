#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

void setup() {
  // put your setup code here, to run once:
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
AFMS.begin();
myMotor->setSpeed(150);
myMotor->run(FORWARD);
}

void loop() {
  // put your main code here, to run repeatedly:

}
