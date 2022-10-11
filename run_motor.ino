#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
char sensorPin = A0;
int sensorValue = 0;
int line_detect_left = analogRead(A4);
int line_detect_right = analogRead(A3);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  AFMS.begin();
  myMotor->setSpeed(0);
  myMotor->run(FORWARD);


}

void loop() {


  sensorValue = analogRead(sensorPin);
  Serial.println(sensorValue);

  
  if (line_detect_left > 600){
    motor_left->setSpeed(300);
  }
  else{
    motor_left->setSpeed(200); 

  }

    if (line_detect_right > 600){
    motor_left->setSpeed(300);
  }
  else{
    motor_left->setSpeed(200); 

  }
  
  // put your main code here, to run repeatedly:
  


}
