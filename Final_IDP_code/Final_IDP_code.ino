#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
//initialise motors
Adafruit_DCMotor *motor_left = AFMS.getMotor(1);
Adafruit_DCMotor *motor_right = AFMS.getMotor(2);
Servo block_servo; //initialise servo

unsigned long running_time1 = 0; 
unsigned long time1 = 0;
unsigned long time2 = 0;
unsigned long time3 = 0;
//used in case of needing to wait a certain amount of time i.e. (time2 - time1) > 3000 gives a 3 second gap. 

int low = 350; //Low motor speed
int medium = 400; //Medium Motor speed
int high = 500; //High motor speed
int stop = 0; // No motor speed
int left_speed = 0; // Holds the value of the current speed of the left motor
int right_speed = 0; // Holds the value of the current speed of the right motor

int buttonstate = 0; // is 0 when arduino starts, is 1 once the button is pressed, allowing the car to move
int start_stage = 0; //Defines what part of the starting stage the car is in

int flash_status = 0; //Holds whether the amber flashing light is on or off
int magnetic_block = 0; //Holds whether the block being carried is magnetic (1) or not (0) 
int mode = 2; // Holds the current "mode" the program is in - different modes encapsulate different tasks the robot is performing
/*
Mode 0 - Run Along the line, waiting for the tunnel
Mode 1 - Checking if the block is magnetic, setting the correct LED on
Mode 2 - Exiting the Starting Area
Mode 3 - Algorithm for finding the correct box to return to
Mode 4 - Approaching the 1st block 
Mode 5 - Doing a spin with the block, and reattaching to the white line
Mode 6 - Place the block
Mode 7 - Hardcoded finish from the magnetic box
Mode 8 - Hardcoded finish from the non - magnetic box
*/
int substate = 0; //Determines the substates of mode 0 (essentially states which side of the board the car is approaching the tunnel from)

int last_junction = 0; //determines how many junctions the robot has gone past when it is finding the correct box to place the block
int block_detect = 0;  //Holds (0) if it hasn't detected a block, holds (1) if it has
int ultra_error = 0; //Acts as way of confirming the ultrasonic sensors outputs as there seemed to be noise

#define  red_led_pin  0 //magnetic - red
#define  green_led_pin 1 //non magnetic - green
#define  moving_pin  2 //while moving - amber
#define echoPin 3 // ultrasonic sensor echo pin
#define trigPin 4 // ultrasonic sensor trigger pin
#define line_sensor_pin_left  5 //left line sensor
#define  line_sensor_pin_middle  6 //middle line sensor
#define  line_sensor_pin_right  7 //right line sensor
#define  line_sensor_pin_junction  8 //farright line sensor
#define  servo_pin  9 //servo pin
#define  magnet_pin  11//hall effect sensor
#define buttonPin  13
#define distance_sensor_pin = A0

int follow_white_line(int line_detect_left, int line_detect_right, int line_detect_middle, int size) { // follows the white line

  if (line_detect_left == 1 && line_detect_middle == 0 && line_detect_right == 0 ){ //make sharp left
    left_speed = stop;
    right_speed = high;
  }

  else if (line_detect_left == 1 && line_detect_middle == 1 && line_detect_right == 0){ //turn left a little
    left_speed = low;
    right_speed = medium;

  }
  else if (line_detect_left == 0 && line_detect_middle == 0 && line_detect_right == 1){ //turn right hard
    left_speed = high;
    right_speed = stop;
  
  }
  else if (line_detect_left == 0 && line_detect_middle == 1 && line_detect_right == 1){ //turn right
    left_speed = medium;
    right_speed = low;

  }
  else if (line_detect_left == 1 && line_detect_middle == 1 && line_detect_right == 1){ //all sensors active go straight
    left_speed = medium;
    right_speed = medium;
  }
  else if (line_detect_left == 0 && line_detect_middle == 0 && line_detect_right == 0){ //all off just go straight 
    left_speed = medium;
    right_speed = medium;
  
  }
  else if (line_detect_left == 0 && line_detect_middle == 1 && line_detect_right == 0){ //Go straight, following the middle line sensor 
    left_speed = medium;
    right_speed = medium;
  
  }
  else if (line_detect_left == 1 && line_detect_middle == 0 && line_detect_right == 1){ //fault, keep going but slowly in the hopes of correction
    left_speed = low;
    right_speed = low;
  
  }
  
  //Sets speed and direction of wheel rotation
  //Wheels are rotating backwards as the motors were mounted backwards
  motor_left->setSpeed(left_speed); 
  motor_right->setSpeed(right_speed);
  motor_right->run(BACKWARD);
  motor_left->run(BACKWARD);

}

int get_ultrasonic_distance(){ //Returns the distance the ultrasonic sensor measures

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  int duration = pulseIn(echoPin, HIGH); 
  int distance = (duration / 2) * 0.0343; //Halving the distance as the soundwave travels twice the wanted distance, then dividing by the speed of sound
  return distance;
  
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  AFMS.begin();

  //defining digital pins as inputs or outputs
  pinMode(line_sensor_pin_left, INPUT); 
  pinMode(line_sensor_pin_middle, INPUT); 
  pinMode(line_sensor_pin_right, INPUT); 
  pinMode(line_sensor_pin_junction, INPUT);
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 
  pinMode(red_led_pin, OUTPUT);
  pinMode(green_led_pin, OUTPUT);
  pinMode(moving_pin, OUTPUT);
  pinMode(magnet_pin, INPUT);
  pinMode(buttonPin, INPUT);

  //Attaching the servo, and setting its starting position
  block_servo.attach(servo_pin);
  block_servo.write(102);

}

void loop() {
  //Get line sensor data every loop
  int line_detect_left = digitalRead(line_sensor_pin_left);
  int line_detect_middle = digitalRead(line_sensor_pin_middle);
  int line_detect_right = digitalRead(line_sensor_pin_right);
  int line_detect_junction = digitalRead(line_sensor_pin_junction);

  if (buttonstate == 1){ //Waiting for button press
    if (mode == 0){ //RUN ALONG THE LINE

      follow_white_line(line_detect_left, line_detect_right, line_detect_middle,  200 );

      if (substate == 1 && line_detect_left == 0 && line_detect_right == 0 && line_detect_middle == 0 && line_detect_junction == 0){ //adjust into tunnel from the starting side
        motor_left->setSpeed(stop);
        motor_right->setSpeed(stop);
        left_speed = right_speed = stop;
        delay(1000); //Brief pause to ensure car is stationary
        left_speed = high;
        motor_left->setSpeed(left_speed);
        delay(10); //10ms of high speed adjustment
        
        substate = 3; //Waiting to find the line again
        //the car drives in a straight line here due to repeated calls of follow_white_line with no sensor detections
        }
      }

      if (substate == 2 && line_detect_left == 0 && line_detect_right == 0 && line_detect_middle == 0 && (millis() - time3) > 12000 ){ //adjust into tunnel
        //Extra millis() statement was required in the line above as the car was detecting the tunnel before it should have been. 
        motor_left->setSpeed(stop);
        motor_right->setSpeed(stop);
        left_speed = right_speed = 0;
        delay(1000); //Brief pause to ensure car is stationary
        right_speed = high;
        motor_right->setSpeed(right_speed);
        delay(15); //determines amount of adjustment in the tunnel needed
        left_speed = right_speed = 450; 
        motor_left->setSpeed(left_speed);
        motor_right->setSpeed(right_speed);
        substate = 4; //Waiting for  line pickup
      }

      if (substate == 3 && (line_detect_left == 1 || line_detect_right == 1 || line_detect_middle == 1)){ //once the line has been picked up again
        mode = 4; //Approach the first block
      }

      if (substate == 4 && (line_detect_left == 1 || line_detect_right == 1 || line_detect_middle == 1)){ //  once line picked up again
        mode = 3; // Go onto box delivery mode
      }


    } else if (mode == 1){ //Check if the block is magnetic, and set the correct LED on
        
        block_servo.write(40); //Open the servo arm
        delay(2000);
        int magnetic = 0;
        unsigned long time5 = millis();
        int total = 0;
        int count = 0;
        int average = 0;
        
        while (millis() - time5 < 1000){ //Takes 100 data points over a second (we found this was more stable) 
  
          total += digitalRead(magnet_pin);
          delay(10);
          count += 1;
          
        }

        if (total >= 5){ //If there are more than 5 counts out of 100 that are magnetic
        //Any less than 5 ran the risk of the sensor being activated by noise
          magnetic = 1;
        }
        else{
          magnetic = 0;
        }

        if (magnetic == 1){ //if the block is deemed magnetic
          magnetic_block = 1;
          digitalWrite(red_led_pin, HIGH); // Turn on the red LED
          delay(5000); //LED on for 5 seconds, as per specification
          digitalWrite(red_led_pin, LOW);

        }
        else if (magnetic == 0){ // if the block is deemed non-magnetic
          magnetic_block = 0;
          digitalWrite(green_led_pin, HIGH);  //Turn on green LED
          delay(5000);//LED on for 5 seconds, as per specification
          digitalWrite(green_led_pin, LOW); 
          Serial.println("NOT MAGNETIC");

        }
        
        block_servo.write(102); //close the arm to lock in the block
        delay(3000); //gives time for the servo arm to close
        mode = 5; //Begin 180 spin
      
    } else if (mode == 2){ // exit start area
        Serial.println(start_stage);
        if (start_stage == 0){ //Drive forwards 
          left_speed = right_speed = medium;
          motor_left->setSpeed(left_speed);
          motor_right->setSpeed(right_speed);
          motor_right->run(BACKWARD);
          motor_left->run(BACKWARD);
          start_stage = 1;
          Serial.println("3333");
        } else if (line_detect_left == 1 && line_detect_right == 1 && line_detect_middle == 1 && start_stage == 1){ //If you touch the first horizontal line (Relative to the car)
          delay(400); //Leaves time to come off of the white line
          start_stage = 2;
        } else if ( line_detect_left == 1 && line_detect_right == 1 && line_detect_middle == 1 && start_stage == 2){ //If you touch the second horizontal line (Relative to the car)
          delay(102); //Go slightly forwards
          Serial.println("2");
          start_stage = 3;
        } else if (start_stage == 3){ //Turn onto the main white line
            left_speed = 0;
            motor_left->setSpeed(stop);
            delay(500);
            Serial.println("4");
            //Turns left for hardcoded amount
            right_speed = high;
            left_speed = -high;
            motor_right->setSpeed(right_speed);
            motor_left->setSpeed(left_speed);
            delay(1600);
            right_speed = left_speed = 0;
            motor_right->setSpeed(right_speed);
            motor_left->setSpeed(left_speed);
            Serial.println("Running mode 2 somehow?");
            mode = 0; //hook onto the white line
            substate = 1; //Approaching tunnel from starting side
            
        }

        



    } else if (mode == 3){ //return to a box
        mode = 3; //Had issues with mode switching randomly - this line fixes it
        follow_white_line(line_detect_left, line_detect_right, line_detect_middle,  200 ); 
        if (line_detect_junction == 1){ // If a junction is detected

          if (last_junction == 0){ //If you are at the first junction
            if (magnetic_block == 0){ //If block isnt magnetic
              mode = 6 ; //Deposit block 

            }
            time1 = millis(); //Else, keep track of current time
            last_junction = 1; //Now met 1 junction

          }

          if (time1 < millis() - 2000){ //If it's been 2 seconds since the last junction (so you can't detect the same junction twice)
            if (magnetic_block == 1 && last_junction == 2) { //Magentic, and at final junction
              
              mode = 6; // place block 
            }

            else{
              time1 = millis(); //Must be at the starting box
              last_junction += 1; //Now passed by 2 junctoins

            }
          } 

        }
        
    
    } else if (mode == 4){ //Approach Only The 1st Block!  
        mode = 4; //Had issues with mode switching, this fixed it
        if (block_detect == 0){ //If you haven't found a block yet
          block_servo.write(102); //Keep the arm closed
          if (get_ultrasonic_distance() <= 7){
            Serial.println("In range");
            ultra_error += 1;  //Eliminating noise / errors
            if (ultra_error >= 3 ){ //i.e consistent read 3 times in a row, block detected
              motor_left->setSpeed(0); //stop the vehicle
              motor_right->setSpeed(0);
              block_servo.write(40); //open the gate
              delay(2000); //time for servo arm to move

            
              //Hardcoded vehicle approach so the block is next to the hall effect sensor
              motor_left->setSpeed(500); //constant speed
              motor_right->setSpeed(475); //Disparity in speed due to the ultrasonic sensor not being in the middle of the car
              motor_right->run(BACKWARD);
              motor_left->run(BACKWARD);

              delay(700); //constant speed lasts for x time
              motor_left->setSpeed(stop);
              motor_right->setSpeed(stop);
              block_detect = 1;  //block found
              mode = 1; //Detect if its magnetic now
            }
          }
          else {
            ultra_error = 0; //Reset the error value
            follow_white_line(line_detect_left, line_detect_right, line_detect_middle,  200 );
          }
          
        }
        
        else if (block_detect == 1){ //Should never be but just in case
          block_servo.write(102); //Ensures the arm is closed
        } 


    } else if (mode == 5){ //Do a 180 with block

      //Sets a timer for approaching the tunnel, and then does a hardcoded 180 with the block
      time3 = millis();
      left_speed = 500;
      right_speed = stop;
      motor_right->setSpeed(stop);
      motor_left->setSpeed(500);
      motor_right->run(BACKWARD);
      motor_left->run(BACKWARD);
      delay(5000);

      mode = 0; //Follow white line
      substate = 2; //Approaching tunnel from the side of the board with blocks

      

    } else if (mode == 6){ //Place block
        //Hardcoded block placement
        left_speed = right_speed = stop;
        motor_left->setSpeed(left_speed);
        motor_right->setSpeed(right_speed);
        delay(500);
        left_speed = high;
        right_speed = -high;
        motor_left->setSpeed(left_speed);
        motor_right->setSpeed(right_speed);
        delay(1800);
        left_speed = right_speed = high;
        motor_right->setSpeed(right_speed);
        motor_left->setSpeed(left_speed);
        delay(600);
        block_servo.write(30);
        left_speed = right_speed = stop;
        motor_right->setSpeed(right_speed);
        motor_left->setSpeed(left_speed);
        if (magnetic_block == 1){
          mode = 7; //If coming from a magnetic block placement, run hardcoded finishing algorithm
        }
        else if (magnetic_block == 0){ //If coming from a non magnetic block placement, run hardcoded finishing algorithm
          mode = 8;

        }


    } else if (mode == 7){ //finish from magnetic box
        //Hardcoded return to the start box, from the magnetic box
        //Reverse
        left_speed = right_speed = 500;
        motor_left->setSpeed(left_speed);
        motor_right->setSpeed(right_speed);
        motor_left->run(FORWARD);
        motor_right->run(FORWARD);
        //Rotate clockwise a bit
        delay(1200);
        motor_right->setSpeed(stop);
        motor_left->run(BACKWARD);
        motor_right->run(BACKWARD);
        delay(1500);

        //Drive forwards for a couple seconds until in the box
        motor_right->setSpeed(460);
        delay(3200);
        //Stop
        motor_left->setSpeed(stop);
        motor_right->setSpeed(stop);
        left_speed = right_speed = 0;

        //Code ends - mode 100 doesn't exist. 
        mode = 100; //Finish

 
    } else if (mode == 8){ //Finish from the non-magnetic box
        //Hardcoded return to the start box, from the non - magnetic box
        //Reverse
        mode = 8; 
        left_speed = right_speed = 500;
        motor_left->setSpeed(left_speed);
        motor_right->setSpeed(right_speed);
        motor_left->run(FORWARD);
        motor_right->run(FORWARD);

        //Spin anticlockwise
        delay(1000);
        motor_left->setSpeed(stop);
        motor_right->run(BACKWARD);
        motor_left->run(BACKWARD);
        delay(1500);

        //Drive into start box
        motor_left->setSpeed(470);
        delay(3200);
        motor_left->setSpeed(stop);
        motor_right->setSpeed(stop);
        left_speed = right_speed = 0;

        //Mode 100 doesn't exist, so this stops the car
        mode = 100;  //Finish

    } 

    //Below code flashes the amber LED when moving at 2hz
    if (right_speed != 0 || left_speed != 0){

      if ((millis() - running_time1 > 250) && flash_status == 0 ){ //If turned off and delta time > 250ms, turn on
        digitalWrite(moving_pin, HIGH); 
        flash_status = 1;
        running_time1 = millis(); //Set delta time as the current time

      }

      else if ((millis() - running_time1 > 250) && flash_status == 1){ // If the light is on and delta time > 250ms, turn off 
        digitalWrite(moving_pin, LOW);
        flash_status = 0;
        running_time1 = millis(); //Set delta time as current time

      }

    }
  
  else{
    buttonstate = digitalRead(buttonPin); //Keep reading the button pin until it's pressed
  }
  }
