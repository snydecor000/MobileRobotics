/*
  NOTE:
   THIS IS THE STANDARD FOR HOW TO PROPERLY COMMENT CODE
   Header comment has program, name, author name, date created
   Header comment has brief description of what program does
   Header comment has list of key functions and variables created with decription
   There are sufficient in line and block comments in the body of the program
   Variables and functions have logical, intuitive names
   Functions are used to improve modularity, clarity, and readability
***********************************
  /*RobotEncoders.ino
   C.A. Berry, 11/20/17
   This program will demonstrate how the retrofit encoders for the CEENBot with the Arduino Mega. The encoders will use an interrupt
   to count the steps moved in order to provide feedback control to correct for odometry error. An interrupt will be used to read the encoder ticks and
   printed to the serial monitor. Try to determine how many ticks for one rotatoin of the wheel.

  encoder links:
    https://www.dfrobot.com/wiki/index.php/Wheel_Encoders_for_DFRobot_3PA_and_4WD_Rovers_(SKU:SEN0038)
    http://image.dfrobot.com/image/data/SEN0038/encoderSketch.zip
  mega links:
    pin mappings: https://www.arduino.cc/en/Hacking/PinMapping2560

  Hardware Connection:
  digital pin 13 - enable LED on microcontroller
  digital pin 48 - enable PIN on A4988 Stepper Motor Driver StepSTICK
  digital pin 50 - left stepper motor step pin
  digital pin 51 - left stepper motor direction pin
  digital pin 52 - right stepper motor step pin
  digital pin 53 - right stepper motor direction pin
  digital pin 2 - left wheel encoder
  digital pin 3 - right wheel encoder

  Key Variables:
  encoder[2]        //variable to hold number of encoder counts (left, right)
  lastSpeed[2]      //variable to hold encoder speed (left, right)

  Functions the program uses:
  move1() - function to move the robot wheels to count encoder ticks
  LWheelSpeed() - interrupt functions to count left encoder ticks, use difference between previous and current to calculate speed
  RWheelSpeed() - interrupt functions to count right encoder ticks, use difference between previous and currenct to calculate speed

  The following functions will be created by using the encoder feedback:
  goToAngle - given an angle in degrees use odomery to turn the robot
  goToGoal - given an x and y position in feet, use the goToAngle() function and trigonometry to move to a goal positoin
  moveSquare - given the side length in feet, move the robot in a square with the given side length
*/

#include <AccelStepper.h>       //include the stepper motor library
#include <MultiStepper.h>       //include multiple stepper motor library

//define pin numbers and variables
const int rtStepPin = 50;       //right stepper motor step pin
const int rtDirPin = 51;        // right stepper motor direction pin
const int ltStepPin = 52;       //left stepper motor step pin
const int ltDirPin = 53;        //left stepper motor direction pin
const int stepTime = 500;       //delay time between high and low on step pin
const int ltEncoder = 2;        //left encoder pin
const int rtEncoder = 3;        //right encoder pin
const int stepperEnable = 48;   //stepper enable pin on stepStick
const int enableLED = 13;       //stepper enabled LED

//define global program variables
#define stepperEnTrue false       //const for enabling stepper motor
#define stepperEnFalse true       //const for disabling stepper motor
#define one_rot 800               //number of counts for one wheel rotation
#define two_rot 1600              //number of counts for 2 wheel rotations
#define half_rot 400              //number of counts for 1/2 a wheel rotation
#define qua_rot 200               //number of counts for 1/4 a wheel rotation
#define LEFT  0                   //constant for left wheel
#define RIGHT 1                   //constant for right wheel
#define FWD 0                     //move constant for forward
#define REV 1                     //move constant for reverse
#define baud 9600                 //serial communication baud rate
#define wait_time 1000              //time to wait between prints and starting robot
volatile long encoder[2] = {0, 0};  //interrupt variable to hold number of encoder counts (left, right)
int lastSpeed[2] = {0, 0};          //variable to hold encoder speed (left, right)
int accumTicks[2] = {0, 0};         //variable to hold accumulated ticks since last reset

#define fwdSpeed 500              // speed for forward movement
#define readySpeed 100            // speed for slightly adjusting wheels to calibrate encoders
#define wheelDiameter 3.375       // diameter of the wheel in inches
#define ticksPerRev 20.0          // number of ticks encoder sees per revolution
#define distancePerTick wheelDiameter*PI/ticksPerRev    // the distance moved for each tick of the encoder

#define stepsPerRev 800
#define wheelDist 8.54      //distance between the center of the wheels in inches
#define stepsToInches wheelDiameter*PI/stepsPerRev
#define inchesToSteps stepsPerRev/(wheelDiameter*PI)

#define pivotDegreesToSteps (2*PI*wheelDist*inchesToSteps)/360.0

#define spinSpeed 500
#define rightSpeedAdjustment 0.99

const double spinDegreesToSteps = 5.65;

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;//create instance to control multiple steppers at the same time


//   the setup function runs only once to set up all variables, inputs, outpus, serial communication and interrupts
void setup() {
  pinMode(rtStepPin, OUTPUT);         //sets right stepper pin as output
  pinMode(rtDirPin, OUTPUT);          //sets right stepper direction pin as output
  pinMode(ltStepPin, OUTPUT);         //sets left stepper pin as output
  pinMode(ltDirPin, OUTPUT);          //sets left stepper director pin as output
  pinMode(stepperEnable, OUTPUT);     //sets stepper enable pin as output
  digitalWrite(stepperEnable, stepperEnFalse);//turns off the stepper motor driver
  pinMode(enableLED, OUTPUT);         //sets pin 13 enable LED on microcontroller as output
  digitalWrite(enableLED, LOW);       //turn off enable LED
  stepperRight.setMaxSpeed(1500);     //set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(10000);//set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(1500);      //set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(10000); //set desired acceleration in steps/s^2
  steppers.addStepper(stepperRight);//add right motor to MultiStepper
  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);      //turn on enable LED
  attachInterrupt(digitalPinToInterrupt(ltEncoder), LwheelSpeed, CHANGE);    //init the interrupt mode for the digital pin 2
  attachInterrupt(digitalPinToInterrupt(rtEncoder), RwheelSpeed, CHANGE);   //init the interrupt mode for the digital pin 3
  Serial.begin(baud);                 //init the Serial port to print the data
  delay(wait_time);                   //wait before moving the robot
}

//the loop funciton runs continuously to move the robot wheels and count encoder ticks
void loop() {
  //move1(FWD, qua_rot);            //move the robot wheels
  //print_data();                   //prints encoder data

  calibrate(true, true, true);
  stopRobot();
  delay(wait_time);
  goToGoal(-10, -12);
  //goToAngle(90.0);
  //moveSquare(18.0);
  //forward(18.0);                  // calls forward function to move robot forward
  delay(wait_time);               //wait to move robot
  
}

//function prints encoder data to serial monitor
void print_data() {
  static unsigned long timer = 0;                           //print manager timer
  if (millis() - timer > 100) {                             //print encoder data every 100 ms or so
    lastSpeed[LEFT] = encoder[LEFT];                        //record the latest left speed value
    lastSpeed[RIGHT] = encoder[RIGHT];                      //record the latest right speed value
    accumTicks[LEFT] = accumTicks[LEFT] + encoder[LEFT];    //record accumulated left ticks
    accumTicks[RIGHT] = accumTicks[RIGHT] + encoder[RIGHT]; //record accumulated right ticks
    Serial.println("Encoder value:");
    Serial.print("\tLeft:\t");
    Serial.print(encoder[LEFT]);
    Serial.print("\tRight:\t");
    Serial.println(encoder[RIGHT]);
    Serial.println("Accumulated Ticks: ");
    Serial.print("\tLeft:\t");
    Serial.print(accumTicks[LEFT]);
    Serial.print("\tRight:\t");
    Serial.println(accumTicks[RIGHT]);
    encoder[LEFT] = 0;                          //clear the left encoder data buffer
    encoder[RIGHT] = 0;                         //clear the right encoder data buffer
    timer = millis();                           //record current time since program started
  }
}

//interrupt function to count left encoder tickes
void LwheelSpeed()
{
  encoder[LEFT] ++;  //count the left wheel encoder interrupts
  accumTicks[LEFT]++; // count accumulated ticks for left wheel
}

//interrupt function to count right encoder ticks
void RwheelSpeed()
{
  encoder[RIGHT] ++; //count the right wheel encoder interrupts
  accumTicks[RIGHT]++; // count accumulated ticks for left wheel
}


/*
   The move1(int dir, int amt) function will move the robot forward or reverse by amount in input.Recall that that there 200 steps in one full rotation or 1.8 degrees per
   step. This function uses setting the step pins high and low with delays to move. The speed is set by
   the length of the delay.
*/
void move1(int dir, int amt) {
  digitalWrite(ltDirPin, dir);
  digitalWrite(rtDirPin, dir);
  for (int x = 0; x < amt; x++) {  // Makes pulses for wheel rotations
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
  }
}

/*
  forward(double distance)
  This function takes in a distance (in inches) and runs the left and right steppers 
  at the same constant speed until the robot has traveled the given distance in the
  forward direction. The encoders keep track of how far the robot has traveled and 
  dictate if the desired distance has been reached. 

  BLOCKING FUNCTION
*/
void forward(double distance) {
  
  int leftEncoderTarget = int(distance/double(distancePerTick));    // caculate the target value based on the desired distance

  // set speeds to move forward if distance is positive
  if(distance < 0){
    stopRobot();
  } else {
    setSpeeds(fwdSpeed, fwdSpeed);
  }

  // clear accumulated ticks and left encoder ticks
  accumTicks[LEFT] = 0;
  encoder[LEFT] = 0;

  // move steppers if desired distance has not been reached according to encoder data
  while(accumTicks[LEFT] <= leftEncoderTarget) {    
    stepperLeft.runSpeed(); 
    stepperRight.runSpeed();
    encoder[LEFT] = 0;
  }

  stopRobot();
}

/*
  stopRobot()

  This function stops both stepper motors by setting their speeds to 0

  NON-BLOCKING FUNCTION
*/
void stopRobot() {
  setSpeeds(0, 0);
}

/*
  calibrate()

  This function increments each stepper until one encoder tick has been seen.
  This helps calibrate the encoders before using them for movement

  BLOCKING FUNCTION
*/
void calibrate(bool moveLeft, bool moveRight, bool clockwise) {
  if(moveLeft){
    // Calibrate left encoder
    if(clockwise){
      setSpeeds(readySpeed, 0);
    } else{
      setSpeeds(-readySpeed, 0);
    }
    encoder[LEFT] = 0;
    while(encoder[LEFT] <= 1) {    
      stepperLeft.runSpeed();
    }
  }

  if(moveRight){
    // Calibrate right encoder
    if(clockwise){
      setSpeeds(0, readySpeed);
    } else{
      setSpeeds(0, -readySpeed);
    }
    encoder[RIGHT] = 0;
    while(encoder[RIGHT] <= 1) {    
      stepperRight.runSpeed();
    }
  }
}

/* 
 *  setSpeeds() 
 *  
 *  This function sets the speed of the left and right steppers.  This allows for the speed of one motor 
 *  to be adjusted if one is faster than the other.  
 */
void setSpeeds(double left, double right) {
  stepperLeft.setSpeed(left);
  stepperRight.setSpeed(right*rightSpeedAdjustment);
}

/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used
  pivoting left is clockwise
  pivoting right is counter-clockwise
*/
void pivot(boolean clockwise, double dgrees) {
  int steps = int(dgrees*pivotDegreesToSteps);
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
  
  if(clockwise){
    stepperLeft.moveTo(steps);
    setSpeeds(spinSpeed,0);
  } else {
    stepperRight.moveTo(steps);
    setSpeeds(0,spinSpeed);
  }

  steppers.runSpeedToPosition(); // Blocks until all are in position
}


/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used
*/
void spin(boolean clockwise, double dgrees) {
  int steps = int(abs(dgrees)*spinDegreesToSteps);
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
  
  if(clockwise){
    stepperLeft.moveTo(steps);
    stepperRight.moveTo(-steps);
    setSpeeds(spinSpeed,-spinSpeed);
  } else {
    stepperLeft.moveTo(-steps);
    stepperRight.moveTo(steps);
    setSpeeds(-spinSpeed,spinSpeed);
  }

  steppers.runSpeedToPosition(); // Blocks until all are in position
}

/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used
*/
void goToAngle(double angle) {
  encoder[RIGHT] = 0;
  encoder[LEFT] = 0;

  accumTicks[RIGHT] = 0;
  accumTicks[LEFT] = 0;

  double angleTicksGoal = (abs(angle)/360.0)*(PI*double(wheelDist)/double(distancePerTick));

  if(angle < 0){
    spin(false, angle);
  } else{
    spin(true, angle);
  }

  if(accumTicks[LEFT] > angleTicksGoal + 1) {
    calibrate(true, false, false);
  } else if(accumTicks[LEFT] < angleTicksGoal - 1) {
    calibrate(true, false, true);
  } 
  
  if(accumTicks[RIGHT] > angleTicksGoal + 1) {
    calibrate(false, true, false);
  } else if(accumTicks[RIGHT] < angleTicksGoal - 1) {
    calibrate(false, true, true);
  }
}

/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used
*/
void goToGoal(int x, int y) {
  y = -y;
  double rads = atan2(y,x);
  double dgrees = rads*(180.0/3.14159265);
  double dist = sqrt(y*y+x*x);
  
  goToAngle(dgrees);
  delay(wait_time);
  forward(dist);
}

/*
  moveSquare()

  This function moves the robot in a square shape by calling the forward function
  and the pivot function 4 times. The forward function utilizes encoders to track
  the distance traveled by the robot

  BLOCKING FUNCTION
*/
void moveSquare(double side) {
  for(int i = 0; i < 4; i++) {
    forward(side);
    stopRobot();
    pivot(true, 90);
  }
  stopRobot();
}
