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
  RobotIntro.ino
  Carlotta Berry 11.21.16

  This program will introduce using the stepper motor library to create motion algorithms for the robot.
  The motions will be go to angle, go to goal, move in a circle, square, figure eight and teleoperation (stop, forward, spin, reverse, turn)
  
  The primary functions created are
  moveCircle - given the diameter in inches and direction of clockwise or counterclockwise, move the robot in a circle with that diameter
  moveFigure8 - given the diameter in inches, use the moveCircle() function with direction input to create a Figure 8
  forward, reverse - both wheels move with same velocity, same direction
  pivot- one wheel stationary, one wheel moves forward or back
  spin - both wheels move with same velocity opposite direction
  turn - both wheels move with same direction different velocity
  stop -both wheels stationary

  Interrupts
  https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
  https://www.arduino.cc/en/Tutorial/CurieTimer1Interrupt
  https://playground.arduino.cc/code/timer1
  https://playground.arduino.cc/Main/TimerPWMCheatsheet
  http://arduinoinfo.mywikis.net/wiki/HOME

  Hardware Connections:
  pin mappings: https://www.arduino.cc/en/Hacking/PinMapping2560
  digital pin 13 - enable LED on microcontroller
  digital pin 48 - enable PIN on A4988 Stepper Motor Driver StepSTICK
  digital pin 50 - right stepper motor step pin
  digital pin 51 - right stepper motor direction pin
  digital pin 52 - left stepper motor step pin
  digital pin 53 - left stepper motor direction pin

  digital pin 5 - red LED in series with 220 ohm resistor
  digital pin 6 - green LED in series with 220 ohm resistor
  digital pin 7 - yellow LED in series with 220 ohm resistor


*/

#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include "PS2X_lib.h"    //include the PS2 game controller library

#if 0
#define DEBUG
#endif

#if 1
#define PS2
#endif

//define pin numbers
const int rtStepPin = 50; //right stepper motor step pin (pin 44 for wireless)
const int rtDirPin = 51;  // right stepper motor direction pin (pin 49 for wireless)
const int ltStepPin = 52; //left stepper motor step pin (pin 46 for wireless)
const int ltDirPin = 53;  //left stepper motor direction pin (no change in pin for wireless)
const int ltEncoder = 2;  //left encoder pin
const int rtEncoder = 3;  //right encoder pin
const int stepperEnable = 48;  //stepper enable pin on stepStick 
const int stepTime = 500; //delay time between high and low on step pin
#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor

const double stepsPerRev = 800;
const double wheelDiameter = 3.375; //diameter of the wheel in inches
const double spinWheelDist = 8.55;  //distance between the wheels (Adjusted for carpet and the spin func)
const double pivotWheelDist = 8.55;  //distance between the wheels (Adjusted for carpet and the spin func)
const double stepsToInches = wheelDiameter*PI/stepsPerRev;
const double inchesToSteps = stepsPerRev/(wheelDiameter*PI);
const double ticksPerRev = 20.0;    // number of ticks encoder sees per revolution
const double distancePerTick = wheelDiameter*PI/ticksPerRev; // the distance moved for each tick of the encoder
const double rightSpeedAdjustment = 1;

//Used for spinning (Both wheels going opposite direction)
const double pivotDegreesToSteps = (2*PI*pivotWheelDist*inchesToSteps)/360.0;
const double spinDegreesToSteps = (PI*spinWheelDist*inchesToSteps)/360.0;


#define maximumSpeed 1500
#define maxAccel 1000
#define fwdSpeed 200
#define revSpeed 500
#define spinSpeed 200
#define turnSpeed 400
#define readySpeed 100            // speed for slightly adjusting wheels to calibrate encoders
#define CLOCKWISE true
#define COUNTERCLOCKWISE false

#define LEFT  0                     //constant for left wheel
#define RIGHT 1                     //constant for right wheel
volatile long encoder[2] = {0, 0};  //interrupt variable to hold number of encoder counts (left, right)
int lastSpeed[2] = {0, 0};          //variable to hold encoder speed (left, right)
int accumTicks[2] = {0, 0};         //variable to hold accumulated ticks since last reset

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;//create instance to control multiple steppers at the same time

#ifdef PS2

//create PS2 Controller Object and variables used with it
#define DATA 22
#define CLOCK 25
#define COMMAND 23
#define ATTENTION 24
PS2X ps2x;
#define joystickDeadband 10
int error;
byte type;
byte vibrate;

#endif

#define enableLED 13    //stepper enabled LED
#define redLED 5        //red LED for displaying states
#define grnLED 6        //green LED for displaying states
#define ylwLED 7        //yellow LED for displaying states

#define pauseTime 5000 //time before robot moves
#define wait_time 1000 //time to wait between prints and starting robot

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

void setup()
{
  pinMode(rtStepPin, OUTPUT);                   //sets pin as output
  pinMode(rtDirPin, OUTPUT);                    //sets pin as output
  pinMode(ltStepPin, OUTPUT);                   //sets pin as output
  pinMode(ltDirPin, OUTPUT);                    //sets pin as output
  pinMode(stepperEnable, OUTPUT);               //sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);  //turns off the stepper motor driver
  pinMode(enableLED, OUTPUT);                   //set enable LED as output
  digitalWrite(enableLED, LOW);                 //turn off enable LED
  pinMode(redLED, OUTPUT);                      //set red LED as output
  pinMode(grnLED, OUTPUT);                      //set green LED as output
  pinMode(ylwLED, OUTPUT);                      //set yellow LED as output
  digitalWrite(redLED, HIGH);                   //turn on red LED
  digitalWrite(ylwLED, HIGH);                   //turn on yellow LED
  digitalWrite(grnLED, HIGH);                   //turn on green LED
  delay(pauseTime / 5);                         //wait 0.5 seconds
  digitalWrite(redLED, LOW);                    //turn off red LED
  digitalWrite(ylwLED, LOW);                    //turn off yellow LED
  digitalWrite(grnLED, LOW);                    //turn off green LED

  attachInterrupt(digitalPinToInterrupt(ltEncoder), LwheelSpeed, CHANGE); //init the interrupt mode for the digital pin 2  
  attachInterrupt(digitalPinToInterrupt(rtEncoder), RwheelSpeed, CHANGE); //init the interrupt mode for the digital pin 3

  stepperRight.setMaxSpeed(maximumSpeed);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setMaxSpeed(maximumSpeed);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(maxAccel);//set desired acceleration in steps/s^2
  stepperLeft.setAcceleration(maxAccel);//set desired acceleration in steps/s^2
  steppers.addStepper(stepperRight);//add right motor to MultiStepper
  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);//turn on enable LED
  delay(pauseTime); //always wait 5 seconds before the robot moves

#ifdef PS2
  //THIS CODE IS BASED OFF OF EXAMPLE CODE FROM Bill Porter
  ////////////////////////////////////////////////////////////////////////////
  //setup pins and settings:
  error = ps2x.config_gamepad(CLOCK, COMMAND, ATTENTION, DATA, true, true);
  
#ifdef DEBUG
  if (error == 0)
  {
    Serial.println("Found Controller, configured successful");
    Serial.println("Go to www.billporter.info for updates and to report bugs.");
    digitalWrite(LED, HIGH);//turn indicator light on to show good connection
  }
  else if (error == 1)
  {
    Serial.println("No controller found, check wiring");
    digitalWrite(LED, LOW);//turn indicator light off
  }
  else if (error == 2)
  {
    Serial.println("Controller found but not accepting commands");
    digitalWrite(LED, LOW);//turn indicator light off
  }
  else if (error == 3)
  {
    Serial.println("Controller refusing to enter Pressures mode");
    digitalWrite(LED, LOW);//turn indicator light off
  }
#endif

  type = ps2x.readType();
  
#ifdef DEBUG
  switch (type)
  {
    case 0:
      Serial.println("Unknown Controller type");
      break;
    case 1:
      Serial.println("DualShock Controller Found");
      break;
    case 2:
      Serial.println("GuitarHero Controller Found");
      break;
  }
#endif
#endif
  ////////////////////////////////////////////////////////////////////////////////
  //Serial.begin(9600); //start serial communication at 9600 baud rate for debugging
}

unsigned long last_read = 0;
void loop()
{
  //Every 50 milliseconds, poll the PS2 controller and get the latest teleop commands
  int temp = millis() - last_read;
  if(temp > 50){
    readController();
    last_read = millis();
  }
  //Run the stepper motors at the speed they were set to in the readController() function
  stepperRight.runSpeed();
  stepperLeft.runSpeed();


//  forward(18.0);
//  delay(3*wait_time);
//  reverse(18.0);
//  delay(3*wait_time);
//  turn(true,90.0,12.0);
//  delay(3*wait_time);
//  pivot(true,90.0);
//  delay(3*wait_time);
//  spin(true,45.0);
//  delay(3*wait_time);
//  stopRobot();
//  moveCircle(true,36.0);
//  delay(3*wait_time);
//  moveFigure8(36.0);
//  delay(3*wait_time);
//  calibrate(true,true,true);
//  delay(5*wait_time);
//  goToAngle(-60.0);
//  delay(3*wait_time);
//  calibrate(true,true,true);
//  delay(5*wait_time);
//  goToAngle(135.0);
//  delay(3*wait_time);
//  calibrate(true,true,true);
//  delay(5*wait_time);
//  goToGoal(0,48.0);
//  delay(3*wait_time);
//  calibrate(true,true,true);
//  delay(5*wait_time);
//  goToGoal(-24.0,36.0);
//  delay(3*wait_time);
//  calibrate(true,true,true);
//  delay(5*wait_time);
//  moveSquare(48.0);
//forward(24.0);
//delay(500);
//pivot(true,90.0);
//  delay(500);
//  forward(24.0);
//delay(500);
//pivot(true,90.0);
//  delay(500);
//  forward(24.0);
//delay(500);
//pivot(true,90.0);
//  delay(500);
//  forward(24.0);
//delay(500);
//pivot(true,90.0);
//  delay(5000);
  

  //uncomment each function one at a time to see what the code does
//  move1();//call move back and forth function
//  move2();//call move back and forth function with AccelStepper library functions
//  move3();//call move back and forth function with MultiStepper library functions
//  move4(); //move to target position with 2 different speeds
//  move5(); //move continuously with 2 different speeds
//  turn(true,90.0,12.0);
//  forward(12);
//  delay(2000);
//  reverse(12);
//  delay(2000);
//  spin(true,90);
//  delay(2000);
//  pivot(false,90);
//  moveFigure8(12);
//  delay(2000);
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

  runSpeedToActualPosition(); // Blocks until all are in position
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
  runSpeedToActualPosition(); // Blocks until all are in position
}

/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used
  test
*/
void turn(boolean clockwise, double dgrees, double radius) {
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
  
  int rightSteps;
  int leftSteps;
  
  if(clockwise){
    leftSteps = int(inchesToSteps*((dgrees/360.0)*2.0*PI*(radius+spinWheelDist/2.0)));
    rightSteps = int(inchesToSteps*((dgrees/360.0)*2.0*PI*(radius-spinWheelDist/2.0)));
  } else {
    leftSteps = int(inchesToSteps*((dgrees/360.0)*2.0*PI*(radius-spinWheelDist/2.0)));
    rightSteps = int(inchesToSteps*((dgrees/360.0)*2.0*PI*(radius+spinWheelDist/2.0)));
  }

  stepperLeft.moveTo(leftSteps);
  stepperRight.moveTo(rightSteps);

  //If the right wheel is the outside wheel in the turn
  if(rightSteps>=leftSteps){
    int insideWheelSpeed = int((double(turnSpeed)/double(rightSteps))*double(leftSteps));
    setSpeeds(insideWheelSpeed,turnSpeed);
  } else {
    int insideWheelSpeed = int((double(turnSpeed)/double(leftSteps))*double(rightSteps));
    setSpeeds(turnSpeed,insideWheelSpeed);
  }

  runSpeedToActualPosition(); // Blocks until all are in position
}

/*
  forward(double distance)
  This function takes in a distance (in inches) and runs the left and right steppers 
  at the same constant speed until the robot has traveled the given distance in the
  forward direction.

  BLOCKING FUNCTION
*/
void forward(double distance) {
  int steps = int(distance*inchesToSteps);
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
  stepperLeft.moveTo(steps);
  stepperRight.moveTo(steps);
  if(distance < 0){
    stopRobot();
  } else {
    setSpeeds(fwdSpeed,fwdSpeed);
  }

  runSpeedToActualPosition(); // Blocks until all are in position
}

/*
 * Forward but with encoders
 */
void forwardEnc(double distance) {
  
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
  reverse(double distance)
  This function takes in a distance (in inches) and runs the left and right steppers 
  at the same constant speed until the robot has traveled the given distance in the
  reverse direction.

  BLOCKING FUNCTION
*/
void reverse(int distance) {
  int steps = int(distance*inchesToSteps);
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
  stepperLeft.moveTo(-steps);
  stepperRight.moveTo(-steps);
  if(distance < 0){
    stopRobot();
  } else {
    setSpeeds(-revSpeed,-revSpeed);
  }

  steppers.runSpeedToPosition(); // Blocks until all are in position
}
/*
  stop()

  This function stops both stepper motors by setting their speeds to 0

  NON-BLOCKING FUNCTION
*/
void stopRobot() {
  setSpeeds(0,0);
}




/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used
*/
void moveCircle(boolean clockwise, double diameter) {
  digitalWrite(redLED, HIGH);
  turn(clockwise,360.0,diameter/2.0);
  digitalWrite(redLED, LOW);
}

/*
  The moveFigure8() function takes the diameter in inches as the input. It uses the moveCircle() function
  twice with 2 different direcitons to create a figure 8 with circles of the given diameter.
*/
void moveFigure8(double diameter) {
  digitalWrite(redLED, HIGH);
  digitalWrite(ylwLED, HIGH);
  moveCircle(true,diameter);
  delay(1000);
  moveCircle(false,diameter);
  digitalWrite(redLED, LOW);
  digitalWrite(ylwLED, LOW);
}


/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used
*/
void goToAngle(double angle) {
  digitalWrite(grnLED, HIGH);
  encoder[RIGHT] = 0;
  encoder[LEFT] = 0;

  accumTicks[RIGHT] = 0;
  accumTicks[LEFT] = 0;

  double angleTicksGoal = (abs(angle)/360.0)*(PI*double(spinWheelDist)/double(distancePerTick));

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
  digitalWrite(grnLED, LOW);
}

/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used
*/
void goToGoal(int x, int y) {
  digitalWrite(ylwLED, HIGH);
  digitalWrite(grnLED, HIGH);
  y = -y;
  double rads = atan2(y,x);
  double dgrees = rads*(180.0/3.14159265);
  double dist = sqrt(y*y+x*x);
  
  goToAngle(dgrees);
  delay(wait_time);
  forwardEnc(dist);
  digitalWrite(ylwLED, LOW);
  digitalWrite(grnLED, LOW);
}

/*
  moveSquare()

  This function moves the robot in a square shape by calling the forward function
  and the pivot function 4 times. The forward function utilizes encoders to track
  the distance traveled by the robot

  BLOCKING FUNCTION
*/
void moveSquare(double side) {
  digitalWrite(redLED, HIGH);
  digitalWrite(grnLED, HIGH);
  digitalWrite(ylwLED, HIGH);
  for(int i = 0; i < 4; i++) {
    forward(side);
    delay(500);
    pivot(true, 90);
    delay(500);
  }
  stopRobot();
  digitalWrite(redLED, LOW);
  digitalWrite(grnLED, LOW);
  digitalWrite(ylwLED, LOW);
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

#ifdef PS2
/*
 * The readController() function interfaces with the PS2 Controller Reciever to
 * retrieve the most recent joystick and button values.  From these joystick and button
 * values, the speed of the left and right stepper motors are then set accordingly
 * using the AccelStepper library
 */
void readController()
{
  int xAxis;//x axis value
  int yAxis;//y axis value

  ps2x.read_gamepad(false, vibrate);//update ps2 controller information

  if (abs(ps2x.Analog(PSS_LY) - 128) >= joystickDeadband)//if the axis is outside of the set deadband
    yAxis = (ps2x.Analog(PSS_LY) - 128) * -1;//y axis is reversed so reverse it and make it so the center is 0
  else
    yAxis = 0;

  if (abs(ps2x.Analog(PSS_RX) - 128) >= joystickDeadband)//if the axis is outside of the set deadband
    xAxis = (ps2x.Analog(PSS_RX) - 128);//make it so the center is 0
  else
    xAxis = 0;

  if (ps2x.Button(PSB_R2)) //Boost Button(Right Trigger)
  {
    xAxis = map(xAxis, 0, 128, 0, 110);//map axis values to drive percentage from 0 to 110%
    yAxis = map(yAxis, 0, 128, 0, 110);
  }
  else//regular diving
  {
    xAxis = map(xAxis, 0, 128, 0, 50);//map axis values to drive percentages from 0 to 50%
    yAxis = map(yAxis, 0, 128, 0, 50);
  }

  //convert mapped x and y axis values into left and right speed values
  //using an arcade configuration ("driving" controlled by y axis, "turning" controlled by x axis)
  int leftSpeed = (yAxis + xAxis)*10;
  int rightSpeed = (yAxis - xAxis)*10;

  //if robot is commanded to move forward or turn right
  if (yAxis > 0 || xAxis > 0){
    //drive robot forward according to speed values
    setSpeeds(leftSpeed,rightSpeed);//set the motor speeds
  } else if (yAxis < 0 || xAxis < 0){ //robot is commanded to move backwards or turn left
    //drive robot in reverse according to speed values
    setSpeeds(leftSpeed,rightSpeed);//set the motor speeds
  } else{
    stopRobot();
  }

  if(ps2x.ButtonPressed(PSB_TRIANGLE)){
//    stopRobot();
//    goToAngle(60.0);
//    stopRobot();
//   stepperLeft.setCurrentPosition(0)?;
//    stepperLeft.moveTo(525);
//    stepperLeft.setSpeed(300);
//    stepperRight.moveTo(-525);
//    stepperRight.setSpeed(-300);
//    
    spin(true,360.0);
    stopRobot();
    delay(1000);
  } else if(ps2x.ButtonPressed(PSB_CROSS)){
    moveSquare(24.0);
    stopRobot();
    delay(1000);
  } else if(ps2x.ButtonPressed(PSB_CIRCLE)){
    moveCircle(true,24.0);
    stopRobot();
    delay(1000);
  } else if(ps2x.ButtonPressed(PSB_L2)){
    pivot(true,360.0);
    stopRobot();
    delay(1000);
  } else if(ps2x.ButtonPressed(PSB_R3)){
    stopRobot();
    moveFigure8(12.0);
    stopRobot();
  }
  

//  if (ps2x.Button(PSB_L1)) //Left Bumper
//    closeGripper();
//  else if (ps2x.Button(PSB_R1)) //Right Bumper
//    openGripper();
}
#endif

static inline int8_t sgn(double val) {
  if (val < 0) return -1;
  if (val==0) return 0;
  return 1;
}

void runSpeedToActualPosition(){
  boolean runLeft = true;
  boolean runRight = true;
  while(runLeft || runRight){
    
    if(runLeft){
      if(abs(stepperLeft.distanceToGo())<=10){
        runLeft = false;
        stepperLeft.setSpeed(0);
      }
      else if(abs(stepperLeft.distanceToGo())<=50){
        stepperLeft.setSpeed(sgn(stepperLeft.distanceToGo())*50);
        stepperLeft.runSpeed();
      }
      else if(abs(stepperLeft.distanceToGo())<abs(stepperLeft.speed())){
        stepperLeft.setSpeed(stepperLeft.distanceToGo());
        stepperLeft.runSpeed();
      }else {
        stepperLeft.runSpeed();
      }
    }
    if(runRight){
      if(abs(stepperRight.distanceToGo())<=10){
        runRight = false;
        stepperRight.setSpeed(0);
      }
      else if(abs(stepperRight.distanceToGo())<=50){
        stepperRight.setSpeed(sgn(stepperRight.distanceToGo())*50);
        stepperRight.runSpeed();
      }
      else if(abs(stepperRight.distanceToGo())<abs(stepperRight.speed())){
        stepperRight.setSpeed(stepperRight.distanceToGo());
        stepperRight.runSpeed();
      }
      else{
        stepperRight.runSpeed();
      }
    }
  }
}

/*
   The move1() function will move the robot forward one full rotation and backwared on
   full rotation.  Recall that that there 200 steps in one full rotation or 1.8 degrees per
   step. This function uses setting the step pins high and low with delays to move. The speed is set by
   the length of the delay.
*/
void move1() {
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(ltDirPin, HIGH); // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, HIGH); // Enables the motor to move in a particular direction
  // Makes 800 pulses for making one full cycle rotation
  for (int x = 0; x < 800; x++) {
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
  }
  delay(1000); // One second delay
  digitalWrite(ltDirPin, LOW); // Enables the motor to move in opposite direction
  digitalWrite(rtDirPin, LOW); // Enables the motor to move in opposite direction
  // Makes 800 pulses for making one full cycle rotation
  for (int x = 0; x < 800; x++) {
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
  }
  delay(1000); // One second delay
}

/*
   The move2() function will use AccelStepper library functions to move the robot
   move() is a library function for relative movement to set a target position
   moveTo() is a library function for absolute movement to set a target position
   stop() is a library function that causes the stepper to stop as quickly as possible
   run() is a library function that uses accel and decel to achieve target position, no blocking
   runSpeed() is a library function that uses constant speed to achieve target position, no blocking
   runToPosition() is a library function that uses blocking with accel/decel to achieve target position
   runSpeedToPosition() is a library function that uses constant speed to achieve target posiiton, no blocking
   runToNewPosition() is a library function that uses blocking with accel/decel to achieve target posiiton
*/
void move2() {
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  stepperRight.moveTo(800);//move one full rotation forward relative to current position
  stepperLeft.moveTo(800);//move one full rotation forward relative to current position
  stepperRight.setSpeed(1000);//set right motor speed
  stepperLeft.setSpeed(1000);//set left motor speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor
  runToStop();//run until the robot reaches the target
 stepperRight.moveTo(1600);//move one full rotation forward relative to current position
  stepperLeft.moveTo(1600);//move one full rotation forward relative to current position
  stepperRight.setSpeed(1000);//set right motor speed
  stepperLeft.setSpeed(1000);//set left motor speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor
  runToStop();//run until the robot reaches the target
  delay(1000); // One second delay
  stepperRight.moveTo(0);//move one full rotation backward relative to current position
  stepperLeft.moveTo(0);//move one full rotation backward relative to current position
  stepperRight.setSpeed(1000);//set right motor speed
  stepperLeft.setSpeed(1000);//set left motor speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor
  runToStop();//run until the robot reaches the target
  delay(1000); // One second delay
}

/*
   The move3() function will use the MultiStepper() class to move both motors at once
   move() is a library function for relative movement to set a target position
   moveTo() is a library function for absolute movement to set a target position
   stop() is a library function that causes the stepper to stop as quickly as possible
   run() is a library function that uses accel and decel to achieve target position, no blocking
   runSpeed() is a library function that uses constant speed to achieve target position, no blocking
   runToPosition() is a library function that uses blocking with accel/decel to achieve target position
   runSpeedToPosition() is a library function that uses constant speed to achieve target posiiton, no blocking
   runToNewPosition() is a library function that uses blocking with accel/decel to achieve target posiiton
*/
void move3() {
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  long positions[2]; // Array of desired stepper positions
  positions[0] = 800;//right motor absolute position
  positions[1] = 800;//left motor absolute position
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);//wait one second
  // Move to a different coordinate
  positions[0] = 0;//right motor absolute position
  positions[1] = 0;//left motor absolute position
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);//wait one second
}

/*this function will move to target at 2 different speeds*/
void move4() {
  long positions[2]; // Array of desired stepper positions
  int leftPos = 1500;//right motor absolute position
  int rightPos = 1000;//left motor absolute position
  int leftSpd = 1500;//right motor speed
  int rightSpd = 1000; //left motor speed

  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED

  //Uncomment the next 4 lines for absolute movement
  stepperLeft.setCurrentPosition(0);//set left wheel position to zero
  stepperRight.setCurrentPosition(0);//set right wheel position to zero
  stepperLeft.moveTo(leftPos);//move left wheel to absolute position
  stepperRight.moveTo(rightPos);//move right wheel to absolute position

  //Unomment the next 2 lines for relative movement
//  stepperLeft.move(leftPos);//move left wheel to relative position
//  stepperRight.move(rightPos);//move right wheel to relative position

  stepperLeft.setSpeed(leftSpd);//set left motor speed
  stepperRight.setSpeed(rightSpd);//set right motor speed
  //runAtSpeedToPosition();//run at speed to target position
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);
}

/*This function will move continuously at 2 different speeds*/
void move5() {
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  int leftSpd = 5000;//right motor speed
  int rightSpd = 1000; //left motor speed
  stepperLeft.setSpeed(leftSpd);//set left motor speed
  stepperRight.setSpeed(rightSpd);//set right motor speed
  runAtSpeed();
}

/*This function, runToStop(), will run the robot until the target is achieved and
   then stop it
*/
void runToStop ( void ) {
  int runNow = 1;
  int rightStopped = 0;
  int leftStopped = 0;

  while (runNow) {
    if (!stepperRight.run()) {
      rightStopped = 1;
      stepperRight.stop();//stop right motor
    }
    if (!stepperLeft.run()) {
      leftStopped = 1;
      stepperLeft.stop();//stop left motor
    }
    if (rightStopped && leftStopped) {
      runNow = 0;
    }
  }
}

/*function to run both wheels to a position at speed*/
void runAtSpeedToPosition() {
  stepperRight.runSpeedToPosition();
  stepperLeft.runSpeedToPosition();
}

/*function to run both wheels continuously at a speed*/
void runAtSpeed ( void ) {
  while (stepperRight.runSpeed() || stepperLeft.runSpeed()) {
  }
}
