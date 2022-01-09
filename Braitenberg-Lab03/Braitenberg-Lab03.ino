/************************************
  Braitenberg-Lab03.ino
  Jordan Asman and Cory Snyder 1.6.2022

  This program will introduce using the stepper motor library to create motion algorithms for the robot.
  The motions will be goToAngle, goToGoal, moveCircle, moveSquare, moveFigure8 and basic movement (stop, forward, spin, reverse, turn)
  There is the inclusion of the PS2 Controller and PS2 Conroller libaray to allow for teleoperated movement and quick execution of the different 
  motions by pressing the buttons
  

  The primary functions created are
  forward, reverse - given the distance in inches, both wheels move with same velocity and same direction
  forwardEnc - same as the forward function, but it uses the encoders to go a distance instead of stepper steps
  pivot - given the direction of clockwise or counterclockwise and the degrees, one wheel stationary, one wheel moves forward or back
  spin - given the direction of clockwise or counterclockwise and the degrees, both wheels move with same velocity but opposite directions
  turn - given the direction of clockwise or counterclockwise, the degrees, and the turn radius in inches, both wheels move with same direction different velocity
  stopRobot - both wheels stationary
  setSpeeds - given the speed for the left and right stepper motors, this sets the speed.  Has the right adjustment factor to help the robot go straight
  runSpeedToActualPosition - Custom implementation of the Multistepper runSpeedToPosition.  Has decceleration at the end of the motion.
  calibrate - given the wheel and direction, this will spin the wheel until a encoder tick is registered. Ensures consistent encoder behavior when used in other functions
  moveSquare - given the side length in inches of a square, move the robot in a square using encoders
  moveCircle - given the diameter in inches and direction of clockwise or counterclockwise, move the robot in a circle with that diameter
  moveFigure8 - given the diameter in inches, use the moveCircle() function with direction input to create a Figure 8
  goToAngle - given an angle in degrees, the robot spins and then uses encoders to ensure that the spin was accurate
  goToGoal - given a coordiate location in inches (x,y), the robot will spin and then drive forward with encoders to move itself to that coordinate

  Hardware Connections:
  digital pin 13 - enable LED on microcontroller
  digital pin 48 - enable PIN on A4988 Stepper Motor Driver StepSTICK
  digital pin 50 - right stepper motor step pin
  digital pin 51 - right stepper motor direction pin
  digital pin 52 - left stepper motor step pin
  digital pin 53 - left stepper motor direction pin

  digital pin 5 - red LED in series with 220 ohm resistor
  digital pin 6 - green LED in series with 220 ohm resistor
  digital pin 7 - yellow LED in series with 220 ohm resistor

  digital pin 22 - Data  pin for the PS2 Communication Module
  digital pin 23 - Command pin for the PS2 Communication Module
  digital pin 24 - Attention pin for the PS2 Communication Module
  digital pin 25 - Clock pin for the PS2 Communication Module

  digital pin 2 - left stepper motor encoder pin
  digital pin 3 - right stepper motor encoder pin

  analog pin A15 - back IR sensor
  analog pin A14 - left IR sensor
  analog pin A13 - front IR sensor
  analog pin A12 - right IR sensor

  digital pin 19 (has Interrupts) - right Sonar sensor 
  digital pin 18 (has Interrupts) - left Sonar sensor
*/

#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include <NewPing.h>
#include <RunningMedian.h>

//Motor Defines and Variables --------------------------------------------------------------------------------------------------
#define R_STEP_PIN  50 //right stepper motor step pin
#define R_DIR_PIN   51 //right stepper motor direction pin
#define L_STEP_PIN  52 //left stepper motor step pin
#define L_DIR_PIN   53 //left stepper motor direction pin
#define L_ENC_PIN   2  //left encoder pin
#define R_ENC_PIN   3  //right encoder pin
#define STEP_EN_PIN 48 //stepper enable pin on stepStick 

#define STEP_ENABLE false //variable for enabling stepper motor
#define STEP_DISABLE true //variable for disabling stepper motor

const double stepsPerRev = 800;
const double wheelDiameter = 3.375; //diameter of the wheel in inches
const double spinWheelDist = 8.55;  //distance between the wheels (Adjusted for carpet and the spin func)
const double pivotWheelDist = 8.55;  //distance between the wheels (Adjusted for carpet and the spin func)
const double stepsToInches = wheelDiameter*PI/stepsPerRev;
const double inchesToSteps = stepsPerRev/(wheelDiameter*PI);
const double ticksPerRev = 20.0;    // number of ticks encoder sees per revolution
const double distancePerTick = wheelDiameter*PI/ticksPerRev; // the distance moved for each tick of the encoder
const double pivotDegreesToSteps = (2*PI*pivotWheelDist*inchesToSteps)/360.0;
const double spinDegreesToSteps = (PI*spinWheelDist*inchesToSteps)/360.0;
const double rightSpeedAdjustment = 1;

#define maximumSpeed 1500
#define maxAccel 1000
#define fwdSpeed 200
#define revSpeed 500
#define spinSpeed 200
#define turnSpeed 400
#define readySpeed 100            // speed for slightly adjusting wheels to calibrate encoders
#define collideSpeed 500
#define avoidSpeed 80
#define CLOCKWISE true
#define COUNTERCLOCKWISE false

#define COLLIDE_DIST 5.0

#define LEFT  0                     //constant for left wheel
#define RIGHT 1                     //constant for right wheel
volatile long encoder[2] = {0, 0};  //interrupt variable to hold number of encoder counts (left, right)
int lastSpeed[2] = {0, 0};          //variable to hold encoder speed (left, right)
int accumTicks[2] = {0, 0};         //variable to hold accumulated ticks since last reset

AccelStepper stepperRight(AccelStepper::DRIVER, R_STEP_PIN, R_DIR_PIN);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, L_STEP_PIN, L_DIR_PIN);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;//create instance to control multiple steppers at the same time

//IR Defines and Variables -----------------------------------------------------------------------------------------------------
#define LEFT_IR A14
#define RIGHT_IR A12
#define FRONT_IR A13
#define BACK_IR A15
#define NUM_IR_SAMPLES 20
RunningMedian irData(NUM_IR_SAMPLES);

// These functions are our custom rational fitted curves to convert the IR sensor's analog values to inches
static inline double frontIRToInches(double analog) {return (-0.9298*analog + 1358.0)/(analog+28.26);}
static inline double rightIRToInches(double analog) {return (-0.3141*analog + 1865.0)/(analog-48.02);}
static inline double leftIRToInches(double analog) {return (-0.7801*analog + 2136.0)/(analog-20.15);}
static inline double backIRToInches(double analog) {return (-0.6391*analog + 1144.0)/(analog+14.04);}



//Sonar Defines and Variables --------------------------------------------------------------------------------------------------

#define LEFT_SONAR 18  //Pin on the Mega2560 that has an interrupt
#define RIGHT_SONAR 19 //Pin on the Mega2560 that has an interrupt
volatile uint32_t leftEchoStart = 0;                      // Records start of left echo pulse 
volatile uint32_t leftEchoEnd = 0;                        // Records end of left echo pulse
volatile uint32_t rightEchoStart = 0;                     // Records start of right echo pulse 
volatile uint32_t rightEchoEnd = 0;                       // Records end of left echo pulse
const double soundInchRoundTrip = 146.0;                  // The microseconds it takes sound to go 2 inches (1 inch round trip)

RunningMedian leftSonarData(5);
RunningMedian rightSonarData(5);


#define enableLED 13    //stepper enabled LED
#define redLED 5        //red LED for displaying states
#define grnLED 6        //green LED for displaying states
#define ylwLED 7        //yellow LED for displaying states

#define pauseTime 1000 //time before robot moves
#define wait_time 1000 //time to wait between prints and starting robot

//interrupt function to count left encoder tickes
void LwheelSpeed()
{
  encoder[LEFT]++;  //count the left wheel encoder interrupts
  accumTicks[LEFT]++; // count accumulated ticks for left wheel
}

//interrupt function to count right encoder ticks
void RwheelSpeed()
{
  encoder[RIGHT]++; //count the right wheel encoder interrupts
  accumTicks[RIGHT]++; // count accumulated ticks for right wheel
}

void leftSonarInt(){
  switch (digitalRead(LEFT_SONAR)){                   // Test to see if the signal is high or low
    case HIGH:                                        // High so must be the start of the echo pulse
      leftEchoEnd = 0;                                // Clear the end time
      leftEchoStart = micros();                       // Save the start time
      break;
    case LOW:                                         // Low so must be the end of hte echo pulse
      leftEchoEnd = micros();                         // Save the end time
      long temp = leftEchoEnd - leftEchoStart;
      if(temp < 2000){
        leftSonarData.add(temp);                      // Calculate the pulse duration and add it to the running median
      }
      break;
  }
}

void rightSonarInt(){
  switch(digitalRead(RIGHT_SONAR)){                      // Test to see if the signal is high or low
    case HIGH:                                           // High so must be the start of the echo pulse
      rightEchoEnd = 0;                                  // Clear the end time
      rightEchoStart = micros();                         // Save the start time
      break;
    case LOW:                                            // Low so must be the end of hte echo pulse
      rightEchoEnd = micros();                           // Save the end time
      long temp = rightEchoEnd - rightEchoStart;
      if(temp < 2000){
        rightSonarData.add(temp);                        // Calculate the pulse duration and add it to the running median
      }
      
      break;
  }
}

void initiateSonarRead(){
  // Start the left sonar
  detachInterrupt(digitalPinToInterrupt(LEFT_SONAR));
  pinMode(LEFT_SONAR,OUTPUT);
  digitalWrite(LEFT_SONAR,LOW);
  delayMicroseconds(4);
  digitalWrite(LEFT_SONAR,HIGH);
  delayMicroseconds(10);
  digitalWrite(LEFT_SONAR,LOW);
  pinMode(LEFT_SONAR,INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_SONAR), leftSonarInt, CHANGE);

  // Start the right sonar
  detachInterrupt(digitalPinToInterrupt(RIGHT_SONAR));
  pinMode(RIGHT_SONAR,OUTPUT);
  digitalWrite(RIGHT_SONAR,LOW);
  delayMicroseconds(4);
  digitalWrite(RIGHT_SONAR,HIGH);
  delayMicroseconds(10);
  digitalWrite(RIGHT_SONAR,LOW);
  pinMode(RIGHT_SONAR,INPUT);
  attachInterrupt(digitalPinToInterrupt(RIGHT_SONAR), rightSonarInt, CHANGE);
}


void setup()
{
// START SERIAL MONITOR
  Serial.begin(9600); //start serial communication at 9600 baud rate for debugging

// SETUP DEBUG LEDS
  pinMode(redLED, OUTPUT);                      //set red LED as output
  pinMode(grnLED, OUTPUT);                      //set green LED as output
  pinMode(ylwLED, OUTPUT);                      //set yellow LED as output

// SETUP STEPPER MOTORS
  pinMode(R_STEP_PIN, OUTPUT);                  //sets pin as output
  pinMode(R_DIR_PIN, OUTPUT);                   //sets pin as output
  pinMode(L_STEP_PIN, OUTPUT);                  //sets pin as output
  pinMode(L_DIR_PIN, OUTPUT);                   //sets pin as output
  pinMode(STEP_EN_PIN, OUTPUT);                 //sets pin as output
  digitalWrite(STEP_EN_PIN, STEP_DISABLE);      //turns off the stepper motor driver
  pinMode(enableLED, OUTPUT);                   //set enable LED as output
  digitalWrite(enableLED, LOW);                 //turn off enable LED
  digitalWrite(redLED, HIGH);                   //turn on red LED
  digitalWrite(ylwLED, HIGH);                   //turn on yellow LED
  digitalWrite(grnLED, HIGH);                   //turn on green LED
  delay(pauseTime / 5);                         //wait 0.5 seconds
  digitalWrite(redLED, LOW);                    //turn off red LED
  digitalWrite(ylwLED, LOW);                    //turn off yellow LED
  digitalWrite(grnLED, LOW);                    //turn off green LED

// SETUP ALL 4 IR SENSORS
  pinMode(LEFT_IR, INPUT);                      //sets pin as input
  pinMode(RIGHT_IR, INPUT);                     //sets pin as input
  pinMode(BACK_IR, INPUT);                      //sets pin as input
  pinMode(FRONT_IR, INPUT);                     //sets pin as input

// SETUP LEFT AND RIGHT SONARS
  pinMode(LEFT_SONAR, INPUT);                   //sets pin as input
  pinMode(RIGHT_SONAR,INPUT);                   //sets pin as input
  attachInterrupt(digitalPinToInterrupt(LEFT_SONAR), leftSonarInt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_SONAR), rightSonarInt, CHANGE);
  NewPing::timer_ms(20, initiateSonarRead); // Create a Timer2 interrupt that calls initiateSonarRead every 15 ms
  
  
  attachInterrupt(digitalPinToInterrupt(L_ENC_PIN), LwheelSpeed, CHANGE); //init the interrupt mode for the digital pin 2  
  attachInterrupt(digitalPinToInterrupt(R_ENC_PIN), RwheelSpeed, CHANGE); //init the interrupt mode for the digital pin 3

  stepperRight.setMaxSpeed(maximumSpeed);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setMaxSpeed(maximumSpeed);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(maxAccel);//set desired acceleration in steps/s^2
  stepperLeft.setAcceleration(maxAccel);//set desired acceleration in steps/s^2
  steppers.addStepper(stepperRight);//add right motor to MultiStepper
  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
  digitalWrite(STEP_EN_PIN, STEP_ENABLE);//turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);//turn on enable LED
  delay(pauseTime); //always wait 5 seconds before the robot moves
}

unsigned long last_read = 0;
double robotHeading = 0.0;
double goalX = 0.0;
double goalY = -48.0;
double leftSonarInch = 0.0;

double leftWheelSpeed = 250.0;
double rightWheelSpeed = 250.0;

int state = 0;

#define FOLLOW_DISTANCE_LOW 4
#define FOLLOW_DISTANCE_HIGH 6
#define WALL_DETECT_DIST 10
#define IR_WALL_DETECT_DIST 4
#define SPEED_DIFF 50

#define RANDOM_WANDER 0
#define LEFT_WALL 1
#define RIGHT_WALL 2
#define BOTH_WALLS 3
#define AVOID 4

// Global force variable.  Gets updated by the feelForce() function every 100 ms
double force[2];
void loop()
{
//  //Every 100 milliseconds, poll the IR sensors 
  int temp = millis() - last_read;
  if(temp > 50){
    stateMachine();
    last_read = millis();
  }

//  followCenter(getLinearizedDistance(LEFT_SONAR), getLinearizedDistance(RIGHT_SONAR));

  //Run the stepper motors at the speed they were set to in the readController() function
  stepperRight.runSpeed();
  stepperLeft.runSpeed();

}

void stateMachine() {
  double leftSonarDist = getLinearizedDistance(LEFT_SONAR);
  double rightSonarDist = getLinearizedDistance(RIGHT_SONAR);
  double frontIRDist = getLinearizedDistance(FRONT_IR);
  double backIRDist = getLinearizedDistance(BACK_IR);
  switch (state) {
  case RANDOM_WANDER:
    randomWander();
    if (leftSonarDist <= WALL_DETECT_DIST && rightSonarDist <= WALL_DETECT_DIST) {
      state = BOTH_WALLS;
    } else if (leftSonarDist <= WALL_DETECT_DIST) {
      state = LEFT_WALL;
    } else if (rightSonarDist <= WALL_DETECT_DIST) {
      state = RIGHT_WALL;
    } else if (frontIRDist <= IR_WALL_DETECT_DIST || backIRDist <= IR_WALL_DETECT_DIST) {
      state = AVOID;
    }
    break;
  case LEFT_WALL:
    digitalWrite(ylwLED, HIGH);
    followLeftWall(leftSonarDist);
    if (rightSonarDist <= WALL_DETECT_DIST) {
      state = BOTH_WALLS;
    } else if (leftSonarDist > WALL_DETECT_DIST) {
      state = RANDOM_WANDER;
    } else if (frontIRDist <= IR_WALL_DETECT_DIST || backIRDist <= IR_WALL_DETECT_DIST) {
      // This will be inside corner later
      state = AVOID;
    }
    digitalWrite(ylwLED, LOW);
    break;
  case RIGHT_WALL:
  digitalWrite(redLED, HIGH);
    followRightWall(rightSonarDist);
    if (leftSonarDist <= WALL_DETECT_DIST) {
      state = BOTH_WALLS;
    } else if (rightSonarDist > WALL_DETECT_DIST) {
      state = RANDOM_WANDER;
    } else if (frontIRDist <= IR_WALL_DETECT_DIST || backIRDist <= IR_WALL_DETECT_DIST) {
      // This will be inside corner later
      state = AVOID;
    }
    digitalWrite(redLED, LOW);
    break;
  case BOTH_WALLS:
  digitalWrite(grnLED, HIGH);
    followCenter(leftSonarDist, rightSonarDist);
    if (leftSonarDist > WALL_DETECT_DIST && rightSonarDist > WALL_DETECT_DIST) {
      state = RANDOM_WANDER;
    } else if (rightSonarDist > WALL_DETECT_DIST) {
      state = LEFT_WALL;
    } else if (leftSonarDist > WALL_DETECT_DIST) {
      state = RIGHT_WALL;
    } else if (frontIRDist <= IR_WALL_DETECT_DIST || backIRDist <= IR_WALL_DETECT_DIST) {
      // This will be inside corner later
      state = AVOID;
    }
    digitalWrite(grnLED, LOW);
    break;
  case AVOID:
    state = RANDOM_WANDER;
    break;
  default:
    state = RANDOM_WANDER;
    break;
  }
}

/* followLeftWall(double dist)
 *  This function uses the distance from the left sonar to set the speeds of
 *  the right and left wheels to exhibit bang bang control when a left wall is
 *  present
 */
void followLeftWall(double dist) {
  if (dist > FOLLOW_DISTANCE_HIGH) {
    setSpeeds(leftWheelSpeed - SPEED_DIFF, rightWheelSpeed + SPEED_DIFF);
  } else if (dist < FOLLOW_DISTANCE_LOW){
    setSpeeds(leftWheelSpeed + SPEED_DIFF, rightWheelSpeed - SPEED_DIFF);
  } else {
    setSpeeds(leftWheelSpeed, rightWheelSpeed);
  }
}

/* followRightWall(double dist)
 *  This function uses the distance from the right sonar to set the speeds of
 *  the right and left wheels to exhibit bang bang control when a right wall is
 *  present
 */
void followRightWall(double dist) {
  if (dist > FOLLOW_DISTANCE_HIGH) {
    setSpeeds(leftWheelSpeed + SPEED_DIFF, rightWheelSpeed - SPEED_DIFF);
  } else if (dist < FOLLOW_DISTANCE_LOW) {
    setSpeeds(leftWheelSpeed - SPEED_DIFF, rightWheelSpeed + SPEED_DIFF);
  } else {
    setSpeeds(leftWheelSpeed, rightWheelSpeed);
  }
}

/* followCenter(double leftDist, double rightDist)
 *  This function uses the distance from the left and right sonars to set the 
 *  speeds of the right and left wheels to exhibit bang bang control when both 
 *  left and right walls are present
 */
void followCenter(double leftDist, double rightDist) {
  double diff = leftDist - rightDist;  
  if (diff > -1 && diff < 1) {
    setSpeeds(leftWheelSpeed, rightWheelSpeed);
  } else if (diff < -1) {
    setSpeeds(leftWheelSpeed + SPEED_DIFF, rightWheelSpeed - SPEED_DIFF);
  } else {
    setSpeeds(leftWheelSpeed - SPEED_DIFF, rightWheelSpeed + SPEED_DIFF);
  }
}

void insideCorner(double leftSonarDist, double rightSonarDist, double frontIRDist) {
  
}

/* randomWander()
 *  This function sets the speeds of the left and the right steppers to a random speed
 *  every 1 second
 */
double randLSpeed = 0.0;
double randRSpeed = 0.0;
long lastRandom = 0;

void randomWander() {
  digitalWrite(grnLED,HIGH);
  if(millis() - lastRandom > 1000){
      randLSpeed = random(800);
      randRSpeed = random(800);
      
      lastRandom = millis();
  }
  setSpeeds(randLSpeed,randRSpeed);
  digitalWrite(grnLED,LOW);
}

/* 
 * feelForce()
 * 
 * This function reads the 4 IR sensors and returns the [angle,magnitude] of the force that the robot feels
 */
void feelForce() {
  double frontDist = getLinearizedDistance(FRONT_IR);
  double backDist = getLinearizedDistance(BACK_IR);
  double rightDist = getLinearizedDistance(RIGHT_IR);
  double leftDist = getLinearizedDistance(LEFT_IR);
  double xForce = 0.0;
  double yForce = 0.0;
  // The front IR feels forces that push in the negative x direction
  // The back IR feels forces that push in the positive x direction
  xForce += -1*(12.0 - frontDist) + (12.0 - backDist);
  // The left IR feels forces that push in the negative y direction
  // The right IR feels forces that push in the positive y direction
  yForce += -1*(12.0 - leftDist) + (12.0 - rightDist);


  force[1] = sqrt(xForce*xForce+yForce*yForce);
  force[0] = atan2(yForce,xForce);
//  Serial.println("---------------------------------------------------------------------------");
//  Serial.print("Front: ");Serial.print(frontDist);Serial.print("   Right: ");Serial.print(rightDist);
//  Serial.print("Back: ");Serial.print(backDist);Serial.print("   Left: ");Serial.println(leftDist);
//  Serial.print("YForce: ");Serial.print(yForce);Serial.print("   XForce: ");Serial.println(xForce);
//  Serial.print("Angle: ");Serial.print(force[0]);
}

void collide(int sensor){
  digitalWrite(redLED, HIGH);
  setSpeeds(collideSpeed,collideSpeed);
  while(getLinearizedDistance(sensor) > COLLIDE_DIST){
    stepperLeft.runSpeed();
    stepperRight.runSpeed();
  }
  stopRobot();
  digitalWrite(redLED, LOW);
}

boolean detectedObstacle(){
  double frontDist = getLinearizedDistance(FRONT_IR);
  double backDist = getLinearizedDistance(BACK_IR);
  double rightDist = getLinearizedDistance(RIGHT_IR);
  double leftDist = getLinearizedDistance(LEFT_IR);

  return frontDist <= COLLIDE_DIST || backDist <= COLLIDE_DIST || leftDist <= COLLIDE_DIST || rightDist <= COLLIDE_DIST;
}

void runAway(){
  double frontDist = getLinearizedDistance(FRONT_IR);
  double backDist = getLinearizedDistance(BACK_IR);
  double rightDist = getLinearizedDistance(RIGHT_IR);
  double leftDist = getLinearizedDistance(LEFT_IR);
  double xForce = 0.0;
  double yForce = 0.0;
  // The front IR feels forces that push in the negative x direction
  // The back IR feels forces that push in the positive x direction
  xForce += -1*(12.0 - frontDist) + (12.0 - backDist);
  // The left IR feels forces that push in the negative y direction
  // The right IR feels forces that push in the positive y direction
  yForce += -1*(12.0 - leftDist) + (12.0 - rightDist);
  
  double magnitude = sqrt(xForce*xForce+yForce*yForce);
  double angle = atan2(yForce,xForce)*180.0/PI;
  if(xForce < 0.0) setSpeeds(-300,-300);
  else setSpeeds(300,300);


  if(frontDist <= 4.0 && backDist <= 4.0 && leftDist <= 4.0 && rightDist <= 4.0){
    //DO NOTHING
    setSpeeds(0.0,0.0);
  } else if(frontDist <= 4.0 && backDist <= 4.0){
    if(yForce>0.0){
      spin(CLOCKWISE,90.0);
      if(yForce < 0.0) setSpeeds(300,300);
      else setSpeeds(-300,-300);
    } else{
      spin(COUNTERCLOCKWISE,90.0);
      if(yForce < 0.0) setSpeeds(-300,-300);
      else setSpeeds(300,300);
    }
  }
  
  while(detectedObstacle()){
    stepperLeft.runSpeed();
    stepperRight.runSpeed();
  }
  long timer = millis();
  while(millis()-timer<4000){
    stepperLeft.runSpeed();
    stepperRight.runSpeed();
  }

  spin(CLOCKWISE,angle);
}


/*
 * getLinearizedDistance(sensorPin)
 * 
 * Depending on the type of distance sensor (sonar or IR) and the specific sensor,
 * this function linearizes the readings and uses a running median to get an accurate 
 * distance.  
 * 
 * The IR sensors have each been specifically tuned for their own linearization Ex. rightIRToInches()
 * The Sonar sensors are being polled every 20 ms by a timer interrupt, so their data just has to be 
 * grabbed from the running median and then processed
 */
double getLinearizedDistance(int sensor){
  double value = 0.0;
  switch(sensor){
    case RIGHT_IR:
      for (int i = 0; i < NUM_IR_SAMPLES-1; i++) {
        irData.add(analogRead(RIGHT_IR));
      }
      value = rightIRToInches(irData.getMedian());
      value = min(value,4.0);
      if(value == 4.0) value = 12.0;//6.0
      value = max(value,2.0);
      return value;
      break;
    case LEFT_IR:
      for (int i = 0; i < NUM_IR_SAMPLES-1; i++) {
        irData.add(analogRead(LEFT_IR));
      }
      value = leftIRToInches(irData.getMedian());
      value = min(value,4.0);
      if(value == 4.0) value = 12.0;//6
      value = max(value,2.0);
      return value;
      break;
    case FRONT_IR:
      for (int i = 0; i < NUM_IR_SAMPLES-1; i++) {
        irData.add(analogRead(FRONT_IR));
      }
      value = frontIRToInches(irData.getMedian());
      value = min(value,4.0);
      if(value == 4.0) value = 12.0;
      value = max(value,2.0);
      return value;
      break;
    case BACK_IR:
      for (int i = 0; i < NUM_IR_SAMPLES-1; i++) {
        irData.add(analogRead(BACK_IR));
      }
      value = backIRToInches(irData.getMedian());
      value = min(value,4.0);
      if(value == 4.0) value = 12.0;//6
      value = max(value,2.0);
      return value;
      break;
    case LEFT_SONAR:
      return (double(leftSonarData.getMedian())/soundInchRoundTrip)-1;
      break;
    case RIGHT_SONAR:
      return (double(rightSonarData.getMedian())/soundInchRoundTrip)-1;
      break;
    default:
      break;
  }
}

/*
  calibrate(left, right, clockwise)

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
 * pivot(clockwise, degrees)
 * 
 * This function takes in a boolean for clockwise or counterclockwise and then
 * a number of degrees that the robot is going to pivot. Pivoting is where one wheel sits 
 * still while the other drives.  
 * 
 * Clockwise will spin the left wheel and keep the right still
 * Counterclockwise will spin the right wheel and keep the left still
 * 
 * BLOCKING FUNCTION
 */
void pivot(boolean clockwise, double dgrees) {
  int steps = int(dgrees*pivotDegreesToSteps);
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
  
  if(clockwise){
    stepperLeft.moveTo(steps);
    setSpeeds(sgn(steps)*spinSpeed,0);
  } else {
    stepperRight.moveTo(steps);
    setSpeeds(0,sgn(steps)*spinSpeed);
  }

  runSpeedToActualPosition(); // Blocks until all are in position
}

/*
 * spin(clockwise, degrees)
 * 
 * This function takes in a boolean for clockwise or counterclockwise and then
 * a number of degrees that the robot is going to spin. Spinning is where both wheels
 * move opposite directions at the same speed.
 * 
 * BLOCKING FUNCTION
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
 * turn(clockwise, degrees, radius)
 * 
 * This function takes in a boolean for clockwise or counterclockwise, the radius of the arc/turn in 
 * inches, and then a number of degrees that the robot is going to drive around the arc/turn. 
 * 
 * BLOCKING FUNCTION
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
  if(abs(rightSteps)>=abs(leftSteps)){
    int insideWheelSpeed = int((double(turnSpeed)/double(abs(rightSteps)))*double(abs(leftSteps)));
    setSpeeds(sgn(leftSteps)*insideWheelSpeed,sgn(rightSteps)*turnSpeed);
  } else {
    int insideWheelSpeed = int((double(turnSpeed)/double(abs(leftSteps)))*double(abs(rightSteps)));
    setSpeeds(sgn(leftSteps)*turnSpeed,sgn(rightSteps)*insideWheelSpeed);
  }

  runSpeedToActualPosition(); // Blocks until all are in position
}

void turnSpeed2(boolean clockwise, double dgrees, double radius) {
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

//  stepperLeft.moveTo(leftSteps);
//  stepperRight.moveTo(rightSteps);

  //If the right wheel is the outside wheel in the turn
  if(abs(rightSteps)>=abs(leftSteps)){
    int insideWheelSpeed = int((double(turnSpeed)/double(abs(rightSteps)))*double(abs(leftSteps)));
    setSpeeds(sgn(leftSteps)*insideWheelSpeed,sgn(rightSteps)*turnSpeed);
  } else {
    int insideWheelSpeed = int((double(turnSpeed)/double(abs(leftSteps)))*double(abs(rightSteps)));
    setSpeeds(sgn(leftSteps)*turnSpeed,sgn(rightSteps)*insideWheelSpeed);
  }

//  runSpeedToActualPosition(); // Blocks until all are in position
}

/*
  forward(double distance)
  
  This function takes in a distance (in inches) and runs the left and right steppers 
  at the same constant speed until the robot has traveled the given distance in the
  forward direction.

  BLOCKING FUNCTION
*/
void forward(double distance) {
//  stepperLeft.setCurrentPosition(0);
//  stepperRight.setCurrentPosition(0);
  
  int steps = int(distance*inchesToSteps);
  stepperLeft.moveTo(steps+stepperLeft.currentPosition());
  stepperRight.moveTo(steps+stepperRight.currentPosition());
  
  if(distance < 0){
    stopRobot();
  } else {
    setSpeeds(fwdSpeed,fwdSpeed);
  }

  runSpeedToActualPosition(); // Blocks until all are in position
}

/*
 * forwardEnc(distance)
 * 
 * Same as the forward() function but  it uses the encoder ticks rather than 
 * stepper steps
 * 
 * BLOCKING FUNCTION
 */
void forwardEnc(double distance) {
  // caculate the target value based on the desired distance
  int leftEncoderTarget = int(distance/double(distancePerTick));    

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
*/
void stopRobot() {
  setSpeeds(0,0);
}


/*
  moveCircle(clockwise, diameter)

  This function takes in a the direction of the circle (clock or counterclockwise) and then
  the diameter in inches.  

  BLOCKING FUNCTION
*/
void moveCircle(boolean clockwise, double diameter) {
  digitalWrite(redLED, HIGH);
  turn(clockwise,360.0,diameter/2.0);
  digitalWrite(redLED, LOW);
}

/*
  The moveFigure8() 
  
  Takes the diameter in inches as the input. It uses the moveCircle() function
  twice with 2 different direcitons to create a figure 8 with circles of the given diameter.

  BLOCKING FUNCTION
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
  goToAngle(angle)

  This function uses the spin function to go to a specific angle in degrees, but it uses the encoders 
  to adjust for inconsistencies after the spin happens

  BLOCKING FUNCTION
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
 * goToGoal(x,y)
 * 
 * This function takes in an x and y coordinate in inches and moves the robot to that location
 * 
 * The positive x axis is out of the front of the robot, and then the positive y axis is coming 
 * out of the left wheel
 * 
 * BLOCKING FUNCTION
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
  moveSquare(sideLength)

  This function moves the robot in a square shape. The sideLength is in inches. 
  by calling the forward function
  and the pivot function 4 times. The forward function utilizes encoders to track
  the distance traveled by the robot

  BLOCKING FUNCTION
*/
void moveSquare(double side) {
  digitalWrite(redLED, HIGH);
  digitalWrite(grnLED, HIGH);
  digitalWrite(ylwLED, HIGH);
  for(int i = 0; i < 4; i++) {
    forwardEnc(side);
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
 *  
 *  This function should be called as apposed to using stepper.setSpeed() when possible
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
 * 
 * Several buttons on the controller are also setup to run different pre-planned actions
 * like moveSquare(), and spin().  These functions are BLOCKING, so there is a chance this function
 * BLOCKS.
 * 
 * SOMETIMES BLOCKING
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
    spin(CLOCKWISE,360.0);
    stopRobot();
    delay(1000);
  } else if(ps2x.ButtonPressed(PSB_CROSS)){
    moveSquare(24.0);
    stopRobot();
    delay(1000);
  } else if(ps2x.ButtonPressed(PSB_CIRCLE)){
    moveCircle(CLOCKWISE,24.0);
    stopRobot();
    delay(1000);
  } else if(ps2x.ButtonPressed(PSB_L2)){
    pivot(CLOCKWISE,360.0);
    stopRobot();
    delay(1000);
  } else if(ps2x.ButtonPressed(PSB_R3)){
    stopRobot();
    moveFigure8(12.0);
    stopRobot();
  }
}
#endif


/* sgn(value)
 *  This function takes in a value and return the sign of the value
 *  -1, 1, or 0
 */
static inline int8_t sgn(double val) {
  if (val < 0) return -1;
  if (val==0) return 0;
  return 1;
}

/*
 * runSpeedToActualPosition()
 * 
 * This function imitates the behavior of Multistepper's runSpeedToPosition() function,
 * but it has a decceleration at the end of the motion in order to prevent slippage. 
 * 
 * The steppers need to have a speed and position set before this function is run:
 * Use stepper.moveTo() and stepper.setSpeed() before this function
 * 
 * BLOCKING FUNCTION
 */
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

/*This function, runToStop(), will run the robot until the target is achieved and
   then stop it

   BLOCKING FUNCTION
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
