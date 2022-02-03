/************************************
  Braitenberg-FinalProject.ino
  Jordan Asman and Cory Snyder 1.17.2022

  This program introduces the 4 Braitenberg Vehicle behaviors using 2 photoresistors attached to analog inputs. 
  The love behavior is then integrated with the random wander and obstacle avoidance behaviors through a simple state machine.
  Finally, a final state machine implements the behavior where the robot will follow a wall until it sees a light where it will home to the light and then dock.
  After that, the robot will return to the wall in the position it was before the homing, and continue to wall following.

  The primary functions created are:
  calibratePhotoresistors - This function is run at the end of setup.  It spins the robot in a circle and records the ambient lighting conditions
  love, explorer, agression, fear - These 4 functions set the speed of the left and right steppers based on the photoresistors to implement the corresponding Braitenberg behavior
  loveStateMachine - This function implements the love Braitenberg behavior with obstacle avoidance and random wander
  homing - This function runs the love behavior to move towards the light.  It also keeps track of its left and right wheel speeds each iteration and saves it to arrays for playback
  returning - This function takes the speeds recorded in the homing function and feeds them back to the motors in reverse order.  This causes the robot to backtrack on its homing path.
  HomingStateMachine - This function combines the homing and returning behaviors with the PD wall following from Lab03
  
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

  analog pin A15 - back IR sensor
  analog pin A14 - left IR sensor
  analog pin A13 - front IR sensor
  analog pin A12 - right IR sensor

  analog pin A0 - left photoresistor
  analog pin A1 - right photoresistor

  digital pin 19 (has Interrupts) - right Sonar sensor 
  digital pin 18 (has Interrupts) - left Sonar sensor
*/

#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include <NewPing.h>
#include <RunningMedian.h>
//#include <SoftwareSerial.h>

#define enableLED 13    //stepper enabled LED
#define redLED 5        //red LED for displaying states
#define grnLED 6        //green LED for displaying states
#define ylwLED 7        //yellow LED for displaying states

#define pauseTime 1000 //time before robot moves
#define wait_time 1000 //time to wait between prints and starting robot

//Bluetooth --------------------------------------------------------------------------------------------------------------------
#define BT_TX_PIN 16
#define BT_RX_PIN 17
//SoftwareSerial blueSerial(BT_RX_PIN, BT_TX_PIN); // RX, TX

//Motor Defines and Variables --------------------------------------------------------------------------------------------------
#define R_STEP_PIN  50 //right stepper motor step pin
#define R_DIR_PIN   51 //right stepper motor direction pin
#define L_STEP_PIN  52 //left stepper motor step pin
#define L_DIR_PIN   53 //left stepper motor direction pin
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
#define fwdSpeed 300//200
#define revSpeed 500
#define spinSpeed 200
#define turnSpeed 400
#define readySpeed 100            // speed for slightly adjusting wheels to calibrate encoders
#define collideSpeed 500
#define avoidSpeed 80
#define calibrateSpeed 500
#define CLOCKWISE true
#define COUNTERCLOCKWISE false

#define COLLIDE_DIST 4.0

#define LEFT  0                     //constant for left wheel
#define RIGHT 1                     //constant for right wheel

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

void leftSonarInt(){
  switch (digitalRead(LEFT_SONAR)){                   // Test to see if the signal is high or low
    case HIGH:                                        // High so must be the start of the echo pulse
      leftEchoEnd = 0;                                // Clear the end time
      leftEchoStart = micros();                       // Save the start time
      break;
    case LOW:                                         // Low so must be the end of hte echo pulse
      leftEchoEnd = micros();                         // Save the end time
      long temp = leftEchoEnd - leftEchoStart;
      if(temp < 10000){
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
      if(temp < 10000){
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

// START BLUETOOTH SERIAL
  Serial2.begin(9600);

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

  stepperRight.setMaxSpeed(maximumSpeed);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setMaxSpeed(maximumSpeed);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(maxAccel);//set desired acceleration in steps/s^2
  stepperLeft.setAcceleration(maxAccel);//set desired acceleration in steps/s^2
  steppers.addStepper(stepperRight);//add right motor to MultiStepper
  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
  digitalWrite(STEP_EN_PIN, STEP_ENABLE);//turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);//turn on enable LED
  delay(pauseTime); //always wait 5 seconds before the robot moves
  Serial.println("Enter Commands: ");
}

// Wall Follow Consts and Defines ----------------------------------------------------------------------------------------------

double leftWheelSpeed = 250.0;
double rightWheelSpeed = 250.0;

int wallState = 0;

#define FOLLOW_DISTANCE_LOW 4
#define FOLLOW_DISTANCE_HIGH 6
#define WALL_DETECT_DIST 10
#define IR_WALL_DETECT_DIST 4
#define SPEED_DIFF 50
#define ROBOT_WIDTH 10
#define ROBOT_LENGTH 12
#define SETPOINT 5
#define MAX_CONTROL_EFFORT 150

#define RANDOM_WANDER 0
#define LEFT_WALL 1
#define RIGHT_WALL 2
#define BOTH_WALLS 3
#define INSIDE_CORNER 4
#define OUTSIDE_CORNER_LEFT 5
#define OUTSIDE_CORNER_RIGHT 6
#define HALLWAY_END 7
#define AVOID 8
#define HOMING 9
#define RETURNING 10

const double kp = 50;                  // The proportional control gain for single wall following
const double kp_center = 50;           // The proportional control gain for center following
const double kd = 200;                 // The derivative control gain for single wall following
const double kd_center = 200;          // The derivative control gain for center following

// Loop ------------------------------------------------------------------------------------------------------------------------
unsigned long last_read = 0;
void loop()
{
//  //Every 100 milliseconds, print the photoresistor values
//  int temp = millis() - last_read;
//  if(temp > 500){
//    double leftPhoto = getPhotoresistorVoltage(LEFT_PHOTO);
//    double rightPhoto = getPhotoresistorVoltage(RIGHT_PHOTO);
//    Serial.print("Right: ");Serial.println(rightPhoto);
//    Serial.print("Left: ");Serial.println(leftPhoto);
//    Serial.println("-----------------------------");
//    last_read = millis();
//  }
  
//  // Keep reading from HC-05 and send to Arduino Serial Monitor
//  if (Serial2.available()){
//    Serial.println(Serial2.read());
//  }
//    
//
//  // Keep reading from Arduino Serial Monitor and send to HC-05
//  if (Serial.available()){
//    Serial2.write(Serial.read());
//  }
    switch(waitForCommand()){
      case 'T': //Topological Path Following: Follow a string of topological commands 
        digitalWrite(redLED,HIGH);
        topologicalPathFollowing();
        digitalWrite(redLED,LOW);
        break;
      case 'P': //Metric Path Planning: Follow a string of metric commands
        break;
      case 'M': //Mapping: Map out the environment using the sensors
        break;
      case 'L': //Localization: Localize in an environment that we already know
        break;
    }
    
    
//  //Every 50 milliseconds, run the state machine
//  int temp = millis() - last_read;
//  if(temp > PHOTO_SAMPLE_MS){
////    loveStateMachine();
//    HomingStateMachine();
//    last_read = millis();
//  }
//  
//  //Run the stepper motors at the speed they were set
//  stepperRight.runSpeed();
//  stepperLeft.runSpeed();

}

/* waitForCommand()
 * 
 * This function waits for Matlab to send a command to the robot
 * and then returns it
 * 
 * Commands are 2 bytes long and always start with decimal 126 or '~'
 */
int waitForCommand() {
  char commandMsg[2] = {' ',' '};
  while(commandMsg[0]!='~'){
    if(Serial2.available()){
      commandMsg[0] = Serial2.read();
    }
  }
  while(!Serial2.available()){}
  commandMsg[1] = Serial2.read();
  return commandMsg[1];
}

void waitForData(char dataArray[]){
  char lastChar = ' ';
  int idx = 0;
  while(lastChar != '-'){
    if(Serial2.available()){
      lastChar = Serial2.read();
      dataArray[idx] = lastChar;
      idx++;
    }
  }
}

void topologicalPathFollowing(){
  char data[8];
  waitForData(data);
  digitalWrite(grnLED,HIGH);
  Serial2.write("Got Path");
  Serial.println(data);
  int idx = 0;
  while(data[idx] != '-'){
    Serial.println(int(data[idx]));
    if(data[idx] == 'L'){
      double leftDist = getLinearizedDistance(LEFT_SONAR);
      while(!lostWall(leftDist)){
        followLeftWallPD(leftDist);
        stepperLeft.runSpeed();
        stepperRight.runSpeed();
        leftDist = getLinearizedDistance(LEFT_SONAR);
      }
      stopRobot();
      forward(5.0);
      stopRobot();
      pivot(COUNTERCLOCKWISE,90.0);
      stopRobot();
      forward(7.0);
      stopRobot();
    } else if(data[idx]=='R'){
      double rightDist = getLinearizedDistance(RIGHT_SONAR);
      while(!lostWall(rightDist)){
        followRightWallPD(rightDist);
        stepperLeft.runSpeed();
        stepperRight.runSpeed();
        rightDist = getLinearizedDistance(RIGHT_SONAR);
      }
      stopRobot();
      forward(5.0);
      stopRobot();
      pivot(CLOCKWISE,90.0);
      stopRobot();
      forward(7.0);
      stopRobot();
    } else {
      digitalWrite(ylwLED,HIGH);
    }
    
    idx++;
  }
  digitalWrite(grnLED,LOW);
}

int lostCount = 0;
bool lostWall(double dist) {
  if(dist>15) lostCount++;
  if(lostCount>15){
    lostCount = 0;
    return true;
  }
  return false;
}

double pastErrorLeft = 0.0;
/* followLeftWallPD(double dist)
 *  This function uses the distance from the left sonar find the error from the 
 *  setpoint. The speeds of the right and left wheels are then set based on the
 *  calculated error, proportional gain, and derivative gain to exhibit PD 
 *  control when a left wall is present
 */
void followLeftWallPD(double dist) {
  double error = SETPOINT - dist; 
  double errorDiff = error - pastErrorLeft;
  double controlEffort = error*kp + errorDiff*kd;
  controlEffort = max(controlEffort, -MAX_CONTROL_EFFORT);
  controlEffort = min(controlEffort, MAX_CONTROL_EFFORT);
  setSpeeds(leftWheelSpeed + controlEffort, rightWheelSpeed - controlEffort);
  pastErrorLeft = error;
}

double pastErrorRight = 0.0;
/* followRightWallPD(double dist)
 *  This function uses the distance from the right sonar find the error from the 
 *  setpoint. The speeds of the right and left wheels are then set based on the
 *  calculated error, proportional gain, and derivative gain to exhibit PD 
 *  control when a right wall is present
 */
void followRightWallPD(double dist) {
  double error = SETPOINT - dist; 
  double errorDiff = error - pastErrorRight;
  double controlEffort = error*kp + errorDiff*kd;
  controlEffort = max(controlEffort, -MAX_CONTROL_EFFORT);
  controlEffort = min(controlEffort, MAX_CONTROL_EFFORT);
  setSpeeds(leftWheelSpeed - controlEffort, rightWheelSpeed + controlEffort);
  pastErrorRight = error;
}

double pastErrorCenter = 0.0;
/* followCenterPD(double dist)
 *  This function uses the differance between the right and left sonar distances 
 *  to find the error. The speeds of the right and left wheels are then set based 
 *  on the calculated error, proportional gain, and derivative gain to exhibit PD 
 *  control when the right and left walls are present
 */
void followCenterPD(double leftDist, double rightDist) {
  double error = leftDist - rightDist;
  double errorDiff = error - pastErrorCenter;
  double controlEffort = error*kp_center + errorDiff*kd_center;
  controlEffort = max(controlEffort, -MAX_CONTROL_EFFORT);
  controlEffort = min(controlEffort, MAX_CONTROL_EFFORT);
  setSpeeds(leftWheelSpeed - controlEffort, rightWheelSpeed + controlEffort);
  pastErrorCenter = error;
}

/*
 * insideCorner(leftSonarDist, rightSonarDist, frontIRDist)
 * 
 * Using the left, right, and front distances from the sensors, this function chooses to execute a
 * left or right inside corner.  It first reverses the robot to make room for a 90 degree pivot in
 * the left or right direction.  It chooses to turn left or right based on which wall (left or right)
 * is clsoer
 */
void insideCorner(double leftSonarDist, double rightSonarDist, double frontIRDist) {
  if (frontIRDist > IR_WALL_DETECT_DIST - 1) {
    reverse(ROBOT_WIDTH - frontIRDist);
  }
  if (leftSonarDist > rightSonarDist) {
    pivot(COUNTERCLOCKWISE, 90);
  }
  else {
    pivot(CLOCKWISE, 90);
  }
}

/*
 * outsideCornerLeft()
 * 
 * This function maneuvers the robot to turn around an outside left corner.
 */
void outsideCornerLeft() {
  forward(ROBOT_LENGTH / 4);
  pivot(COUNTERCLOCKWISE, 90);
  forward(ROBOT_LENGTH / 2);
}

/*
 * outsideCornerRight()
 * 
 * This function maneuvers the robot to turn around an outside right corner.
 */
void outsideCornerRight() {
  forward(ROBOT_LENGTH / 4);
  pivot(CLOCKWISE, 90);
  forward(ROBOT_LENGTH / 2);
}

/*
 * hallwayEnd(frontIRDist)
 * 
 * This function backs the robot up and spins 180 degrees to turn around at the end of a hallway.
 * The frontIRDist is used to calculate how much the robot needs to reverese before executing the
 * spin
 * 
 * BLOCKING FUNCTION
 */
void hallwayEnd(double frontIRDist) {
  reverse(ROBOT_LENGTH - frontIRDist);
  spin(CLOCKWISE, 180);
}

/* randomWander()
 *  This function sets the speeds of the left and the right steppers to a random speed
 *  every 1 second
 */
double randLSpeed = 0.0;
double randRSpeed = 0.0;
long lastRandom = 0;

void randomWander() {
  if(millis() - lastRandom > 1000){
      randLSpeed = random(800);
      randRSpeed = random(800);
      
      lastRandom = millis();
  }
  setSpeeds(randLSpeed,randRSpeed);
}

/* detectedObstacle()
 *  This function returns true if there is an object detected by the 4 IR sensors within the COLLIDE_DIST
 */
boolean detectedObstacle(){
  double frontDist = getLinearizedDistance(FRONT_IR);
  double backDist = getLinearizedDistance(BACK_IR);
  double rightDist = getLinearizedDistance(RIGHT_IR);
  double leftDist = getLinearizedDistance(LEFT_IR);

  return frontDist < COLLIDE_DIST || backDist < COLLIDE_DIST || leftDist < COLLIDE_DIST || rightDist < COLLIDE_DIST;
}

/* runAway()
 *  This function uses the 4 IR sensors to move away from any obstacle that is detected
 */
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
    long timer1 = millis();
    while(millis()-timer1<100){
      stepperLeft.runSpeed();
      stepperRight.runSpeed();
    }
  }
  long timer2 = millis();
  while(millis()-timer2<4000){
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
      return (double(rightSonarData.getMedian())/soundInchRoundTrip)-0.7;
      break;
    default:
      break;
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
  
  if(clockwise){
    stepperLeft.moveTo(steps+stepperLeft.currentPosition());
    setSpeeds(sgn(steps)*spinSpeed,0);
  } else {
    stepperRight.moveTo(steps+stepperRight.currentPosition());
    setSpeeds(0,sgn(steps)*spinSpeed);
  }

//  runSpeedToActualPosition(); // Blocks until all are in position
  steppers.runSpeedToPosition(); // Blocks until all are in position
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
  
  if(clockwise){
    stepperLeft.moveTo(steps+stepperLeft.currentPosition());
    stepperRight.moveTo(-steps+stepperRight.currentPosition());
    setSpeeds(spinSpeed,-spinSpeed);
  } else {
    stepperLeft.moveTo(-steps+stepperLeft.currentPosition());
    stepperRight.moveTo(steps+stepperRight.currentPosition());
    setSpeeds(-spinSpeed,spinSpeed);
  }
//  runSpeedToActualPosition(); // Blocks until all are in position
  steppers.runSpeedToPosition(); // Blocks until all are in position
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
  stepperLeft.moveTo(steps+stepperLeft.currentPosition());
  stepperRight.moveTo(steps+stepperRight.currentPosition());
  
  if(distance < 0){
    stopRobot();
  } else {
    setSpeeds(fwdSpeed,fwdSpeed);
  }

//  runSpeedToActualPosition(); // Blocks until all are in position
  steppers.runSpeedToPosition(); // Blocks until all are in position
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
  stepperLeft.moveTo(-steps+stepperLeft.currentPosition());
  stepperRight.moveTo(-steps+stepperRight.currentPosition());
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
