/////////////////////////////////////////////////////////////////////////////
// Group 11
// EENG 350
// Spring 2022
// Dr. Sager
// Demo 1
// 
/////////////////////////////////////////////////////////////////////////////
#include <Encoder.h>

// name pins for motor to run 
const int D2Left = 4;
const int D2Right = 5;
const int motDirLeft = 7;
const int motDirRight = 8;
const int motorVLeft = 9;
const int motorVRight = 10;
const float diameter = 5.75;  //inches
const float robRad = 5.875;   //inches
int commandLeft = 0;
int commandRight = 0;
int commandLeftPrev = 0;
int commandRightPrev = 0;
bool reset = 0;
int vmax = 150;

// encoder declarations
int aPinLeft = 3;   //aPin will use interrupts, but bPin will not. 
int bPinLeft = 11;
int aPinRight = 2;   //aPin will use interrupts, but bPin will not. 
int bPinRight = 6;
//float oldPosLeft = 0; //wheel starts at an angular position of 0 radians
//float oldPosRight = 0;
float currPosLeft = 0;
float currPosRight = 0;
//int currPosIntLeft = 0;
//int currPosIntRight = 0;
//float radPosLeft = 0.0;
//float radPosRight = 0.0;
bool resetLeft = 0;
bool resetRight = 0;
float tau = 10; //ms?
int t1 = 0;
int t2 = 0;

Encoder myEncLeft(aPinLeft, bPinLeft);
Encoder myEncRight(aPinRight, bPinRight);

// controller parameters
float Kp1 = 7;
float Ki1 = 0.0;

float Kp2 = 9;
float Ki2 = 0;

// time variables to keep track of period wait times
int period = 10;
unsigned long time_now = 0;

// controller time variables
unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long elapsedTime = 0;

// controller error variables
float errorL = 1;
float errorR = 1;
float cumErrorLeft = 0;
float cumErrorRight = 0;

//float givenDist = 3;
//float setDist = givenDist * 1.015;    //feet
float setDist = 3;
float setAngle = 90;   //degrees
float fudgeFactor = 0.03;
float angleDist = 0;
float setStraight = 0;

float vBar = 0;
float deltaV = 0;

bool isDone = false;

void setup() {
  // declare inputs/outputs
  pinMode(D2Left, OUTPUT);
  pinMode(D2Right, OUTPUT);
  pinMode(motDirLeft, OUTPUT);
  pinMode(motDirRight, OUTPUT);
  pinMode(motorVLeft, OUTPUT);
  pinMode(motorVRight, OUTPUT);
  pinMode(aPinLeft, INPUT_PULLUP);
  pinMode(bPinLeft, INPUT_PULLUP);  
  pinMode(aPinRight, INPUT_PULLUP);
  pinMode(bPinRight, INPUT_PULLUP);
  Serial.begin(9600);
  setAngle = setAngle*(1+fudgeFactor);

}

void loop() {
  
  // motors off
  digitalWrite(D2Left, LOW);
  digitalWrite(D2Right, LOW);

  if(isDone == false){
    // run motors
    digitalWrite(D2Left, HIGH);
    digitalWrite(D2Right, HIGH);

    // turn only
    angleDist = (2*PI*robRad/(12*360)) * setAngle;
    //angleDist = 0;
    setStraight = 0;

    while((errorL > 0.1 || errorL < -0.1) || (errorR > 0.1 || errorR < -0.1)){
    t1 = millis();
    // use PI controllers
    vBarFunc();
    deltaVFunc();

    // update motor commands
    commandLeft = vBar + deltaV;
    commandRight = vBar - deltaV;  

    // adjust for saturation
    motSat();

    // Can only send positive PWM
    commandLeft = abs(commandLeft);
    commandRight = abs(commandRight);

    // write voltage to the motor
    analogWrite(motorVLeft, commandLeft); 
    analogWrite(motorVRight, commandRight);
/*
    Serial.print("Current Pos Left: \t");
    Serial.println(currPosLeft);
    Serial.print("Current Pos Right: \t");
    Serial.println(currPosRight);
    Serial.print("Desired Distance: \t");
    Serial.println(setStraight);
    Serial.print("Desired Angle: \t");
    Serial.println(angleDist);
    Serial.print("Error Distance: \t");
    Serial.println(errorL);
    Serial.print("Error Angle: \t");
    Serial.println(errorR);
    Serial.print("vBar: \t");
    Serial.println(vBar);
    Serial.print("deltaV: \t");
    Serial.println(deltaV);
    Serial.print("Command Left: \t");
    Serial.println(commandLeft);
    Serial.print("Command Right: \t");
    Serial.println(commandRight);
    Serial.print("\n");
*/
    t2 = millis();
    Serial.println("Time:  ");
    Serial.println(t1-t2);
  }
  
  errorL = 1;
  errorR = 1;

  myEncLeft.write(0);
  myEncRight.write(0);
  delay(1000);
  
  // go straight only
  setStraight = setDist; 
  setAngle = 0;
   angleDist = 0; 

  while((errorL > 0.03 || errorL < -0.03) || (errorR > 0.03 || errorR < -0.03)){
    // use PI controllers
    vBarFunc();
    deltaVFunc();

    // update motor commands
    commandLeft = vBar + deltaV;
    commandRight = vBar - deltaV;  

    // adjust for saturation
    motSat();

    // Can only send positive PWM
    commandLeft = abs(commandLeft);
    commandRight = abs(commandRight);

    // write voltage to the motor
    analogWrite(motorVLeft, commandLeft); 
    analogWrite(motorVRight, commandRight); 
/*
    Serial.print("Current Pos Left: \t");
    Serial.println(currPosLeft);
    Serial.print("Current Pos Right: \t");
    Serial.println(currPosRight);
    Serial.print("Desired Distance: \t");
    Serial.println(setStraight);
    Serial.print("Desired Angle: \t");
    Serial.println(angleDist);
    Serial.print("Error Distance: \t");
    Serial.println(errorL);
    Serial.print("Error Angle: \t");
    Serial.println(errorR);
    Serial.print("vBar: \t");
    Serial.println(vBar);
    Serial.print("deltaV: \t");
    Serial.println(deltaV);
    */
    Serial.print("Command Left: \t");
    Serial.println(commandLeft);
    Serial.print("Command Right: \t");
    Serial.println(commandRight);
    Serial.print("\n");
    }
    
  }

  // stop moving mom when both tasks are complete
  isDone = true; 

  // stop sending PWM to the motors
  analogWrite(motorVLeft, 0); 
  analogWrite(motorVRight, 0); 
  
}
void vBarFunc() {
  // controller
  // time variables
  currentTime = millis();
  time_now = currentTime;
  elapsedTime = currentTime - previousTime;

  // check encoder position FEET
  currPosLeft = (-1 * myEncLeft.read() * (PI * diameter)) / (3200);
  currPosLeft = currPosLeft / 12; // overflow

  // check encoder position FEET
  currPosRight = (myEncRight.read() * (PI * diameter)) / (3200);
  currPosRight = currPosRight / 12; // overflow

  // calculate error from current position to desired position
  errorL = setStraight - ((currPosLeft + currPosRight) / 2); 
  cumErrorLeft+= (errorL * elapsedTime) / 1000;

  // for PI (V)
  vBar = ((Kp1 * errorL) + (Ki1 * cumErrorLeft)) * (255/8);

  // adjust previous time variable
  previousTime = currentTime;
}

void deltaVFunc() {
  // controller
  // time variables
  currentTime = millis();
  time_now = currentTime;
  elapsedTime = currentTime - previousTime;

  // check encoder position FEET
  currPosLeft = (-1 * myEncLeft.read() * (PI * diameter)) / (3200);
  currPosLeft = currPosLeft / 12; // overflow

  // check encoder position FEET
  currPosRight = (myEncRight.read() * (PI * diameter)) / (3200);
  currPosRight = currPosRight / 12; // overflow

  // calculate error from current position to desired position
  errorR = angleDist - ((currPosLeft - currPosRight) / 2);
  cumErrorRight += (errorR * elapsedTime) / 1000;

  // for PI (V)
  deltaV = ((Kp2 * errorR) + (Ki2 * cumErrorRight)) * (255/8); 

  // adjust previous time variable
  previousTime = currentTime;
}

void motSat(){
  //commandLeft = (commandLeft-commandLeftPrev)*(1-exp(-elapsedTime/tau));
  //commandRight = (commandRight-commandRightPrev)*(1-exp(-elapsedTime/tau));
  if(commandLeft > vmax){
    commandLeft = vmax;
  }
  else if(commandLeft < -vmax){
    commandLeft = -vmax;
  }
  if(commandRight > vmax){
    commandRight = vmax;
  }
  else if(commandRight < -vmax){
    commandRight = -vmax;
  }

  // adjust voltage pin signs (direction) to turn angle
  if(commandLeft <= 0){ 
    // set motor to negative direction
    digitalWrite(motDirLeft, HIGH);
  }
  else{
    // set motor to positive direction
    digitalWrite(motDirLeft, LOW);
  }

  // adjust voltage pin signs (direction) to turn angle
  if(commandRight >= 0){ 
    // set motor to negative direction
    digitalWrite(motDirRight, LOW);
  }
  else{
    // set motor to positive direction
    digitalWrite(motDirRight, HIGH);
  }
  commandLeftPrev = commandLeft;
  commandRightPrev = commandRight;
}
