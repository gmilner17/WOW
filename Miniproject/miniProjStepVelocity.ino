//////////////////////////////////////////////////////////////////////////////
// Group 11
// EENG 350
// Spring 2022
// Dr. Sager
// Mini Project 4.6
// This code runs a step response to the motor and outputs a constant velocity
//////////////////////////////////////////////////////////////////////////////
#include <Encoder.h>
//#include <KeyboardController.h>

// name pins
const int encoderPin = 12; 
const int D2 = 4;
const int motSign1 = 7;
const int motSign2 = 8;
const int motorV1 = 9;
const int motorV2 = 10;
// V
int command = 0;

// variable for angular velocity
// rad/s
float angVel = 0;

// for millis command
// ms
int period = 10;
unsigned long time_now = 0;

// for keeping track of time for voltage
unsigned long timeVolt = 0;

// encoder declarations
int aPin = 2;   //aPin will use interrupts, but bPin will not. 
int bPin = 12;
float oldPos = 0; //wheel starts at an angular position of 0 radians
// counts
float currPos = 0;
// rad
float radPos = 0.0;
bool reset = 0;

Encoder myEnc(aPin, bPin);

void setup() {  
  // declare inputs/outputs
  pinMode(encoderPin, INPUT);
  pinMode(D2, OUTPUT);
  pinMode(motSign1, OUTPUT);
  pinMode(motSign2, OUTPUT);
  pinMode(motorV1, OUTPUT);
  pinMode(motorV2, OUTPUT);
  pinMode(aPin, INPUT_PULLUP);
  pinMode(bPin, INPUT_PULLUP);
  Serial.begin(9600);
  oldPos = 0;
  currPos = 0;
}

void loop() {
  // set motor to positive direction
  digitalWrite(motSign1, HIGH);

  // set motor to positive direction
  digitalWrite(D2, HIGH);
  
  // get time 
  time_now = millis();
  timeVolt = time_now;
  
  // write voltage to the motor
  analogWrite(motorV1, command);

  // check encoder position
  if (!reset) {
    currPos = myEnc.read();
    if (currPos != oldPos) {
      radPos += (currPos - oldPos) * ((2*PI) / 3200);   //change in counts
      
      angVel = ((currPos - oldPos) * ((2*PI) / 3200)) * (1000 / period);
    }
    
    oldPos = currPos;
  }
  else {
      currPos = 0;
      radPos = 0;
      reset = 0;
  }

  if((millis() - time_now) > period){
    Serial.print("Error \n");
  }


  // check for reset
  char inchar = (char)Serial.read();
  if (inchar == 'r')  reset = 1;
  
  // wait 5 ms
  while(millis() < time_now + period){
        //wait approx. [period] ms
  }

  if(timeVolt >= 1000){
    // generate pwm wave at 50% duty cycle
    command = 123;
  } 
  
  //print current time
  //Serial.print("current time: \n"); 
  Serial.print(time_now); 
  Serial.print("\t");

  //print motor voltage command
  //Serial.print("motor voltage command: \n"); 
  Serial.print(command);
  Serial.print("\t"); 
  
  // print angular velocity
  //Serial.print("Angular Velocity: \n");
  Serial.print(angVel); 
  Serial.print("\n");  
  
}
void keyPressed() {
  char inchar = (char)Serial.read();
  if (inchar == 'r')  reset = 1;
}
