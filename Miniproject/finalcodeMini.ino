/////////////////////////////////////////////////////////////////////////////
// Group 11
// EENG 350
// Spring 2022
// Dr. Sager
// Mini Project
// This code runs the PI controller for the overall system.
/////////////////////////////////////////////////////////////////////////////
#include <Encoder.h>

#include <Wire.h>

#define SLAVE_ADDRESS 0x05

// name pins for motor to run
const int encoderPin = 12; 
const int D2 = 4;
const int motSign1 = 7;
const int motorV1 = 9;
int command = 0;  // V

// controller parameters
float Kp = 4.5;
float Ki = 0.05;

// time variables to keep track of period wait times
int period = 10;  // ms
unsigned long time_now = 0;

// controller time variables
unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long elapsedTime = 0;

// controller error variables
float error = 1;
float cumError = 0;

int numCam = 0;
float desPos[] = {PI/2, PI, (3*PI)/2, 2*PI};
// rad
float setPoint = 0;

// encoder declarations
int aPin = 2;   //aPin will use interrupts, but bPin will not. 
int bPin = 12;
float oldPos = 0; //wheel starts at an angular position of 0 radians
float currPos = 0;
float radPos = 0.0;
bool reset = 0;

Encoder myEnc(aPin, bPin);

void setup() {
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  
  // declare inputs/outputs
  pinMode(encoderPin, INPUT);
  
  pinMode(D2, OUTPUT);
  pinMode(motSign1, OUTPUT);
  pinMode(motorV1, OUTPUT);
  pinMode(aPin, INPUT_PULLUP);
  pinMode(bPin, INPUT_PULLUP);
  Serial.begin(9600);
}

void loop() {
  // run motor
  digitalWrite(D2, HIGH);

  // read in value from the Pi
  if(numCam != -1){
    //Serial.println(numCam);
    setPoint = desPos[numCam - 1];
    //Serial.println(setPoint);
  }

  // controller
  
  // time variables
  currentTime = millis();
  time_now = currentTime;
  elapsedTime = currentTime - previousTime;

  // check encoder position
  // rad
  currPos = myEnc.read() * ((2*PI) / 3200);
    
  // calculate error from current position to desired position
  error = setPoint - currPos;
  cumError += (error * elapsedTime) / 1000;

  // for PI V
  command = ((Kp * error) + (Ki * cumError)) * (255/8);

  if(command > 255){
    command = 255;
  }
  else if(command < -255){
    command = -255;
  }
  if(command >= 0){
    // set motor to positive direction
    digitalWrite(motSign1, HIGH);
  }
  else{
    // set motor to negative direction
    digitalWrite(motSign1, LOW);
  }

  command = abs(command);
  
  // write voltage to the motor
  analogWrite(motorV1, command); 

  // adjust previous time variable
  previousTime = currentTime;  
  
  //print current time
  Serial.print(currentTime); 
  Serial.print("\t");

  //print current position
  Serial.println(currPos); 

  // wait period
  while(millis() < time_now + period){ 
    //wait approx. [period] ms
  }
}
// function to reset postion
void keyPressed() {
  char inchar = (char)Serial.read();
  if (inchar == 'r')  reset = 1;
}
//callback for recieved data
void receiveData(int byteCount){
  while(Wire.available()){
    numCam = Wire.read();
    }
}
