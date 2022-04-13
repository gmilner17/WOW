/////////////////////////////////////////////////////////////////////////////
// Group 11
// EENG 350
// Spring 2022
// Dr. Sager
// Demo 1
//
/////////////////////////////////////////////////////////////////////////////
#include <Encoder.h>
#include <Wire.h>

#define SLAVE_ADDRESS 0x05

int counter1 = 0;
int counter2 = 0;

float currr = 0;

bool demo1 = false;

bool inRange = false;
bool stopNow = false;
bool outOfAngle = false;
int readIn = 0;

bool straightDone = false;

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
int vmax = 45;

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
//vBar control
float Kp1 = 8; // was 7
float Ki1 = 0.0;

//deltaV control
float Kp2 = 5; // was 9
float Ki2 = 0.1;

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
float setDist = 0;
float setAngle = 0;   //degrees
float fudgeFactor = 0.03;
float angleDist = 0;
float setStraight = 0;

float vBar = 0;
float deltaV = 0;

bool isDone = false;

float dumVar = 0;

void setup() {
  // communication with PI
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

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
  setAngle = setAngle * (1 + fudgeFactor);

  // motors off
  digitalWrite(D2Left, LOW);
  digitalWrite(D2Right, LOW);

  // stop sending PWM to the motors
  analogWrite(motorVLeft, 0);
  analogWrite(motorVRight, 0);

}

void loop() {

  /*
    while (readIn == 0) {
    Serial.println(readIn);
    }*/

  if (readIn != 0) {

    if (isDone == false) {

      // run motors
      digitalWrite(D2Left, HIGH);
      digitalWrite(D2Right, HIGH);

      // turn only
      angleDist = (2 * PI * robRad / (12 * 360)) * dumVar; //from Pi
      //angleDist = 0;
      setStraight = 0;

      while ((errorL > 0.1 || errorL < -0.1) || (errorR > 0.1 || errorR < -0.1)) {
        //currr = ((abs(currPosLeft) + abs(currPosRight)) / 2) * (360 / (PI * diameter));
        //Serial.print("curr pos: \t");
        //Serial.println(currr);
        Serial.print("dumVar: \t");
        Serial.println(dumVar);
        /*
          if((errorL < 0.5 || errorL > -0.5) || (errorR < 0.5 || errorR > -0.5)){
          float Ki1 = 0.5;
          float Ki2 = 0.5;
          }
          Serial.print("Error left = \t");
          Serial.println(errorL);
          Serial.print("Error right = \t");
          Serial.println(errorR);
   */
        angleDist = (2 * PI * robRad / (12 * 360)) * (dumVar); //from Pi

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

        t2 = millis();
        // wait 5 ms
        while (millis() < period + (t2 - t1)) {
          //wait approx. [period] ms
        }
      }

      outOfAngle = true;

      errorL = 1;
      errorR = 1;

      myEncLeft.write(0);
      myEncRight.write(0);
      delay(2000);

      // go straight only
      // EDIT HERE////////////////////////////////////////////////////////////////////
      //readIn = 300;
      setStraight = 9;
      setAngle = 0;
      angleDist = 0;
      vmax = 70;
      
      while (straightDone == false) {
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
        commandRight = abs(1.05*commandRight);  //CHANGE THIS VALUE

        // write voltage to the motor
        analogWrite(motorVLeft, commandLeft);
        analogWrite(motorVRight, commandRight);

        t2 = millis();

        // wait 5 ms
        while (millis() < period + (t2 - t1)) {
          //wait approx. [period] ms
        }
      }

      if (demo1 == false) {
        errorL = 1;
        errorR = 1;

        myEncLeft.write(0);
        myEncRight.write(0);
        //delay(2000);

        // go straight only
        // EDIT HERE////////////////////////////////////////////////////////////////////
        setStraight = 0;
        setAngle = 0;
        angleDist = 0;
        vmax = 100;
        //Kp1 = 6;
        //Kp2 = 6;

        //Ki1 = 0;
        //Ki2 = 0;

        while ((errorL > 0.05 || errorL < -0.05) || (errorR > 0.05 || errorR < -0.05)) {
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

          t2 = millis();
          // wait 5 ms
          while (millis() < period + (t2 - t1)) {
            //wait approx. [period] ms
          }
        }
        // stop moving mom when both tasks are complete
        isDone = true;
      }
    }
    else {
      // motors off
      digitalWrite(D2Left, LOW);
      digitalWrite(D2Right, LOW);
      //Serial.println(isDone);

      // stop sending PWM to the motors
      analogWrite(motorVLeft, 0);
      analogWrite(motorVRight, 0);
    }
  }


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
  cumErrorLeft += (errorL * elapsedTime) / 1000;

  // for PI (V)
  vBar = ((Kp1 * errorL) + (Ki1 * cumErrorLeft)) * (255 / 8);

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
  deltaV = ((Kp2 * errorR) + (Ki2 * cumErrorRight)) * (255 / 8);

  // adjust previous time variable
  previousTime = currentTime;
}

void motSat() {
  //commandLeft = (commandLeft-commandLeftPrev)*(1-exp(-elapsedTime/tau));
  //commandRight = (commandRight-commandRightPrev)*(1-exp(-elapsedTime/tau));
  if (commandLeft > vmax) {
    commandLeft = vmax;
  }
  else if (commandLeft < -vmax) {
    commandLeft = -vmax;
  }
  if (commandRight > vmax) {
    commandRight = vmax;
  }
  else if (commandRight < -vmax) {
    commandRight = -vmax;
  }

  // adjust voltage pin signs (direction) to turn angle
  if (commandLeft <= 0) {
    // set motor to negative direction
    digitalWrite(motDirLeft, HIGH);
  }
  else {
    // set motor to positive direction
    digitalWrite(motDirLeft, LOW);
  }

  // adjust voltage pin signs (direction) to turn angle
  if (commandRight >= 0) {
    // set motor to negative direction
    digitalWrite(motDirRight, LOW);
  }
  else {
    // set motor to positive direction
    digitalWrite(motDirRight, HIGH);
  }
  commandLeftPrev = commandLeft;
  commandRightPrev = commandRight;
}

//callback for recieved data
void receiveData(int byteCount) {
  while (Wire.available()) {
    readIn = Wire.read();
    if (outOfAngle == false) {

      if (readIn == 100) {
        dumVar = 360; //284
        Serial.print("Got value of 100, dumVar: \t");
        Serial.println(dumVar);
      }
      if (((readIn < 27) && (readIn > -27))) {
        //counter1 = counter1 + 1;
        //if (counter1 == ) {
        //dumVar = readIn + ((abs(currPosLeft) + abs(currPosRight)) / 2) * ((360 / (PI * diameter))) + 0;
        myEncLeft.write(0);
        myEncRight.write(0);
        dumVar = (readIn);
        
        Serial.print("Marker found, dumVar: \t");
        Serial.println(dumVar);
        Serial.print("From Rich: \t");
        Serial.println(readIn);
        Serial.print("curr pos: \t");
        Serial.println(currr);
        //readIn = 300;

        //}
        //readIn = 300;

      }
    }
    else {
      //counter2 = counter2 + 1;
      if (readIn == 100) {
        straightDone = true;
      }
    }
  }
}

void sendData() {
  Wire.write(outOfAngle);
}
