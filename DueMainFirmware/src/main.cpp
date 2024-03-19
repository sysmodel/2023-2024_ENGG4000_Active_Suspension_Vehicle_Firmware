/* 
* Sys-MoDEL Active Suspension Main.ino file
*
* Authors: Gregory Stewart, John Estafanos, Andrew Kennah, Patrick Laforest
* Creation Date: Jan 6, 2024
* Last Update: March 6, 2024
*
* Version 1.0
*
* Description: 
* This code is the Main.ino file that is used to retrieve data from the array of sensors. 
* Each retrieval of data is done by executing an internal ISR on the Arduino DUE. 
*
* Functions & Descriptions: 
* Name: 
* Description: 
*
* References:
*
*/

#include <Arduino.h>
#include "DueTimer.h"
#include "QuadEncoder.h"
#include "AbsEncoders.h"
#include "Waveforms.h"
#include "FFIControl.h"

//------------------------------------------------------------------

#define led LED_BUILTIN
#define steerPotPin A4
#define battCell1Pin A3
#define battCell2Pin A2

//------------------------------------------------------------------

// Variable definitions
float current[4] = {0,0,0,0}; // array of measured current values
int i = 0; // used for indexing
float voltArray[2] = {0,0};
float battVoltage = 0;
long timeCount = millis();
int sineCount = 0;
int funcTime = 0;
int currentFlag = 0;

// // Motor driver
int pwm[4] = {0,0,0,0};

// Jetson communication
float setI[4] = {0,0,0,0}; // array of stored current setpoints

// Encoders
uint8_t quadEncoderFlag = 0;
uint8_t absEncoderFlag = 0;
uint8_t resolution = 12;
// - Define global variable to store abs encoder positions and speeds
uint16_t absEncCurrentPositionFR; // Front Right
uint16_t absEncCurrentPositionFL; // Front Left
uint16_t absEncCurrentPositionBR; // Back Right
uint16_t absEncCurrentPositionBL; // Back Left
double absEncCurrentVelocityFR; // Front Right
double absEncCurrentVelocityFL; // Front Left
double absEncCurrentVelocityBR; // Back Right
double absEncCurrentVelocityBL; // Back Left
// - Define pins for each encoder; structure of arrays: {FR, FL, BR, BL}; indices: {0,1,2,3}
uint8_t sdoPin[4] = {11, 13, 7, 9};
uint8_t sckPin[4] = {10, 12, 6, 8};
uint8_t csPin[4] = {25, 24, 27, 26};

//------------------------------------------------------------------

// Creation of absolute encoder objects; structure of arrays: {FR, FL, BR, BL}; indices: {0,1,2,3}
AbsEnc absEncoderFR(sckPin[0], csPin[0], sdoPin[0], resolution);
AbsEnc absEncoderFL(sckPin[1], csPin[1], sdoPin[1], resolution);
AbsEnc absEncoderBR(sckPin[2], csPin[2], sdoPin[2], resolution);
AbsEnc absEncoderBL(sckPin[3], csPin[3], sdoPin[3], resolution);

//------------------------------------------------------------------

void GetVoltage() {
  voltArray[0] = float(map(analogRead(battCell1Pin), 0, 1023, 0, 3300)) * 0.005;
  battVoltage = float(map(analogRead(battCell2Pin), 0, 1023, 0, 3300)) * 0.005;
  voltArray[1] = battVoltage - voltArray[0];
}

void GetQuadEncoderData() {
  // Get the vehicle speed in rad/s
  quadEncoderVel = GetQuadEncoderSpeed();

  // Set a flag to print this value in the loop()
  quadEncoderFlag = 1;
}

void GetAbsEncoderData() {
  // Get the positions from each abs encoder
  absEncCurrentPositionFR = absEncoderFR.AbsEncPos();
  absEncCurrentPositionFL = absEncoderFL.AbsEncPos();
  absEncCurrentPositionBR = absEncoderBR.AbsEncPos();
  absEncCurrentPositionBL = absEncoderBL.AbsEncPos();

  // Get the velocities from each abs encoder
  absEncCurrentVelocityFR = absEncoderFR.AbsEncVel();
  absEncCurrentVelocityFL = absEncoderFL.AbsEncVel();
  absEncCurrentVelocityBR = absEncoderBR.AbsEncVel();
  absEncCurrentVelocityBL = absEncoderBL.AbsEncVel();

  // Set flag to print this value in the loop()
  absEncoderFlag = 1;
}

void InitStuff() {
  // Initializing serial communication at 115200 bps
  Serial.begin(115200);
  
  // Setting pin modes
  pinMode(led, OUTPUT); // for built-in LED, HIGH is on and vice versa
  pinMode(steerPotPin, INPUT);  // steering potentiometer pin
  pinMode(battCell1Pin, INPUT); // pin of voltage sensor measuring battery cell #1
  pinMode(battCell2Pin, INPUT); // pin of voltage sensor measuring battery cell #2

  // Initializing the FFI controller program
  InitFFIC();
}

void ActuateAction() {
  // This function calls on independent functions to read the current and ...
  // ... compute the FF-PI controller to actuate a PWM accordingly.
  funcTime = micros();
  for(i=0;i<4;i++) {
    current[i] = GetCurrent(i);
    pwm[i] = CCUpdatePWM(i,setI[i],current[i],battVoltage);
  }
  funcTime = micros() - funcTime;
}

void SineInput() {
  setI[0] = -5;
  setI[1] = 0;
  setI[2] = 0;
  setI[3] = 0;
}

//------------------------------------------------------------------

void setup() {

  InitStuff();

  // Wait until serial port is opened
  while (!Serial) {delay(1);}

  // Print initial current setpoints
  for(int i=0;i<4;i++) {Serial.println(setI[i]);}

  // Initialize Timer Interupts for 33Hz
  // Timer1.attachInterrupt(GetQuadEncoderData).start(30303); // Timer for Quad Encoder (33Hz)
  // Timer2.attachInterrupt(GetAbsEncoderData).start(30303);  // Timer for Abs Encoder (33Hz)
  Timer3.attachInterrupt(ActuateAction).start(10000); // Timer for ActuateAction function
  Timer4.attachInterrupt(SineInput).start(5000000); // Timer for sinusoidal input to actuators

}

void loop() {

  GetVoltage();


  timeCount = millis();

  /*
  // Print out quadrature encoder data (Validation)
  if (quadEncoderFlag == 1) {
    Serial.print("Quad Encoder Velocity (rad/s): ");
    Serial.println(quadEncoderVel);
    quadEncoderFlag = 0;
  }
  */

  // Print out absolute encoder data (Validation)
  if (absEncoderFlag == 1) {
    Serial.print("FR Abs Encoder Position: ");
    Serial.print(absEncCurrentPositionFR);
    Serial.print(" FR Abs Encoder Velocity: ");
    Serial.println(absEncCurrentVelocityFR);
    absEncoderFlag = 0;


    // Independent of the abs. encoders but should print at the same time
    Serial.print("Currents: ");
    for(int i=0;i<4;i++) {Serial.print(current[i]); Serial.print(",");}
    Serial.println("");
  }

  // Serial.print("Currents: ");
  // for(i=0;i<4;i++) {Serial.print(current[i],3); Serial.print(",");}
  // Serial.println("");
  // for(i=0;i<4;i++) {Serial.print(setI[i],3); Serial.print(",");}
  // Serial.println("");
  // Serial.print("Voltage: "); Serial.println(battVoltage,2);

  // if(currentFlag == 1) {
  //   Serial.println("Currents: ");
  //   for(int i=0;i<4;i++) {Serial.print(current[i],3); Serial.print(",");}
  //   // Serial.println("");
  //   // for(i=0;i<4;i++) {Serial.print(setI[i],3); Serial.print(",");}
  //   for(int i=0;i<4;i++) {Serial.print(pwm[i]); Serial.print(",");}
  //   Serial.print("--");
  //   Serial.print("Voltage: "); Serial.print(battVoltage,2);
  //   Serial.print(", "); Serial.println(funcTime);
  //   currentFlag = 0;
  // }

  Serial.println("Currents: ");
  for(int i=0;i<4;i++) {Serial.print(current[i],3); Serial.print(",");}
  // Serial.println("");
  // for(i=0;i<4;i++) {Serial.print(setI[i],3); Serial.print(",");}
  for(int i=0;i<4;i++) {Serial.print(pwm[i]); Serial.print(",");}
  Serial.print("--");
  Serial.print("Voltage: "); Serial.print(battVoltage,2);
  Serial.print(", "); Serial.print(funcTime);
  // Serial.print(", "); for(int i=0;i<4;i++) {Serial.print(direc[i]); Serial.print(",");}
  Serial.println("");
  currentFlag = 0;


  delay(100);
}
