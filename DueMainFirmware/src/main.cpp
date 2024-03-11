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
#include "Adafruit_INA260.h"
#include "PID_v1.h"

//------------------------------------------------------------------

#define led LED_BUILTIN
#define steerPotPin A0
#define battCell1Pin A1
#define battCell2Pin A2

//------------------------------------------------------------------

// Variable definitions
uint8_t quadEncoderFlag = 0;
uint8_t absEncoderFlag = 0;
uint8_t resolution = 12;
float current[4] = {0,0,0,0}; // array of measured current values
float offset[4] = {0,0,0,0};  // array of current offset values
int i = 0; // used for indexing
float voltArray[2] = {0,0};
float battVoltage = 0;
uint32_t mdEnPins[4] = {14,14,14,14};
uint32_t mdIn1Pins[4] = {14,14,14,14};
uint32_t mdIn2Pins[4] = {14,14,14,14};

// FF-PI controller
float resistance = 0.663; // in ohm; original value was 0.2234 ohm, but this was not reflected in the current control
double Kp=10, Ki=200, Kd=0;  // specify PID tuning parameters
double maxCorrect = 255; // used in piFR.SetOutputLimits() function
double currentPI[4] = {0,0,0,0}; // array of current values to be used by the PI objects
double outPI[4] = {0,0,0,0}; // array to store the outputs of the PI objects
double setIPI[4] = {0,0,0,0}; // array of current setpoint values to be used by the PI objects

// Define global variable to store abs encoder positions and speeds
uint16_t absEncCurrentPositionFR; // Front Right
uint16_t absEncCurrentPositionFL; // Front Left
uint16_t absEncCurrentPositionBR; // Back Right
uint16_t absEncCurrentPositionBL; // Back Left
double absEncCurrentVelocityFR; // Front Right
double absEncCurrentVelocityFL; // Front Left
double absEncCurrentVelocityBR; // Back Right
double absEncCurrentVelocityBL; // Back Left

// Define pins for each encoder
uint8_t sdoPinFR = 5;
uint8_t sckPinFR = 6;
uint8_t csPinFR = 7;
uint8_t sdoPinFL = 2;
uint8_t sckPinFL = 3;
uint8_t csPinFL = 4;
uint8_t sdoPinBR = 11;
uint8_t sckPinBR = 12;
uint8_t csPinBR = 13;
uint8_t sdoPinBL = 8;
uint8_t sckPinBL = 9;
uint8_t csPinBL = 10;
// John's suggestion for the above. Structure of arrays: {FR, FL, BR, BL}; indices: {0,1,2,3}.
// uint8_t sdoPin[4] = {5, 2, 11, 8};
// uint8_t sckPin[4] = {6, 3, 12, 9};
// uint8_t csPin[4] = {7, 4, 13, 10};

//------------------------------------------------------------------

// Creation of absolute encoder objects
AbsEnc absEncoderFR(sckPinFR, csPinFR, sdoPinFR, resolution);
AbsEnc absEncoderFL(sckPinFL, csPinFL, sdoPinFL, resolution);
AbsEnc absEncoderBR(sckPinBR, csPinBR, sdoPinBR, resolution);
AbsEnc absEncoderBL(sckPinBL, csPinBL, sdoPinBL, resolution);
// John's suggestion for the above. Structure of arrays: {FR, FL, BR, BL}; indices: {0,1,2,3}.
// AbsEnc absEncoderFR(sckPin[0], csPin[0], sdoPin[0], resolution);
// AbsEnc absEncoderFL(sckPin[1], csPin[1], sdoPin[1], resolution);
// AbsEnc absEncoderBR(sckPin[2], csPin[2], sdoPin[2], resolution);
// AbsEnc absEncoderBL(sckPin[3], csPin[3], sdoPin[3], resolution);

// Creation of INA260 current sensor objects
Adafruit_INA260 ina260FR = Adafruit_INA260();
Adafruit_INA260 ina260FL = Adafruit_INA260();
Adafruit_INA260 ina260BR = Adafruit_INA260();
Adafruit_INA260 ina260BL = Adafruit_INA260();

// Creating of PID objects
PID piFR(&currentPI[0], &outPI[0], &setIPI[0], Kp, Ki, Kd, DIRECT);
PID piFL(&currentPI[1], &outPI[1], &setIPI[1], Kp, Ki, Kd, DIRECT);
PID piBR(&currentPI[2], &outPI[2], &setIPI[2], Kp, Ki, Kd, DIRECT);
PID piBL(&currentPI[3], &outPI[3], &setIPI[3], Kp, Ki, Kd, DIRECT);

//------------------------------------------------------------------

void GetCurrent() {
  for(i=0;i<4;i++) {
    switch(i) {
      case 0:
        current[i] = float(ina260FR.readCurrent())/1000.0 - offset[i];
      case 1:
        current[i] = float(ina260FL.readCurrent())/1000.0 - offset[i];
      case 2:
        current[i] = float(ina260BR.readCurrent())/1000.0 - offset[i];
      case 3:
        current[i] = float(ina260BL.readCurrent())/1000.0 - offset[i];
    }
    if (current[i] < 0) {
    current[i] = -current[i];
    }
    current[i] = 1.0637*current[i] + 0.1416;
  }
}

void GetVoltage() {
  voltArray[0] = float(map(analogRead(battCell1Pin), 0, 1023, 0, 3300)) * 0.005;
  voltArray[1] = float(map(analogRead(battCell2Pin), 0, 1023, 0, 3300)) * 0.005;
  battVoltage = voltArray[0] + voltArray[1];
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
  // Initialize serial communication at 115200 bps
  Serial.begin(115200);
  
  // Setting pin modes
  pinMode(led, OUTPUT); // for built-in LED, HIGH is on and vice versa
  for(i=0;i<4;i++) {  // pins of motor drivers
    pinMode(mdEnPins[i], OUTPUT);
    pinMode(mdIn1Pins[i], OUTPUT);
    pinMode(mdIn2Pins[i], OUTPUT);
  }
  pinMode(steerPotPin, INPUT);  // steering potentiometer pin
  pinMode(battCell1Pin, INPUT); // pin of voltage sensor measuring battery cell #1
  pinMode(battCell2Pin, INPUT); // pin of voltage sensor measuring battery cell #2

  // Begin current sensor reading and set averaging count
  if (!ina260FR.begin() && !ina260FL.begin() && !ina260BR.begin() && !ina260BL.begin()) {
    Serial.println("Couldn't find INA260 chip");
    while (1);
  }
  Serial.println("Found INA260 chip");
  ina260FR.setAveragingCount(INA260_COUNT_4);
  ina260FL.setAveragingCount(INA260_COUNT_4);
  ina260BR.setAveragingCount(INA260_COUNT_4);
  ina260BL.setAveragingCount(INA260_COUNT_4);

  // Set PID mode, sampling time, and output limits
  piFR.SetMode(AUTOMATIC);
  piFL.SetMode(AUTOMATIC);
  piBR.SetMode(AUTOMATIC);
  piBL.SetMode(AUTOMATIC);
  piFR.SetSampleTime(10);
  piFL.SetSampleTime(10);
  piBR.SetSampleTime(10);
  piBL.SetSampleTime(10);
  piFR.SetOutputLimits(-maxCorrect, maxCorrect);
  piFL.SetOutputLimits(-maxCorrect, maxCorrect);
  piBR.SetOutputLimits(-maxCorrect, maxCorrect);
  piBL.SetOutputLimits(-maxCorrect, maxCorrect);
}

//------------------------------------------------------------------

void setup() {

  InitStuff();

  // Wait until serial port is opened
  while (!Serial) {delay(1);}

  // Initialize Timmer Interupts for 33Hz
  Timer1.attachInterrupt(GetQuadEncoderData).start(30303); // Timer for Quad Encoder (33Hz)
  Timer2.attachInterrupt(GetAbsEncoderData).start(30303); // Timer for Abs Encoder (33Hz)
}

void loop() {

  GetVoltage();

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
  }
  delay(500);
}
