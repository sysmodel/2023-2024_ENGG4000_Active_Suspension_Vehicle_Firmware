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
#include "Waveforms.h"

//------------------------------------------------------------------

#define led LED_BUILTIN
#define steerPotPin A4
#define battCell1Pin A3
#define battCell2Pin A2

//------------------------------------------------------------------

// Variable definitions
float current[4] = {0,0,0,0}; // array of measured current values
float offset[4] = {0,0,0,0};  // array of current offset values
int i = 0; // used for indexing
float voltArray[2] = {0,0};
float battVoltage = 0;
long timeCount = millis();
int sineCount = 0;

// Motor driver
uint32_t mdEnPins[4] = {4,5,2,3};
uint32_t mdIn1Pins[4] = {51,53,47,49};
uint32_t mdIn2Pins[4] = {50,52,46,48};
float pwm[4] = {0,0,0,0};
bool direc[4] = {1,1,1,1}; // motor action directions; 1 is up, 0 is down

// Jetson communication
float setI[4] = {0,0,0,0}; // array of stored current setpoints

// FF-PI controller
float resistance = 0.663; // in ohm; original value was 0.2234 ohm, but this was not reflected in the current control
double Kp=10, Ki=200, Kd=0;  // specify PID tuning parameters
double maxCorrect = 255; // used in piFR.SetOutputLimits() function
double currentPI[4] = {0,0,0,0}; // array of current values to be used by the PI objects
double outPI[4] = {0,0,0,0}; // array to store the outputs of the PI objects
double setIPI[4] = {0,0,0,0}; // array of current setpoint values to be used by the PI objects
float setV[4] = {0,0,0,0};
float pwmOffset[4] = {0,0,0,0};
int pwmCeiling = 40;

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

void SetDirec(int wheel, String dir) {
  if (dir == "UP") {
    digitalWrite(mdIn1Pins[wheel], LOW);
    digitalWrite(mdIn2Pins[wheel], HIGH);
  } else if (dir == "DOWN") {
    digitalWrite(mdIn1Pins[wheel], HIGH);
    digitalWrite(mdIn2Pins[wheel], LOW);
  }
}

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
    SetDirec(i,"DOWN");
    } else {SetDirec(i,"UP");}
    current[i] = 1.0637*current[i] + 0.1416;
  }
}

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
  ina260FR.begin(0x41);
  ina260FL.begin(0x44);
  ina260BR.begin(0x45);
  ina260BL.begin(0x40);
  while (!ina260FR.begin()) {
    Serial.println("Couldn't find INA260 chip (FR)");
    // while (1);
  }
  while (!ina260FL.begin()) {
    Serial.println("Couldn't find INA260 chip (FL)");
    // while (1);
  }
  while (!ina260BR.begin()) {
    Serial.println("Couldn't find INA260 chip (BR)");
    // while (1);
  }
  while (!ina260BL.begin()) {
    Serial.println("Couldn't find INA260 chip (BL)");
    // while (1);
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

void CCUpdatePWM() {
  for(i=0;i<4;i++) {
    setIPI[i] = setI[i];
    currentPI[i] = current[i];
    setV[i] = resistance * setI[i];
    pwmOffset[i] = int(setV[i] / battVoltage * 255.0); // Feedforward (FF) control
  }
  piFR.Compute();
  piFL.Compute();
  piBR.Compute();
  piBL.Compute();
  for(i=0;i<4;i++) {
    pwm[i] = pwmOffset[i] + int(outPI[i]); // FF + PI control
    if (pwm[i] > pwmCeiling) {pwm[i] = pwmCeiling;} else if (pwm[i] < 0) {pwm[i] = 0;}
    analogWrite(mdEnPins[i], pwm[i]);
  }
}

void ActuateAction() {
  // This function calls on independent functions to read the current and ...
  // ... compute the FF-PI controller to actuate a PWM accordingly.

  GetCurrent();
  CCUpdatePWM();
}

void SineInput() {
  for(i=0;i<4;i++) {
    setI[i] = float(int(waveformsTable[0][sineCount+i*5])/700);
  }
  // setI[0] = int(waveformsTable[0][sineCount]);
  // setI[1] = int(waveformsTable[0][sineCount+5]);
  // setI[2] = int(waveformsTable[0][sineCount+10]);
  // setI[3] = int(waveformsTable[0][sineCount+15]);
  sineCount++;
  if((sineCount+i*5) >= maxSamplesNum) {sineCount = 0;}
}

//------------------------------------------------------------------

void setup() {

  InitStuff();

  // Wait until serial port is opened
  while (!Serial) {delay(1);}

  // Initialize Timmer Interupts for 33Hz
  // Timer1.attachInterrupt(GetQuadEncoderData).start(30303); // Timer for Quad Encoder (33Hz)
  // Timer2.attachInterrupt(GetAbsEncoderData).start(30303);  // Timer for Abs Encoder (33Hz)
  Timer3.attachInterrupt(ActuateAction).start(10000); // Timer for ActuateAction function
  Timer4.attachInterrupt(SineInput).start(8000); // Timer for sinusoidal input to actuators
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
    for(i=0;i<4;i++) {Serial.print(current[i]); Serial.print(",");}
    Serial.println("");
  }

  // Serial.print("Currents: ");
  // for(i=0;i<4;i++) {Serial.print(current[i],3); Serial.print(",");}
  // Serial.println("");
  // for(i=0;i<4;i++) {Serial.print(setI[i],3); Serial.print(",");}
  // Serial.println("");
  // Serial.print("Voltage: "); Serial.println(battVoltage,2);

  Serial.println("Currents: ");
  for(i=0;i<4;i++) {Serial.print(current[i],3); Serial.print(",");}
  // Serial.println("");
  for(i=0;i<4;i++) {Serial.print(setI[i],3); Serial.print(",");}
  Serial.print("--");
  Serial.print("Voltage: "); Serial.println(battVoltage,2);

  delay(50);
}
