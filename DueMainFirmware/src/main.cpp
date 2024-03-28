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
#include "SafetyStop.h"
#include "VCNL4040.h"
#include "BNO08x.h"

//------------------------------------------------------------------

#define led LED_BUILTIN
#define steerPotPin A4
#define battCell1Pin A3
#define battCell2Pin A2
#define STOP_SWITCH_PIN_OUT 30
#define STOP_SWITCH_PIN_IN 31

//------------------------------------------------------------------

// Variable definitions
float current[4] = {0,0,0,0}; // array of measured current values
float offset[4] = {0,0,0,0};  // array of current offset values
int i = 0; // used for indexing
float voltArray[2] = {0,0};
float battVoltage = 0;
long timeCount = millis();
int sineCount = 0;
int funcTime = 0;
int currentFlag = 0;

// Motor driver
uint32_t mdEnPins[4] = {4,5,2,3};
uint32_t mdIn1Pins[4] = {51,53,47,49};
uint32_t mdIn2Pins[4] = {50,52,46,48};
int pwm[4] = {0,0,0,0};
bool direc[4] = {0,0,0,0}; // motor action directions; 1 is up, 0 is down
bool desDirec[4] = {0,0,0,0}; // desired motor action directions; 1 is up, 0 is down

// Jetson communication
float setI[4] = {0,0,0,0}; // array of stored current setpoints

// FF-PI controller
float resistance[4] = {0.372,0.301,0.310,0.325}; // in ohm; original value was 0.2234 ohm, but this was not reflected in the current control
double Kp=0, Ki=400, Kd=0;  // specify PID tuning parameters
double maxCorrect = 255; // used in piFR.SetOutputLimits() function
double currentPI[4] = {0,0,0,0}; // array of current values to be used by the PI objects
double outPI[4] = {0,0,0,0}; // array to store the outputs of the PI objects
double setIPI[4] = {0,0,0,0}; // array of current setpoint values to be used by the PI objects
float setV[4] = {0,0,0,0};
int pwmOffset[4] = {0,0,0,0};
int pwmCeiling = 120;

// Encoders
uint8_t quadEncoderFlag = 0;
uint8_t absEncoderFlag = 0;
uint8_t resolution = 12;
// - Define global variable to store abs encoder positions and speeds
double absEncCurrentPosition[4] = {0,0,0,0}; // array of absolute encoder positions
double absEncCurrentVelocity[4] = {0,0,0,0}; // array of absolute encoder velocities
// - Define pins for each encoder; structure of arrays: {FR, FL, BR, BL}; indices: {0,1,2,3}
uint8_t sdoPin[4] = {11, 31, 7, 9};
uint8_t sckPin[4] = {10, 33, 6, 8};
uint8_t csPin[4] = {25, 35, 27, 26};

// Safety stop
int stopCondition = 0;
int lastStopCondition = 0;
bool switchStatus = false;

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

// Create IR sensor object
VCNL4040IR vcnl4040 = VCNL4040IR();
double carHeight = 0.0;

// Create IMU object and corresponding global variables
BNO08xIMU bno08x = BNO08xIMU();
sh2_SensorValue_t gyroValue;
sh2_SensorValue_t linearAccelValue;
sh2_SensorValue_t pitchAndRollValue;
float pitch;
float roll;
float accelerationZ;
float gyroRoll;
float gyroPitch;
int FlagIMU = 0;

//------------------------------------------------------------------

void SetDirec(int wheel, bool dir) {
  digitalWrite(mdIn1Pins[wheel], HIGH);
  digitalWrite(mdIn2Pins[wheel], HIGH);
  if (dir == 1) { // car up
    digitalWrite(mdIn2Pins[wheel], HIGH);
    digitalWrite(mdIn1Pins[wheel], LOW);
  } else if (dir == 0) { // car down
    digitalWrite(mdIn1Pins[wheel], HIGH);
    digitalWrite(mdIn2Pins[wheel], LOW);
  }
}

void GetCurrent() {
  for(int i=0;i<4;i++) {
    switch(i) {
      case 0:
        current[i] = (ina260FR.readCurrent() - offset[i])/1000.0;
        break;
      case 1:
        current[i] = (ina260FL.readCurrent() - offset[i])/1000.0;
        break;
      case 2:
        current[i] = (ina260BR.readCurrent() - offset[i])/1000.0;
        break;
      case 3:
        current[i] = (ina260BL.readCurrent() - offset[i])/1000.0;
        break;
    }
    current[i] = 1.0887*current[i];
  }
  currentFlag = 1;
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
  absEncCurrentPosition[0] = absEncoderFR.GetRackPosition();
  absEncCurrentPosition[1] = absEncoderFL.GetRackPosition();
  absEncCurrentPosition[2] = absEncoderBR.GetRackPosition();
  absEncCurrentPosition[3] = absEncoderBL.GetRackPosition();

  // Get the velocities from each abs encoder
  absEncCurrentVelocity[0] = absEncoderFR.AbsEncVel();
  absEncCurrentVelocity[1] = absEncoderFL.AbsEncVel();
  absEncCurrentVelocity[2] = absEncoderBR.AbsEncVel();
  absEncCurrentVelocity[3] = absEncoderBL.AbsEncVel();

  // Set flag to print this value in the loop()
  absEncoderFlag = 1;
}

void GetDataIMU()
{
  bno08x.GetDataIMU();
  pitch = bno08x._ypr._pitch - bno08x._imuCAL._pitchOffset;
  roll = bno08x._ypr._roll - bno08x._imuCAL._rollOffset;
  gyroPitch = bno08x._rpRates._pitchRate;
  gyroRoll = bno08x._rpRates._rollRate;
  accelerationZ = bno08x._rpRates._zAcc;

  FlagIMU = 1;
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
  pinMode(STOP_SWITCH_PIN_OUT, OUTPUT); // high pin will passed/stopped by switch
  pinMode(STOP_SWITCH_PIN_IN, INPUT); // input pin to read switch output
  digitalWrite(STOP_SWITCH_PIN_OUT, HIGH);

  // IMU Setup
  bno08x.BeginBNO08x();
  bno08x.SetReports();

  // Begin current sensor reading and set averaging count
  if (!ina260FR.begin(0x41)) {
    Serial.println("Couldn't find INA260 chip (FR)");
    while (1);
  }
  if (!ina260FL.begin(0x44)) {
    Serial.println("Couldn't find INA260 chip (FL)");
    while (1);
  }
  if (!ina260BR.begin(0x45)) {
    Serial.println("Couldn't find INA260 chip (BR)");
    // while (1);
  }
  if (!ina260BL.begin(0x40)) {
    Serial.println("Couldn't find INA260 chip (BL)");
    // while (1);
  }
  
  ina260FR.setMode(INA260_MODE_CURRENT_CONTINUOUS);
  ina260FL.setMode(INA260_MODE_CURRENT_CONTINUOUS);
  ina260BR.setMode(INA260_MODE_CURRENT_CONTINUOUS);
  ina260BL.setMode(INA260_MODE_CURRENT_CONTINUOUS);
  ina260FR.setCurrentConversionTime(INA260_TIME_2_116_ms);
  ina260FL.setCurrentConversionTime(INA260_TIME_2_116_ms);
  ina260BR.setCurrentConversionTime(INA260_TIME_2_116_ms);
  ina260BL.setCurrentConversionTime(INA260_TIME_2_116_ms);

  // Set PID mode, sampling time, and output limits
  piFR.SetMode(AUTOMATIC);
  piFL.SetMode(AUTOMATIC);
  piBR.SetMode(AUTOMATIC);
  piBL.SetMode(AUTOMATIC);
  piFR.SetSampleTime(3);
  piFL.SetSampleTime(3);
  piBR.SetSampleTime(3);
  piBL.SetSampleTime(3);
  piFR.SetOutputLimits(-maxCorrect, maxCorrect);
  piFL.SetOutputLimits(-maxCorrect, maxCorrect);
  piBR.SetOutputLimits(-maxCorrect, maxCorrect);
  piBL.SetOutputLimits(-maxCorrect, maxCorrect);

  absEncoderFR.SetEncPositions(362, 1247, false);
  absEncoderFL.SetEncPositions(3906, 3196, true);
  absEncoderBR.SetEncPositions(1648, 544, true);
  absEncoderBL.SetEncPositions(3901, 810, false);


}

void GetIRData()
{
  carHeight = vcnl4040.GetDistance();
}

void CCUpdatePWM() {
  for(int i=0;i<4;i++) {
    setIPI[i] = setI[i];
    currentPI[i] = current[i];
    setV[i] = resistance[i] * setI[i];
    pwmOffset[i] = int(setV[i] / battVoltage * 255.0); // Feedforward (FF) control
  }
  piFR.Compute();
  piFL.Compute();
  piBR.Compute();
  piBL.Compute();
  for(int i=0;i<4;i++) {
    pwm[i] = pwmOffset[i] + int(outPI[i]); // FF + PI control
    if(pwm[i] < 0) {
      pwm[i] = -pwm[i];
      desDirec[i] = 0;
    } else {
      desDirec[i] = 1;
    }
    if (direc[i] != desDirec[i]) {SetDirec(i,desDirec[i]); direc[i] = desDirec[i];}
    if (pwm[i] > pwmCeiling) {pwm[i] = pwmCeiling;} else if (pwm[i] < 0) {pwm[i] = 0;}
    analogWrite(mdEnPins[i], pwm[i]);
  }
}

void ActuateAction() {
  // This function calls on independent functions to read the current and ...
  // ... compute the FF-I controller to actuate a PWM accordingly.
  if (stopCondition == 0) {
    funcTime = micros();
    GetCurrent();
    CCUpdatePWM();
    funcTime = micros() - funcTime;
  }
}

void SineInput() {
  // for(i=0;i<4;i++) {
  //   setI[i] = float(int(waveformsTable[0][sineCount+i*5]-1000))/800;
  //   if(setI[i] < 0) {setI[i] = -setI[i]; desDirec[i] = 0;} else {desDirec[i] = 1;}
  // }
  // // setI[0] = int(waveformsTable[0][sineCount]);
  // // setI[1] = int(waveformsTable[0][sineCount+5]);
  // // setI[2] = int(waveformsTable[0][sineCount+10]);
  // // setI[3] = int(waveformsTable[0][sineCount+15]);
  // sineCount++;
  // if((sineCount+i*5) >= maxSamplesNum) {sineCount = 0;}

  // for(i=0;i<4;i++) {
  //   setI[i] = -1*sineCount+i/2;
  //   if(setI[i] < 0) {setI[i] = -setI[i]; desDirec[i] = 0;} else {desDirec[i] = 1;}
  // }
  // sineCount++;
  // if(sineCount>4) {sineCount=0;}
  setI[0] = -10;
  setI[1] = 0;
  setI[2] = 0;
  setI[3] = 0;
  // for(int i=0;i<4;i++) {
  //   desDirec[i] = 0;
  //   SetDirec(i,desDirec[i]);
  // }
}

void SendDataFunc()
{
  Serial.print("{");
  Serial.print(absEncCurrentPosition[0],2);Serial.print(","); 
  Serial.print(absEncCurrentPosition[1],2);Serial.print(","); 
  Serial.print(absEncCurrentPosition[2],2);Serial.print(","); 
  Serial.print(absEncCurrentPosition[3],2);Serial.print(","); 
  Serial.print(absEncCurrentVelocity[0],2);Serial.print(","); 
  Serial.print(absEncCurrentVelocity[1],2);Serial.print(","); 
  Serial.print(absEncCurrentVelocity[2],2);Serial.print(","); 
  Serial.print(absEncCurrentVelocity[3],2);Serial.print(","); 
  Serial.print(quadEncoderVel,2);Serial.print(","); 
  Serial.print(pitch,2);Serial.print(","); 
  Serial.print(roll,2);Serial.print(","); 
  Serial.print(accelerationZ,2);Serial.print(","); 
  Serial.print(gyroRoll,2);Serial.print(","); 
  Serial.print(gyroPitch,2);
  Serial.println("}");
}

void CheckStop() {
  if (digitalRead(STOP_SWITCH_PIN_IN) == HIGH) {
    switchStatus = true;
  } else {
    switchStatus = false;
  }
  switchStatus = CheckStopCondition(&(voltArray[0]), &(current[0]), (double*)&(absEncCurrentPosition[0]), switchStatus);
}

//------------------------------------------------------------------

void setup() {

  InitStuff();

  // Wait until serial port is opened
  while (!Serial) {delay(1);}

  // setI[4] = {-8.0,-8.0,1.0,1.0};
  // for(int i=0;i<4;i++) {Serial.println(setI[i]);}
  for(int i=0;i<4;i++) {desDirec[i] = 0;}
  for(int i=0;i<4;i++) {SetDirec(i,0);}

  // Initialize Timmer Interupts for 33Hz
  Timer1.attachInterrupt(GetQuadEncoderData).start(30303); // Timer for Quad Encoder (33Hz)
  Timer2.attachInterrupt(GetAbsEncoderData).start(30303);  // Timer for Abs Encoder (33Hz)
  // Timer3.attachInterrupt(ActuateAction).start(3000); // Timer for ActuateAction function
  // Timer4.attachInterrupt(SineInput).start(5000000); // Timer for sinusoidal input to actuators
  // Timer5.attachInterrupt(CheckStop).start(1000000);
  Timer6.attachInterrupt(GetDataIMU).start(30303);
}

void loop() {
  if (stopCondition == 0) {
    GetVoltage();

    // timeCount = millis();

    if (Serial.available())
    {
      while(Serial.available())
      {
        Serial.read();
      }
      SendDataFunc();
    }

    delay(100);

  } else if (stopCondition == MANUAL_STOP) {
    // do stuff
    for(i=0;i<4;i++) {
      analogWrite(pwm[i],0);
      SetDirec(i,0);
    }
    Serial.print("Manual stop condition. Code: ");
    Serial.print(stopCondition);
    Serial.println();
  } else {
    // do stuff
    for(i=0;i<4;i++) {
      analogWrite(pwm[i],0);
      SetDirec(i,0);
    }
    Serial.print("Auto stop condition. Code: ");
    Serial.print(stopCondition);
    Serial.println();
    while(1);
    // If a limit is crossed, one should check the serial log.
    // The only way to get out of an auto stop is to restart the program.
  }
}
