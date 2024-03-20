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

// Stop condition codes
#define BATT_VOLTAGE_TOO_LOW 1
#define FR_CURRENT_TOO_HIGH 2
#define FL_CURRENT_TOO_HIGH 3
#define BR_CURRENT_TOO_HIGH 4
#define BL_CURRENT_TOO_HIGH 5
#define FR_ABSENC_TOO_FAR 6
#define FL_ABSENC_TOO_FAR 7
#define BR_ABSENC_TOO_FAR 8
#define BL_ABSENC_TOO_FAR 9
#define MANUAL_STOP 10
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
float resistance[4] = {0.663*(3/6.2)*(3/2.35)*(3/3.3),0.663,0.663,0.663}; // in ohm; original value was 0.2234 ohm, but this was not reflected in the current control
double Kp=0, Ki=400, Kd=0;  // specify PID tuning parameters
double maxCorrect = 255; // used in piFR.SetOutputLimits() function
double currentPI[4] = {0,0,0,0}; // array of current values to be used by the PI objects
double outPI[4] = {0,0,0,0}; // array to store the outputs of the PI objects
double setIPI[4] = {0,0,0,0}; // array of current setpoint values to be used by the PI objects
float setV[4] = {0,0,0,0};
int pwmOffset[4] = {0,0,0,0};
int pwmCeiling = 120;
long controlTimer = 10000;

// Encoders
uint8_t quadEncoderFlag = 0;
uint8_t absEncoderFlag = 0;
uint8_t resolution = 12;
// - Define global variable to store abs encoder positions and speeds
uint16_t absEncCurrentPosition[4] = {0,0,0,0}; // array of absolute encoder positions
double absEncCurrentVelocity[4] = {0,0,0,0}; // array of absolute encoder velocities
// - Define pins for each encoder; structure of arrays: {FR, FL, BR, BL}; indices: {0,1,2,3}
uint8_t sdoPin[4] = {11, 13, 7, 9};
uint8_t sckPin[4] = {10, 12, 6, 8};
uint8_t csPin[4] = {25, 24, 27, 26};

// Safety stop
int stopCondition = 0;
int lastStopCondition = 0;
float cellVoltLowLimit = 3.4; // in V, true low limit of battery is 3.2V/cell (6.4V)
float currentHighLimit = 10; // administrative limit for high current
int absEncPosLimits[4] = {0,0,0,0}; // absolute encoder limits
bool switchStatus = 0;

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

void SetDirec(int wheel, bool dir) {
  if (dir == 1) {
    digitalWrite(mdIn1Pins[wheel], LOW);
    digitalWrite(mdIn2Pins[wheel], HIGH);
  } else if (dir == 0) {
    digitalWrite(mdIn1Pins[wheel], HIGH);
    digitalWrite(mdIn2Pins[wheel], LOW);
  }
}

void GetCurrent() {
  for(int i=0;i<4;i++) {
    switch(i) {
      case 0:
        current[i] = float(ina260FR.readCurrent())/1000.0 - offset[i];
        current[i] = 1.0887*current[i];
        if (current[i] > currentHighLimit) {stopCondition = FR_CURRENT_TOO_HIGH;}
        break;
      case 1:
        current[i] = float(ina260FL.readCurrent())/1000.0 - offset[i];
        current[i] = 1.0887*current[i];
        if (current[i] > currentHighLimit) {stopCondition = FL_CURRENT_TOO_HIGH;}
        break;
      case 2:
        current[i] = float(ina260BR.readCurrent())/1000.0 - offset[i];
        current[i] = 1.0887*current[i];
        if (current[i] > currentHighLimit) {stopCondition = BR_CURRENT_TOO_HIGH;}
        break;
      case 3:
        current[i] = float(ina260BL.readCurrent())/1000.0 - offset[i];
        current[i] = 1.0887*current[i];
        if (current[i] > currentHighLimit) {stopCondition = BL_CURRENT_TOO_HIGH;}
        break;
    }
    if (current[i] < 0) {
      direc[i] = 0;
    } else {
      direc[i] = 1;
    }
  }
  currentFlag = 1;
}

void GetVoltage() {
  voltArray[0] = float(map(analogRead(battCell1Pin), 0, 1023, 0, 3300)) * 0.005;
  battVoltage = float(map(analogRead(battCell2Pin), 0, 1023, 0, 3300)) * 0.005;
  voltArray[1] = battVoltage - voltArray[0];
  for(i=0;i<2;i++) {if (voltArray[i] < cellVoltLowLimit) {stopCondition = BATT_VOLTAGE_TOO_LOW;}}
}

void GetQuadEncoderData() {
  // Get the vehicle speed in rad/s
  quadEncoderVel = GetQuadEncoderSpeed();

  // Set a flag to print this value in the loop()
  quadEncoderFlag = 1;
}

void GetAbsEncoderData() {
  // Get positions and velocities from each abs encoder, set flags depending on limits
  for(i=0;i<4;i++) {
    switch(i) {
      case 0:
        absEncCurrentPosition[i] = absEncoderFR.AbsEncPos();
        absEncCurrentVelocity[i] = absEncoderFR.AbsEncVel();
        if (absEncoderFR.AbsEncPos() > absEncPosLimits[i]) {stopCondition = FR_ABSENC_TOO_FAR;}
        break;
      case 1:
        absEncCurrentPosition[i] = absEncoderFL.AbsEncPos();
        absEncCurrentVelocity[i] = absEncoderFL.AbsEncVel();
        if (absEncoderFL.AbsEncPos() < absEncPosLimits[i]) {stopCondition = FL_ABSENC_TOO_FAR;}
        break;
      case 2:
        absEncCurrentPosition[i] = absEncoderBR.AbsEncPos();
        absEncCurrentVelocity[i] = absEncoderBR.AbsEncVel();
        if (absEncoderBR.AbsEncPos() < absEncPosLimits[i]) {stopCondition = BR_ABSENC_TOO_FAR;}
        break;
      case 3:
        absEncCurrentPosition[i] = absEncoderBL.AbsEncPos();
        absEncCurrentVelocity[i] = absEncoderBL.AbsEncVel();
        if (absEncoderBL.AbsEncPos() > absEncPosLimits[i]) {stopCondition = BL_ABSENC_TOO_FAR;}
        break;
    }
  }

  // Set flag to print this value in the loop()
  absEncoderFlag = 1;
}

void InitStuff() {
  // Initialize serial communication at 115200 bps
  Serial.begin(115200);
  
  // Setting pin modes
  pinMode(led, OUTPUT); // for built-in LED, HIGH is on and vice versa
  pinMode(STOP_SWITCH_PIN_OUT, OUTPUT); // high pin will passed/stopped by switch
  pinMode(STOP_SWITCH_PIN_IN, INPUT); // input pin to read switch output
  digitalWrite(STOP_SWITCH_PIN_OUT, HIGH);
  for(i=0;i<4;i++) {  // pins of motor drivers
    pinMode(mdEnPins[i], OUTPUT);
    pinMode(mdIn1Pins[i], OUTPUT);
    pinMode(mdIn2Pins[i], OUTPUT);
  }
  pinMode(steerPotPin, INPUT);  // steering potentiometer pin
  pinMode(battCell1Pin, INPUT); // pin of voltage sensor measuring battery cell #1
  pinMode(battCell2Pin, INPUT); // pin of voltage sensor measuring battery cell #2

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
    SetDirec(i,desDirec[i]);
    if (pwm[i] > pwmCeiling) {pwm[i] = pwmCeiling;} else if (pwm[i] < 0) {pwm[i] = 0;}
    analogWrite(mdEnPins[i], pwm[i]);
  }
}

void ActuateAction() {
  // This function calls on independent functions to read the current and ...
  // ... compute the FF-PI controller to actuate a PWM accordingly.
  funcTime = micros();
  GetCurrent();
  CCUpdatePWM();
  funcTime = micros() - funcTime;
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

void CheckManualStop() {
  switchStatus = digitalRead(STOP_SWITCH_PIN_IN);
  if (switchStatus == 1) {
    stopCondition = MANUAL_STOP;
  } else {
    stopCondition = 0;
  }
}

//------------------------------------------------------------------

void setup() {

  InitStuff();

  // Wait until serial port is opened
  while (!Serial) {delay(1);}

  // setI[4] = {-8.0,-8.0,1.0,1.0};
  for(int i=0;i<4;i++) {Serial.println(setI[i]);}
  for(int i=0;i<4;i++) {desDirec[i] = 0;}
  for(int i=0;i<4;i++) {SetDirec(i,0);}

  // Initialize Timmer Interupts for 33Hz
  // Timer1.attachInterrupt(GetQuadEncoderData).start(30303); // Timer for Quad Encoder (33Hz)
  // Timer2.attachInterrupt(GetAbsEncoderData).start(30303);  // Timer for Abs Encoder (33Hz)
  Timer3.attachInterrupt(ActuateAction).start(controlTimer); // Timer for ActuateAction function
  Timer4.attachInterrupt(SineInput).start(5000000); // Timer for sinusoidal input to actuators
  Timer4.attachInterrupt(CheckManualStop).start(1000000); // Timer for switch status check

}

void loop() {
  if (stopCondition == 0) {
    
    if (lastStopCondition != 0) {
      Timer3.start(controlTimer);
      lastStopCondition = stopCondition;
    }
    
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
      Serial.print(absEncCurrentPosition[0]);
      Serial.print(" FR Abs Encoder Velocity: ");
      Serial.println(absEncCurrentVelocity[0]);
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
    Serial.print(", "); for(int i=0;i<4;i++) {Serial.print(direc[i]); Serial.print(",");}
    Serial.println("");
    currentFlag = 0;


    delay(100);








  } else if (stopCondition == MANUAL_STOP) {
    Timer3.stop();
    Serial.print("Manual stop, code ");
    Serial.println(stopCondition);
    lastStopCondition = stopCondition;
    delay(5000);
  } else {
    Timer3.stop();
    for(i=0;i<4;i++) {
      analogWrite(pwm[i], 0);
      digitalWrite(mdIn1Pins[i], LOW);
      digitalWrite(mdIn2Pins[i], LOW);
    }
    Serial.print("Safety stop code ");
    Serial.println(stopCondition);
    lastStopCondition = stopCondition;
    while(1);
  }
}
